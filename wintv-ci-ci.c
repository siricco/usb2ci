/*
 * wintv-ci-ci.c : WinTV-CI - USB2 Common Interface driver
 *
 * Copyright (C) 2017 Helmut Binder (cco@aon.at)
 #
 * (+HB+) 2017-08-13
 * (+HB+) 2017-09-18 first descrambling
 * (+HB+) 2017-10-21 partialy solved usb-lock with ts-streaming
 * (+HB+) 2018-03-04 Version 0.2
 * (+HB+) 2018-10-13 Version 0.3
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * version 2 only, as published by the Free Software Foundation.
 *
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * To obtain the license, point your browser to
 * http://www.gnu.org/copyleft/gpl.html
 */

/*
 *  C I / S E C - D E V I C E
 */

#include "wintv-ci.h"

#include <linux/module.h>
#include <linux/wait.h>
#include <linux/poll.h>

#define TS_PACKET_SIZE		188

#define ISOC_MAX_TRANSFERS	16
#define ISOC_MAX_UFRAMES	240 /* 30 ms | frames */

#define ISOC_NUM_TRANSFERS	8
#define ISOC_NUM_UFRAMES	120 /* 15 ms | frames */

#define ISOC_MIN_UF_CHUNK	8 /* transmit only in chunks of 8 micro-frames (1 full frame) */
#define ISOC_MIN_UF_SUBMIT	ISOC_MIN_UF_CHUNK /* the minimum of data to start a URB submission to the CAM */

/* a lot of TS in/out massages */
#define DEBUG_TS_IO 0
#define DEBUG_TS_IN 0

/* --- C I / C A M   Q U I R K S --- */
unsigned int dummy_ts_marker		= 0;
unsigned int dummy_ts_echo_marker	= 0;

/* --- P A R A M E T E R S --- */
static int use_dma_coherent = 0;
/*
*  despite hardware coherent DMA is supported (allmost) only on x86 systems
*  and the usage very likely slows down memcpy() on all other platforms,
*  it seems that for plain streaming it is neither required or has any real benefit.
*  So better don't use it. See also -> https://patchwork.kernel.org/patch/10468937/
*/
module_param(use_dma_coherent, int, 0644);
MODULE_PARM_DESC(use_dma_coherent, " Use usb_alloc_coherent() for isoc-urbs - not recommended for most non-x86 hardware (default:off).");

static int show_ts_bitrate = 0;

module_param(show_ts_bitrate, int, 0644);
MODULE_PARM_DESC(show_ts_bitrate, " Report the current TS datarate (every 10 secs) (default:off).");

static int dummy_half_uframes = 1;

module_param(dummy_half_uframes, int, 0644);
MODULE_PARM_DESC(dummy_half_uframes, "Quirk to reliable bring the last 2 TS packets of each USB-transfer into the CAM (default:on).");

/* --- R I N G B U F F E R --- */

static void ts_rb_free(struct ep_info *ep)
{
	if (!ep)
		return;

	dvb_ringbuffer_flush_spinlock_wakeup(&ep->erb.buffer);
	kfree(ep->erb.buffer.data);
	ep->erb.buffer.data = NULL;
}

static int ts_rb_alloc(struct ep_info *ep,
				int num_transfers,
				int num_uframes)
{
	/*  dvb_ringbuffer_free returns the buffer-size - 1 byte */
	int size = ep->maxp * num_uframes * num_transfers + ep->maxp;
	int ts = size / TS_PACKET_SIZE;
	void *buf = ci_kmalloc(size, 0, (char *)__func__);

	if (!buf)
		return -ENOMEM;

	dvb_ringbuffer_init(&ep->erb.buffer, buf, size);
	ep->erb.num_items = 0;

	pr_info("%s : EP(%02X) ringbuffer size %d bytes (%d x %d | %d TS-packets)\n",
				__func__, ep->addr, size,
				num_transfers, ts/num_transfers, ts);
	return 0;
}

/*
 *  Q U I R K   H A L F   U F R A M E S
 */

static int dummy_ts_fill_uframes(unsigned char *buf, int ts_num)
{
	unsigned char load[TS_PACKET_SIZE];
	unsigned char *b = buf;
	int i;
	/* initialize the dummy-TS - any values are possible, we enumerate */
	for (i = 0; i < sizeof(load); i++) {
		load[i] = i;
	}
	/*
	 * As we send onyl a half filled uframes, we receive 2 additional "echo"-TS packets.
	 * The data of this echo-TS is the repetiition of the last 2 bytes of the dummy-TS.
	 * insert this pattern as marker in the dummy-TS too (at offset 4)
	 */
	dummy_ts_marker = *(u16*)(load+TS_PACKET_SIZE-2);
	dummy_ts_marker |= dummy_ts_marker << 16; /* the data of echo-TS */
	dummy_ts_echo_marker = cpu_to_be32(dummy_ts_marker) & 0x00FFFFFF; /* ignore 1 byte of echo-TS */

	for (i = 0; i < ts_num; i++, b += TS_PACKET_SIZE) {
		memcpy(b, load, sizeof(load));
		b[0] = 0x47;
		b[1] = 0x1F;
		b[2] = 0xFF;
		b[3] = 0x10 | (i & 0xF);
		*(u32*)(b+4) = dummy_ts_marker;
	}
#if 0
	pr_info("%s : m1:%08X m2:%08X\n",
				__func__, dummy_ts_marker, dummy_ts_echo_marker);
	print_hex_dump(KERN_DEBUG, " Dummy-TS : ",
					DUMP_PREFIX_OFFSET, 16, 1,
					buf, TS_PACKET_SIZE*2, 1);
#endif
	return 0;
}

/*
 *  U S B - U R B   ( S T R E A M I N G )
 */

static void ci_isoc_kill_urbs(struct ep_info *ep)
{
	struct isoc_info *isoc	= &ep->u.isoc;
	int num_urbs		= isoc->num_urbs;
	int i;

	pr_info("%s : EP(%02X) kill %d isoc urbs\n",
				__func__, ep->addr, num_urbs);

	for (i = 0; i < num_urbs; i++) {
		struct urb_transfer *xfer = &isoc->urbs[i];
		usb_kill_urb(xfer->urb);
	}
}

static void ci_isoc_free(struct ep_info *ep)
{
	struct isoc_info *isoc	= &ep->u.isoc;
	int num_urbs		= isoc->num_urbs;
	int i;

	pr_info("%s : EP(%02X) kill and de-allocate %d isoc urbs\n",
				__func__, ep->addr, num_urbs);

	for (i = 0; i < num_urbs; i++) {
		struct urb_transfer *xfer = &isoc->urbs[i];

		usb_kill_urb(xfer->urb);
		usb_free_urb(xfer->urb);
		if (use_dma_coherent)
			usb_free_coherent(ep->wintvci->udev,
						isoc->transfer_size,
						xfer->xfer_buffer,
						xfer->dma_addr);
		else
			kfree(xfer->xfer_buffer);
	}
	kfree(isoc->urbs);
	isoc->num_urbs		= 0;
	isoc->urbs		= NULL;
}

static int ci_isoc_allocate(struct ep_info *ep,
				int num_transfers,
				int num_uframes,
				int min_submit_uf)
{
	struct isoc_info *isoc	= &ep->u.isoc;
	int uframe_size		= ep->maxp;
	int num_urbs		= num_transfers + 1; /* add 1 special transfer */
	int i;

	isoc->uframe_size	= uframe_size;
	isoc->num_uframes	= num_uframes;
	isoc->transfer_size	= num_uframes * uframe_size;
	isoc->min_chunk_size	= ISOC_MIN_UF_CHUNK * uframe_size;
	isoc->min_submit_size	= min_submit_uf * uframe_size;

	isoc->num_urbs		= 0;
	isoc->num_transfers	= 0;

	isoc->urbs		= ci_kmalloc(num_urbs * sizeof(*isoc->urbs),
						1, (char *)__func__);
	if (!isoc->urbs)
		return -ENOMEM;

	pr_info("%s : EP(%02X) init %d urbs (%d uframes | %d TS-packets each urb)\n",
				__func__, ep->addr, num_urbs, num_uframes,
				isoc->transfer_size / TS_PACKET_SIZE);

	for (i = 0; i < num_urbs; i++) {
		struct urb_transfer	*xfer = &isoc->urbs[i];
		struct urb		*urb;
		unsigned char		*tbuf;
		dma_addr_t		dma_addr = 0;

		/* allocate urbs */
		urb = usb_alloc_urb(num_uframes, GFP_KERNEL);
		if (!urb) {
			pr_err("%s : EP(%02X) allocate urb #%d failed\n",
						__func__, ep->addr, i);
			ci_isoc_free(ep);
			return -ENOMEM;
		}

		/* allocate transfer buffer */
		if (use_dma_coherent)
			tbuf = usb_alloc_coherent(ep->wintvci->udev,
				isoc->transfer_size, GFP_KERNEL, &dma_addr);
		else
			tbuf = kzalloc(isoc->transfer_size, GFP_KERNEL);

		if (!tbuf) {
			pr_err("%s : EP(%02X) allocate buffer #%d failed\n",
						__func__, ep->addr, i);
			usb_free_urb(urb);
			ci_isoc_free(ep);
			return -ENOMEM;
		}

		xfer->urb		= urb;
		xfer->xfer_buffer	= tbuf;
		xfer->dma_addr		= dma_addr;

		isoc->num_urbs++;
	}

	isoc->num_transfers	= num_transfers;
	isoc->transfers		= &isoc->urbs[0];

	return 0;
}

static int ci_isoc_setup(struct ep_info *ep, struct usb_device *udev)
{
	struct isoc_info *isoc	= &ep->u.isoc;
	int i;

	if (!isoc->urbs) {
		pr_info("%s : EP(%02X) no urbs allocated\n",
					__func__, ep->addr);
		return -EINVAL;
	}

	pr_info("%s : EP(%02X) initialize %d urbs (%s)\n",
			__func__, ep->addr, isoc->num_urbs,
			(ep->dir == USB_DIR_IN) ? "DIR_IN" : "DIR_OUT");

	for (i = 0; i < isoc->num_urbs; i++) {
		struct urb_transfer *xfer	= &isoc->urbs[i];
		struct urb *urb			= xfer->urb;
		int j;
		int uframe_ofs = 0;
		int uframe_len = isoc->uframe_size;

		urb->dev			= udev;
		urb->context			= ep;
		urb->pipe			= ep->pipe;
		/* USB2 High-Speed: encoded in power of 2 */
		urb->interval			= 1 << (ep->binterval - 1); /* 1 << (1 - 1) = 1 */

		urb->complete			= ts_urb_complete;

		urb->transfer_flags		= URB_ISO_ASAP;

		if (use_dma_coherent) {
			urb->transfer_flags	|= URB_NO_TRANSFER_DMA_MAP;
			urb->transfer_dma	= xfer->dma_addr;
		}
		urb->transfer_buffer		= xfer->xfer_buffer;
		urb->number_of_packets		= isoc->num_uframes;
		urb->transfer_buffer_length 	= isoc->transfer_size;

		for (j = 0; j < isoc->num_uframes; j++) {
			urb->iso_frame_desc[j].offset = uframe_ofs;
			urb->iso_frame_desc[j].length = uframe_len;
			uframe_ofs += uframe_len;
		}
		if (i == isoc->num_transfers && dummy_half_uframes) {
			if (ep->dir == USB_DIR_OUT) {
				dummy_ts_fill_uframes(urb->transfer_buffer,
							isoc->uframe_size / TS_PACKET_SIZE); /* only 1 uframe */
				for (j = 0; j < isoc->num_uframes; j++) {
					urb->iso_frame_desc[j].length = 0; /*no data to send */
				}
			}
		}
	}
	return 0;
}

#define DUMMY_IN_FAST 1
#define DUMMY_IN_SLOW 0
static void ci_quirks_set_dummy_IN_num_packets(struct urb *urb, int fast) {
#define DUMMY_IN_FAST_UF  1
#define DUMMY_IN_SLOW_UF  4
	int uframes = (fast) ? DUMMY_IN_FAST_UF : DUMMY_IN_SLOW_UF;
	if (urb->number_of_packets != uframes) {
		pr_info("*** Quirks : set dummy IN packets from %d to %d uframes\n",
			    urb->number_of_packets, uframes);
		urb->number_of_packets = uframes;
	}
}

static void ci_quirks_set_defaults(struct ci_device *ci_dev) {
	if (dummy_half_uframes) {
#define DUMMY_OUT_TS      2 /* only 2 TS-packets are required for slower CAMs! */
#define DUMMY_OUT_UF      1
		struct isoc_info *isoc;//	= &ep->u.isoc;
		struct urb *urb;
		/* OUT */
		isoc = &ci_dev->ep_isoc_out.u.isoc;
		urb = isoc->transfers[isoc->num_transfers].urb;/* the dummy OUT URB */
		urb->number_of_packets = DUMMY_OUT_UF;
		urb->iso_frame_desc[0].length = TS_PACKET_SIZE * DUMMY_OUT_TS;
		/* IN */
		isoc = &ci_dev->ep_isoc_in.u.isoc;
		urb = isoc->transfers[isoc->num_transfers].urb;/* the dummy IN URB */
		ci_quirks_set_dummy_IN_num_packets(urb, DUMMY_IN_FAST);
	}
}

static void ci_isoc_exit(struct ci_device *ci_dev)
{
	struct ep_info *ep;

	ep = &ci_dev->ep_isoc_in;
	ci_isoc_free(ep);
	ts_rb_free(ep);

	ep = &ci_dev->ep_isoc_out;
	ci_isoc_free(ep);
	ts_rb_free(ep);
}

static int ci_isoc_init(struct ci_device *ci_dev)
{
	struct usb_device *udev   = ci_dev->wintvci->udev;
	struct ep_info *ep;
	int rc;

	int num_transfers	= min(ISOC_NUM_TRANSFERS, ISOC_MAX_TRANSFERS);
	int num_uframes		= min(ISOC_NUM_UFRAMES,   ISOC_MAX_UFRAMES);
	int min_submit_uf	= min(ISOC_MIN_UF_SUBMIT, ISOC_MIN_UF_CHUNK);

	/* OUT */
	ep = &ci_dev->ep_isoc_out;
	rc = ts_rb_alloc(ep, num_transfers, num_uframes);
	if (rc)
		return rc;
	rc = ci_isoc_allocate(ep, num_transfers, num_uframes, min_submit_uf);
	if (rc)
		return rc;
	ci_isoc_setup(ep, udev);

	/* IN */
	ep = &ci_dev->ep_isoc_in;
	rc = ts_rb_alloc(ep, num_transfers, num_uframes);
	if (rc)
		return rc;
	rc = ci_isoc_allocate(ep, num_transfers, num_uframes, min_submit_uf);
	if (rc)
		return rc;
	ci_isoc_setup(ep, udev);

	return 0;
}

/* --- U R B - C O M P L E T E --- */

void ts_stop_streaming(struct ci_device *ci_dev)
{
	pr_info("%s : stop streaming\n", __func__);

	ci_dev->isoc_enabled = 0;

	if (ci_dev->isoc_urbs_running) {
		ci_isoc_kill_urbs(&ci_dev->ep_isoc_out);
		ci_isoc_kill_urbs(&ci_dev->ep_isoc_in);
		ci_dev->isoc_urbs_running = 0;
	}
	dvb_ringbuffer_flush_spinlock_wakeup(&ci_dev->ep_isoc_out.erb.buffer);
	dvb_ringbuffer_flush_spinlock_wakeup(&ci_dev->ep_isoc_in.erb.buffer);
}

void ts_read_CAM_complete(struct urb *urb)	/* CAM --> TS-IN ringbuffer */
{
	struct ep_info *ep_in		= urb->context;
	struct ci_device *ci_dev	= &ep_in->wintvci->ci_dev;

	struct dvb_ringbuffer *rb	= &ep_in->erb.buffer;
	struct isoc_info *isoc		= &ep_in->u.isoc;
	int rb_free			= dvb_ringbuffer_free(rb);
	u8 *b				= (u8 *)urb->transfer_buffer;

	int act_size = 0;
	int num_uframes = 0;
	int i;
	int nts = 0;

	for (i = 0; i < urb->number_of_packets; i++) {
		int uf_ofs		= urb->iso_frame_desc[i].offset;
		int uf_size		= urb->iso_frame_desc[i].actual_length;

		if (urb->iso_frame_desc[i].status != 0) {
			pr_err("%s : uframe #%d: error with status(%d)\n",
				__func__, i, urb->iso_frame_desc[i].status);
			continue;
		}
		else if (!uf_size)
			continue;
		else if (uf_size != isoc->uframe_size) {
			pr_err("%s : uframe #%d: bad size %d\n",
						__func__, i , uf_size);
			continue;
		}

		if (rb_free < uf_size) {
			pr_warn("%s : uframe #%d RB full - lost %d old TS\n",
					__func__, i, uf_size/TS_PACKET_SIZE);
			DVB_RINGBUFFER_SKIP(rb, uf_size);
			rb_free += uf_size;
		}

		if (dummy_half_uframes) { /* filter out the special half-uframes TS */
			unsigned char *ts	= b + uf_ofs;
			int ts_num		= isoc->uframe_size / TS_PACKET_SIZE;
			int j;

			for (j = 0; j < ts_num; j++, ts += TS_PACKET_SIZE) {
				u32 mark = *(u32 *)(ts + 4);
#if DEBUG_TS_IN
				pr_info(" * TS[%d/%d] %*ph\n", i, j, 8, ts);
#endif
				if (mark == dummy_ts_marker) { /* found the marker at offset 4 */
					mark = cpu_to_be32(*(u32 *)ts);
					if ( ((mark & 0x00FFFFF0) == 0x1FFF10) ||		/* our special dummy-TS */
					     ((mark & 0x00FFFFFF) == dummy_ts_echo_marker)) {	/* additional "echo"-TS from obove */
#if DEBUG_TS_IN
						pr_info("%s(%d) : skip Dummy-TS [%d/%d]\n",
								__func__, ci_dev->isoc_urbs_running, i, j);
#endif
						nts++;
						continue;
					}
				}
				dvb_ringbuffer_write(rb, ts, TS_PACKET_SIZE);
				rb_free -= TS_PACKET_SIZE;
				act_size += TS_PACKET_SIZE;
			}
		}
		else {
			if (b[uf_ofs] != 0x47)
				pr_warn("%s : uframe #%d not SYNC: 0x%02X\n",
							__func__, i, b[uf_ofs]);
			dvb_ringbuffer_write(rb, b + uf_ofs, uf_size);
			rb_free -= uf_size;
			act_size += uf_size;
		}
		num_uframes++;
	}
#if DEBUG_TS_IN
	if (nts)
		pr_info("%s(%d) : skipped %d dummy-TS\n", __func__, ci_dev->isoc_urbs_running, nts);
#endif
	if (act_size) {
		ci_dev->isoc_TS_CAM -= act_size / TS_PACKET_SIZE;

		/* adjust dummy reader for slow respronsing CAMs */
		if (ci_dev->isoc_urbs_running == 1 && ci_dev->isoc_TS_CAM < 0 &&
		    dummy_half_uframes && urb->number_of_packets == DUMMY_IN_FAST_UF) { /* more IN then OUT */
			 /* set for all following submisssions */
			ci_quirks_set_dummy_IN_num_packets(urb, DUMMY_IN_SLOW);
		}
#if DEBUG_TS_IO
		pr_info("%s(%d) : --- %d x TS, %2d uframes - rb-avail(%zu) CAM(%+d)\n",
				__func__, ci_dev->isoc_urbs_running, act_size / TS_PACKET_SIZE, num_uframes,
				dvb_ringbuffer_avail(rb)/TS_PACKET_SIZE, ci_dev->isoc_TS_CAM);
#endif
		wake_up_interruptible(&ep_in->erb.wq); /* ep-in waitqueue RB --> HOST*/
	}
	return;
}

void ts_write_CAM_complete(struct urb *urb)
{
	return;
}

void ts_urb_complete(struct urb *urb)
{
	struct ep_info *ep		= urb->context;
	struct ci_device *ci_dev	= &ep->wintvci->ci_dev;

	unsigned int dir = (ep->addr & USB_ENDPOINT_DIR_MASK);

	if (urb->status) { /* no success */
		switch (urb->status) {
		case -ETIMEDOUT:    /* NAK */
			pr_err("NACK[%X] %d\n", ep->addr, urb->status);
			break;
		case -ECONNRESET:   /* kill */
		case -ENOENT:
		case -ESHUTDOWN:
			pr_err("KILL[%X] %d\n", ep->addr, urb->status);
			return;
		default:        /* error */
			pr_err("%s : EP(%02X) urb completion failed=%d\n",
					__func__, ep->addr, urb->status);
			break;
		}
	}
	if (dir == USB_DIR_IN)
		ts_read_CAM_complete(urb); /* CAM --> TS-IN ringbuffer */
	else
		ts_write_CAM_complete(urb); /* TS-OUT ringbuffer --> CAM */

	ci_dev->isoc_urbs_running--;

	if (!ci_dev->isoc_urbs_running && ci_dev->isoc_TS_CAM) { // not all packets have been sent to CAM
		pr_warn("%s : TS CAM-IO: %+d", __func__, ci_dev->isoc_TS_CAM);
		ci_dev->isoc_TS_CAM_total += ci_dev->isoc_TS_CAM;
		ci_dev->isoc_TS_CAM = 0;
	}

	wake_up_interruptible(&ci_dev->isoc_urbs_wq); /* urb waitqueue */

#if DEBUG_TS_IO
	pr_info("%s(%s) : start frame: %d, running URBs: %d\n",
			__func__, (dir == USB_DIR_IN) ? "IN " : "OUT", urb->start_frame, ci_dev->isoc_urbs_running);
#endif
}

static int ts_write_CAM_submit(struct ci_device *ci_dev, int transfers) /* URBs --> CAM */
{
	struct ep_info *ep_out		= &ci_dev->ep_isoc_out;
	struct ep_info *ep_in		= &ci_dev->ep_isoc_in;
	struct urb *urb_out, *urb_in;
	int i, rc = 0;

	for (i = 0; i < transfers; i++) {
		urb_out	= ep_out->u.isoc.transfers[i].urb;
		urb_in	= ep_in->u.isoc.transfers[i].urb;
		/* start the first urb transfers at the very beginning of a new uframe! */
		if (i == 0) {
			int cf = usb_get_current_frame_number(ci_dev->wintvci->udev);
			int nf = 0;
			int cnt = 0;
			/*
			 * lock the whole submission sequence - detection of SOF and
			 * submissions must be within 0.125ms
			 */
			spin_lock_irqsave(&ci_dev->ci_lock, ci_dev->ci_lock_flags);
			do {
				nf = usb_get_current_frame_number(ci_dev->wintvci->udev);
				cnt++;
			} while (nf == cf); // loops up to 1ms
			//if (cnt > 200) pr_info("%s : frame-number:%d, %d\n", __func__, nf << 3, cnt);
		}

		/* submit out-urb - at first */
		rc = usb_submit_urb(urb_out, GFP_ATOMIC);
		if (rc < 0)
			pr_warn("%s : Could not submit TS-OUT URB[%d/%d]: (%d)\n",
						__func__, i, transfers, rc);
		ci_dev->isoc_urbs_running++;

		/* submit in-urb - as second */
		rc = usb_submit_urb(urb_in, GFP_ATOMIC);
		if (rc < 0)
			pr_warn("%s : Could not submit TS-IN URB[%d/%d]: (%d)\n",
						__func__, i, transfers, rc);
		ci_dev->isoc_urbs_running++;

		if (i == 0)
			spin_unlock_irqrestore(&ci_dev->ci_lock, ci_dev->ci_lock_flags);

		if (urb_out->start_frame != urb_in->start_frame)
			pr_err("%s[%d] : diff. start_frames: out/in: %d/%d\n",
				__func__, i, urb_out->start_frame, urb_in->start_frame);
	}
	if (i && dummy_half_uframes) {
		/* submit special URBs */
		int j = ep_out->u.isoc.num_transfers;

		urb_out = ep_out->u.isoc.transfers[j].urb;
		urb_in  = ep_in->u.isoc.transfers[j].urb;

		rc = usb_submit_urb(urb_out, GFP_ATOMIC);
		ci_dev->isoc_urbs_running++;

		rc = usb_submit_urb(urb_in, GFP_ATOMIC);
		ci_dev->isoc_urbs_running++;
	}
	return rc;
}

void report_ts_bitrate(struct ci_device *ci_dev)
{
#define TS_COUNT_TIME    10 /* secs */
#define TS_COUNT_TIMEOUT (HZ * TS_COUNT_TIME)
	unsigned long timer_now = jiffies;

	if (ci_dev->ts_count_timeout <= timer_now ) {
		int time = timer_now - ci_dev->ts_count_timeout + TS_COUNT_TIMEOUT;
		int scam100 = ci_dev->cam_subs * 100 * HZ / time;
		int uf100 = (!ci_dev->cam_subs) ? 0 :
			ci_dev->cam_uframes * 10000 / (ISOC_NUM_TRANSFERS * ISOC_NUM_UFRAMES * ci_dev->cam_subs);
		int mbitx100 = ci_dev->ts_count / time * HZ * 8; /* bits/second */
			mbitx100 /= (1000 * 1000 / 100);

		pr_info(" +++ TS-BITRATE : %2d.%02d Mbit/s, %2d.%02d CAM-subs/s, URB-load: %2d.%02d%%, TS-miss: %d\n",
			mbitx100 / 100,mbitx100 % 100, scam100 / 100, scam100 % 100, uf100/100, uf100 % 100, ci_dev->isoc_TS_CAM_total);

		ci_dev->cam_subs = 0;
		ci_dev->cam_uframes = 0;
		ci_dev->ts_count = 0;
		ci_dev->ts_count_timeout = timer_now + TS_COUNT_TIMEOUT;
	}
}

static int ts_write_CAM_prepare(struct ci_device *ci_dev)	/* TS-OUT ringbuffer --> URBs */
{
	struct ep_info *ep_out	= &ci_dev->ep_isoc_out;
	struct ep_info *ep_in	= &ci_dev->ep_isoc_in;
	struct urb *urb_out, *urb_in;

	int uframe_size		= ep_out->u.isoc.uframe_size;
	int transfer_size	= ep_out->u.isoc.transfer_size;
	int max_transfers	= ep_out->u.isoc.num_transfers;
	int act_size		= 0;
	int i;

	ssize_t rb_avail = dvb_ringbuffer_avail(&ep_out->erb.buffer);

	if (rb_avail < ep_out->u.isoc.min_submit_size)
		return 0;

	if (rb_avail % TS_PACKET_SIZE)
		pr_warn("%s : TS-packets with %zd trailing bytes\n",
					__func__, rb_avail % TS_PACKET_SIZE);

	rb_avail -= rb_avail % ep_out->u.isoc.min_chunk_size;

	for (i = 0; i < max_transfers; i++) {
		size_t left = min(transfer_size, (int)rb_avail);
		/*
		 * write only chunks of full micro-frame size or the
		 * incoming data is filled up with undefined data
		 */
		if (left < ep_out->u.isoc.min_chunk_size)
			break;

		/* we have enough TS-data */
		urb_out		= ep_out->u.isoc.transfers[i].urb;
		urb_in		= ep_in->u.isoc.transfers[i].urb;
		/* usb_submit_urb resets all .status and .actual_length fields */
		urb_out->number_of_packets = left / uframe_size;
		urb_in->number_of_packets = left / uframe_size;

		dvb_ringbuffer_read(&ep_out->erb.buffer, urb_out->transfer_buffer, left);
		rb_avail -= left;
		act_size += left;
	}
	if (i) {
		int act_ts	= act_size / TS_PACKET_SIZE;
		int act_uframes	= act_size / uframe_size;

		ci_dev->isoc_TS_CAM += act_ts;

		if (show_ts_bitrate) {
			ci_dev->ts_count += act_size; /* FIXME - handle overflow */
			ci_dev->cam_uframes += act_uframes;
			ci_dev->cam_subs++; /* count transmission */
		}
#if DEBUG_TS_IO
		pr_info("%s[%d] : --- %d x TS, %2d uframes - rb-avail(%zd) CAM(%+d)\n",
				__func__, i, act_ts, act_uframes,
				rb_avail / TS_PACKET_SIZE, ci_dev->isoc_TS_CAM);
#endif
	}
	if (show_ts_bitrate)
		report_ts_bitrate(ci_dev);
	return i;
}

#define RB_READ_CONDITION(ep)  (dvb_ringbuffer_avail(&ep->erb.buffer) >= TS_PACKET_SIZE)
#define RB_WRITE_CONDITION(ep) (dvb_ringbuffer_free(&ep->erb.buffer) >= TS_PACKET_SIZE)

/* TS-OUT ringbuffer --> CAM --> TS-IN ringbuffer */
static void ts_CAM_exchange(struct ci_device *ci_dev)
{
	int transfers;

	if (ci_dev->isoc_urbs_running)
		if (wait_event_interruptible(
				ci_dev->isoc_urbs_wq,
				!ci_dev->isoc_urbs_running) < 0)
			return;

	transfers = ts_write_CAM_prepare(ci_dev); /* TS-OUT ringbuffer --> URBs */
	if (transfers)
		ts_write_CAM_submit(ci_dev, transfers); /* URBs --> CAM */
}

/* Host --> TS-OUT ringbuffer */
static ssize_t ts_write(struct file *file, const __user char *buf,
			size_t count, loff_t *ppos)
{
	struct dvb_device *dvbdev	= file->private_data;
	struct wintv_ci_dev *wintvci	= dvbdev->priv;
	struct ci_device *ci_dev	= &wintvci->ci_dev;

	struct ep_info *ep		= &ci_dev->ep_isoc_out;
	struct dvb_ringbuffer *rb	= &ep->erb.buffer;

	size_t todo = 0;
	size_t written = 0;
	size_t rb_free = dvb_ringbuffer_free(rb);

	ci_dev->isoc_enabled = 1;

	if (rb_free < TS_PACKET_SIZE)
		pr_err("%s : *** ringbuffer full\n", __func__);

	if (buf[0] != 0x47)
		pr_warn("%s : TS-Data[0] not SYNC: 0x%02X\n",
						__func__, buf[0]);
	if (count % TS_PACKET_SIZE)
		pr_warn("%s : TS-data with %zu trailing bytes(count %zu)\n",
				__func__, todo % TS_PACKET_SIZE, count);

	todo = min(rb_free, count);
	todo -= todo % TS_PACKET_SIZE;

	if (todo) {
		/* copy_from_user */
		written = dvb_ringbuffer_write_user(rb, buf, todo);
		if (written != todo)
			pr_err("%s : *** written(%zu) != todo(%zu)\n",
						__func__, written, todo);
#if DEBUG_TS_IO
		pr_info("%s : *** TS{%02X} (%zu)[%zu]<%zu>\n", __func__,
				(u8) buf[0], count, written, written/TS_PACKET_SIZE);
#endif
	}

	if (dvb_ringbuffer_avail(rb) >= ep->u.isoc.min_submit_size)
		ts_CAM_exchange(ci_dev);

	return written;
}

/* TS-IN ringbuffer --> Host */
static ssize_t ts_read(struct file *file, __user char *buf,
			size_t count, loff_t *ppos)
{
	struct dvb_device *dvbdev	= file->private_data;
	struct wintv_ci_dev *wintvci	= dvbdev->priv;
	struct ci_device *ci_dev	= &wintvci->ci_dev;

	struct ep_info *ep		= &ci_dev->ep_isoc_in;
	struct dvb_ringbuffer *rb	= &ep->erb.buffer;

	ssize_t read = 0;
	ssize_t avail = min(dvb_ringbuffer_avail(rb), (ssize_t)count);

	if (avail > 0) {
		/* copy_to_user */
		read = dvb_ringbuffer_read_user(rb, buf, avail);
		if (read != avail)
			pr_err("%s : *** read(%zd) != avail(%zd)\n",
						__func__, read, avail);
#if DEBUG_TS_IO
		pr_info("%s : *** TS{%02X} (%zu)[%zd]<%zd>\n", __func__,
				(u8) buf[0], count, read, read/TS_PACKET_SIZE);
#endif
	}
	return read;
}

static unsigned int ts_poll(struct file *file, poll_table *wait)
{
	struct dvb_device *dvbdev	= file->private_data;
	struct wintv_ci_dev *wintvci	= dvbdev->priv;
	struct ci_device *ci_dev	= &wintvci->ci_dev;

	struct ep_info *ep_in		= &ci_dev->ep_isoc_in;
	struct ep_info *ep_out		= &ci_dev->ep_isoc_out;

	unsigned int mask = 0;

	if (!RB_READ_CONDITION(ep_in))
		poll_wait(file, &ep_in->erb.wq, wait);

	if (RB_READ_CONDITION(ep_in))
		mask |= POLLIN;
	if (RB_WRITE_CONDITION(ep_out))
		mask |= POLLOUT;

	return mask;
}

static const struct file_operations ci_fops = {
	.owner		= THIS_MODULE,
	.read		= ts_read,
	.write		= ts_write,
	.open		= dvb_generic_open,
	.release	= dvb_generic_release,
	.poll		= ts_poll,
};

static struct dvb_device ci_dvbdev = {
	.readers	= -1,
	.writers	= -1,
	.users		= -1,
	.fops		= &ci_fops,
};

void ci_reset(struct wintv_ci_dev *wintvci)
{
	struct ci_device *ci_dev = &wintvci->ci_dev;

	pr_info("Reset CI Device\n");

	ts_stop_streaming(ci_dev);
	ci_quirks_set_defaults(ci_dev);

	ci_dev->cam_subs = 0;
	ci_dev->cam_uframes = 0;
	ci_dev->ts_count = 0;
	ci_dev->ts_count_timeout = jiffies + TS_COUNT_TIMEOUT;
	ci_dev->isoc_TS_CAM = 0;
	ci_dev->isoc_TS_CAM_total = 0;
}

void ci_detach(struct wintv_ci_dev *wintvci)
{
	struct ci_device *ci_dev = &wintvci->ci_dev;

	if (!ci_dev->regdev_ci)
		return;

	pr_info("Detaching DVB CI Device\n");

	//ts_stop_streaming(ci_dev);
	dvb_unregister_device(ci_dev->regdev_ci);

	ci_isoc_exit(ci_dev);
}

int ci_attach(struct wintv_ci_dev *wintvci)
{
	struct ci_device *ci_dev = &wintvci->ci_dev;
	int rc = 0;

	pr_info("Attaching DVB CI Device\n");
	ci_dev->wintvci = wintvci;

	init_waitqueue_head(&ci_dev->isoc_urbs_wq);
	init_waitqueue_head(&ci_dev->ep_isoc_in.erb.wq);
	init_waitqueue_head(&ci_dev->ep_isoc_out.erb.wq);

	mutex_init(&ci_dev->ci_mutex);
	spin_lock_init(&ci_dev->ci_lock);

	rc = ci_isoc_init(ci_dev);
	if (rc)
		return rc;

	ci_reset(wintvci);

	rc = dvb_register_device(&wintvci->adapter, &ci_dev->regdev_ci,
					&ci_dvbdev, wintvci, DVB_DEVICE_SEC, 0);
	return rc;
}
