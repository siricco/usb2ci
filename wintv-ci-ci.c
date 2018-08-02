/*
 * wintv-ci-ci.c : WinTV-CI - USB2 Common Interface driver
 *
 * Copyright (C) 2017 Helmut Binder (cco@aon.at)
 #
 * (+HB+) 2017-08-13 
 * (+HB+) 2017-09-18 first descrambling
 * (+HB+) 2017-10-21 partialy solved usb-lock with ts-streaming
 * (+HB+) 2018-03-04 Version 0.2
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
#include <linux/spinlock.h>

#define TS_PACKET_SIZE		188

#define ISOC_MAX_TRANSFER	16
#define ISOC_MAX_UFRAMES	240

#define ISOC_NUM_TRANSFER	8
#define ISOC_NUM_UFRAMES	120

/* a lot of TS in/out massages */
#define DEBUG_TS_IO 0

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
	int size = ep->maxp * num_uframes * num_transfers;
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
 *  U S B - U R B   ( S T R E A M I N G )
 */

static void ci_isoc_kill_urbs(struct ep_info *ep)
{
	struct isoc_info *isoc	= &ep->u.isoc;
	int num_transfers	= isoc->num_transfers;
	int i;

	pr_info("%s : EP(%02X) kill %d isoc transfers\n",
				__func__, ep->addr, num_transfers);

	for (i = 0; i < num_transfers; i++) {
		struct urb_transfer *xfer = &isoc->transfers[i];
		usb_kill_urb(xfer->urb);
	}
}

static void ci_isoc_free(struct ep_info *ep)
{
	struct isoc_info *isoc	= &ep->u.isoc;
	int num_transfers	= isoc->num_transfers;
	int i;

	pr_info("%s : EP(%02X) kill and de-allocate %d isoc transfers\n",
				__func__, ep->addr, num_transfers);

	for (i = 0; i < num_transfers; i++) {
		struct urb_transfer *xfer = &isoc->transfers[i];

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
	kfree(isoc->transfers);
	isoc->num_transfers	= 0;
	isoc->transfers		= NULL;
}

static int ci_isoc_allocate(struct ep_info *ep,
				int num_transfers,
				int num_uframes)
{
	struct isoc_info *isoc	= &ep->u.isoc;
	int uframe_size		= ep->maxp;
	int i;

	if (isoc->transfers) {
		pr_err("%s : EP(%02X) transfers not NULL\n",
					__func__, ep->addr);
		return -1;
	}

	isoc->uframe_size	= uframe_size;
	isoc->num_uframes	= num_uframes;
	isoc->transfer_size	= num_uframes * uframe_size;

	isoc->num_transfers	= 0;
	isoc->transfers 	= ci_kmalloc(num_transfers *
					sizeof(*isoc->transfers),
					1, (char *)__func__);
	if (!isoc->transfers)
		return -ENOMEM;

	pr_info("%s : EP(%02X) init %d urbs (%d uframes | %d TS-packets each urb)\n",
				__func__, ep->addr, num_transfers, num_uframes,
				isoc->transfer_size / TS_PACKET_SIZE);

	for (i = 0; i < num_transfers; i++) {
		struct urb_transfer	*xfer = &isoc->transfers[i];
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

		isoc->num_transfers++;
	}
	return 0;
}

static int ci_isoc_setup(struct ep_info *ep, struct usb_device *udev)
{
	struct isoc_info *isoc	= &ep->u.isoc;
	int i;

	if (!isoc->transfers) {
		pr_info("%s : EP(%02X) transfer urbs not allocated\n",
					__func__, ep->addr);
		return -1;
	}

	pr_info("%s : EP(%02X) initialize %d urbs (%s)\n",
			__func__, ep->addr, isoc->num_transfers,
			(ep->dir == USB_DIR_IN) ? "DIR_IN" : "DIR_OUT");

	for (i = 0; i < isoc->num_transfers; i++) {
		struct urb_transfer *xfer	= &isoc->transfers[i];
		struct urb *urb			= xfer->urb;
		int frame_ofs = 0;
		int j;

		urb->dev			= udev;
		urb->context			= ep;
		urb->pipe			= ep->pipe;
		 /* USB2 High-Speed: encoded in power of 2 */
		urb->interval			= 1 << (ep->binterval - 1); /* 1 << (1 - 1) = 1 */

		urb->complete			= ts_urb_complete;
		/*
		   The isochonous in/out urbs for the TS-stream must both be
		   scheduled EXACTLY to the first micro-frame at SOF otherwise
		   interrupt-in urbs are not responding anymore!
		   With URB_ISO_ASAP the iso-in is randomly
		   scheduled 1 micro-frame after iso-in.
		*/
		urb->transfer_flags		= 0;

		if (use_dma_coherent) {
			urb->transfer_flags	|= URB_NO_TRANSFER_DMA_MAP;
			urb->transfer_dma	= xfer->dma_addr;
		}
		else
			urb->transfer_buffer	= xfer->xfer_buffer;

		urb->number_of_packets		= isoc->num_uframes;
		urb->transfer_buffer_length 	= isoc->transfer_size;

		for (j = 0; j < isoc->num_uframes; j++) {
			urb->iso_frame_desc[j].offset = frame_ofs;
			urb->iso_frame_desc[j].length = isoc->uframe_size;
			frame_ofs += isoc->uframe_size;
		}
	}
	return 0;
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
	struct usb_device *udev = ci_dev->wintvci->udev;
	struct ep_info *ep;
	int num_transfers = ISOC_NUM_TRANSFER;
	int num_uframes   = ISOC_NUM_UFRAMES;
	int rc;

	if (num_transfers > ISOC_MAX_TRANSFER) {
		pr_err("%s : limit num_transfers to %d\n",
					__func__, ISOC_MAX_TRANSFER);
		num_transfers = ISOC_MAX_TRANSFER;
	}
	if (num_uframes > ISOC_MAX_UFRAMES) {
		pr_err("%s : limit num_uframes to %d\n",
					__func__, ISOC_MAX_UFRAMES);
		num_uframes = ISOC_MAX_UFRAMES;
	}

	ep = &ci_dev->ep_isoc_out;
	rc = ts_rb_alloc(ep, num_transfers, num_uframes);
	if (rc)
		return rc;
	rc = ci_isoc_allocate(ep, num_transfers, num_uframes);
	if (rc)
		return rc;
	ci_isoc_setup(ep, udev);

	ep = &ci_dev->ep_isoc_in;
	rc = ts_rb_alloc(ep, num_transfers, num_uframes);
	if (rc)
		return rc;
	rc = ci_isoc_allocate(ep, num_transfers, num_uframes);
	if (rc)
		return rc;
	ci_isoc_setup(ep, udev);

	return rc;
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

	for (i = 0; i < urb->number_of_packets; i++) {
		int ofs  = urb->iso_frame_desc[i].offset;
		int size = urb->iso_frame_desc[i].actual_length;

		if (!size)
			continue;
		else if (urb->iso_frame_desc[i].status != 0) {
			pr_err("%s : uframe #%d: error with status(%d)\n",
				__func__, i, urb->iso_frame_desc[i].status);
			continue;
		}
		else if (size != isoc->uframe_size) {
			pr_err("%s : uframe #%d: bad size %d\n",
						__func__, i , size);
			continue;
		}
		if (b[ofs] != 0x47)
			pr_warn("%s : uframe #%d not SYNC: 0x%02X\n",
						__func__, i, b[ofs]);

		if (rb_free < size) {
			pr_warn("%s : uframe #%d RB full - lost %d old TS\n",
					__func__, i, size/TS_PACKET_SIZE);
			DVB_RINGBUFFER_SKIP(rb, size);
			rb_free += size;
		}

		dvb_ringbuffer_write(rb, b+ofs, size);
		rb_free -= size;

		act_size += size;
		num_uframes++;
	}
	if (num_uframes) {
		ci_dev->isoc_bytes_CAM	-= act_size;

		if (ci_dev->isoc_bytes_CAM < 0) {
			pr_err("%s :  paket-count underrun (%d)\n",
					__func__, ci_dev->isoc_bytes_CAM);
			print_hex_dump(KERN_DEBUG, " CAM-PRE : ", DUMP_PREFIX_OFFSET, 16, 1,
						b, act_size, 1);
			ci_dev->isoc_bytes_CAM = 0;
		}
#if DEBUG_TS_IO
		pr_info("%s : --- %d x TS, %2d uframes - rb-avail(%zu) CAM(%d) sframe %d\n",
				__func__, act_size / TS_PACKET_SIZE, num_uframes,
				dvb_ringbuffer_avail(rb)/TS_PACKET_SIZE,
				ci_dev->isoc_bytes_CAM/TS_PACKET_SIZE,
				urb->start_frame);
#endif
		wake_up_interruptible(&ep_in->erb.wq); /* ep-in waitqueue RB --> HOST*/
	}
	ci_dev->isoc_urbs_running--;
	wake_up_interruptible(&ci_dev->isoc_urbs_wq); /* urb waitqueue */
#if DEBUG_TS_IO
	pr_info("%s : urb submission #%d suspended, sframe %d\n",
			__func__, ci_dev->isoc_urbs_running, urb->start_frame);
#endif
	return;
}

void ts_write_CAM_complete(struct urb *urb)
{
	struct ep_info *ep_out		= urb->context;
	struct ci_device *ci_dev	= &ep_out->wintvci->ci_dev;

	ci_dev->isoc_urbs_running--;
	wake_up_interruptible(&ci_dev->isoc_urbs_wq); /* urb waitqueue */
#if DEBUG_TS_IO
	if (!ci_dev->isoc_urbs_running) // last urb
		pr_info("%s : ISOC write end - sframe %d\n", __func__, urb->start_frame);
#endif
	return;
}

void ts_urb_complete(struct urb *urb)
{
	struct ep_info *ep = urb->context;
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
			pr_err("%s : EP(%02X) urb completition failed=%d\n",
					__func__, ep->addr, urb->status);
			break;
		}
	}
	if (dir == USB_DIR_IN)
		ts_read_CAM_complete(urb); /* CAM --> TS-IN ringbuffer */
	else
		ts_write_CAM_complete(urb); /* TS-OUT ringbuffer --> CAM */
}

/* write a multiple of a full frame (8 micro-frames) */
#define NUF 8
#define TS_MIN_UF(u) (u * NUF)

static int ts_write_CAM(struct ci_device *ci_dev, int urb_index)	/* TS-OUT ringbuffer --> CAM */
{
	struct ep_info *ep_out		= &ci_dev->ep_isoc_out;
	struct ep_info *ep_in		= &ci_dev->ep_isoc_in;
	struct urb *urb_out, *urb_in;

	int uframe_size			= ep_out->u.isoc.uframe_size;
	int frame_bufsize		= ep_out->u.isoc.transfer_size;
	int num_uframes;

	size_t rb_avail			= dvb_ringbuffer_avail(&ep_out->erb.buffer);
	size_t left			= MIN(frame_bufsize, rb_avail);
	int cf, nf, ff = 0;
	int rc, more = 0;

	unsigned long flags = 0;

	if (left % TS_PACKET_SIZE)
		pr_warn("%s : TS-packets with %zu trailing bytes\n", __func__, left % TS_PACKET_SIZE);

	/* write only full micro-frames or reader returns undefined data up to uframe-size ! */
	/* write only full frames or interrupt urbs (ca-device) are blocked ! */
	/* start the urbs exactly at a SOF or interrupt urbs (ca-device) are blocked ! */
	/* 1 full frame -> 8 uframes * 4*188=752 = 6016 bytes per frame (1ms) ==> max.~48 Mbit/s */

	left -= left % TS_MIN_UF(uframe_size);
	if (!left)
		return 0;

	/* we have enough TS-data */
	urb_out		= ep_out->u.isoc.transfers[urb_index].urb;
	urb_in		= ep_in->u.isoc.transfers[urb_index].urb;
	num_uframes	= left / uframe_size;

	/* usb_submit_urb resets all .status and .actual_length fields */
	urb_out->number_of_packets = num_uframes;
	urb_in->number_of_packets = num_uframes;
	dvb_ringbuffer_read(&ep_out->erb.buffer, urb_out->transfer_buffer, left);

	more = ((rb_avail - left) >= TS_MIN_UF(uframe_size)); /* we can send more urbs */
	ci_dev->isoc_bytes_CAM += left;

	/* +++IMPORTANT++ align the first urb transfers to SOF ! */
	/* it seems, there is currently no way to set the urb start_frame */

	if (!urb_index) {
		cf = usb_get_current_frame_number(ci_dev->wintvci->udev);
		do {
			nf = usb_get_current_frame_number(ci_dev->wintvci->udev);
			ff++;
		} while (nf > -1 && nf == cf); // loops many 100 times !

		spin_lock_irqsave(&ci_dev->ci_lock, flags);
	}

	/* submit out-urb - at first */
	rc = usb_submit_urb(urb_out, GFP_ATOMIC);
	if (rc < 0)
		pr_warn("%s : Could not submit TS-OUT URB[%d]: (%d)\n",
						__func__, urb_index, rc);
	ci_dev->isoc_urbs_running++;

	/* submit in-urb - as second */
	rc = usb_submit_urb(urb_in, GFP_ATOMIC);
	if (rc < 0)
		pr_warn("%s : Could not submit TS-IN URB[%d]: (%d)\n",
						__func__, urb_index, rc);
	ci_dev->isoc_urbs_running++;

	if (!urb_index)
		spin_unlock_irqrestore(&ci_dev->ci_lock, flags);

	if (urb_out->start_frame != urb_in->start_frame)
		pr_warn("%s : diff. start_frames: out/in: %d/%d",
				__func__, urb_out->start_frame, urb_in->start_frame);
#if DEBUG_TS_IO
	pr_info("%s : --- %zu x TS, %2d uframes - rb-avail(%zu) CAM(%d) ff:%d\n",
				__func__, left / TS_PACKET_SIZE,
				num_uframes, rb_avail/TS_PACKET_SIZE,
				ci_dev->isoc_bytes_CAM/TS_PACKET_SIZE,
				ff);
#endif
	return more;
}

#define RB_READ_CONDITION(ep)  (dvb_ringbuffer_avail(&ep->erb.buffer) >= TS_PACKET_SIZE)
#define RB_WRITE_CONDITION(ep) (dvb_ringbuffer_free(&ep->erb.buffer) >= TS_PACKET_SIZE)

/* Host --> TS-OUT ringbuffer */
static void ts_CAM_exchange(struct ci_device *ci_dev)
{
	struct ep_info *ep_out	= &ci_dev->ep_isoc_out;
	struct isoc_info *isoc	= &ep_out->u.isoc;
	int i, more;

	if (ci_dev->isoc_urbs_running)
		if (wait_event_interruptible(
				ci_dev->isoc_urbs_wq,
				!ci_dev->isoc_urbs_running) < 0)
			return;

	/* short delay before submitting first urb */
#define URB_DELAY 5000
	usleep_range(URB_DELAY, URB_DELAY+1);

	for (i = 0; i < isoc->num_transfers; i++) {
		more = ts_write_CAM(ci_dev, i);
		if (!more)
			break;
	}
#if DEBUG_TS_IO
	if (i++)
	    pr_info("%s :  %d urb transfers", __func__, i); // shows 4 .. 6 Mbit/s within 1 transfer
#endif
}

static ssize_t ts_write(struct file *file, const __user char *buf,
			size_t count, loff_t *ppos)
{
	struct dvb_device *dvbdev	= file->private_data;
	struct wintv_ci_dev *wintvci	= dvbdev->priv;
	struct ci_device *ci_dev	= &wintvci->ci_dev;

	struct ep_info *ep		= &ci_dev->ep_isoc_out;
	struct dvb_ringbuffer *rb	= &ep->erb.buffer;

	size_t rb_free = dvb_ringbuffer_free(rb);
	size_t todo = MIN(rb_free, count);
	size_t written = 0;

	ci_dev->isoc_enabled = 1;

	if (!rb_free)
		pr_err("     *** TS-write : ringbuffer overrun\n");

	if (todo % TS_PACKET_SIZE)
		pr_info("%s : TS-packets with %zu trailing bytes(count %zu, free %zu)\n",
				__func__, todo % TS_PACKET_SIZE, count, rb_free);

	if (todo) {
		/* copy_from_user */
		written = dvb_ringbuffer_write_user(rb, buf, todo);
		if (written != todo)
			pr_err("     *** TS-write error written(%zu) != want(%zu)\n",
								written, todo);
		ci_dev->isoc_bytes_RB += written; /* ringbuffer write-read diff. */
#if DEBUG_TS_IO
		pr_info("     *** TS-write{%02X} (%zu)[%zu]<%zu> - rb(%d)\n", (u8) buf[0], count,
					todo, todo/TS_PACKET_SIZE, ci_dev->isoc_bytes_RB/TS_PACKET_SIZE);
#endif
	}
#define TS_COUNT_TIMEOUT (HZ * 10) /* secs */
	if (show_ts_bitrate) {
		unsigned long timer_now = jiffies;
		ci_dev->ts_count_interval += written; /* FIXME - handle overflow */

		if (ci_dev->ts_count_timeout <= timer_now) {
			int time = timer_now - ci_dev->ts_count_timeout + TS_COUNT_TIMEOUT;
			int mbitx100 = ci_dev->ts_count_interval/time*HZ*8; /* bits/second */
			mbitx100 /= (1000*1000/100);

			pr_info("     +++ TS-BITRATE : %d.%02d Mbit/s\n", mbitx100/100,mbitx100 % 100);
			ci_dev->ts_count_timeout = timer_now + TS_COUNT_TIMEOUT;
			ci_dev->ts_count_interval = 0;
		}
	}

	if (written)
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

	size_t read = 0;
	size_t avail = MIN(dvb_ringbuffer_avail(rb), count);

	if (avail) {
		/* copy_to_user */
		read = dvb_ringbuffer_read_user(rb, buf, avail);
		if (read != avail)
			pr_err("     *** TS-read error read(%zu) != want(%zu)\n",
								read, avail);
		ci_dev->isoc_bytes_RB -= read; /* ringbuffer read-write diff */
#if DEBUG_TS_IO
		pr_info("     *** TS-read{%02X} (%zu)[%zu]<%zu> - rb(%d)\n", (u8) buf[0], count,
					read, read/TS_PACKET_SIZE, ci_dev->isoc_bytes_RB/TS_PACKET_SIZE);
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

//	pr_info("%s\n",__func__);

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

	ci_dev->isoc_bytes_RB = 0;
	ci_dev->isoc_bytes_CAM = 0;

	ci_dev->ts_count_total = 0;
	ci_dev->ts_count_interval = 0;
	ci_dev->ts_count_timeout = jiffies + TS_COUNT_TIMEOUT;
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

	ci_isoc_init(ci_dev);

	ci_reset(wintvci);

	rc = dvb_register_device(&wintvci->adapter, &ci_dev->regdev_ci,
					&ci_dvbdev, wintvci, DVB_DEVICE_SEC, 0);

	return rc;
}
