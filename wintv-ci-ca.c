/*
 * wintv-ci-ca.c : WinTV-CI - USB2 Common Interface driver
 *
 * Copyright (C) 2017 Helmut Binder (cco@aon.at)
 #
 * (+HB+) 2017-08-13 
 * (+HB+) 2017-09-18 first descrambling
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
 *  C A - D E V I C E  ( T P D U )
 */

#include "wintv-ci.h"

/* show some TPDU massages */
#define DEBUG_TPDU

/* a lot of CA in/out massages */
//#define DEBUG_CA_IO

/* --- R I N G B U F F E R --- */

static void ca_rb_free(struct ep_info *ep)
{
	if (!ep)
		return;

	kfree(ep->erb.buffer.data);
	ep->erb.buffer.data = NULL;
}

static int ca_rb_alloc(struct ep_info *ep, int size)
{
	void *buf = ci_kmalloc(size, 0, (char *)__func__);
	if (!buf)
		return -ENOMEM;

	dvb_ringbuffer_init(&ep->erb.buffer, buf, size);
	ep->erb.num_items = 0;

	return 0;
}

/* ---  U S B - E N D P O I N T S --- */

static int ca_bulk_allocate(struct ep_info *ep)
{
	struct bulk_info *bulk	= &ep->u.bulk;
	int pkt_size		= CA_CTRL_MAXPKT;

	pr_info("%s : EP(%02X) init packet buffer: %d bytes\n",
				__func__, ep->addr,pkt_size);

	bulk->pkt.buffer = ci_kmalloc(pkt_size, 1, (char *)__func__);
	if (!bulk->pkt.buffer)
		return -ENOMEM;

	return 0;
}

static int ca_intr_allocate(struct ep_info *ep)
{
	struct intr_info *intr	= &ep->u.intr;
	int pkt_size		= CA_CTRL_MAXPKT;
	int msg_size		= CA_CTRL_MAXMSG;

	pr_info("%s : EP(%02X) init packet/message buffers: %d/%d bytes\n",
				__func__, ep->addr, pkt_size, msg_size);

	intr->pkt.buffer = ci_kmalloc(pkt_size, 1, (char *)__func__);
	if (!intr->pkt.buffer)
		return -ENOMEM;

	intr->msg.buffer = ci_kmalloc(msg_size, 1, (char *)__func__);
	if (!intr->msg.buffer) {
		kfree(intr->pkt.buffer);
		intr->pkt.buffer = NULL;
		return -ENOMEM;
	}

	return 0;
}

static void ca_intr_bulk_exit(struct ca_device *ca_dev)
{
	struct ep_info *ep;

	ep = &ca_dev->ep_intr_in;
	kfree(ep->u.intr.msg.buffer);
	kfree(ep->u.intr.pkt.buffer);
	ca_rb_free(ep);

	ep = &ca_dev->ep_bulk_out;
	kfree(ep->u.bulk.pkt.buffer);
	ca_rb_free(ep);
}

#define CA_RB_ITEMS 100 /* 100 x LL-size = ca. 16 x MAX_TPDU */

static int ca_intr_bulk_init(struct ca_device *ca_dev)
{
	struct ep_info *ep;
	int rc;

	ep = &ca_dev->ep_bulk_out;
	rc = ca_rb_alloc(ep, CA_RB_ITEMS * CA_LINK_LAYER_SIZE);
	if (rc)
		return rc;
	rc = ca_bulk_allocate(ep); /* only packet-out buffer */
	if (rc)
		return rc;

	ep = &ca_dev->ep_intr_in;
	rc = ca_rb_alloc(ep, CA_RB_ITEMS * CA_LINK_LAYER_SIZE);
	if (rc)
		return rc;
	rc = ca_intr_allocate(ep); /* pkt-in and msg-in buffers */
	if (rc)
		return rc;

	return rc;
}

/*
 * Kernel thread which monitors CAM changes.
 */

static int ca_thread(void *data)
{
	struct ca_device *ca_dev = data;

//	pr_info("%s starting\n", __func__);

	/* choose the initial delay */
	ca_dev->delay = HZ; /* 1 sec */

	/* main loop */
	while (!kthread_should_stop()) {
		/* sleep for a bit */
		if (!ca_dev->wakeup) {
			set_current_state(TASK_INTERRUPTIBLE);
			schedule_timeout(ca_dev->delay);
			if (kthread_should_stop())
				return 0;
		}
		ca_dev->wakeup = 0;

		//pr_info("%s working\n", __func__);
		cam_state_monitor(ca_dev->wintvci);
	}

	return 0;
}

/* --- T P D U --- */

static int ca_wait_for_status(struct wintv_ci_dev *wintvci, u8 mask,
					u8 *pstatus, unsigned int timeout_ms)
{
	unsigned long start, timeout;
	int rc, n = 0;

	if (!mask)
		return -EINVAL;

	start	= jiffies;
	timeout	= start + msecs_to_jiffies(timeout_ms);
	do {
		rc = CI_50_GetStatus(wintvci, pstatus);
		if (rc)
			return rc;

		if (*pstatus & mask) {
			if (n)
				pr_err("%s : succeeded %02X: %u ms (%d)\n",
					__func__, *pstatus,
					jiffies_to_msecs(jiffies-start), n);
			return 0;
		}
		/* check for timeout */
		if (time_after(jiffies, timeout))
			break;
		/* wait */
		usleep_range(500,1000);
		n++;
	} while (1);

	if (timeout_ms)
		pr_err("%s timed out: %u ms (%d)\n", __func__,
				jiffies_to_msecs(jiffies-start), n);

	return -ETIMEDOUT;
}

/* --- C A M --> H O S T --- */

/*
 * -- read one complete TPDU -- STATUSBIT_DA is SET --
 *
 * the STATUSBIT_DA never disappears until one full TPDU
 * is received -> no need to re-check the status
 */

static ssize_t CA_recv_TPDU(struct wintv_ci_dev *wintvci, u8 slot, u8 *msg_tcid,
				char *buf, size_t bufsize)
{
	/* param slot is ignored -> we have only 1 slot */
	struct msg_reply *reply;
	u8 tcid = 0; /* a reserved value - valid tcids are 1..255 */
	size_t ofs = 0;
	size_t free = bufsize;
	size_t frag_size;

	int frag_more = 1;

	reply = ci_kmalloc(sizeof *reply, 0, (char *)__func__);
	if (!reply)
		return -1;

	while (frag_more) {
		/* read one LPDU
		    [0] = tcid
		    [1] = more(0x80) | last(0x0)
		    [2..n] = TPDU-fragment
		*/
		if (CI_80_ReadLPDU(wintvci, reply))
			/* if (rc == 0x10) */
			goto error;

		if (!tcid) /* first fragment - save our tc-id */
			tcid = reply->buffer[0];

		if (tcid != reply->buffer[0]) { /* other tcid ? */
			pr_err("%s : *** SKIP *** tcid(%d) != %d\n",
					__func__, tcid, reply->buffer[0]);
			continue;
		}

		frag_more = (reply->buffer[1] == 0x80);
		frag_size = reply->size-2;

		if (free < 0)
			continue;	/* overflow - read in, but skip this TPDU */

		if (frag_size <= free)
			memcpy((void *)buf+ofs, reply->buffer+2, frag_size);
		else
			pr_err("%s : *** ERROR *** TPDU too large (> %d)\n",
							__func__, bufsize);
		ofs += frag_size;
		free -= frag_size;
	}
	if (free < 0)
		goto error;

	kfree(reply);
	*msg_tcid = tcid;
	return ofs;
error:
	kfree(reply);
	return 0;
}

static int rb_read_tpdu(struct wintv_ci_dev *wintvci)	// CAM INTR_IN --> ringbuffer
{
	struct ep_info *ep_in		= &wintvci->ca_dev.ep_intr_in;
	struct dvb_ringbuffer *rb	= &ep_in->erb.buffer;
	size_t rb_free			= dvb_ringbuffer_free(rb);

	u8 *buf = NULL;
	size_t size = 0;

	u8 slot = 0; /* read from slot 0 */
	u8 tcid = 0; /* get from received TPDU */

	//pr_info("%20s : %d\n", __func__, CA_CTRL_MAXTPDU);

	buf = ci_kmalloc(CA_CTRL_MAXTPDU, 0, (char *)__func__);
	if (!buf)
		return -1;

	size = CA_recv_TPDU(wintvci, slot, &tcid, buf+2, CA_CTRL_MAXTPDU - 2);

	if (size <= 0) {
		pr_err("%20s : FAILED\n", __func__);
		goto error;
	}

	/* fill header */
	buf[0] = slot;
	buf[1] = tcid;
	size += 2;

	if ((size + 2) > rb_free) {
		pr_err("%20s : CA_IN overflow of %d bytes\n", __func__,
						size + 2 - rb_free);
		goto error;
	}

	DVB_RINGBUFFER_WRITE_BYTE(rb, (size >> 8) & 0xff);
	DVB_RINGBUFFER_WRITE_BYTE(rb, size & 0xff);
	dvb_ringbuffer_write(rb, buf, size);
	ep_in->erb.num_items++;
	wake_up_interruptible(&ep_in->erb.wq); /* ep-in waitqueue */
#ifdef DEBUG_CA_IO
	pr_info("%20s : msg[%d] %5d bytes - rb-free(%d)\n",
				__func__, ep_in->erb.num_items, size,
				dvb_ringbuffer_free(rb));
#endif
#ifdef DEBUG_TPDU
	dump_io_tpdu((u8 *)buf, size, __func__, 1);
#endif
	kfree(buf);
	return 0;
error:
	kfree(buf);
	return -1;
}

#define TMO_STATUS_MS 500 /* ms */

static int ca_poll_tpdu(struct wintv_ci_dev *wintvci)
{
	unsigned char status;
	int rc;

	if (mutex_lock_interruptible(&wintvci->slot.cam_mutex))
		return -ERESTARTSYS;

	/* wait for FR, on DA fetch incoming TPDUs */
	do {
		rc = ca_wait_for_status(wintvci, STATUSBIT_DA | STATUSBIT_FR,
						&status, TMO_STATUS_MS);
		if (rc)
			break;
		if (status & STATUSBIT_FR)
			break;
		rc = rb_read_tpdu(wintvci);
		if (rc)
			break;
	} while (1);

	mutex_unlock(&wintvci->slot.cam_mutex);

	if (rc)
		pr_err("%20s : rc=0x%02X\n", __func__, rc);

	return rc;
}

/* --- H O S T --> C A M --- */

/*
 * -- send one full TPDU -- wait for STATUSBIT_FR --
 *
 # due late CAM replies the STATUSBIT_DA
 * can pop up at ANY time - check for it and
 * fetch incoming TPDUs before sending next fragment
 */

static ssize_t CA_send_TPDU(struct wintv_ci_dev *wintvci, u8 slot, u8 tcid,
				char *buf, size_t bufsize)
{
	/* param slot is ignored -> we have only 1 slot */
	char tpdu_frag[CA_LINK_LAYER_SIZE];

	size_t ofs = 0;
	size_t todo = bufsize;
	int rc;

	/* setup LPDU-header */
	tpdu_frag[0] = tcid;

	while (todo) {
		/* send one LPDU
		    [0] = tcid
		    [1] = more(0x80) | last(0x0)
		    [2..n] = TPDU-fragment
		*/
		size_t frag_size = MIN(todo, CA_LINK_LAYER_DATA);

		tpdu_frag[1] = (frag_size < todo) ? 0x80 : 0x00;
		memcpy(tpdu_frag+2, buf+ofs, frag_size);

		/* wait for FR, fetch incoming TPDUs */
		rc = ca_poll_tpdu(wintvci);
		if (rc)
			goto error;

		/* send LPDU */
		rc = CI_20_WriteLPDU(wintvci, tpdu_frag, frag_size+2);
		if (rc) {
			//if (rc == 0x20)
			goto error;
		}

		ofs += frag_size;
		todo -= frag_size;
	}
	return 0;
error:
	pr_err("%20s : *** FAILED[%d] *** (%d of %d bytes transmitted)\n",
		__func__, rc, ofs, bufsize);

	return rc;
}

static int rb_write_tpdu(struct wintv_ci_dev *wintvci)	// ringbuffer --> CAM BULK_OUT
{
	struct ep_info *ep_out		= &wintvci->ca_dev.ep_bulk_out;
	struct dvb_ringbuffer *rb	= &ep_out->erb.buffer;
	int rc;

	u8 slot, tcid;
	size_t count;
	u8 *buf;

	if (!ep_out->erb.num_items)
		return 0;

	count  = DVB_RINGBUFFER_PEEK(rb,0) << 8;
	count |= DVB_RINGBUFFER_PEEK(rb,1);

	buf = ci_kmalloc(count, 0, (char *)__func__);
	if (!buf)
		return -1;

	//pr_info("%20s : %d\n", __func__, count);

	DVB_RINGBUFFER_SKIP(rb, 2);
	dvb_ringbuffer_read(rb, buf, count);
	ep_out->erb.num_items--;
	wake_up_interruptible(&ep_out->erb.wq); /* ep-out waitqueue */

	/* read header */
	slot = buf[0];
	tcid = buf[1];
	count -= 2;

	rc = CA_send_TPDU(wintvci, slot, tcid, buf+2, count);
	if (rc)
		goto error;
#ifdef DEBUG_CA_IO
	pr_info("%20s : msg[%d] %5d bytes\n",
				__func__, ep_out->erb.num_items+1, count);
#endif
#ifdef DEBUG_TPDU
	dump_io_tpdu( (u8 *)buf, count+2, __func__, 0);
#endif
	rc = ca_poll_tpdu(wintvci);
	if (rc)
		goto error;

	kfree(buf);
	return 0;
error:
	kfree(buf);
	return -1;
}

/*
#define CA_RESET          _IO('o', 128)
#define CA_GET_CAP        _IOR('o', 129, ca_caps_t)
#define CA_GET_SLOT_INFO  _IOR('o', 130, ca_slot_info_t)
#define CA_GET_DESCR_INFO _IOR('o', 131, ca_descr_info_t)
#define CA_GET_MSG	  _IOR('o', 132, ca_msg_t)
#define CA_SEND_MSG       _IOW('o', 133, ca_msg_t)
#define CA_SET_DESCR      _IOW('o', 134, ca_descr_t)
*/

static int ca_ioctl(struct file *file, unsigned int cmd, void *parg) {

	struct dvb_device *dvbdev = file->private_data;
	struct wintv_ci_dev *wintvci = dvbdev->priv;
	struct ca_device *ca_dev = &wintvci->ca_dev;

	int rc = 0;

	if (mutex_lock_interruptible(&ca_dev->ca_ioctl_mutex))
		//return -ERESTARTSYS;
		return -1;

	switch (cmd) {
	case CA_RESET:
		mutex_lock(&wintvci->slot.cam_mutex);

		pr_info("%s : %s\n", __func__, "CA_RESET");
		cam_state_set(wintvci, USBCI_STATE_NON);

		mutex_unlock(&wintvci->slot.cam_mutex);
		break;

	case CA_GET_CAP: {
		struct ca_caps *caps = parg;

		pr_info("%s : %s\n", __func__, "CA_GET_CAP");
		caps->slot_num   = 1;
#if 0
		caps->slot_type  = CA_CI_LINK | CA_DESCR;
		caps->descr_num  = 1;
		caps->descr_type = CA_ECD;
#else
		caps->slot_type  = CA_CI_LINK;
		caps->descr_num  = 0;
		caps->descr_type = 0;
#endif
		break;
	}

	case CA_GET_SLOT_INFO: {
		struct ca_slot_info *info = parg;

		if (info->num != 0) { /* we have only 1 slot */
			rc = -EINVAL;
			goto out_unlock;
		}
		info->type  = CA_CI_LINK;
		info->flags = wintvci->slot.cam_state;

		if (info->flags != CA_CI_MODULE_READY)
		    pr_info("%s : %s [%d] : CAM %s\n", __func__,
			"CA_GET_SLOT_INFO",
			info->num,
			(info->flags == CA_CI_MODULE_READY) ? "READY" :
			(info->flags == CA_CI_MODULE_PRESENT) ? "PRESENT" :
			"NO_CAM");
		break;
	}

	case CA_GET_DESCR_INFO: {
		struct ca_descr_info *info = parg;

		pr_info("%s : %s\n", __func__, "CA_GET_DESCR_INFO");
		info->num  = 16;
		info->type = CA_ECD;
		break;
	}

	case CA_GET_MSG:
		pr_info("%s : %s\n", __func__, "--CA_GET_MSG");
		break;

	case CA_SEND_MSG:
		pr_info("%s : %s\n", __func__, "--CA_SEND_MSG");
		break;

	case CA_SET_DESCR:
		pr_info("%s : %s\n", __func__, "--CA_SET_DESCR");
		break;

	default:
		rc = -EINVAL;
		break;
	}

out_unlock:
	mutex_unlock(&ca_dev->ca_ioctl_mutex);
	return rc;
}

#define CA_READ_CONDITION(ep)  (ep->erb.num_items > 0)
#define CA_WRITE_CONDITION(ep) (dvb_ringbuffer_free(&ep->erb.buffer) > 2)

/*
 # buf[0] = slot
 # buf[1] = tcid
 # buf[2...n] = TPDU-Tag, rsn-size, TPDU-body ....
 */

static ssize_t ca_write(struct file *file, const char __user *buf,
			size_t count, loff_t *ppos) /* HOST -> RB */
{
	struct dvb_device *dvbdev	= file->private_data;
	struct wintv_ci_dev *wintvci	= dvbdev->priv;
	struct ca_device *ca_dev	= &wintvci->ca_dev;

	struct ep_info *ep_out		= &ca_dev->ep_bulk_out;
	struct dvb_ringbuffer *rb	= &ep_out->erb.buffer;

	int rb_free			= dvb_ringbuffer_free(rb);

	//pr_info("%s: %d\n", __func__, count);

	if (rb_free >= (count + 2)) {
		DVB_RINGBUFFER_WRITE_BYTE(rb, count >> 8);
		DVB_RINGBUFFER_WRITE_BYTE(rb, count & 0xff);
		dvb_ringbuffer_write_user(rb, buf, count);
		ep_out->erb.num_items++;
	}
	else {
		pr_err("%20s : msg[%d] too large - count/free %d/%d\n",
			__func__, ep_out->erb.num_items, count, rb_free);
		count = 0;
	}
#ifdef DEBUG_CA_IO
	pr_info("%20s : msg[%d] size %5d - rb-free(%d)\n",
			__func__, ep_out->erb.num_items-1, count,
			dvb_ringbuffer_free(rb));
#endif
	if (ep_out->erb.num_items)
		if (rb_write_tpdu(wintvci)) /* RB --> CAM BULK_OUT */
			return -1;
	return count;
}

static ssize_t ca_read( struct file *file, char __user *buf,
			size_t count, loff_t *ppos) /* RB -> HOST */
{
	struct dvb_device *dvbdev	= file->private_data;
	struct wintv_ci_dev *wintvci	= dvbdev->priv;
	struct ca_device *ca_dev	= &wintvci->ca_dev;

	struct ep_info *ep_in		= &ca_dev->ep_intr_in;
	struct dvb_ringbuffer *rb	= &ep_in->erb.buffer;

	size_t size = 0;

	//pr_info("%s: %d\n", __func__, count);

	if (!CA_READ_CONDITION(ep_in))	/* CAM -> RB */
		if (wait_event_interruptible(
					ep_in->erb.wq,
					CA_READ_CONDITION(ep_in)) < 0)
			return 0;

	ep_in->erb.num_items--;

	size = DVB_RINGBUFFER_PEEK(rb, 0) << 8;
	size |= DVB_RINGBUFFER_PEEK(rb, 1);

	DVB_RINGBUFFER_SKIP(rb, 2);
	if (count >= size)
		dvb_ringbuffer_read_user(rb, buf, size);
	else {
		pr_err("%20s : msg[%d] too large - count/size %d/%d\n",
			__func__, ep_in->erb.num_items, count, size);
		DVB_RINGBUFFER_SKIP(rb, size);
		size = 0;
	}
#ifdef DEBUG_CA_IO
	pr_info("%20s : msg[%d] size %5d - rb-free(%d)\n",
			__func__, ep_in->erb.num_items, size,
			dvb_ringbuffer_free(rb));
#endif
	return size;
}

#define PR_POLL_CNT 0x0F
static unsigned int ca_poll(struct file *file, poll_table *wait)
{
	struct dvb_device *dvbdev	= file->private_data;
	struct wintv_ci_dev *wintvci	= dvbdev->priv;
	struct ca_device *ca_dev	= &wintvci->ca_dev;

	struct ep_info *ep_in		= &ca_dev->ep_intr_in;
	struct ep_info *ep_out		= &ca_dev->ep_bulk_out;

	unsigned int mask = 0;

	//pr_info("%s:\n", __func__);

	poll_wait(file, &ep_in->erb.wq, wait);	/* CAM -> RB */
	poll_wait(file, &ep_out->erb.wq, wait);	/* RB -> CAM */

	if (CA_READ_CONDITION(ep_in)) 
		mask |= POLLIN | POLLRDNORM;
	if (CA_WRITE_CONDITION(ep_out))
		mask |= POLLOUT | POLLWRNORM;

	/* DEBUG */
	if ((ca_dev->ca_poll_cnt & PR_POLL_CNT) == 0) {
		pr_info("   *** CA-poll(%02X) [%d]***\n",
					mask, ca_dev->ca_poll_cnt);
		ca_dev->ca_poll_cnt = 0;
	}
	if (ca_dev->ca_last_poll_state != mask) {
		ca_dev->ca_last_poll_state = mask;
		ca_dev->ca_poll_cnt = 0;
	}
	ca_dev->ca_poll_cnt++;

	return mask;
}

static const struct file_operations ca_fops = {
	.owner = THIS_MODULE,

	.unlocked_ioctl = dvb_generic_ioctl,
	.read		= ca_read,
	.write		= ca_write,
	.poll		= ca_poll,

	.open		= dvb_generic_open,
	.release	= dvb_generic_release,
	.llseek		= noop_llseek,
};

static struct dvb_device ca_dvbdev = {
	.priv		= NULL,
	.users		= 1,
	.readers	= 1,
	.writers	= 1,
	.kernel_ioctl	= ca_ioctl,
	.fops		= &ca_fops
};

void ca_detach(struct wintv_ci_dev *wintvci)
{
	struct ca_device *ca_dev = &wintvci->ca_dev;

	if (!ca_dev->regdev_ca)
		return;

	pr_info("Detaching DVB CA Device\n");

	/* shutdown the thread if there was one */
	kthread_stop(ca_dev->thread);

	/* de-register ca-device */
	dvb_unregister_device(ca_dev->regdev_ca);

	/* free usb pkt/msg/ringbuffers */
	ca_intr_bulk_exit(ca_dev);
}

int ca_attach(struct wintv_ci_dev *wintvci)
{
	struct ca_device *ca_dev = &wintvci->ca_dev;
	int rc;

	pr_info("Attaching DVB CA Device\n");
	ca_dev->wintvci = wintvci;

	mutex_init(&ca_dev->ca_cmd_mutex);
	mutex_init(&ca_dev->ca_ioctl_mutex);
	mutex_init(&wintvci->slot.cam_mutex);

	init_waitqueue_head(&ca_dev->ep_intr_in.erb.wq);
	init_waitqueue_head(&ca_dev->ep_bulk_out.erb.wq);

	ca_dev->ca_poll_cnt = 0;
	ca_dev->ca_last_poll_state = 0;

	/* allocate usb pkt/msg/ringbuffers */
	rc = ca_intr_bulk_init(ca_dev);
	if (rc)
		return rc;

	/* register ca-device */
	rc = dvb_register_device(&wintvci->adapter, &ca_dev->regdev_ca,
					&ca_dvbdev, wintvci, DVB_DEVICE_CA, 0);
	if (rc)
		goto free_ep;

	/* create a kthread for monitoring this CA device */
	ca_dev->thread = kthread_run(ca_thread, ca_dev, "kdvb-ca-%i:%i",
				 ca_dev->regdev_ca->adapter->num,
				 ca_dev->regdev_ca->id);

	if (IS_ERR(ca_dev->thread)) {
		rc = PTR_ERR(ca_dev->thread);
		pr_err("%s: failed to start kernel_thread (%d)\n",
				__func__, rc);
		goto unregister;
	}
	return rc;

unregister:
	dvb_unregister_device(ca_dev->regdev_ca);
free_ep:
	ca_intr_bulk_exit(ca_dev);

	return rc;
}
