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

#include <linux/module.h>

/* a lot of CA in/out massages */
#define DEBUG_CA_IO 0

static int show_tpdu_info = 0;

module_param(show_tpdu_info, int, 0644);
MODULE_PARM_DESC(show_tpdu_info, " Show/Dump TPDU messages (default:off).");

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
}

#define CA_RB_ITEMS 100 /* 100 x LL-size = ca. 16 x MAX_TPDU */

static int ca_intr_bulk_init(struct ca_device *ca_dev)
{
	struct ep_info *ep;
	int rc;

	ep = &ca_dev->ep_bulk_out;
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

	return 0;
}

/*
 * Kernel thread which monitors CAM changes.
 */

// Wake up the CAM monitor

static void ca_thread_wakeup(struct ca_device *ca_dev)
{
//	pr_info("%s\n", __func__);

	ca_dev->ca_task.wakeup = 1;
	wake_up_process(ca_dev->ca_task.thread);
}

// run the CAM monitor

static int ca_thread(void *data)
{
	struct ca_device *ca_dev = data;

//	pr_info("%s starting\n", __func__);

	/* choose the initial delay */
	ca_dev->ca_task.delay = HZ * 2; /* 2 sec */
	ca_dev->ca_task.wakeup = 0;
	ca_dev->ca_task.running = 1;

	/* main loop */
	while (!kthread_should_stop()) {
		/* sleep if not woken up */
		if (!ca_dev->ca_task.wakeup) {
			set_current_state(TASK_INTERRUPTIBLE);
			schedule_timeout(ca_dev->ca_task.delay);
			if (kthread_should_stop())
				break;
		}
		ca_dev->ca_task.wakeup = 0;
		cam_state_monitor(ca_dev->wintvci);
	}
	ca_dev->ca_task.running = 0;
	do_exit(0);

	return 0;
}

/* --- T P D U --- */

static int ca_wait_for_status(struct ca_device *ca_dev, u8 mask,
					u8 *pstatus, unsigned int timeout_ms)
{
	unsigned long start, timeout;
	int rc, n = 0;

	if (!mask)
		return -EINVAL;

	start	= jiffies;
	timeout	= start + msecs_to_jiffies(timeout_ms);
	do {
		rc = CI_50_GetStatus(ca_dev->wintvci, pstatus);
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

static size_t CA_recv_TPDU(struct ca_device *ca_dev, u8 slot, u8 *msg_tcid,
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
		return 0;

	while (frag_more) {
		/* read one LPDU
		    [0] = tcid
		    [1] = more(0x80) | last(0x0)
		    [2..n] = TPDU-fragment
		*/
		if (CI_80_ReadLPDU(ca_dev->wintvci, reply))
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
			pr_err("%s : *** ERROR *** TPDU too large (> %zu)\n",
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

static int rb_read_tpdu(struct ca_device *ca_dev)	// CAM INTR_IN --> ringbuffer
{
	struct ep_info *ep_in		= &ca_dev->ep_intr_in;
	struct dvb_ringbuffer *rb	= &ep_in->erb.buffer;
	size_t rb_free			= dvb_ringbuffer_free(rb);

	u8 *buf = NULL;
	size_t size = 0;

	u8 slot = 0; /* read from slot 0 */
	u8 tcid = 0; /* get from received TPDU */

	//pr_info("%20s : %d\n", __func__, CA_CTRL_MAXTPDU);

	buf = ci_kmalloc(CA_CTRL_MAXTPDU, 0, (char *)__func__);
	if (!buf)
		return -ENOMEM;

	size = CA_recv_TPDU(ca_dev, slot, &tcid, buf+2, CA_CTRL_MAXTPDU-2);

	if (!size) {
		pr_err("%20s : FAILED\n", __func__);
		goto error;
	}

	/* fill header */
	buf[0] = slot;
	buf[1] = tcid;
	size += 2;

	if ((size + 2) > rb_free) {
		pr_err("%20s : CA_IN overflow of %zu bytes\n", __func__,
						size + 2 - rb_free);
		goto error;
	}

	DVB_RINGBUFFER_WRITE_BYTE(rb, (size >> 8) & 0xff);
	DVB_RINGBUFFER_WRITE_BYTE(rb, size & 0xff);
	dvb_ringbuffer_write(rb, buf, size);
	ep_in->erb.num_items++;
	wake_up_interruptible(&ep_in->erb.wq); /* ep-in waitqueue */
#if DEBUG_CA_IO
	pr_info("%20s : msg[%d] %5zu bytes - rb-free(%zu)\n",
				__func__, ep_in->erb.num_items, size,
				dvb_ringbuffer_free(rb));
#endif
	if (show_tpdu_info)
		dump_io_tpdu((u8 *)buf, size, __func__, 1);

	kfree(buf);
	return 0;
error:
	kfree(buf);
	return -1;
}

#define TMO_STATUS_MS 500 /* ms */

static int ca_poll_tpdu(struct ca_device *ca_dev)
{
	unsigned char status;
	int rc = 0;

	if (mutex_lock_interruptible(&ca_dev->ca_mutex))
		return -ERESTARTSYS;

	/* wait for FREE, on DATA fetch incoming TPDUs */
	do {
		rc = ca_wait_for_status(ca_dev, STATUSBIT_DA | STATUSBIT_FR,
						&status, TMO_STATUS_MS);
		if (!rc) {
			/* some CAMs (SmarDTV) have FR-bit allways set,
			   check for DA-bit first */
			if (status & STATUSBIT_DA)
				rc = rb_read_tpdu(ca_dev);
			else /* (status & STATUSBIT_FR) */
				break;
		}
	} while (!rc); // while DA-bit was set

	mutex_unlock(&ca_dev->ca_mutex);

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

static int CA_send_TPDU(struct ca_device *ca_dev, u8 slot, u8 tcid,
				char *buf, size_t len)
{
	/* param slot is ignored -> we have only 1 slot */
	char tpdu_frag[CA_LINK_LAYER_SIZE];

	size_t ofs = 0;
	size_t todo = len;
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
		rc = ca_poll_tpdu(ca_dev);
		if (rc)
			goto error;

		/* send LPDU */
		rc = CI_20_WriteLPDU(ca_dev->wintvci, tpdu_frag, frag_size+2);
		if (rc) {
			//if (rc == 0x20)
			goto error;
		}

		ofs += frag_size;
		todo -= frag_size;
	}
	return 0;
error:
	pr_err("%20s : *** FAILED[%d] *** (%zu of %zu bytes transmitted)\n",
		__func__, rc, ofs, len);

	return rc;
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

static int ca_ioctl(struct file *file, unsigned int cmd, void *parg)
{
	struct dvb_device *dvbdev = file->private_data;
	struct wintv_ci_dev *wintvci = dvbdev->priv;
	struct ca_device *ca_dev = &wintvci->ca_dev;

	switch (cmd) {
	case CA_RESET:
		pr_info("%s : %s\n", __func__, "CA_RESET");

		if (mutex_lock_interruptible(&ca_dev->ca_ioctl_mutex))
			//return -ERESTARTSYS;
			return -1;

		cam_state_set(wintvci, USBCI_STATE_RST);
		// wait until the reset is done
		wait_event_interruptible(
				wintvci->slot.cam_wq,
				wintvci->slot.usbci_state != USBCI_STATE_RST);
		ca_thread_wakeup(ca_dev);

		mutex_unlock(&ca_dev->ca_ioctl_mutex);
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

		if (info->num != 0) /* we have only 1 slot */
			//return -EINVAL;
			return -1;

		// if the cam_state has not changed since last query
		// slow down polling to the frequency of the cam_monitor events
		if (ca_dev->ca_cam_state != wintvci->slot.cam_state)
			wait_event_interruptible(
					wintvci->slot.cam_wq,
					true);

		info->type  = CA_CI_LINK;
		info->flags = wintvci->slot.cam_state;

		if (ca_dev->ca_cam_state != wintvci->slot.cam_state)
			pr_info("%s : %s [%d] : CAM %s\n", __func__,
				"CA_GET_SLOT_INFO",
				info->num,
				(info->flags == CA_CI_MODULE_READY) ? "READY" :
				(info->flags == CA_CI_MODULE_PRESENT) ? "PRESENT" :
				"REMOVED");
		ca_dev->ca_cam_state = wintvci->slot.cam_state;
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
		//return -EINVAL;
		return -1;
	}

	return 0;
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

	int rc;
	u8 slot, tcid;
	u8 *msg;

	//pr_info("%s: %d\n", __func__, count);

	if (count <= 0)
		return 0;

	msg = ci_kmalloc(count, 0, (char *)__func__);
	if (!msg)
		return -1;

	rc = copy_from_user(msg, buf, count);
	if (rc)
		goto done;

	/* read header */
	slot = msg[0];
	tcid = msg[1];

#if DEBUG_CA_IO
	pr_info("%20s : %d:%d size %5zu\n", __func__, slot, tcid, count-2);
#endif
	if (show_tpdu_info)
		dump_io_tpdu(msg, count, __func__, 0);

	rc = CA_send_TPDU(ca_dev, slot, tcid, msg+2, count-2); /* MSG --> CAM BULK_OUT */
	if (rc)
		goto done;

	rc = ca_poll_tpdu(ca_dev);
done:
	kfree(msg);
	return (rc) ? -1 : count;
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

	//pr_info("%s: %d / %d\n", __func__, count, ep_in->erb.num_items);

	if (!CA_READ_CONDITION(ep_in))
		goto done;

	ep_in->erb.num_items--;

	size = DVB_RINGBUFFER_PEEK(rb, 0) << 8;
	size |= DVB_RINGBUFFER_PEEK(rb, 1);
	DVB_RINGBUFFER_SKIP(rb, 2);

	if (count >= size)
		dvb_ringbuffer_read_user(rb, buf, size);
	else {
		pr_err("%20s : msg[%d] too large - count/size %zu/%zu\n",
			__func__, ep_in->erb.num_items, count, size);
		DVB_RINGBUFFER_SKIP(rb, size);
		size = 0;
	}
#if DEBUG_CA_IO
	pr_info("%20s : msg[%d] size %5zu - rb-free(%zu)\n",
			__func__, ep_in->erb.num_items, size,
			dvb_ringbuffer_free(rb));
#endif
done:
	return size;
}

static unsigned int ca_poll(struct file *file, poll_table *wait)
{
	struct dvb_device *dvbdev	= file->private_data;
	struct wintv_ci_dev *wintvci	= dvbdev->priv;
	struct ca_device *ca_dev	= &wintvci->ca_dev;

	struct ep_info *ep_in		= &ca_dev->ep_intr_in;

	unsigned int mask = POLLOUT;

	//pr_info("%s:\n", __func__);

	if (!CA_READ_CONDITION(ep_in))
		poll_wait(file, &ep_in->erb.wq, wait);	/* CAM -> RB */

	if (CA_READ_CONDITION(ep_in))
		mask |= POLLIN;

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
	if (ca_dev->ca_task.running)
		kthread_stop(ca_dev->ca_task.thread);

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

	mutex_init(&ca_dev->ca_mutex);
	mutex_init(&ca_dev->ca_ioctl_mutex);

	init_waitqueue_head(&ca_dev->ep_intr_in.erb.wq);

	ca_dev->ca_cam_state = -1;

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
	ca_dev->ca_task.thread = kthread_run(ca_thread, ca_dev, "kdvb-ca-%i:%i",
					     ca_dev->regdev_ca->adapter->num,
					     ca_dev->regdev_ca->id);

	if (IS_ERR(ca_dev->ca_task.thread)) {
		rc = PTR_ERR(ca_dev->ca_task.thread);
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
