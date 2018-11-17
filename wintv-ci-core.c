/*
 * wintv-ci-core.c : WinTV-CI - USB2 Common Interface driver
 *
 * Copyright (C) 2017 Helmut Binder (cco@aon.at)
 #
 * (+HB+) 2017-08-13
 * (+HB+) 2017-09-18 first descrambling
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

#include "wintv-ci.h"

#include <linux/module.h>
#include <linux/firmware.h>

#include <linux/kref.h>

#include <linux/delay.h>
#include <linux/mutex.h>

DVB_DEFINE_MOD_OPT_ADAPTER_NR(adapter_nr);

static const struct ezusb_fx_type ezusb_fx = { /* AN21.., FX */
	.cpucs_reg = 0x7F92,
	.internal_ram_size = 0x1B40
};

static const struct ezusb_fx_type ezusb_fx2 = { /* FX2  */
	.cpucs_reg = 0xE600,
	.internal_ram_size = 0x2000
};

static const struct ezusb_fx_type ezusb_fx2lp = { /* FX2LP */
	.cpucs_reg = 0xE600,
	.internal_ram_size = 0x4000
};

static struct usb_id_info wintv_ci_info = {
	.fw_ci_name = "wintvci_r%d.fw",
	.fw_cb_name = "wintvci_cb.fw",
	.max_ver_hw = 4,
	.max_ver_fw = 4,
	.fx = &ezusb_fx2lp,
};

static struct usb_id_info usb2_ci_info = {
	.fw_ci_name = "usb2ci_r%d.fw",
	.fw_cb_name = "usb2ci_cb.fw",
	.max_ver_hw = 4,
	.max_ver_fw = 3,
	.fx = &ezusb_fx2lp,
};

/*
 *  E Z U S B
 */

/* FX vendor request for accessing RAM/ROM */
#define VR_A0_INTRAM 0xA0 /* handled internal by CPU */
#define VR_A2_EEPROM 0xA2 /* handled with code-bulker firmware only */
#define VR_A3_EXTRAM 0xA3 /* handled with code-bulker firmware only */

#define USB_CTL_TIMEOUT 2000
#define USB_CMD_TIMEOUT 5000 /* 5 seconds */

static int ezusb_ctrl_write(struct wintv_ci_dev *wintvci,
				unsigned char request,
				int address, unsigned char *data, int length)
{
	int rc;

	if (!wintvci)
		return -ENODEV;

	if (length > USB_EP0_SIZE)
		return -EINVAL;

	if (request == VR_A2_EEPROM) {
		pr_warn("Writing to EEPROM can brick the device and \
			therefore blocked by driver!\n");
		return 0;
	}

	/* move data (possibly) out of stack -> kernel 4.13.3 */
	memcpy(wintvci->ep0_buffer, data, length);

	rc = usb_control_msg(	wintvci->udev,
				usb_sndctrlpipe(wintvci->udev, USB_EP0_ADDR),
				request,
				USB_DIR_OUT | USB_TYPE_VENDOR | USB_RECIP_DEVICE,
				address, 0, wintvci->ep0_buffer, length,
				USB_CTL_TIMEOUT);
	return rc;
}

static int ezusb_ctrl_read(struct wintv_ci_dev *wintvci,
				unsigned char request,
				int address, unsigned char *data, int length)
{
	int rc;

	if (!wintvci)
		return -ENODEV;

	if (length > USB_EP0_SIZE)
		return -EINVAL;

	rc = usb_control_msg(	wintvci->udev,
				usb_rcvctrlpipe(wintvci->udev, USB_EP0_ADDR),
				request,
				USB_DIR_IN | USB_TYPE_VENDOR | USB_RECIP_DEVICE,
				address, 0, wintvci->ep0_buffer, length,
				USB_CTL_TIMEOUT);
	if (rc == length)
		memcpy(data, wintvci->ep0_buffer, length);
	return rc;
}

static int ezusb_cpucs_reset(struct wintv_ci_dev *wintvci,
			unsigned short cpucs_reg, unsigned char stop_bit)
{
	int rc;

	//pr_info("%s - %s\n",__func__, (stop_bit) ? "STOP" : "START");
	rc = ezusb_ctrl_write(wintvci, VR_A0_INTRAM, cpucs_reg, &stop_bit, 1);
	if (rc != 1)
		pr_err("%s(%d) failed: %d\n",__func__, stop_bit, rc);

	return rc;
}

#define EZ_CPU_STOP(u,fx)  ezusb_cpucs_reset(u, fx->cpucs_reg, 1)
#define EZ_CPU_START(u,fx) ezusb_cpucs_reset(u, fx->cpucs_reg, 0)

static int usb_ctrl_req_set_config(struct wintv_ci_dev *wintvci, int cfg)
{
	int rc = usb_control_msg(wintvci->udev,
				usb_sndctrlpipe(wintvci->udev, USB_EP0_ADDR),
				USB_REQ_SET_CONFIGURATION,
				USB_DIR_OUT | USB_RECIP_DEVICE,
				cfg, 0, NULL, 0,
				USB_CTL_TIMEOUT);
	return rc;
}

/*
 *  W I N T V - C I   C O M M A N D S
 */

enum { /* commands and expected reply values */
	CI_00_undef		= 0x00,
	CI_10_HW_RESET		= 0x10 | 0x2,
	CI_20_LPDU_WRITE	= 0x20 | 0x0,
	CI_30_undef		= 0x30,
	CI_40_GET_CIS 		= 0x40 | 0x3,
	CI_50_STATUS		= 0x50 | 0x5,
	CI_60_NEGOTIATE		= 0x60 | 0x4,
	CI_70_WRITE_COR		= 0x70 | 0x7,
	CI_80_LPDU_READ		= 0x80 | 0x6,
	CI_90_GET_VER		= 0x90 | 0x8,

	CI_CMD_SEND_MASK	= 0xF0,
	CI_CMD_REPLY_MASK	= 0x0F
};

enum { /* reply error values */
	CI_ERR_10_LPDU_READ	= 0x10,
	CI_ERR_20_LPDU_WRITE	= 0x20,
	CI_ERR_30_NO_CAM	= 0x30,
	CI_ERR_60_TIMEOUT	= 0x60,
	CI_ERR_90_CMD_INV	= 0x90,

	CI_ERR_E0_USB		= 0xE0,

	CI_ERR_ERROR_MASK	= 0xF0
};

struct ci_cmd_info {
	const char	*name;
	int		on_state;
	int		to_state;
};

static struct ci_cmd_info CI_CMD_INFO[] = {
	{ "CI_00_undef", 0,0 },
	{ "CI_10_HW_RESET",	0,		 USBCI_STATE_CIS },
	{ "CI_20_LPDU_WRITE",	USBCI_STATE_RDY, 0 },
	{ "CI_30_undef", 0,0 },
	{ "CI_40_GET_CIS",	USBCI_STATE_CIS, USBCI_STATE_COR },
	{ "CI_50_STATUS",	USBCI_STATE_RDY, 0 }, /* timeout if no link */
	{ "CI_60_NEGOTIATE",	USBCI_STATE_LNK, USBCI_STATE_RDY },
	{ "CI_70_WRITE_COR",	USBCI_STATE_COR, USBCI_STATE_LNK },
	{ "CI_80_LPDU_READ",	USBCI_STATE_RDY, 0 },
	{ "CI_90_GET_VER",	0,		 0 }
};

#define DBG_CMD_MASK(c) ( 1 << ((c) >> 4) )

#define INF_CMD_N \
	( DBG_CMD_MASK(CI_20_LPDU_WRITE) \
	| DBG_CMD_MASK(CI_80_LPDU_READ) \
	| DBG_CMD_MASK(CI_50_STATUS) )

#define INF_CMD_Y \
	( ~INF_CMD_N )

#define DEB_CMD_Y \
	( 0 )

#define SHOW_CMD_INF 0x1
#define SHOW_CMD_DEB 0x2

static int CI_send_CMD(struct wintv_ci_dev *wintvci, u8 CI_CMD_S,
					char *data, int data_len, int debug)
{
	struct usb_device *udev	= wintvci->udev;
	struct ep_info *ep	= &wintvci->ca_dev.ep_bulk_out;
	struct bulk_info *bulk	= &ep->u.bulk;

	char *msg_ptr		= data;
	int rem_size		= data_len;
	int frag_len, transmitted, rc;

	if (debug & SHOW_CMD_DEB)
		print_hex_dump(KERN_DEBUG, " CI_send_CMD : ",
					DUMP_PREFIX_OFFSET, 16, 1,
					data, data_len, 1);

	if (data_len > CA_CTRL_MAXMSG) { /* == CA_LINK_LAYER_SIZE */
		pr_err("%s msg (%d) too large for link-layer (%d)\n",
					__func__, data_len, CA_CTRL_MAXMSG);
		return CI_ERR_20_LPDU_WRITE;
	}

	bulk->pkt.hdr->cmd	= CI_CMD_S;
	bulk->pkt.hdr->slot	= 0;
	bulk->pkt.hdr->xFF	= 0xff;

	do {	/* run command at least once */
		bulk->pkt.hdr->len = rem_size;

		frag_len = min(rem_size, CA_CTRL_MAXPKT_DATA);
		rem_size -= frag_len;

		memcpy(&bulk->pkt.hdr->data, msg_ptr, frag_len);
		msg_ptr += frag_len;

		if ((data_len > CA_CTRL_MAXPKT_DATA) || (debug & SHOW_CMD_INF))
			pr_info("%-20s: [%02X] %d/%d/%d\n", __func__,
				bulk->pkt.hdr->cmd, bulk->pkt.hdr->len,
				frag_len, rem_size);

		rc = usb_bulk_msg(udev, ep->pipe,
					bulk->pkt.buffer,
					frag_len+4,
					&transmitted, USB_CMD_TIMEOUT);

		if (rc) {
			pr_err("%-20s: [%02X] rc: %d (w:%d)\n",
				__func__, CI_CMD_S, rc,
				data_len-rem_size);
			return CI_ERR_E0_USB;
		}

	} while (rem_size);

	return 0;
}

static int CI_read_CMD_REPLY(struct wintv_ci_dev *wintvci, u8 CI_CMD_R,
					struct msg_reply *reply, int debug)
{
	struct usb_device *udev		= wintvci->udev;
	struct ep_info *ep		= &wintvci->ca_dev.ep_intr_in;
	struct intr_info *intr		= &ep->u.intr;

	unsigned char *msg_ptr		= (reply) ? reply->buffer : intr->msg.buffer;
	int msg_len			= -1;
	int rem_size			= CA_CTRL_MAXMSG;
	int frag_len, transmitted, rc;

	do {
		rc = usb_interrupt_msg(udev, ep->pipe,
					intr->pkt.buffer,
					CA_CTRL_MAXPKT,
					&transmitted, USB_CMD_TIMEOUT);

		if (rc || (intr->pkt.hdr->reply != CI_CMD_R)) {
			if (intr->pkt.hdr->reply != CI_ERR_30_NO_CAM)
				pr_err("%-20s: [%02X] rc: %d s:%02X f:%02X r:%02X l:%d\n",
					__func__, CI_CMD_R, rc,
					intr->pkt.hdr->state, intr->pkt.hdr->flag,
					intr->pkt.hdr->reply, intr->pkt.hdr->len);
			if (rc)
				return CI_ERR_E0_USB;		/* USB error */
			else if (intr->pkt.hdr->reply & CI_ERR_ERROR_MASK)
				return intr->pkt.hdr->reply;	/* CMD error */
			else
				continue; /* ignore reply from allready timedout cmd */
		}

		if (msg_len < 0) /* get the total msg-len */
			msg_len = intr->pkt.hdr->len;
		else if (intr->pkt.hdr->len != rem_size) {/* shpuld never happen */
			pr_err("%20s: [%02X] data with unexpected len (%d != %d)\n",
					__func__, intr->pkt.hdr->reply,
					intr->pkt.hdr->len, rem_size);
			return CI_ERR_10_LPDU_READ;
		}

		frag_len = min(intr->pkt.hdr->len, (unsigned char)CA_CTRL_MAXPKT_DATA);
		rem_size = intr->pkt.hdr->len - frag_len;

		memcpy(msg_ptr, &intr->pkt.hdr->data, frag_len);
		msg_ptr += frag_len;

		if ((msg_len > CA_CTRL_MAXPKT_DATA) || (debug & SHOW_CMD_INF))
			pr_info("%-20s: [%02X] %d/%d/%d\n",
				__func__, intr->pkt.hdr->reply,
				intr->pkt.hdr->len, frag_len, rem_size);
	} while (rem_size);

	if (reply)
		reply->size = msg_len;
	else
		intr->msg.size = msg_len;

	if (debug & SHOW_CMD_DEB)
		print_hex_dump(KERN_DEBUG, " CI_read_CMD : ",
					DUMP_PREFIX_OFFSET, 16, 1,
					msg_ptr - msg_len, msg_len, 1);
	return 0;
}

int cam_state_set(struct wintv_ci_dev *wintvci, int usbci_state)
{
	if (!usbci_state || wintvci->slot.usbci_state == usbci_state)
		return 0;

	pr_info("%-20s: %02X -> %02X\n",
			__func__, wintvci->slot.usbci_state, usbci_state);

	switch(usbci_state) {
	case USBCI_STATE_NON:
	default:
		wintvci->slot.cam_state  = 0;
		wintvci->last_status     = 0;
		wintvci->last_status_cnt = 0;
		break;
	case USBCI_STATE_CAM:
	case USBCI_STATE_RST:
	case USBCI_STATE_CIS:
	case USBCI_STATE_COR:
	case USBCI_STATE_LNK:
		wintvci->slot.cam_state = CA_CI_MODULE_PRESENT;
		break;
	case USBCI_STATE_RDY:
		wintvci->slot.cam_state = CA_CI_MODULE_READY;
		break;
	}
	wintvci->slot.usbci_state       = usbci_state;
	return 0;
}

static int CI_WriteExchange(struct wintv_ci_dev *wintvci, u8 CI_CMD,
				struct msg_reply *reply,
				char *data, int data_len) {

	unsigned char CI_CMD_S = CI_CMD & CI_CMD_SEND_MASK;
	unsigned char CI_CMD_R = CI_CMD & CI_CMD_REPLY_MASK;

	struct ci_cmd_info *cinfo = &CI_CMD_INFO[CI_CMD_S >> 4];

	unsigned long t_start;
	unsigned int t_response;
	int debug_mask, debug;
	int curr_state, rc;

	if (mutex_lock_interruptible(&wintvci->usb_mutex))
		return -ERESTARTSYS;

	curr_state = wintvci->slot.usbci_state;
	if (cinfo->on_state && (curr_state != cinfo->on_state)) {
		pr_err("%s *** invalid usbci-state (%02X/%02X) ***\n", __func__,
					curr_state, cinfo->on_state);
		rc = CI_ERR_90_CMD_INV;
		goto done;
	}

	debug_mask = DBG_CMD_MASK(CI_CMD_S);
	debug = ((debug_mask & DEB_CMD_Y) ? SHOW_CMD_DEB : 0) |
		((debug_mask & INF_CMD_Y) ? SHOW_CMD_INF : 0);

	t_start = jiffies;
	rc = CI_send_CMD(wintvci, CI_CMD_S, data, data_len, debug);
	if (rc) {
		pr_err("XW-rc(%d != %d) *** CAM-ERROR(%02X) ***\n",
						rc, data_len, CI_CMD_S);
		goto done;
	}

	rc = CI_read_CMD_REPLY(wintvci, CI_CMD_R, reply, debug);
	t_response = jiffies_to_msecs(jiffies - t_start);

	if (t_response > 50) /* typical 30ms - 33ms */
		pr_info("%-20s: [%02X/%02X] response after %u ms\n",
				__func__, CI_CMD_S, CI_CMD_R, t_response);

	if (rc && (rc != CI_ERR_30_NO_CAM))	/* FAILURE */
		pr_err("XR-rc(%d != %d) *** CAM-ERROR(%02X) ***\n",
						rc, CI_CMD_R, CI_CMD_S);
done:
	if (!rc) {	/* SUCCESS */
		if (cinfo->to_state)
			cam_state_set(wintvci, cinfo->to_state); /* set new cam-state */
	}
	else if (rc != CI_ERR_90_CMD_INV)
		cam_state_set(wintvci, USBCI_STATE_NON);

	mutex_unlock(&wintvci->usb_mutex);
	return rc;
}

/* --- */

static int CI_10_HwReset(struct wintv_ci_dev *wintvci)
{
	char buf[1] = { 0xF }; /* FW-Ver. 3 and 4 (?) */

	int rc = CI_WriteExchange(wintvci, CI_10_HW_RESET, NULL, buf, sizeof buf);
	return rc;
}

int CI_20_WriteLPDU(struct wintv_ci_dev *wintvci, char *data, int len)
{
	int rc = CI_WriteExchange(wintvci, CI_20_LPDU_WRITE, NULL, data, len);
	return rc;
}

static int CI_40_GetCIS(struct wintv_ci_dev *wintvci)
{
	struct msg_reply reply;

	int rc = CI_WriteExchange(wintvci, CI_40_GET_CIS, &reply, NULL, 0);
	if(!rc)
		rc = parse_cis(reply.buffer, reply.size, &wintvci->slot);
	return rc;
}

#define DGB_STATUS 0
int CI_50_GetStatus(struct wintv_ci_dev *wintvci, unsigned char *status)
{
	struct msg_reply reply;
	unsigned char cstat;

	int rc = CI_WriteExchange(wintvci, CI_50_STATUS, &reply, NULL, 0);
	if (rc)
		return rc;

	cstat = *status = reply.buffer[0];
#if DGB_STATUS
	if (cstat != wintvci->last_status ) {
		pr_info("STATUS : %02X --> %02X ( %d polls )\n",
			wintvci->last_status, cstat, wintvci->last_status_cnt);
		wintvci->last_status = cstat;
		wintvci->last_status_cnt = 0;
	}
	else
		wintvci->last_status_cnt++;
#endif
	return rc;
}
#undef DGB_STATUS

static int CI_60_Negotiate(struct wintv_ci_dev *wintvci,
						unsigned short link_size)
{
	struct msg_reply reply;
	unsigned short nego_size;
	char buf[2];
	int rc;

	buf[0] = (link_size >> 8) & 0xff; /* msb */
	buf[1] = link_size & 0xff;
	rc = CI_WriteExchange(wintvci, CI_60_NEGOTIATE, &reply, buf, sizeof buf);
	if (!rc) {
		nego_size =	reply.buffer[0] << 8 |
				reply.buffer[1];
		if (nego_size != link_size)
			pr_info("LINK_LAYER SIZE : %d bytes\n", nego_size);
		wintvci->slot.link_layer_size = nego_size;
	}
	return rc;
}

static int CI_70_WriteCOR(struct wintv_ci_dev *wintvci,
			unsigned short cfg_base, unsigned char cfg_option)
{
	char buf[5];
	int rc;

	buf[0] = cfg_base & 0xff; /* lsb */
	buf[1] = (cfg_base >> 8) & 0xff;
	buf[2] = cfg_option;
	buf[3] = 0; /* FW-Ver. 3 and 4 (?) */
	buf[4] = 0;

	rc = CI_WriteExchange(wintvci, CI_70_WRITE_COR, NULL, buf, sizeof buf);
	return rc;
}

int CI_80_ReadLPDU(struct wintv_ci_dev *wintvci, struct msg_reply *reply)
{
	int rc = CI_WriteExchange(wintvci, CI_80_LPDU_READ, reply, NULL, 0);
	return rc;
}

static int CI_90_GetVersion(struct wintv_ci_dev *wintvci,
						struct msg_reply *reply)
{
	int rc = CI_WriteExchange(wintvci, CI_90_GET_VER, reply, NULL, 0);
	return rc;
}

int cam_state_monitor(struct wintv_ci_dev *wintvci)
{
	unsigned short link_size;
	unsigned char status;
	int rc = 0;

	//pr_info(">>> %s\n",__func__);
	switch(wintvci->slot.usbci_state) {
	case USBCI_STATE_NON:
	case USBCI_STATE_CAM:
	case USBCI_STATE_RST:
		rc = CI_10_HwReset(wintvci);
		if (rc)
			break;
		ci_reset(wintvci);

	case USBCI_STATE_CIS:
		rc = CI_40_GetCIS(wintvci);
		if (rc)
			break;

	case USBCI_STATE_COR:
		rc = CI_70_WriteCOR(wintvci,
					wintvci->slot.config_base,
					wintvci->slot.config_option);
		if (rc)
			break;

	case USBCI_STATE_LNK:
		link_size = CA_LINK_LAYER_SIZE;
		// link_size = 0xffff; /* returns a max. link-layer size of 0x400 (1024.) bytes */
		rc = CI_60_Negotiate(wintvci, link_size);
		if (rc)
			break;

	case USBCI_STATE_RDY:
		rc = CI_50_GetStatus(wintvci, &status);
		break;

	default:
		cam_state_set(wintvci, USBCI_STATE_NON);
		break;
	}

	wake_up_interruptible(&wintvci->slot.cam_wq);
	return rc;
}

/*** U T I L I T I E S ***/

void *ci_kmalloc(int size, int zero, char * caller)
{
	unsigned char *buf;

	buf = (zero)
		? kzalloc(size, GFP_KERNEL)
		: kmalloc(size, GFP_KERNEL);

	if (!buf)
		pr_err("%s - failed to allocate %d bytes of memory\n",
							caller, size);

	return buf;
}

/*
 *  F I R M W A R E
 */

#define FW_ADR_TOP 0x4000 /* all extracted firmwares are below this address */

struct fw_block_header {
	unsigned short clen;
	unsigned short cadr;
	unsigned char last;
	unsigned char cdata;
} __attribute__ ((packed));

struct eeprom_header {
	unsigned char eboot;
	unsigned short vid;
	unsigned short pid;
	unsigned char didH;
	unsigned char didL;
	unsigned char ecfg;
} __attribute__ ((packed));

static int wintv_usb_ci_parse_firmware( const struct firmware *fw,
					int *adr_min, int *adr_max)
{
	int n;
	unsigned char block_sizes[] = { 0x46, 0x16 }; /* possible fw block-sizes */

	for (n = 0; n < sizeof(block_sizes); n++) {
		int block_size = block_sizes[n];
		int num_blocks = fw->size/block_size;
		int ram_min = FW_ADR_TOP;
		int ram_top = 0;
		const unsigned char *cptr;
		int i;

		/* even block-count ? */
		if (!num_blocks || (fw->size % block_size))
			continue;

		for (i = 1, cptr = fw->data; i <= num_blocks; i++, cptr += block_size) {

			struct fw_block_header *fwbh = (struct fw_block_header *) cptr;

			if (i < num_blocks) {
				if (	(fwbh->last == 0) &&
					fwbh->clen &&
					(fwbh->clen < (block_size-5)) &&
					(fwbh->cadr <= (FW_ADR_TOP - fwbh->clen))) { /* valid block */

					if (fwbh->cadr < ram_min)
						ram_min = fwbh->cadr;
					if ((fwbh->cadr+fwbh->clen) > ram_top)
						ram_top = fwbh->cadr + fwbh->clen;
					continue;
				}
			}
			else { /* last block */
				if (	(fwbh->last == 1) &&
					(fwbh->clen == 0) &&
					(fwbh->cadr == 0)) { /* FW end-marker found */
					*adr_min = ram_min;
					*adr_max = ram_top-1;
					pr_info("*** FW *** block-size %02X, block-cnt %d, RAM address range: 0x%04X - 0x%04X\n",
							block_size, num_blocks, ram_min, ram_top);
					return block_size;
				}
			}
			break; /* bad block -> check fw with next block-size */
		}
	}

	pr_info("*** FW *** Error: Invalid Firmware\n");

	return 0;
}

static int wintv_usb_ci_load_firmware(  struct wintv_ci_dev *wintvci,
					char *fw_name )
{
	const struct firmware *fw = NULL;
	struct usb_device *udev = wintvci->udev;

	unsigned char *fw_code = NULL;
	unsigned char *cptr;
	int adr_min, adr_max, block_size, num_blocks, blocks_written;
	int use_ext_ram = 0;
	int i;

	int rc = request_firmware(&fw, fw_name, &udev->dev);
	if (rc)
		return rc;

	pr_info("located firmware %s, size %zu bytes\n",fw_name, fw->size);
	block_size = wintv_usb_ci_parse_firmware(fw, &adr_min, &adr_max);
	if (!block_size)
		goto fw_error;

	num_blocks = fw->size/block_size;
	use_ext_ram = (adr_max >= wintvci->info->fx->internal_ram_size);

	if (use_ext_ram && (wintvci->fw_state != FW_STATE_EZUSB)) {
		pr_err("Can't write to external ram without code-bulker firmware loaded\n");
		goto fw_error;
	}

	fw_code = ci_kmalloc(fw->size, 0, (char *)__func__);
	if (!fw_code)
		goto fw_error;

	memcpy(fw_code,fw->data,fw->size);

	 /* load code into external ram */
	if (use_ext_ram) {
		for ( i = 1, cptr = fw_code, blocks_written = 0; i < num_blocks; i++, cptr += block_size ) { /* skip last block */
			struct fw_block_header *fwbh = (struct fw_block_header *) cptr;

			if (fwbh->cadr >= wintvci->info->fx->internal_ram_size) {
				//pr_info("extram adr 0x%04X, len 0x%X bytes\n",fwbh->cadr, fwbh->clen);

				rc = ezusb_ctrl_write( wintvci,
					VR_A3_EXTRAM, fwbh->cadr, &fwbh->cdata, fwbh->clen );

				if (rc != fwbh->clen)
					goto wr_error;
				blocks_written++;
			}
		}
		pr_info(" * %d firmware blocks written to external RAM\n", blocks_written);
	}
	/* load code into internal ram */
	EZ_CPU_STOP(wintvci,wintvci->info->fx);

	for ( i = 1, cptr = fw_code, blocks_written = 0; i < num_blocks; i++, cptr += block_size ) { /* skip last block */
		struct fw_block_header *fwbh = (struct fw_block_header *) cptr;

		if (fwbh->cadr < wintvci->info->fx->internal_ram_size) {
			//pr_info("intram adr 0x%04X, len 0x%X bytes\n",fwbh->cadr, fwbh->clen);

			rc = ezusb_ctrl_write( wintvci,
				VR_A0_INTRAM, fwbh->cadr, &fwbh->cdata, fwbh->clen );

			if (rc != fwbh->clen)
				goto wr_error;
			blocks_written++;
		}
	}
	EZ_CPU_START(wintvci,wintvci->info->fx);

	pr_info(" * %d firmware blocks written to internal RAM\n", blocks_written);

	kfree(fw_code);
	release_firmware(fw);
	return 0;
wr_error:
	kfree(fw_code);
fw_error:
	release_firmware(fw);
	return -EINVAL;
}

static void wintv_usb_ci_show_hw_info( struct wintv_ci_dev *wintvci )
{
	unsigned char eebuf[16];
	unsigned char regval;
	int rc = 0;

	if (wintvci->fw_state != FW_STATE_EZUSB) {
		pr_err("Can't show harware-info without code-bulker firmware loaded\n");
		return;
	}

	rc = ezusb_ctrl_read( wintvci,
				VR_A2_EEPROM, 0, eebuf, sizeof eebuf);
	if (rc == sizeof eebuf) {
		struct eeprom_header *ee_info = (struct eeprom_header *)eebuf;
		unsigned char eecmp[sizeof eebuf];
		int i;

		pr_info(" * EEPROM: Boot-Mode: %02X, V:P:D: %04X:%04X-%d.%02d cfg: %02X\n",
			ee_info->eboot, ee_info->vid, ee_info->pid,
			ee_info->didH, ee_info->didL, ee_info->ecfg);
		/* try to find eeprom-size:  1,2,4,8,16 .. 128 kB */
		for ( i = 1024; i <= (1024*128); i <<= 1 ) {
			rc = ezusb_ctrl_read( wintvci,
					VR_A2_EEPROM, i, eecmp, sizeof eebuf);
			if (rc != sizeof eebuf )
				break;
			//pr_info(" * ******* %*ph\n", 32, eecmp);
			if ( !strncmp( eebuf, eecmp, sizeof eebuf) ) {
				pr_info(" * EEPROM: Size %d kB ( %d [0x%X] bytes )\n",
						i >> 10, i, i);
				break;
			}
		}
	}
#define EZ_REVID 0xE60A
	rc = ezusb_ctrl_read( wintvci,
				VR_A3_EXTRAM, EZ_REVID, &regval, 1);
	if (rc == 1) {
		pr_info(" * REVID [0x%04X]: Silicon Revision 0x%02X\n", EZ_REVID, regval);
	}
#define EZ_REVCTL 0xE60B
	rc = ezusb_ctrl_read( wintvci,
				VR_A3_EXTRAM, EZ_REVCTL, &regval, 1);
	if (rc == 1) {
		pr_info(" * REVCTL [0x%04X]: 0x%02X\n", EZ_REVCTL, regval);
	}
#define EZ_IFCONFIG 0xE601
	rc = ezusb_ctrl_read( wintvci,
				VR_A3_EXTRAM, EZ_IFCONFIG, &regval, 1);
	if (rc == 1) {
		pr_info(" * IFCONFIG [0x%04X]=0x%02X : FIFO/GPIF Clock %d Mhz (%s)\n",
			EZ_IFCONFIG, regval,
			((regval & 0x40) ? 48 : 30),
			((regval & 0x80) ? "internal": "external"));
	}
#define EZ_USBCS 0xE680
	rc = ezusb_ctrl_read( wintvci,
				VR_A3_EXTRAM, EZ_USBCS, &regval, 1);
	if (rc == 1) {
		pr_info(" * USBCS [0x%04X]=0x%02X\n", EZ_USBCS, regval);
	}

	rc = ezusb_ctrl_read( wintvci,
				VR_A3_EXTRAM, wintvci->info->fx->cpucs_reg, &regval, 1);
	if (rc == 1) {
		pr_info(" * CPUCS [0x%04X]=0x%02X : CPU clock-speed %d Mhz (12|24|48)\n",
			wintvci->info->fx->cpucs_reg, regval,
			(12 << ((regval>>3) & 0x3)) );
	}
}

static void wintv_usb_ci_show_sw_info( struct wintv_ci_dev *wintvci )
{
	struct msg_reply reply;
	unsigned char *ver = reply.buffer;

	int rc = CI_90_GetVersion(wintvci, &reply);
	if (!rc) {
	    ver[5] = 0; /* terminate fw-version string */
	    pr_info(" * FW_Version(%s) FPGA_Version(%c.%c)\n", ver+1, ver[6], ver[7]);
	}
}

/*
 *  U S B - E N D P O I N T S
 */

#define CI_USB_INTERFACE  0
#define CI_USB_ALTSETTING 1
#define CI_USB_CONFIGURATION 1

static char *name_ep_type[] = { "CTRL","ISOC","BULK","INTR" };

static int wintv_usb_ci_setup_endpoints(struct wintv_ci_dev *wintvci)
{
	struct usb_device *udev = wintvci->udev;
	struct usb_host_interface *host_intf;
	int i, rc;

	rc = usb_set_interface(udev, CI_USB_INTERFACE, CI_USB_ALTSETTING);
	if (rc < 0) {
		pr_err(" *ERR* unable to set altsetting[%d] of interface[%d]\n",
				CI_USB_ALTSETTING, CI_USB_INTERFACE);
		return rc;
	}

	rc = usb_ctrl_req_set_config(wintvci, CI_USB_CONFIGURATION);
	if (rc < 0) {
		pr_err(" *ERR* unable to set configuration[%d]\n", CI_USB_CONFIGURATION);
		return rc;
	}
	pr_info(" * current configuration:%d\n", udev->actconfig->desc.bConfigurationValue);

	host_intf = wintvci->intf->cur_altsetting;
	if (host_intf->desc.bNumEndpoints != 4) {
		pr_err(" *ERR* expect exact 4 endpoints in altsetting[%d]\n",
				CI_USB_ALTSETTING);
		return -EINVAL;
	}

	for (i = 0; i < host_intf->desc.bNumEndpoints; i++) {

		struct usb_endpoint_descriptor *epd = &host_intf->endpoint[i].desc;

		unsigned char type	= (epd->bmAttributes & USB_ENDPOINT_XFERTYPE_MASK);
		unsigned char addr	= epd->bEndpointAddress;
		unsigned char dir	= (epd->bEndpointAddress & USB_ENDPOINT_DIR_MASK);

		struct ep_info *epi	= NULL;
		int pipe		= 0;

		if ((type == USB_ENDPOINT_XFER_INT) && (dir == USB_DIR_IN)) {
			epi	= &wintvci->ca_dev.ep_intr_in;
			pipe	= usb_rcvintpipe(udev, addr);
		}
		else if ((type == USB_ENDPOINT_XFER_BULK) && (dir == USB_DIR_OUT)) {
			epi	= &wintvci->ca_dev.ep_bulk_out;
			pipe	= usb_sndbulkpipe(udev, addr);
		}
		else if ((type == USB_ENDPOINT_XFER_ISOC) && (dir == USB_DIR_IN)) {
			epi	= &wintvci->ci_dev.ep_isoc_in;
			pipe	= usb_rcvisocpipe(udev, addr);
		}
		else if ((type == USB_ENDPOINT_XFER_ISOC) && (dir == USB_DIR_OUT)) {
			epi	= &wintvci->ci_dev.ep_isoc_out;
			pipe	= usb_sndisocpipe(udev, addr);
		}
		else
			continue;

		epi->wintvci		= wintvci;
		epi->type		= type;
		epi->dir		= dir;
		epi->addr		= addr;
		epi->pipe		= pipe;
		/*
		 * max_packet_size for 1 micro-frame(0.125ms) - 8 uframea -> 1. frame (1ms)
		 * Hw.Rev.1 : 5*188=940 = 7520 bytes per frame (1ms) ==> max.~60 Mbit/s
		 * Hw.Rev.2 : 4*188=752 = 6016 bytes per frame (1ms) ==> max.~48 Mbit/s
		 *
		 * no MAXP_MULT in kernels <= 4.8 ?
		 */
		epi->maxp		= __le16_to_cpu(epd->wMaxPacketSize) & 0x7FF;
		epi->maxp		*= ((__le16_to_cpu(epd->wMaxPacketSize) >> 11) & 0x3) + 1;
		epi->binterval		= epd->bInterval;

		pr_info("  EP %d %-3s (%-4s) Addr : 0x%02X, Maxp %4d Ival %d\n",
			addr & USB_ENDPOINT_NUMBER_MASK,
			(dir == USB_DIR_IN) ? "IN" : "OUT",
			name_ep_type[type],
			addr, epi->maxp, epi->binterval);
	}

	if (	!wintvci->ca_dev.ep_intr_in.addr  ||
		!wintvci->ca_dev.ep_bulk_out.addr ||
		!wintvci->ci_dev.ep_isoc_in.addr   ||
		!wintvci->ci_dev.ep_isoc_out.addr ) {

		pr_info(" *ERR* not all expected endpoints found\n");
		return -EINVAL;
	}
	if (	(wintvci->ca_dev.ep_intr_in.maxp < CA_CTRL_MAXPKT) ||
		(wintvci->ca_dev.ep_bulk_out.maxp < CA_CTRL_MAXPKT) ) {
		pr_info(" *ERR* invalid packet-size in control-interface(s)\n");
		return -EINVAL;
	}
	/* clear all endpoints */
	usb_clear_halt(udev, wintvci->ca_dev.ep_intr_in.pipe);
	usb_clear_halt(udev, wintvci->ca_dev.ep_bulk_out.pipe);
	usb_clear_halt(udev, wintvci->ci_dev.ep_isoc_in.pipe);
	usb_clear_halt(udev, wintvci->ci_dev.ep_isoc_out.pipe);

	return 0;
}

/*
 *  D V B - A D A P T E R
 */

static int wintv_usb_ci_adapter_attach(struct wintv_ci_dev *wintvci) {

	int rc;
	pr_info("Registering DVB Adapter\n");

	wintvci->slot.cis_valid = 0;
	wintvci->slot.usbci_state = 0;
	wintvci->slot.cam_state = 0;
	init_waitqueue_head(&wintvci->slot.cam_wq);
	mutex_init(&wintvci->usb_mutex);

	rc = dvb_register_adapter(&wintvci->adapter, "WinTV-CI", THIS_MODULE,
				  &wintvci->udev->dev, adapter_nr);
	return rc;
}

/*
 *  D R I V E R - I N I T
 */

static int wintv_usb_ci_probe(struct usb_interface *intf,
				const struct usb_device_id *id)
{
	struct wintv_ci_dev *wintvci;
	char fw_name[32];
	int rc = 0;

	struct usb_device *udev = usb_get_dev(interface_to_usbdev(intf));
	struct usb_id_info *info = (struct usb_id_info *) id->driver_info;
	u16 hw_version = le16_to_cpu(udev->descriptor.bcdDevice);
	/****/
	wintvci = ci_kmalloc(sizeof(*wintvci), 1, (char *)__func__);
	if (!wintvci)
		goto error1;

	wintvci->udev     = udev;
	wintvci->intf     = intf;
	wintvci->info     = info;
	wintvci->fw_state = (hw_version) ? FW_STATE_COLD : FW_STATE_WARM;
	/****/

	pr_info("Found USB-CI device %04x:%04x (Ver.%d) in %s state\n",
		le16_to_cpu(udev->descriptor.idVendor),
		le16_to_cpu(udev->descriptor.idProduct),
		le16_to_cpu(udev->descriptor.bcdDevice),
		(hw_version) ? "cold" : "warm");

	if (udev->product)
		pr_info("%20s : %s\n", "Product", udev->product);
	if (udev->manufacturer)
		pr_info("%20s : %s\n", "Manufacturer", udev->manufacturer);
	if (udev->serial)
		pr_info("%20s : %s\n", "SerialNumber", udev->serial);

	/* ------ COLD ------ */
	if (wintvci->fw_state == FW_STATE_COLD) {
		int fw_ver = hw_version;

		if (hw_version > info->max_ver_hw) {
			pr_err("Unexpected hardware version %d\n", hw_version);
			goto error;
		}

		if (fw_ver > info->max_ver_fw) /* for Cinergy CI USB */
			fw_ver = info->max_ver_fw; /* same FW for versions 3 and 4 */
		/*
		 * First load EZUSB firmware with support of 0xA3 requests
		 * and show some hardware-info. No automatic USB-renumbering !
		 */
		rc = wintv_usb_ci_load_firmware(wintvci,(char *)info->fw_cb_name);
		if (rc)
			goto error;
		wintvci->fw_state = FW_STATE_EZUSB;
		/*---*/
		wintv_usb_ci_show_hw_info(wintvci);

		/*
		 * Now load the matching CI-firmware - automatic USB renumbering !
		 */
		snprintf(fw_name, sizeof(fw_name),info->fw_ci_name, fw_ver);
		pr_info("CI-firmware %s selected\n", fw_name);

		rc = wintv_usb_ci_load_firmware(wintvci, fw_name);
		if (rc)
			goto error;
		/* Now the USB-device disconnects and re-appears in warm state */
	}

	kref_init(&wintvci->kref);

	/* save our data pointer in this interface device */
	usb_set_intfdata(intf, wintvci);

	/* ------ WARM ------ */
	if (wintvci->fw_state == FW_STATE_WARM ) {
		rc = wintv_usb_ci_setup_endpoints(wintvci);
		if (rc)
			goto error;

		rc = wintv_usb_ci_adapter_attach(wintvci);
		if (rc < 0) /* adapter number */
			goto error;

		rc = ca_attach(wintvci);
		if (rc)
			goto error;

		rc = ci_attach(wintvci);
		if (rc)
			goto error;

		wintv_usb_ci_show_sw_info(wintvci);
		cam_state_set(wintvci, USBCI_STATE_NON);
	}

	pr_err("probe succesfull\n");
	return 0;
error:
	usb_put_dev(wintvci->udev);
error1:
	pr_err("***** probe failed(%d) *****\n",rc);
	return -ENODEV;
}

static void wintv_usb_ci_delete(struct kref *kref)
{
	struct wintv_ci_dev *wintvci = container_of(kref,
						struct wintv_ci_dev, kref);
	pr_err("%s\n", __func__);

	usb_put_dev(wintvci->udev);

	kfree(wintvci);
}

static void wintv_usb_ci_disconnect(struct usb_interface *intf) {

	struct wintv_ci_dev *wintvci;
	int minor = intf->minor;

	pr_err("%s\n",__func__);

	wintvci = usb_get_intfdata(intf);

	if (wintvci->fw_state == FW_STATE_WARM) {
		ci_detach( wintvci );
		ca_detach( wintvci );
		dvb_unregister_adapter( &wintvci->adapter );
	}

	/* decrement our usage count */
	kref_put(&wintvci->kref, wintv_usb_ci_delete);

	usb_set_intfdata(intf, NULL);
	pr_info("USB wintv-ci #%d now disconnected\n", minor);
}

static struct usb_device_id wintv_usb_ci_table [] = {
	/* SmarDTV / Mascom - SmarCAM Adapter */
	{ USB_DEVICE( 0x2040, 0x1100 ), /* Hauppauge WintTV-CI USB */
	    .driver_info = (unsigned long) &wintv_ci_info
	},
	{ USB_DEVICE( 0x1B0D, 0x5F10 ), /* SmarDTV */
	    .driver_info = (unsigned long) &usb2_ci_info
	},
	{ USB_DEVICE( 0x1B0D, 0x5F20 ), /* SmarDTV */
	    .driver_info = (unsigned long) &usb2_ci_info
	},
	/* SmarDTV - USBCAM-T */
	{ USB_DEVICE( 0x0CCD, 0x0074 ), /* TerraTec Cinergy CI USB */
	    .driver_info = (unsigned long) &usb2_ci_info
	},
	{ USB_DEVICE( 0x1B0D, 0x5F0F ), /* SmarDTV / Terratec */
	    .driver_info = (unsigned long) &usb2_ci_info
	},
	{ USB_DEVICE( 0x1B0D, 0xDF01 ), /* SmarDTV */
	    .driver_info = (unsigned long) &usb2_ci_info
	},
	{}
};
MODULE_DEVICE_TABLE (usb, wintv_usb_ci_table);

MODULE_AUTHOR("Helmut Binder");
MODULE_DESCRIPTION("Hauppauge WinTV-CI USB2 Common Interface driver");
MODULE_VERSION("0.3.0");
MODULE_LICENSE("GPL");

static struct usb_driver wintv_usb_ci_driver = {
	.name       = "wintv_usb2ci",
	.id_table   = wintv_usb_ci_table,
	.probe      = wintv_usb_ci_probe,
	.disconnect = wintv_usb_ci_disconnect,
};

module_usb_driver(wintv_usb_ci_driver);
