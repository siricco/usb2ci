/*
 * wintv-ci.h : WinTV-CI - USB2 Common Interface driver
 *
 * Copyright (C) 2017 Helmut Binder (cco@aon.at)
 #
 * (+HB+) 2017-08-13
 * (+HB+) 2019-11-11
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

#define pr_fmt(fmt) KBUILD_MODNAME ": " fmt

#include <linux/types.h>
#include <linux/slab.h>
#include <linux/usb.h>
#include <linux/kthread.h>

#include <linux/version.h>
#if LINUX_VERSION_CODE < KERNEL_VERSION(4,16,0)
#include <dvbdev.h>
#include "dvb_ringbuffer.h"
#else
#include <media/dvbdev.h>
#include "media/dvb_ringbuffer.h"
#endif

#include <linux/dvb/ca.h> /* CI_CA_LINK,... */

/*
 * --- C O R E  ---
 */

#define STATUSBIT_FR 0x40
#define STATUSBIT_DA 0x80

struct ezusb_fx_type {
	/* EZ-USB Control and Status Register.  Bit 0 controls 8051 reset */
	unsigned short cpucs_reg;
	unsigned short internal_ram_size;
};

struct usb_id_info {
	bool is_wintvci;
	const char *fw_ci_name;
	const char *fw_cb_name;
	int max_ver_hw;
	int max_ver_fw;
	const struct ezusb_fx_type *fx;
};

/* USB endpoints */

#define USB_EP0_ADDR		0x0
#define USB_EP0_SIZE		0x40

/* Control-interface in/out max. packet-size*/
#define CA_CTRL_MAXPKT		0x40
#define CA_CTRL_MAXPKT_DATA	(CA_CTRL_MAXPKT - 4) /* -4 byte cmd-header */
/* Control interface max. message-size */
#define CA_CTRL_MAXMSG		0x100
/* Control interface max. TPDU-size */
#define CA_CTRL_MAXTPDU		4096

/* Link-Layer max. message-size */
#define CA_LINK_LAYER_SIZE	CA_CTRL_MAXMSG
#define CA_LINK_LAYER_DATA	(CA_LINK_LAYER_SIZE - 2)

struct msg_reply {
	unsigned char	buffer[CA_CTRL_MAXMSG];
	int		size;
};

struct slot_info {
	int			usbci_state;

	int			cis_valid;
	u32			config_base;
	u8			config_option;
	int			link_layer_size;

	int			cam_state;
	wait_queue_head_t	cam_wq;
};

struct urb_transfer {
	struct urb 		*urb;
	unsigned char		*xfer_buffer;
	dma_addr_t		dma_addr;
};

struct isoc_info {
	int			uframe_size;	/* maxp */
	int			num_uframes;	/* 1..255 */
	int			transfer_size;
	int			min_chunk_size;
	int			min_submit_size;

	int			num_urbs;	/* all urbs */
	struct urb_transfer	*urbs;

	int			num_transfers;	/* 1.. */
	struct urb_transfer	*transfers;	/* regular TS transfers */

	int			num_spares;	/* 1.. */
	struct urb_transfer	*spares;	/* spare transfers */
};

struct ca_cmd_sndhd {
	unsigned char cmd;
	unsigned char slot;
	unsigned char xFF;
	unsigned char len;
	unsigned char data;
} __attribute__ ((packed));

struct ca_cmd_rcvhd {
	unsigned char state;
	unsigned char flag;
	unsigned char reply;
	unsigned char len;
	unsigned char data;
} __attribute__ ((packed));

struct bulk_info { /* SEND */
	union {
	    unsigned char	*buffer;	/* size = maxp */
	    struct ca_cmd_sndhd *hdr;
	} pkt;
};

struct intr_info { /* RECEIVE */
	union {
	    unsigned char	*buffer;	/* size = maxp */
	    struct ca_cmd_rcvhd *hdr;
	} pkt;
	struct {
	    unsigned char	*buffer;	/* size = CA_MAX_MSG_SIZE */
	    int			size;
	} msg;
};

struct ep_ringbuffer {
	struct dvb_ringbuffer	buffer;
	int			num_items;
	wait_queue_head_t	wq;
};

struct ep_info {
	struct wintv_ci_dev *wintvci;

	unsigned char	type;
	unsigned char	dir;
	unsigned char	addr;
	unsigned int	pipe;
	int		maxp;
	int		binterval;

	union {
		struct isoc_info isoc;
		struct bulk_info bulk;
		struct intr_info intr;
	} u;

	/* ringbuffer */
	struct ep_ringbuffer	erb;
};

struct task {
	/* PID of the monitoring thread */
	struct task_struct	*thread;
	/* Flag indicating the thread should wake up now */
	unsigned int		wakeup:1;
	/* Delay the main thread should use */
	unsigned long		delay;
	/* running or stopped */
	int			running;
};

struct ca_device {
	struct wintv_ci_dev	*wintvci;

	struct dvb_device 	*regdev_ca;
	struct mutex		ca_mutex;
	struct mutex		ca_ioctl_mutex;

	struct ep_info		ep_intr_in;  /* interrupt */
	struct ep_info		ep_bulk_out; /* bulk */

	struct task		ca_task;

	int			ca_cam_state;
};

struct ci_device {
	struct wintv_ci_dev	*wintvci;

	struct dvb_device	*regdev_ci;
	struct mutex		ci_mutex;
	spinlock_t		ci_lock;
	unsigned long		ci_lock_flags;

	struct ep_info		ep_isoc_in;	/* isochronous */
	struct ep_info		ep_isoc_out;	/* isochronous */

	int			isoc_enabled;		/* streaming 0/1 */
	int			isoc_tsnull_pending;	/* #of unread ts-null uframes */
	int			isoc_is_sync;
	int			isoc_urbs_running;	/* #of urbs running */
	wait_queue_head_t	isoc_urbs_wq;		/* urb wait-queue */

	/* packets write/read observing */
	int		cam_subs;
	int		cam_uframes;
	int		cam_syncs;
	int		ts_count;
	unsigned long	ts_count_timeout;
	int		isoc_TS_CAM;
	int		isoc_TS_CAM_total;
};

#define FW_STATE_COLD  1 /* no firmware loaded */
#define FW_STATE_EZUSB 2 /* EZUSB basic code-bulker firmware loaded */
#define FW_STATE_WARM  3 /* CI-firmware loaded */

struct wintv_ci_dev {
	struct usb_device		*udev; /* the usb device for this device */
	struct usb_interface		*intf; /* the interface for this device */

	const struct usb_id_info	*info;

	int fw_state;

	struct ca_device		ca_dev; /* TPDU exchange (EN 50221)  */
	struct ci_device		ci_dev; /* TS-streaming */

	struct slot_info	slot;

	u8 ep0_buffer[USB_EP0_SIZE];

	/* logs */
	u8			last_status;
	int			last_status_cnt;
	unsigned long		last_exchange;

	/* DVB */
	struct dvb_adapter	adapter;
	struct mutex		usb_mutex;

	struct kref		kref;
};

int CI_50_GetStatus(struct wintv_ci_dev *wintvci, unsigned char *status);
int CI_20_WriteLPDU(struct wintv_ci_dev *wintvci, char *data, int len);
int CI_80_ReadLPDU (struct wintv_ci_dev *wintvci, struct msg_reply *reply);

void *ci_kmalloc(int size, int zero, char * caller);

/* internal CAM status */
enum {
	USBCI_STATE_NON = 1,
	USBCI_STATE_CAM,
	USBCI_STATE_RST,
	USBCI_STATE_CIS,
	USBCI_STATE_COR,
	USBCI_STATE_LNK,
	USBCI_STATE_RDY
};

int cam_state_monitor(struct wintv_ci_dev *wintvci);
int cam_state_set(struct wintv_ci_dev *wintvci, int usbci_state);

/*
 * --- P C M C I A - C I S ---
 */

int parse_cis(unsigned char *cis, int size, struct slot_info *info);

/*
 * --- E N 5 0 2 2 1 ---
 */

void dump_io_tpdu( u8 *buf, size_t count, const char *func, int dir_in);

/*
 * --- C A - D E V I C E  ( T P D U )
 */

int  ca_attach(struct wintv_ci_dev *wintvci);
void ca_detach(struct wintv_ci_dev *wintvci);

/*
 * --- C I / S E C - D E V I C E ---
 */

void ci_reset(struct wintv_ci_dev *wintvci);
int  ci_attach(struct wintv_ci_dev *wintvci);
void ci_detach(struct wintv_ci_dev *wintvci);

int ci_CAM_sync(struct ci_device *ci_dev, bool wait_sync_done);
/***/
