/* SPDX-License-Identifier: GPL-2.0 */
/*
 *
 * Includes for ch9344.c
 *
 */

#ifndef _CH9344_H
#define _CH9344_H

/*
 * Baud rate and default timeout
 */
#define DEFAULT_BAUD_RATE 9600
#define DEFAULT_TIMEOUT 5000
#define DEFAULT_CLOSETIMEOUT 100

/*
 * CMSPAR, some architectures can't have space and mark parity.
 */
#ifndef CMSPAR
#define CMSPAR 0
#endif

/*
 * Major and minor numbers.
 */
#define CH9344_TTY_MAJOR 168
#define CH9344_TTY_MINORS 256

#define USB_MINOR_BASE 192

#define CH9344_CTO_D 0x01
#define CH9344_CTO_R 0x02
#define CH9344_CTI_C 0x01
#define CH9344_CTI_DS 0x02
#define CH9344_CTI_R 0x04
#define CH9344_CTI_DC 0x08

#define CH9344_LO 0x10
#define CH9344_LP 0x20
#define CH9344_LF 0x40
#define CH9344_LB 0x80

#define CMD_W_R 0xC0
#define CMD_W_BR 0x80
#define CMD_S_T 0x20

#define CMD_WB_E 0x90
#define CMD_RB_E 0xC0

#define VEN_R 0x85
#define VEN_W 0x8A

#define M_NOR 0x00
#define M_RS 0x01
#define M_IO 0x02
#define M_HF 0x03

#define G_DO 0x01
#define G_DI 0x00

#define R_MOD 0x97
#define R_IO_D 0x98
#define R_IO_O 0x99
#define R_IO_I 0x9B
#define R_TM_O 0x9C
#define R_UP_O 0x9D
#define R_INIT 0xA1

#define R_IO_CE 0xA3
#define R_IO_CD 0xA4
#define R_IO_CO 0xA5
#define R_IO_COS 0xA6
#define R_IO_CI 0xA7
#define R_IO_RE 0xAA
#define R_IO_RD 0xAB

#define R_C1 0x01
#define R_C2 0x02
#define R_C3 0x03
#define R_C4 0x04
#define R_C5 0x06

#define R_II_B1 0x06
#define R_II_B2 0x02
#define R_II_B3 0x00

#define R_EE_CFG 0xA9

#define CMD_VER 0x96
#define FIFOSIZE 4096

#define CH9344_NW 4
#define CH9344_NR 4

#define NUMSTEP 8
#define MAXGPIO 48
#define MAXPORT 8
#define CFGLEN 256
#define IO_H 1
#define IO_L 0

#define IOID 0x57136824

/*
 * Internal driver structures.
 */
struct ch9344_wb {
	unsigned char *buf;
	dma_addr_t dmah;
	int len;
	int use;
	struct urb *urb;
	struct ch9344 *instance;
	int portnum;
};

struct ch9344_rb {
	int size;
	unsigned char *base;
	dma_addr_t dma;
	int index;
	struct ch9344 *instance;
};

struct usb_ch9344_line_coding {
	__le32 dwDTERate;
	__u8 bCharFormat;
#define SB1 0
#define SB1_5 1
#define SB2 2

	__u8 bParityType;
#define PAN 0
#define PAO 1
#define PAE 2
#define PAM 3
#define PAS 4

	__u8 bDataBits;
} __packed;

struct ch9344_ttyport {
	struct tty_port port;
	int portnum;
	void *portdata;
	bool write_empty;
	struct usb_ch9344_line_coding line; /* baudrate, data format */
	unsigned int ctrlin; /* input lines (CTS, DSR, DCD, RI) */
	unsigned int ctrlout; /* output control lines (DTR, RTS) */
	struct async_icount iocount; /* counters for control line changes */
	struct async_icount oldcount; /* for comparison of counter */
	u8 uartmode; /* uart mode */
	unsigned char clocal; /* termios CLOCAL */
	wait_queue_head_t wioctl; /* for write */
	wait_queue_head_t wmodemioctl; /* for ioctl */
	bool isopen;
	struct work_struct work; /* used for line discipline waking up */
	struct timer_list timer;
	struct kfifo rfifo;
	int rlen;
	int rtimes;
	int interval;
};

enum CHIPTYPE {
	CHIP_CH9344L = 0,
	CHIP_CH9344Q,
	CHIP_CH348L,
	CHIP_CH348Q,
};

struct ch9344 {
	struct usb_device *dev; /* the corresponding usb device */
	struct usb_interface *control; /* control interface */
	struct usb_interface *data; /* data interface */
	unsigned int num_ports;
	bool modeline9;
	struct ch9344_ttyport ttyport[MAXPORT]; /* our tty port data */
	enum CHIPTYPE chiptype;
	u8 chipver;
	int port_offset;

	__le16 idVendor;
	__le16 idProduct;
	__le16 bcdDevice;

	struct urb *cmdreadurb; /* urbs */
	u8 *cmdread_buffer; /* buffers of urbs */
	dma_addr_t cmdread_dma; /* dma handles of buffers */

	int opencounts;
	struct ch9344_wb wb[MAXPORT][CH9344_NW];
	unsigned long read_urbs_free;
	struct urb *read_urbs[CH9344_NR];
	struct ch9344_rb read_buffers[CH9344_NR];
	struct ch9344_wb *putbuffer; /* for ch9344_tty_put_char() */
	int rx_buflimit;
	int rx_endpoint;
	int tx_endpoint;
	int cmdtx_endpoint;
	int cmdrx_endpoint;
	int ctrl_endpoint;

	spinlock_t read_lock;
	int write_used; /* number of non-empty write buffers */
	int transmitting;
	spinlock_t write_lock;
	struct mutex mutex;
	bool disconnected;
	u64 gpioenables;
	u64 gpiodirs;
	u64 gpiovals;
	u64 gpiovalins;
	u8 gpiodir[MAXGPIO]; /* gpio direction */
	u8 gpioval[MAXGPIO]; /* gpio output value */
	u16 gpiovalin; /* gpio input value */
	bool gpio_recv; /* gpio input sync flag */
	wait_queue_head_t wgpioioctl; /* for gpio input ioctl */
	struct mutex gpiomutex;
	u8 cfgval[256];
	bool cfg_recv;
	u16 cfgindex;
	wait_queue_head_t wcfgioctl; /* for config ioctl */
	unsigned int writesize; /* max packet size */
	unsigned int readsize, cmdsize; /* buffer sizes for freeing */
	unsigned int ctrlsize;
	unsigned int minor; /* ch9344 minor number */
	unsigned int susp_count; /* number of suspended interfaces */
	u8 bInterval;
	struct usb_anchor delayed; /* used for a device about to be woken */
	unsigned long quirks;
	struct kref kref;
	u32 io_id;
};

/* constants describing various quirks and errors */
#define SINGLE_RX_URB BIT(1)
#define NO_DATA_INTERFACE BIT(4)
#define IGNORE_DEVICE BIT(5)
#define QUIRK_CONTROL_LINE_STATE BIT(6)
#define CLEAR_HALT_CONDITIONS BIT(7)

#endif
