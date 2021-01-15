/*
 *
 * Includes for ch9344.c
 *
 */

/*
 * Baud rate and default timeout
 */
#define DEFAULT_BAUD_RATE 9600
#define DEFAULT_TIMEOUT   2000

/*
 * CMSPAR, some architectures can't have space and mark parity.
 */

#ifndef CMSPAR
#define CMSPAR			0
#endif

/*
 * Major and minor numbers.
 */

#define CH9344_TTY_MAJOR		168
#define CH9344_TTY_MINORS		256

/*
 * Output control lines.
 */

#define CH9344_CTRL_DTR		0x01
#define CH9344_CTRL_RTS		0x02

/*
 * Input control lines and line errors.
 */

#define CH9344_CTRL_CTS		0x01
#define CH9344_CTRL_DSR		0x02
#define CH9344_CTRL_RI		0x04
#define CH9344_CTRL_DCD		0x08

#define CH9344_CTRL_OVERRUN	0x10
#define CH9344_CTRL_PARITY	0x20
#define CH9344_CTRL_FRAMING	0x40
#define CH9344_CTRL_BRK		0x80

//Vendor define
#define VENDOR_WRITE_REG_TYPE	0xC0
#define VENDOR_WRITE_BIT_TYPE	0x80
#define VENDOR_SET_TYPE		    0x20

#define VENDOR_WRITE_BIT_EXT_TYPE 0x90
#define VENDOR_READ_BIT_EXT_TYPE 0xC0

#define PORT_OFFSET 0x04

#define MODE_NORMAL   0x00
#define MODE_RS485    0x01
#define MODE_GPIO     0x02
#define MODE_HARDFLOW 0x03

#define GPIO_DIR_OUT  0x01
#define GPIO_DIR_IN	  0x00

#define REG_UART_MODE 0x97
#define REG_GPIO_DIR  0x98
#define REG_GPIO_OUT  0x99
#define REG_GPIO_IN   0x9b

//Reg define
#define REG_IER 0x01
#define REG_IIR 0x02
#define REG_LCR 0x03
#define REG_MCR 0x04
#define REG_MSR 0x06

//Reg status
#define REG_IIR_RLS 0x06
#define REG_IIR_THR 0x02
#define REG_IIR_MS	0x00

/*
 * Internal driver structures.
 */

/*
 * The only reason to have several buffers is to accommodate assumptions
 * in line disciplines. They ask for empty space amount, receive our URB size,
 * and proceed to issue several 1-character writes, assuming they will fit.
 * The very first write takes a complete URB. Fortunately, this only happens
 * when processing onlcr, so we only need 2 buffers. These values must be
 * powers of 2.
 */
#define CH9344_NW  16
#define CH9344_NR  16

#define MAXDEV 4
#define MAXPORT 4
#define MAXGPIO 12
#define NUMSTEP 4

#define GPIO_HIGH 1
#define GPIO_LOW 0

struct ch9344_wb {
	unsigned char *buf;
	dma_addr_t dmah;
	int len;
	int use;
	struct urb		*urb;
	struct ch9344		*instance;
};

struct ch9344_rb {
	int			size;
	unsigned char		*base;
	dma_addr_t		dma;
	int			index;
	struct ch9344		*instance;
};

struct ch9344_ttyport {
	struct tty_port port;
	int portnum;
	void *portdata;
};

struct usb_ch9344_line_coding {
	__le32	dwDTERate;
	__u8	bCharFormat;
#define USB_CH9344_1_STOP_BITS			0
#define USB_CH9344_1_5_STOP_BITS			1
#define USB_CH9344_2_STOP_BITS			2

	__u8	bParityType;
#define USB_CH9344_NO_PARITY			0
#define USB_CH9344_ODD_PARITY			1
#define USB_CH9344_EVEN_PARITY			2
#define USB_CH9344_MARK_PARITY			3
#define USB_CH9344_SPACE_PARITY			4

	__u8	bDataBits;
} __attribute__ ((packed));

struct ch9344 {
	struct usb_device *dev;				/* the corresponding usb device */
	struct usb_interface *control;			/* control interface */
	struct usb_interface *data;			/* data interface */
	unsigned int num_ports;
	bool modeline9;
	struct ch9344_ttyport ttyport[MAXPORT];			 	/* our tty port data */
	
	struct urb *cmdreadurb;				/* urbs */
	u8 *cmdread_buffer;				/* buffers of urbs */
	dma_addr_t cmdread_dma;				/* dma handles of buffers */
	
	struct ch9344_wb wb[CH9344_NW];
	unsigned long read_urbs_free;
	struct urb *read_urbs[CH9344_NR];
	struct ch9344_rb read_buffers[CH9344_NR];
	int rx_buflimit;
	int rx_endpoint;
    int tx_endpoint;
    int cmdtx_endpoint;
    int cmdrx_endpoint;
    int ctrl_endpoint;
	
	spinlock_t read_lock;
	int write_used;					/* number of non-empty write buffers */
	int transmitting;
	bool write_empty[MAXPORT];
	spinlock_t write_lock;
	struct mutex mutex;
	bool disconnected;
	struct usb_ch9344_line_coding line[CH9344_NR];		/* bits, stop, parity */
	struct work_struct work;			/* work queue entry for line discipline waking up */
	unsigned int ctrlin[CH9344_NR];				/* input control lines (DCD, DSR, RI, break, overruns) */
	unsigned int ctrlout[CH9344_NR];				/* output control lines (DTR, RTS) */
	struct async_icount iocount[CH9344_NR];			/* counters for control line changes */
	struct async_icount oldcount[CH9344_NR];			/* for comparison of counter */
	u8 uartmode[CH9344_NR];				/* uart mode */
	u8 gpiodir[MAXGPIO];				/* gpio direction */
	u8 gpioval[MAXGPIO];				/* gpio output value */
	u16 gpiovalin;						/* gpio input value */
	bool gpio_recv;						/* gpio input sync flag */
	wait_queue_head_t wioctl;			/* for write */
	wait_queue_head_t wmodemioctl;		/* for ioctl */
	wait_queue_head_t wgpioioctl;		/* for gpio input ioctl */
	struct mutex gpiomutex;
	unsigned int writesize;				/* max packet size for the output bulk endpoint */
	unsigned int readsize, cmdsize;			/* buffer sizes for freeing */
	unsigned int ctrlsize;
	unsigned int minor;				    /* ch9344 minor number */
	unsigned char clocal[CH9344_NR];				/* termios CLOCAL */
	unsigned int susp_count;			/* number of suspended interfaces */
	u8 bInterval;
	struct usb_anchor delayed;			/* writes queued for a device about to be woken */
	unsigned long quirks;
};


/* constants describing various quirks and errors */
#define SINGLE_RX_URB			BIT(1)
#define NO_DATA_INTERFACE		BIT(4)
#define IGNORE_DEVICE			BIT(5)
#define QUIRK_CONTROL_LINE_STATE	BIT(6)
#define CLEAR_HALT_CONDITIONS		BIT(7)


