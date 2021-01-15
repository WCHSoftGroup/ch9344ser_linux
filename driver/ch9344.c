/*
 * ch9344.c - V1.16
 *
 * Copyright (c) 2020 SoldierJazz <zhangj@wch.cn>
 *
 * USB driver for USB to serial chip ch9344
 *
 * Sponsored by SuSE
 *
 * System required:
 * Kernel version beyond 3.4.x
 * Update Log:
 * V1.0 - initial version
 * V1.01 - added supports for big-endian CPUs
 * V1.10 - fix bugs that only one serial port can communicate on some systems
 * V1.11 - added supports for high baudrates 2M, 4M and 6M
 * V1.12 - added supports for modem signal, flow control
 * V1.13 - added long packet for tty_write
 * V1.14 - added ioctl for rs485 mode
 * V1.15 - fixed baud rate 0 bugs
 * V1.16 - fixed modem out bugs, added gpio function,
 *		 - removed rs485 control(hardware auto supports)
 *		 - added supports for baudrates 250k, 500k, 1M, 1.5M, 3M, 12M
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA 02111-1307 USA
 */

#define DEBUG
#define VERBOSE_DEBUG

#undef DEBUG
#undef VERBOSE_DEBUG

#include <linux/kernel.h>
#include <linux/errno.h>
#include <linux/init.h>
#include <linux/slab.h>
#include <linux/tty.h>
#include <linux/serial.h>
#include <linux/tty_driver.h>
#include <linux/tty_flip.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/uaccess.h>
#include <linux/usb.h>
#include <linux/usb/cdc.h>
#include <asm/byteorder.h>
#include <asm/unaligned.h>
#include <linux/idr.h>
#include <linux/list.h>
#include <linux/version.h>

#if (LINUX_VERSION_CODE >= KERNEL_VERSION(4, 11, 0))
#include <linux/sched/signal.h>
#endif

#include "ch9344.h"

#define DRIVER_AUTHOR "SoldierJazz"
#define DRIVER_DESC "USB driver for USB to serial chip ch9344."
#define VERSION_DESC "V1.16 On 2020.12.05"

#define IOCTL_MAGIC 'W'
#define IOCTL_CMD_GPIOENABLE 	_IOW(IOCTL_MAGIC, 0x80, u16)
#define IOCTL_CMD_GPIODIR 		_IOW(IOCTL_MAGIC, 0x81, u16)
#define IOCTL_CMD_GPIOSET		_IOW(IOCTL_MAGIC, 0x82, u16)
#define IOCTL_CMD_GPIOGET		_IOWR(IOCTL_MAGIC, 0x83, u16)


static struct usb_driver ch9344_driver;
static struct tty_driver *ch9344_tty_driver;

static DEFINE_IDR(ch9344_minors);
static DEFINE_MUTEX(ch9344_minors_lock);


static void ch9344_tty_set_termios(struct tty_struct *tty,
				struct ktermios *termios_old);

static int ch9344_get_portnum(int index);

static int ch9344_set_uartmode(struct ch9344 *ch9344, int portnum, u8 index, u8 mode);

/*
 * ch9344_minors accessors
 */

/*
 * Look up an CH9344 structure by minor. If found and not disconnected, increment
 * its refcount and return it with its mutex held.
 */
static struct ch9344 *ch9344_get_by_index(unsigned int index)
{
	struct ch9344 *ch9344;

	mutex_lock(&ch9344_minors_lock);
	ch9344 = idr_find(&ch9344_minors, index / NUMSTEP);
	if (ch9344) {
		mutex_lock(&ch9344->mutex);
		if (ch9344->disconnected) {
			mutex_unlock(&ch9344->mutex);
			ch9344 = NULL;
		} else {
			tty_port_get(&ch9344->ttyport[ch9344_get_portnum(index)].port);
			mutex_unlock(&ch9344->mutex);
		}
	}
	mutex_unlock(&ch9344_minors_lock);
	return ch9344;
}

/*
 * Try to find an available minor number and if found, associate it with 'ch9344'.
 */
static int ch9344_alloc_minor(struct ch9344 *ch9344)
{
	int minor;

	mutex_lock(&ch9344_minors_lock);
	minor = idr_alloc(&ch9344_minors, ch9344, 0, CH9344_TTY_MINORS, GFP_KERNEL);
	mutex_unlock(&ch9344_minors_lock);

	return minor;
}

/* Release the minor number associated with 'ch9344'. */
static void ch9344_release_minor(struct ch9344 *ch9344)
{
	mutex_lock(&ch9344_minors_lock);
	idr_remove(&ch9344_minors, ch9344->minor);
	mutex_unlock(&ch9344_minors_lock);
}

static int ch9344_get_portnum(int index) {
	return index % NUMSTEP;
}

/*
 * Functions for CH9344 control messages.
 */

static int ch9344_control_out(struct ch9344 *ch9344, u8 request,
                u16 value, u16 index)
{	
    int retval;
        
    retval = usb_control_msg(ch9344->dev, usb_sndctrlpipe(ch9344->dev, 0),
        request, USB_TYPE_VENDOR | USB_RECIP_DEVICE | USB_DIR_OUT,
        value, index, NULL, 0, DEFAULT_TIMEOUT);

    dev_vdbg(&ch9344->data->dev,
           "ch9344_control_out(%02x,%02x,%04x,%04x)\n",
            USB_DIR_OUT|0x40, request, value, index);

	return retval < 0 ? retval : 0;
}

static int ch9344_control_in(struct ch9344 *ch9344, 
                u8 request, u16 value, u16 index,
                char *buf, unsigned bufsize)
{	
    int retval;
    
	retval = usb_control_msg(ch9344->dev, usb_rcvctrlpipe(ch9344->dev, 0), request,
        USB_TYPE_VENDOR | USB_RECIP_DEVICE | USB_DIR_IN,
        value, index, buf, bufsize, DEFAULT_TIMEOUT);
    
    dev_vdbg(&ch9344->data->dev,
        "ch9344_control_in(%02x,%02x,%04x,%04x,%p,%u)\n",
        USB_DIR_IN|0x40, (int)request, (int)value, (int)index, buf,
        (int)bufsize);

	return retval;
}

/*
 * Functions for CH9344 cmd messages.
 */

static int ch9344_cmd_out(struct ch9344 *ch9344, u8 *buf,
                int count)
{	
    int retval, actual_len;
        
    retval = usb_bulk_msg(ch9344->dev, ch9344->cmdtx_endpoint,
                buf,
                min((unsigned int)count, ch9344->cmdsize),
                &actual_len, DEFAULT_TIMEOUT);

	if (retval ) {
		dev_dbg(&ch9344->data->dev,
			"usb_bulk_msg(send) failed, err %i\n", retval);
		return retval;
	}

	if (actual_len != count) {
		dev_dbg(&ch9344->data->dev, "only wrote %d of %d bytes\n",
			actual_len, count);
		return -1;
	}

    dev_dbg(&ch9344->data->dev, "ch9344_cmd_out--->\n");

	return actual_len;
}

static int ch9344_cmd_in(struct ch9344 *ch9344,
            u8 *buf, int count)
{	
	int retval, actual_len;

	if (buf == NULL)
		return -EINVAL;

	retval = usb_bulk_msg(ch9344->dev, ch9344->cmdrx_endpoint,
                buf,
                min((unsigned int)count, ch9344->cmdsize),
                &actual_len, DEFAULT_TIMEOUT);
                
	if (retval) {
		dev_dbg(&ch9344->data->dev,
			"usb_bulk_msg(recv) failed, err %i\n", retval);
		return retval;
	}

	if (actual_len != count) {
		dev_dbg(&ch9344->data->dev, "only read %d of %d bytes\n",
			actual_len, count);
		return -1;
	}
	
    dev_dbg(&ch9344->data->dev, "ch9344_cmd_in--->\n");
	
	return actual_len;
}



static inline int ch9344_set_control(struct ch9344 *ch9344, 
						int portnum, int control)
{
	char *buffer;
	int retval;
	const unsigned size = 3;
	u8 regaddr = 0x10 * portnum + 0x08;
	
	dev_dbg(&ch9344->data->dev, "%s\n", __func__);

	buffer = kzalloc(size, GFP_KERNEL);
	if (!buffer)
		return -ENOMEM;

	if (ch9344->quirks & QUIRK_CONTROL_LINE_STATE)
		return -EOPNOTSUPP;
	
    buffer[0] = VENDOR_WRITE_BIT_TYPE;
    buffer[1] = regaddr + REG_MCR;
    buffer[2] = control;
	
    retval = ch9344_cmd_out(ch9344, buffer, size);
	
    kfree(buffer);
	return retval;
}

static inline int ch9344_enum_portnum(struct ch9344 *ch9344)
{
	char *buffer;
	int retval;
	const unsigned size = 4;

	buffer = kzalloc(size, GFP_KERNEL);
	if (!buffer)
		return -ENOMEM;

	retval = ch9344_control_in(ch9344, 0x96, 0x0000, 0, buffer, size);
	if (retval < 0)
		goto out;

	if (retval == 4) {
		retval = buffer[1] & 0x3f;
		if (buffer[1] & (0x01 << 6))
			ch9344->modeline9 = true;
		else
			ch9344->modeline9 = false;
		dev_dbg(&ch9344->data->dev, "%s - portnum:%d\n", __func__, retval);
	} else
		retval = -EPROTO;

out:	
    kfree(buffer);
	return retval;
}

/* -------------------------------------------------------------------------- */

static int ch9344_configure(struct ch9344 *ch9344, int portnum)
{
	char *buffer;
	int ret;
	u8 request;
	u8 regaddr;

	buffer = kzalloc(ch9344->cmdsize, GFP_KERNEL);
	if (!buffer)
		return -ENOMEM;

    request = VENDOR_WRITE_REG_TYPE;
    regaddr = 0x10 * portnum + 0x08;
    
    buffer[0] = request;
    buffer[1] = regaddr + REG_IIR;
    buffer[2] = 0x87;
    ret = ch9344_cmd_out(ch9344, buffer, 0x03);
    if (ret < 0)
        goto out;

    buffer[1] = regaddr + REG_LCR;
    buffer[2] = 0x03;
    ret = ch9344_cmd_out(ch9344, buffer, 0x03);
    if (ret < 0)
        goto out;

    buffer[1] = regaddr + REG_IER;
	if (ch9344->modeline9)
    	buffer[2] = 0x0F;
	else
		buffer[2] = 0x07;
    ret = ch9344_cmd_out(ch9344, buffer, 0x03);
    if (ret < 0)
        goto out;

    buffer[1] = regaddr + REG_MCR;
    buffer[2] = 0x08;
    ret = ch9344_cmd_out(ch9344, buffer, 0x03);
    if (ret < 0)
        goto out;
/*
    buffer[0] = VENDOR_WRITE_BIT_TYPE;
    buffer[1] = regaddr + REG_IER;
    buffer[2] = pendent + 0x50;
    ret = ch9344_cmd_out(ch9344, buffer, 0x03);
    if (ret < 0)
        goto out;

    buffer[0] = VENDOR_SET_TYPE;
    buffer[1] = regaddr + REG_LCR;
    buffer[2] = baud1;
    buffer[3] = baud2;
    ret = ch9344_cmd_out(ch9344, buffer, 0x06);
    if (ret < 0)
        goto out;

    buffer[0] = VENDOR_WRITE_REG_TYPE;
    buffer[1] = regaddr + REG_LCR;
    buffer[2] = databit | paritybit | stopbit;
    ret = ch9344_cmd_out(ch9344, buffer, 0x03);
    if (ret < 0) {
        goto out;
    }
*/	
out:	
	kfree(buffer);
	return ret < 0 ? ret : 0;
}

/*
 * Write buffer management.
 * All of these assume proper locks taken by the caller.
 */

static int ch9344_wb_alloc(struct ch9344 *ch9344)
{
	int i, wbn;
	struct ch9344_wb *wb;

	wbn = 0;
	i = 0;
	for (;;) {
		wb = &ch9344->wb[wbn];
		if (!wb->use) {
			wb->use = 1;
			return wbn;
		}
		wbn = (wbn + 1) % CH9344_NW;
		if (++i >= CH9344_NW)
			return -1;
	}
}

static int ch9344_wb_is_avail(struct ch9344 *ch9344)
{
	int i, n;
	unsigned long flags;

	n = CH9344_NW;
	spin_lock_irqsave(&ch9344->write_lock, flags);
	for (i = 0; i < CH9344_NW; i++)
		n -= ch9344->wb[i].use;
	spin_unlock_irqrestore(&ch9344->write_lock, flags);
	return n;
}

/*
 * Finish write. Caller must hold ch9344->write_lock
 */
static void ch9344_write_done(struct ch9344 *ch9344, struct ch9344_wb *wb)
{
	wb->use = 0;
	ch9344->transmitting--;
	usb_autopm_put_interface_async(ch9344->data);
}

/*
 * Poke write.
 *
 * the caller is responsible for locking
 */

static int ch9344_start_wb(struct ch9344 *ch9344, struct ch9344_wb *wb)
{
	int rc;

	ch9344->transmitting++;

	wb->urb->transfer_buffer = wb->buf;
	wb->urb->transfer_dma = wb->dmah;
	wb->urb->transfer_buffer_length = wb->len;
	wb->urb->dev = ch9344->dev;

	rc = usb_submit_urb(wb->urb, GFP_ATOMIC);
	if (rc < 0) {
		dev_err(&ch9344->data->dev,
			"%s - usb_submit_urb(write bulk) failed: %d\n",
			__func__, rc);
		ch9344_write_done(ch9344, wb);
	}
	return rc;
}

/*
 * Status handlers for device responses
 */

/* bulk interface reports status changes with "bulk" transfers */
static void ch9344_cmd_irq(struct urb *urb)
{
	struct ch9344 *ch9344 = urb->context;
	unsigned char *data = urb->transfer_buffer;
	int len = urb->actual_length;
	int retval;
	int status = urb->status;
	int i;
	int portnum;
	u8 reg_iir;
	int ctrlval;
	int newctrl;
	int difference;
	unsigned long flags;

    dev_vdbg(&ch9344->data->dev, "%s, len:%d--->\n", __func__, len);

	for (i = 0; i < len; i++)
		dev_vdbg(&ch9344->data->dev, "0x%2x\n", data[i]);
	
	switch (status) {
	case 0:
		/* success */
		for (i = 0; i < len;) {
			reg_iir = *(data + i + 1);
			portnum = *(data + i) & 0x0f;
			if (reg_iir >= REG_UART_MODE && reg_iir <= REG_GPIO_IN) {
				dev_vdbg(&ch9344->data->dev, "%s portnum:%d, ext\n", __func__, portnum);
				if ((portnum >= PORT_OFFSET) || (portnum < 2 * PORT_OFFSET)) {
					portnum -= PORT_OFFSET;
					spin_lock_irqsave(&ch9344->write_lock, flags);
					if (reg_iir == REG_GPIO_IN) {
						ch9344->gpio_recv = true;
						ch9344->gpiovalin = (*(data + i + 3) << 8) + *(data + i + 2);
						dev_vdbg(&ch9344->data->dev, "%s portnum:%d, gpiovalin:0x%4x\n", __func__,
							portnum, ch9344->gpiovalin);
					}
					wake_up_interruptible(&ch9344->wgpioioctl);
					spin_unlock_irqrestore(&ch9344->write_lock, flags);
				} else
					break;
				i += 4;
				continue;
			} else if ((reg_iir & 0x0f) == REG_IIR_THR) {
				if ((portnum >= PORT_OFFSET) || (portnum < 2 * PORT_OFFSET)) {
					portnum -= PORT_OFFSET;
					spin_lock_irqsave(&ch9344->write_lock, flags);
					ch9344->write_empty[portnum] = true;
					dev_vdbg(&ch9344->data->dev, "%s portnum:%d, thr\n", __func__, portnum);
					wake_up_interruptible(&ch9344->wioctl);
					spin_unlock_irqrestore(&ch9344->write_lock, flags);
				} else
					break;
			} else if ((reg_iir & 0x0f) == REG_IIR_MS) {
				if ((portnum >= PORT_OFFSET) || (portnum < 2 * PORT_OFFSET)) {
					portnum -= PORT_OFFSET;
					dev_vdbg(&ch9344->data->dev, "%s portnum:%d, modem\n", __func__, portnum);

					spin_lock(&ch9344->read_lock);
					newctrl = ch9344->ctrlin[portnum];
					//modem signal
					ctrlval = *(data + i + 2);
					if (ctrlval & CH9344_CTRL_CTS) {
						if (ctrlval & (CH9344_CTRL_CTS << 4)) {
							newctrl |= CH9344_CTRL_CTS;
							dev_vdbg(&ch9344->data->dev, "%s portnum:%d, cts valid\n", __func__, portnum);

						}
						else {
							newctrl &= ~CH9344_CTRL_CTS;
							dev_vdbg(&ch9344->data->dev, "%s portnum:%d, cts invalid\n", __func__, portnum);
						}
					}
					if (ctrlval & CH9344_CTRL_DSR) {
						if (ctrlval & (CH9344_CTRL_DSR << 4)) {
							newctrl |= CH9344_CTRL_DSR;
							dev_vdbg(&ch9344->data->dev, "%s portnum:%d, dsr valid\n", __func__, portnum);

						}
						else {
							newctrl &= ~CH9344_CTRL_DSR;
							dev_vdbg(&ch9344->data->dev, "%s portnum:%d, dsr invalid\n", __func__, portnum);
						}
					}
					if (ctrlval & CH9344_CTRL_RI) {
						if (ctrlval & (CH9344_CTRL_RI << 4)) {
							newctrl |= CH9344_CTRL_RI;
							dev_vdbg(&ch9344->data->dev, "%s portnum:%d, ring valid\n", __func__, portnum);

						}
						else {
							newctrl &= ~CH9344_CTRL_RI;
							dev_vdbg(&ch9344->data->dev, "%s portnum:%d, ring invalid\n", __func__, portnum);
						}
					}
					if (ctrlval & CH9344_CTRL_DCD) {
						if (ctrlval & (CH9344_CTRL_DCD << 4)) {
							newctrl |= CH9344_CTRL_DCD;
							dev_vdbg(&ch9344->data->dev, "%s portnum:%d, dcd valid\n", __func__, portnum);

						}
						else {
							newctrl &= ~CH9344_CTRL_DCD;
							dev_vdbg(&ch9344->data->dev, "%s portnum:%d, dcd invalid\n", __func__, portnum);
						}
					}
					
					if (!ch9344->clocal[portnum] && 
						(ch9344->ctrlin[portnum] & ~newctrl & CH9344_CTRL_DCD)) {
						dev_vdbg(&ch9344->data->dev, "%s portnum:%d, calling hangup\n",
							__func__, portnum);
						spin_unlock(&ch9344->read_lock);
						tty_port_tty_hangup(&ch9344->ttyport[portnum].port, false);
						spin_lock(&ch9344->read_lock);
					}
					
					difference = ch9344->ctrlin[portnum] ^ newctrl;
					ch9344->ctrlin[portnum] = newctrl;
					ch9344->oldcount[portnum] = ch9344->iocount[portnum];

					if (difference & CH9344_CTRL_CTS)
						ch9344->iocount[portnum].cts++;					
					if (difference & CH9344_CTRL_DSR)
						ch9344->iocount[portnum].dsr++;					
					if (difference & CH9344_CTRL_RI)
						ch9344->iocount[portnum].rng++;					
					if (difference & CH9344_CTRL_DCD)
						ch9344->iocount[portnum].dcd++;
					spin_unlock(&ch9344->read_lock);
					
					if (difference)
						wake_up_interruptible(&ch9344->wmodemioctl);
				} else
					break;
			}  else if ((reg_iir & 0x0f) == REG_IIR_RLS) {
				if ((portnum >= PORT_OFFSET) || (portnum < 2 * PORT_OFFSET)) {
					portnum -= PORT_OFFSET;
					dev_vdbg(&ch9344->data->dev, "%s portnum:%d, modem\n", __func__, portnum);

					spin_lock(&ch9344->read_lock);
					newctrl = ch9344->ctrlin[portnum];
					//lsr signal
					ctrlval = *(data + i + 2);
					if (ctrlval & (CH9344_CTRL_OVERRUN >> 3)) {
						if (ctrlval & (CH9344_CTRL_OVERRUN >> 3)) {
							newctrl |= CH9344_CTRL_OVERRUN;
							dev_vdbg(&ch9344->data->dev, "%s portnum:%d, overrun\n", __func__, portnum);
						}
						else {
							newctrl &= ~CH9344_CTRL_OVERRUN;
						}
					}
					if (ctrlval & (CH9344_CTRL_PARITY >> 3)) {
						if (ctrlval & (CH9344_CTRL_PARITY >> 3)) {
							newctrl |= CH9344_CTRL_PARITY;
							dev_vdbg(&ch9344->data->dev, "%s portnum:%d, parity error\n", __func__, portnum);
						}
						else {
							newctrl &= ~CH9344_CTRL_PARITY;
						}
					}
					if (ctrlval & (CH9344_CTRL_FRAMING >> 3)) {
						if (ctrlval & (CH9344_CTRL_FRAMING >> 3)) {
							newctrl |= CH9344_CTRL_FRAMING;
							dev_vdbg(&ch9344->data->dev, "%s portnum:%d, frame error\n", __func__, portnum);
						}
						else {
							newctrl &= ~CH9344_CTRL_FRAMING;
						}
					}
					if (ctrlval & (CH9344_CTRL_BRK >> 3)) {
						if (ctrlval & (CH9344_CTRL_BRK >> 3)) {
							newctrl |= CH9344_CTRL_BRK;
							dev_vdbg(&ch9344->data->dev, "%s portnum:%d, break signal\n", __func__, portnum);
						}
						else {
							newctrl &= ~CH9344_CTRL_BRK;
						}
					}
					difference = ch9344->ctrlin[portnum] ^ newctrl;
					ch9344->ctrlin[portnum] = newctrl;
					ch9344->oldcount[portnum] = ch9344->iocount[portnum];

					if (difference & CH9344_CTRL_OVERRUN)
						ch9344->iocount[portnum].overrun++;					
					if (difference & CH9344_CTRL_PARITY)
						ch9344->iocount[portnum].parity++;					
					if (difference & CH9344_CTRL_FRAMING)
						ch9344->iocount[portnum].frame++;					
					if (difference & CH9344_CTRL_BRK)
						ch9344->iocount[portnum].brk++;
					spin_unlock(&ch9344->read_lock);
					
					if (difference)
						wake_up_interruptible(&ch9344->wmodemioctl);
				} else
					break;
			} else {
				dev_dbg(&ch9344->data->dev,
						"%s - wrong status received",
						__func__);				
			}
			i += 3;
		}
		break;
	case -ECONNRESET:
	case -ENOENT:
	case -ESHUTDOWN:
		/* this urb is terminated, clean up */
		dev_dbg(&ch9344->data->dev,
				"%s - urb shutting down with status: %d\n",
				__func__, status);
		return;
	default:
		dev_dbg(&ch9344->data->dev,
				"%s - nonzero urb status received: %d\n",
				__func__, status);
		goto exit;
	}

	usb_mark_last_busy(ch9344->dev);

exit:
	retval = usb_submit_urb(urb, GFP_ATOMIC);
	if (retval && retval != -EPERM)
		dev_err(&ch9344->data->dev, "%s - usb_submit_urb failed: %d\n",
							__func__, retval);
}

static int ch9344_submit_read_urb(struct ch9344 *ch9344, int index, gfp_t mem_flags)
{
	int res;

	if (!test_and_clear_bit(index, &ch9344->read_urbs_free))
		return 0;

	dev_vdbg(&ch9344->data->dev, "%s - urb %d\n", __func__, index);

	res = usb_submit_urb(ch9344->read_urbs[index], mem_flags);
	if (res) {
		if (res != -EPERM) {
			dev_err(&ch9344->data->dev,
					"%s - usb_submit_urb failed: %d\n",
					__func__, res);
		}
		set_bit(index, &ch9344->read_urbs_free);
		return res;
	}

	return 0;
}

static int ch9344_submit_read_urbs(struct ch9344 *ch9344, gfp_t mem_flags)
{
	int res;
	int i;

	for (i = 0; i < ch9344->rx_buflimit; ++i) {
		res = ch9344_submit_read_urb(ch9344, i, mem_flags);
		if (res)
			return res;
	}

	return 0;
}

static void ch9344_process_read_urb(struct ch9344 *ch9344, struct urb *urb)
{
	int size;
	char buffer[512];
	int i = 0;
	int portnum;
	u8 usblen;

	if (!urb->actual_length)
		return;
	size = urb->actual_length;

	memcpy(buffer, urb->transfer_buffer, urb->actual_length);

    for (i = 0; i < size; i += 32) {
        portnum = *(buffer + i);
        if (portnum < PORT_OFFSET || portnum > 2 * PORT_OFFSET - 1) {
            break;
        }
        portnum -= PORT_OFFSET;
        usblen = *(buffer + i + 1);
        if (usblen > 30) {
            break;
        }
        tty_insert_flip_string(&ch9344->ttyport[portnum].port, buffer + i + 2,
			usblen);
	    tty_flip_buffer_push(&ch9344->ttyport[portnum].port);
    }
}

static void ch9344_read_bulk_callback(struct urb *urb)
{
	struct ch9344_rb *rb = urb->context;
	struct ch9344 *ch9344 = rb->instance;
	unsigned long flags;
	int status = urb->status;

	dev_vdbg(&ch9344->data->dev, "%s - urb %d, len %d\n", __func__,
					rb->index, urb->actual_length);

	if (!ch9344->dev) {
		set_bit(rb->index, &ch9344->read_urbs_free);
		dev_dbg(&ch9344->data->dev, "%s - disconnected\n", __func__);
		return;
	}

	if (status) {
		set_bit(rb->index, &ch9344->read_urbs_free);
		dev_dbg(&ch9344->data->dev, "%s - non-zero urb status: %d\n",
							__func__, status);
		return;
	}

	usb_mark_last_busy(ch9344->dev);

	ch9344_process_read_urb(ch9344, urb);
	set_bit(rb->index, &ch9344->read_urbs_free);
	ch9344_submit_read_urb(ch9344, rb->index, GFP_ATOMIC);
}

/* data interface wrote those outgoing bytes */
static void ch9344_write_bulk(struct urb *urb)
{
	struct ch9344_wb *wb = urb->context;
	struct ch9344 *ch9344 = wb->instance;
	unsigned long flags;
	int status = urb->status;

	if (status || (urb->actual_length != urb->transfer_buffer_length))
		dev_vdbg(&ch9344->data->dev, "%s - len %d/%d, status %d\n",
			__func__,
			urb->actual_length,
			urb->transfer_buffer_length,
			status);

	spin_lock_irqsave(&ch9344->write_lock, flags);
	ch9344_write_done(ch9344, wb);
	spin_unlock_irqrestore(&ch9344->write_lock, flags);
//	schedule_work(&ch9344->work);
}

static void ch9344_softint(struct work_struct *work)
{
	struct ch9344 *ch9344 = container_of(work, struct ch9344, work);

	dev_vdbg(&ch9344->data->dev, "%s\n", __func__);

	tty_port_tty_wakeup(&ch9344->ttyport[0].port);
}

/*
 * TTY handlers
 */

static int ch9344_tty_install(struct tty_driver *driver, struct tty_struct *tty)
{
	struct ch9344 *ch9344;
	int retval;

	dev_dbg(tty->dev, "%s, index:%d\n", __func__, tty->index);

	ch9344 = ch9344_get_by_index(tty->index);
	if (!ch9344)
		return -ENODEV;

	retval = tty_standard_install(driver, tty);
	if (retval)
		goto error_init_termios;

	tty->driver_data = ch9344;

	return 0;

error_init_termios:
	tty_port_put(&ch9344->ttyport[ch9344_get_portnum(tty->index)].port);
	return retval;
}


static int ch9344_tty_open(struct tty_struct *tty, struct file *filp)
{
	struct ch9344 *ch9344 = tty->driver_data;
	return tty_port_open(&ch9344->ttyport[ch9344_get_portnum(tty->index)].port, tty, filp);
}

static inline void tty_set_portdata(struct ch9344_ttyport *port, void *data)
{
	port->portdata = data;
}

static inline void *tty_get_portdata(struct ch9344_ttyport *port)
{
	return (port->portdata);
}

static void ch9344_port_dtr_rts(struct tty_port *port, int raise)
{
	struct ch9344_ttyport *ttyport = container_of(port, struct ch9344_ttyport, port);
	struct ch9344 *ch9344 = tty_get_portdata(ttyport);
	int portnum = ttyport->portnum;
	int val;
	int rv;

	dev_dbg(&ch9344->data->dev, "%s\n", __func__);
	return;

	if (raise)
		val = CH9344_CTRL_DTR | CH9344_CTRL_RTS;
	else
		val = 0;

	// FIXME: add missing ctrlout locking throughout driver 
	ch9344->ctrlout[portnum] = val;

	rv = ch9344_set_control(ch9344, portnum, 0x01);
	if (rv)
		dev_err(&ch9344->control->dev, "failed to set dtr\n");
	rv = ch9344_set_control(ch9344, portnum, 0x11);
	if (rv)
		dev_err(&ch9344->control->dev, "failed to set rts\n");
}

static int ch9344_port_activate(struct tty_port *port, struct tty_struct *tty)
{
	struct ch9344_ttyport *ttyport = container_of(port, struct ch9344_ttyport, port);
	struct ch9344 *ch9344 = tty_get_portdata(ttyport);
	int portnum = ttyport->portnum;
	int retval = -ENODEV;

	dev_dbg(&ch9344->data->dev, "%s\n", __func__);

	mutex_lock(&ch9344->mutex);
	if (ch9344->disconnected)
		goto disconnected;


	retval = usb_autopm_get_interface(ch9344->data);
	if (retval)
		goto error_get_interface;

	// FIXME: Why do we need this? Allocating 64K of physically contiguous
	// memory is really nasty...
	
	set_bit(TTY_NO_WRITE_SPLIT, &tty->flags);
	ch9344->data->needs_remote_wakeup = 1;

	retval = ch9344_configure(ch9344, portnum);
	if (retval)
		goto error_configure;

	ch9344_tty_set_termios(tty, NULL);

	usb_autopm_put_interface(ch9344->data);

	mutex_unlock(&ch9344->mutex);

	return 0;

error_configure:
	usb_autopm_put_interface(ch9344->data);
error_get_interface:
disconnected:
	mutex_unlock(&ch9344->mutex);

	return usb_translate_errors(retval);
}

static void ch9344_port_destruct(struct tty_port *port)
{
	struct ch9344_ttyport *ttyport = container_of(port, struct ch9344_ttyport, port);
	struct ch9344 *ch9344 = tty_get_portdata(ttyport);
	int portnum = ttyport->portnum;

	dev_dbg(&ch9344->data->dev, "%s, portnum:%d\n", __func__, portnum);

    if (portnum == (ch9344->num_ports - 1)) {
    	ch9344_release_minor(ch9344);
    	usb_put_intf(ch9344->data);
    	kfree(ch9344);
    }
}

static void ch9344_port_shutdown(struct tty_port *port)
{
	struct ch9344_ttyport *ttyport = container_of(port, struct ch9344_ttyport, port);
	struct ch9344 *ch9344 = tty_get_portdata(ttyport);
	int portnum = ttyport->portnum;
	
	dev_dbg(&ch9344->data->dev, "%s, portnum:%d\n", __func__, portnum); 
}

static void ch9344_tty_cleanup(struct tty_struct *tty)
{
	struct ch9344 *ch9344 = tty->driver_data;
	dev_dbg(&ch9344->data->dev, "%s\n", __func__);
	tty_port_put(&ch9344->ttyport[ch9344_get_portnum(tty->index)].port);
}

static void ch9344_tty_hangup(struct tty_struct *tty)
{
	struct ch9344 *ch9344 = tty->driver_data;
	dev_dbg(&ch9344->data->dev, "%s\n", __func__);
	tty_port_hangup(&ch9344->ttyport[ch9344_get_portnum(tty->index)].port);
}

static void ch9344_tty_close(struct tty_struct *tty, struct file *filp)
{
	struct ch9344 *ch9344 = tty->driver_data;
	int portnum = ch9344_get_portnum(tty->index);
	int ret;
	
	dev_dbg(&ch9344->data->dev, "%s\n", __func__);
	tty_port_close(&ch9344->ttyport[portnum].port, tty, filp);
	if (ch9344->uartmode[portnum] == MODE_HARDFLOW) {
		ret = ch9344_set_uartmode(ch9344, portnum, portnum, MODE_NORMAL);
		if (!ret) {
			ch9344->uartmode[portnum] = MODE_NORMAL;
		}
	}

}

static int ch9344_tty_write(struct tty_struct *tty,
					const unsigned char *buf, int count)
{
	struct ch9344 *ch9344 = tty->driver_data;
	int stat;
	unsigned long flags;
	int wbn;
	struct ch9344_wb *wb;
	int portnum = ch9344_get_portnum(tty->index);
	int timeout;
	int maxep = ch9344->writesize / 20;
	int packnum, maxpacknum;
	int packlen, total_len, sendlen;
	DECLARE_WAITQUEUE(wait, current);

	if (!count)
		return 0;

	if (ch9344->writesize % (maxep - 3))
		maxpacknum = ch9344->writesize / (maxep - 3) + 1;
	else
		maxpacknum = ch9344->writesize / (maxep - 3);

	count = count > (ch9344->writesize - maxpacknum * 3) ? \
		(ch9344->writesize - maxpacknum * 3) : count;
	total_len = count;
	sendlen = 0;

	dev_vdbg(&ch9344->data->dev, "%s - count:%d\n", __func__, count);

	if (count % (maxep - 3))
		packnum = count / (maxep - 3) + 1;
	else
		packnum = count / (maxep - 3);

//	dev_vdbg(&ch9344->data->dev, "%s - maxpacknum:%d, packnum:%d\n",
//		__func__, maxpacknum, packnum);

transmit:
	spin_lock_irqsave(&ch9344->write_lock, flags);
	wbn = ch9344_wb_alloc(ch9344);
	if (wbn < 0) {
		spin_unlock_irqrestore(&ch9344->write_lock, flags);
		return 0;
	}
	wb = &ch9344->wb[wbn];

	if (!ch9344->dev) {
		wb->use = 0;
		spin_unlock_irqrestore(&ch9344->write_lock, flags);
		return -ENODEV;
	}

    // add deal
	if (total_len > maxep - 3) {
		packlen = maxep - 3;
		total_len -= packlen;
	} else {
		packlen = total_len;
		total_len = 0;
	}
	*(wb->buf) = PORT_OFFSET + ch9344_get_portnum(tty->index);
	*(wb->buf + 1) = packlen;
	*(wb->buf + 2) = packlen >> 8;
	memcpy(wb->buf + 3, buf + sendlen, packlen);
	wb->len = packlen + 3;
	sendlen += packlen;

	stat = usb_autopm_get_interface_async(ch9344->data);
	if (stat) {
		wb->use = 0;
		spin_unlock_irqrestore(&ch9344->write_lock, flags);
		return stat;
	}

	if (ch9344->susp_count) {
		usb_anchor_urb(wb->urb, &ch9344->delayed);
		spin_unlock_irqrestore(&ch9344->write_lock, flags);
		return sendlen;
	}

	ch9344->write_empty[portnum] = false;
	stat = ch9344_start_wb(ch9344, wb);
	if (stat < 0) {
		ch9344->write_empty[portnum] = true;
		spin_unlock_irqrestore(&ch9344->write_lock, flags);
		return stat;
	}
	
	add_wait_queue(&ch9344->wioctl, &wait);
	set_current_state(TASK_INTERRUPTIBLE);
	timeout = msecs_to_jiffies(DEFAULT_TIMEOUT);
	if (!ch9344->write_empty[portnum]) {
		spin_unlock_irqrestore(&ch9344->write_lock, flags);
		timeout = schedule_timeout(timeout);
		spin_lock_irqsave(&ch9344->write_lock, flags);
	}
	set_current_state(TASK_RUNNING);
	remove_wait_queue(&ch9344->wioctl, &wait);
	spin_unlock_irqrestore(&ch9344->write_lock, flags);

	if (timeout <= 0) {
		dev_dbg(&ch9344->data->dev, "%s - schedule_timeout\n", __func__);
		return sendlen ? sendlen : -ETIMEDOUT;
	} 

	if (signal_pending(current)) {
		dev_dbg(&ch9344->data->dev, "%s - signal_pending\n", __func__);
		return sendlen ? sendlen : -EINTR;
	}
	if (total_len != 0)
		goto transmit;

	return sendlen;
}

static int ch9344_tty_write_room(struct tty_struct *tty)
{
	struct ch9344 *ch9344 = tty->driver_data;
	/*
	 * Do not let the line discipline to know that we have a reserve,
	 * or it might get too enthusiastic.
	 */
	return ch9344_wb_is_avail(ch9344) ? ch9344->writesize : 0;
}

static int ch9344_tty_chars_in_buffer(struct tty_struct *tty)
{
	struct ch9344 *ch9344 = tty->driver_data;
	/*
	 * if the device was unplugged then any remaining characters fell out
	 * of the connector ;)
	 */
	if (ch9344->disconnected)
		return 0;
	/*
	 * This is inaccurate (overcounts), but it works.
	 */
	return (CH9344_NW - ch9344_wb_is_avail(ch9344)) * ch9344->writesize;
}

static int ch9344_tty_tiocmget(struct tty_struct *tty)
{
	struct ch9344 *ch9344 = tty->driver_data;
	int portnum = ch9344_get_portnum(tty->index);
	unsigned int result;

	result = (ch9344->ctrlout[portnum] & CH9344_CTRL_DTR ? TIOCM_DTR : 0) |
	       (ch9344->ctrlout[portnum] & CH9344_CTRL_RTS ? TIOCM_RTS : 0) |
	       (ch9344->ctrlin[portnum]  & CH9344_CTRL_CTS ? TIOCM_CTS : 0) | 
	       (ch9344->ctrlin[portnum]  & CH9344_CTRL_DSR ? TIOCM_DSR : 0) |
	       (ch9344->ctrlin[portnum]  & CH9344_CTRL_RI  ? TIOCM_RI  : 0) |
	       (ch9344->ctrlin[portnum]  & CH9344_CTRL_DCD ? TIOCM_CD  : 0);
	dev_dbg(&ch9344->data->dev, "%s - result = %x\n", __func__, result);

	return result;
}

static int ch9344_tty_tiocmset(struct tty_struct *tty,
			    unsigned int set, unsigned int clear)
{
	struct ch9344 *ch9344 = tty->driver_data;
	unsigned int newctrl;
	int rv;
	int portnum = ch9344_get_portnum(tty->index);
	unsigned int xorval;

	newctrl = ch9344->ctrlout[portnum];
	dev_dbg(&ch9344->data->dev, "%s, newctrl:0x%2x, set:0x%2x, clear:0x%2x\n",
		__func__, newctrl, set, clear);
	set = (set & TIOCM_DTR ? CH9344_CTRL_DTR : 0) |
					(set & TIOCM_RTS ? CH9344_CTRL_RTS : 0);
	clear = (clear & TIOCM_DTR ? CH9344_CTRL_DTR : 0) |
					(clear & TIOCM_RTS ? CH9344_CTRL_RTS : 0);

	newctrl = (newctrl & ~clear) | set;

	if (ch9344->ctrlout[portnum] == newctrl)
		return 0;
	xorval = newctrl ^ ch9344->ctrlout[portnum];
	if (xorval & CH9344_CTRL_DTR) {
		if (newctrl & CH9344_CTRL_DTR)
			rv = ch9344_set_control(ch9344, portnum, 0x01);
		else
			rv = ch9344_set_control(ch9344, portnum, 0x00);
	}
	if (xorval & CH9344_CTRL_RTS) {
		if (newctrl & CH9344_CTRL_RTS)
			rv = ch9344_set_control(ch9344, portnum, 0x11);
		else
			rv = ch9344_set_control(ch9344, portnum, 0x10);
	}	
	ch9344->ctrlout[portnum] = newctrl;
	
	return rv;
}

static int ch9344_set_uartmode(struct ch9344 *ch9344, int portnum, u8 index, u8 mode)
{
	u8 *buffer;
	int ret;
	u8 request;
	u8 regaddr;
	u8 mode4 = 0;
	int i;
	
	buffer = kzalloc(ch9344->cmdsize, GFP_KERNEL);
	if (!buffer)
		return -ENOMEM;

	if ((ch9344->uartmode[portnum] == MODE_NORMAL) && (mode == MODE_HARDFLOW)) {
		regaddr = 0x10 * portnum + 0x08;
		buffer[0] = VENDOR_WRITE_BIT_TYPE;
		buffer[1] = regaddr + REG_MCR;
		buffer[2] = 0x51;
		ret = ch9344_cmd_out(ch9344, buffer, 0x03);
		if (ret < 0)
			goto out;
	}

	if ((ch9344->uartmode[portnum] == MODE_HARDFLOW) && (mode == MODE_NORMAL)) {
		buffer[0] = VENDOR_WRITE_BIT_TYPE;
		buffer[1] = regaddr + REG_MCR;
		buffer[2] = 0x50;
		ret = ch9344_cmd_out(ch9344, buffer, 0x03);
		if (ret < 0)
			goto out;
	}

	for (i = 0; i < MAXPORT; i++) {
		if (i != index)
			mode4 |= (ch9344->uartmode[i]) << (i * 2);
	}
	mode4 |= mode << (index * 2);
	request = VENDOR_WRITE_BIT_EXT_TYPE + portnum + PORT_OFFSET;
	regaddr = REG_UART_MODE;
	memset(buffer, 0x00, ch9344->cmdsize);
	buffer[0] = request;
	buffer[1] = regaddr;
	buffer[2] = mode4;
	ret = ch9344_cmd_out(ch9344, buffer, 0x04);

out:	
	kfree(buffer);
	return ret < 0 ? ret : 0;
}

static int ch9344_set_gpiodir(struct ch9344 *ch9344, int portnum, u8 gpionumber, u8 gpiodir)
{
	u8 *buffer;
	int ret;
	u8 request;
	u8 regaddr;
	u16 gpiodirs = 0;
	int i;
	
	buffer = kzalloc(ch9344->cmdsize, GFP_KERNEL);
	if (!buffer)
		return -ENOMEM;

	for (i = 0; i < MAXGPIO; i++) {
		if (i != gpionumber)
			gpiodirs |= (ch9344->gpiodir[i]) << i;
	}
	gpiodirs |= gpiodir << gpionumber;

	request = VENDOR_WRITE_BIT_EXT_TYPE + portnum + PORT_OFFSET;
	regaddr = REG_GPIO_DIR;
	buffer[0] = request;
	buffer[1] = regaddr;
	buffer[2] = gpiodirs;
	buffer[3] = gpiodirs >> 8;
	ret = ch9344_cmd_out(ch9344, buffer, 0x04);
	kfree(buffer);
	return ret < 0 ? ret : 0;
}

static int ch9344_set_gpioval(struct ch9344 *ch9344, int portnum, u8 gpionumber, u8 gpioval)
{
	u8 *buffer;
	int ret;
	u8 request;
	u8 regaddr;
	u16 gpiovalues = 0;
	int i;
	
	buffer = kzalloc(ch9344->cmdsize, GFP_KERNEL);
	if (!buffer)
		return -ENOMEM;

	for (i = 0; i < MAXGPIO; i++) {
		if (i != gpionumber)
			gpiovalues |= (ch9344->gpioval[i]) << i;
	}
	gpiovalues |= gpioval << gpionumber;

	request = VENDOR_WRITE_BIT_EXT_TYPE + portnum + PORT_OFFSET;
	regaddr = REG_GPIO_OUT;
	buffer[0] = request;
	buffer[1] = regaddr;
	buffer[2] = gpiovalues;
	buffer[3] = gpiovalues >> 8;
	ret = ch9344_cmd_out(ch9344, buffer, 0x04);
	kfree(buffer);
	return ret < 0 ? ret : 0;

}

static int ch9344_get_gpioval(struct ch9344 *ch9344, int portnum)
{
	u8 *buffer;
	int ret;
	u8 request;
	u8 regaddr;
	u16 gpiodirs = 0;
	int i;
	
	buffer = kzalloc(ch9344->cmdsize, GFP_KERNEL);
	if (!buffer)
		return -ENOMEM;

	for (i = 0; i < MAXGPIO; i++) {
		gpiodirs |= (ch9344->gpiodir[i]) << i;
	}

	request = VENDOR_WRITE_BIT_EXT_TYPE + portnum + PORT_OFFSET;
	regaddr = REG_GPIO_IN;
	buffer[0] = request;
	buffer[1] = regaddr;
	buffer[2] = gpiodirs;
	buffer[3] = gpiodirs >> 8;
	ret = ch9344_cmd_out(ch9344, buffer, 0x04);
	kfree(buffer);
	if (ret < 0)
		goto out;

	ret = wait_event_interruptible_timeout(ch9344->wgpioioctl,
							ch9344->gpio_recv,
							msecs_to_jiffies(DEFAULT_TIMEOUT));
	if (ret == 0) {
		dev_dbg(&ch9344->data->dev, "%s - schedule_timeout\n", __func__);
		return -ETIMEDOUT;
	} 

	if (ret < 0) {
		dev_dbg(&ch9344->data->dev, "%s - error while wait_event\n", __func__);
		return ret;
	}

	ch9344->gpio_recv = false;
	
out:
	return ret < 0 ? ret : 0;
}

static int ch9344_tty_ioctl(struct tty_struct *tty,
					unsigned int cmd, unsigned long arg)
{
	struct ch9344 *ch9344 = tty->driver_data;
	int rv = -ENOIOCTLCMD;
	int portnum = ch9344_get_portnum(tty->index);
	u8 uartmode;
	int i;
	
	u16 inarg;
	u8 inargH, inargL;
	u16 __user *argval = (u16 __user *)arg;
	u8 gpioval;
	u8 gpiogroup;
	u8 gpionumber;
	u8 gpiodir;
	u8 portindex;

	switch (cmd) {
	case TIOCGSERIAL: /* gets serial port data */
		break;
	case TIOCSSERIAL:
		break;
	case TIOCMIWAIT:
		break;
	case TIOCGICOUNT:
		break;
	case IOCTL_CMD_GPIOENABLE:
		if (get_user(inarg, argval)) {
			rv = -EFAULT;
			goto out;
		}
		inargH = inarg >> 8;
		inargL = inarg;
		portindex = inargH;
		if (inargL)
			uartmode = MODE_GPIO;
		else
			uartmode = MODE_NORMAL;
		if ((portindex > 3) || (ch9344->uartmode[portindex] == MODE_HARDFLOW)) {
			rv = -EINVAL;
			goto out;
		}
		mutex_lock(&ch9344->gpiomutex);
		rv = ch9344_set_uartmode(ch9344, portnum, portindex, uartmode);
		if (!rv) {
			ch9344->uartmode[portindex] = uartmode;
			if (uartmode == MODE_NORMAL) {
				//restores gpio related vars here
				for (i = portindex * 3; i < portindex * 3 + 3; i++) {
					ch9344->gpiodir[i] = GPIO_DIR_IN;
					ch9344->gpioval[i] = GPIO_LOW;
				}
			}
		}
		mutex_unlock(&ch9344->gpiomutex);
		break;
	case IOCTL_CMD_GPIODIR:
		if (get_user(inarg, argval)) {
			rv = -EFAULT;
			goto out;
		}
		//inargH indicates gpio number, inargL indicates gpio direction
		inargH = inarg >> 8;
		inargL = inarg;
		gpionumber = inargH;
		gpiogroup = gpionumber / 3;
		if (inargL)
			gpiodir = GPIO_DIR_OUT;
		else
			gpiodir = GPIO_DIR_IN;
		if ((gpionumber > MAXGPIO) || (ch9344->uartmode[gpiogroup] != MODE_GPIO)) {
			rv = -EINVAL;
			goto out;
		}
		mutex_lock(&ch9344->gpiomutex);
		rv = ch9344_set_gpiodir(ch9344, portnum, gpionumber, gpiodir);
		if (!rv) {
			ch9344->gpiodir[gpionumber] = gpiodir;
			if (gpiodir == GPIO_DIR_IN)
				ch9344->gpioval[gpionumber] = GPIO_HIGH;
		}
		mutex_unlock(&ch9344->gpiomutex);
		break;
	case IOCTL_CMD_GPIOSET:
		if (get_user(inarg, argval)) {
			rv = -EFAULT;
			goto out;
		}
		//inargH indicates gpio number, inargL indicates gpio output value
		inargH = inarg >> 8;
		inargL = inarg;
		gpionumber = inargH;
		gpiogroup = gpionumber / 3;
		if (inargL)
			gpioval = GPIO_HIGH;
		else
			gpioval = GPIO_LOW;
		if ((gpionumber > MAXGPIO) || (ch9344->uartmode[gpiogroup] != MODE_GPIO)
				|| (ch9344->gpiodir[gpionumber] != GPIO_DIR_OUT)) {
			rv = -EINVAL;
			goto out;
		}
		mutex_lock(&ch9344->gpiomutex);
		rv = ch9344_set_gpioval(ch9344, portnum, gpionumber, gpioval);
		if (!rv) {
			ch9344->gpioval[gpionumber] = gpioval;
		}
		mutex_unlock(&ch9344->gpiomutex);
		break;
	case IOCTL_CMD_GPIOGET:
		if (get_user(inarg, argval)) {
			rv = -EFAULT;
			goto out;
		}
		//inargH indicates gpio number, inargL indicates gpio output value
		inargH = inarg >> 8;
		inargL = inarg;
		gpionumber = inargH;
		gpiogroup = gpionumber / 3;
		if ((gpionumber > MAXGPIO) || (ch9344->uartmode[gpiogroup] != MODE_GPIO)
				|| (ch9344->gpiodir[gpionumber] != GPIO_DIR_IN)) {
			rv = -EINVAL;
			goto out;
		}
		mutex_lock(&ch9344->gpiomutex);
		rv = ch9344_get_gpioval(ch9344, portnum);
		if (!rv) {
			gpioval = ((ch9344->gpiovalin) & (1 << gpionumber)) ? GPIO_HIGH : GPIO_LOW;
			if (put_user(gpioval, argval)) {
				rv = -EFAULT;
				goto out;
			}
		}
		mutex_unlock(&ch9344->gpiomutex);
		break;
	default:
		break;
	}

out:
	return rv;
}

static int ch9344_get_baud_rate(int clockRate, int baudRate,
    unsigned char *baud1, unsigned char *baud2)
{
    int division, x2;
    
    if (baudRate < 0 || baudRate > 12000000) {
        return -1;
    }
	//caculate division from baudRate
	if (baudRate == 2000000) {
		*baud1 = 2;
		*baud2 = 0;
	} else {
		division = 10 * clockRate / 16 / baudRate;
		x2 = division % 10;
		division /= 10;
		if (x2 >= 5)
			division++;
		*baud1 = division;
		*baud2 = (unsigned char)(division >> 8);
	}
	
    return 0;
}

static void ch9344_tty_set_termios(struct tty_struct *tty,
						struct ktermios *termios_old)
{
	struct ch9344 *ch9344 = tty->driver_data;
	struct ktermios *termios = &tty->termios;
	struct usb_cdc_line_coding newline;
	int portnum = ch9344_get_portnum(tty->index);
	int newctrl = ch9344->ctrlout[portnum];
	int ret;
	unsigned char baud1 = 0;
    unsigned char baud2 = 0;
	unsigned char baud3 = 0;
    unsigned char databit = 0, paritybit = 0, stopbit = 0;
    unsigned char pendent = 0x00;
    int clockrate = 1843200;
    u8 regaddr;
	char *buffer;

	dev_dbg(tty->dev, "%s\n", __func__);
	
	if (termios_old &&
		!tty_termios_hw_change(&tty->termios, termios_old)) {
		dev_dbg(tty->dev, "%s - nothing to change\n", __func__);
		return;
	}

	buffer = kzalloc(ch9344->cmdsize, GFP_KERNEL);
	if (!buffer) {
		//restore termios
		if (termios_old)
			tty->termios = *termios_old;
		return;
	}

	newline.dwDTERate = tty_get_baud_rate(tty);
	if (newline.dwDTERate == 0)
		newline.dwDTERate = 9600;
	if (newline.dwDTERate > 115200) {
        pendent = 0x01;
        clockrate = 44236800;
    }
    ch9344_get_baud_rate(clockrate, newline.dwDTERate, &baud1, &baud2);
	switch (newline.dwDTERate) {
	case 250000:
		baud3 = 1;
		break;
	case 500000:
		baud3 = 2;
		break;
	case 1000000:
		baud3 = 3;
		break;
	case 1500000:
		baud3 = 4;
		break;
	case 3000000:
		baud3 = 5;
		break;
	case 12000000:
		baud3 = 6;
		break;
	default:
		baud3 = 0;
		break;
	}

	newline.bCharFormat = termios->c_cflag & CSTOPB ? 2 : 1;
	if (newline.bCharFormat == 2)
	    stopbit = 0x04;
	    
	newline.bParityType = termios->c_cflag & PARENB ?
				(termios->c_cflag & PARODD ? 1 : 2) +
				(termios->c_cflag & CMSPAR ? 2 : 0) : 0;

	switch (newline.bParityType) {
    case 0x01:
        paritybit = 0x08;
        dev_vdbg(&ch9344->data->dev, "parity = odd");
        break;
    case 0x02:
		paritybit = (0x01 << 4) + 0x08;
		dev_vdbg(&ch9344->data->dev, "parity = even");
        break;
    case 0x03:
		paritybit = (0x02 << 4) + 0x08;
		dev_vdbg(&ch9344->data->dev, "parity = mark");
        break;
    case 0x04:
		paritybit = (0x03 << 4) + 0x08;
		dev_vdbg(&ch9344->data->dev, "parity = space");
        break;
    default:
        paritybit = 0x00;
        dev_vdbg(&ch9344->data->dev, "parity = none");
        break;
	}
		
	switch (termios->c_cflag & CSIZE) {
	case CS5:
		newline.bDataBits = 5;
		databit = 0x00;
		break;
	case CS6:
		newline.bDataBits = 6;
		databit = 0x01;
		break;
	case CS7:
		newline.bDataBits = 7;
		databit = 0x02;
		break;
	case CS8:
	default:
		newline.bDataBits = 8;
		databit = 0x03;
		break;
	}
	
	/* FIXME: Needs to clear unsupported bits in the termios */
	ch9344->clocal[portnum] = ((termios->c_cflag & CLOCAL) != 0);

	if (C_BAUD(tty) == B0) {
		newline.dwDTERate = ch9344->line[portnum].dwDTERate;
		newctrl &= ~CH9344_CTRL_DTR;
	} else if (termios_old && (termios_old->c_cflag & CBAUD) == B0) {
		newctrl |= CH9344_CTRL_DTR;
	}

	regaddr = 0x10 * ch9344_get_portnum(tty->index) + 0x08;

    memset(buffer, 0x00, ch9344->cmdsize);
    buffer[0] = VENDOR_WRITE_BIT_TYPE;
    buffer[1] = regaddr + 0x01;
    buffer[2] = pendent + 0x50;
    ret = ch9344_cmd_out(ch9344, buffer, 0x03);
    if (ret < 0)
        goto out;

    memset(buffer, 0x00, ch9344->cmdsize);
    buffer[0] = VENDOR_SET_TYPE;
    buffer[1] = regaddr + 0x03;
    buffer[2] = baud1;
    buffer[3] = baud2;
	buffer[4] = baud3;
    ret = ch9344_cmd_out(ch9344, buffer, 0x06);
    if (ret < 0)
        goto out;

    memset(buffer, 0x00, ch9344->cmdsize);
    buffer[0] = VENDOR_WRITE_REG_TYPE;
    buffer[1] = regaddr + 0x03;
    buffer[2] = databit | paritybit | stopbit;
    ret = ch9344_cmd_out(ch9344, buffer, 0x03);
    if (ret < 0) {
        goto out;
    }

	dev_vdbg(&ch9344->data->dev, "newctrl:0x%2x\n", newctrl);
	if (newctrl != ch9344->ctrlout[portnum]) {
		if (newctrl & CH9344_CTRL_DTR)
			ch9344_set_control(ch9344, portnum, 0x01);
		else
			ch9344_set_control(ch9344, portnum, 0x00);
		ch9344->ctrlout[portnum] = newctrl;
	}

	if (memcmp(&ch9344->line[portnum], &newline, sizeof newline)) {
		memcpy(&ch9344->line[portnum], &newline, sizeof newline);
		dev_vdbg(&ch9344->data->dev, "%s - set line: %d %d %d %d\n",
			__func__,
			newline.dwDTERate,
			newline.bCharFormat, newline.bParityType,
			newline.bDataBits);
	}
	
	if (C_CRTSCTS(tty)) {
		dev_vdbg(&ch9344->data->dev, "hardware flow enable\n");
		if (ch9344->uartmode[portnum] == MODE_GPIO) {
			ret = -EINVAL;
			goto out;
		}
		ret = ch9344_set_uartmode(ch9344, portnum, portnum, MODE_HARDFLOW);
		if (!ret) {
			ch9344->uartmode[portnum] = MODE_HARDFLOW;
		}	
	} else {
		dev_vdbg(&ch9344->data->dev, "hardware flow disable\n");
		if (ch9344->uartmode[portnum] == MODE_HARDFLOW) {
			ret = ch9344_set_uartmode(ch9344, portnum, portnum, MODE_NORMAL);
			if (!ret) {
				ch9344->uartmode[portnum] = MODE_NORMAL;
			}
		}
	}

out:
    kfree(buffer);
    return;
}

static const struct tty_port_operations ch9344_port_ops = {
	.dtr_rts = ch9344_port_dtr_rts,
	.shutdown = ch9344_port_shutdown,
	.activate = ch9344_port_activate,
	.destruct = ch9344_port_destruct,
};

/*
 * USB probe and disconnect routines.
 */

/* Little helpers: write/read buffers free */
static void ch9344_write_buffers_free(struct ch9344 *ch9344)
{
	int i;
	struct ch9344_wb *wb;
	struct usb_device *usb_dev = interface_to_usbdev(ch9344->data);

	for (wb = &ch9344->wb[0], i = 0; i < CH9344_NW; i++, wb++)
		usb_free_coherent(usb_dev, ch9344->writesize, wb->buf, wb->dmah);
}

static void ch9344_read_buffers_free(struct ch9344 *ch9344)
{
	struct usb_device *usb_dev = interface_to_usbdev(ch9344->data);
	int i;

	for (i = 0; i < ch9344->rx_buflimit; i++)
		usb_free_coherent(usb_dev, ch9344->readsize,
			  ch9344->read_buffers[i].base, ch9344->read_buffers[i].dma);
}

/* Little helper: write buffers allocate */
static int ch9344_write_buffers_alloc(struct ch9344 *ch9344)
{
	int i;
	struct ch9344_wb *wb;

	for (wb = &ch9344->wb[0], i = 0; i < CH9344_NW; i++, wb++) {
		wb->buf = usb_alloc_coherent(ch9344->dev, ch9344->writesize, GFP_KERNEL,
		    &wb->dmah);
		if (!wb->buf) {
			while (i != 0) {
				--i;
				--wb;
				usb_free_coherent(ch9344->dev, ch9344->writesize,
				    wb->buf, wb->dmah);
			}
			return -ENOMEM;
		}
	}
	return 0;
}

static int ch9344_probe(struct usb_interface *intf,
		     const struct usb_device_id *id)
{
	struct usb_interface *data_interface;
	struct usb_endpoint_descriptor *epcmdread = NULL;
	struct usb_endpoint_descriptor *epcmdwrite = NULL;
	struct usb_endpoint_descriptor *epread = NULL;
	struct usb_endpoint_descriptor *epwrite = NULL;
	struct usb_device *usb_dev = interface_to_usbdev(intf);
	struct ch9344 *ch9344;
	int minor;
	int portnum;
	int cmdsize, readsize;
	u8 *buf;
	unsigned long quirks;
	int num_rx_buf = CH9344_NR;
	int i;
	struct device *tty_dev;
	int rv = -ENOMEM;

	/* normal quirks */
	quirks = (unsigned long)id->driver_info;

	/* handle quirks deadly to normal probing*/
	data_interface = usb_ifnum_to_if(usb_dev, 0);

//	if (usb_interface_claimed(data_interface)) {
		/* valid in this context */
//		dev_dbg(&intf->dev, "The data interface isn't available\n");
//		return -EBUSY;
//	}

	if (data_interface->cur_altsetting->desc.bNumEndpoints < 3 ||
	    data_interface->cur_altsetting->desc.bNumEndpoints == 0)
		return -EINVAL;

	epread = &data_interface->cur_altsetting->endpoint[0].desc;
	epwrite = &data_interface->cur_altsetting->endpoint[1].desc;
	epcmdread = &data_interface->cur_altsetting->endpoint[2].desc;
	epcmdwrite = &data_interface->cur_altsetting->endpoint[3].desc;

	/* workaround for switched endpoints */
	if (!usb_endpoint_dir_in(epread)) {
		/* descriptors are swapped */
		dev_dbg(&intf->dev,
			"The data interface has switched endpoints\n");
		swap(epread, epwrite);
	}

	if (!usb_endpoint_dir_in(epcmdread)) {
		/* descriptors are swapped */
		dev_dbg(&intf->dev,
			"The data interface has switched endpoints\n");
		swap(epcmdread, epcmdwrite);
	}

	dev_dbg(&intf->dev, "ch9344 interfaces are valid\n");

	ch9344 = kzalloc(sizeof(struct ch9344), GFP_KERNEL);
	if (ch9344 == NULL)
		goto alloc_fail;
		
    minor = ch9344_alloc_minor(ch9344);
	if (minor < 0) {
		dev_err(&intf->dev, "no more free ch9344 devices\n");
		kfree(ch9344);
		return -ENODEV;
	}

	cmdsize = usb_endpoint_maxp(epcmdread);
	readsize = usb_endpoint_maxp(epread);
	ch9344->writesize = usb_endpoint_maxp(epwrite) * 20;
	ch9344->data = data_interface;
	ch9344->minor = minor;
	ch9344->dev = usb_dev;
	ch9344->cmdsize = cmdsize;
	ch9344->readsize = readsize;
	ch9344->rx_buflimit = num_rx_buf;

	dev_dbg(&intf->dev, "epcmdread: %d, epcmdwrite: %d, epread: %d, epwrite: %d\n",
		usb_endpoint_maxp(epcmdread), usb_endpoint_maxp(epcmdwrite),
		usb_endpoint_maxp(epread), usb_endpoint_maxp(epwrite));

	INIT_WORK(&ch9344->work, ch9344_softint);
	init_waitqueue_head(&ch9344->wioctl);
	init_waitqueue_head(&ch9344->wmodemioctl);
	init_waitqueue_head(&ch9344->wgpioioctl);
	spin_lock_init(&ch9344->write_lock);
	spin_lock_init(&ch9344->read_lock);
	mutex_init(&ch9344->mutex);
	mutex_init(&ch9344->gpiomutex);
	
	ch9344->rx_endpoint = usb_rcvbulkpipe(usb_dev, epread->bEndpointAddress);
    ch9344->tx_endpoint = usb_sndbulkpipe(usb_dev, epwrite->bEndpointAddress);
    ch9344->cmdrx_endpoint = usb_rcvbulkpipe(usb_dev, epcmdread->bEndpointAddress);
    ch9344->cmdtx_endpoint = usb_sndbulkpipe(usb_dev, epcmdwrite->bEndpointAddress);

	portnum = ch9344_enum_portnum(ch9344);
	if (portnum <= 0) {
		dev_err(&intf->dev, "wrong port number\n");
		return -ENODEV;
		goto enum_fail;
	}
    ch9344->num_ports = portnum;
	
	for (i = 0; i < portnum; i++) {
	    tty_port_init(&ch9344->ttyport[i].port);
	    ch9344->ttyport[i].port.ops = &ch9344_port_ops;
	    tty_set_portdata(&ch9344->ttyport[i], ch9344);
	    ch9344->ttyport[i].portnum = i;
	    ch9344->write_empty[i] = true;
	}
	ch9344->gpio_recv = false;
	init_usb_anchor(&ch9344->delayed);
	ch9344->quirks = quirks;


	buf = usb_alloc_coherent(usb_dev, cmdsize, GFP_KERNEL, &ch9344->cmdread_dma);
	if (!buf)
		goto alloc_fail2;
	ch9344->cmdread_buffer= buf;


	if (ch9344_write_buffers_alloc(ch9344) < 0)
		goto alloc_fail4;

	ch9344->cmdreadurb= usb_alloc_urb(0, GFP_KERNEL);
	if (!ch9344->cmdreadurb)
		goto alloc_fail5;

	for (i = 0; i < num_rx_buf; i++) {
		struct ch9344_rb *rb = &(ch9344->read_buffers[i]);
		struct urb *urb;

		rb->base = usb_alloc_coherent(ch9344->dev, readsize, GFP_KERNEL,
								&rb->dma);
		if (!rb->base)
			goto alloc_fail6;
		rb->index = i;
		rb->instance = ch9344;

		urb = usb_alloc_urb(0, GFP_KERNEL);
		if (!urb)
			goto alloc_fail6;

		urb->transfer_flags |= URB_NO_TRANSFER_DMA_MAP;
		urb->transfer_dma = rb->dma;
		
		usb_fill_bulk_urb(urb, ch9344->dev,
				  ch9344->rx_endpoint,
				  rb->base,
				  ch9344->readsize,
				  ch9344_read_bulk_callback, rb);

		ch9344->read_urbs[i] = urb;
		__set_bit(i, &ch9344->read_urbs_free);
	}
	for (i = 0; i < CH9344_NW; i++) {
		struct ch9344_wb *snd = &(ch9344->wb[i]);

		snd->urb = usb_alloc_urb(0, GFP_KERNEL);
		if (snd->urb == NULL)
			goto alloc_fail7;

		usb_fill_bulk_urb(snd->urb, usb_dev,
			usb_sndbulkpipe(usb_dev, epwrite->bEndpointAddress),
			NULL, ch9344->writesize, ch9344_write_bulk, snd);
		snd->urb->transfer_flags |= URB_NO_TRANSFER_DMA_MAP;
		snd->instance = ch9344;
	}

	usb_set_intfdata(intf, ch9344);

	usb_fill_bulk_urb(ch9344->cmdreadurb, usb_dev,
			 usb_rcvbulkpipe(usb_dev, epcmdread->bEndpointAddress),
			 ch9344->cmdread_buffer, cmdsize, ch9344_cmd_irq, ch9344);
	ch9344->cmdreadurb->transfer_flags |= URB_NO_TRANSFER_DMA_MAP;
	ch9344->cmdreadurb->transfer_dma = ch9344->cmdread_dma;
			 
	dev_info(&intf->dev, "ttyWCHUSB from %d - %d: USB WCH device\n", NUMSTEP * minor,
		NUMSTEP * minor + portnum - 1);

//	ch9344->line.dwDTERate = cpu_to_le32(9600);
//	ch9344->line.bDataBits = 8;
//	ch9344_set_line(ch9344, &ch9344->line);

	usb_driver_claim_interface(&ch9344_driver, data_interface, ch9344);
	usb_set_intfdata(data_interface, ch9344);

	usb_get_intf(data_interface);

    for (i = 0; i < portnum; i++) {
	    tty_dev = tty_port_register_device(&ch9344->ttyport[i].port, ch9344_tty_driver, 
	        NUMSTEP * minor + i, &data_interface->dev);
	    if (IS_ERR(tty_dev)) {
			rv = PTR_ERR(tty_dev);
			goto alloc_fail7;
		}
    }

	if (quirks & CLEAR_HALT_CONDITIONS) {
		usb_clear_halt(usb_dev, usb_rcvbulkpipe(usb_dev, epread->bEndpointAddress));
		usb_clear_halt(usb_dev, usb_sndbulkpipe(usb_dev, epwrite->bEndpointAddress));
	}

    // deal with urb when usb plugged in
	rv = usb_submit_urb(ch9344->cmdreadurb, GFP_KERNEL);
	if (rv) {
		dev_err(&ch9344->data->dev,
			"%s - usb_submit_urb(ctrl cmd) failed\n", __func__);
		goto error_submit_urb;
	}

	rv = ch9344_submit_read_urbs(ch9344, GFP_KERNEL);
	if (rv)
		goto error_submit_read_urbs;

	dev_dbg(&intf->dev, "ch9344_probe finished!\n");

	return 0;

error_submit_read_urbs:
	for (i = 0; i < ch9344->rx_buflimit; i++)
		usb_kill_urb(ch9344->read_urbs[i]);
error_submit_urb:
	usb_kill_urb(ch9344->cmdreadurb);	
alloc_fail7:
	usb_set_intfdata(intf, NULL);
	for (i = 0; i < CH9344_NW; i++)
		usb_free_urb(ch9344->wb[i].urb);
alloc_fail6:
	for (i = 0; i < num_rx_buf; i++)
		usb_free_urb(ch9344->read_urbs[i]);
	ch9344_read_buffers_free(ch9344);
	usb_free_urb(ch9344->cmdreadurb);
alloc_fail5:
	ch9344_write_buffers_free(ch9344);
alloc_fail4:
	usb_free_coherent(usb_dev, cmdsize, ch9344->cmdread_buffer, ch9344->cmdread_dma);
enum_fail:
alloc_fail2:
	ch9344_release_minor(ch9344);
	kfree(ch9344);
alloc_fail:
	return rv;
}


static void stop_data_traffic(struct ch9344 *ch9344)
{
	int i;
	struct urb *urb;
	struct ch9344_wb *wb;

	dev_dbg(&ch9344->data->dev, "%s\n", __func__);

	usb_autopm_get_interface_no_resume(ch9344->data);
	ch9344->data->needs_remote_wakeup = 0;
	usb_autopm_put_interface(ch9344->data);

	for (;;) {
		urb = usb_get_from_anchor(&ch9344->delayed);
		if (!urb)
			break;
		wb = urb->context;
		wb->use = 0;
		usb_autopm_put_interface_async(ch9344->data);
	}

	usb_kill_urb(ch9344->cmdreadurb);
	for (i = 0; i < CH9344_NW; i++)
		usb_kill_urb(ch9344->wb[i].urb);
	for (i = 0; i < ch9344->rx_buflimit; i++)
		usb_kill_urb(ch9344->read_urbs[i]);

	cancel_work_sync(&ch9344->work);
}

static void ch9344_disconnect(struct usb_interface *intf)
{
	struct ch9344 *ch9344 = usb_get_intfdata(intf);
	struct usb_device *usb_dev = interface_to_usbdev(intf);
	struct tty_struct *tty;
	int i;

	dev_dbg(&intf->dev, "%s\n", __func__);

	/* sibling interface is already cleaning up */
	if (!ch9344)
		return;

	mutex_lock(&ch9344->mutex);
	ch9344->disconnected = true;
	wake_up_interruptible(&ch9344->wioctl);
	wake_up_interruptible(&ch9344->wmodemioctl);
	wake_up_interruptible(&ch9344->wgpioioctl);
	usb_set_intfdata(ch9344->data, NULL);
	mutex_unlock(&ch9344->mutex);

    for (i = 0; i < ch9344->num_ports; i++) {
    	tty = tty_port_tty_get(&ch9344->ttyport[i].port);
    	if (tty) {
    		tty_vhangup(tty);
    		tty_kref_put(tty);
    	}
    }

	stop_data_traffic(ch9344);

    for (i = 0; i < ch9344->num_ports; i++) {
    	tty_unregister_device(ch9344_tty_driver, NUMSTEP * ch9344->minor + i);
    }

	usb_free_urb(ch9344->cmdreadurb);
	for (i = 0; i < CH9344_NW; i++)
		usb_free_urb(ch9344->wb[i].urb);
	for (i = 0; i < ch9344->rx_buflimit; i++)
		usb_free_urb(ch9344->read_urbs[i]);
	ch9344_write_buffers_free(ch9344);
	usb_free_coherent(usb_dev, ch9344->cmdsize, ch9344->cmdread_buffer, ch9344->cmdread_dma);
	ch9344_read_buffers_free(ch9344);

	usb_driver_release_interface(&ch9344_driver, ch9344->data);

    for (i = 0; i < ch9344->num_ports; i++)
	    tty_port_put(&ch9344->ttyport[i].port);

	dev_info(&intf->dev, "%s\n", "ch9344 usb device disconnect.");
}

#ifdef CONFIG_PM
static int ch9344_suspend(struct usb_interface *intf, pm_message_t message)
{
	struct ch9344 *ch9344 = usb_get_intfdata(intf);
	int cnt;

	spin_lock_irq(&ch9344->write_lock);
	if (PMSG_IS_AUTO(message)) {
		if (ch9344->transmitting) {
			spin_unlock_irq(&ch9344->write_lock);
			return -EBUSY;
		}
	}
	cnt = ch9344->susp_count++;
	spin_unlock_irq(&ch9344->write_lock);

	if (cnt)
		return 0;

	stop_data_traffic(ch9344);

	return 0;
}

static int ch9344_resume(struct usb_interface *intf)
{
	struct ch9344 *ch9344 = usb_get_intfdata(intf);
	struct urb *urb;
	int rv = 0;

	spin_lock_irq(&ch9344->write_lock);

	if (--ch9344->susp_count)
		goto out;

	if (test_bit(ASYNCB_INITIALIZED, &ch9344->ttyport[0].port.flags)) {
		rv = usb_submit_urb(ch9344->cmdreadurb, GFP_ATOMIC);

		for (;;) {
			urb = usb_get_from_anchor(&ch9344->delayed);
			if (!urb)
				break;

			ch9344_start_wb(ch9344, urb->context);
		}

		/*
		 * delayed error checking because we must
		 * do the write path at all cost
		 */
		if (rv < 0)
			goto out;

		rv = ch9344_submit_read_urbs(ch9344, GFP_ATOMIC);
	}
out:
	spin_unlock_irq(&ch9344->write_lock);

	return rv;
}

static int ch9344_reset_resume(struct usb_interface *intf)
{
	struct ch9344 *ch9344 = usb_get_intfdata(intf);

	if (test_bit(ASYNCB_INITIALIZED, &ch9344->ttyport[0].port.flags))
		tty_port_tty_hangup(&ch9344->ttyport[0].port, false);

	return ch9344_resume(intf);
}

#endif /* CONFIG_PM */

/*
 * USB driver structure.
 */

static const struct usb_device_id ch9344_ids[] = {
	/* quirky and broken devices */
	{ USB_DEVICE(0x1a86, 0xe018), /* ch9344 chip */
	.driver_info = CLEAR_HALT_CONDITIONS, },/* has no union descriptor */

	{ }
};

MODULE_DEVICE_TABLE(usb, ch9344_ids);

static struct usb_driver ch9344_driver = {
	.name =		"usb_wch",
	.probe =	ch9344_probe,
	.disconnect =	ch9344_disconnect,
#ifdef CONFIG_PM
//	.suspend =	ch9344_suspend,
//	.resume =	ch9344_resume,
//	.reset_resume =	ch9344_reset_resume,
#endif
	.id_table =	ch9344_ids,
#ifdef CONFIG_PM
	.supports_autosuspend = 1,
#endif
	.disable_hub_initiated_lpm = 1,
};

/*
 * TTY driver structures.
 */

static const struct tty_operations ch9344_ops = {
	.install =		ch9344_tty_install,
	.open =			ch9344_tty_open,
	.close =		ch9344_tty_close,
	.cleanup =		ch9344_tty_cleanup,
	.hangup =		ch9344_tty_hangup,
	.write =		ch9344_tty_write,
	.write_room =		ch9344_tty_write_room,
	.ioctl = 		ch9344_tty_ioctl,
	.chars_in_buffer =	ch9344_tty_chars_in_buffer,
	.set_termios =		ch9344_tty_set_termios,
	.tiocmget = ch9344_tty_tiocmget,
	.tiocmset = ch9344_tty_tiocmset,
};

/*
 * Init / exit.
 */

static int __init ch9344_init(void)
{
	int retval;

	ch9344_tty_driver = alloc_tty_driver(CH9344_TTY_MINORS);
	if (!ch9344_tty_driver)
		return -ENOMEM;
	ch9344_tty_driver->driver_name = "usbch9344",
	ch9344_tty_driver->name = "ttyWCHUSB",
	ch9344_tty_driver->major = CH9344_TTY_MAJOR,
	ch9344_tty_driver->minor_start = 0,
	ch9344_tty_driver->type = TTY_DRIVER_TYPE_SERIAL,
	ch9344_tty_driver->subtype = SERIAL_TYPE_NORMAL,
	ch9344_tty_driver->flags = TTY_DRIVER_REAL_RAW | TTY_DRIVER_DYNAMIC_DEV;
	ch9344_tty_driver->init_termios = tty_std_termios;
	ch9344_tty_driver->init_termios.c_cflag = B9600 | CS8 | CREAD |
								HUPCL | CLOCAL;
	tty_set_operations(ch9344_tty_driver, &ch9344_ops);

	retval = tty_register_driver(ch9344_tty_driver);
	if (retval) {
		put_tty_driver(ch9344_tty_driver);
		return retval;
	}

	retval = usb_register(&ch9344_driver);
	if (retval) {
		tty_unregister_driver(ch9344_tty_driver);
		put_tty_driver(ch9344_tty_driver);
		return retval;
	}
	printk(KERN_INFO KBUILD_MODNAME ": " DRIVER_DESC "\n");
	printk(KERN_INFO KBUILD_MODNAME ": " VERSION_DESC "\n");

	return 0;
}

static void __exit ch9344_exit(void)
{
	usb_deregister(&ch9344_driver);
	tty_unregister_driver(ch9344_tty_driver);
	put_tty_driver(ch9344_tty_driver);
	idr_destroy(&ch9344_minors);
	printk(KERN_INFO KBUILD_MODNAME ": " "ch9344 driver exit.\n");
}

module_init(ch9344_init);
module_exit(ch9344_exit);

MODULE_AUTHOR(DRIVER_AUTHOR);
MODULE_DESCRIPTION(DRIVER_DESC);
MODULE_LICENSE("GPL");
MODULE_ALIAS_CHARDEV_MAJOR(CH9344_TTY_MAJOR);


