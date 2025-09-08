// SPDX-License-Identifier: GPL-2.0+
/*
 * USB serial driver for USB to Quad UARTs chip ch9344 and
 * USB to Octal UARTs chip ch348.
 *
 * Copyright (C) 2025 Nanjing Qinheng Microelectronics Co., Ltd.
 * Web: http://wch.cn
 * Author: WCH <tech@wch.cn>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * System required:
 * Kernel version beyond 3.2.x
 * Update Log:
 * V1.0 - initial version
 * V1.1 - add support for high baudrates 2M, 4M and 6M
 * V1.2 - add support for modem signal, flow control
 * V1.3 - add long packet for tty_write
 * V1.4 - add ioctl for rs485 mode
 * V1.5 - fix baud rate 0 bugs
 * V1.6 - fix modem out bugs, add gpio function,
 *      - remove rs485 control(hardware auto support)
 *      - add support for baudrates 250k, 500k, 1M, 1.5M, 3M, 12M
 * V1.7 - add support for break function
 * V1.8 - add support for ch348
 * V1.9 - fix os suspend/hibernate issues, add gpio character device
 *      - add support for kernel version beyond 5.14.x
 * V2.0 - add support for get_icount and ioctl operation
 *      - add support for kernel version beyond 6.3.x
 * V2.1 - remove put_char and flush_chars methods of tty_operations
 *      - add support for kernel version beyond 6.5.x
 *      - add support for ch9344q
 *      - update modem status when uart open
 * V2.2 - add support for independent upload on multiple serial ports
 * V2.3 - set default termios baudrate to B115200
 *      - fix issue of automatically sending data upon opening tty
 *        for the first time on some systems
 */

#define DEBUG
#define VERBOSE_DEBUG

#undef DEBUG
#undef VERBOSE_DEBUG

#include <linux/errno.h>
#include <linux/idr.h>
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/list.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/random.h>
#include <linux/serial.h>
#include <linux/slab.h>
#include <linux/tty.h>
#include <linux/tty_driver.h>
#include <linux/tty_flip.h>
#include <linux/uaccess.h>
#include <linux/usb.h>
#include <linux/version.h>
#include <linux/random.h>
#include <linux/timer.h>
#include <linux/kfifo.h>
#include <asm/byteorder.h>
#if (LINUX_VERSION_CODE >= KERNEL_VERSION(6, 12, 0))
#include <linux/unaligned.h>
#else
#include <asm/unaligned.h>
#endif

#if (LINUX_VERSION_CODE >= KERNEL_VERSION(4, 11, 0))
#include <linux/sched/signal.h>
#endif

#include "ch9344.h"

#define DRIVER_AUTHOR "WCH"
#define DRIVER_DESC "USB serial driver for ch9344/ch348."
#define VERSION_DESC "V2.3 On 2025.07"

#define IOCTL_MAGIC 'W'
#define IOCTL_CMD_GPIOENABLE _IOW(IOCTL_MAGIC, 0x80, u16)
#define IOCTL_CMD_GPIODIR _IOW(IOCTL_MAGIC, 0x81, u16)
#define IOCTL_CMD_GPIOSET _IOW(IOCTL_MAGIC, 0x82, u16)
#define IOCTL_CMD_GPIOGET _IOWR(IOCTL_MAGIC, 0x83, u16)
#define IOCTL_CMD_GETCHIPTYPE _IOR(IOCTL_MAGIC, 0x84, u16)
#define IOCTL_CMD_GETUARTINDEX _IOR(IOCTL_MAGIC, 0x85, u16)
#define IOCTL_CMD_CTRLIN _IOWR(IOCTL_MAGIC, 0x90, u16)
#define IOCTL_CMD_CTRLOUT _IOW(IOCTL_MAGIC, 0x91, u16)
#define IOCTL_CMD_CMDIN _IOWR(IOCTL_MAGIC, 0x92, u16)
#define IOCTL_CMD_CMDOUT _IOW(IOCTL_MAGIC, 0x93, u16)

#define PACKLOAD
#undef PACKLOAD

static struct usb_driver ch9344_driver;
static struct tty_driver *ch9344_tty_driver;

static DEFINE_MUTEX(ch9344_minors_lock);

#if (LINUX_VERSION_CODE >= KERNEL_VERSION(3, 5, 0))
static DEFINE_IDR(ch9344_minors);
#else
static struct ch9344 *ch9344_table[CH9344_TTY_MINORS];
#endif

#if (LINUX_VERSION_CODE >= KERNEL_VERSION(6, 1, 0))
static void ch9344_tty_set_termios(struct tty_struct *tty,
				   const struct ktermios *termios_old);
#else
static void ch9344_tty_set_termios(struct tty_struct *tty,
				   struct ktermios *termios_old);
#endif

static int ch9344_get_portnum(int index);

static int ch9344_set_uartmode(struct ch9344 *ch9344, int portnum,
			       u8 index, u8 mode);

static int ch9344_cmd_out(struct ch9344 *ch9344, u8 *buf, int count);

#if (LINUX_VERSION_CODE >= KERNEL_VERSION(3, 5, 0))

/*
 * Look up an ch9344 structure by minor. If found and not disconnected,
 * increment its refcount and return it with its mutex held.
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
			tty_port_get(
				&ch9344->ttyport[ch9344_get_portnum(index)]
					 .port);
			mutex_unlock(&ch9344->mutex);
		}
	}
	mutex_unlock(&ch9344_minors_lock);

	return ch9344;
}

/*
 * Try to find an available minor number and if found,
 * associate it with 'ch9344'.
 */
static int ch9344_alloc_minor(struct ch9344 *ch9344)
{
	int minor;

	mutex_lock(&ch9344_minors_lock);
	minor = idr_alloc(&ch9344_minors, ch9344, 0, CH9344_TTY_MINORS,
			  GFP_KERNEL);
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

#else

/*
 * Look up an ch9344 structure by index. If found and not disconnected,
 * increment its refcount and return it with its mutex held.
 */
static struct ch9344 *ch9344_get_by_index(unsigned int index)
{
	struct ch9344 *ch9344;

	mutex_lock(&ch9344_minors_lock);
	ch9344 = ch9344_table[index / NUMSTEP];
	if (ch9344) {
		mutex_lock(&ch9344->mutex);
		if (ch9344->disconnected) {
			mutex_unlock(&ch9344->mutex);
			ch9344 = NULL;
		} else {
			tty_port_get(
				&ch9344->ttyport[ch9344_get_portnum(index)]
					 .port);
			mutex_unlock(&ch9344->mutex);
		}
	}
	mutex_unlock(&ch9344_minors_lock);

	return ch9344;
}

/*
 * Try to find an available minor number and if found,
 * associate it with 'ch9344'.
 */
static int ch9344_alloc_minor(struct ch9344 *ch9344)
{
	int minor;

	mutex_lock(&ch9344_minors_lock);
	for (minor = 0; minor < CH9344_TTY_MINORS; minor++) {
		if (!ch9344_table[minor]) {
			ch9344_table[minor] = ch9344;
			break;
		}
	}
	mutex_unlock(&ch9344_minors_lock);

	return minor;
}

/* Release the minor number associated with 'ch9344'. */
static void ch9344_release_minor(struct ch9344 *ch9344)
{
	mutex_lock(&ch9344_minors_lock);
	ch9344_table[ch9344->minor] = NULL;
	mutex_unlock(&ch9344_minors_lock);
}

#endif

static int ch9344_get_portnum(int index)
{
	return index % NUMSTEP;
}

/*
 * Functions for control messages.
 */
static int ch9344_control_out(struct ch9344 *ch9344, u8 request,
			      u8 requesttype, u16 value, u16 index,
			      void *buf, unsigned int bufsize)
{
	int retval;
	char *buffer;

	buffer = kmalloc(bufsize, GFP_KERNEL);
	if (!buffer)
		return -ENOMEM;

	retval = copy_from_user(buffer, (char __user *)buf, bufsize);
	if (retval)
		goto out;

	retval = usb_autopm_get_interface(ch9344->data);
	if (retval)
		goto out;
	retval = usb_control_msg(ch9344->dev,
				 usb_sndctrlpipe(ch9344->dev, 0), request,
				 requesttype, value, index, buffer,
				 bufsize, DEFAULT_TIMEOUT);
	usb_autopm_put_interface(ch9344->data);

out:
	kfree(buffer);
	return retval;
}

static int ch9344_control_in(struct ch9344 *ch9344, u8 request,
			     u8 requesttype, u16 value, u16 index,
			     char *buf, unsigned int bufsize)
{
	int retval = 0;
	char *buffer;

	buffer = kmalloc(bufsize, GFP_KERNEL);
	if (!buffer)
		return -ENOMEM;

	retval = usb_autopm_get_interface(ch9344->data);
	if (retval)
		goto out;
	retval = usb_control_msg(ch9344->dev,
				 usb_rcvctrlpipe(ch9344->dev, 0), request,
				 requesttype, value, index, buf, bufsize,
				 DEFAULT_TIMEOUT);

	usb_autopm_put_interface(ch9344->data);

out:
	kfree(buffer);
	return retval;
}

/*
 * Functions for ch9344 cmd messages.
 */
static int ch9344_cmd_in(struct ch9344 *ch9344, void *sbuf, int count,
			 void *rbuf)
{
	u8 *buffer;
	int ret;

	buffer = kzalloc(count, GFP_KERNEL);
	if (!buffer)
		return -ENOMEM;

	ch9344->cfg_recv = false;

	ret = copy_from_user(buffer, (char __user *)sbuf, count);
	if (ret)
		goto out;

	ret = ch9344_cmd_out(ch9344, buffer, count);
	if (ret < 0)
		goto out;

	ret = wait_event_interruptible_timeout(
		ch9344->wcfgioctl, ch9344->cfg_recv,
		msecs_to_jiffies(DEFAULT_TIMEOUT));
	if (ret == 0) {
		ret = -ETIMEDOUT;
		goto out;
	}

	if (ret < 0)
		goto out;

	ret = ch9344->cfgindex;

	ch9344->cfgindex = 0;

	if (copy_to_user((char __user *)rbuf, ch9344->cfgval, ret)) {
		ret = -EFAULT;
		goto out;
	}

out:
	kfree(buffer);
	return ret;
}

static int ch9344_cmd_out(struct ch9344 *ch9344, u8 *buf, int count)
{
	int retval = 0, actual_len = 0;

	retval = usb_autopm_get_interface(ch9344->data);
	if (retval)
		goto out;

	retval = usb_bulk_msg(ch9344->dev, ch9344->cmdtx_endpoint, buf,
			      count, &actual_len, DEFAULT_TIMEOUT);
	usb_autopm_put_interface(ch9344->data);
	if (retval) {
		dev_err(&ch9344->data->dev,
			"usb_bulk_msg(send) failed, err %i\n", retval);
		goto out;
	}

	if (actual_len != count) {
		dev_err(&ch9344->data->dev, "only wrote %d of %d bytes\n",
			actual_len, count);
		retval = -EPIPE;
		goto out;
	}

out:
	return retval;
}

static inline int ch9344_set_control(struct ch9344 *ch9344, int portnum,
				     int control)
{
	char *buffer;
	int retval;
	const unsigned int size = 3;
	u8 rgadd = 0;

	if (ch9344->chiptype == CHIP_CH9344L ||
	    ch9344->chiptype == CHIP_CH9344Q)
		rgadd = 0x10 * portnum + 0x08;
	else if (ch9344->chiptype == CHIP_CH348L ||
		 ch9344->chiptype == CHIP_CH348Q) {
		if (portnum < 4)
			rgadd = 0x10 * portnum;
		else
			rgadd = 0x10 * (portnum - 4) + 0x08;
	}

	buffer = kzalloc(size, GFP_KERNEL);
	if (!buffer)
		return -ENOMEM;

	if (ch9344->quirks & QUIRK_CONTROL_LINE_STATE)
		return -EOPNOTSUPP;

	buffer[0] = CMD_W_BR;
	buffer[1] = rgadd + R_C4;
	buffer[2] = control;

	retval = ch9344_cmd_out(ch9344, buffer, size);

	kfree(buffer);

	return retval < 0 ? retval : 0;
}

static inline int ch9344_enum_chiptype(struct ch9344 *ch9344)
{
	unsigned char *buffer;
	int retval;
	const unsigned int size = 4;

	buffer = kzalloc(size, GFP_KERNEL);
	if (!buffer)
		return -ENOMEM;

	retval = ch9344_control_in(ch9344, CMD_VER,
				   USB_TYPE_VENDOR | USB_RECIP_DEVICE |
					   USB_DIR_IN,
				   0x00, 0x00, buffer, size);
	if (retval < 0)
		goto out;

	if (retval == 4) {
		if (ch9344->idProduct == 0xe018) {
			if (buffer[0] >= 0x40)
				ch9344->chiptype = CHIP_CH9344Q;
			else
				ch9344->chiptype = CHIP_CH9344L;
		} else {
			if (buffer[1] & (0x02 << 6))
				ch9344->chiptype = CHIP_CH348Q;
			else
				ch9344->chiptype = CHIP_CH348L;
		}
		ch9344->chipver = buffer[0];
	} else
		retval = -EPROTO;

out:
	kfree(buffer);
	return retval < 0 ? retval : 0;
}

static int ch9344_configure(struct ch9344 *ch9344, int portnum)
{
	char *buffer;
	int ret;
	u8 request;
	u8 rgadd = 0;

	buffer = kzalloc(ch9344->cmdsize, GFP_KERNEL);
	if (!buffer)
		return -ENOMEM;

	request = CMD_W_R;
	if (ch9344->chiptype == CHIP_CH9344L ||
	    ch9344->chiptype == CHIP_CH9344Q)
		rgadd = 0x10 * portnum + 0x08;
	else if (ch9344->chiptype == CHIP_CH348L ||
		 ch9344->chiptype == CHIP_CH348Q) {
		if (portnum < 4)
			rgadd = 0x10 * portnum;
		else
			rgadd = 0x10 * (portnum - 4) + 0x08;
	}

	buffer[0] = request;
	buffer[1] = rgadd + R_C2;
	buffer[2] = 0x87;
	ret = ch9344_cmd_out(ch9344, buffer, 0x03);
	if (ret < 0)
		goto out;

	if (ch9344->chiptype == CHIP_CH9344L ||
	    ch9344->chiptype == CHIP_CH9344Q) {
		buffer[1] = rgadd + R_C3;
		buffer[2] = 0x03;
		ret = ch9344_cmd_out(ch9344, buffer, 0x03);
		if (ret < 0)
			goto out;
	}

	buffer[1] = rgadd + R_C4;
	buffer[2] = 0x08;
	ret = ch9344_cmd_out(ch9344, buffer, 0x03);
	if (ret < 0)
		goto out;

out:
	kfree(buffer);
	return ret < 0 ? ret : 0;
}

/*
 * Write buffer management.
 * All of these assume proper locks taken by the caller.
 */
static int ch9344_wb_alloc(struct ch9344 *ch9344, int portnum)
{
	int i, wbn;
	struct ch9344_wb *wb;

	wbn = 0;
	i = 0;
	for (;;) {
		wb = &ch9344->wb[portnum][wbn];
		if (!wb->use) {
			wb->use = 1;
			return wbn;
		}
		wbn = (wbn + 1) % CH9344_NW;
		if (++i >= CH9344_NW)
			return -1;
	}
}

static int ch9344_wb_is_avail(struct ch9344 *ch9344, int portnum)
{
	int i, n;
	unsigned long flags;

	n = CH9344_NW;
	spin_lock_irqsave(&ch9344->write_lock, flags);
	for (i = 0; i < CH9344_NW; i++)
		n -= ch9344->wb[portnum][i].use;
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
			"%s - usb_submit_urb(write) failed: %d\n",
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
	u8 reg_iir, rgadd;
	int ctrlval;
	int newctrl;
	int difference;
	unsigned long flags;
	int left = 0, right = 0;
	u64 gpiovalins;
#if (LINUX_VERSION_CODE < KERNEL_VERSION(3, 10, 0))
	struct tty_struct *tty;
#endif

	if (ch9344->chiptype == CHIP_CH9344L ||
	    ch9344->chiptype == CHIP_CH9344Q) {
		left = 4;
		right = 8;
	} else if (ch9344->chiptype == CHIP_CH348L ||
		   ch9344->chiptype == CHIP_CH348Q) {
		left = 0;
		right = 8;
	}

	switch (status) {
	case 0:
		/* success */
		for (i = 0; i < len;) {
			reg_iir = *(data + i + 1);
			portnum = *(data + i) & 0x0f;

			if (ch9344->chiptype == CHIP_CH9344L ||
			    ch9344->chiptype == CHIP_CH9344Q)
				rgadd = 0x10 * portnum + 0x08;
			else {
				if (portnum < 4)
					rgadd = 0x10 * portnum;
				else
					rgadd = 0x10 * (portnum - 4) +
						0x08;
			}

			if (reg_iir == R_INIT) {
				i += 12;
				continue;
			} else if (reg_iir >= R_MOD && reg_iir <= R_TM_O) {
				if ((portnum >= left) &&
				    (portnum < right)) {
					portnum -= ch9344->port_offset;
					if (reg_iir == R_IO_I) {
						spin_lock_irqsave(
							&ch9344->write_lock,
							flags);
						ch9344->gpio_recv = true;
						ch9344->gpiovalin =
							(*(data + i + 3)
							 << 8) +
							*(data + i + 2);
						wake_up_interruptible(
							&ch9344->wgpioioctl);
						spin_unlock_irqrestore(
							&ch9344->write_lock,
							flags);
					}
				} else
					break;
				i += 4;
				continue;
			} else if (reg_iir >= R_IO_CE &&
				   reg_iir <= R_IO_CI) {
				if (reg_iir == R_IO_CI) {
					spin_lock_irqsave(
						&ch9344->write_lock,
						flags);
					ch9344->gpio_recv = true;
					memcpy(&gpiovalins, data + 2,
					       0x08);
					ch9344->gpiovalins =
						le64_to_cpu(gpiovalins);
					wake_up_interruptible(
						&ch9344->wgpioioctl);
					spin_unlock_irqrestore(
						&ch9344->write_lock,
						flags);
					i += 10;
				} else
					i += 3;
				continue;
			} else if ((reg_iir & 0x0f) == R_II_B2) {
				if ((portnum >= left) &&
				    (portnum < right)) {
					portnum -= ch9344->port_offset;
					spin_lock_irqsave(
						&ch9344->write_lock,
						flags);
					ch9344->ttyport[portnum]
						.write_empty = true;
					wake_up_interruptible(
						&ch9344->ttyport[portnum]
							 .wioctl);
					spin_unlock_irqrestore(
						&ch9344->write_lock,
						flags);
				} else
					break;
			} else if (((reg_iir & 0x0f) == R_II_B3) ||
				   ((reg_iir == VEN_R) &&
				    (*(data + i + 2) == (rgadd | 0x06)))) {
				if ((portnum >= left) &&
				    (portnum < right)) {
					portnum -= ch9344->port_offset;

					spin_lock(&ch9344->read_lock);
					newctrl = ch9344->ttyport[portnum]
							  .ctrlin;
					/* modem signal */
					if ((reg_iir & 0x0f) == R_II_B3)
						ctrlval = *(data + i + 2);
					else
						ctrlval = *(data + i + 3) |
							  0x0f;
					if (ctrlval & CH9344_CTI_C) {
						if (ctrlval &
						    (CH9344_CTI_C << 4))
							newctrl |=
								CH9344_CTI_C;
						else
							newctrl &=
								~CH9344_CTI_C;
					}
					if (ctrlval & CH9344_CTI_DS) {
						if (ctrlval &
						    (CH9344_CTI_DS << 4))
							newctrl |=
								CH9344_CTI_DS;
						else
							newctrl &=
								~CH9344_CTI_DS;
					}
					if (ctrlval & CH9344_CTI_R) {
						if (ctrlval &
						    (CH9344_CTI_R << 4))
							newctrl |=
								CH9344_CTI_R;
						else
							newctrl &=
								~CH9344_CTI_R;
					}
					if (ctrlval & CH9344_CTI_DC) {
						if (ctrlval &
						    (CH9344_CTI_DC << 4))
							newctrl |=
								CH9344_CTI_DC;
						else
							newctrl &=
								~CH9344_CTI_DC;
					}

					if (!ch9344->ttyport[portnum]
						     .clocal &&
					    (ch9344->ttyport[portnum]
						     .ctrlin &
					     ~newctrl & CH9344_CTI_DC)) {
						spin_unlock(
							&ch9344->read_lock);
#if (LINUX_VERSION_CODE < KERNEL_VERSION(3, 10, 0))
						tty = tty_port_tty_get(
							&ch9344->ttyport[portnum]
								 .port);
						tty_hangup(tty);
						tty_kref_put(tty);
#else
						tty_port_tty_hangup(
							&ch9344->ttyport[portnum]
								 .port,
							false);
#endif
						spin_lock(
							&ch9344->read_lock);
					}

					difference =
						ch9344->ttyport[portnum]
							.ctrlin ^
						newctrl;
					ch9344->ttyport[portnum].ctrlin =
						newctrl;
					ch9344->ttyport[portnum].oldcount =
						ch9344->ttyport[portnum]
							.iocount;

					if (difference & CH9344_CTI_C)
						ch9344->ttyport[portnum]
							.iocount.cts++;
					if (difference & CH9344_CTI_DS)
						ch9344->ttyport[portnum]
							.iocount.dsr++;
					if (difference & CH9344_CTI_R)
						ch9344->ttyport[portnum]
							.iocount.rng++;
					if (difference & CH9344_CTI_DC)
						ch9344->ttyport[portnum]
							.iocount.dcd++;
					spin_unlock(&ch9344->read_lock);

					if (difference)
						wake_up_interruptible(
							&ch9344->ttyport[portnum]
								 .wmodemioctl);
				} else
					break;
				if (reg_iir == VEN_R)
					i += 4;
				else
					i += 3;
				continue;
			} else if ((reg_iir & 0x0f) == R_II_B1) {
				if ((portnum >= left) &&
				    (portnum < right)) {
					portnum -= ch9344->port_offset;

					spin_lock(&ch9344->read_lock);
					/* lsr signal */
					ctrlval = *(data + i + 2);
					ch9344->ttyport[portnum].oldcount =
						ch9344->ttyport[portnum]
							.iocount;
					if (ctrlval & (CH9344_LO >> 3)) {
						ch9344->ttyport[portnum]
							.iocount.overrun++;
					}
					if (ctrlval & (CH9344_LP >> 3)) {
						ch9344->ttyport[portnum]
							.iocount.parity++;
					}
					if (ctrlval & (CH9344_LF >> 3)) {
						ch9344->ttyport[portnum]
							.iocount.frame++;
					}
					if (ctrlval & (CH9344_LB >> 3)) {
						ch9344->ttyport[portnum]
							.iocount.brk++;
					}
					spin_unlock(&ch9344->read_lock);
				} else
					break;
				i += 3;
				continue;
			} else if (reg_iir == R_EE_CFG) {
				if (*(data + i) == CMD_W_R) {
					spin_lock_irqsave(
						&ch9344->write_lock,
						flags);
					if (ch9344->cfgindex + 0x04 >
					    CFGLEN)
						ch9344->cfgindex = 0;
					memcpy(ch9344->cfgval +
						       ch9344->cfgindex,
					       data + i, 0x04);
					ch9344->cfgindex += 0x04;
					ch9344->cfg_recv = true;
					wake_up_interruptible(
						&ch9344->wcfgioctl);
					spin_unlock_irqrestore(
						&ch9344->write_lock,
						flags);
				} else
					break;
				i += 4;
				continue;
			} else if (reg_iir == R_UP_O) {
				i += 4;
				continue;
			} else {
				dev_dbg(&ch9344->data->dev,
					"%s - wrong status received",
					__func__);
				break;
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
			"%s - nonzero urb status received: %d\n", __func__,
			status);
		return;
	}

	usb_mark_last_busy(ch9344->dev);

	retval = usb_submit_urb(urb, GFP_ATOMIC);
	if (retval && retval != -EPERM)
		dev_err(&ch9344->data->dev,
			"%s - usb_submit_urb failed: %d\n", __func__,
			retval);
}

static int ch9344_submit_read_urb(struct ch9344 *ch9344, int index,
				  gfp_t mem_flags)
{
	int res;

	if (!test_and_clear_bit(index, &ch9344->read_urbs_free))
		return 0;

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

#ifdef PACKLOAD
#if (LINUX_VERSION_CODE < KERNEL_VERSION(4, 14, 0))
static void timer_function(unsigned long arg)
{
	unsigned char *buffer;
	struct ch9344_ttyport *ttyport = (struct ch9344_ttyport *)arg;
	int fifolen = kfifo_len(&ttyport->rfifo);
	int len;

	buffer = kmalloc(FIFOSIZE, GFP_KERNEL);
	len = kfifo_out(&ttyport->rfifo, buffer, fifolen);
	ttyport->iocount.rx += len;
	tty_insert_flip_string(&ttyport->port, buffer, len);
	tty_flip_buffer_push(&ttyport->port);
	kfree(buffer);
}
#else
static void timer_function(struct timer_list *t)
{
	unsigned char *buffer;
	struct ch9344_ttyport *ttyport = from_timer(ttyport, t, timer);
	int fifolen = kfifo_len(&ttyport->rfifo);
	int len;

	buffer = kmalloc(FIFOSIZE, GFP_KERNEL);
	len = kfifo_out(&ttyport->rfifo, buffer, fifolen);
	ttyport->iocount.rx += len;
	tty_insert_flip_string(&ttyport->port, buffer, len);
	tty_flip_buffer_push(&ttyport->port);
	kfree(buffer);
}
#endif
#endif

static void ch9344_process_read_urb(struct ch9344 *ch9344, struct urb *urb)
{
	int size;
	char buffer[512];
	int i = 0;
	int portnum;
	u8 usblen;
	int left = 0, right = 0;
#ifndef PACKLOAD
	bool upflag[MAXPORT];
#endif
#if (LINUX_VERSION_CODE < KERNEL_VERSION(3, 9, 0))
	struct tty_struct *tty;
#endif

	if (ch9344->chiptype == CHIP_CH9344L ||
	    ch9344->chiptype == CHIP_CH9344Q) {
		left = 4;
		right = 8;
	} else if (ch9344->chiptype == CHIP_CH348L ||
		   ch9344->chiptype == CHIP_CH348Q) {
		left = 0;
		right = 8;
	}

	if (!urb->actual_length)
		return;
	size = urb->actual_length;

	memcpy(buffer, urb->transfer_buffer, urb->actual_length);

	for (i = 0; i < size; i += 32) {
		portnum = *(buffer + i);
		if (portnum < left || portnum >= right) {
			break;
		}
		portnum -= ch9344->port_offset;
		usblen = *(buffer + i + 1);
		if (usblen > 30) {
			break;
		}

		if (ch9344->ttyport[portnum].isopen) {
#ifndef PACKLOAD
			ch9344->ttyport[portnum].iocount.rx += usblen;
			upflag[portnum] = true;
#if (LINUX_VERSION_CODE < KERNEL_VERSION(3, 9, 0))
			tty = tty_port_tty_get(
				&ch9344->ttyport[portnum].port);
			if (tty) {
				tty_insert_flip_string(tty, buffer + i + 2,
						       usblen);
				tty_kref_put(tty);
			}
#else
			tty_insert_flip_string(
				&ch9344->ttyport[portnum].port,
				buffer + i + 2, usblen);
#endif
#else
			kfifo_in(&ch9344->ttyport[portnum].rfifo,
				 buffer + i + 2, usblen);
			mod_timer(
				&ch9344->ttyport[portnum].timer,
				jiffies +
					ch9344->ttyport[portnum].interval);
#endif
		}
	}
#ifndef PACKLOAD
	for (i = 0; i < ch9344->num_ports; i++) {
		if (upflag[i]) {
#if (LINUX_VERSION_CODE < KERNEL_VERSION(3, 9, 0))
			tty = tty_port_tty_get(&ch9344->ttyport[i].port);
			if (tty) {
				tty_flip_buffer_push(tty);
				tty_kref_put(tty);
			}
#else
			tty_flip_buffer_push(&ch9344->ttyport[i].port);
#endif
		}
	}
#endif
}

static void ch9344_read_bulk_callback(struct urb *urb)
{
	struct ch9344_rb *rb = urb->context;
	struct ch9344 *ch9344 = rb->instance;
	int status = urb->status;

	if (!ch9344->dev) {
		set_bit(rb->index, &ch9344->read_urbs_free);
		return;
	}

	if (status) {
		set_bit(rb->index, &ch9344->read_urbs_free);
		dev_dbg(&ch9344->data->dev,
			"%s - non-zero urb status: %d\n", __func__,
			status);
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

	spin_lock_irqsave(&ch9344->write_lock, flags);
	ch9344_write_done(ch9344, wb);
	spin_unlock_irqrestore(&ch9344->write_lock, flags);
	schedule_work(&ch9344->ttyport[wb->portnum].work);
}

static void ch9344_softint(struct work_struct *work)
{
	struct ch9344_ttyport *ttyport =
		container_of(work, struct ch9344_ttyport, work);
#if (LINUX_VERSION_CODE < KERNEL_VERSION(3, 10, 0))
	struct tty_struct *tty;

	tty = tty_port_tty_get(&ttyport->port);
	if (!tty)
		return;
	tty_wakeup(tty);
	tty_kref_put(tty);
#else
	tty_port_tty_wakeup(&ttyport->port);
#endif
}

/*
 * TTY handlers
 */
static int ch9344_tty_install(struct tty_driver *driver,
			      struct tty_struct *tty)
{
	struct ch9344 *ch9344;
	int retval;

	ch9344 = ch9344_get_by_index(tty->index);
	if (!ch9344)
		return -ENODEV;

#if (LINUX_VERSION_CODE >= KERNEL_VERSION(3, 5, 0))
	retval = tty_standard_install(driver, tty);
	if (retval)
		goto error_init_termios;

	tty->driver_data = ch9344;
#else
	retval = tty_init_termios(tty);
	if (retval)
		goto error_init_termios;

	tty->driver_data = ch9344;

	/* Final install (we use the default method) */
	tty_driver_kref_get(driver);
	tty->count++;
	driver->ttys[tty->index] = tty;
#endif

	return 0;

error_init_termios:
	tty_port_put(
		&ch9344->ttyport[ch9344_get_portnum(tty->index)].port);
	return retval;
}

static int ch9344_tty_open(struct tty_struct *tty, struct file *filp)
{
	struct ch9344 *ch9344 = tty->driver_data;
	int rv;

	rv = tty_port_open(
		&ch9344->ttyport[ch9344_get_portnum(tty->index)].port, tty,
		filp);
	if (!rv) {
		ch9344->ttyport[ch9344_get_portnum(tty->index)]
			.write_empty = true;
	}

	return rv;
}

static inline void tty_set_portdata(struct ch9344_ttyport *port,
				    void *data)
{
	port->portdata = data;
}

static inline void *tty_get_portdata(struct ch9344_ttyport *port)
{
	return (port->portdata);
}

#if (LINUX_VERSION_CODE >= KERNEL_VERSION(6, 3, 0))
static void ch9344_port_dtr_rts(struct tty_port *port, bool raise)
#else
static void ch9344_port_dtr_rts(struct tty_port *port, int raise)
#endif
{
	struct ch9344_ttyport *ttyport =
		container_of(port, struct ch9344_ttyport, port);
	struct ch9344 *ch9344 = tty_get_portdata(ttyport);
	int portnum = ttyport->portnum;
	int val;

	if ((ch9344->chiptype == CHIP_CH348Q) && (portnum > 3))
		return;

	if (raise)
		val = CH9344_CTO_D | CH9344_CTO_R;
	else
		val = 0;

	ch9344->ttyport[portnum].ctrlout = val;

	ch9344_set_control(ch9344, portnum, 0x01);
	ch9344_set_control(ch9344, portnum, 0x11);
}

static int ch9344_port_activate(struct tty_port *port,
				struct tty_struct *tty)
{
	struct ch9344_ttyport *ttyport =
		container_of(port, struct ch9344_ttyport, port);
	struct ch9344 *ch9344 = tty_get_portdata(ttyport);
	int portnum = ttyport->portnum;
	int retval = -ENODEV;

	mutex_lock(&ch9344->mutex);
	if (ch9344->disconnected)
		goto disconnected;

	retval = usb_autopm_get_interface(ch9344->data);
	if (retval)
		goto error_get_interface;

	set_bit(TTY_NO_WRITE_SPLIT, &tty->flags);
	ch9344->data->needs_remote_wakeup = 1;

	retval = ch9344_configure(ch9344, portnum);
	if (retval)
		goto error_configure;

	ch9344_tty_set_termios(tty, NULL);
#ifdef PACKLOAD
#if (LINUX_VERSION_CODE < KERNEL_VERSION(4, 14, 0))
	init_timer(&ttyport->timer);
	ttyport->timer.data = (unsigned long)ttyport;
	ttyport->timer.function = timer_function;
#else
	timer_setup(&ttyport->timer, timer_function, 0);
#endif
#endif

	usb_autopm_put_interface(ch9344->data);

	mutex_unlock(&ch9344->mutex);

	ttyport->isopen = true;

	return 0;

error_configure:
error_get_interface:
disconnected:
	mutex_unlock(&ch9344->mutex);

	return usb_translate_errors(retval);
}

static void ch9344_port_destruct(struct tty_port *port)
{
	struct ch9344_ttyport *ttyport =
		container_of(port, struct ch9344_ttyport, port);
	struct ch9344 *ch9344 = tty_get_portdata(ttyport);

	if (ch9344->opencounts-- == 1) {
		ch9344_release_minor(ch9344);
		usb_put_intf(ch9344->data);
		memset(ch9344, 0x00, sizeof(struct ch9344));
		kfree(ch9344);
	}
}

static void ch9344_port_shutdown(struct tty_port *port)
{
	struct ch9344_ttyport *ttyport =
		container_of(port, struct ch9344_ttyport, port);
	struct ch9344 *ch9344 = tty_get_portdata(ttyport);
	int portnum = ttyport->portnum;

#ifdef PACKLOAD
	del_timer(&ttyport->timer);
#endif
	ch9344_set_control(ch9344, portnum, 0x00);
	ch9344_set_control(ch9344, portnum, 0x10);
	ttyport->isopen = false;
}

static void ch9344_tty_cleanup(struct tty_struct *tty)
{
	struct ch9344 *ch9344 = tty->driver_data;

	tty_port_put(
		&ch9344->ttyport[ch9344_get_portnum(tty->index)].port);
}

static void ch9344_tty_hangup(struct tty_struct *tty)
{
	struct ch9344 *ch9344 = tty->driver_data;
	tty_port_hangup(
		&ch9344->ttyport[ch9344_get_portnum(tty->index)].port);
}

static void ch9344_tty_close(struct tty_struct *tty, struct file *filp)
{
	struct ch9344 *ch9344 = tty->driver_data;
	int portnum = ch9344_get_portnum(tty->index);
	int ret;

	tty_port_close(&ch9344->ttyport[portnum].port, tty, filp);
	if (!ch9344->disconnected &&
	    (ch9344->ttyport[portnum].uartmode == M_HF)) {
		ret = ch9344_set_uartmode(ch9344, portnum, portnum, M_NOR);
		if (!ret) {
			ch9344->ttyport[portnum].uartmode = M_NOR;
		}
	}
}

#if (LINUX_VERSION_CODE >= KERNEL_VERSION(6, 6, 0))
static ssize_t ch9344_tty_write(struct tty_struct *tty, const u8 *buf,
				size_t count)
#else
static int ch9344_tty_write(struct tty_struct *tty,
			    const unsigned char *buf, int count)
#endif
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

	if (!count)
		return 0;

	if (ch9344->writesize % (maxep - 3))
		maxpacknum = ch9344->writesize / (maxep - 3) + 1;
	else
		maxpacknum = ch9344->writesize / (maxep - 3);

	count = count > (ch9344->writesize - maxpacknum * 3) ?
			(ch9344->writesize - maxpacknum * 3) :
			count;
	total_len = count;
	sendlen = 0;

	if (count % (maxep - 3))
		packnum = count / (maxep - 3) + 1;
	else
		packnum = count / (maxep - 3);

transmit:
	spin_lock_irqsave(&ch9344->write_lock, flags);
	wbn = ch9344_wb_alloc(ch9344, portnum);
	if (wbn < 0) {
		spin_unlock_irqrestore(&ch9344->write_lock, flags);
		return 0;
	}
	wb = &ch9344->wb[portnum][wbn];

	if (!ch9344->dev) {
		wb->use = 0;
		spin_unlock_irqrestore(&ch9344->write_lock, flags);
		return -ENODEV;
	}

	/* packets deal */
	if (total_len > maxep - 3) {
		packlen = maxep - 3;
		total_len -= packlen;
	} else {
		packlen = total_len;
		total_len = 0;
	}
	*(wb->buf) = ch9344->port_offset + ch9344_get_portnum(tty->index);
	*(wb->buf + 1) = packlen;
	*(wb->buf + 2) = packlen >> 8;
	memcpy(wb->buf + 3, buf + sendlen, packlen);
	wb->len = packlen + 3;
	sendlen += packlen;
	wb->portnum = portnum;

	stat = usb_autopm_get_interface_async(ch9344->data);
	if (stat) {
		wb->use = 0;
		spin_unlock_irqrestore(&ch9344->write_lock, flags);
		return stat;
	}

	if (ch9344->susp_count) {
		if (ch9344->putbuffer) {
			usb_anchor_urb(ch9344->putbuffer->urb,
				       &ch9344->delayed);
			ch9344->putbuffer = NULL;
		}
		usb_anchor_urb(wb->urb, &ch9344->delayed);
		spin_unlock_irqrestore(&ch9344->write_lock, flags);
		return sendlen;
	} else {
		if (ch9344->putbuffer) {
			ch9344_start_wb(ch9344, ch9344->putbuffer);
			ch9344->putbuffer = NULL;
		}
	}

	if (!ch9344->ttyport[portnum].write_empty) {
		spin_unlock_irqrestore(&ch9344->write_lock, flags);
		timeout = wait_event_interruptible_timeout(
			ch9344->ttyport[portnum].wioctl,
			ch9344->ttyport[portnum].write_empty,
			msecs_to_jiffies(DEFAULT_TIMEOUT));
		if (timeout <= 0) {
			wb->use = 0;
			ch9344->ttyport[portnum].write_empty = true;
			dev_err(&ch9344->data->dev,
				"%s - write timeout\n",
				__func__);
			return sendlen ? sendlen : -ETIMEDOUT;
		}
		spin_lock_irqsave(&ch9344->write_lock, flags);
	}

	ch9344->ttyport[portnum].write_empty = false;
	stat = ch9344_start_wb(ch9344, wb);
	if (stat < 0) {
		ch9344->ttyport[portnum].write_empty = true;
		spin_unlock_irqrestore(&ch9344->write_lock, flags);
		return stat;
	}
	spin_unlock_irqrestore(&ch9344->write_lock, flags);

	if (total_len != 0)
		goto transmit;

	if (sendlen > 0)
		ch9344->ttyport[portnum].iocount.tx += sendlen;

	return sendlen;
}

#if (LINUX_VERSION_CODE >= KERNEL_VERSION(5, 14, 0))
static unsigned int ch9344_tty_write_room(struct tty_struct *tty)
#else
static int ch9344_tty_write_room(struct tty_struct *tty)
#endif
{
	struct ch9344 *ch9344 = tty->driver_data;
	int portnum = ch9344_get_portnum(tty->index);

	/*
     * Do not let the line discipline to know that we have a reserve,
     * or it might get too enthusiastic.
     */
	return ch9344_wb_is_avail(ch9344, portnum) ? ch9344->writesize : 0;
}

#if (LINUX_VERSION_CODE >= KERNEL_VERSION(5, 14, 0))
static unsigned int ch9344_tty_chars_in_buffer(struct tty_struct *tty)
#else
static int ch9344_tty_chars_in_buffer(struct tty_struct *tty)
#endif
{
	struct ch9344 *ch9344 = tty->driver_data;
	int portnum = ch9344_get_portnum(tty->index);

	/*
     * if the device was unplugged then any remaining characters fell out
     * of the connector ;)
     */
	if (ch9344->disconnected)
		return 0;
	/*
     * This is inaccurate (overcounts), but it works.
     */
	return (CH9344_NW - ch9344_wb_is_avail(ch9344, portnum)) *
	       ch9344->writesize;
}

static int ch9344_tty_break_ctl(struct tty_struct *tty, int state)
{
	struct ch9344 *ch9344 = tty->driver_data;
	int portnum = ch9344_get_portnum(tty->index);
	char *buffer;
	int retval;
	const unsigned size = 3;
	u8 rgadd = 0;

	if (ch9344->chiptype == CHIP_CH9344L ||
	    ch9344->chiptype == CHIP_CH9344Q)
		rgadd = 0x10 * portnum + 0x08;
	else if (ch9344->chiptype == CHIP_CH348L ||
		 ch9344->chiptype == CHIP_CH348Q) {
		if (portnum < 4)
			rgadd = 0x10 * portnum;
		else
			rgadd = 0x10 * (portnum - 4) + 0x08;
	}

	buffer = kzalloc(size, GFP_KERNEL);
	if (!buffer)
		return -ENOMEM;

	buffer[0] = CMD_W_BR;
	buffer[1] = rgadd + R_C3;

	if (state != 0)
		buffer[2] = 0x61;
	else
		buffer[2] = 0x60;

	retval = ch9344_cmd_out(ch9344, buffer, size);

	kfree(buffer);
	return retval < 0 ? retval : 0;
}

static int ch9344_tty_tiocmget(struct tty_struct *tty)
{
	struct ch9344 *ch9344 = tty->driver_data;
	int portnum = ch9344_get_portnum(tty->index);
	unsigned int result;

	result = (ch9344->ttyport[portnum].ctrlout & CH9344_CTO_D ?
			  TIOCM_DTR :
			  0) |
		 (ch9344->ttyport[portnum].ctrlout & CH9344_CTO_R ?
			  TIOCM_RTS :
			  0) |
		 (ch9344->ttyport[portnum].ctrlin & CH9344_CTI_C ?
			  TIOCM_CTS :
			  0) |
		 (ch9344->ttyport[portnum].ctrlin & CH9344_CTI_DS ?
			  TIOCM_DSR :
			  0) |
		 (ch9344->ttyport[portnum].ctrlin & CH9344_CTI_R ?
			  TIOCM_RI :
			  0) |
		 (ch9344->ttyport[portnum].ctrlin & CH9344_CTI_DC ?
			  TIOCM_CD :
			  0);

	return result;
}

static int ch9344_tty_tiocmset(struct tty_struct *tty, unsigned int set,
			       unsigned int clear)
{
	struct ch9344 *ch9344 = tty->driver_data;
	unsigned int newctrl;
	int rv = 0;
	int portnum = ch9344_get_portnum(tty->index);
	unsigned int xorval;

	if ((ch9344->chiptype == CHIP_CH348Q) && (portnum > 3))
		return 0;

	newctrl = ch9344->ttyport[portnum].ctrlout;
	set = (set & TIOCM_DTR ? CH9344_CTO_D : 0) |
	      (set & TIOCM_RTS ? CH9344_CTO_R : 0);
	clear = (clear & TIOCM_DTR ? CH9344_CTO_D : 0) |
		(clear & TIOCM_RTS ? CH9344_CTO_R : 0);

	newctrl = (newctrl & ~clear) | set;

	if (ch9344->ttyport[portnum].ctrlout == newctrl)
		return 0;
	xorval = newctrl ^ ch9344->ttyport[portnum].ctrlout;
	if (xorval & CH9344_CTO_D) {
		if (newctrl & CH9344_CTO_D)
			rv = ch9344_set_control(ch9344, portnum, 0x01);
		else
			rv = ch9344_set_control(ch9344, portnum, 0x00);
	}
	if (xorval & CH9344_CTO_R) {
		if (newctrl & CH9344_CTO_R)
			rv = ch9344_set_control(ch9344, portnum, 0x11);
		else
			rv = ch9344_set_control(ch9344, portnum, 0x10);
	}
	ch9344->ttyport[portnum].ctrlout = newctrl;

	return rv;
}

static int ch9344_get_icount(struct tty_struct *tty,
			     struct serial_icounter_struct *icount)
{
	struct ch9344 *ch9344 = tty->driver_data;
	int portnum = ch9344_get_portnum(tty->index);

	struct async_icount cnow;
	unsigned long flags;

	spin_lock_irqsave(&ch9344->read_lock, flags);
	cnow = ch9344->ttyport[portnum].iocount;
	spin_unlock_irqrestore(&ch9344->read_lock, flags);

	icount->cts = cnow.cts;
	icount->dsr = cnow.dsr;
	icount->rng = cnow.rng;
	icount->dcd = cnow.dcd;
	icount->tx = cnow.tx;
	icount->rx = cnow.rx;
	icount->frame = cnow.frame;
	icount->parity = cnow.parity;
	icount->overrun = cnow.overrun;
	icount->brk = cnow.brk;
	icount->buf_overrun = cnow.buf_overrun;

	return 0;
}

static int ch348_gpioenable(struct ch9344 *ch9344, u8 gpiogroup,
			    u8 gpioenable)
{
	u8 *buffer;
	int ret;
	u8 request;
	u8 rgadd;
	u64 gpioenables = ch9344->gpioenables;

	buffer = kzalloc(ch9344->cmdsize, GFP_KERNEL);
	if (!buffer)
		return -ENOMEM;

	request = R_MOD;
	rgadd = R_IO_CE;
	buffer[0] = request;
	buffer[1] = rgadd;
	gpioenables = cpu_to_be64(gpioenables);
	memcpy(buffer + 2, &gpioenables, 0x08);
	*(buffer + 9 - gpiogroup) = gpioenable;

	ret = ch9344_cmd_out(ch9344, buffer, 0x0a);
	if (ret == 0) {
		memcpy(&gpioenables, buffer + 2, 0x08);
		ch9344->gpioenables = be64_to_cpu(gpioenables);
	}
	kfree(buffer);

	return ret < 0 ? ret : 0;
}

static int ch348_set_gpiodir(struct ch9344 *ch9344, u8 gpionumber,
			     u8 gpiodir)
{
	u8 *buffer;
	int ret;
	u8 request;
	u8 rgadd;
	u64 gpiodirs = ch9344->gpiodirs;

	buffer = kzalloc(ch9344->cmdsize, GFP_KERNEL);
	if (!buffer)
		return -ENOMEM;

	request = R_MOD;
	rgadd = R_IO_CD;
	buffer[0] = request;
	buffer[1] = rgadd;
	if (gpiodir)
		gpiodirs |= (u64)1 << gpionumber;
	else
		gpiodirs &= ~((u64)1 << gpionumber);
	buffer[4] = gpiodirs >> 40;
	buffer[5] = gpiodirs >> 32;
	buffer[6] = gpiodirs >> 24;
	buffer[7] = gpiodirs >> 16;
	buffer[8] = gpiodirs >> 8;
	buffer[9] = gpiodirs;

	ret = ch9344_cmd_out(ch9344, buffer, 0x0a);
	kfree(buffer);
	if (ret == 0)
		ch9344->gpiodirs = gpiodirs;

	return ret < 0 ? ret : 0;
}

static int ch348_set_gpioval(struct ch9344 *ch9344, u8 gpionumber,
			     u8 gpioval)
{
	u8 *buffer;
	int ret;
	u8 request;
	u8 rgadd;

	buffer = kzalloc(ch9344->cmdsize, GFP_KERNEL);
	if (!buffer)
		return -ENOMEM;

	request = R_MOD;
	rgadd = R_IO_COS;
	buffer[0] = request;
	buffer[1] = rgadd;
	buffer[2] = gpionumber;
	buffer[3] = gpioval;

	ret = ch9344_cmd_out(ch9344, buffer, 0x04);
	kfree(buffer);

	return ret < 0 ? ret : 0;
}

static int ch348_get_gpioval(struct ch9344 *ch9344)
{
	u8 *buffer;
	int ret;
	u8 request;
	u8 rgadd;

	buffer = kzalloc(ch9344->cmdsize, GFP_KERNEL);
	if (!buffer)
		return -ENOMEM;

	ch9344->gpio_recv = false;

	request = R_MOD;
	rgadd = R_IO_CI;
	buffer[0] = request;
	buffer[1] = rgadd;
	ret = ch9344_cmd_out(ch9344, buffer, 0x03);
	kfree(buffer);

	if (ret < 0)
		goto out;

	ret = wait_event_interruptible_timeout(
		ch9344->wgpioioctl, ch9344->gpio_recv,
		msecs_to_jiffies(DEFAULT_TIMEOUT));
	if (ret == 0) {
		return -ETIMEDOUT;
	}

	if (ret < 0) {
		return ret;
	}

out:
	return ret < 0 ? ret : 0;
}

static int ch9344_set_uartmode(struct ch9344 *ch9344, int portnum,
			       u8 index, u8 mode)
{
	u8 *buffer;
	int ret = 0;
	u8 request;
	u8 rgadd = 0;
	u8 mode4 = 0;
	int i;

	if (ch9344->chiptype == CHIP_CH9344L ||
	    ch9344->chiptype == CHIP_CH9344Q)
		rgadd = 0x10 * portnum + 0x08;
	else if (ch9344->chiptype == CHIP_CH348L ||
		 ch9344->chiptype == CHIP_CH348Q) {
		if (portnum < 4)
			rgadd = 0x10 * portnum;
		else
			rgadd = 0x10 * (portnum - 4) + 0x08;
	}

	buffer = kzalloc(ch9344->cmdsize, GFP_KERNEL);
	if (!buffer)
		return -ENOMEM;

	if (mode == M_HF) {
		buffer[0] = CMD_W_BR;
		buffer[1] = rgadd + R_C4;
		buffer[2] = 0x51;
		ret = ch9344_cmd_out(ch9344, buffer, 0x03);
		if (ret < 0)
			goto out;
	} else if (mode == M_NOR) {
		buffer[0] = CMD_W_BR;
		buffer[1] = rgadd + R_C4;
		buffer[2] = 0x50;
		ret = ch9344_cmd_out(ch9344, buffer, 0x03);
		if (ret < 0)
			goto out;
	}

	if (ch9344->chiptype == CHIP_CH348L ||
	    ch9344->chiptype == CHIP_CH348Q)
		goto out;

	for (i = 0; i < ch9344->num_ports; i++) {
		if (i != index)
			mode4 |= (ch9344->ttyport[i].uartmode) << (i * 2);
	}
	mode4 |= mode << (index * 2);
	request = CMD_WB_E + portnum + ch9344->port_offset;
	rgadd = R_MOD;
	memset(buffer, 0x00, ch9344->cmdsize);
	buffer[0] = request;
	buffer[1] = rgadd;
	buffer[2] = mode4;
	ret = ch9344_cmd_out(ch9344, buffer, 0x04);

out:
	kfree(buffer);
	return ret < 0 ? ret : 0;
}

static int ch9344_set_gpiodir(struct ch9344 *ch9344, int portnum,
			      u8 gpionumber, u8 gpiodir)
{
	u8 *buffer;
	int ret;
	u8 request;
	u8 rgadd;
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

	request = CMD_WB_E + portnum + ch9344->port_offset;
	rgadd = R_IO_D;
	buffer[0] = request;
	buffer[1] = rgadd;
	buffer[2] = gpiodirs;
	buffer[3] = gpiodirs >> 8;
	ret = ch9344_cmd_out(ch9344, buffer, 0x04);
	kfree(buffer);
	return ret < 0 ? ret : 0;
}

static int ch9344_set_gpioval(struct ch9344 *ch9344, int portnum,
			      u8 gpionumber, u8 gpioval)
{
	u8 *buffer;
	int ret;
	u8 request;
	u8 rgadd;
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

	request = CMD_WB_E + portnum + ch9344->port_offset;
	rgadd = R_IO_O;
	buffer[0] = request;
	buffer[1] = rgadd;
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
	u8 rgadd;
	u16 gpiodirs = 0;
	int i;

	buffer = kzalloc(ch9344->cmdsize, GFP_KERNEL);
	if (!buffer)
		return -ENOMEM;

	ch9344->gpio_recv = false;

	for (i = 0; i < MAXGPIO; i++) {
		gpiodirs |= (ch9344->gpiodir[i]) << i;
	}

	request = CMD_WB_E + portnum + ch9344->port_offset;
	rgadd = R_IO_I;
	buffer[0] = request;
	buffer[1] = rgadd;
	buffer[2] = gpiodirs;
	buffer[3] = gpiodirs >> 8;
	ret = ch9344_cmd_out(ch9344, buffer, 0x04);
	kfree(buffer);
	if (ret < 0)
		goto out;

	ret = wait_event_interruptible_timeout(
		ch9344->wgpioioctl, ch9344->gpio_recv,
		msecs_to_jiffies(DEFAULT_TIMEOUT));
	if (ret == 0) {
		return -ETIMEDOUT;
	}

	if (ret < 0) {
		return ret;
	}

out:
	return ret < 0 ? ret : 0;
}

static int ch9344_get_serial_info(struct ch9344 *ch9344, int portnum,
				  struct serial_struct __user *info)
{
	struct serial_struct tmp;

	if (!info)
		return -EINVAL;

	memset(&tmp, 0, sizeof(tmp));
	tmp.flags = ASYNC_LOW_LATENCY;
	tmp.xmit_fifo_size = ch9344->writesize;
	tmp.baud_base =
		le32_to_cpu(ch9344->ttyport[portnum].line.dwDTERate);
	tmp.close_delay = ch9344->ttyport[portnum].port.close_delay / 10;
	tmp.closing_wait =
		ch9344->ttyport[portnum].port.closing_wait ==
				ASYNC_CLOSING_WAIT_NONE ?
			ASYNC_CLOSING_WAIT_NONE :
			ch9344->ttyport[portnum].port.closing_wait / 10;

	if (copy_to_user(info, &tmp, sizeof(tmp)))
		return -EFAULT;
	else
		return 0;
}

static int ch9344_set_serial_info(struct ch9344 *ch9344, int portnum,
				  struct serial_struct __user *newinfo)
{
	struct serial_struct new_serial;
	unsigned int closing_wait, close_delay;
	int retval = 0;

	if (copy_from_user(&new_serial, newinfo, sizeof(new_serial)))
		return -EFAULT;

	close_delay = new_serial.close_delay * 10;
	closing_wait = new_serial.closing_wait == ASYNC_CLOSING_WAIT_NONE ?
			       ASYNC_CLOSING_WAIT_NONE :
			       new_serial.closing_wait * 10;

	mutex_lock(&ch9344->ttyport[portnum].port.mutex);

	if (!capable(CAP_SYS_ADMIN)) {
		if ((close_delay !=
		     ch9344->ttyport[portnum].port.close_delay) ||
		    (closing_wait !=
		     ch9344->ttyport[portnum].port.closing_wait))
			retval = -EPERM;
		else
			retval = -EOPNOTSUPP;
	} else {
		ch9344->ttyport[portnum].port.close_delay = close_delay;
		ch9344->ttyport[portnum].port.closing_wait = closing_wait;
	}

	mutex_unlock(&ch9344->ttyport[portnum].port.mutex);
	return retval;
}

static int ch9344_wait_serial_change(struct ch9344 *ch9344, int portnum,
				     unsigned long arg)
{
	int rv = 0;
	DECLARE_WAITQUEUE(wait, current);
	struct async_icount old, new;

	do {
		spin_lock_irq(&ch9344->read_lock);
		old = ch9344->ttyport[portnum].oldcount;
		new = ch9344->ttyport[portnum].iocount;
		ch9344->ttyport[portnum].oldcount = new;
		spin_unlock_irq(&ch9344->read_lock);

		if ((arg & TIOCM_CTS) && old.cts != new.cts)
			break;
		if ((arg & TIOCM_DSR) && old.dsr != new.dsr)
			break;
		if ((arg & TIOCM_RI) && old.rng != new.rng)
			break;
		if ((arg & TIOCM_CD) && old.dcd != new.dcd)
			break;

		add_wait_queue(&ch9344->ttyport[portnum].wioctl, &wait);
		set_current_state(TASK_INTERRUPTIBLE);
		schedule();
		remove_wait_queue(&ch9344->ttyport[portnum].wioctl, &wait);
		if (ch9344->disconnected) {
			if (arg & TIOCM_CD)
				break;
			else
				rv = -ENODEV;
		} else {
			if (signal_pending(current))
				rv = -ERESTARTSYS;
		}
	} while (!rv);

	return rv;
}

static int
ch9344_get_serial_usage(struct ch9344 *ch9344, int portnum,
			struct serial_icounter_struct __user *count)
{
	struct serial_icounter_struct icount;
	int rv = 0;

	memset(&icount, 0, sizeof(icount));
	icount.cts = ch9344->ttyport[portnum].iocount.cts;
	icount.dsr = ch9344->ttyport[portnum].iocount.dsr;
	icount.rng = ch9344->ttyport[portnum].iocount.rng;
	icount.dcd = ch9344->ttyport[portnum].iocount.dcd;
	icount.tx = ch9344->ttyport[portnum].iocount.tx;
	icount.rx = ch9344->ttyport[portnum].iocount.rx;
	icount.frame = ch9344->ttyport[portnum].iocount.frame;
	icount.overrun = ch9344->ttyport[portnum].iocount.overrun;
	icount.parity = ch9344->ttyport[portnum].iocount.parity;
	icount.brk = ch9344->ttyport[portnum].iocount.brk;
	icount.buf_overrun = ch9344->ttyport[portnum].iocount.buf_overrun;

	if (copy_to_user(count, &icount, sizeof(icount)) > 0)
		rv = -EFAULT;

	return rv;
}

static int ch9344_tty_ioctl(struct tty_struct *tty, unsigned int cmd,
			    unsigned long arg)
{
	struct ch9344 *ch9344 = tty->driver_data;
	int rv = -ENOIOCTLCMD;
	u16 __user *argval = (u16 __user *)arg;
	int portnum = ch9344_get_portnum(tty->index);

	unsigned long arg1, arg2, arg3, arg4, arg5, arg6;
	u8 *buffer;

	buffer = kmalloc(512, GFP_KERNEL);
	if (!buffer)
		return -ENOMEM;

	switch (cmd) {
	case TIOCGSERIAL: /* gets serial port data */
		rv = ch9344_get_serial_info(
			ch9344, portnum,
			(struct serial_struct __user *)arg);
		break;
	case TIOCSSERIAL:
		rv = ch9344_set_serial_info(
			ch9344, portnum,
			(struct serial_struct __user *)arg);
		break;
	case TIOCMIWAIT:
		rv = usb_autopm_get_interface(ch9344->data);
		if (rv < 0) {
			rv = -EIO;
			break;
		}
		rv = ch9344_wait_serial_change(ch9344, portnum, arg);
		usb_autopm_put_interface(ch9344->data);
		break;
	case TIOCGICOUNT:
		rv = ch9344_get_serial_usage(
			ch9344, portnum,
			(struct serial_icounter_struct __user *)arg);
		break;
	case IOCTL_CMD_GETCHIPTYPE:
		if (put_user(ch9344->chiptype, argval)) {
			rv = -EFAULT;
			goto out;
		} else
			rv = 0;
		break;
	case IOCTL_CMD_GETUARTINDEX:
		if (put_user(portnum, argval)) {
			rv = -EFAULT;
			goto out;
		} else
			rv = 0;
		break;
	case IOCTL_CMD_CMDIN:
		get_user(arg1, (u16 __user *)arg);
		arg2 = (unsigned long)((u8 __user *)arg + 2);
		arg3 = (unsigned long)((u8 __user *)arg + 2 + 256);
		rv = ch9344_cmd_in(ch9344, (u8 __user *)arg3, (int)arg1,
				   (u8 __user *)arg2);
		break;
	case IOCTL_CMD_CMDOUT:
		get_user(arg1, (u16 __user *)arg);
		arg2 = (unsigned long)((u8 __user *)arg + 2);
		rv = copy_from_user(buffer, (u8 __user *)arg2, arg1);
		if (rv)
			goto out;
		rv = ch9344_cmd_out(ch9344, buffer, (int)arg1);
		break;
	case IOCTL_CMD_CTRLIN:
		get_user(arg1, (u8 __user *)arg);
		get_user(arg2, ((u8 __user *)arg + 1));
		get_user(arg3, (u16 __user *)((u8 *)arg + 2));
		get_user(arg4, (u16 __user *)((u8 *)arg + 4));
		get_user(arg5, (u16 __user *)((u8 *)arg + 6));
		arg6 = (unsigned long)((u8 __user *)arg + 8);
		rv = copy_from_user(buffer, (u8 __user *)arg6, arg5);
		if (rv)
			goto out;
		rv = ch9344_control_in(ch9344, (u8)arg1, (u8)arg2,
				       (u16)arg3, (u16)arg4,
				       (u8 __user *)buffer, (u16)arg5);
		rv = copy_to_user((u8 __user *)arg6, buffer, arg5);
		if (rv)
			goto out;
		break;
	case IOCTL_CMD_CTRLOUT:
		get_user(arg1, (u8 __user *)arg);
		get_user(arg2, ((u8 __user *)arg + 1));
		get_user(arg3, (u16 __user *)((u8 *)arg + 2));
		get_user(arg4, (u16 __user *)((u8 *)arg + 4));
		get_user(arg5, (u16 __user *)((u8 *)arg + 6));
		arg6 = (unsigned long)((u8 __user *)arg + 8);
		rv = ch9344_control_out(ch9344, (u8)arg1, (u8)arg2,
					(u16)arg3, (u16)arg4,
					(u8 __user *)arg6, (u16)arg5);
		if (rv != (u16)arg5) {
			rv = -EINVAL;
			goto out;
		} else
			rv = 0;
		break;
	default:
		break;
	}

out:
	kfree(buffer);
	return rv;
}

static int ch9344_get(int clockRate, int bval, unsigned char *bd1,
		      unsigned char *bd2)
{
	int dis, x2;

	if (bval < 0 || bval > 12000000) {
		return -1;
	}
	/* calculate dis from bval */
	if (bval == 2000000) {
		*bd1 = 2;
		*bd2 = 0;
	} else {
		dis = 10 * clockRate / 16 / bval;
		x2 = dis % 10;
		dis /= 10;
		if (x2 >= 5)
			dis++;
		*bd1 = dis;
		*bd2 = (unsigned char)(dis >> 8);
	}

	return 0;
}

static void cal_outdata(char *buffer, u8 rol, u8 xor)
{
	u8 en_status, i;

	for (i = 0; i < rol; i++) {
		en_status = buffer[0];
		buffer[0] = buffer[0] << 1;
		buffer[0] = buffer[0] | ((buffer[1] & 0x80) ? 1 : 0);
		buffer[1] = buffer[1] << 1;
		buffer[1] = buffer[1] | ((buffer[2] & 0x80) ? 1 : 0);
		buffer[2] = buffer[2] << 1;
		buffer[2] = buffer[2] | ((buffer[3] & 0x80) ? 1 : 0);
		buffer[3] = buffer[3] << 1;
		buffer[3] = buffer[3] | ((buffer[4] & 0x80) ? 1 : 0);
		buffer[4] = buffer[4] << 1;
		buffer[4] = buffer[4] | ((buffer[5] & 0x80) ? 1 : 0);
		buffer[5] = buffer[5] << 1;
		buffer[5] = buffer[5] | ((buffer[6] & 0x80) ? 1 : 0);
		buffer[6] = buffer[6] << 1;
		buffer[6] = buffer[6] | ((buffer[7] & 0x80) ? 1 : 0);
		buffer[7] = buffer[7] << 1;
		buffer[7] = buffer[7] | ((en_status & 0x80) ? 1 : 0);
	}
	for (i = 0; i < 8; i++) {
		buffer[i] = buffer[i] ^ xor;
	}
}

static u8 cal_recv_tmt(__le32 bd)
{
	u8 tmt;
	int dly = 1000000 * 15 / bd;

	if (bd >= 921600)
		return 5;

	tmt = dly / 100 + 1;

#ifdef PACKLOAD
	if (bd < 9600)
		tmt /= 2;
#endif

	return tmt;
}

#if (LINUX_VERSION_CODE >= KERNEL_VERSION(6, 1, 0))
static void ch9344_tty_set_termios(struct tty_struct *tty,
				   const struct ktermios *termios_old)
#else
static void ch9344_tty_set_termios(struct tty_struct *tty,
				   struct ktermios *termios_old)
#endif
{
	struct ch9344 *ch9344 = tty->driver_data;
#if (LINUX_VERSION_CODE >= KERNEL_VERSION(3, 7, 0))
	struct ktermios *termios = &tty->termios;
#else
	struct ktermios *termios = tty->termios;
#endif
	struct usb_ch9344_line_coding newline;
	int portnum = ch9344_get_portnum(tty->index);
	int newctrl = ch9344->ttyport[portnum].ctrlout;
	int ret;
	unsigned char bd1 = 0;
	unsigned char bd2 = 0;
	unsigned char bd3 = 0;
	unsigned char dbit = 0, pbit = 0, sbit = 0;
	unsigned char pedt = 0x00;
	int clrt = 1843200;
	u8 rgadd = 0;
	char *buffer;
	u8 xor, rol;
	u8 rbytes[2];
#ifdef PACKLOAD
	int dly;
#endif

#if (LINUX_VERSION_CODE >= KERNEL_VERSION(3, 7, 0))
	if (termios_old &&
	    !tty_termios_hw_change(&tty->termios, termios_old)) {
#else
	if (termios_old &&
	    !tty_termios_hw_change(tty->termios, termios_old)) {
#endif
		return;
	}

	buffer = kzalloc(ch9344->cmdsize, GFP_KERNEL);
	if (!buffer) {
		/* restore termios */
		if (termios_old)
#if (LINUX_VERSION_CODE >= KERNEL_VERSION(3, 7, 0))
			tty->termios = *termios_old;
#else
			tty->termios = termios_old;
#endif
		return;
	}

	newline.dwDTERate = tty_get_baud_rate(tty);
	if (newline.dwDTERate == 0)
		newline.dwDTERate = 9600;
	if (newline.dwDTERate > 115200) {
		pedt = 0x01;
		clrt = 44236800;
	}
	ch9344_get(clrt, newline.dwDTERate, &bd1, &bd2);
	switch (newline.dwDTERate) {
	case 250000:
		bd3 = 1;
		break;
	case 500000:
		bd3 = 2;
		break;
	case 1000000:
		bd3 = 3;
		break;
	case 1500000:
		bd3 = 4;
		break;
	case 3000000:
		bd3 = 5;
		break;
	case 12000000:
		bd3 = 6;
		break;
	default:
		bd3 = 0;
		break;
	}

	newline.bCharFormat = termios->c_cflag & CSTOPB ? 2 : 1;
	if (newline.bCharFormat == 2)
		sbit = 0x04;

	newline.bParityType =
		termios->c_cflag & PARENB ?
			(termios->c_cflag & PARODD ? 1 : 2) +
				(termios->c_cflag & CMSPAR ? 2 : 0) :
			0;

	switch (newline.bParityType) {
	case 0x01:
		pbit = 0x08;
		break;
	case 0x02:
		pbit = (0x01 << 4) + 0x08;
		break;
	case 0x03:
		pbit = (0x02 << 4) + 0x08;
		break;
	case 0x04:
		pbit = (0x03 << 4) + 0x08;
		break;
	default:
		pbit = 0x00;
		break;
	}

	switch (termios->c_cflag & CSIZE) {
	case CS5:
		newline.bDataBits = 5;
		dbit = 0x00;
		break;
	case CS6:
		newline.bDataBits = 6;
		dbit = 0x01;
		break;
	case CS7:
		newline.bDataBits = 7;
		dbit = 0x02;
		break;
	case CS8:
	default:
		newline.bDataBits = 8;
		dbit = 0x03;
		break;
	}

	/* FIXME: Needs to clear unsupported bits in the termios */
	ch9344->ttyport[portnum].clocal =
		((termios->c_cflag & CLOCAL) != 0);

	if (C_BAUD(tty) == B0) {
		newline.dwDTERate =
			ch9344->ttyport[portnum].line.dwDTERate;
		newctrl &= ~(CH9344_CTO_D | CH9344_CTO_R);
	} else if (termios_old && (termios_old->c_cflag & CBAUD) == B0) {
		newctrl |= CH9344_CTO_D | CH9344_CTO_R;
	}

	if (ch9344->chiptype == CHIP_CH9344L ||
	    ch9344->chiptype == CHIP_CH9344Q) {
		rgadd = 0x10 * ch9344_get_portnum(tty->index) + 0x08;

		memset(buffer, 0x00, ch9344->cmdsize);
		buffer[0] = CMD_W_BR;
		buffer[1] = rgadd + 0x01;
		buffer[2] = pedt + 0x50;
		ret = ch9344_cmd_out(ch9344, buffer, 0x03);
		if (ret < 0)
			goto out;

		if (ch9344->chiptype == CHIP_CH9344L) {
			memset(buffer, 0x00, ch9344->cmdsize);
			buffer[0] = CMD_S_T;
			buffer[1] = rgadd + 0x03;
			buffer[2] = bd1;
			buffer[3] = bd2;
			buffer[4] = bd3;
			ret = ch9344_cmd_out(ch9344, buffer, 0x06);
			if (ret < 0)
				goto out;
		} else {
			buffer[0] = CMD_S_T;
			buffer[1] = rgadd + 0x03;
			buffer[2] = 0x00;
			buffer[3] = 0x00;
			buffer[4] = 0x00;
			buffer[5] = (char)(newline.dwDTERate);
			buffer[6] = (char)(newline.dwDTERate >> 8);
			buffer[7] = (char)(newline.dwDTERate >> 16);
			buffer[8] = (char)(newline.dwDTERate >> 24);
			ret = ch9344_cmd_out(ch9344, buffer, 0x09);
			if (ret < 0)
				goto out;
		}

		memset(buffer, 0x00, ch9344->cmdsize);
		buffer[0] = CMD_W_R;
		buffer[1] = rgadd + 0x03;
		buffer[2] = dbit | pbit | sbit;
		ret = ch9344_cmd_out(ch9344, buffer, 0x03);
		if (ret < 0)
			goto out;

		/* uart receive timeout */
		memset(buffer, 0x00, ch9344->cmdsize);
		buffer[0] = CMD_WB_E + portnum + ch9344->port_offset;
		buffer[1] = R_TM_O;
		buffer[2] = portnum + ch9344->port_offset;
		buffer[3] = cal_recv_tmt(newline.dwDTERate);
		ret = ch9344_cmd_out(ch9344, buffer, 0x04);
		if (ret < 0)
			goto out;
	} else {
		get_random_bytes(rbytes, 2);
		rol = (u8)rbytes[0] & 0x0f;
		xor = (u8)rbytes[1];
		buffer[0] = CMD_WB_E | (portnum & 0x0F);
		buffer[1] = R_INIT;
		buffer[2] = (u8)((portnum & 0x0F) | ((rol << 4) & 0xf0));
		buffer[3] = (char)(newline.dwDTERate >> 24);
		buffer[4] = (char)(newline.dwDTERate >> 16);
		buffer[5] = (char)(newline.dwDTERate >> 8);
		buffer[6] = (char)(newline.dwDTERate);
		if (newline.bCharFormat == 2)
			buffer[7] = 0x02;
		else if (newline.bCharFormat == 1)
			buffer[7] = 0x00;
		buffer[8] = newline.bParityType;
		buffer[9] = newline.bDataBits;
		buffer[10] = cal_recv_tmt(newline.dwDTERate);
		buffer[11] = xor;
		cal_outdata(buffer + 3, rol & 0x0f, xor);

		ret = ch9344_cmd_out(ch9344, buffer, 0x0c);
		if (ret < 0)
			goto out;
	}

#ifdef PACKLOAD
	dly = 1000000 * 30 / newline.dwDTERate / 1000 + 1;
	ch9344->ttyport[portnum].interval =
		dly * HZ / 1000 > 0 ? dly * HZ / 1000 : 2;
	ch9344->ttyport[portnum].timer.expires =
		jiffies + ch9344->ttyport[portnum].interval;
#endif

	if (ch9344->chiptype == CHIP_CH9344L ||
	    ch9344->chiptype == CHIP_CH9344Q)
		rgadd = 0x10 * portnum + 0x08;
	else {
		if (portnum < 4)
			rgadd = 0x10 * portnum;
		else
			rgadd = 0x10 * (portnum - 4) + 0x08;
	}

	buffer[0] = CMD_W_R;
	buffer[1] = rgadd + R_C1;
	if (ch9344->modeline9)
		buffer[2] = 0x0F;
	else
		buffer[2] = 0x07;
	ret = ch9344_cmd_out(ch9344, buffer, 0x03);
	if (ret < 0)
		goto out;

	if ((ch9344->chiptype == CHIP_CH348Q) && (portnum > 3)) {
	} else {
		if (newctrl != ch9344->ttyport[portnum].ctrlout) {
			if (newctrl & CH9344_CTO_D)
				ch9344_set_control(ch9344, portnum, 0x01);
			else
				ch9344_set_control(ch9344, portnum, 0x00);
			ch9344->ttyport[portnum].ctrlout = newctrl;
		}
	}

	if (memcmp(&ch9344->ttyport[portnum].line, &newline,
		   sizeof newline)) {
		memcpy(&ch9344->ttyport[portnum].line, &newline,
		       sizeof newline);
	}

	if (C_CRTSCTS(tty)) {
		if (ch9344->ttyport[portnum].uartmode == M_IO) {
			ret = -EINVAL;
			goto out;
		}
		ret = ch9344_set_uartmode(ch9344, portnum, portnum, M_HF);
		if (!ret) {
			ch9344->ttyport[portnum].uartmode = M_HF;
		}
	} else {
		if (ch9344->ttyport[portnum].uartmode == M_HF) {
			ret = ch9344_set_uartmode(ch9344, portnum, portnum,
						  M_NOR);
			if (!ret) {
				ch9344->ttyport[portnum].uartmode = M_NOR;
			}
		}
	}

	buffer[0] = CMD_WB_E;
	buffer[1] = VEN_R;
	buffer[2] = rgadd | 0x06;
	ret = ch9344_cmd_out(ch9344, buffer, 0x03);
	if (ret < 0)
		goto out;

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
	int i, index;
	struct ch9344_wb *wb;
	struct usb_device *usb_dev = interface_to_usbdev(ch9344->data);

	for (index = 0; index < ch9344->num_ports; index++) {
		for (i = 0; i < CH9344_NW; i++) {
			wb = &ch9344->wb[index][i];
			usb_free_coherent(usb_dev, ch9344->writesize,
					  wb->buf, wb->dmah);
		}
	}
}

static void ch9344_read_buffers_free(struct ch9344 *ch9344)
{
	struct usb_device *usb_dev = interface_to_usbdev(ch9344->data);
	int i;

	for (i = 0; i < ch9344->rx_buflimit; i++)
		usb_free_coherent(usb_dev, ch9344->readsize,
				  ch9344->read_buffers[i].base,
				  ch9344->read_buffers[i].dma);
}

/* Little helper: write buffers allocate */
static int ch9344_write_buffers_alloc(struct ch9344 *ch9344)
{
	int i, index;
	struct ch9344_wb *wb;

	for (index = 0; index < ch9344->num_ports; index++) {
		for (i = 0; i < CH9344_NW; i++) {
			wb = &ch9344->wb[index][i];
			wb->buf = usb_alloc_coherent(ch9344->dev,
						     ch9344->writesize,
						     GFP_KERNEL,
						     &wb->dmah);
			if (!wb->buf) {
				while (i != 0) {
					--i;
					--wb;
					usb_free_coherent(
						ch9344->dev,
						ch9344->writesize, wb->buf,
						wb->dmah);
				}
				return -ENOMEM;
			}
		}
	}

	return 0;
}

static int ch9344_open(struct inode *inode, struct file *file)
{
	struct ch9344 *ch9344;
	struct usb_interface *interface;
	int subminor;
	int retval = 0;

	subminor = iminor(inode);

	interface = usb_find_interface(&ch9344_driver, subminor);
	if (!interface) {
		pr_err("%s - error, can't find device for minor %d\n",
		       __func__, subminor);
		retval = -ENODEV;
		goto exit;
	}

	ch9344 = usb_get_intfdata(interface);
	if (!ch9344) {
		retval = -ENODEV;
		goto exit;
	}

	/* save our object in the file's private structure */
	file->private_data = ch9344;

exit:
	return retval;
}

static int ch9344_release(struct inode *inode, struct file *file)
{
	struct ch9344 *ch9344 = file->private_data;

	if (ch9344 == NULL || ch9344->io_id != IOID)
		return -ENODEV;

	mutex_lock(&ch9344->mutex);

	if (ch9344->disconnected) {
		mutex_unlock(&ch9344->mutex);
		return -ENODEV;
	}
	mutex_unlock(&ch9344->mutex);
	return 0;
}

static long ch9344_ioctl(struct file *file, unsigned int cmd,
			 unsigned long arg)
{
	struct ch9344 *ch9344 = file->private_data;
	int rv = -ENOIOCTLCMD;
	u8 uartmode;
	int i;
	u16 inarg;
	u8 inargH, inargL;
	u16 __user *argval = (u16 __user *)arg;
	u8 gpioval;
	u8 gpiogroup;
	u8 gpioenable;
	u8 gpionumber;
	u8 gpiodir;
	u8 portindex;
	u8 *buffer;

	if (ch9344 == NULL || ch9344->io_id != IOID)
		return -ENODEV;

	mutex_lock(&ch9344->mutex);

	if (ch9344->disconnected) {
		rv = -ENODEV;
		goto out_unfree;
	}

	buffer = kmalloc(8, GFP_KERNEL);
	if (!buffer) {
		rv = -ENOMEM;
		goto out_unfree;
	}

	switch (cmd) {
	case IOCTL_CMD_GETCHIPTYPE:
		if (put_user(ch9344->chiptype, argval)) {
			rv = -EFAULT;
			goto out;
		} else
			rv = 0;
		break;
	case IOCTL_CMD_GPIOENABLE:
		if (get_user(inarg, argval)) {
			rv = -EFAULT;
			goto out;
		}
		inargH = inarg >> 8;
		inargL = inarg;
		if (ch9344->chiptype == CHIP_CH9344L ||
		    ch9344->chiptype == CHIP_CH9344Q) {
			portindex = inargH;
			if (inargL)
				uartmode = M_IO;
			else
				uartmode = M_NOR;
			if ((portindex > 3) ||
			    (ch9344->ttyport[portindex].uartmode ==
			     M_HF)) {
				rv = -EINVAL;
				goto out;
			}
			mutex_lock(&ch9344->gpiomutex);
			rv = ch9344_set_uartmode(ch9344, portindex,
						 portindex, uartmode);
			if (!rv) {
				ch9344->ttyport[portindex].uartmode =
					uartmode;
				if (uartmode == M_NOR) {
					/* restores gpio related vars here */
					for (i = portindex * 3;
					     i < portindex * 3 + 3; i++) {
						ch9344->gpiodir[i] = G_DI;
						ch9344->gpioval[i] = IO_L;
					}
				}
			}
			mutex_unlock(&ch9344->gpiomutex);
		} else if (ch9344->chiptype == CHIP_CH348L ||
			   ch9344->chiptype == CHIP_CH348Q) {
			gpiogroup = inargH;
			gpioenable = inargL;
			mutex_lock(&ch9344->gpiomutex);
			rv = ch348_gpioenable(ch9344, gpiogroup,
					      gpioenable);
			mutex_unlock(&ch9344->gpiomutex);
		}
		break;
	case IOCTL_CMD_GPIODIR:
		if (get_user(inarg, argval)) {
			rv = -EFAULT;
			goto out;
		}
		/* inargH indicates gpio number, inargL indicates gpio direction */
		inargH = inarg >> 8;
		inargL = inarg;
		if (ch9344->chiptype == CHIP_CH9344L ||
		    ch9344->chiptype == CHIP_CH9344Q) {
			gpionumber = inargH;
			gpiogroup = gpionumber / 3;
			if (inargL)
				gpiodir = G_DO;
			else
				gpiodir = G_DI;
			if ((gpionumber > MAXGPIO) ||
			    (ch9344->ttyport[gpiogroup].uartmode !=
			     M_IO)) {
				rv = -EINVAL;
				goto out;
			}
			mutex_lock(&ch9344->gpiomutex);
			rv = ch9344_set_gpiodir(ch9344, gpiogroup,
						gpionumber, gpiodir);
			if (!rv) {
				ch9344->gpiodir[gpionumber] = gpiodir;
				if (gpiodir == G_DI)
					ch9344->gpioval[gpionumber] = IO_H;
			}
			mutex_unlock(&ch9344->gpiomutex);
		} else if (ch9344->chiptype == CHIP_CH348L ||
			   ch9344->chiptype == CHIP_CH348Q) {
			mutex_lock(&ch9344->gpiomutex);
			gpionumber = inargH;
			gpiodir = inargL;
			rv = ch348_set_gpiodir(ch9344, gpionumber,
					       gpiodir);
			mutex_unlock(&ch9344->gpiomutex);
		}
		break;
	case IOCTL_CMD_GPIOSET:
		if (get_user(inarg, argval)) {
			rv = -EFAULT;
			goto out;
		}
		/* inargH indicates gpio number, inargL indicates gpio output value */
		inargH = inarg >> 8;
		inargL = inarg;
		if (ch9344->chiptype == CHIP_CH9344L ||
		    ch9344->chiptype == CHIP_CH9344Q) {
			gpionumber = inargH;
			gpiogroup = gpionumber / 3;
			if (inargL)
				gpioval = IO_H;
			else
				gpioval = IO_L;
			if ((gpionumber > MAXGPIO) ||
			    (ch9344->ttyport[gpiogroup].uartmode !=
			     M_IO) ||
			    (ch9344->gpiodir[gpionumber] != G_DO)) {
				rv = -EINVAL;
				goto out;
			}
			mutex_lock(&ch9344->gpiomutex);
			rv = ch9344_set_gpioval(ch9344, gpiogroup,
						gpionumber, gpioval);
			if (!rv) {
				ch9344->gpioval[gpionumber] = gpioval;
			}
			mutex_unlock(&ch9344->gpiomutex);
		} else if (ch9344->chiptype == CHIP_CH348L ||
			   ch9344->chiptype == CHIP_CH348Q) {
			mutex_lock(&ch9344->gpiomutex);
			gpionumber = inargH;
			gpioval = inargL;
			rv = ch348_set_gpioval(ch9344, gpionumber,
					       gpioval);
			mutex_unlock(&ch9344->gpiomutex);
		}
		break;
	case IOCTL_CMD_GPIOGET:
		if (get_user(inarg, argval)) {
			rv = -EFAULT;
			goto out;
		}
		/* inargH indicates gpio number, inargL indicates gpio output value */
		inargH = inarg >> 8;
		inargL = inarg;
		if (ch9344->chiptype == CHIP_CH9344L ||
		    ch9344->chiptype == CHIP_CH9344Q) {
			gpionumber = inargH;
			gpiogroup = gpionumber / 3;
			if ((gpionumber > MAXGPIO) ||
			    (ch9344->ttyport[gpiogroup].uartmode !=
			     M_IO) ||
			    (ch9344->gpiodir[gpionumber] != G_DI)) {
				rv = -EINVAL;
				goto out;
			}
			mutex_lock(&ch9344->gpiomutex);
			rv = ch9344_get_gpioval(ch9344, gpiogroup);
			if (!rv) {
				gpioval = ((ch9344->gpiovalin) &
					   (1 << gpionumber)) ?
						  IO_H :
						  IO_L;
				if (put_user(gpioval, argval)) {
					rv = -EFAULT;
				}
			}
			mutex_unlock(&ch9344->gpiomutex);
		} else if (ch9344->chiptype == CHIP_CH348L ||
			   ch9344->chiptype == CHIP_CH348Q) {
			gpionumber = inargH;
			mutex_lock(&ch9344->gpiomutex);
			rv = ch348_get_gpioval(ch9344);
			if (!rv) {
				gpioval = ((ch9344->gpiovalins) &
					   ((u64)1 << gpionumber)) ?
						  IO_H :
						  IO_L;
				if (put_user(gpioval, argval)) {
					rv = -EFAULT;
				}
			}
			mutex_unlock(&ch9344->gpiomutex);
		}
		break;
	default:
		break;
	}

out:
	kfree(buffer);
out_unfree:
	mutex_unlock(&ch9344->mutex);

	return rv;
}

#ifdef CONFIG_COMPAT
static long ch9344_compat_ioctl(struct file *file, unsigned int cmd,
				unsigned long arg)
{
	return ch9344_ioctl(file, cmd, arg);
}
#endif

static const struct file_operations ch9344_fops = {
	.owner = THIS_MODULE,
	.open = ch9344_open,
	.unlocked_ioctl = ch9344_ioctl,
#ifdef CONFIG_COMPAT
	.compat_ioctl = ch9344_compat_ioctl,
#endif
	.release = ch9344_release,
};

/*
 * usb class driver info in order to get a minor number from the usb core,
 * and to have the device registered with the driver core
 */
static struct usb_class_driver ch9344_class = {
	.name = "ch9344_iodev%d",
	.fops = &ch9344_fops,
	.minor_base = USB_MINOR_BASE,
};

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
	int portnum = 0;
	int cmdsize, readsize;
	u8 *buf;
	bool bpackload;
	char *buffer;
	unsigned long quirks;
	int num_rx_buf = CH9344_NR;
	int i, index;
	struct device *tty_dev;
	int rv = -ENOMEM;

	/* normal quirks */
	quirks = (unsigned long)id->driver_info;

	/* handle quirks deadly to normal probing*/
	data_interface = usb_ifnum_to_if(usb_dev, 0);

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
		swap(epread, epwrite);
	}

	if (!usb_endpoint_dir_in(epcmdread)) {
		/* descriptors are swapped */
		swap(epcmdread, epcmdwrite);
	}

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

	init_waitqueue_head(&ch9344->wcfgioctl);
	init_waitqueue_head(&ch9344->wgpioioctl);
	spin_lock_init(&ch9344->write_lock);
	spin_lock_init(&ch9344->read_lock);
	mutex_init(&ch9344->mutex);
	mutex_init(&ch9344->gpiomutex);

	ch9344->rx_endpoint =
		usb_rcvbulkpipe(usb_dev, epread->bEndpointAddress);
	ch9344->tx_endpoint =
		usb_sndbulkpipe(usb_dev, epwrite->bEndpointAddress);
	ch9344->cmdrx_endpoint =
		usb_rcvbulkpipe(usb_dev, epcmdread->bEndpointAddress);
	ch9344->cmdtx_endpoint =
		usb_sndbulkpipe(usb_dev, epcmdwrite->bEndpointAddress);
	ch9344->idProduct = id->idProduct;
	if (ch9344->idProduct == 0xe018) {
		rv = ch9344_enum_chiptype(ch9344);
		if (rv < 0)
			goto enum_fail;
		portnum = ch9344->num_ports = ch9344->port_offset = 4;
	} else if (ch9344->idProduct == 0x55d9) {
		rv = ch9344_enum_chiptype(ch9344);
		if (rv < 0)
			goto enum_fail;
		portnum = ch9344->num_ports = 8;
		ch9344->port_offset = 0;
	}

	buffer = kzalloc(ch9344->cmdsize, GFP_KERNEL);
	if (buffer == NULL)
		goto enum_fail;

	/* uart upload mode */
	if (ch9344->chiptype == CHIP_CH348L ||
	    ch9344->chiptype == CHIP_CH348Q) {
		if (ch9344->chipver >= 0x8a) {
			bpackload = true;
		}
	} else if ((ch9344->chiptype == CHIP_CH9344Q) ||
		   (ch9344->chiptype == CHIP_CH9344L &&
		    ch9344->chipver >= 0x39)) {
		bpackload = true;
	}

	if (bpackload) {
		memset(buffer, 0x00, ch9344->cmdsize);
		buffer[0] = CMD_WB_E + 0x00 + ch9344->port_offset;
		buffer[1] = R_UP_O;
		buffer[2] = 0x01;
		rv = ch9344_cmd_out(ch9344, buffer, 0x08);
		if (rv < 0)
			goto cmd_fail;
	}

	ch9344->opencounts = ch9344->num_ports;
	ch9344->modeline9 = true;

	for (i = 0; i < portnum; i++) {
		tty_port_init(&ch9344->ttyport[i].port);
		ch9344->ttyport[i].port.ops = &ch9344_port_ops;
		tty_set_portdata(&ch9344->ttyport[i], ch9344);
		ch9344->ttyport[i].portnum = i;
		ch9344->ttyport[i].write_empty = true;
		ch9344->ttyport[i].port.closing_wait =
			msecs_to_jiffies(DEFAULT_CLOSETIMEOUT);
		ch9344->ttyport[i].port.close_delay =
			msecs_to_jiffies(DEFAULT_CLOSETIMEOUT);
		init_waitqueue_head(&ch9344->ttyport[i].wioctl);
		init_waitqueue_head(&ch9344->ttyport[i].wmodemioctl);
		INIT_WORK(&ch9344->ttyport[i].work, ch9344_softint);
	}
	ch9344->gpio_recv = false;
	init_usb_anchor(&ch9344->delayed);
	ch9344->quirks = quirks;

	buf = usb_alloc_coherent(usb_dev, cmdsize, GFP_KERNEL,
				 &ch9344->cmdread_dma);
	if (!buf)
		goto alloc_fail2;
	ch9344->cmdread_buffer = buf;

	if (ch9344_write_buffers_alloc(ch9344) < 0)
		goto alloc_fail4;

	ch9344->cmdreadurb = usb_alloc_urb(0, GFP_KERNEL);
	if (!ch9344->cmdreadurb)
		goto alloc_fail5;

	for (i = 0; i < num_rx_buf; i++) {
		struct ch9344_rb *rb = &(ch9344->read_buffers[i]);
		struct urb *urb;

		rb->base = usb_alloc_coherent(ch9344->dev, readsize,
					      GFP_KERNEL, &rb->dma);
		if (!rb->base)
			goto alloc_fail6;
		rb->index = i;
		rb->instance = ch9344;

		urb = usb_alloc_urb(0, GFP_KERNEL);
		if (!urb)
			goto alloc_fail6;

		urb->transfer_flags |= URB_NO_TRANSFER_DMA_MAP;
		urb->transfer_dma = rb->dma;

		usb_fill_bulk_urb(urb, ch9344->dev, ch9344->rx_endpoint,
				  rb->base, ch9344->readsize,
				  ch9344_read_bulk_callback, rb);

		ch9344->read_urbs[i] = urb;
		__set_bit(i, &ch9344->read_urbs_free);
	}

	for (index = 0; index < ch9344->num_ports; index++) {
		for (i = 0; i < CH9344_NW; i++) {
			struct ch9344_wb *snd = &ch9344->wb[index][i];

			snd->urb = usb_alloc_urb(0, GFP_KERNEL);
			if (snd->urb == NULL)
				goto alloc_fail7;

			usb_fill_bulk_urb(
				snd->urb, usb_dev,
				usb_sndbulkpipe(usb_dev,
						epwrite->bEndpointAddress),
				NULL, ch9344->writesize, ch9344_write_bulk,
				snd);
			snd->urb->transfer_flags |=
				URB_NO_TRANSFER_DMA_MAP;
			snd->instance = ch9344;
		}
	}

	usb_set_intfdata(intf, ch9344);
	usb_fill_bulk_urb(
		ch9344->cmdreadurb, usb_dev,
		usb_rcvbulkpipe(usb_dev, epcmdread->bEndpointAddress),
		ch9344->cmdread_buffer, cmdsize, ch9344_cmd_irq, ch9344);
	ch9344->cmdreadurb->transfer_flags |= URB_NO_TRANSFER_DMA_MAP;
	ch9344->cmdreadurb->transfer_dma = ch9344->cmdread_dma;

	usb_driver_claim_interface(&ch9344_driver, data_interface, ch9344);
	usb_set_intfdata(data_interface, ch9344);
	usb_get_intf(data_interface);

	for (i = 0; i < portnum; i++) {
		ch9344->ttyport[i].line.dwDTERate = 9600;
#if (LINUX_VERSION_CODE >= KERNEL_VERSION(3, 7, 0))
		tty_dev = tty_port_register_device(
			&ch9344->ttyport[i].port, ch9344_tty_driver,
			NUMSTEP * minor + i, &data_interface->dev);
		if (IS_ERR(tty_dev)) {
			rv = PTR_ERR(tty_dev);
			goto alloc_fail7;
		}
#else
		tty_register_device(ch9344_tty_driver, NUMSTEP * minor + i,
				    &data_interface->dev);
#endif

#ifdef PACKLOAD
		rv = kfifo_alloc(&ch9344->ttyport[i].rfifo, FIFOSIZE,
				 GFP_KERNEL);
		if (rv) {
			dev_err(&ch9344->data->dev,
				"%s - kfifo_alloc failed\n", __func__);
			goto alloc_fail7;
		}
#endif
	}

	/* deal with urb when usb plugged in */
	rv = usb_submit_urb(ch9344->cmdreadurb, GFP_KERNEL);
	if (rv) {
		dev_err(&ch9344->data->dev,
			"%s - usb_submit_urb(ctrl cmd) failed\n",
			__func__);
		goto error_submit_urb;
	}

	rv = ch9344_submit_read_urbs(ch9344, GFP_KERNEL);
	if (rv)
		goto error_submit_read_urbs;

	/* register the device now, as it is ready */
	rv = usb_register_dev(intf, &ch9344_class);
	if (rv) {
		/* error when registering this driver */
		dev_err(&ch9344->data->dev,
			"Not able to get a minor for this device.\n");
		usb_set_intfdata(intf, NULL);
		goto error_submit_read_urbs;
	}

	ch9344->io_id = IOID;

	dev_info(&intf->dev,
		 "ttyCH9344USB from %d - %d: ch9344 device attached.\n",
		 NUMSTEP * minor, NUMSTEP * minor + portnum - 1);

	kfree(buffer);
	return 0;

error_submit_read_urbs:
#ifdef PACKLOAD
	for (i = 0; i < portnum; i++)
		kfifo_free(&ch9344->ttyport[i].rfifo);
#endif
	for (i = 0; i < ch9344->rx_buflimit; i++)
		usb_kill_urb(ch9344->read_urbs[i]);
error_submit_urb:
	usb_kill_urb(ch9344->cmdreadurb);
alloc_fail7:
	usb_set_intfdata(intf, NULL);
	for (index = 0; index < ch9344->num_ports; index++) {
		for (i = 0; i < CH9344_NW; i++)
			usb_free_urb(ch9344->wb[index][i].urb);
	}
alloc_fail6:
	for (i = 0; i < num_rx_buf; i++)
		usb_free_urb(ch9344->read_urbs[i]);
	ch9344_read_buffers_free(ch9344);
	usb_free_urb(ch9344->cmdreadurb);
alloc_fail5:
	ch9344_write_buffers_free(ch9344);
alloc_fail4:
	usb_free_coherent(usb_dev, cmdsize, ch9344->cmdread_buffer,
			  ch9344->cmdread_dma);
alloc_fail2:
cmd_fail:
	kfree(buffer);
enum_fail:
	ch9344_release_minor(ch9344);
	kfree(ch9344);
alloc_fail:
	return rv;
}

static void stop_data_traffic(struct ch9344 *ch9344)
{
	int i, index;
	struct urb *urb;
	struct ch9344_wb *wb;

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
	for (index = 0; index < ch9344->num_ports; index++) {
		for (i = 0; i < CH9344_NW; i++)
			usb_kill_urb(ch9344->wb[index][i].urb);
	}
	for (i = 0; i < ch9344->rx_buflimit; i++)
		usb_kill_urb(ch9344->read_urbs[i]);
	for (i = 0; i < ch9344->num_ports; i++)
		cancel_work_sync(&ch9344->ttyport[i].work);
}

static void ch9344_disconnect(struct usb_interface *intf)
{
	struct ch9344 *ch9344 = usb_get_intfdata(intf);
	struct usb_device *usb_dev = interface_to_usbdev(intf);
	struct tty_struct *tty;
	int i, index;
	int num_ports = ch9344->num_ports;

	/* sibling interface is already cleaning up */
	if (!ch9344)
		return;

	/* give back minor */
	usb_deregister_dev(intf, &ch9344_class);

	mutex_lock(&ch9344->mutex);
	ch9344->disconnected = true;
	wake_up_interruptible(&ch9344->wcfgioctl);
	wake_up_interruptible(&ch9344->wgpioioctl);
	usb_set_intfdata(ch9344->data, NULL);
	mutex_unlock(&ch9344->mutex);

	for (i = 0; i < num_ports; i++) {
		wake_up_interruptible(&ch9344->ttyport[i].wioctl);
		wake_up_interruptible(&ch9344->ttyport[i].wmodemioctl);
		tty = tty_port_tty_get(&ch9344->ttyport[i].port);
		if (tty) {
			tty_vhangup(tty);
			tty_kref_put(tty);
		}
	}
	stop_data_traffic(ch9344);

#ifdef PACKLOAD
	for (i = 0; i < num_ports; i++)
		kfifo_free(&ch9344->ttyport[i].rfifo);
#endif

	for (i = 0; i < num_ports; i++) {
		tty_unregister_device(ch9344_tty_driver,
				      NUMSTEP * ch9344->minor + i);
	}

	usb_free_urb(ch9344->cmdreadurb);
	for (index = 0; index < num_ports; index++) {
		for (i = 0; i < CH9344_NW; i++)
			usb_free_urb(ch9344->wb[index][i].urb);
	}

	for (i = 0; i < ch9344->rx_buflimit; i++)
		usb_free_urb(ch9344->read_urbs[i]);
	ch9344_write_buffers_free(ch9344);
	usb_free_coherent(usb_dev, ch9344->cmdsize, ch9344->cmdread_buffer,
			  ch9344->cmdread_dma);
	ch9344_read_buffers_free(ch9344);

	usb_driver_release_interface(&ch9344_driver, ch9344->data);

	for (i = 0; i < num_ports; i++) {
		tty_port_put(&ch9344->ttyport[i].port);
	}

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
out:
	spin_unlock_irq(&ch9344->write_lock);

	return rv;
}

static int ch9344_reset_resume(struct usb_interface *intf)
{
	struct ch9344 *ch9344 = usb_get_intfdata(intf);
	int i;
	int retval;

	for (i = 0; i < ch9344->num_ports; i++) {
#if (LINUX_VERSION_CODE >= KERNEL_VERSION(4, 7, 0))
		if (tty_port_initialized(&ch9344->ttyport[i].port)) {
#else
		if (test_bit(ASYNCB_INITIALIZED,
			     &ch9344->ttyport[i].port.flags)) {
#endif
			retval = ch9344_port_activate(
				&ch9344->ttyport[i].port,
				tty_port_tty_get(
					&ch9344->ttyport[i].port));
			if (retval)
				goto error_activate;
		}
	}
	return ch9344_resume(intf);

error_activate:
	return retval;
}

#endif /* CONFIG_PM */

/*
 * USB driver structure.
 */
static const struct usb_device_id ch9344_ids[] = {
	{
		USB_DEVICE(0x1a86, 0xe018),
	}, /* ch9344 chip */
	{
		USB_DEVICE(0x1a86, 0x55d9),
	}, /* ch348 chip */
	{}
};

MODULE_DEVICE_TABLE(usb, ch9344_ids);

static struct usb_driver ch9344_driver = {
	.name = "usb_ch9344",
	.probe = ch9344_probe,
	.disconnect = ch9344_disconnect,
#ifdef CONFIG_PM
	.suspend = ch9344_suspend,
	.resume = ch9344_resume,
	.reset_resume = ch9344_reset_resume,
#endif
	.id_table = ch9344_ids,
#ifdef CONFIG_PM
	.supports_autosuspend = 1,
#endif
#if (LINUX_VERSION_CODE >= KERNEL_VERSION(3, 5, 0))
	.disable_hub_initiated_lpm = 1,
#endif
};

/*
 * TTY driver structures.
 */
static const struct tty_operations ch9344_ops = {
	.install = ch9344_tty_install,
	.open = ch9344_tty_open,
	.close = ch9344_tty_close,
	.cleanup = ch9344_tty_cleanup,
	.hangup = ch9344_tty_hangup,
	.write = ch9344_tty_write,
	.write_room = ch9344_tty_write_room,
	.ioctl = ch9344_tty_ioctl,
	.chars_in_buffer = ch9344_tty_chars_in_buffer,
	.break_ctl = ch9344_tty_break_ctl,
	.set_termios = ch9344_tty_set_termios,
	.tiocmget = ch9344_tty_tiocmget,
	.tiocmset = ch9344_tty_tiocmset,
	.get_icount = ch9344_get_icount,
};

static int __init ch9344_init(void)
{
	int retval;

#if (LINUX_VERSION_CODE >= KERNEL_VERSION(5, 15, 0))
	ch9344_tty_driver = tty_alloc_driver(
		CH9344_TTY_MINORS,
		TTY_DRIVER_REAL_RAW | TTY_DRIVER_DYNAMIC_DEV);
	if (IS_ERR(ch9344_tty_driver))
		return PTR_ERR(ch9344_tty_driver);
#else
	ch9344_tty_driver = alloc_tty_driver(CH9344_TTY_MINORS);
	if (!ch9344_tty_driver)
		return -ENOMEM;
#endif
	ch9344_tty_driver->driver_name = "ch9344",
	ch9344_tty_driver->name = "ttyCH9344USB",
	ch9344_tty_driver->major = CH9344_TTY_MAJOR,
	ch9344_tty_driver->minor_start = 0,
	ch9344_tty_driver->type = TTY_DRIVER_TYPE_SERIAL,
	ch9344_tty_driver->subtype = SERIAL_TYPE_NORMAL,
#if (LINUX_VERSION_CODE < KERNEL_VERSION(5, 15, 0))
	ch9344_tty_driver->flags = TTY_DRIVER_REAL_RAW |
				   TTY_DRIVER_DYNAMIC_DEV;
#endif
	ch9344_tty_driver->init_termios = tty_std_termios;
	ch9344_tty_driver->init_termios.c_cflag = B115200 | CS8 | CREAD |
						  HUPCL | CLOCAL;
	ch9344_tty_driver->init_termios.c_lflag &=
		~(ECHO | ECHONL | ICANON | ISIG);
	ch9344_tty_driver->init_termios.c_oflag &= ~OPOST;

	tty_set_operations(ch9344_tty_driver, &ch9344_ops);

	retval = tty_register_driver(ch9344_tty_driver);
	if (retval) {
#if (LINUX_VERSION_CODE >= KERNEL_VERSION(5, 15, 0))
		tty_driver_kref_put(ch9344_tty_driver);
#else
		put_tty_driver(ch9344_tty_driver);
#endif
		return retval;
	}

	retval = usb_register(&ch9344_driver);
	if (retval) {
		tty_unregister_driver(ch9344_tty_driver);
#if (LINUX_VERSION_CODE >= KERNEL_VERSION(5, 15, 0))
		tty_driver_kref_put(ch9344_tty_driver);
#else
		put_tty_driver(ch9344_tty_driver);
#endif
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
#if (LINUX_VERSION_CODE >= KERNEL_VERSION(5, 15, 0))
	tty_driver_kref_put(ch9344_tty_driver);
#else
	put_tty_driver(ch9344_tty_driver);
#endif

#if (LINUX_VERSION_CODE >= KERNEL_VERSION(3, 9, 0))
	idr_destroy(&ch9344_minors);
#endif
	printk(KERN_INFO KBUILD_MODNAME ": "
					"ch9344 driver exit.\n");
}

module_init(ch9344_init);
module_exit(ch9344_exit);

MODULE_AUTHOR(DRIVER_AUTHOR);
MODULE_DESCRIPTION(DRIVER_DESC);
MODULE_VERSION(VERSION_DESC);
MODULE_LICENSE("GPL");
MODULE_ALIAS_CHARDEV_MAJOR(CH9344_TTY_MAJOR);
