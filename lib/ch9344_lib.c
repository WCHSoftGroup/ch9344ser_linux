/*
 * application library of USB to Quad UARTs chip ch9344 and USB to Octal UARTs chip ch348.
 *
 * Copyright (C) 2024 Nanjing Qinheng Microelectronics Co., Ltd.
 * Web: http://wch.cn
 * Author: WCH <tech@wch.cn>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License.
 *
 * Cross-compile with cross-gcc -I /path/to/cross-kernel/include
 *
 * Version: V1.3
 *
 * Update Log:
 * V1.0 - initial version
 * V1.1 - add support for ch348
 * V1.2 - modify gpio operation
 * V1.3 - add support for ch9344q
 */

#include <fcntl.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/ioctl.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <unistd.h>

#include "ch9344_lib.h"

#define LIB_INFO "V1.3 On 2024.07"

#define CMD_EE_R 0x54
#define CMD_EE_W 0x5E

/**
 * libch9344_getinfo - get ch9344 library information
 */
const char *libch9344_getinfo(void)
{
	return LIB_INFO;
}

/**
 * libch9344_open - open ch9344 device
 * @devname: ch9344 tty device or gpio device name, tty device: /dev/tty*, gpio device: /dev/ch9344_iodev*
 *
 * In this demo device is opened blocked, you could modify it at will.
 */
int libch9344_open(const char *devname)
{
	int fd;
	int flags;

	if (strstr(devname, "tty")) {
		fd = open(devname, O_RDWR | O_NOCTTY | O_NDELAY);
		if (fd < 0) {
			printf("open tty device failed.\n");
			return fd;
		}

		flags = fcntl(fd, F_GETFL, 0);
		flags &= ~O_NONBLOCK;

		if (fcntl(fd, F_SETFL, flags) < 0) {
			printf("fcntl failed.\n");
			return -1;
		}

		if (isatty(fd) == 0) {
			printf("not tty device.\n");
			return -1;
		}
	} else {
		fd = open(devname, O_RDWR);
	}

	return fd;
}

/**
 * libch9344_close - close ch9344 device
 * @fd: file descriptor of ch9344 tty device or gpio device
 *
 * The function returns 0 if successful, others if fail.
 */
int libch9344_close(int fd)
{
	return close(fd);
}

/**
 * libch9344_get_chiptype - get chip model
 * @fd: file descriptor of ch9344 tty device or gpio device
 * @type: pointer to chip model
 *
 * The function returns 0 if successful, others if fail.
 */
int libch9344_get_chiptype(int fd, CH9344_CHIPTYPE *type)
{
	return ioctl(fd, IOCTL_CMD_GETCHIPTYPE, type);
}

/**
 * libch9344_get_uartindex - get uart index number
 *         0->UART0, 1->UART1, 2->UART2, 3->UART3,
 *         4->UART4, 5->UART5, 6->UART6, 7->UART7
 * @fd: file descriptor of ch9344 tty device
 * @index: pointer to uart index
 *
 * The function returns 0 if successful, others if fail.
 */
int libch9344_get_uartindex(int fd, uint8_t *index)
{
	return ioctl(fd, IOCTL_CMD_GETUARTINDEX, index);
}

/**
 * libch9344_control_msg_in - control trasfer in
 * @fd: file descriptor of ch9344 tty device or gpio device
 * @request: USB message request value
 * @requesttype: USB message request type value
 * @value: USB message value
 * @index: USB message index value
 * @data: pointer to the data to receive
 * @size: length in bytes of the data to receive
 *
 * The function returns the number of bytes transferred if successful. Otherwise, a negative
 * error number.
 */
int libch9344_control_msg_in(int fd, uint8_t request, uint8_t requesttype, uint16_t value, uint16_t index,
			     uint8_t *data, uint16_t size)
{
	struct _controlmsg {
		uint8_t request;
		uint8_t requesttype;
		uint16_t value;
		uint16_t index;
		uint16_t size;
		uint8_t data[0];
	} __attribute__((packed));

	int ret;
	struct _controlmsg *controlmsg;

	controlmsg = malloc(sizeof(struct _controlmsg) + size);
	if (!controlmsg)
		return -1;

	controlmsg->request = request;
	controlmsg->requesttype = requesttype;
	controlmsg->value = value;
	controlmsg->index = index;
	controlmsg->size = size;

	ret = ioctl(fd, IOCTL_CMD_CTRLIN, (unsigned long)controlmsg);
	if (ret <= 0) {
		goto exit;
	}

	memcpy(data, controlmsg->data, ret);

exit:
	free(controlmsg);
	return ret;
}

/**
 * libch9344_control_msg_out - control trasfer out
 * @fd: file descriptor of ch9344 tty device or gpio device
 * @request: USB message request value
 * @requesttype: USB message request type value
 * @value: USB message value
 * @index: USB message index value
 * @data: pointer to the data to send
 * @size: length in bytes of the data to send
 *
 * The function returns 0 if successful, others if fail.
 */
int libch9344_control_msg_out(int fd, uint8_t request, uint8_t requesttype, uint16_t value, uint16_t index,
			      uint8_t *data, uint16_t size)
{
	struct _controlmsg {
		uint8_t request;
		uint8_t requesttype;
		uint16_t value;
		uint16_t index;
		uint16_t size;
		uint8_t data[0];
	} __attribute__((packed));

	int ret;
	struct _controlmsg *controlmsg;

	controlmsg = malloc(sizeof(struct _controlmsg) + size);
	if (!controlmsg)
		return -1;

	controlmsg->request = request;
	controlmsg->requesttype = requesttype;
	controlmsg->value = value;
	controlmsg->index = index;
	controlmsg->size = size;
	memcpy(controlmsg->data, data, size);

	ret = ioctl(fd, IOCTL_CMD_CTRLOUT, (unsigned long)controlmsg);

	free(controlmsg);
	return ret;
}

/**
 * libch9344_cmd_msg_in - command trasfer in
 * @fd: file descriptor of ch9344 tty device or gpio device
 * @sdata: pointer to the data to send
 * @size: length in bytes of the data to send
 * @rdata: pointer to the data to receive
 *
 * The function returns the number of bytes received if successful. Otherwise, a negative
 * error number.
 */
int libch9344_cmd_msg_in(int fd, uint8_t *sdata, uint16_t size, uint8_t *rdata)
{
	struct _cmdmsg {
		uint16_t size;
		uint8_t rdata[256];
		uint8_t sdata[0];
	} __attribute__((packed));

	int ret;
	struct _cmdmsg *cmdmsg;

	cmdmsg = malloc(sizeof(struct _cmdmsg) + size);
	if (!cmdmsg)
		return -1;

	cmdmsg->size = size;
	memcpy(cmdmsg->sdata, sdata, size);

	ret = ioctl(fd, IOCTL_CMD_CMDIN, (unsigned long)cmdmsg);
	if (ret <= 0)
		goto exit;

	memcpy(rdata, cmdmsg->rdata, ret);

exit:
	free(cmdmsg);
	return ret;
}

/**
 * libch9344_cmd_msg_out - command trasfer out
 * @fd: file descriptor of ch9344 tty device or gpio device
 * @data: pointer to the data to send
 * @size: length in bytes of the data to send
 *
 * The function returns 0 if successful, others if fail.
 */
int libch9344_cmd_msg_out(int fd, uint8_t *data, uint16_t size)
{
	struct _cmdmsg {
		uint16_t size;
		uint8_t data[0];
	} __attribute__((packed));

	int ret;
	struct _cmdmsg *cmdmsg;

	cmdmsg = malloc(sizeof(struct _cmdmsg) + size);
	if (!cmdmsg)
		return -1;

	cmdmsg->size = size;
	memcpy(cmdmsg->data, data, size);

	ret = ioctl(fd, IOCTL_CMD_CMDOUT, (unsigned long)cmdmsg);

	free(cmdmsg);
	return ret;
}

/**
 * libch9344_gpioenable - gpio enable
 * @fd: file descriptor of ch9344 gpio device
 * @gpiogroup: gpio group
 * 			   CH9344: gpio0-11, 0 on gpio0-2, 1 on gpio3-5, 2 on gpio6-8, 3 on gpio9-11
 * 			   CH348L: gpio0-47, 0 on gpio0-7, 1 on gpio8-15, 2 on gpio16-23
 * 					   3 on gpio24-31, 4 on gpio32-39, 5 on gpio40-47
 * @gpioenable: gpio enable value
 * 				CH9344: 1 on gpios of group all enable, 0 on all disable
 * 				CH348L&Q: bits0-7 on gpio[0*N-7*N], 1 on enable, 0 on disable
 *
 * The function returns 0 if successful, others if fail.
 */
int libch9344_gpioenable(int fd, uint8_t gpiogroup, uint8_t gpioenable)
{
	unsigned long val = (gpiogroup << 8) | gpioenable;

	return ioctl(fd, IOCTL_CMD_GPIOENABLE, &val);
}

/**
 * libch9344_gpiodirset - gpio direction set
 * @fd: file descriptor of ch9344 gpio device
 * @gpionumber: gpio number
 * @gpiodir: gpio direction value, 1 on output, 0 on input
 *
 * The function returns 0 if successful, others if fail.
 */
int libch9344_gpiodirset(int fd, uint8_t gpionumber, uint8_t gpiodir)
{
	unsigned long val = (gpionumber << 8) | gpiodir;

	return ioctl(fd, IOCTL_CMD_GPIODIR, &val);
}

/**
 * libch9344_gpioset - gpio output level set
 * @fd: file descriptor of ch9344 gpio device
 * @gpionumber: gpio number
 * @gpioval: gpio output value, 1 on high, 0 on low
 *
 * The function returns 0 if successful, others if fail.
 */
int libch9344_gpioset(int fd, uint8_t gpionumber, uint8_t gpioval)
{
	unsigned long val = (gpionumber << 8) | gpioval;

	return ioctl(fd, IOCTL_CMD_GPIOSET, &val);
}

/**
 * libch9344_gpioget - get gpio input
 * @fd: file descriptor of ch9344 gpio device
 * @gpionumber: gpio number
 * @gpioval: pointer to gpio input value, 1 on high, 0 on low
 *
 * The function returns 0 if successful, others if fail.
 */
int libch9344_gpioget(int fd, uint8_t gpionumber, uint8_t *gpioval)
{
	unsigned long val = gpionumber << 8;

	if (ioctl(fd, IOCTL_CMD_GPIOGET, &val) != 0)
		return -1;
	*gpioval = (uint8_t)val;

	return 0;
}

/**
 * libch9344_get_gpio_count - get gpio amounts of specific chip model
 * @chiptype: chip model
 *
 * The function returns value larger then 0 if successful, -1 if fail.
 */
int libch9344_get_gpio_count(CH9344_CHIPTYPE chiptype)
{
	int ret;

	if (chiptype == CHIP_CH9344L || chiptype == CHIP_CH9344Q)
		ret = 12;
	else if (chiptype == CHIP_CH348L)
		ret = 48;
	else if (chiptype == CHIP_CH348Q)
		ret = 12;
	else {
		printf("current chip not support gpio function.\n");
		ret = -1;
	}
	return ret;
}

/**
 * libch9344_get_gpio_group - get gpio groups of specific chip model
 * @chiptype: chip model
 *
 * The function returns value larger then 0 if successful, -1 if fail.
 */
int libch9344_get_gpio_group(CH9344_CHIPTYPE chiptype)
{
	int ret;

	if (chiptype == CHIP_CH9344L || chiptype == CHIP_CH9344Q)
		ret = 4;
	else if (chiptype == CHIP_CH348L)
		ret = 6;
	else if (chiptype == CHIP_CH348Q)
		ret = 2;
	else {
		printf("current chip not support gpio function.\n");
		ret = -1;
	}
	return ret;
}

/**
 * libch9344_eeprom_read_byte - read one byte from eeprom area
 * @fd: file descriptor of ch9344 tty device
 * @offset: offset address of eeprom
 * @val: pointer to read value
 *
 * The function returns 0 if successful, others if fail.
 */
int libch9344_eeprom_read_byte(int fd, uint8_t offset, uint8_t *val)
{
	uint16_t index = 0, size;
	uint8_t data[5] = {0};
	uint8_t rbuf[256] = {0};
	int rlen;
	
	data[index++] = 0x90;
	data[index++] = 0xa9;
	data[index++] = 0x00;
	data[index++] = offset;
	data[index++] = 0x01;
	size = index;

	rlen = libch9344_cmd_msg_in(fd, data, size, rbuf);
	if (rlen != 0x04)
		goto out;
	*val = rbuf[3];

	return 0;
out:
	return -1;
}

/**
 * libch9344_eeprom_write_byte - write one byte to eeprom area
 * @fd: file descriptor of ch9344 tty device
 * @offset: offset address of eeprom
 * @val: value to write
 *
 * The function returns 0 if successful, others if fail.
 */
int libch9344_eeprom_write_byte(int fd, uint8_t offset, uint8_t val)
{
	int ret;
	uint16_t index = 0;
	uint16_t size;
	uint8_t data[5] = {0};
	uint8_t rbuf[256] = {0};

	data[index++] = 0x90;
	data[index++] = 0xa9;
	data[index++] = 0x01;
	data[index++] = offset;
	data[index++] = 0x01;
	data[index++] = val;
	size = index;

	ret = libch9344_cmd_msg_in(fd, data, size, rbuf);
	if (ret != 0x04) 
		goto out;

	return 0;
	
out:
	return ret;
}

/**
 * libch9344_eeprom_read_area - read bytes from eeprom area
 * @fd: file descriptor of ch9344 tty device
 * @offset: offset address of eeprom
 * @data: pointer to read values
 * @size: read length
 *
 * The function returns 0 if successful, others if fail.
 */
int libch9344_eeprom_read_area(int fd, uint8_t offset, uint8_t *data, uint8_t size)
{
	int ret;
	int i;
	uint8_t val;

	for (i = 0; i < size; i++) {
		ret = libch9344_eeprom_read_byte(fd, offset + i, &val);
		if (ret)
			return -1;
		*(data + i) = val;
	}

	return 0;
}

/**
 * libch9344_eeprom_write_area - write bytes to eeprom area
 * @fd: file descriptor of ch9344 tty device
 * @offset: offset address of eeprom
 * @data: values to write
 * @size: write length
 *
 * The function returns 0 if successful, others if fail.
 */
int libch9344_eeprom_write_area(int fd, uint8_t offset, uint8_t *data, uint8_t size)
{
	int ret;
	int i;

	for (i = 0; i < size; i++) {
		ret = libch9344_eeprom_write_byte(fd, offset + i, *(data + i));
		if (ret)
			return -1;
	}

	return 0;
}
