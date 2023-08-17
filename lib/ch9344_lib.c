/*
 * application library of USB to Quad UARTs chip ch9344 and USB to Octal UARTs chip ch348.
 *
 * Copyright (C) 2023 Nanjing Qinheng Microelectronics Co., Ltd.
 * Web: http://wch.cn
 * Author: WCH <tech@wch.cn>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License.
 *
 * Cross-compile with cross-gcc -I /path/to/cross-kernel/include
 *
 * Version: V1.2
 *
 * Update Log:
 * V1.0 - initial version
 * V1.1 - add support for ch348
 * V1.2 - modify gpio operation
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
 * The function return 0 if success, others if fail.
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
 * The function return 0 if success, others if fail.
 */
int libch9344_get_chiptype(int fd, CHIPTYPE *type)
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
 * The function return 0 if success, others if fail.
 */
int libch9344_get_uartindex(int fd, uint8_t *index)
{
	return ioctl(fd, IOCTL_CMD_GETUARTINDEX, index);
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
 * The function return 0 if success, others if fail.
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
 * The function return 0 if success, others if fail.
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
 * The function return 0 if success, others if fail.
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
 * The function return 0 if success, others if fail.
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
 * The function return value larger then 0 if success, -1 if fail.
 */
int libch9344_get_gpio_count(CHIPTYPE chiptype)
{
	int ret;

	if (chiptype == CHIP_CH9344)
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
 * The function return value larger then 0 if success, -1 if fail.
 */
int libch9344_get_gpio_group(CHIPTYPE chiptype)
{
	int ret;

	if (chiptype == CHIP_CH9344)
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
