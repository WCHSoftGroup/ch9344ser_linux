/*
 * application library of ch9344.
 *
 * Copyright (C) 2021 WCH.
 * Author: TECH39 <zhangj@wch.cn>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License.
 *
 * Cross-compile with cross-gcc -I /path/to/cross-kernel/include
 *
 * Version: V1.0
 *
 * Update Log:
 * V1.0 - initial version
 */

#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <unistd.h>
#include <fcntl.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <sys/ioctl.h>
#include "ch9344_lib.h"

static const char *device = "/dev/ttyCH343USB0";

/**
 * libch9344_open - open tty device
 * @devname: the device name to open
 *
 * In this demo device is opened blocked, you could modify it at will.
 */
int libch9344_open(const char *devname)
{
	int fd = open(devname, O_RDWR | O_NOCTTY | O_NDELAY);
	int flags = 0;

	if (fd < 0) {
		perror("open device failed");
		return -1;
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
	} else
		printf("tty device open successfully.\n");

	return fd;
}

/**
 * libch9344_close - close tty device
 * @fd: the device handle
 *
 * The function return 0 if success, others if fail.
 */
int libch9344_close(int fd)
{
	return close(fd);
}

/**
 * libch9344_gpioenable - gpio enable
 * @fd: file descriptor of tty device
 * @gpiogroup: gpio group of gpio0-11, 0 on gpio0-2, 1 on gpio 3-5,
 *             2 on gpio6-8, 3 on gpio9-11
 * @gpioenable: gpio enable value, 1 on enable, 0 on disable
 *
 * The function return 0 if success, others if fail.
 */
int libch9344_gpioenable(int fd, uint8_t gpiogroup, uint8_t gpioenable)
{
	unsigned long val = (gpiogroup << 8) | gpioenable;

	return ioctl(fd, IOCTL_CMD_GPIOENABLE, &val);
}

/**
 * libch9344_gpioenable - gpio enable
 * @fd: file descriptor of tty device
 * @gpionumber: gpio number of gpio0-11
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
 * libch9344_gpioset - gpio output
 * @fd: file descriptor of tty device
 * @gpionumber: gpio number of gpio0-11
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
 * @fd: file descriptor of tty device
 * @gpionumber: gpio number of gpio0-11
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