/*
 * ch9344 gpio application example
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
 * Update Log:
 * V1.0 - initial version
 */

#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <unistd.h>
#include <fcntl.h>
#include <errno.h>
#include <string.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <signal.h>
#include <getopt.h>
#include <linux/serial.h>
#include <sys/ioctl.h>
#include <termios.h>

#define IOCTL_MAGIC 'W'
#define IOCTL_CMD_GPIOENABLE 	_IOW(IOCTL_MAGIC, 0x80, uint16_t)
#define IOCTL_CMD_GPIODIR 		_IOW(IOCTL_MAGIC, 0x81, uint16_t)
#define IOCTL_CMD_GPIOSET		_IOW(IOCTL_MAGIC, 0x82, uint16_t)
#define IOCTL_CMD_GPIOGET		_IOWR(IOCTL_MAGIC, 0x83, uint16_t)

#define MAXGPIOGROUP 4
#define MAXGPIO 12

static const char *device = "/dev/ttyCH9344USB0";

/**
 * libtty_open - open tty device
 * @devname: the device name to open
 *
 * In this demo device is opened blocked, you could modify it at will.
 */
int libtty_open(const char *devname)
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
		printf("tty device test ok.\n");

	return fd;
}

/**
 * libtty_close - close tty device
 * @fd: the device handle
 *
 * The function return 0 if success, others if fail.
 */
int libtty_close(int fd)
{
	return close(fd);
}

/**
 * libtty_gpioenable - gpio enable
 * @fd: file descriptor of tty device
 * @gpiogroup: gpio group of gpio0-11, 0 on gpio0-2, 1 on gpio 3-5,
 *             2 on gpio6-8, 3 on gpio9-11
 * @gpioenable: gpio enable value, 1 on enable, 0 on disable
 *
 * The function return 0 if success, others if fail.
 */
int libtty_gpioenable(int fd, uint8_t gpiogroup, uint8_t gpioenable)
{
	unsigned long val = (gpiogroup << 8) | gpioenable;

	return ioctl(fd, IOCTL_CMD_GPIOENABLE, &val);
}

/**
 * libtty_gpioenable - gpio enable
 * @fd: file descriptor of tty device
 * @gpionumber: gpio number of gpio0-11
 * @gpiodir: gpio direction value, 1 on output, 0 on input
 *
 * The function return 0 if success, others if fail.
 */
int libtty_gpiodirset(int fd, uint8_t gpionumber, uint8_t gpiodir)
{
	unsigned long val = (gpionumber << 8) | gpiodir;

	return ioctl(fd, IOCTL_CMD_GPIODIR, &val);
}

/**
 * libtty_gpioset - gpio output
 * @fd: file descriptor of tty device
 * @gpionumber: gpio number of gpio0-11
 * @gpioval: gpio output value, 1 on high, 0 on low
 *
 * The function return 0 if success, others if fail.
 */
int libtty_gpioset(int fd, uint8_t gpionumber, uint8_t gpioval)
{
	unsigned long val = (gpionumber << 8) | gpioval;

	return ioctl(fd, IOCTL_CMD_GPIOSET, &val);
}

/**
 * libtty_gpioget - get gpio input
 * @fd: file descriptor of tty device
 * @gpionumber: gpio number of gpio0-11
 * @gpioval: pointer to gpio input value, 1 on high, 0 on low
 *
 * The function return 0 if success, others if fail.
 */
int libtty_gpioget(int fd, uint8_t gpionumber, uint8_t *gpioval)
{
	unsigned long val = gpionumber << 8;

	if (ioctl(fd, IOCTL_CMD_GPIOGET, &val) != 0)
		return -1;
	*gpioval = (uint8_t)val;

	return 0;
}

void libtty_gpiotest(int fd)
{
	char c;
	uint8_t gpioval;
	int i;
	int ret;

	while (1) {
		if (c != '\n')
			printf("press e to enable gpio, d to disable gpio, "
			       "o to set gpio output, i to set gpio input, "
			       "h to output high, l to low, g to get gpio, q to quit.\n");
		scanf("%c", &c);
		if (c == 'q')
			break;
		switch (c) {
		case 'e':
			for (i = 0; i < MAXGPIOGROUP; i++) {
				ret = libtty_gpioenable(fd, i, 0x01);
				if (ret != 0) {
					printf("gpio %d enable failed.\n", i);
					break;
				}
			}
			break;
		case 'd':
			for (i = 0; i < MAXGPIOGROUP; i++) {
				ret = libtty_gpioenable(fd, i, 0x00);
				if (ret != 0) {
					printf("gpio %d disable failed.\n", i);
					break;
				}
			}
			break;
		case 'o':
			for (i = 0; i < MAXGPIO; i++) {
				ret = libtty_gpiodirset(fd, i, 0x01);
				if (ret != 0) {
					printf("gpio %d direction output set failed.\n", i);
					break;
				}
			}
			break;
		case 'i':
			for (i = 0; i < MAXGPIO; i++) {
				ret = libtty_gpiodirset(fd, i, 0x00);
				if (ret != 0) {
					printf("gpio %d direction input set failed.\n", i);
					break;
				}
			}
			break;
		case 'h':
			for (i = 0; i < MAXGPIO; i++) {
				ret = libtty_gpioset(fd, i, 0x01);
				if (ret != 0) {
					printf("gpio %d level set failed.\n", i);
					break;
				}
			}
			break;
		case 'l':
			for (i = 0; i < MAXGPIO; i++) {
				ret = libtty_gpioset(fd, i, 0x00);
				if (ret != 0) {
					printf("gpio %d level set failed.\n", i);
					break;
				}
			}

			break;
		case 'g':
			for (i = 0; i < MAXGPIO; i++) {
				ret = libtty_gpioget(fd, i, &gpioval);
				if (ret != 0) {
					printf("gpio %d level get failed.\n", i);
					break;
				} else
					printf("gpio%d : %s\n", i, gpioval ? "high" : "low");
			}
			break;
		default:
			break;
		}
	}
}

int main(int argc, char *argv[])
{
	int fd;
	int ret;
	char c;

	fd = libtty_open(device);
	if (fd < 0) {
		printf("libtty_open error.\n");
		exit(0);
	}

	libtty_gpiotest(fd);

	ret = libtty_close(fd);
	if (ret != 0) {
		printf("libtty_close error.\n");
		exit(0);
	}
}