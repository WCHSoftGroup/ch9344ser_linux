/*
 * ch9344 gpio application example for USB to Quad UARTs chip ch9344 and USB to Octal UARTs chip ch348.
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

#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>

#include "ch9344_lib.h"

static CHIPTYPE chiptype;
static int gpiocount;
static int gpiogroup;

void libch9344_gpiotest(int fd)
{
	char c;
	uint8_t gpioval;
	int i;
	int ret;

	ret = libch9344_get_chiptype(fd, &chiptype);
	if (ret) {
		return;
	}

	gpiocount = libch9344_get_gpio_count(chiptype);
	if (gpiocount <= 0) {
		printf("get gpio count error.\n");
		return;
	}
	printf("current chip has %d gpios.\n", gpiocount);

	gpiogroup = libch9344_get_gpio_group(chiptype);
	if (gpiogroup <= 0) {
		printf("get gpio group error.\n");
		return;
	}
	printf("current chip has %d gpio groups.\n", gpiogroup);

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
			for (i = 0; i < gpiogroup; i++) {
				if (chiptype == CHIP_CH9344)
					ret = libch9344_gpioenable(fd, i, 0x01);
				else
					ret = libch9344_gpioenable(fd, i, 0xff);
				if (ret != 0) {
					printf("gpio %d enable failed.\n", i);
					break;
				}
			}
			break;
		case 'd':
			for (i = 0; i < gpiogroup; i++) {
				ret = libch9344_gpioenable(fd, i, 0x00);
				if (ret != 0) {
					printf("gpio %d disable failed.\n", i);
					break;
				}
			}
			break;
		case 'o':
			for (i = 0; i < gpiocount; i++) {
				ret = libch9344_gpiodirset(fd, i, 0x01);
				if (ret != 0) {
					printf("gpio %d direction output set failed.\n", i);
					break;
				}
			}
			break;
		case 'i':
			for (i = 0; i < gpiocount; i++) {
				ret = libch9344_gpiodirset(fd, i, 0x00);
				if (ret != 0) {
					printf("gpio %d direction input set failed.\n", i);
					break;
				}
			}
			break;
		case 'h':
			for (i = 0; i < gpiocount; i++) {
				ret = libch9344_gpioset(fd, i, 0x01);
				if (ret != 0) {
					printf("gpio %d level set failed.\n", i);
					break;
				}
			}
			break;
		case 'l':
			for (i = 0; i < gpiocount; i++) {
				ret = libch9344_gpioset(fd, i, 0x00);
				if (ret != 0) {
					printf("gpio %d level set failed.\n", i);
					break;
				}
			}
			break;
		case 'g':
			for (i = 0; i < gpiocount; i++) {
				ret = libch9344_gpioget(fd, i, &gpioval);
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

	if (argc != 2) {
		printf("Usage: sudo %s [device]\n", argv[0]);
		return -1;
	}

	if (!strstr(argv[1], "iodev")) {
        printf("the gpio device is named ch9344_iodev*\n");
        return -1;
    }

	fd = libch9344_open(argv[1]);
	if (fd < 0) {
		printf("libch9344_open error.\n");
		return fd;
	}

	libch9344_gpiotest(fd);

	ret = libch9344_close(fd);
	if (ret != 0) {
		printf("libch9344_close error.\n");
		goto exit;
	}

exit:
	return ret;
}
