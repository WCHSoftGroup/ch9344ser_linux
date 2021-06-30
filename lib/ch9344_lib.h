#ifndef _CH9344_LIB_H
#define _CH9344_LIB_H

#define IOCTL_MAGIC 'W'
#define IOCTL_CMD_GPIOENABLE 	_IOW(IOCTL_MAGIC, 0x80, uint16_t)
#define IOCTL_CMD_GPIODIR 		_IOW(IOCTL_MAGIC, 0x81, uint16_t)
#define IOCTL_CMD_GPIOSET		_IOW(IOCTL_MAGIC, 0x82, uint16_t)
#define IOCTL_CMD_GPIOGET		_IOWR(IOCTL_MAGIC, 0x83, uint16_t)

#define MAXGPIOGROUP 4
#define MAXGPIO 12

/**
 * libch9344_open - open tty device
 * @devname: the device name to open
 *
 * In this demo device is opened blocked, you could modify it at will.
 */
extern int libch9344_open(const char *devname);

/**
 * libch9344_close - close tty device
 * @fd: the device handle
 *
 * The function return 0 if success, others if fail.
 */
extern int libch9344_close(int fd);

/**
 * libch9344_gpioenable - gpio enable
 * @fd: file descriptor of tty device
 * @gpiogroup: gpio group of gpio0-11, 0 on gpio0-2, 1 on gpio 3-5,
 *             2 on gpio6-8, 3 on gpio9-11
 * @gpioenable: gpio enable value, 1 on enable, 0 on disable
 *
 * The function return 0 if success, others if fail.
 */
extern int libch9344_gpioenable(int fd, uint8_t gpiogroup, uint8_t gpioenable);

/**
 * libch9344_gpioenable - gpio enable
 * @fd: file descriptor of tty device
 * @gpionumber: gpio number of gpio0-11
 * @gpiodir: gpio direction value, 1 on output, 0 on input
 *
 * The function return 0 if success, others if fail.
 */
extern int libch9344_gpiodirset(int fd, uint8_t gpionumber, uint8_t gpiodir);

/**
 * libch9344_gpioset - gpio output
 * @fd: file descriptor of tty device
 * @gpionumber: gpio number of gpio0-11
 * @gpioval: gpio output value, 1 on high, 0 on low
 *
 * The function return 0 if success, others if fail.
 */
extern int libch9344_gpioset(int fd, uint8_t gpionumber, uint8_t gpioval);

/**
 * libch9344_gpioget - get gpio input
 * @fd: file descriptor of tty device
 * @gpionumber: gpio number of gpio0-11
 * @gpioval: pointer to gpio input value, 1 on high, 0 on low
 *
 * The function return 0 if success, others if fail.
 */
extern int libch9344_gpioget(int fd, uint8_t gpionumber, uint8_t *gpioval);

#endif