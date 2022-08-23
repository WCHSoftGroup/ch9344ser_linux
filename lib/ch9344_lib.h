#ifndef _CH9344_LIB_H
#define _CH9344_LIB_H

#define IOCTL_MAGIC           'W'
#define IOCTL_CMD_GPIOENABLE  _IOW(IOCTL_MAGIC, 0x80, uint16_t)
#define IOCTL_CMD_GPIODIR     _IOW(IOCTL_MAGIC, 0x81, uint16_t)
#define IOCTL_CMD_GPIOSET     _IOW(IOCTL_MAGIC, 0x82, uint16_t)
#define IOCTL_CMD_GPIOGET     _IOWR(IOCTL_MAGIC, 0x83, uint16_t)
#define IOCTL_CMD_GETCHIPTYPE _IOR(IOCTL_MAGIC, 0x84, uint16_t)

typedef enum {
    CHIP_CH9344 = 0,
    CHIP_CH348L = 1,
    CHIP_CH348Q = 2,
} CHIPTYPE;

/**
 * libch9344_open - open ch9344 gpio device
 * @devname: the device name to open
 *
 * The function return 0 if success, others if fail.
 */
extern int libch9344_open(const char *devname);

/**
 * libch9344_close - close ch9344 gpio device
 * @fd: the device handle
 *
 * The function return 0 if success, others if fail.
 */
extern int libch9344_close(int fd);

/**
 * libch9344_gpioenable - gpio enable
 * @fd: file descriptor of gpio device
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
extern int libch9344_gpioenable(int fd, uint8_t gpiogroup, uint8_t gpioenable);

/**
 * libch9344_gpiodirset - gpio direction set
 * @fd: file descriptor of gpio device
 * @gpionumber: gpio number
 * @gpiodir: gpio direction value, 1 on output, 0 on input
 *
 * The function return 0 if success, others if fail.
 */
extern int libch9344_gpiodirset(int fd, uint8_t gpionumber, uint8_t gpiodir);

/**
 * libch9344_gpioset - gpio output level set
 * @fd: file descriptor of gpio device
 * @gpionumber: gpio number
 * @gpioval: gpio output value, 1 on high, 0 on low
 *
 * The function return 0 if success, others if fail.
 */
extern int libch9344_gpioset(int fd, uint8_t gpionumber, uint8_t gpioval);

/**
 * libch9344_gpioget - get gpio input
 * @fd: file descriptor of gpio device
 * @gpionumber: gpio number
 * @gpioval: pointer to gpio input value, 1 on high, 0 on low
 *
 * The function return 0 if success, others if fail.
 */
extern int libch9344_gpioget(int fd, uint8_t gpionumber, uint8_t *gpioval);

/**
 * libch9344_get_chiptype - get chip model
 * @fd: file descriptor of gpio device
 * @type: pointer to chip model
 *
 * The function return 0 if success, others if fail.
 */
extern int libch9344_get_chiptype(int fd, CHIPTYPE *type);

/**
 * libch9344_get_gpio_count - get gpio amounts of specific chip model
 * @chiptype: chip model
 *
 * The function return value larger then 0 if success, -1 if fail.
 */
extern int libch9344_get_gpio_count(CHIPTYPE chiptype);

/**
 * libch9344_get_gpio_group - get gpio groups of specific chip model
 * @chiptype: chip model
 *
 * The function return value larger then 0 if success, -1 if fail.
 */
extern int libch9344_get_gpio_group(CHIPTYPE chiptype);

#endif
