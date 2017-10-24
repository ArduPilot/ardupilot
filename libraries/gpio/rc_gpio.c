/*******************************************************************************
* rc_gpio.c
*******************************************************************************/
/*******************************************************************************
* Thank you Derek Molloy for this excellent gpio code. Here is it distributed
* as a modified version for better integration into the rest of the library
*******************************************************************************/

#include "roboticscape/roboticscape.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <errno.h>
#include <unistd.h>
#include <fcntl.h>
#include <poll.h>

#define SYSFS_GPIO_DIR "/sys/class/gpio"
#define MAX_BUF 64

/****************************************************************
 * rc_gpio_export
 ****************************************************************/
int rc_gpio_export(unsigned int gpio){
	int fd, len;
	char buf[MAX_BUF];

	fd = open(SYSFS_GPIO_DIR "/export", O_WRONLY);
	if (fd < 0) {
		perror("gpio/export");
		return fd;
	}
	len = snprintf(buf, sizeof(buf), "%d", gpio);
	write(fd, buf, len);
	close(fd);

	return 0;
}

/****************************************************************
 * rc_gpio_unexport
 ****************************************************************/
int rc_gpio_unexport(unsigned int gpio){
	int fd, len;
	char buf[MAX_BUF];

	fd = open(SYSFS_GPIO_DIR "/unexport", O_WRONLY);
	if (fd < 0) {
		perror("gpio/export");
		return fd;
	}

	len = snprintf(buf, sizeof(buf), "%d", gpio);
	write(fd, buf, len);
	close(fd);
	return 0;
}

/****************************************************************
 * rc_gpio_set_dir
 ****************************************************************/
int rc_gpio_set_dir(int gpio, rc_pin_direction_t out_flag){
	int fd;
	char buf[MAX_BUF];
	snprintf(buf, sizeof(buf), "/sys/class/gpio/gpio%i/direction", gpio);
	fd = open(buf, O_WRONLY);
	//printf("%d\n", gpio);
	if (fd < 0) {
		perror("gpio/direction");
		return fd;
	}

	if (out_flag == OUTPUT_PIN)
		write(fd, "out", 4);
	else
		write(fd, "in", 3);

	close(fd);
	return 0;
}

/****************************************************************
 * rc_gpio_set_value
 ****************************************************************/
int rc_gpio_set_value(unsigned int gpio, int value){
	int fd;
	char buf[MAX_BUF];

	snprintf(buf, sizeof(buf), SYSFS_GPIO_DIR "/gpio%d/value", gpio);

	fd = open(buf, O_WRONLY);
	if (fd < 0) {
		perror("gpio/set-value");
		return fd;
	}

	if(value)
		write(fd, "1", 2);
	else
		write(fd, "0", 2);

	close(fd);
	return 0;
}

/****************************************************************
 * rc_gpio_get_value
 ****************************************************************/
int rc_gpio_get_value(unsigned int gpio){
	int fd, ret;
	char buf[MAX_BUF];
	char ch;

	snprintf(buf, sizeof(buf), SYSFS_GPIO_DIR "/gpio%d/value", gpio);

	fd = open(buf, O_RDONLY);
	if (fd < 0) {
		perror("gpio/get-value");
		return fd;
	}

	read(fd, &ch, 1);

	if (ch != '0') ret = 1;
	else ret = 0;

	close(fd);
	return ret;
}


/****************************************************************
 * rc_gpio_set_edge
 ****************************************************************/

int rc_gpio_set_edge(unsigned int gpio, rc_pin_edge_t edge){
	int fd, ret, bytes;
	char buf[MAX_BUF];

	snprintf(buf, sizeof(buf), SYSFS_GPIO_DIR "/gpio%d/edge", gpio);

	fd = open(buf, O_WRONLY);
	if (fd < 0) {
		perror("gpio/set-edge");
		return fd;
	}

	switch(edge){
		case EDGE_NONE:
			bytes=5;
			ret=write(fd, "none", bytes);
			break;
		case EDGE_RISING:
			bytes=7;
			ret=write(fd, "rising", bytes);
			break;
		case EDGE_FALLING:
			bytes=8;
			ret=write(fd, "falling", bytes);
			break;
		case EDGE_BOTH:
			bytes=5;
			ret=write(fd, "both", bytes);
			break;
		default:
			printf("ERROR: invalid edge direction\n");
			return -1;
	}
	
	if(ret!=bytes){
		printf("ERROR: failed to set edge\n");
		close(fd);
		return -1;
	}

	close(fd);
	return 0;
}

/****************************************************************
 * rc_gpio_fd_open
 ****************************************************************/

int rc_gpio_fd_open(unsigned int gpio)
{
	int fd;
	char buf[MAX_BUF];

	snprintf(buf, sizeof(buf), SYSFS_GPIO_DIR "/gpio%d/value", gpio);

	fd = open(buf, O_RDONLY | O_NONBLOCK );
	if (fd < 0) {
		perror("gpio/fd_open");
	}
	return fd;
}

/****************************************************************
 * rc_gpio_fd_close
 ****************************************************************/

int rc_gpio_fd_close(int fd){
	return close(fd);
}


