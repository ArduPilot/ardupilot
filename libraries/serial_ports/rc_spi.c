/*******************************************************************************
* rc_spi.c
*
* Functions for interfacing with SPI1 on the beaglebone and Robotics Cape
*******************************************************************************/

#include "roboticscape/roboticscape.h"
#include "roboticscape/rc_defs.h"
#include "mmap/rc_mmap_gpio_adc.h"	// for toggling gpio pins
#include <stdint.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>	// for memset
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <linux/spi/spidev.h>

#define SPI10_PATH			"/dev/spidev1.0"
#define SPI11_PATH			"/dev/spidev1.1"
#define SPI_MAX_SPEED		24000000 	// 24mhz
#define SPI_MIN_SPEED		1000		// 1khz
#define SPI_BITS_PER_WORD 	8
#define SPI_BUF_SIZE		2

int fd[2];			// file descriptor for SPI1_PATH device cs0, cs1
int initialized[2];	// set to 1 after successful initialization 
int gpio_ss[2];		// holds gpio pins for slave select lines

struct spi_ioc_transfer xfer[2]; // ioctl transfer structs for tx & rx
char tx_buf[SPI_BUF_SIZE];
char rx_buf[SPI_BUF_SIZE];

/*******************************************************************************
* @ int rc_spi_init(ss_mode_t ss_mode, int spi_mode, int speed_hz, int slave)
*
* Functions for interfacing with SPI1 on the beaglebone and Robotics Cape
*******************************************************************************/
int rc_spi_init(ss_mode_t ss_mode, int spi_mode, int speed_hz, int slave){
	int bits = SPI_BITS_PER_WORD;
	int mode_proper;
	// sanity checks
	if(speed_hz>SPI_MAX_SPEED || speed_hz<SPI_MIN_SPEED){
		printf("ERROR: SPI speed_hz must be between %d & %d\n", SPI_MIN_SPEED,\
																SPI_MAX_SPEED);
		return -1;
	}
	if(rc_get_bb_model()!=BB_BLUE && slave==2 && ss_mode==SS_MODE_AUTO){
		printf("ERROR: Can't use SS_MODE_AUTO on slave 2 with Cape\n");
		return -1;
	}
	// switch 4 standard SPI modes 0-3. return error otherwise
	switch(spi_mode){
		case 0: mode_proper = SPI_MODE_0; break;
		case 1: mode_proper = SPI_MODE_1; break;
		case 2: mode_proper = SPI_MODE_2; break;
		case 3: mode_proper = SPI_MODE_3; break;
		default: 
			printf("ERROR: SPI mode must be 0, 1, 2, or 3\n");
			printf("check your device datasheet to see which to use\n");
			return -1;
	}
	// get file descriptor for spi1 device
	switch(slave){
	case 1: 
		fd[0] = open(SPI10_PATH, O_RDWR);
		if(fd[0] < 0) {
			printf("ERROR: %s missing\n", SPI10_PATH); 
		return -1; 
		}
		break;
	case 2:
		fd[1] = open(SPI11_PATH, O_RDWR);
		if(fd[1] < 0) {
			printf("ERROR: %s missing\n", SPI11_PATH); 
		return -1; 
		}
		break;
	default:
		printf("ERROR: SPI slave must be 1 or 2\n");
		return -1;
	}
	// set settings
	if(ioctl(fd[slave-1], SPI_IOC_WR_MODE, &mode_proper)<0){
		printf("can't set spi mode");
		close(fd[slave-1]);
		return -1;
	}if(ioctl(fd[slave-1], SPI_IOC_RD_MODE, &mode_proper)<0){
		printf("can't get spi mode");
		close(fd[slave-1]);
		return -1;
	}if(ioctl(fd[slave-1], SPI_IOC_WR_BITS_PER_WORD, &bits)<0){
		printf("can't set bits per word");
		close(fd[slave-1]);
		return -1;
	}if(ioctl(fd[slave-1], SPI_IOC_RD_BITS_PER_WORD, &bits)<0){
		printf("can't get bits per word");
		close(fd[slave-1]);
		return -1;
	} if(ioctl(fd[slave-1], SPI_IOC_WR_MAX_SPEED_HZ, &speed_hz)<0){
		printf("can't set max speed hz");
		close(fd[slave-1]);
		return -1;
	} if(ioctl(fd[slave-1], SPI_IOC_RD_MAX_SPEED_HZ, &speed_hz)<0){
		printf("can't get max speed hz");
		close(fd[slave-1]);
		return -1;
	}

	// store settings
	xfer[0].cs_change = 1;
	xfer[0].delay_usecs = 0;
	xfer[0].speed_hz = speed_hz;
	xfer[0].bits_per_word = SPI_BITS_PER_WORD;
	xfer[1].cs_change = 1;
	xfer[1].delay_usecs = 0;
	xfer[1].speed_hz = speed_hz;
	xfer[1].bits_per_word = SPI_BITS_PER_WORD;

	// set up slave select pins
	if(rc_get_bb_model()==BB_BLUE){
		gpio_ss[0] = BLUE_SPI_PIN_6_SS1;
		gpio_ss[1] = BLUE_SPI_PIN_6_SS2;
	}
	else{
		gpio_ss[0] = CAPE_SPI_PIN_6_SS1;
		gpio_ss[1] = CAPE_SPI_PIN_6_SS2;
	}
	if(ss_mode==SS_MODE_AUTO){
		if(rc_set_pinmux_mode(gpio_ss[slave-1], PINMUX_SPI)){
			printf("ERROR: failed to set slave select pin to SPI mode\n");
			return -1;
		}
	}
	else{
		rc_set_pinmux_mode(gpio_ss[slave-1], PINMUX_GPIO);
		if(rc_gpio_export(gpio_ss[slave-1])){
			printf("ERROR: failed to export gpio %d for manual slave select\n",\
															 gpio_ss[slave-1]);
			return -1;
		}
		rc_manual_deselect_spi_slave(slave);
	}
	// all done
	initialized[slave-1] = 1;
	return 0;
}

/*******************************************************************************
* int rc_spi_fd(int slave)
*
* Returns the file descriptor for spi1 once initialized.
* Use this if you want to do your own reading and writing to the bus instead
* of the basic functions defined here. If the bus has not been initialized, 
* return -1
*******************************************************************************/
int rc_spi_fd(int slave){
	switch(slave){
	case 1:
		if(initialized[0]==0){
			printf("ERROR: SPI1 slave 1 not initialized yet\n");
			return -1;
		}
		else return fd[0];
	case 2:
		if(initialized[1]==0){
			printf("ERROR: SPI1 slave 2 not initialized yet\n");
			return -1;
		}
		else return fd[1];
	}
	printf("ERROR: SPI Slave must be 1 or 2\n");
	return -1;
}

/*******************************************************************************
* @ int rc_spi_close(int slave)
*
* Closes the file descriptor and sets initialized to 0.
*******************************************************************************/
int rc_spi_close(int slave){
	switch(slave){
	case 1:
		rc_manual_deselect_spi_slave(slave);
		close(fd[0]);
		initialized[0] = 0;
		return 0;
	case 2:
		rc_manual_deselect_spi_slave(slave);
		close(fd[1]);
		initialized[1] = 0;
		return 0;
	}
	printf("ERROR: SPI Slave must be 1 or 2\n");
	return -1;
}

/*******************************************************************************
* @ int manual_select_spi1_slave(int slave)
*
* Selects a slave (1 or 2) by pulling the corresponding slave select pin
* to ground. It also ensures the other slave is not selected.
*******************************************************************************/
int rc_manual_select_spi_slave(int slave){
	switch(slave){
		case 1:
			rc_gpio_set_value_mmap(gpio_ss[0], LOW);
			rc_gpio_set_value_mmap(gpio_ss[1], HIGH);
			break;
		case 2:
			rc_gpio_set_value_mmap(gpio_ss[0], HIGH);
			rc_gpio_set_value_mmap(gpio_ss[1], LOW);
			break;
		default:
			printf("SPI slave number must be 1 or 2\n");
			return -1;
	}
	return 0;
}

/*******************************************************************************
* @ int rc_manual_deselect_spi_slave(int slave)
*
* Deselects a slave (1 or 2) by pulling the corresponding slave select pin
* to 3.3V.
*******************************************************************************/
int rc_manual_deselect_spi_slave(int slave){
	switch(slave){
		case 1:
			rc_gpio_set_value_mmap(gpio_ss[0], HIGH);
			break;
		case 2:
			rc_gpio_set_value_mmap(gpio_ss[1], HIGH);
			break;
		default:
			printf("ERROR: SPI slave number must be 1 or 2\n");
			return -1;
	}
	return 0;
}

/*******************************************************************************
* int rc_spi_send_bytes(char* data, int bytes, int slave)
*
* Like rc_uart_send_bytes, this lets you send any byte sequence you like.
*******************************************************************************/
int rc_spi_send_bytes(char* data, int bytes, int slave){
	int ret;
	// sanity checks
	if(slave!=1 && slave!=2){
		printf("ERROR: SPI slave must be 1 or 2\n");
		return -1;
	}
	if(initialized[slave-1]==0){
		printf("ERROR: SPI slave %d not yet initialized\n", slave);
		return -1;
	} 
	if(bytes<1){
		printf("ERROR: rc_spi_send_bytes, bytes to send must be >=1\n");
		return -1;
	}
	// fill in ioctl xfer struct. speed and bits were already set in initialize
	xfer[0].rx_buf = 0;
	xfer[0].tx_buf = (unsigned long) data;
	xfer[0].len = bytes;
	// send
	ret = ioctl(fd[slave-1], SPI_IOC_MESSAGE(1), xfer);
	if(ret<0){
		printf("ERROR: SPI_IOC_MESSAGE_FAILED\n");
		return -1;
	}
	return ret;
}

/*******************************************************************************
* int rc_spi_read_bytes(char* data, int bytes, int slave)
*
* Like rc_uart_read_bytes, this lets you read a byte sequence without sending.
*******************************************************************************/
int rc_spi_read_bytes(char* data, int bytes, int slave){
	int ret;
	// sanity checks
	if(slave!=1 && slave!=2){
		printf("ERROR: SPI slave must be 1 or 2\n");
		return -1;
	}
	if(initialized[slave-1]==0){
		printf("ERROR: SPI slave %d not yet initialized\n", slave);
		return -1;
	} 
	if(bytes<1){
		printf("ERROR: rc_spi_read_bytes, bytes to read must be >=1\n");
		return -1;
	}
	// fill in ioctl xfer struct. speed and bits were already set in initialize
	xfer[0].rx_buf = (unsigned long) data;;
	xfer[0].tx_buf = 0;
	xfer[0].len = bytes;
	// receive
	ret=ioctl(fd[slave-1], SPI_IOC_MESSAGE(1), xfer);
	if(ret<0){
		printf("ERROR: SPI_IOC_MESSAGE_FAILED\n");
		return -1;
	}
	return ret;
}

/*******************************************************************************
* @ int rc_spi_transfer(char* tx_data, int tx_bytes, char* rx_data, int slave)
*
* This is a generic wrapper for the ioctl spi transfer function. It lets the
* user send any sequence of bytes and read the response. The return value is
* the number of bytes received or -1 on error.
*******************************************************************************/
int rc_spi_transfer(char* tx_data, int tx_bytes, char* rx_data, int slave){
	int ret;
	// sanity checks
	if(slave!=1 && slave!=2){
		printf("ERROR: SPI slave must be 1 or 2\n");
		return -1;
	}
	if(initialized[slave-1]==0){
		printf("ERROR: SPI slave %d not yet initialized\n", slave);
		return -1;
	} 
	if(tx_bytes<1){
		printf("ERROR: spi1_transfer, bytes must be >=1\n");
	}
	// fill in send struct 
	xfer[0].tx_buf = (unsigned long) tx_data; 
	xfer[0].rx_buf = (unsigned long) rx_data;
	xfer[0].len = tx_bytes;
	ret=ioctl(fd[slave-1], SPI_IOC_MESSAGE(1), xfer);
	if(ret<0){
		printf("SPI_IOC_MESSAGE_FAILED\n");
		return -1;
	}
	return ret;
}

/*******************************************************************************
* int rc_spi_write_reg_byte(char reg_addr, char data, int slave)
*
* Used for writing a byte value to a register. This sends in order the address
* and byte to be written. It also sets the MSB of the register to 1 which 
* indicates a write operation on many ICs. If you do not want this particular 
* functionality, use spi1_send_bytes() to send a byte string of your choosing.
*******************************************************************************/
int rc_spi_write_reg_byte(char reg_addr, char data, int slave){
	// sanity checks
	if(slave!=1 && slave!=2){
		printf("ERROR: SPI slave must be 1 or 2\n");
		return -1;
	}
	if(initialized[slave-1]==0){
		printf("ERROR: SPI slave %d not yet initialized\n", slave);
		return -1;
	}
	//wipe TX buffer and fill in register address and data
	memset(tx_buf, 0, sizeof tx_buf);
	tx_buf[0] = reg_addr | 0x80; /// set MSBit = 1 to indicate it's a write
	tx_buf[1] = data;
	// fill in ioctl zfer struct. speed and bits were already set in initialize
	xfer[0].tx_buf = (unsigned long) tx_buf;
	xfer[0].len = 2;
	// send
	if(ioctl(fd[slave-1], SPI_IOC_MESSAGE(1), xfer)<0){
		printf("ERROR: SPI_IOC_MESSAGE_FAILED\n");
		return -1;
	}
	return 0;
}

/*******************************************************************************
* char rc_spi_read_reg_byte(char reg_addr, int slave)
*
* Reads a single character located at address reg_addr. This is accomplished
* by sending the reg_addr with the MSB set to 0 indicating a read on many
* ICs. 
*******************************************************************************/
char rc_spi_read_reg_byte(char reg_addr, int slave){
	// sanity checks
	if(slave!=1 && slave!=2){
		printf("ERROR: SPI slave must be 1 or 2\n");
		return -1;
	}
	if(initialized[slave-1]==0){
		printf("ERROR: SPI slave %d not yet initialized\n", slave);
		return -1;
	}
	// wipe buffers
	memset(tx_buf, 0, sizeof tx_buf);
	memset(rx_buf, 0, sizeof rx_buf);
	// fill in xfer struct 
	tx_buf[0] = reg_addr & 0x7f; // MSBit = 0 to indicate it's a read
	xfer[0].tx_buf = (unsigned long) tx_buf; 
	xfer[0].rx_buf = (unsigned long) rx_buf;
	xfer[0].len = 1;
	if(ioctl(fd[slave-1], SPI_IOC_MESSAGE(1), xfer)<0){
		printf("SPI_IOC_MESSAGE_FAILED\n");
		return -1;
	}
	return rx_buf[0];
}

/*******************************************************************************
* int rc_spi_read_reg_bytes(char reg_addr, char* data, int bytes, int slave)
*
* Reads multiple bytes located at address reg_addr. This is accomplished
* by sending the reg_addr with the MSB set to 0 indicating a read on many
* ICs. 
*******************************************************************************/
int rc_spi_read_reg_bytes(char reg_addr, char* data, int bytes, int slave){
	int ret;
	// sanity checks
	if(slave!=1 && slave!=2){
		printf("ERROR: SPI slave must be 1 or 2\n");
		return -1;
	}
	if(initialized[slave-1]==0){
		printf("ERROR: SPI slave %d not yet initialized\n", slave);
		return -1;
	} 
	if(bytes<1){
		printf("ERROR: spi1_read_reg_bytes, bytes must be >=1\n");
	}
	// wipe buffers
	memset(tx_buf, 0, sizeof tx_buf);
	memset(data, 0, bytes);
	// fill in send struct 
	tx_buf[0] = reg_addr & 0x7f; // MSBit = 0 to indicate it's a read
	xfer[0].tx_buf = (unsigned long) tx_buf; 
	xfer[0].rx_buf = 0;
	xfer[0].len = 1;
	// fill in receive struct
	xfer[1].tx_buf = 0;
	xfer[1].rx_buf = (unsigned long) data;
	xfer[1].len = bytes;
	ret=ioctl(fd[slave-1], SPI_IOC_MESSAGE(2), xfer);
	if (ret<0){
		printf("SPI_IOC_MESSAGE_FAILED\n");
		return -1;
	}
	return 0;
}


