/*******************************************************************************
* rc_uart.c
*
* This is a collection of C functions to make interfacing with UART ports on 
* the BeagleBone easier. This could be used on other linux platforms too.
*******************************************************************************/

#include "roboticscape/roboticscape.h"
#include <stdio.h>
#include <termios.h>
#include <errno.h>
#include <time.h>
#include <sys/time.h> // for timeval
#include <fcntl.h> // for open
#include <unistd.h> // for close
#include <string.h>
#include <sys/ioctl.h>
#include <math.h>

#define MIN_BUS 0
#define MAX_BUS 5

// Most bytes to read at once. This is the size of the Sitara UART FIFO buffer.
#define MAX_READ_LEN 128

/*******************************************************************************
* Local Global Variables
*******************************************************************************/
int initialized[MAX_BUS-MIN_BUS+1]; // keep track of if a bus is initialized
char *paths[6] = { \
	"/dev/ttyO0", \
	"/dev/ttyO1", \
	"/dev/ttyO2", \
	"/dev/ttyO3", \
	"/dev/ttyO4", \
	"/dev/ttyO5" };

int fd[6]; // file descriptors for all ports
float bus_timeout_s[6]; // user-requested timeout in seconds for each bus

/*******************************************************************************
* int rc_uart_init(int bus, int baudrate, float timeout_s)
* 
* bus needs to be between MIN_BUS and MAX_BUS which here is 0 & 5.
* baudrate must be one of the standard speeds in the UART spec. 115200 and 
* 57600 are most common.
* timeout is in seconds and must be >=0.1
*
* returns -1 for failure or 0 for success
*******************************************************************************/ 
int rc_uart_init(int bus, int baudrate, float timeout_s){

	struct termios config;
	speed_t speed; //baudrate
	
	// sanity checks
	if(bus<MIN_BUS || bus>MAX_BUS){
		printf("ERROR: uart bus must be between %d & %d\n", MIN_BUS, MAX_BUS);
		return -1;
	}
	if(timeout_s<0.1){
		printf("ERROR: timeout must be >=0.1 seconds\n");
		return -1;
	}
	
	switch(baudrate){
	case (230400): 
		speed=B230400;
		break;
	case (115200): 
		speed=B115200;
		break;
	case (57600): 
		speed=B57600;
		break;
	case (38400): 
		speed=B38400;
		break;
	case (19200): 
		speed=B19200;
		break;
	case (9600): 
		speed=B9600;
		break;
	case (4800): 
		speed=B4800;
		break;
	case (2400): 
		speed=B2400;
		break;
	case (1800): 
		speed=B1800;
		break;
	case (1200): 
		speed=B1200;
		break;
	case (600): 
		speed=B600;
		break;
	case (300): 
		speed=B300;
		break;
	case (200): 
		speed=B200;
		break;
	case (150): 
		speed=B150;
		break;
	case (134): 
		speed=B134;
		break;
	case (110): 
		speed=B110;
		break;
	case (75): 
		speed=B75;
		break;
	case (50): 
		speed=B50;
		break;
	default:
		printf("ERROR: invalid speed. Please use a standard baudrate\n");
		return -1;
	}
	
	// close the bus in case it was already open
	rc_uart_close(bus);
	
	// open file descriptor for blocking reads
	if ((fd[bus] = open(paths[bus], O_RDWR | O_NOCTTY | O_NDELAY)) < 0) {
		printf("error opening uart%d in /dev/\n", bus);
		printf("device tree probably isn't loaded\n");
		return -1;
	}
	
	// get current attributes
	if (tcgetattr(fd[bus],&config)!=0){
		close(fd[bus]);
		printf("Cannot get uart attributes\n");
		return -1;
	}
	
	// set up tc config
	memset(&config,0,sizeof(config));
	config.c_iflag=0;
	config.c_oflag=0;
	config.c_lflag = 0;
	config.c_cflag = 0;
	//turning off these settings really does nothing since we just set
	// all the flags to 0, but they are here for reference and completel
	config.c_lflag &= ~ICANON;	// turn off canonical read
	config.c_cflag &= ~PARENB;  // no parity
	config.c_cflag &= ~CSTOPB;  // disable 2 stop bits (use just 1)
	config.c_cflag &= ~CSIZE;  	// wipe all size masks
	config.c_cflag |= CS8;		// set size to 8 bit characters
	config.c_cflag |= CREAD;    // enable reading
	config.c_cflag |= CLOCAL;	// ignore modem status lines
	
	// convert float timeout in seconds to int timeout in tenths of a second
	int tenths = (timeout_s*10);

	//config.c_cc[VTIME]=0;
	// if VTIME>0 & VMIN>0, read() will return when either the requested number
	// of bytes are ready or when VMIN bytes are ready, whichever is smaller.
	// since we set VMIN to the size of the buffer, read() should always return
	// when the user's requested number of bytes are ready.
	config.c_cc[VMIN]=MAX_READ_LEN;
	//config.c_cc[VMIN] = 0;
	config.c_cc[VTIME] = tenths+1;
	
	if(cfsetispeed(&config, speed) < 0) {
		printf("ERROR: cannot set uart%d baud rate\n", bus);
		return -1;
	}
	if(cfsetospeed(&config, speed) < 0) {
		printf("ERROR: cannot set uart%d baud rate\n", bus);
		return -1;
	}
	
	tcflush(fd[bus],TCIOFLUSH);
	if(tcsetattr(fd[bus], TCSANOW, &config) < 0) { 
		printf("cannot set uart%d attributes\n", bus);
		close(fd[bus]);
		return -1;
	}
	tcflush(fd[bus],TCIOFLUSH);

	// turn off the FNDELAY flag
	fcntl(fd[bus], F_SETFL, 0);
	
	initialized[bus] = 1;
	bus_timeout_s[bus]=timeout_s;
	
	rc_uart_flush(bus);
	return 0;
}


/*******************************************************************************
*	int rc_uart_close(int bus)
*
* If the bus is open and has been initialized, close it and return 0.
* If the bus in uninitialized, just return right away.
* Return -1 if bus is out of bounds.
*******************************************************************************/
int rc_uart_close(int bus){
	// sanity checks
	if(bus<MIN_BUS || bus>MAX_BUS){
		printf("ERROR: uart bus must be between %d & %d\n", MIN_BUS, MAX_BUS);
		return -1;
	}
	
	// if not initialized already, return
	if(initialized[bus]==0){
		return 0;
	}
	tcflush(fd[bus],TCIOFLUSH);
	close(fd[bus]);
	initialized[bus]=0;
	return 0;
}




/*******************************************************************************
* int rc_uart_fd(int bus)
*
* Returns the file descriptor for a uart bus once it has been initialized.
* use this if you want to do your own reading and writing to the bus instead
* of the basic functions defined here. If the bus has not been initialized, 
* return -1;
*******************************************************************************/
int rc_uart_fd(int bus){
	// sanity checks
	if(bus<MIN_BUS || bus>MAX_BUS){
		printf("ERROR: uart bus must be between %d & %d\n", MIN_BUS, MAX_BUS);
		return -1;
	}
	
	if (initialized[bus]==0){
		printf("ERROR: uart%d not initialized yet\n", bus);
		return -1;
	}
	
	return fd[bus];
}

/*******************************************************************************
* int rc_uart_flush(int bus)
*
* flushes (discards) any data received but not read. Or written but not sent.
*******************************************************************************/
int rc_uart_flush(int bus){
	// sanity checks
	if(bus<MIN_BUS || bus>MAX_BUS){
		printf("ERROR: uart bus must be between %d & %d\n", MIN_BUS, MAX_BUS);
		return -1;
	}
	if(initialized[bus]==0){
		printf("ERROR: uart%d must be initialized first\n", bus);
		return -1;
	}
	return tcflush(fd[bus],TCIOFLUSH);
}

/*******************************************************************************
*	int rc_uart_send_bytes(int bus, int bytes, char* data);
*
* This is essentially a wrapper for the linux write() function with some sanity
* checks. Returns -1 on error, otherwise returns number of bytes sent.
*******************************************************************************/
int rc_uart_send_bytes(int bus, int bytes, char* data){
	// sanity checks
	if(bus<MIN_BUS || bus>MAX_BUS){
		printf("ERROR: uart bus must be between %d & %d\n", MIN_BUS, MAX_BUS);
		return -1;
	}
	if(bytes<1){
		printf("ERROR: number of bytes to send must be >1\n");
		return -1;
	}
	if(initialized[bus]==0){
		printf("ERROR: uart%d must be initialized first\n", bus);
		return -1;
	}
	
	return write(fd[bus], data, bytes);
}

/*******************************************************************************
* int rc_uart_send_byte(int bus, char data);
*
* This is essentially a wrapper for the linux write() function with some sanity
* checks. Returns -1 on error, otherwise returns number of bytes sent.
*******************************************************************************/
int rc_uart_send_byte(int bus, char data){
	
	// sanity checks
	if(bus<MIN_BUS || bus>MAX_BUS){
		printf("ERROR: uart bus must be between %d & %d\n", MIN_BUS, MAX_BUS);
		return -1;
	}
	
	if(initialized[bus]==0){
		printf("ERROR: uart%d must be initialized first\n", bus);
		return -1;
	}
	
	return write(fd[bus], &data, 1);
}
		

/*******************************************************************************
* int rc_uart_read_bytes(int bus, int bytes, char* buf)
*
* This is a blocking function call. It will only return once the desired number
* of bytes has been read from the buffer or if the global flow state defined
* in robotics_cape.h is set to EXITING.
* Due to the Sitara's UART FIFO buffer, MAX_READ_LEN (128bytes) is the largest
* packet that can be read with a single call to read(). For reads larger than 
* 128bytes, we run a loop instead.
*******************************************************************************/
int rc_uart_read_bytes(int bus, int bytes, char* buf){
	int bytes_to_read, ret;
	// sanity checks
	if(bus<MIN_BUS || bus>MAX_BUS){
		printf("ERROR: uart bus must be between %d & %d\n", MIN_BUS, MAX_BUS);
		return -1;
	}
	if(bytes<1){
		printf("ERROR: number of bytes to read must be >1\n");
		return -1;
	}
	if(initialized[bus]==0){
		printf("ERROR: uart%d must be initialized first\n", bus);
		return -1;
	}
	
	// // a single call to 'read' just isn't reliable, don't do it
	// if(bytes<=MAX_READ_LEN){
	// 	// small read, return in one read() call
	// 	// this uses built-in timeout instead of select()
	// 	ret = read(fd[bus], buf, bytes);
	// 	return ret;
	// }

	// any read under 128 bytes should have returned by now.
	// everything below this line is for longer extended reads >128 bytes
	
	
	fd_set set; // for select()
	struct timeval timeout;
	int bytes_read; // number of bytes read so far
	int bytes_left; // number of bytes still need to be read
	
	bytes_read = 0;
	bytes_left = bytes;

	// set up the timeout OUTSIDE of the read loop. We will likely be calling
	// select() multiple times and that will decrease the timeout struct each
	// time ensuring the TOTAL timeout requested by the user is honoured instead
	// of the timeout value compounding each loop.
	timeout.tv_sec = (int)bus_timeout_s[bus];
	timeout.tv_usec = (int)(1000000*fmod(bus_timeout_s[bus],1));
	
	// exit the read loop once enough bytes have been read
	// or the global flow state becomes EXITING. This prevents programs
	// getting stuck here and not exiting properly
	while((bytes_left>0)&&rc_get_state()!=EXITING){
		FD_ZERO(&set); /* clear the set */
		FD_SET(fd[bus], &set); /* add our file descriptor to the set */
		ret = select(fd[bus] + 1, &set, NULL, NULL, &timeout);
		if(ret == -1){
			// select returned and error. EINTR means interrupted by SIGINT
			// aka ctrl-c. Don't print anything as this happens normally
			// in case of EINTR/Ctrl-C just return how many bytes got read up 
			// until then without raising alarms.
			if(errno!=EINTR){
				printf("uart select() error: %s\n", strerror(errno));
				return -1;
			}
			return bytes_read;  
		}
		else if(ret == 0){
			// timeout
			return bytes_read;
		}
		else{
			// There was data to read. Read up to the number of bytes left
			// and no more. This most likely will return fewer bytes than
			// bytes_left, but we will loop back to get the rest.
			
			// read no more than MAX_READ_LEN at a time
			if(bytes_left>MAX_READ_LEN)	bytes_to_read = MAX_READ_LEN;
			else bytes_to_read = bytes_left;
			ret=read(fd[bus], buf+bytes_read, bytes_to_read);
			if(ret<0){
				printf("ERROR: uart read() returned %d\n", ret);
				return -1;
			}
			else if(ret>0){
				// success, actually read something
				bytes_read += ret;
				bytes_left -= ret;
			}
		}
	}
	
	return bytes_read;
}

/*******************************************************************************
* int rc_uart_read_line(int bus, int max_bytes, char* buf)
*
* Function for reading a line of characters ending in '\n' newline character.
* This is a blocking function call. It will only return on these conditions:
* - a '\n' new line character was read, this is discarded.
* - max_bytes were read, this prevents overflowing a user buffer.
* - timeout declared in rc_uart_init() is reached
* - Global flow state in robotics_cape.h is set to EXITING.
*******************************************************************************/
int rc_uart_read_line(int bus, int max_bytes, char* buf){
	int ret; // holder for return values
	char temp;
	fd_set set; // for select()
	struct timeval timeout;
	int bytes_read=0; // number of bytes read so far

	// set up the timeout OUTSIDE of the read loop. We will likely be calling
	// select() multiple times and that will decrease the timeout struct each
	// time ensuring the TOTAL timeout requested by the user is honoured instead
	// of the timeout value compounding each loop.
	timeout.tv_sec = (int)bus_timeout_s[bus];
	timeout.tv_usec = (int)(1000000*fmod(bus_timeout_s[bus],1));
	
	// exit the read loop once enough bytes have been read
	// or the global flow state becomes EXITING. This prevents programs
	// getting stuck here and not exiting properly
	while(bytes_read<max_bytes && rc_get_state()!=EXITING){
		FD_ZERO(&set); /* clear the set */
		FD_SET(fd[bus], &set); /* add our file descriptor to the set */
		ret = select(fd[bus] + 1, &set, NULL, NULL, &timeout);
		if(ret == -1){
			// select returned and error. EINTR means interrupted by SIGINT
			// aka ctrl-c. Don't print anything as this happens normally
			// in case of EINTR/Ctrl-C just return how many bytes got read up 
			// until then without raising alarms.
			if(errno!=EINTR){
				printf("uart select() error: %s\n", strerror(errno));
				return -1;
			}
			return bytes_read;  
		}
		else if(ret == 0){
			// timeout
			return bytes_read;
		}
		else{
			// There was data to read. Read one bytes;
			ret=read(fd[bus], &temp, 1);
			if(ret<0){
				printf("ERROR: uart read() returned %d\n", ret);
				return -1;
			}
			else if(ret==1){
				// success, actually read something
				if(temp=='\n') return bytes_read;
				else{
					*(buf+bytes_read)=temp;
					bytes_read++;
				}	
			}
		}
	}
	
	return bytes_read;
}


int rc_uart_bytes_available(int bus){
	int out;
	// sanity checks
	if(bus<MIN_BUS || bus>MAX_BUS){
		printf("ERROR: uart bus must be between %d & %d\n", MIN_BUS, MAX_BUS);
		return -1;
	}
	if(initialized[bus]==0){
		printf("ERROR: uart%d must be initialized first\n", bus);
		return -1;
	}

	if(ioctl(fd[bus], FIONREAD, &out)<0){
		printf("ERROR: can't use ioctl on UART bus %d\n", bus);
		return -1;
	}

	return out;
}
