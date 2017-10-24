/*******************************************************************************
* roboticscape-usefulincludes.h
*
* This is a collection of all of the system includes necessary to compile the 
* robotics cape libraries. This may be used by the user as an "uber-include" 
* in their projects to clean up what would otherwise be a cluttered list of 
* includes at the top of their own program.
*
* We recommended including this BEFORE robotics_cape.h to make sure _GNU_SOURCE
* is the first include, otherwise macros may not work.
*
* James Strawson - 2016
*******************************************************************************/

#define _GNU_SOURCE  // to enable macros in pthread

#ifndef USEFUL_INCLUDES
#define USEFUL_INCLUDES

#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <string.h>
#include <fcntl.h>
#include <unistd.h>
#include <getopt.h>
#include <termios.h>
#include <errno.h>
#include <sys/ioctl.h>
#include <sys/types.h>	// for uint8_t types etc
#include <sys/stat.h>
#include <time.h>		// usleep, nanosleep
#include <math.h>		// atan2 and fabs
#include <signal.h>		// capture ctrl-c
#include <pthread.h>	// multi-threading
#include <linux/input.h>// buttons
#include <poll.h> 		// interrupt events
#include <sys/mman.h>	// mmap for accessing eQep
#include <sys/socket.h>	// udp socket	
#include <netinet/in.h>	// udp socket	
#include <sys/time.h>
#include <arpa/inet.h>	// udp socket	
#include <ctype.h>		// for isprint()
#include <sys/select.h>	// for read timeout

/*******************************************************************************
* Useful Constants
*******************************************************************************/
#define DEG_TO_RAD		0.0174532925199
#define RAD_TO_DEG		57.295779513
#define PI				M_PI
#define TWO_PI			(2.0 * M_PI)

/*******************************************************************************
* Useful Macros
*******************************************************************************/
#define ARRAY_SIZE(array) sizeof(array)/sizeof(array[0])
#define min(a, b) 	((a < b) ? a : b)

#endif // USEFUL_INCLUDES