#include <stdio.h>
#include <unistd.h>
#include <time.h>
#include <sys/time.h>
#include <sched.h>
#include <wiring.h>
#include <getopt.h>
#include <signal.h>
#include <string.h>
#include <AP_Common.h>
#include <AP_Param.h>
#include "desktop.h"

void setup(void);
void loop(void);

// the state of the desktop simulation
struct desktop_info desktop_state;

// catch floating point exceptions
static void sig_fpe(int signum)
{
	printf("ERROR: Floating point exception\n");
	exit(1);
}

static void usage(void)
{
	printf("Options:\n");
	printf("\t-s          enable CLI slider switch\n");
	printf("\t-w          wipe eeprom and dataflash\n");
	printf("\t-r RATE     set SITL framerate\n");
	printf("\t-H HEIGHT   initial barometric height\n");
	printf("\t-C          use console instead of TCP ports\n");
}

int main(int argc, char * const argv[])
{
	int opt;
	// default state
	desktop_state.slider = false;
	gettimeofday(&desktop_state.sketch_start_time, NULL);

	signal(SIGFPE, sig_fpe);

	while ((opt = getopt(argc, argv, "swhr:H:C")) != -1) {
		switch (opt) {
		case 's':
			desktop_state.slider = true;
			break;
		case 'w':
			AP_Param::erase_all();
			unlink("dataflash.bin");
			break;
		case 'r':
			desktop_state.framerate = (unsigned)atoi(optarg);
			break;
		case 'H':
			desktop_state.initial_height = atof(optarg);
			break;
		case 'C':
			desktop_state.console_mode = true;
			break;
		default:
			usage();
			exit(1);
		}
	}

	printf("Starting sketch '%s'\n", SKETCH);

	if (strcmp(SKETCH, "ArduCopter") == 0) {
		desktop_state.vehicle = ArduCopter;
		if (desktop_state.framerate == 0) {
			desktop_state.framerate = 200;
		}
	} else if (strcmp(SKETCH, "APMrover2") == 0) {
		desktop_state.vehicle = APMrover2;
		if (desktop_state.framerate == 0) {
			desktop_state.framerate = 50;
		}
		// set right default throttle for rover (allowing for reverse)
		ICR4.set(2, 1500);
	} else {
		desktop_state.vehicle = ArduPlane;
		if (desktop_state.framerate == 0) {
			desktop_state.framerate = 50;
		}
	}

	sitl_setup();
	setup();

	while (true) {
		struct timeval tv;
		fd_set fds;
		int fd_high = 0;
        
#ifdef __CYGWIN__
        // under windows if this loop is using alot of cpu,
        // the alarm gets called at a slower rate.
        sleep(5);
#endif

		FD_ZERO(&fds);
		loop();

		desktop_serial_select_setup(&fds, &fd_high);
		tv.tv_sec = 0;
		tv.tv_usec = 100;

		select(fd_high+1, &fds, NULL, NULL, &tv);
	}
	return 0;
}
