#include <stdio.h>
#include <unistd.h>
#include <time.h>
#include <sys/time.h>
#include <sched.h>
#include <wiring.h>
#include "desktop.h"

void setup(void);
void loop(void);

struct timeval sketch_start_time;

int main(void)
{
	gettimeofday(&sketch_start_time, NULL);
	setup();
	while (true) {
		struct timeval tv;
		fd_set fds;
		int fd_high = 0;

		FD_ZERO(&fds);
		loop();

		desktop_serial_select_setup(&fds, &fd_high);
		tv.tv_sec = 0;
		tv.tv_usec = 100;

		select(fd_high+1, &fds, NULL, NULL, &tv);
	}
	return 0;
}
