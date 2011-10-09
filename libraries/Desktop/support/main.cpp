#include <stdio.h>
#include <unistd.h>
#include <time.h>
#include <sys/time.h>

void setup(void);
void loop(void);

struct timeval sketch_start_time;

int main(void)
{
	gettimeofday(&sketch_start_time, NULL);
	setup();
	while (true) {
		struct timespec ts;
		loop();
		ts.tv_sec = 0;
		ts.tv_nsec = 100;
		nanosleep(&ts, NULL);
	}
	return 0;
}
