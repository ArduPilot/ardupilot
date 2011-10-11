#include <stdio.h>
#include <unistd.h>
#include <time.h>
#include <sys/time.h>
#include <sched.h>
#include <wiring.h>

void setup(void);
void loop(void);

struct timeval sketch_start_time;

int main(void)
{
	gettimeofday(&sketch_start_time, NULL);
	setup();
	while (true) {
		struct timeval tv;
		unsigned long t1, t2;
		t1 = micros();
		loop();
		t2 = micros();
		if (t2 - t1 < 2) {
			tv.tv_sec = 0;
			tv.tv_usec = 1;
			select(0, NULL, NULL, NULL, &tv);
		}
	}
	return 0;
}
