#include <stdio.h>
#include <time.h>
#include <sys/time.h>

void setup(void);
void loop(void);

struct timeval sketch_start_time;

int main(void)
{
	gettimeofday(&sketch_start_time, NULL);
	setup();
	while (true) loop();
	return 0;
}
