#include "AP_Compass_ULTRA96.h"

#include <AP_HAL/AP_HAL.h>

#if CONFIG_HAL_BOARD_SUBTYPE == HAL_BOARD_SUBTYPE_LINUX_ULTRA96

#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <sys/mman.h>
#include <unistd.h>

extern const AP_HAL::HAL& hal;

AP_Compass_ULTRA96::AP_Compass_ULTRA96()
    : AP_Compass_Backend()
{

	//assign pointer here
	int mem_fd = open("/dev/mem", O_RDWR|O_SYNC|O_CLOEXEC);
	if (mem_fd == -1) {
        hal.console->printf("Unable to open /dev/mem\n");
	}
	data_pointer = (volatile int32_t*) mmap(NULL, 0x1000, PROT_READ|PROT_WRITE, MAP_SHARED, mem_fd, 0x78000000);
	close(mem_fd);
	_compass_instance = register_compass();

    hal.scheduler->register_timer_process(FUNCTOR_BIND(this, &AP_Compass_ULTRA96::timer_update, void));
}


void AP_Compass_ULTRA96::timer_update(void)
{
	//check offsets
    float xMag = (*((volatile int32_t *)(data_pointer+24)))*0.1;
    float yMag = (*((volatile int32_t *)(data_pointer+25)))*0.1;
    float zMag = (*((volatile int32_t *)(data_pointer+26)))*0.1;
	
	Vector3f raw_field = Vector3f(xMag, yMag, zMag);
	accumulate_sample(raw_field, _compass_instance);
}


void AP_Compass_ULTRA96::read(void)
{
        drain_accumulated_samples(_compass_instance);
}
#endif
