#include <AP_HAL/AP_HAL.h>
#include <AP_Vehicle/AP_Vehicle_Type.h>

#if CONFIG_HAL_BOARD_SUBTYPE == HAL_BOARD_SUBTYPE_LINUX_ULTRA96

#include "AP_Baro_ULTRA96.h"

#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <sys/mman.h>
#include <unistd.h>

extern const AP_HAL::HAL& hal;

/*
  constructor - registers instance at top Baro driver
 */
AP_Baro_ULTRA96::AP_Baro_ULTRA96(AP_Baro &baro) :
    AP_Baro_Backend(baro)
{

	//assign pointer here
	int mem_fd = open("/dev/mem", O_RDWR|O_SYNC|O_CLOEXEC);
	if (mem_fd == -1) {
        hal.console->printf("Unable to open /dev/mem\n");
	}
	data_pointer = (volatile int32_t*) mmap(NULL, 0x1000, PROT_READ|PROT_WRITE, MAP_SHARED, mem_fd, 0x78000000);
	close(mem_fd);
	
    _instance = _frontend.register_sensor();
}

// Read the sensor
void AP_Baro_ULTRA96::update(void)
{
	//check offsets
    float press =  (*((volatile int32_t *)(data_pointer+27)))*0.01;
    float temp =  (*((volatile int32_t *)(data_pointer+28)))*0.0001;

    _copy_to_frontend(_instance, press, temp);
}

#endif  // CONFIG_HAL_BOARD_SUBTYPE

