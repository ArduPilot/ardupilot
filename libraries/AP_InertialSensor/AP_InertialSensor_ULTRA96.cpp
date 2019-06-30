#include <AP_HAL/AP_HAL.h>
#include "AP_InertialSensor_ULTRA96.h"

#if CONFIG_HAL_BOARD_SUBTYPE == HAL_BOARD_SUBTYPE_LINUX_ULTRA96

#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <sys/mman.h>
#include <unistd.h>

const extern AP_HAL::HAL& hal;

AP_InertialSensor_ULTRA96::AP_InertialSensor_ULTRA96(AP_InertialSensor &imu) :
    AP_InertialSensor_Backend(imu)
{
	//assign pointer here
	int mem_fd = open("/dev/mem", O_RDWR|O_SYNC|O_CLOEXEC);
	if (mem_fd == -1) {
        hal.console->printf("Unable to open /dev/mem\n");
	}
	data_pointer = (volatile int32_t*) mmap(NULL, 0x1000, PROT_READ|PROT_WRITE, MAP_SHARED, mem_fd, 0x78000000);
	close(mem_fd);

    // grab the used instances
    gyro_instance = _imu.register_gyro(1000,
                                       AP_HAL::Device::make_bus_id(AP_HAL::Device::BUS_TYPE_UNKNOWN,0,0,0));
    accel_instance= _imu.register_accel(1000,
                                        AP_HAL::Device::make_bus_id(AP_HAL::Device::BUS_TYPE_UNKNOWN,0,0,0));

    hal.scheduler->register_timer_process(FUNCTOR_BIND_MEMBER(&AP_InertialSensor_ULTRA96::timer_update, void));
}

void AP_InertialSensor_ULTRA96::timer_update(void)
{
	
	// ACC Data  m/s^2
	float xAccel = (*((volatile int32_t *)(data_pointer+18)))*0.0001;
	float yAccel = (*((volatile int32_t *)(data_pointer+19)))*0.0001;
	float zAccel = (*((volatile int32_t *)(data_pointer+20)))*0.0001;
	//GYRO DATA rad/s
	float xGyro = (*((volatile int32_t *)(data_pointer+21)))*0.0001;
	float yGyro = (*((volatile int32_t *)(data_pointer+22)))*0.0001;
	float zGyro = (*((volatile int32_t *)(data_pointer+23)))*0.0001;
    Vector3f accel = Vector3f(xAccel, yAccel, zAccel);
    Vector3f gyro = Vector3f(xGyro, yGyro, zGyro);
	
	_rotate_and_correct_gyro(gyro_instance, gyro);
	_notify_new_gyro_raw_sample(gyro_instance, gyro, AP_HAL::micros64());
	_rotate_and_correct_accel(accel_instance, accel);
	_notify_new_accel_raw_sample(accel_instance, accel, AP_HAL::micros64());
}

bool AP_InertialSensor_ULTRA96::update(void)
{
    update_accel(accel_instance);
    update_gyro(gyro_instance);
    return true;
}

#endif // HAL_BOARD_ULTRA96
