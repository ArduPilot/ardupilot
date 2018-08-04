#include "AP_Baro_InputEvent.h"

#if CONFIG_HAL_BOARD_SUBTYPE == HAL_BOARD_SUBTYPE_LINUX_MAMBO

#include <AP_HAL/AP_HAL.h>
#include <stdio.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <unistd.h>
#include <linux/input.h>

//some code copied from paparazzi

extern const AP_HAL::HAL& hal;

AP_Baro_InputEvent::AP_Baro_InputEvent(AP_Baro &baro) :
    AP_Baro_Backend(baro)
{
}

AP_Baro_Backend *AP_Baro_InputEvent::probe(AP_Baro &baro)
{
    AP_Baro_InputEvent *sensor = new AP_Baro_InputEvent(baro);
    if (sensor->_init()) return sensor; else return 0;
}

bool AP_Baro_InputEvent::_init() 
{
    _data_avail = false;
    _fd = open("/dev/input/baro_event", O_RDONLY);
    if (_fd == -1) {
        printf("Unable to open baro event to read pressure\n");
        return false;
    }
    _instance = _frontend.register_sensor();
    hal.scheduler->register_io_process(FUNCTOR_BIND_MEMBER(&AP_Baro_InputEvent::tick, void));
    return true;
}

void AP_Baro_InputEvent::update()
{
    if (_data_avail && _sem->take(HAL_SEMAPHORE_BLOCK_FOREVER)) {
        _data_avail = false;
        _copy_to_frontend(_instance, _pressure, 27.0f);
        _sem->give();
    }
}

void AP_Baro_InputEvent::tick()
{
    struct input_event ev;
    ssize_t n;
	n = read(_fd, &ev, sizeof(ev));
	if (n == sizeof(ev) && ev.type == EV_ABS && ev.code == ABS_PRESSURE) {
		_pressure = 100.f * ((float)ev.value) / 4096.f;
		_data_avail = true;
	}
}

#endif
