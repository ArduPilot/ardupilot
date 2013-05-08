
#ifndef __AP_INERTIAL_SENSOR_USER_INTERACT_H__
#define __AP_INERTIAL_SENSOR_USER_INTERACT_H__

#include <AP_Progmem.h>

/* Pure virtual interface class */
class AP_InertialSensor_UserInteract {
public:
    virtual uint8_t blocking_read() = 0;
    virtual void _printf_P(const prog_char *, ...) = 0;
};

#endif // __AP_INERTIAL_SENSOR_USER_INTERACT_H__

