#pragma once

#include <AP_Param/AP_Param.h>

class AP_TemperatureSensor_Params {
public:
    static const struct AP_Param::GroupInfo var_info[];

    AP_TemperatureSensor_Params(void);

    CLASS_NO_COPY(AP_TemperatureSensor_Params);

    AP_Int8  _type;             // 0=disabled, others see frontend enum TYPE
    AP_Int8  _i2c_bus;          // I2C bus number
    AP_Int8  _i2c_address;      // I2C address
};
