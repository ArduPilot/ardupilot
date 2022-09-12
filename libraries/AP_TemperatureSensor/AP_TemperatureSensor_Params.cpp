#include "AP_TemperatureSensor_Params.h"
#include "AP_TemperatureSensor.h"

#if AP_TEMPERATURE_SENSOR_ENABLED

#ifndef HAL_TEMPERATURE_SENSOR_I2C_TYPE_DEFAULT
#define HAL_TEMPERATURE_SENSOR_I2C_TYPE_DEFAULT 0
#endif

#ifndef HAL_TEMPERATURE_SENSOR_I2C_ADDR_DEFAULT
#define HAL_TEMPERATURE_SENSOR_I2C_ADDR_DEFAULT 0
#endif

#ifndef HAL_TEMPERATURE_SENSOR_I2C_BUS_DEFAULT
#define HAL_TEMPERATURE_SENSOR_I2C_BUS_DEFAULT 0
#endif

const AP_Param::GroupInfo AP_TemperatureSensor_Params::var_info[] = {
    // @Param: TYPE
    // @DisplayName: Temperature Sensor Type
    // @Description: Enables temperature sensors
    // @Values: 0:Disabled, 1:TSYS01
    // @User: Standard
    // @RebootRequired: True
    AP_GROUPINFO_FLAGS("TYPE", 1, AP_TemperatureSensor_Params, _type, HAL_TEMPERATURE_SENSOR_I2C_TYPE_DEFAULT, AP_PARAM_FLAG_ENABLE),

    // @Param: BUS
    // @DisplayName: Temperature sensor I2C bus
    // @Description: Temperature sensor I2C bus number
    // @Range: 0 3
    // @User: Advanced
    // @RebootRequired: True
    AP_GROUPINFO("BUS", 3, AP_TemperatureSensor_Params, _i2c_bus, HAL_TEMPERATURE_SENSOR_I2C_BUS_DEFAULT),

    // @Param: ADDR
    // @DisplayName: Temperature sensor I2C address
    // @Description: Temperature sensor I2C address
    // @Range: 0 127
    // @User: Advanced
    // @RebootRequired: True
    AP_GROUPINFO("ADDR", 4, AP_TemperatureSensor_Params, _i2c_address, HAL_TEMPERATURE_SENSOR_I2C_ADDR_DEFAULT),

    AP_GROUPEND
};

AP_TemperatureSensor_Params::AP_TemperatureSensor_Params(void) {
    AP_Param::setup_object_defaults(this, var_info);
}

#endif // AP_TEMPERATURE_SENSOR_ENABLED
