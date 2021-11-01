#pragma once

#include <AP_HAL/AP_HAL_Boards.h>

#ifndef HAL_HYGROMETER_ENABLED
#define HAL_HYGROMETER_ENABLED (BOARD_FLASH_SIZE > 1024)
#endif

#if HAL_HYGROMETER_ENABLED

#include <stdint.h>
#include "stdio.h"
#include <AP_Param/AP_Param.h>
#include <AP_Logger/AP_Logger.h>

#ifndef HYGROMETER_MAX_SENSORS
#define HYGROMETER_MAX_SENSORS  2
#endif

class AP_Hygrometer_Backend;

class AP_Hygrometer
{
public:
    friend class AP_Hygrometer_Backend;

    // constructor
    AP_Hygrometer();

    void init(void);

    bool get_temperature(uint8_t i, float &temperature) WARN_IF_UNUSED;
    bool get_temperature(float &temperature) { return get_temperature(primary, temperature); }

    bool get_humidity(uint8_t i, float &humidity);
    bool get_humidity(float &humidity) { return get_humidity(primary, humidity); }

    bool get_id(uint8_t i, uint8_t &id); 
    bool get_id(uint8_t &id) { return get_id(primary, id); }

    static const struct AP_Param::GroupInfo var_info[];

    enum class Type {
        NONE=0,
        UAVCAN=1,
    };

    struct State {
        float temperature;
        float humidity;
    };

    struct Param {
         AP_Enum<Type> type;
    };

        // return true if hygrometer is enabled
    bool enabled(uint8_t i) const {
        if (i < HYGROMETER_MAX_SENSORS) {
             return (Type)param[i].type.get() != Type::NONE;
        }
        return false;
    }
    bool enabled(void) const { return enabled(primary); }

    // get current primary sensor
    uint8_t get_primary(void) const { return primary; }

    static AP_Hygrometer *get_singleton() { return _singleton; }

    AP_Hygrometer_Backend *get_backend(uint8_t id) const ;

    void update(void);
    void Log_HYGR(void);

private:

    static AP_Hygrometer *_singleton;

    AP_Int8 primary_sensor;

    Param param[HYGROMETER_MAX_SENSORS];

    State state[HYGROMETER_MAX_SENSORS];

    // current primary sensor
    uint8_t primary;
    uint8_t num_sensors;

    AP_Hygrometer_Backend *sensor[HYGROMETER_MAX_SENSORS];
};

namespace AP {

AP_Hygrometer *hygrometer();

};

#endif // HAL_HYGROMETER_ENABLED
