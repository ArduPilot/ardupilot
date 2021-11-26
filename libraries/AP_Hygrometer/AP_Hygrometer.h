#pragma once

#include <AP_HAL/AP_HAL_Boards.h>

#ifndef AP_HYGROMETER_ENABLED
#define AP_HYGROMETER_ENABLED (BOARD_FLASH_SIZE > 1024)
#endif

#if AP_HYGROMETER_ENABLED

#include <stdint.h>
#include "stdio.h"
#include <AP_Param/AP_Param.h>
#include <AP_Logger/AP_Logger.h>

#ifndef AP_HYGROMETER_MAX_SENSORS
#define AP_HYGROMETER_MAX_SENSORS  2
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
    bool get_humidity(uint8_t i, float &humidity) WARN_IF_UNUSED;

    float get_humidity(uint8_t instance);

    float get_temperature(uint8_t instance);

    static const struct AP_Param::GroupInfo var_info[];

    enum class Type {
        NONE=0,
#if HAL_ENABLE_LIBUAVCAN_DRIVERS
        UAVCAN=1,
#endif
    };

    struct State {
        float temperature;
        float humidity;
    };

    struct Param {
        AP_Enum<Type> type;
    };

    // return true if hygrometer is enabled
    bool enabled(uint8_t i) const
    {
        if (i < num_sensors) {
            return (Type)param[i].type.get() != Type::NONE;
        }
        return false;
    }

    static AP_Hygrometer *get_singleton() { return _singleton; }

    AP_Hygrometer_Backend *get_backend(uint8_t id) const;

    // allow threads to lock against hygrometer update
    HAL_Semaphore &get_semaphore(void) { return _rsem; }

    void update(void);
    void Log_HYGR(void);

private:

    static AP_Hygrometer *_singleton;

    Param param[AP_HYGROMETER_MAX_SENSORS];

    State state[AP_HYGROMETER_MAX_SENSORS];

    uint8_t num_sensors;

    // semaphore for API access from threads
    HAL_Semaphore _rsem;

    AP_Hygrometer_Backend *sensor[AP_HYGROMETER_MAX_SENSORS];
};

namespace AP
{

AP_Hygrometer *hygrometer();

};

#endif // AP_HYGROMETER_ENABLED
