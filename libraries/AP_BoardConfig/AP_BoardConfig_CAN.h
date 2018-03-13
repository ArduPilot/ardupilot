#pragma once

#include <AP_HAL/AP_HAL.h>
#include "AP_BoardConfig.h"
#include <AP_Common/AP_Common.h>
#include <AP_Param/AP_Param.h>
#if CONFIG_HAL_BOARD != HAL_BOARD_CHIBIOS //we don't have ioctls in ChibiOS
#include <sys/ioctl.h>
#endif
#if HAL_WITH_UAVCAN
#define UAVCAN_PROTOCOL_ENABLE  1

class AP_BoardConfig_CAN {
public:
    AP_BoardConfig_CAN() {
        AP_Param::setup_object_defaults(this, var_info);
    };

    /* Do not allow copies */
    AP_BoardConfig_CAN(const AP_BoardConfig_CAN &other) = delete;
    AP_BoardConfig_CAN &operator=(const AP_BoardConfig_CAN&) = delete;

    void init(void);

    static const struct AP_Param::GroupInfo var_info[];

    class CAN_if_var_info {
        friend class AP_BoardConfig_CAN;

    public:
        CAN_if_var_info()
        {
            AP_Param::setup_object_defaults(this, var_info);
        }

        static const struct AP_Param::GroupInfo var_info[];

    private:
        AP_Int8 _driver_number;
        AP_Int8 _can_debug;
        AP_Int32 _can_bitrate;
    };

    class CAN_driver_var_info {
        friend class AP_BoardConfig_CAN;

    public:
        CAN_driver_var_info() :
                _uavcan(nullptr)
        {
            AP_Param::setup_object_defaults(this, var_info);
        }
        static const struct AP_Param::GroupInfo var_info[];

    private:
        AP_Int8 _protocol;

        AP_UAVCAN* _uavcan;
    };

    // returns number of enabled CAN interfaces
    static int8_t get_can_num_ifaces(void)
    {
        uint8_t ret = 0;

        for (uint8_t i = 0; i < MAX_NUMBER_OF_CAN_INTERFACES; i++) {
            if (_st_driver_number[i]) {
                ret++;
            }
        }

        return ret;
    }

    static int8_t get_can_debug(uint8_t i)
    {
        if (i < MAX_NUMBER_OF_CAN_INTERFACES) {
            return _st_can_debug[i];
        }
        return 0;
    }

    // return maximum level of debug
    static int8_t get_can_debug(void)
    {
        uint8_t ret = 0;
        for (uint8_t i = 0; i < MAX_NUMBER_OF_CAN_INTERFACES; i++) {
            uint8_t dbg = get_can_debug(i);
            ret = (dbg > ret) ? dbg : ret;
        }
        return ret;
    }

    CAN_if_var_info _var_info_can[MAX_NUMBER_OF_CAN_INTERFACES];
    CAN_driver_var_info _var_info_can_protocol[MAX_NUMBER_OF_CAN_DRIVERS];

    static int8_t _st_driver_number[MAX_NUMBER_OF_CAN_INTERFACES];
    static int8_t _st_can_debug[MAX_NUMBER_OF_CAN_INTERFACES];

    void setup_canbus(void);

};
#endif
