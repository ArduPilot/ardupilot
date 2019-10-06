#pragma once

#include <AP_HAL/AP_HAL.h>

#if HAL_WITH_UAVCAN

#include <AP_Param/AP_Param.h>

#ifndef AP_CAN_DEBUG
  #define AP_CAN_DEBUG 0
#endif

class AP_BoardConfig_CAN {
public:
    AP_BoardConfig_CAN();

    /* Do not allow copies */
    AP_BoardConfig_CAN(const AP_BoardConfig_CAN &other) = delete;
    AP_BoardConfig_CAN &operator=(const AP_BoardConfig_CAN&) = delete;

    static AP_BoardConfig_CAN* get_singleton() {
        return _singleton;
    }

    enum Protocol_Type : uint8_t {
        Protocol_Type_None = 0,
        Protocol_Type_UAVCAN = 1,
        Protocol_Type_KDECAN = 2,
        Protocol_Type_ToshibaCAN = 3
    };

    void init(void);

    // returns number of active CAN drivers
    uint8_t get_num_drivers(void) const { return _num_drivers; }

    // return debug level for interface i
    uint8_t get_debug_level(uint8_t i) const {
#if AP_CAN_DEBUG
        if (i < MAX_NUMBER_OF_CAN_INTERFACES) {
            return _interfaces[i]._driver_number_cache ? _interfaces[i]._debug_level : 0;
        }
#endif
        return 0;
    }

    // return maximum level of debug of all interfaces
    uint8_t get_debug_level(void) const {
        uint8_t ret = 0;
#if AP_CAN_DEBUG
        for (uint8_t i = 0; i < MAX_NUMBER_OF_CAN_INTERFACES; i++) {
            uint8_t dbg = get_debug_level(i);
            ret = (dbg > ret) ? dbg : ret;
        }
#endif
        return ret;
    }

    // return maximum level of debug for driver index i
    uint8_t get_debug_level_driver(uint8_t i) const {
        uint8_t ret = 0;
#if AP_CAN_DEBUG
        for (uint8_t j = 0; j < MAX_NUMBER_OF_CAN_INTERFACES; j++) {
            if (_interfaces[j]._driver_number_cache == i) {
                uint8_t dbg = get_debug_level(j);
                ret = (dbg > ret) ? dbg : ret;
            }
        }
#endif
        return ret;
    }

    // return driver for index i
    AP_HAL::CANProtocol* get_driver(uint8_t i) const {
        if (i < MAX_NUMBER_OF_CAN_DRIVERS) {
            return _drivers[i]._driver;
        }
        return nullptr;
    }

    // return protocol type index i
    Protocol_Type get_protocol_type(uint8_t i) const {
        if (i < MAX_NUMBER_OF_CAN_DRIVERS) {
            return _drivers[i]._protocol_type_cache;
        }
        return Protocol_Type_None;
    }

    static const struct AP_Param::GroupInfo var_info[];
#if AP_UAVCAN_SLCAN_ENABLED
    AP_HAL::UARTDriver *get_slcan_serial();
    uint8_t get_slcan_timeout() { return _slcan._timeout; }
    void reset_slcan_serial() { _slcan._ser_port.set_and_save_ifchanged(-1); }
#endif
private:
    class Interface {
        friend class AP_BoardConfig_CAN;

    public:
        Interface() {
            AP_Param::setup_object_defaults(this, var_info);
        }

        static const struct AP_Param::GroupInfo var_info[];

    private:
        AP_Int8 _driver_number;
        uint8_t _driver_number_cache;
        AP_Int32 _bitrate;
#if AP_CAN_DEBUG
        AP_Int8 _debug_level;
#endif
    };

    class Driver {
        friend class AP_BoardConfig_CAN;

    public:
        Driver() {
            AP_Param::setup_object_defaults(this, var_info);
        }

        static const struct AP_Param::GroupInfo var_info[];

    private:
        AP_Int8 _protocol_type;
        Protocol_Type _protocol_type_cache;
        AP_HAL::CANProtocol* _driver;
        AP_HAL::CANProtocol* _uavcan;
        AP_HAL::CANProtocol* _kdecan;
        AP_HAL::CANProtocol* _tcan;
    };

#if AP_UAVCAN_SLCAN_ENABLED
    class SLCAN_Interface {
        friend class AP_BoardConfig_CAN;

    public:
        SLCAN_Interface() {
            AP_Param::setup_object_defaults(this, var_info);
        }

        static const struct AP_Param::GroupInfo var_info[];

    private:
        AP_Int8 _can_port;
        AP_Int8 _ser_port;
        AP_Int16 _timeout;
    };
    SLCAN_Interface _slcan;
#endif
    Interface _interfaces[MAX_NUMBER_OF_CAN_INTERFACES];
    Driver _drivers[MAX_NUMBER_OF_CAN_DRIVERS];
    uint8_t _num_drivers;
    static AP_BoardConfig_CAN *_singleton;
};

namespace AP {
    AP_BoardConfig_CAN& can();
}
#endif
