/*
 * This file is free software: you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the
 * Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This file is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 * Code by Siddharth Bharat Purohit
 */

#pragma once

#include "AP_CANManager_config.h"

#if HAL_CANMANAGER_ENABLED

#include <AP_HAL/AP_HAL.h>

#include <AP_Param/AP_Param.h>
#include "AP_SLCANIface.h"
#include "AP_CANDriver.h"

#include "AP_CAN.h"

class CANSensor;

class AP_CANManager
{
public:
    AP_CANManager();

    /* Do not allow copies */
    CLASS_NO_COPY(AP_CANManager);

    static AP_CANManager* get_singleton()
    {
        if (_singleton == nullptr) {
            AP_HAL::panic("CANManager used before allocation.");
        }
        return _singleton;
    }

    enum LogLevel : uint8_t {
        LOG_NONE,
        LOG_ERROR,
        LOG_WARNING,
        LOG_INFO,
        LOG_DEBUG,
    };

    __INITFUNC__ void init(void);

    // register a new driver
    bool register_driver(AP_CAN::Protocol dtype, AP_CANDriver *driver);

    // register a new auxillary sensor driver for 11 bit address frames
    bool register_11bit_driver(AP_CAN::Protocol dtype, CANSensor *sensor, uint8_t &driver_index);

    // returns number of active CAN Drivers
    uint8_t get_num_drivers(void) const
    {
        return HAL_MAX_CAN_PROTOCOL_DRIVERS;
    }

    // return driver for index i
    AP_CANDriver* get_driver(uint8_t i) const
    {
        if (i < ARRAY_SIZE(_drivers)) {
            return _drivers[i];
        }
        return nullptr;
    }

    // returns current log level
    LogLevel get_log_level(void) const
    {
        return LogLevel(_loglevel.get());
    }
    
    // Method to log status and debug information for review while debugging
    void log_text(AP_CANManager::LogLevel loglevel, const char *tag, const char *fmt, ...) FMT_PRINTF(4,5);

    void log_retrieve(ExpandingString &str) const;

    // return driver type index i
    AP_CAN::Protocol get_driver_type(uint8_t i) const
    {
        if (i < ARRAY_SIZE(_driver_type_cache)) {
            return _driver_type_cache[i];
        }
        return AP_CAN::Protocol::None;
    }

    static const struct AP_Param::GroupInfo var_info[];

private:

    // Parameter interface for CANIfaces
    class CANIface_Params
    {
        friend class AP_CANManager;

    public:
        CANIface_Params()
        {
            AP_Param::setup_object_defaults(this, var_info);
        }

        static const struct AP_Param::GroupInfo var_info[];

        enum class Options : uint32_t {
            LOG_ALL_FRAMES = (1U<<0),
        };

        bool option_is_set(Options option) const {
            return (_options & uint32_t(option)) != 0;
        }

    private:
        AP_Int8 _driver_number;
        AP_Int32 _bitrate;
        AP_Int32 _fdbitrate;
        AP_Int32 _options;

#if AP_CAN_LOGGING_ENABLED && HAL_LOGGING_ENABLED
        uint8_t logging_id;
#endif
    };

    //Parameter Interface for CANDrivers
    class CANDriver_Params
    {
        friend class AP_CANManager;

    public:
        CANDriver_Params()
        {
            AP_Param::setup_object_defaults(this, var_info);
        }
        static const struct AP_Param::GroupInfo var_info[];

    private:
        AP_Int8 _driver_type;
        AP_Int8 _driver_type_11bit;
        AP_CANDriver* _uavcan;
        AP_CANDriver* _piccolocan;
    };

    CANIface_Params _interfaces[HAL_NUM_CAN_IFACES];
    AP_CANDriver* _drivers[HAL_MAX_CAN_PROTOCOL_DRIVERS];
    CANDriver_Params _drv_param[HAL_MAX_CAN_PROTOCOL_DRIVERS];
    AP_CAN::Protocol _driver_type_cache[HAL_MAX_CAN_PROTOCOL_DRIVERS];

    AP_Int8 _loglevel;
    uint8_t _num_drivers;
#if AP_CAN_SLCAN_ENABLED
    SLCAN::CANIface _slcan_interface;
#endif

    static AP_CANManager *_singleton;

    char* _log_buf;
    uint32_t _log_pos;

    HAL_Semaphore _sem;

#if AP_CAN_LOGGING_ENABLED && HAL_LOGGING_ENABLED
    /*
      handler for CAN frames for logging
    */
    void can_logging_callback(uint8_t bus, const AP_HAL::CANFrame &frame, AP_HAL::CANIface::CanIOFlags flags);
    void check_logging_enable(void);
#endif
};

namespace AP
{
AP_CANManager& can();
}

#endif  // HAL_CANMANAGER_ENABLED
