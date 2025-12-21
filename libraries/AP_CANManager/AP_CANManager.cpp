/*
  This program is free software: you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation, either version 3 of the License, or
  (at your option) any later version.

  This program is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  GNU General Public License for more details.

  You should have received a copy of the GNU General Public License
  along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/
/*
 *   AP_CANManager - board specific configuration for CAN interface
 */

#include <AP_HAL/AP_HAL.h>
#include <AP_Common/AP_Common.h>
#include "AP_CANManager.h"

#if HAL_CANMANAGER_ENABLED

#include <AP_BoardConfig/AP_BoardConfig.h>
#include <AP_Vehicle/AP_Vehicle_Type.h>
#include <AP_DroneCAN/AP_DroneCAN.h>
#include <AP_KDECAN/AP_KDECAN.h>
#include <AP_SerialManager/AP_SerialManager.h>
#include <AP_PiccoloCAN/AP_PiccoloCAN.h>
#include <AP_EFI/AP_EFI_NWPMU.h>
#include <GCS_MAVLink/GCS.h>
#if CONFIG_HAL_BOARD == HAL_BOARD_LINUX
#include <AP_HAL_Linux/CANSocketIface.h>
#elif CONFIG_HAL_BOARD == HAL_BOARD_SITL
#include <AP_HAL_SITL/CANSocketIface.h>
#elif CONFIG_HAL_BOARD == HAL_BOARD_CHIBIOS
#include <hal.h>
#include <AP_HAL_ChibiOS/CANIface.h>
#endif

#include <AP_Common/ExpandingString.h>
#include <AP_Common/sorting.h>
#include <AP_Logger/AP_Logger.h>

#define LOG_TAG "CANMGR"
#define LOG_BUFFER_SIZE 1024

extern const AP_HAL::HAL& hal;

// table of user settable parameters
const AP_Param::GroupInfo AP_CANManager::var_info[] = {

#if HAL_NUM_CAN_IFACES > 0
    // @Group: P1_
    // @Path: ../AP_CANManager/AP_CANIfaceParams.cpp
    AP_SUBGROUPINFO(_interfaces[0], "P1_", 1, AP_CANManager, AP_CANManager::CANIface_Params),
#endif

#if HAL_NUM_CAN_IFACES > 1
    // @Group: P2_
    // @Path: ../AP_CANManager/AP_CANIfaceParams.cpp
    AP_SUBGROUPINFO(_interfaces[1], "P2_", 2, AP_CANManager, AP_CANManager::CANIface_Params),
#endif

#if HAL_NUM_CAN_IFACES > 2
    // @Group: P3_
    // @Path: ../AP_CANManager/AP_CANIfaceParams.cpp
    AP_SUBGROUPINFO(_interfaces[2], "P3_", 3, AP_CANManager, AP_CANManager::CANIface_Params),
#endif

#if HAL_MAX_CAN_PROTOCOL_DRIVERS > 0
    // @Group: D1_
    // @Path: ../AP_CANManager/AP_CANManager_CANDriver_Params.cpp
    AP_SUBGROUPINFO(_drv_param[0], "D1_", 4, AP_CANManager, AP_CANManager::CANDriver_Params),
#endif

#if HAL_MAX_CAN_PROTOCOL_DRIVERS > 1
    // @Group: D2_
    // @Path: ../AP_CANManager/AP_CANManager_CANDriver_Params.cpp
    AP_SUBGROUPINFO(_drv_param[1], "D2_", 5, AP_CANManager, AP_CANManager::CANDriver_Params),
#endif

#if HAL_MAX_CAN_PROTOCOL_DRIVERS > 2
    // @Group: D3_
    // @Path: ../AP_CANManager/AP_CANManager_CANDriver_Params.cpp
    AP_SUBGROUPINFO(_drv_param[2], "D3_", 6, AP_CANManager, AP_CANManager::CANDriver_Params),
#endif

#if AP_CAN_SLCAN_ENABLED
    // @Group: SLCAN_
    // @Path: ../AP_CANManager/AP_SLCANIface.cpp
    AP_SUBGROUPINFO(_slcan_interface, "SLCAN_", 7, AP_CANManager, SLCAN::CANIface),
#endif

    // @Param: LOGLEVEL
    // @DisplayName: Loglevel
    // @Description: Loglevel for recording initialisation and debug information from CAN Interface
    // @Range: 0 4
    // @Values: 0: Log None, 1: Log Error, 2: Log Warning and below, 3: Log Info and below, 4: Log Everything
    // @User: Advanced
    AP_GROUPINFO("LOGLEVEL", 8, AP_CANManager, _loglevel, AP_CANManager::LOG_NONE),

    AP_GROUPEND
};

AP_CANManager *AP_CANManager::_singleton;

AP_CANManager::AP_CANManager()
{
    AP_Param::setup_object_defaults(this, var_info);
    if (_singleton != nullptr) {
        AP_HAL::panic("AP_CANManager must be singleton");
    }
    _singleton = this;
}

#if !AP_TEST_DRONECAN_DRIVERS
void AP_CANManager::init()
{
    WITH_SEMAPHORE(_sem);

    // we need to mutate the HAL to install new CAN interfaces
    AP_HAL::HAL& hal_mutable = AP_HAL::get_HAL_mutable();

#if CONFIG_HAL_BOARD == HAL_BOARD_SITL
    if (AP::sitl() == nullptr) {
        AP_HAL::panic("CANManager: SITL not initialised!");
    }
#endif
    // We only allocate log buffer only when under debug
    if (_loglevel != AP_CANManager::LOG_NONE) {
        _log_buf = NEW_NOTHROW char[LOG_BUFFER_SIZE];
        _log_pos = 0;
    }

#if AP_CAN_SLCAN_ENABLED
    //Reset all SLCAN related params that needs resetting at boot
    _slcan_interface.reset_params();
#endif

    AP_CAN::Protocol drv_type[HAL_MAX_CAN_PROTOCOL_DRIVERS] = {};
    // loop through interfaces and allocate and initialise Iface,
    // Also allocate Driver objects, and add interfaces to them
    for (uint8_t i = 0; i < HAL_NUM_CAN_IFACES; i++) {
        // Get associated Driver to the interface
        uint8_t drv_num = _interfaces[i]._driver_number;
        if (drv_num == 0 || drv_num > HAL_MAX_CAN_PROTOCOL_DRIVERS) {
            continue;
        }
        drv_num--;

        if (hal_mutable.can[i] == nullptr) {
            // So if this interface is not allocated allocate it here,
            // also pass the index of the CANBus
            hal_mutable.can[i] = NEW_NOTHROW HAL_CANIface(i);
        }

        // Initialise the interface we just allocated
        if (hal_mutable.can[i] == nullptr) {
            continue;
        }
        AP_HAL::CANIface* iface = hal_mutable.can[i];

        // Find the driver type that we need to allocate and register this interface with
        drv_type[drv_num] = (AP_CAN::Protocol) _drv_param[drv_num]._driver_type.get();
        bool can_initialised = false;
        // Check if this interface need hooking up to slcan passthrough
        // instead of a driver
#if AP_CAN_SLCAN_ENABLED
        if (_slcan_interface.init_passthrough(i)) {
            // we have slcan bridge setup pass that on as can iface
            can_initialised = hal_mutable.can[i]->init(_interfaces[i]._bitrate, _interfaces[i]._fdbitrate*1000000);
            iface = &_slcan_interface;
        } else {
#else
        if (true) {
#endif
            can_initialised = hal_mutable.can[i]->init(_interfaces[i]._bitrate, _interfaces[i]._fdbitrate*1000000);
        }

        if (!can_initialised) {
            log_text(AP_CANManager::LOG_ERROR, LOG_TAG, "Failed to initialise CAN Interface %d", i+1);
            continue;
        }

        log_text(AP_CANManager::LOG_INFO, LOG_TAG, "CAN Interface %d initialized well", i + 1);

        if (_drivers[drv_num] != nullptr) {
            //We already initialised the driver just add interface and move on
            log_text(AP_CANManager::LOG_INFO, LOG_TAG, "Adding Interface %d to Driver %d", i + 1, drv_num + 1);
            _drivers[drv_num]->add_interface(iface);
            continue;
        }

        if (_num_drivers >= HAL_MAX_CAN_PROTOCOL_DRIVERS) {
            // We are exceeding number of drivers,
            // this can't be happening time to panic
            AP_BoardConfig::config_error("Max number of CAN Drivers exceeded\n\r");
        }

        // Allocate the set type of Driver
        switch (drv_type[drv_num]) {
#if HAL_ENABLE_DRONECAN_DRIVERS
        case AP_CAN::Protocol::DroneCAN:
            _drivers[drv_num] = _drv_param[drv_num]._uavcan = NEW_NOTHROW AP_DroneCAN(drv_num);

            if (_drivers[drv_num] == nullptr) {
                AP_BoardConfig::allocation_error("uavcan %d", i + 1);
                continue;
            }

            AP_Param::load_object_from_eeprom((AP_DroneCAN*)_drivers[drv_num], AP_DroneCAN::var_info);
            break;
#endif
#if HAL_PICCOLO_CAN_ENABLE
        case AP_CAN::Protocol::PiccoloCAN:
            _drivers[drv_num] = _drv_param[drv_num]._piccolocan = NEW_NOTHROW AP_PiccoloCAN;

            if (_drivers[drv_num] == nullptr) {
                AP_BoardConfig::allocation_error("PiccoloCAN %d", drv_num + 1);
                continue;
            }

            AP_Param::load_object_from_eeprom((AP_PiccoloCAN*)_drivers[drv_num], AP_PiccoloCAN::var_info);
            break;
#endif
        default:
            continue;
        }

        _num_drivers++;

        // Hook this interface to the selected Driver Type
        _drivers[drv_num]->add_interface(iface);
        log_text(AP_CANManager::LOG_INFO, LOG_TAG, "Adding Interface %d to Driver %d", i + 1, drv_num + 1);

    }

    for (uint8_t drv_num = 0; drv_num < HAL_MAX_CAN_PROTOCOL_DRIVERS; drv_num++) {
        //initialise all the Drivers

        // Cache the driver type, initialized or not, so we can detect that it is in the params at boot via get_driver_type().
        // This allows drivers that are initialized by CANSensor instead of CANManager to know if they should init or not
        _driver_type_cache[drv_num] = drv_type[drv_num];

        if (_drivers[drv_num] == nullptr) {
            continue;
        }

        _drivers[drv_num]->init(drv_num);
    }

#if AP_CAN_LOGGING_ENABLED
    hal.scheduler->register_io_process(FUNCTOR_BIND_MEMBER(&AP_CANManager::check_logging_enable, void));
#endif
}
#else
void AP_CANManager::init()
{
    WITH_SEMAPHORE(_sem);
    for (uint8_t i = 0; i < HAL_NUM_CAN_IFACES; i++) {
        if ((AP_CAN::Protocol) _drv_param[i]._driver_type.get() == AP_CAN::Protocol::DroneCAN) {
            _drivers[i] = _drv_param[i]._uavcan = NEW_NOTHROW AP_DroneCAN(i);

            if (_drivers[i] == nullptr) {
                AP_BoardConfig::allocation_error("uavcan %d", i + 1);
                continue;
            }

            AP_Param::load_object_from_eeprom((AP_DroneCAN*)_drivers[i], AP_DroneCAN::var_info);
            _drivers[i]->init(i);
            _driver_type_cache[i] = (AP_CAN::Protocol) _drv_param[i]._driver_type.get();
        }
    }
}
#endif

/*
  register a new CAN driver
 */
bool AP_CANManager::register_driver(AP_CAN::Protocol dtype, AP_CANDriver *driver)
{
    WITH_SEMAPHORE(_sem);

    // we need to mutate the HAL to install new CAN interfaces
    AP_HAL::HAL& hal_mutable = AP_HAL::get_HAL_mutable();

    for (uint8_t i = 0; i < HAL_NUM_CAN_IFACES; i++) {
        uint8_t drv_num = _interfaces[i]._driver_number;
        if (drv_num == 0 || drv_num > HAL_MAX_CAN_PROTOCOL_DRIVERS) {
            continue;
        }
        // from 1 based to 0 based
        drv_num--;

        if (dtype != (AP_CAN::Protocol)_drv_param[drv_num]._driver_type.get()) {
            continue;
        }
        if (_drivers[drv_num] != nullptr) {
            continue;
        }
        if (_num_drivers >= HAL_MAX_CAN_PROTOCOL_DRIVERS) {
            continue;
        }

        if (hal_mutable.can[i] == nullptr) {
            // if this interface is not allocated allocate it here,
            // also pass the index of the CANBus
            hal_mutable.can[i] = NEW_NOTHROW HAL_CANIface(i);
        }

        // Initialise the interface we just allocated
        if (hal_mutable.can[i] == nullptr) {
            continue;
        }
        AP_HAL::CANIface* iface = hal_mutable.can[i];

        _drivers[drv_num] = driver;
        _drivers[drv_num]->add_interface(iface);
        log_text(AP_CANManager::LOG_INFO, LOG_TAG, "Adding Interface %d to Driver %d", i + 1, drv_num + 1);

        _drivers[drv_num]->init(drv_num);
        _driver_type_cache[drv_num] = dtype;

        _num_drivers++;

        return true;
    }
    return false;
}

// register a new auxillary sensor driver for 11 bit address frames
bool AP_CANManager::register_11bit_driver(AP_CAN::Protocol dtype, CANSensor *sensor, uint8_t &driver_index)
{
    WITH_SEMAPHORE(_sem);

    for (uint8_t i = 0; i < HAL_NUM_CAN_IFACES; i++) {
        uint8_t drv_num = _interfaces[i]._driver_number;
        if (drv_num == 0 || drv_num > HAL_MAX_CAN_PROTOCOL_DRIVERS) {
            continue;
        }
        // from 1 based to 0 based
        drv_num--;

        if (dtype != (AP_CAN::Protocol)_drv_param[drv_num]._driver_type_11bit.get()) {
            continue;
        }
        if (_drivers[drv_num] != nullptr &&
            _drivers[drv_num]->add_11bit_driver(sensor)) {
            driver_index = drv_num;
            return true;
        }
    }
    return false;

}

// Method used by CAN related library methods to report status and debug info
// The result of this method can be accessed via ftp get @SYS/can_log.txt
void AP_CANManager::log_text(AP_CANManager::LogLevel loglevel, const char *tag, const char *fmt, ...)
{
    if (_log_buf == nullptr) {
        return;
    }
    if (loglevel > _loglevel) {
        return;
    }
    WITH_SEMAPHORE(_sem);

    if ((LOG_BUFFER_SIZE - _log_pos) < (10 + strlen(tag) + strlen(fmt))) {
        // reset log pos
        _log_pos = 0;
    }
    //Tag Log Message
    const char *log_level_tag = "";
    switch (loglevel) {
    case AP_CANManager::LOG_DEBUG :
        log_level_tag = "DEBUG";
        break;

    case AP_CANManager::LOG_INFO :
        log_level_tag = "INFO";
        break;

    case AP_CANManager::LOG_WARNING :
        log_level_tag = "WARN";
        break;

    case AP_CANManager::LOG_ERROR :
        log_level_tag = "ERROR";
        break;

    case AP_CANManager::LOG_NONE:
        return;
    }

    _log_pos += hal.util->snprintf(&_log_buf[_log_pos], LOG_BUFFER_SIZE - _log_pos, "\n%s %s :", log_level_tag, tag);

    va_list arg_list;
    va_start(arg_list, fmt);
    _log_pos += hal.util->vsnprintf(&_log_buf[_log_pos], LOG_BUFFER_SIZE - _log_pos, fmt, arg_list);
    va_end(arg_list);
}

// log retrieve method used by file sys method to report can log
void AP_CANManager::log_retrieve(ExpandingString &str) const
{
    if (_log_buf == nullptr) {
        GCS_SEND_TEXT(MAV_SEVERITY_ERROR, "Log buffer not available");
        return;
    }
    str.append(_log_buf, _log_pos);
}

#if AP_CAN_LOGGING_ENABLED
/*
  handler for CAN frames for frame logging
 */
void AP_CANManager::can_logging_callback(uint8_t bus, const AP_HAL::CANFrame &frame, AP_HAL::CANIface::CanIOFlags flags)
{
#if HAL_CANFD_SUPPORTED
    if (frame.canfd) {
        struct log_CAFD pkt {
            LOG_PACKET_HEADER_INIT(LOG_CAFD_MSG),
            time_us : AP_HAL::micros64(),
            bus     : bus,
            id      : frame.id,
            dlc     : frame.dlc
        };
        memcpy(pkt.data, frame.data, frame.dlcToDataLength(frame.dlc));
        AP::logger().WriteBlock(&pkt, sizeof(pkt));
        return;
    }
#endif
    struct log_CANF pkt {
        LOG_PACKET_HEADER_INIT(LOG_CANF_MSG),
        time_us : AP_HAL::micros64(),
        bus     : bus,
        id      : frame.id,
        dlc     : frame.dlc
    };
    memcpy(pkt.data, frame.data, frame.dlc);
    AP::logger().WriteBlock(&pkt, sizeof(pkt));
}

/*
  see if we need to enable/disable the CAN logging callback
 */
void AP_CANManager::check_logging_enable(void)
{
    for (uint8_t i = 0; i < HAL_NUM_CAN_IFACES; i++) {
        const bool enabled = _interfaces[i].option_is_set(CANIface_Params::Options::LOG_ALL_FRAMES);
        uint8_t &logging_id = _interfaces[i].logging_id;
        auto *can = hal.can[i];
        if (can == nullptr) {
            continue;
        }
        if (enabled && logging_id == 0) {
            can->register_frame_callback(
                FUNCTOR_BIND_MEMBER(&AP_CANManager::can_logging_callback, void, uint8_t, const AP_HAL::CANFrame &, AP_HAL::CANIface::CanIOFlags),
                logging_id);
        } else if (!enabled && logging_id != 0) {
            can->unregister_frame_callback(logging_id);
        }
    }
}

#endif // AP_CAN_LOGGING_ENABLED

AP_CANManager& AP::can()
{
    return *AP_CANManager::get_singleton();
}

#endif

