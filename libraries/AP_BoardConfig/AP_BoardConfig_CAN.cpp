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
 *   AP_BoardConfig_CAN - board specific configuration for CAN interface
 */

#include <AP_HAL/AP_HAL.h>
#include <AP_Common/AP_Common.h>
#include "AP_BoardConfig.h"
#include "AP_BoardConfig_CAN.h"

#if HAL_WITH_UAVCAN

#if CONFIG_HAL_BOARD == HAL_BOARD_PX4 || CONFIG_HAL_BOARD == HAL_BOARD_VRBRAIN
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <unistd.h>

#include <AP_HAL_PX4/CAN.h>
#elif CONFIG_HAL_BOARD == HAL_BOARD_LINUX
#include <AP_HAL_Linux/CAN.h>
#elif CONFIG_HAL_BOARD == HAL_BOARD_CHIBIOS
#include <AP_HAL_ChibiOS/CAN.h>
#include <AP_HAL_ChibiOS/CANSerialRouter.h>
#endif

#include <AP_Vehicle/AP_Vehicle.h>
#include <AP_UAVCAN/AP_UAVCAN_SLCAN.h>
#include <AP_UAVCAN/AP_UAVCAN.h>
#include <AP_KDECAN/AP_KDECAN.h>
#include <AP_ToshibaCAN/AP_ToshibaCAN.h>

extern const AP_HAL::HAL& hal;

// table of user settable parameters
const AP_Param::GroupInfo AP_BoardConfig_CAN::var_info[] = {

#if MAX_NUMBER_OF_CAN_INTERFACES > 0
    // @Group: P1_
    // @Path: ../AP_BoardConfig/canbus_interface.cpp
    AP_SUBGROUPINFO(_interfaces[0], "P1_", 1, AP_BoardConfig_CAN, AP_BoardConfig_CAN::Interface),
#endif

#if MAX_NUMBER_OF_CAN_INTERFACES > 1
    // @Group: P2_
    // @Path: ../AP_BoardConfig/canbus_interface.cpp
    AP_SUBGROUPINFO(_interfaces[1], "P2_", 2, AP_BoardConfig_CAN, AP_BoardConfig_CAN::Interface),
#endif

#if MAX_NUMBER_OF_CAN_INTERFACES > 2
    // @Group: P3_
    // @Path: ../AP_BoardConfig/canbus_interface.cpp
    AP_SUBGROUPINFO(_interfaces[2], "P3_", 3, AP_BoardConfig_CAN, AP_BoardConfig_CAN::Interface),
#endif

#if MAX_NUMBER_OF_CAN_DRIVERS > 0
    // @Group: D1_
    // @Path: ../AP_BoardConfig/canbus_driver.cpp
    AP_SUBGROUPINFO(_drivers[0], "D1_", 4, AP_BoardConfig_CAN, AP_BoardConfig_CAN::Driver),
#endif

#if MAX_NUMBER_OF_CAN_DRIVERS > 1
    // @Group: D2_
    // @Path: ../AP_BoardConfig/canbus_driver.cpp
    AP_SUBGROUPINFO(_drivers[1], "D2_", 5, AP_BoardConfig_CAN, AP_BoardConfig_CAN::Driver),
#endif

#if MAX_NUMBER_OF_CAN_DRIVERS > 2
    // @Group: D3_
    // @Path: ../AP_BoardConfig/canbus_driver.cpp
    AP_SUBGROUPINFO(_drivers[2], "D3_", 6, AP_BoardConfig_CAN, AP_BoardConfig_CAN::Driver),
#endif

    // @Param: SLCAN_RT
    // @DisplayName: SLCAN Route
    // @Description: CAN Driver ID to be routed to SLCAN, 0 means no routing
    // @Range: 0 3
    // @User: Advanced
    AP_GROUPINFO("SLCAN_RT", 7, AP_BoardConfig_CAN, _slcan_rt, 0),

    AP_GROUPEND
};

AP_BoardConfig_CAN *AP_BoardConfig_CAN::_singleton;

AP_BoardConfig_CAN::AP_BoardConfig_CAN()
{
    AP_Param::setup_object_defaults(this, var_info);
#if CONFIG_HAL_BOARD == HAL_BOARD_SITL
    if (_singleton != nullptr) {
        AP_HAL::panic("AP_BoardConfig_CAN must be singleton");
    }
#endif // CONFIG_HAL_BOARD == HAL_BOARD_SITL
    _singleton = this;
}

void AP_BoardConfig_CAN::init()
{
    // Create all drivers that we need
    bool initret = true;
    for (uint8_t i = 0; i < MAX_NUMBER_OF_CAN_INTERFACES; i++) {
        // Check the driver number assigned to this physical interface
        uint8_t drv_num = _interfaces[i]._driver_number_cache = _interfaces[i]._driver_number;
        if (drv_num != 0 && drv_num <= MAX_NUMBER_OF_CAN_DRIVERS) {
            if (hal.can_mgr[drv_num - 1] == nullptr) {
                // CAN Manager is the driver
                // So if this driver was not created before for other physical interface - do it
                #if CONFIG_HAL_BOARD == HAL_BOARD_PX4 || CONFIG_HAL_BOARD == HAL_BOARD_VRBRAIN
                    const_cast <AP_HAL::HAL&> (hal).can_mgr[drv_num - 1] = new PX4::PX4CANManager;
                #elif CONFIG_HAL_BOARD == HAL_BOARD_LINUX
                    const_cast <AP_HAL::HAL&> (hal).can_mgr[drv_num - 1] = new Linux::CANManager;
                #elif CONFIG_HAL_BOARD == HAL_BOARD_CHIBIOS
                    const_cast <AP_HAL::HAL&> (hal).can_mgr[drv_num - 1] = new ChibiOS::CANManager;
                #endif
            }

            // For this now existing driver (manager), start the physical interface
            if (hal.can_mgr[drv_num - 1] != nullptr) {
                initret = initret && hal.can_mgr[drv_num - 1]->begin(_interfaces[i]._bitrate, i);
                #if CONFIG_HAL_BOARD == HAL_BOARD_CHIBIOS
                    if (_slcan_rt == (i+1) && hal.can_mgr[drv_num - 1] != nullptr ) {
                        ChibiOS_CAN::CanDriver* drv = (ChibiOS_CAN::CanDriver*)hal.can_mgr[drv_num - 1]->get_driver();
                        slcan_router().init(drv->getIface(i), drv->getUpdateEvent());
                    }
                #endif
            } else {
                printf("Failed to initialize can interface %d\n\r", i + 1);
            }
        }
    }

    if (initret) {
        for (uint8_t i = 0; i < MAX_NUMBER_OF_CAN_DRIVERS; i++) {
            Protocol_Type prot_type = _drivers[i]._protocol_type_cache = (Protocol_Type) _drivers[i]._protocol_type.get();

            if (hal.can_mgr[i] == nullptr) {
                continue;
            }

            _num_drivers = i + 1;
            hal.can_mgr[i]->initialized(true);
            printf("can_mgr %d initialized well\n\r", i + 1);

            if (prot_type == Protocol_Type_UAVCAN) {
                _drivers[i]._driver = _drivers[i]._uavcan =  new AP_UAVCAN;

                if (_drivers[i]._driver == nullptr) {
                    AP_HAL::panic("Failed to allocate uavcan %d\n\r", i + 1);
                    continue;
                }

                AP_Param::load_object_from_eeprom(_drivers[i]._uavcan, AP_UAVCAN::var_info);
            } else if (prot_type == Protocol_Type_KDECAN) {
// To be replaced with macro saying if KDECAN library is included
#if APM_BUILD_TYPE(APM_BUILD_ArduCopter) || APM_BUILD_TYPE(APM_BUILD_ArduPlane) || APM_BUILD_TYPE(APM_BUILD_ArduSub)
                _drivers[i]._driver = _drivers[i]._kdecan =  new AP_KDECAN;

                 if (_drivers[i]._driver == nullptr) {
                    AP_HAL::panic("Failed to allocate KDECAN %d\n\r", i + 1);
                    continue;
                }

                AP_Param::load_object_from_eeprom(_drivers[i]._kdecan, AP_KDECAN::var_info);
#endif
            } else if (prot_type == Protocol_Type_ToshibaCAN) {
                _drivers[i]._driver = _drivers[i]._tcan = new AP_ToshibaCAN;

                if (_drivers[i]._driver == nullptr) {
                    AP_BoardConfig::sensor_config_error("ToshibaCAN init failed");
                    continue;
                }
            } else {
                continue;
            }
            if (_slcan_rt == 0) {
                _drivers[i]._driver->init(i, true);
            } else {
                _drivers[i]._driver->init(i, false);
            }
        }
    }
}

AP_BoardConfig_CAN& AP::can() {
    return *AP_BoardConfig_CAN::get_singleton();
}
#endif

