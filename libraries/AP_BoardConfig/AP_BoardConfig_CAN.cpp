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
#endif

#include <AP_UAVCAN/AP_UAVCAN.h>

extern const AP_HAL::HAL& hal;

// table of user settable parameters
const AP_Param::GroupInfo AP_BoardConfig_CAN::var_info[] = {

#if MAX_NUMBER_OF_CAN_INTERFACES > 0
    // @Group: P1_
    // @Path: ../AP_BoardConfig/canbus.cpp
    AP_SUBGROUPINFO(_var_info_can[0], "P1_", 1, AP_BoardConfig_CAN, AP_BoardConfig_CAN::CAN_if_var_info),
#endif

#if MAX_NUMBER_OF_CAN_INTERFACES > 1
    // @Group: P2_
    // @Path: ../AP_BoardConfig/canbus.cpp
    AP_SUBGROUPINFO(_var_info_can[1], "P2_", 2, AP_BoardConfig_CAN, AP_BoardConfig_CAN::CAN_if_var_info),
#endif

#if MAX_NUMBER_OF_CAN_INTERFACES > 2
    // @Group: P3_
    // @Path: ../AP_BoardConfig/canbus.cpp
    AP_SUBGROUPINFO(_var_info_can[2], "P3_", 3, AP_BoardConfig_CAN, AP_BoardConfig_CAN::CAN_if_var_info),
#endif

#if MAX_NUMBER_OF_CAN_DRIVERS > 0
    // @Group: D1_
    // @Path: ../AP_BoardConfig/canbus_driver.cpp
    AP_SUBGROUPINFO(_var_info_can_protocol[0], "D1_", 4, AP_BoardConfig_CAN, AP_BoardConfig_CAN::CAN_driver_var_info),
#endif

#if MAX_NUMBER_OF_CAN_DRIVERS > 1
    // @Group: D2_
    // @Path: ../AP_BoardConfig/canbus_driver.cpp
    AP_SUBGROUPINFO(_var_info_can_protocol[1], "D2_", 5, AP_BoardConfig_CAN, AP_BoardConfig_CAN::CAN_driver_var_info),
#endif

#if MAX_NUMBER_OF_CAN_DRIVERS > 2
    // @Group: D3_
    // @Path: ../AP_BoardConfig/canbus_driver.cpp
    AP_SUBGROUPINFO(_var_info_can_protocol[2], "D3_", 6, AP_BoardConfig_CAN, AP_BoardConfig_CAN::CAN_driver_var_info),
#endif

    AP_GROUPEND
};

int8_t AP_BoardConfig_CAN::_st_driver_number[MAX_NUMBER_OF_CAN_INTERFACES];
int8_t AP_BoardConfig_CAN::_st_can_debug[MAX_NUMBER_OF_CAN_INTERFACES];

void AP_BoardConfig_CAN::init()
{
    for (uint8_t i = 0; i < MAX_NUMBER_OF_CAN_INTERFACES; i++)
    {
        _st_driver_number[i] = (int8_t) _var_info_can[i]._driver_number;
        _st_can_debug[i] = (int8_t) _var_info_can[i]._can_debug;
    }

    setup_canbus();
}

void AP_BoardConfig_CAN::setup_canbus(void)
{
    // Create all drivers that we need
    bool initret = true;
    for (uint8_t i = 0; i < MAX_NUMBER_OF_CAN_INTERFACES; i++) {
        // Check the driver number assigned to this physical interface
        uint8_t drv_num = _var_info_can[i]._driver_number;

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
                initret &= hal.can_mgr[drv_num - 1]->begin(_var_info_can[i]._can_bitrate, i);
            } else {
                printf("Failed to initialize can interface %d\n\r", i + 1);
            }
        }
    }

    bool any_uavcan_present = false;

    if (initret) {
        for (uint8_t i = 0; i < MAX_NUMBER_OF_CAN_DRIVERS; i++) {
            if (hal.can_mgr[i] == nullptr) {
                continue;
            }
            hal.can_mgr[i]->initialized(true);
            printf("can_mgr %d initialized well\n\r", i + 1);

            if (_var_info_can_protocol[i]._protocol == UAVCAN_PROTOCOL_ENABLE) {
                _var_info_can_protocol[i]._uavcan = new AP_UAVCAN;

                if (_var_info_can_protocol[i]._uavcan == nullptr) {
                    AP_HAL::panic("Failed to allocate uavcan %d\n\r", i + 1);
                    continue;
                }
                
                AP_Param::load_object_from_eeprom(_var_info_can_protocol[i]._uavcan, AP_UAVCAN::var_info);

                hal.can_mgr[i]->set_UAVCAN(_var_info_can_protocol[i]._uavcan);
                _var_info_can_protocol[i]._uavcan->set_parent_can_mgr(hal.can_mgr[i]);

                if (_var_info_can_protocol[i]._uavcan->try_init() == true) {
                    any_uavcan_present = true;
                } else {
                    printf("Failed to initialize uavcan interface %d\n\r", i + 1);
                }
            }
        }

        if (any_uavcan_present) {
            // start UAVCAN working thread
            hal.scheduler->create_uavcan_thread();

            // Delay for magnetometer and barometer discovery
            hal.scheduler->delay(5000);
        }
    }
}
#endif

