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
 * Code by Andrew Tridgell and Siddharth Bharat Purohit
 */

#pragma once

#include "AP_HAL_ChibiOS.h"

#if HAL_WITH_UAVCAN

#define UAVCAN_STM32_LOG(fmt, ...)  hal.console->printf("CANManager: " fmt "\n", ##__VA_ARGS__)

#include <uavcan/uavcan.hpp>
#include <uavcan/time.hpp>

#include <uavcan_stm32/thread.hpp>
#include <uavcan_stm32/clock.hpp>
#include <uavcan_stm32/can.hpp>

#include <AP_UAVCAN/AP_UAVCAN.h>

#define MAX_NUMBER_OF_CAN_INTERFACES    2
#define MAX_NUMBER_OF_CAN_DRIVERS       2
#define CAN_STM32_RX_QUEUE_SIZE         64
class AP_UAVCAN;

namespace ChibiOS {
/**
 * Generic CAN driver.
 */
class CANManager: public AP_HAL::CANManager {
public:
    CANManager()
        :  can_helper(), AP_HAL::CANManager(&can_helper.driver)   { }

    /**
     * Whether at least one iface had at least one successful IO since previous call of this method.
     * This is designed for use with iface activity LEDs.
     */
    //bool hadActivity();

    static CANManager *from(AP_HAL::CANManager *can)
    {
        return static_cast<CANManager*>(can);
    }

    bool begin(uint32_t bitrate, uint8_t can_number) override;

    /*
     Test if CAN manager is ready and initialized
     return      false - CAN manager not initialized
     true - CAN manager is initialized
     */
    bool is_initialized() override;
    void initialized(bool val) override;

    AP_UAVCAN *get_UAVCAN(void) override;
    void set_UAVCAN(AP_UAVCAN *uavcan) override;
    void _timer_tick();

private:
    AP_UAVCAN *p_uavcan;
    bool initialized_;
    uint32_t bitrate_;
    uavcan_stm32::CanInitHelper<CAN_STM32_RX_QUEUE_SIZE> can_helper;
};

}
#endif
