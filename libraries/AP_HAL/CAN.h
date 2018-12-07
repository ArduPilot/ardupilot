/*
 * Copyright (C) 2017 Eugene Shamaev
 *
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
 */

#pragma once

#include <inttypes.h>
#include <string.h>
#include <assert.h>
#include <cmath>

#if HAL_WITH_UAVCAN

#include "AP_HAL_Namespace.h"
#include "utility/functor.h"
#include <uavcan/driver/can.hpp>
#include <uavcan/time.hpp>

#define MAX_NUMBER_OF_CAN_INTERFACES    2
#define MAX_NUMBER_OF_CAN_DRIVERS       2

/**
 * Interface that CAN protocols need to implement
 */
class AP_HAL::CANProtocol {
public:
    /* method called when initializing the CAN interfaces
     *
     * if initialization doesn't have errors, protocol class
     * should create a thread to do send and receive operations
     */
    virtual void init(uint8_t driver_index) = 0;
};

/**
 * Single non-blocking CAN interface.
 */
class AP_HAL::CAN: public uavcan::ICanIface {
public:
    /*  CAN port open method

     bitrate     Selects the speed that the port will be configured to.  If zero, the port speed is left unchanged.

     return      false - CAN port open failed
                 true  - CAN port open succeeded
     */
    virtual bool begin(uint32_t bitrate) = 0;

    /*
     CAN port close
     */
    virtual void end() = 0;

    /*
     Reset opened CAN port

     Pending messages to be transmitted are deleted and receive state and FIFO also reset.
     All pending errors are cleared.
     */
    virtual void reset() = 0;

    /*
     Test if CAN port is opened and initialized

     return      false - CAN port not initialized
                 true - CAN port is initialized
     */
    virtual bool is_initialized() = 0;

    /*
     Return if CAN port has some untransmitted pending messages

     return      -1 - CAN port is not opened or initialized
                 0 - no messages are pending
                 positive - number of pending messages
     */
    virtual int32_t tx_pending() = 0;

    /*
     Return if CAN port has received but not yet read messages

     return      -1 - CAN port is not opened or initialized
                 0 - no messages are pending for read
                 positive - number of pending messages for read
     */
    virtual int32_t available() = 0;

};

/**
 * Generic CAN driver.
 */
class AP_HAL::CANManager {
public:
    CANManager(uavcan::ICanDriver* driver) : _driver(driver) {}

    /*  CAN port open method
     Opens port with specified bit rate
     bitrate - selects the speed that the port will be configured to.  If zero, the port speed is left
     unchanged.
     can_number - the index of can interface to be opened

     return      false - CAN port open failed
                 true - CAN port open succeeded
     */
    virtual bool begin(uint32_t bitrate, uint8_t can_number) = 0;

    /*
     Test if CAN manager is ready and initialized
     return      false - CAN manager not initialized
     true - CAN manager is initialized
     */
    virtual bool is_initialized() = 0;
    virtual void initialized(bool val) = 0;

    uavcan::ICanDriver* get_driver() { return _driver; }
private:
    uavcan::ICanDriver* _driver;
};

#endif
