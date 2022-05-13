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
 * Author: Dmitry Ponomarev
 */
#pragma once

#include <AP_HAL/AP_HAL.h>

#if HAL_ENABLE_CYPHAL_DRIVERS

#include <AP_Param/AP_Param.h>
#include <AP_CYPHAL/AP_CYPHAL_IfaceMgr.h>
#include <AP_CYPHAL/AP_CYPHAL_subscriber.h>
#include <AP_CYPHAL/AP_CYPHAL_publisher.h>
#include <AP_CYPHAL/AP_CYPHAL_registers.h>
#include <AP_CYPHAL/AP_CYPHAL_esc.h>
#include <AP_CANManager/AP_CANDriver.h>
#include <AP_ESC_Telem/AP_ESC_Telem_Backend.h>
#include "canard.h"
#include "o1heap.h"

#ifndef CYPHAL_TX_QUEUE_FRAME_SIZE
#define CYPHAL_TX_QUEUE_FRAME_SIZE      512
#endif

#ifndef CYPHAL_STACK_SIZE
#define CYPHAL_STACK_SIZE               768
#endif

#ifndef CYPHAL_HEAP_SIZE
#define CYPHAL_HEAP_SIZE                (1024 * 3)
#endif


class AP_CYPHAL : public AP_CANDriver, public AP_ESC_Telem_Backend
{
public:
    AP_CYPHAL(): _registers(_parameters_table), _esc_controller(_registers) {};
    ~AP_CYPHAL() {};

    static const struct AP_Param::GroupInfo var_info[];

    // Return cyphal from @driver_index or nullptr if it's not ready or doesn't exist
    static AP_CYPHAL *get_cyphal(uint8_t driver_index);

    void init(uint8_t driver_index, bool enable_filters) override;
    bool add_interface(AP_HAL::CANIface* can_iface) override;

    void loop(void);

    void SRV_push_servos(void);

private:
    char _thread_name[10] = "cyphal";
    bool _initialized = false;

    CyphalTransportIface _transport_iface;
    AP_HAL::CANIface* _can_iface;

    CanardInstance _canard;
    CanardTxQueue _tx_queue;

    /**
     * @note Runs the node.
     * Normally your application should not block anywhere else.
     * @return 0 if no errors occurred, or negative if something failed.
     */
    int8_t spinReceive(uint16_t us);

    void spinTransmit();
    void processReceivedTransfer(const uint8_t iface_index, const CanardRxTransfer* transfer);

    ///< Application layer
    AP_Int16 _parameters_table[CyphalRegisters::NUMBER_OF_REGISTERS];

    CyphalRegisters _registers;
    CyphalSubscriberManager subscriber_manager;
    CyphalPublisherManager publisher_manager;

    CyphalEscController _esc_controller;
};

#endif // HAL_ENABLE_CYPHAL_DRIVERS
