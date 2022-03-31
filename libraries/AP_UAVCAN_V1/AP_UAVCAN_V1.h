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
#include <AP_Param/AP_Param.h>
#include <AP_UAVCAN_V1/AP_UAVCAN_V1_IfaceMgr.h>
#include <AP_UAVCAN_V1/AP_UAVCAN_V1_subscriber.h>
#include <AP_UAVCAN_V1/AP_UAVCAN_V1_publisher.h>
#include <AP_UAVCAN_V1/AP_UAVCAN_V1_registers.h>
#include <AP_UAVCAN_V1/AP_UAVCAN_V1_esc.h>
#include <AP_CANManager/AP_CANDriver.h>
#include <AP_ESC_Telem/AP_ESC_Telem_Backend.h>
#include "canard.h"
#include "o1heap.h"

#include "uavcan/node/GetInfo_1_0.h"


#ifndef UAVCAN_TX_QUEUE_FRAME_SIZE
#define UAVCAN_TX_QUEUE_FRAME_SIZE      512
#endif

#ifndef UAVCAN_STACK_SIZE
#define UAVCAN_STACK_SIZE               768
#endif

#ifndef UAVCAN_HEAP_SIZE
#define UAVCAN_HEAP_SIZE                (1024 * 3)
#endif


class AP_UAVCAN_V1 : public AP_CANDriver, public AP_ESC_Telem_Backend
{
public:
    AP_UAVCAN_V1(): _registers(_parameters_table), _esc_controller(_registers) {};
    ~AP_UAVCAN_V1() {};

    static const struct AP_Param::GroupInfo var_info[];

    // Return uavcan from @driver_index or nullptr if it's not ready or doesn't exist
    static AP_UAVCAN_V1 *get_uavcan(uint8_t driver_index);

    void init(uint8_t driver_index, bool enable_filters) override;
    bool add_interface(AP_HAL::CANIface* can_iface) override;

    void loop(void);

    void SRV_push_servos(void);
    void set_ESC_status();

private:
    char _thread_name[10] = "uavcan_v1";
    bool _initialized = false;

    UavcanFirstTransportIface _transport_iface;
    AP_HAL::CANIface* _can_iface;

    CanardInstance _canard;
    CanardTxQueue _tx_queue;

    void spinReceive();
    void spinTransmit();
    void processReceivedTransfer(const uint8_t iface_index, const CanardRxTransfer* transfer);

    ///< Application layer
    AP_Int16 _parameters_table[UavcanRegisters::NUMBER_OF_REGISTERS];

    UavcanRegisters _registers;
    UavcanSubscriberManager subscriber_manager;
    UavcanPublisherManager publisher_manager;

    UavcanEscController _esc_controller;
};
