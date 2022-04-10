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
#include "AP_CYPHAL_subscriber.h"


enum UavcanRegister_t {
    UAVCAN_PUB_NODE_ID,

    UAVCAN_PUB_NOTE_RESPONSE_ID,
    UAVCAN_PUB_SETPOINT_ID,
    UAVCAN_PUB_READINESS_ID,

    UAVCAN_SUB_ESC_HEARTBEAT_0_ID,
    UAVCAN_SUB_FEEDBACK_0_ID,
    UAVCAN_SUB_POWER_0_ID,
    UAVCAN_SUB_STATUS_0_ID,
    UAVCAN_SUB_DYNAMICS_0_ID,

    UAVCAN_SUB_ESC_HEARTBEAT_1_ID,
    UAVCAN_SUB_FEEDBACK_1_ID,
    UAVCAN_SUB_POWER_1_ID,
    UAVCAN_SUB_STATUS_1_ID,
    UAVCAN_SUB_DYNAMICS_1_ID,

    UAVCAN_SUB_ESC_HEARTBEAT_2_ID,
    UAVCAN_SUB_FEEDBACK_2_ID,
    UAVCAN_SUB_POWER_2_ID,
    UAVCAN_SUB_STATUS_2_ID,
    UAVCAN_SUB_DYNAMICS_2_ID,

    UAVCAN_SUB_ESC_HEARTBEAT_3_ID,
    UAVCAN_SUB_FEEDBACK_3_ID,
    UAVCAN_SUB_POWER_3_ID,
    UAVCAN_SUB_STATUS_3_ID,
    UAVCAN_SUB_DYNAMICS_3_ID,
};


class UavcanRegisters
{
public:
    static constexpr uint8_t NUMBER_OF_REGISTERS = 24;
    static constexpr uint16_t UAVCAN_INVALID_REGISTER_VALUE = 65535;

    UavcanRegisters(AP_Int16 (&parameters_table)[NUMBER_OF_REGISTERS]) : _parameters_table(parameters_table) {}
    bool init(UavcanSubscriberManager &sub_manager, CanardInstance &ins, CanardTxQueue& tx_queue);

    // Return size of register name, otherwise 0
    uint8_t getRegisterNameByIndex(uint8_t register_index, uint8_t register_name[]);

    // Return [0, 127] if correct, -1 if register is not exist
    int8_t getRegisterIndexByRegisterName(const uint8_t register_name[], uint8_t name_length);

    // Return [0, 8191] if correct, -1 if register is not exist
    int16_t getPortIdByIndex(uint8_t param_idx);

    void setPortIdByIndex(uint8_t register_index, int16_t value);

private:
    AP_Int16 (&_parameters_table)[NUMBER_OF_REGISTERS];
};


/**
 * @note uavcan.register.Access.1.0
 */
class UavcanRegisterAccessRequest: public UavcanRequestSubscriber
{
public:
    UavcanRegisterAccessRequest(CanardInstance &ins, CanardTxQueue& tx_queue, UavcanRegisters &uavcan_registers) :
        UavcanRequestSubscriber(ins, tx_queue, uavcan_register_Access_1_0_FIXED_PORT_ID_),
        _registers(uavcan_registers) {};
    virtual void subscribe() override;
    virtual void handler(const CanardRxTransfer* transfer) override;
private:
    uavcan_register_Access_Request_1_0 _request_msg;
    uavcan_register_Access_Response_1_0 response_msg;
    UavcanRegisters &_registers;

    int8_t parseRequest(const CanardRxTransfer* transfer);
    void makeResponse(const CanardRxTransfer* transfer, int8_t reg_index);
};


/**
 * @note uavcan.register.List.1.0
 */
class UavcanRegisterListRequest: public UavcanRequestSubscriber
{
public:
    UavcanRegisterListRequest(CanardInstance &ins, CanardTxQueue& tx_queue, UavcanRegisters &uavcan_registers) :
        UavcanRequestSubscriber(ins, tx_queue, uavcan_register_List_1_0_FIXED_PORT_ID_),
        _registers(uavcan_registers) {};
    virtual void subscribe() override;
    virtual void handler(const CanardRxTransfer* transfer) override;
private:
    uavcan_register_List_Response_1_0 _response_msg = {};
    UavcanRegisters &_registers;

    uint16_t parseRequest(const CanardRxTransfer* transfer);
    void makeResponse(const CanardRxTransfer* transfer, uint16_t index);
};

#endif // HAL_ENABLE_CYPHAL_DRIVERS
