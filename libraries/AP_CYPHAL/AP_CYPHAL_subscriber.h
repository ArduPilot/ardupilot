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

#include "canard.h"

#include "uavcan/node/Heartbeat_1_0.h"
#include "uavcan/node/ExecuteCommand_1_0.h"
#include "uavcan/node/GetInfo_1_0.h"
#include "uavcan/_register/Access_1_0.h"
#include "uavcan/_register/List_1_0.h"


class CyphalBaseSubscriber;


class CyphalSubscriberManager
{
public:
    CyphalSubscriberManager() {};
    CyphalSubscriberManager(const CyphalSubscriberManager&) = delete;
    void init(CanardInstance &ins, CanardTxQueue& tx_queue);
    void process_all(const CanardRxTransfer *transfer);
    bool add_subscriber(CyphalBaseSubscriber *subsriber);
private:
    static constexpr uint8_t max_number_of_subscribers = 17;    /// default (5) + esc (4*3)
    uint8_t number_of_subscribers = 0;
    CyphalBaseSubscriber* subscribers[max_number_of_subscribers];
};


class CyphalBaseSubscriber
{
public:
    CyphalBaseSubscriber(uint8_t register_idx);
    CyphalBaseSubscriber(CanardInstance &ins, CanardTxQueue& tx_queue, CanardPortID port_id) :
        _canard(&ins), _tx_queue(&tx_queue), _port_id(port_id) {};

    CanardPortID get_port_id();
    virtual void subscribe() = 0;
    virtual void handler(const CanardRxTransfer* transfer) = 0;
protected:
    void subscribeOnMessage(const size_t extent);
    void subscribeOnRequest(const size_t extent);

    CanardRxSubscription _subscription;
    CanardPortID _port_id;
    CanardInstance *_canard;
    CanardTxQueue *_tx_queue;
private:
    bool init(uint8_t register_idx);
};


class CyphalRequestSubscriber: public CyphalBaseSubscriber
{
public:
    CyphalRequestSubscriber(CanardInstance &ins, CanardTxQueue& tx_queue, CanardPortID port_id) :
        CyphalBaseSubscriber(ins, tx_queue, port_id)
    {
        _transfer_metadata.priority = CanardPriorityNominal;
        _transfer_metadata.transfer_kind = CanardTransferKindResponse;
        _transfer_metadata.port_id = port_id;
    }
protected:
    void push_response(size_t buf_size, uint8_t* buf);
    CanardTransferMetadata _transfer_metadata;
};


/**
 * @note uavcan.node.Heartbeat.1.0
 */
class CyphalHeartbeatSubscriber: public CyphalBaseSubscriber
{
public:
    CyphalHeartbeatSubscriber(CanardInstance &ins, CanardTxQueue& tx_queue);
    virtual void subscribe() override;
    virtual void handler(const CanardRxTransfer* transfer) override;
};


/**
 * @note uavcan.node.GetInfo.1.0
 */
class CyphalGetInfoRequest: public CyphalRequestSubscriber
{
public:
    CyphalGetInfoRequest(CanardInstance &ins, CanardTxQueue& tx_queue);
    virtual void subscribe() override;
    virtual void handler(const CanardRxTransfer* transfer) override;
private:
    void makeResponse(const CanardRxTransfer* transfer);
    uavcan_node_GetInfo_Response_1_0 _node_status;
};


/**
 * @note uavcan.node.ExecuteCommand
 */
class CyphalNodeExecuteCommandRequest: public CyphalRequestSubscriber
{
public:
    CyphalNodeExecuteCommandRequest(CanardInstance &ins, CanardTxQueue& tx_queue) :
        CyphalRequestSubscriber(ins, tx_queue, uavcan_node_ExecuteCommand_1_0_FIXED_PORT_ID_) {};
    virtual void subscribe() override;
    virtual void handler(const CanardRxTransfer* transfer) override;
private:
    void makeResponse(const CanardRxTransfer* transfer);
};

#endif // HAL_ENABLE_CYPHAL_DRIVERS
