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

#include <AP_CYPHAL/AP_CYPHAL_subscriber.h>

#if HAL_ENABLE_CYPHAL_DRIVERS

#include <AP_CYPHAL/AP_CYPHAL.h>
#include <AP_CYPHAL/AP_CYPHAL_registers.h>
#include <AP_Param/AP_Param.h>
#include <GCS_MAVLink/GCS.h>
#include <GCS_MAVLink/GCS_MAVLink.h>


void CyphalSubscriberManager::init(CanardInstance &ins, CanardTxQueue& tx_queue)
{
    CyphalBaseSubscriber *subsriber;

    ///< Minimal requrement
    subsriber = new CyphalHeartbeatSubscriber(ins, tx_queue);
    add_subscriber(subsriber);

    subsriber = new CyphalGetInfoRequest(ins, tx_queue);
    add_subscriber(subsriber);

    subsriber = new CyphalNodeExecuteCommandRequest(ins, tx_queue);
    add_subscriber(subsriber);
}

bool CyphalSubscriberManager::add_subscriber(CyphalBaseSubscriber *subsriber)
{
    if (subsriber == nullptr || number_of_subscribers >= max_number_of_subscribers) {
        return false;
    }

    auto port_id = subsriber->get_port_id();
    if (port_id == 0 || port_id == CyphalRegisters::CYPHAL_INVALID_REGISTER_VALUE) {
        return false;
    }

    subscribers[number_of_subscribers] = subsriber;
    subsriber->subscribe();
    number_of_subscribers++;
    return true;
}


void CyphalSubscriberManager::process_all(const CanardRxTransfer* transfer)
{
    auto port_id = transfer->metadata.port_id;
    for (uint_fast8_t sub_idx = 0; sub_idx < number_of_subscribers; sub_idx++) {
        if (subscribers[sub_idx] != nullptr && port_id == subscribers[sub_idx]->get_port_id()) {
            subscribers[sub_idx]->handler(transfer);
            break;
        }
    }
}


CanardPortID CyphalBaseSubscriber::get_port_id()
{
    return _port_id;
}

void CyphalBaseSubscriber::subscribeOnMessage(const size_t extent)
{
    (void) canardRxSubscribe(_canard,
                             CanardTransferKindMessage,
                             _port_id,
                             extent,
                             CANARD_DEFAULT_TRANSFER_ID_TIMEOUT_USEC,
                             &_subscription);
}

void CyphalBaseSubscriber::subscribeOnRequest(const size_t extent)
{
    (void) canardRxSubscribe(_canard,
                             CanardTransferKindRequest,
                             _port_id,
                             extent,
                             CANARD_DEFAULT_TRANSFER_ID_TIMEOUT_USEC,
                             &_subscription);
}

CyphalBaseSubscriber::CyphalBaseSubscriber(uint8_t register_idx) {
    init(register_idx);
}

bool CyphalBaseSubscriber::init(uint8_t register_idx) {
    AP_CYPHAL* cyphal = AP_CYPHAL::get_cyphal(0);
    if (cyphal == nullptr) {
        return false;
    }

    auto registers = cyphal->get_registers();
    _canard = &cyphal->get_canard_instance();
    _tx_queue = &cyphal->get_tx_queue();
    _port_id = registers.getPortIdByIndex(register_idx);
    return true;
}

void CyphalRequestSubscriber::push_response(size_t buf_size, uint8_t* buf)
{
    const CanardMicrosecond tx_deadline_usec = AP_HAL::micros() + 100000;
    auto result = canardTxPush(_tx_queue, _canard, tx_deadline_usec, &_transfer_metadata, buf_size, buf);

    // An error has occurred: either an argument is invalid, the TX queue is full, or we've
    // run out of memory. It is possible to statically prove that an out-of-memory will
    // never occur for a given application if the heap is sized correctly; for background,
    // refer to the Robson's Proof and the documentation for O1Heap.
    if (result == -CANARD_ERROR_OUT_OF_MEMORY) {
        GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "v1: response err: OUT_OF_MEMORY");
    } else if (result < 0) {
        GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "v1: response err");
    }
}


/**
 * @note uavcan.node.Heartbeat.1.0
 */
CyphalHeartbeatSubscriber::CyphalHeartbeatSubscriber(CanardInstance &ins, CanardTxQueue& tx_queue) :
    CyphalBaseSubscriber(ins, tx_queue, uavcan_node_Heartbeat_1_0_FIXED_PORT_ID_)
{
}

void CyphalHeartbeatSubscriber::subscribe()
{
    subscribeOnMessage(uavcan_node_Heartbeat_1_0_EXTENT_BYTES_);
}

void CyphalHeartbeatSubscriber::handler(const CanardRxTransfer* transfer)
{
}


/**
 * @note uavcan.node.GetInfo.1.0
 */
CyphalGetInfoRequest::CyphalGetInfoRequest(CanardInstance &ins, CanardTxQueue& tx_queue) :
    CyphalRequestSubscriber(ins, tx_queue, uavcan_node_GetInfo_1_0_FIXED_PORT_ID_)
{
    _node_status.protocol_version.major = CANARD_CYPHAL_SPECIFICATION_VERSION_MAJOR;
    _node_status.protocol_version.minor = CANARD_CYPHAL_SPECIFICATION_VERSION_MINOR;
    _node_status.hardware_version.major = 0;
    _node_status.hardware_version.minor = 1;
    _node_status.software_version.major = 0;
    _node_status.software_version.minor = 1;

    char node_name[] = "org.ardupilot";
    _node_status.name.count = sizeof(node_name);
    memcpy(_node_status.name.elements, node_name, _node_status.name.count);
}

void CyphalGetInfoRequest::subscribe()
{
    subscribeOnRequest(uavcan_node_GetInfo_Request_1_0_EXTENT_BYTES_);
}

void CyphalGetInfoRequest::handler(const CanardRxTransfer* transfer)
{
    makeResponse(transfer);
}

void CyphalGetInfoRequest::makeResponse(const CanardRxTransfer* transfer)
{
    _transfer_metadata.transfer_id = transfer->metadata.transfer_id;
    _transfer_metadata.remote_node_id = transfer->metadata.remote_node_id;

    uint8_t buf[uavcan_node_GetInfo_Response_1_0_SERIALIZATION_BUFFER_SIZE_BYTES_];
    size_t buf_size = uavcan_node_GetInfo_Response_1_0_SERIALIZATION_BUFFER_SIZE_BYTES_;
    int32_t result = uavcan_node_GetInfo_Response_1_0_serialize_(&_node_status, buf, &buf_size);
    if (NUNAVUT_SUCCESS == result) {
        push_response(buf_size, buf);
    }
}


/**
 * @note uavcan.node.ExecuteCommand
 */
void CyphalNodeExecuteCommandRequest::subscribe()
{
    subscribeOnRequest(uavcan_node_ExecuteCommand_Request_1_0_EXTENT_BYTES_);
}

void CyphalNodeExecuteCommandRequest::handler(const CanardRxTransfer* transfer)
{
    makeResponse(transfer);
}

void CyphalNodeExecuteCommandRequest::makeResponse(const CanardRxTransfer* transfer)
{
    GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "AP_CYPHAL: ExecuteCommand not implemented yet");
}

#endif // HAL_ENABLE_CYPHAL_DRIVERS
