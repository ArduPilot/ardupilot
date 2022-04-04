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

#include <AP_UAVCAN_V1/AP_UAVCAN_V1_publisher.h>

#if HAL_ENABLE_LIBUAVCAN_DRIVERS

#include <AP_UAVCAN_V1/AP_UAVCAN_V1_registers.h>


void UavcanPublisherManager::init(CanardInstance &ins, CanardTxQueue& tx_queue)
{
    UavcanBasePublisher *publisher;

    ///< Minimal requrement
    publisher = new UavcanHeartbeatPublisher(ins, tx_queue);
    add_publisher(publisher);

    publisher = new UavcanPortListPublisher(ins, tx_queue);
    add_publisher(publisher);
}

bool UavcanPublisherManager::add_publisher(UavcanBasePublisher *publisher)
{
    if (publisher == nullptr || number_of_publishers >= max_number_of_publishers) {
        return false;
    }

    publishers[number_of_publishers] = publisher;
    number_of_publishers++;
    return true;
}

void UavcanPublisherManager::process_all()
{
    for (uint_fast8_t pub_idx = 0; pub_idx < number_of_publishers; pub_idx++) {
        if (publishers[pub_idx] != nullptr && publishers[pub_idx]->get_port_id() != 0) {
            publishers[pub_idx]->update();
        }
    }
}


void UavcanBasePublisher::push(size_t buf_size, uint8_t* buf)
{
    auto result = canardTxPush(&_tx_queue, &_canard, 0, &_transfer_metadata, buf_size, buf);
    if (result < 0) {
        // An error has occurred: either an argument is invalid, the TX queue is full, or we've
        // run out of memory. It is possible to statically prove that an out-of-memory will
        // never occur for a given application if the heap is sized correctly; for background,
        // refer to the Robson's Proof and the documentation for O1Heap.
    }
}


/**
 * @note uavcan.node.Heartbeat_1_0
 */
UavcanHeartbeatPublisher::UavcanHeartbeatPublisher(CanardInstance &ins, CanardTxQueue& tx_queue) :
    UavcanBasePublisher(ins, tx_queue, uavcan_node_Heartbeat_1_0_FIXED_PORT_ID_)
{
    msg.health.value = uavcan_node_Health_1_0_NOMINAL;
    msg.mode.value = uavcan_node_Mode_1_0_OPERATIONAL;
    msg.vendor_specific_status_code = static_cast<uint8_t>(0);

    _transfer_metadata.priority = CanardPriorityNominal;
    _transfer_metadata.transfer_kind = CanardTransferKindMessage;
    _transfer_metadata.port_id = uavcan_node_Heartbeat_1_0_FIXED_PORT_ID_;
    _transfer_metadata.remote_node_id = CANARD_NODE_ID_UNSET;
    _transfer_metadata.transfer_id = 0;
}

void UavcanHeartbeatPublisher::update()
{
    if (AP_HAL::millis() < next_publish_time_ms) {
        return;
    }
    next_publish_time_ms += publish_period_ms;

    publish();
}

void UavcanHeartbeatPublisher::publish()
{
    msg.uptime = AP_HAL::millis() / 1000;

    uint8_t buf[uavcan_node_Heartbeat_1_0_SERIALIZATION_BUFFER_SIZE_BYTES_];
    size_t buf_size = uavcan_node_Heartbeat_1_0_SERIALIZATION_BUFFER_SIZE_BYTES_;
    int32_t result = uavcan_node_Heartbeat_1_0_serialize_(&msg, buf, &buf_size);
    if (NUNAVUT_SUCCESS == result) {
        push(buf_size, buf);
    }

    _transfer_metadata.transfer_id++;
}


/**
 * @note uavcan.node.port.List_0_1
 */
UavcanPortListPublisher::UavcanPortListPublisher(CanardInstance &ins, CanardTxQueue& tx_queue) :
    UavcanBasePublisher(ins, tx_queue, uavcan_node_port_List_0_1_FIXED_PORT_ID_)
{
    _transfer_metadata.priority = CanardPriorityNominal;
    _transfer_metadata.transfer_kind = CanardTransferKindMessage;
    _transfer_metadata.port_id = uavcan_node_port_List_0_1_FIXED_PORT_ID_;
    _transfer_metadata.remote_node_id = CANARD_NODE_ID_UNSET;
    _transfer_metadata.transfer_id = 0;
}

void UavcanPortListPublisher::update()
{
    if (AP_HAL::millis() < next_publish_time_ms) {
        return;
    }
    next_publish_time_ms += publish_period_ms;

    publish();
}

void UavcanPortListPublisher::publish()
{
    ///< @todo How much bits does it need? 8466?
    // uint8_t buf[uavcan_node_port_List_0_1_SERIALIZATION_BUFFER_SIZE_BYTES_];
    // size_t buf_size = uavcan_node_port_List_0_1_SERIALIZATION_BUFFER_SIZE_BYTES_;
    // auto result = uavcan_node_port_List_0_1_serialize_(&msg, buf, &buf_size);

    // if (NUNAVUT_SUCCESS == result) {
    //     push(buf_size, buf);
    // }

    // _transfer_metadata.transfer_id++;
}

#endif // HAL_ENABLE_LIBUAVCAN_DRIVERS
