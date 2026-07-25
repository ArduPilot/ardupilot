#include "cxx_test_interface.h"

using namespace Canard;

void TestNetwork::route_msg(CoreTestInterface *send_iface, uint8_t source_node_id, uint8_t destination_node_id,
			    Transfer transfer, CanardTransferType transfer_type) {
    (void)destination_node_id;
    // prepare CanardRxTransfer
    CanardRxTransfer rx_transfer {};
    rx_transfer.data_type_id = transfer.data_type_id;
    rx_transfer.payload_len = (uint16_t)transfer.payload_len; 
    rx_transfer.payload_head = (uint8_t*)transfer.payload;
    rx_transfer.transfer_id = *transfer.inout_transfer_id;
    rx_transfer.source_node_id = source_node_id;
    rx_transfer.transfer_type = transfer.transfer_type;
#if CANARD_ENABLE_CANFD
    rx_transfer.tao = !transfer.canfd;
#elif CANARD_ENABLE_TAO_OPTION
    rx_transfer.tao = true;
#endif
    // send to all interfaces
    for (auto iface : ifaces) {
        if (iface != send_iface && iface != nullptr) {
	    iface->handle_transfer(rx_transfer, transfer_type);
        }
    }
}

/// @brief broadcast message to all listeners on Interface
/// @param bc_transfer
/// @return true if message was added to the queue
bool CoreTestInterface::broadcast(const Transfer &bcast_transfer) {
    // call network router
    TestNetwork::get_network().route_msg(this, node_id, 255, bcast_transfer, CanardTransferTypeBroadcast);
    return true;
}

/// @brief request message from
/// @param destination_node_id
/// @param req_transfer
/// @return true if request was added to the queue
bool CoreTestInterface::request(uint8_t destination_node_id, const Transfer &req_transfer) {
    // call network router
    TestNetwork::get_network().route_msg(this, node_id, destination_node_id, req_transfer, CanardTransferTypeRequest);
    return true;
}

/// @brief respond to a request
/// @param destination_node_id
/// @param res_transfer
/// @return true if response was added to the queue
bool CoreTestInterface::respond(uint8_t destination_node_id, const Transfer &res_transfer) {
    // call network router
    TestNetwork::get_network().route_msg(this, node_id, destination_node_id, res_transfer, CanardTransferTypeResponse);
    return true;
}

/// @brief set node id
/// @param node_id
void CoreTestInterface::set_node_id(uint8_t _node_id) {
    node_id = _node_id;
}

void CoreTestInterface::handle_transfer(CanardRxTransfer &transfer, CanardTransferType transfer_type) {
    uint64_t signature = 0;
    // check if message should be accepted
    if (accept_message(transfer.data_type_id, transfer_type, signature)) {
        // call message handler
        handle_message(transfer);
    }
}
