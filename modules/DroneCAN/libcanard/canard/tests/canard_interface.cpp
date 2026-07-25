#include "canard_interface.h"
#include <iostream>
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wmissing-field-initializers"
#include <time.h>

using namespace Canard;

void CanardInterface::init(void* mem_arena, size_t mem_arena_size) {
    canardInit(&canard, mem_arena, mem_arena_size, onTransferReception, shouldAcceptTransfer, this);
}

bool CanardInterface::broadcast(const Transfer &bcast_transfer) {
#if CANARD_ENABLE_DEADLINE
    // get current time in microseconds
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    uint64_t timestamp_usec = uint64_t(ts.tv_sec) * 1000000ULL + uint64_t(ts.tv_nsec) / 1000ULL;
#endif
    // do canard broadcast
    CanardTxTransfer tx_transfer = {
        .transfer_type = bcast_transfer.transfer_type,
        .data_type_signature = bcast_transfer.data_type_signature,
        .data_type_id = bcast_transfer.data_type_id,
        .inout_transfer_id = bcast_transfer.inout_transfer_id,
        .priority = bcast_transfer.priority,
        .payload = (const uint8_t*)bcast_transfer.payload,
        .payload_len = (uint16_t)bcast_transfer.payload_len,
#if CANARD_ENABLE_CANFD
        .canfd = bcast_transfer.canfd,
#endif
#if CANARD_ENABLE_DEADLINE
        .deadline_usec = timestamp_usec + (bcast_transfer.timeout_ms*1000),
#endif
#if CANARD_MULTI_IFACE
        .iface_mask = CANARD_IFACE_ALL,
#endif
    };

    return canardBroadcastObj(&canard, &tx_transfer) > 0;
}

bool CanardInterface::request(uint8_t destination_node_id, const Transfer &req_transfer) {
#if CANARD_ENABLE_DEADLINE
    // get current time in microseconds
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    uint64_t timestamp_usec = uint64_t(ts.tv_sec) * 1000000ULL + uint64_t(ts.tv_nsec) / 1000ULL;
#endif

    // do canard request
    CanardTxTransfer tx_transfer = {
        .transfer_type = req_transfer.transfer_type,
        .data_type_signature = req_transfer.data_type_signature,
        .data_type_id = req_transfer.data_type_id,
        .inout_transfer_id = req_transfer.inout_transfer_id,
        .priority = req_transfer.priority,
        .payload = (const uint8_t*)req_transfer.payload,
        .payload_len = (uint16_t)req_transfer.payload_len,
#if CANARD_ENABLE_CANFD
        .canfd = req_transfer.canfd,
#endif
#if CANARD_ENABLE_DEADLINE
        .deadline_usec = timestamp_usec + (req_transfer.timeout_ms*1000),
#endif
#if CANARD_MULTI_IFACE
        .iface_mask = CANARD_IFACE_ALL,
#endif
    };
    return canardRequestOrRespondObj(&canard, destination_node_id, &tx_transfer) > 0;
}

bool CanardInterface::respond(uint8_t destination_node_id, const Transfer &res_transfer) {
#if CANARD_ENABLE_DEADLINE
    // get current time in microseconds
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    uint64_t timestamp_usec = uint64_t(ts.tv_sec) * 1000000ULL + uint64_t(ts.tv_nsec) / 1000ULL;
#endif
    // do canard respond
    CanardTxTransfer tx_transfer = {
        .transfer_type = res_transfer.transfer_type,
        .data_type_signature = res_transfer.data_type_signature,
        .data_type_id = res_transfer.data_type_id,
        .inout_transfer_id = res_transfer.inout_transfer_id,
        .priority = res_transfer.priority,
        .payload = (const uint8_t*)res_transfer.payload,
        .payload_len = (uint16_t)res_transfer.payload_len,
#if CANARD_ENABLE_CANFD
        .canfd = res_transfer.canfd,
#endif
#if CANARD_ENABLE_DEADLINE
        .deadline_usec = timestamp_usec + (res_transfer.timeout_ms*1000),
#endif
#if CANARD_MULTI_IFACE
        .iface_mask = CANARD_IFACE_ALL,
#endif
    };
    return canardRequestOrRespondObj(&canard, destination_node_id, &tx_transfer) > 0;
}

void CanardInterface::handle_frame(const CanardCANFrame &frame, uint64_t timestamp_usec) {
    int16_t err = canardHandleRxFrame(&canard, &frame, timestamp_usec);
    if (err < 0) {
        std::cout << "Error handling frame: " << err << std::endl;
        CANARD_ASSERT(false);
    }
}

void CanardInterface::onTransferReception(CanardInstance* ins, CanardRxTransfer* transfer) {
    CanardInterface* iface = (CanardInterface*) ins->user_reference;
    iface->handle_message(*transfer);
}

bool CanardInterface::shouldAcceptTransfer(const CanardInstance* ins,
                                           uint64_t* out_data_type_signature,
                                           uint16_t data_type_id,
                                           CanardTransferType transfer_type,
                                           uint8_t source_node_id) {
    (void)transfer_type;
    (void)source_node_id;
    CanardInterface* iface = (CanardInterface*) ins->user_reference;
    return iface->accept_message(data_type_id, transfer_type, *out_data_type_signature);
}

void CanardTestNetwork::route_frame(CanardTestInterface *send_iface, const CanardCANFrame &frame, uint64_t timestamp_usec) {
    for (int i = 0; i < N; i++) {
        if ((send_iface != ifaces[i]) && (ifaces[i] != nullptr)) {
            ifaces[i]->handle_frame(frame, timestamp_usec);
        }
    }
}

void CanardTestInterface::update_tx(uint64_t timestamp_usec) {
    for (const CanardCANFrame* txf = canardPeekTxQueue(&canard); txf != nullptr; txf = canardPeekTxQueue(&canard)) {
        CanardTestNetwork::get_network().route_frame(this, *txf, timestamp_usec);
        canardPopTxQueue(&canard);
    }
}

void CanardTestInterface::set_node_id(uint8_t node_id) {
    canardSetLocalNodeID(&canard, node_id);
}
#pragma GCC diagnostic pop
