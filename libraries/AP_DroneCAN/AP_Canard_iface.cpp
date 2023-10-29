#include "AP_Canard_iface.h"
#include <AP_HAL/AP_HAL.h>
#include <AP_CANManager/AP_CANManager.h>
#if HAL_ENABLE_DRONECAN_DRIVERS
#include <canard/handler_list.h>
#include <canard/transfer_object.h>
#include <AP_Math/AP_Math.h>
#include <dronecan_msgs.h>
extern const AP_HAL::HAL& hal;
#define LOG_TAG "DroneCANIface"
#include <canard.h>

#define DEBUG_PKTS 0

#define CANARD_MSG_TYPE_FROM_ID(x)                         ((uint16_t)(((x) >> 8U)  & 0xFFFFU))

DEFINE_HANDLER_LIST_HEADS();
DEFINE_HANDLER_LIST_SEMAPHORES();

DEFINE_TRANSFER_OBJECT_HEADS();
DEFINE_TRANSFER_OBJECT_SEMAPHORES();

#if AP_TEST_DRONECAN_DRIVERS
CanardInterface* CanardInterface::canard_ifaces[] = {nullptr, nullptr, nullptr};
CanardInterface CanardInterface::test_iface{2};
uint8_t test_node_mem_area[1024];
HAL_Semaphore test_iface_sem;
#endif

void canard_allocate_sem_take(CanardPoolAllocator *allocator) {
    if (allocator->semaphore == nullptr) {
        allocator->semaphore = new HAL_Semaphore;
        if (allocator->semaphore == nullptr) {
            // out of memory
            CANARD_ASSERT(0);
            return;
        }
    }
    ((HAL_Semaphore*)allocator->semaphore)->take_blocking();
}

void canard_allocate_sem_give(CanardPoolAllocator *allocator) {
    if (allocator->semaphore == nullptr) {
        // it should have been allocated by canard_allocate_sem_take
        CANARD_ASSERT(0);
        return;
    }
    ((HAL_Semaphore*)allocator->semaphore)->give();
}

CanardInterface::CanardInterface(uint8_t iface_index) :
Interface(iface_index) {
#if AP_TEST_DRONECAN_DRIVERS
    if (iface_index < 3) {
        canard_ifaces[iface_index] = this;
    }
    if (iface_index == 0) {
        test_iface.init(test_node_mem_area, sizeof(test_node_mem_area), 125);
    }
    canardInitTxTransfer(&tx_transfer);
#endif
}

void CanardInterface::init(void* mem_arena, size_t mem_arena_size, uint8_t node_id) {
    canardInit(&canard, mem_arena, mem_arena_size, onTransferReception, shouldAcceptTransfer, this);
    canardSetLocalNodeID(&canard, node_id);
    initialized = true;
}

bool CanardInterface::broadcast(const Canard::Transfer &bcast_transfer) {
    if (!initialized) {
        return false;
    }
    WITH_SEMAPHORE(_sem_tx);

#if AP_TEST_DRONECAN_DRIVERS
    if (this == &test_iface) {
        test_iface_sem.take_blocking();
    }
#endif

    tx_transfer = {
        .transfer_type = bcast_transfer.transfer_type,
        .data_type_signature = bcast_transfer.data_type_signature,
        .data_type_id = bcast_transfer.data_type_id,
        .inout_transfer_id = bcast_transfer.inout_transfer_id,
        .priority = bcast_transfer.priority,
        .payload = (const uint8_t*)bcast_transfer.payload,
        .payload_len = uint16_t(bcast_transfer.payload_len),
#if CANARD_ENABLE_CANFD
        .canfd = bcast_transfer.canfd,
#endif
        .deadline_usec = AP_HAL::micros64() + (bcast_transfer.timeout_ms * 1000),
#if CANARD_MULTI_IFACE
        .iface_mask = uint8_t((1<<num_ifaces) - 1),
#endif
    };
    // do canard broadcast
    int16_t ret = canardBroadcastObj(&canard, &tx_transfer);
#if AP_TEST_DRONECAN_DRIVERS
    if (this == &test_iface) {
        test_iface_sem.give();
    }
#endif
    if (ret <= 0) {
        protocol_stats.tx_errors++;
    } else {
        protocol_stats.tx_frames += ret;
    }
    return ret > 0;
}

bool CanardInterface::request(uint8_t destination_node_id, const Canard::Transfer &req_transfer) {
    if (!initialized) {
        return false;
    }
    WITH_SEMAPHORE(_sem_tx);

    tx_transfer = {
        .transfer_type = req_transfer.transfer_type,
        .data_type_signature = req_transfer.data_type_signature,
        .data_type_id = req_transfer.data_type_id,
        .inout_transfer_id = req_transfer.inout_transfer_id,
        .priority = req_transfer.priority,
        .payload = (const uint8_t*)req_transfer.payload,
        .payload_len = uint16_t(req_transfer.payload_len),
#if CANARD_ENABLE_CANFD
        .canfd = req_transfer.canfd,
#endif
        .deadline_usec = AP_HAL::micros64() + (req_transfer.timeout_ms * 1000),
#if CANARD_MULTI_IFACE
        .iface_mask = uint8_t((1<<num_ifaces) - 1),
#endif
    };
    // do canard request
    int16_t ret = canardRequestOrRespondObj(&canard, destination_node_id, &tx_transfer);
    if (ret <= 0) {
        protocol_stats.tx_errors++;
    } else {
        protocol_stats.tx_frames += ret;
    }
    return ret > 0;
}

bool CanardInterface::respond(uint8_t destination_node_id, const Canard::Transfer &res_transfer) {
    if (!initialized) {
        return false;
    }
    WITH_SEMAPHORE(_sem_tx);

    tx_transfer = {
        .transfer_type = res_transfer.transfer_type,
        .data_type_signature = res_transfer.data_type_signature,
        .data_type_id = res_transfer.data_type_id,
        .inout_transfer_id = res_transfer.inout_transfer_id,
        .priority = res_transfer.priority,
        .payload = (const uint8_t*)res_transfer.payload,
        .payload_len = uint16_t(res_transfer.payload_len),
#if CANARD_ENABLE_CANFD
        .canfd = res_transfer.canfd,
#endif
        .deadline_usec = AP_HAL::micros64() + (res_transfer.timeout_ms * 1000),
#if CANARD_MULTI_IFACE
        .iface_mask = uint8_t((1<<num_ifaces) - 1),
#endif
    };
    // do canard respond
    int16_t ret = canardRequestOrRespondObj(&canard, destination_node_id, &tx_transfer);
    if (ret <= 0) {
        protocol_stats.tx_errors++;
    } else {
        protocol_stats.tx_frames += ret;
    }
    return ret > 0;
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
    CanardInterface* iface = (CanardInterface*) ins->user_reference;
    return iface->accept_message(data_type_id, *out_data_type_signature);
}

#if AP_TEST_DRONECAN_DRIVERS
void CanardInterface::processTestRx() {
    if (!test_iface.initialized) {
        return;
    }
    WITH_SEMAPHORE(test_iface_sem);
    for (const CanardCANFrame* txf = canardPeekTxQueue(&test_iface.canard); txf != NULL; txf = canardPeekTxQueue(&test_iface.canard)) {
        if (canard_ifaces[0]) {
            canardHandleRxFrame(&canard_ifaces[0]->canard, txf, AP_HAL::micros64());   
        }
        canardPopTxQueue(&test_iface.canard);
    }
}
#endif

void CanardInterface::processTx(bool raw_commands_only = false) {
    WITH_SEMAPHORE(_sem_tx);

    for (uint8_t iface = 0; iface < num_ifaces; iface++) {
        if (ifaces[iface] == NULL) {
            continue;
        }
        auto txq = canard.tx_queue;
        if (txq == nullptr) {
            return;
        }
        // scan through list of pending transfers
        while (true) {
            auto txf = &txq->frame;
            if (raw_commands_only &&
                CANARD_MSG_TYPE_FROM_ID(txf->id) != UAVCAN_EQUIPMENT_ESC_RAWCOMMAND_ID &&
                CANARD_MSG_TYPE_FROM_ID(txf->id) != COM_HOBBYWING_ESC_RAWCOMMAND_ID) {
                // look at next transfer
                txq = txq->next;
                if (txq == nullptr) {
                    break;
                }
                continue;
            }
            AP_HAL::CANFrame txmsg {};
            txmsg.dlc = AP_HAL::CANFrame::dataLengthToDlc(txf->data_len);
            memcpy(txmsg.data, txf->data, txf->data_len);
            txmsg.id = (txf->id | AP_HAL::CANFrame::FlagEFF);
#if HAL_CANFD_SUPPORTED
            txmsg.canfd = txf->canfd;
#endif
            bool write = true;
            bool read = false;
            ifaces[iface]->select(read, write, &txmsg, 0);
            if (!write) {
                // if there is no space then we need to start from the
                // top of the queue, so wait for the next loop
                break;
            }
            if ((txf->iface_mask & (1U<<iface)) && (AP_HAL::micros64() < txf->deadline_usec)) {
                // try sending to interfaces, clearing the mask if we succeed
                if (ifaces[iface]->send(txmsg, txf->deadline_usec, 0) > 0) {
                    txf->iface_mask &= ~(1U<<iface);
                } else {
                    // if we fail to send then we try sending on next interface
                    break;
                }
            }
            // look at next transfer
            txq = txq->next;
            if (txq == nullptr) {
                break;
            }
        }
    }

}

void CanardInterface::update_rx_protocol_stats(int16_t res)
{
    switch (res) {
    case CANARD_OK:
        protocol_stats.rx_frames++;
        break;
    case -CANARD_ERROR_OUT_OF_MEMORY:
        protocol_stats.rx_error_oom++;
        break;
    case -CANARD_ERROR_INTERNAL:
        protocol_stats.rx_error_internal++;
        break;
    case -CANARD_ERROR_RX_INCOMPATIBLE_PACKET:
        protocol_stats.rx_ignored_not_wanted++;
        break;
    case -CANARD_ERROR_RX_WRONG_ADDRESS:
        protocol_stats.rx_ignored_wrong_address++;
        break;
    case -CANARD_ERROR_RX_NOT_WANTED:
        protocol_stats.rx_ignored_not_wanted++;
        break;
    case -CANARD_ERROR_RX_MISSED_START:
        protocol_stats.rx_error_missed_start++;
        break;
    case -CANARD_ERROR_RX_WRONG_TOGGLE:
        protocol_stats.rx_error_wrong_toggle++;
        break;
    case -CANARD_ERROR_RX_UNEXPECTED_TID:
        protocol_stats.rx_ignored_unexpected_tid++;
        break;
    case -CANARD_ERROR_RX_SHORT_FRAME:
        protocol_stats.rx_error_short_frame++;
        break;
    case -CANARD_ERROR_RX_BAD_CRC:
        protocol_stats.rx_error_bad_crc++;
        break;
    default:
        // mark all other errors as internal
        protocol_stats.rx_error_internal++;
        break;
    }
}

void CanardInterface::processRx() {
    AP_HAL::CANFrame rxmsg;
    for (uint8_t i=0; i<num_ifaces; i++) {
        while(true) {
            if (ifaces[i] == NULL) {
                break;
            }
            bool read_select = true;
            bool write_select = false;
            ifaces[i]->select(read_select, write_select, nullptr, 0);
            if (!read_select) { // No data pending
                break;
            }
            CanardCANFrame rx_frame {};

            //palToggleLine(HAL_GPIO_PIN_LED);
            uint64_t timestamp;
            AP_HAL::CANIface::CanIOFlags flags;
            if (ifaces[i]->receive(rxmsg, timestamp, flags) <= 0) {
                break;
            }
            rx_frame.data_len = AP_HAL::CANFrame::dlcToDataLength(rxmsg.dlc);
            memcpy(rx_frame.data, rxmsg.data, rx_frame.data_len);
#if HAL_CANFD_SUPPORTED
            rx_frame.canfd = rxmsg.canfd;
#endif
            rx_frame.id = rxmsg.id;
#if CANARD_MULTI_IFACE
            rx_frame.iface_id = i;
#endif
            {
                WITH_SEMAPHORE(_sem_rx);

                const int16_t res = canardHandleRxFrame(&canard, &rx_frame, timestamp);
                if (res == -CANARD_ERROR_RX_MISSED_START) {
                    // this might remaining frames from a message that we don't accept, so check
                    uint64_t dummy_signature;
                    if (shouldAcceptTransfer(&canard,
                                        &dummy_signature,
                                        extractDataType(rx_frame.id),
                                        extractTransferType(rx_frame.id),
                                        1)) { // doesn't matter what we pass here
                        update_rx_protocol_stats(res);
                    } else {
                        protocol_stats.rx_ignored_not_wanted++;
                    }
                } else {
                    update_rx_protocol_stats(res);
                }
            }
        }
    }
}

void CanardInterface::process(uint32_t duration_ms) {
#if AP_TEST_DRONECAN_DRIVERS
    const uint64_t deadline = AP_HAL::micros64() + duration_ms*1000;
    while (AP_HAL::micros64() < deadline) {
        processTestRx();
        hal.scheduler->delay_microseconds(1000);
    }
#else
    const uint64_t deadline = AP_HAL::micros64() + duration_ms*1000;
    while (true) {
        processRx();
        processTx();
        {
            WITH_SEMAPHORE(_sem_rx);
            WITH_SEMAPHORE(_sem_tx);
            canardCleanupStaleTransfers(&canard, AP_HAL::micros64());
        }
        uint64_t now = AP_HAL::micros64();
        if (now < deadline) {
            _event_handle.wait(MIN(UINT16_MAX - 2U, deadline - now));
            hal.scheduler->delay_microseconds(50);
        } else {
            break;
        }
    }
#endif
}

bool CanardInterface::add_interface(AP_HAL::CANIface *can_iface)
{
    if (num_ifaces > HAL_NUM_CAN_IFACES) {
        AP::can().log_text(AP_CANManager::LOG_ERROR, LOG_TAG, "DroneCANIfaceMgr: Num Ifaces Exceeded\n");
        return false;
    }
    if (can_iface == nullptr) {
        AP::can().log_text(AP_CANManager::LOG_ERROR, LOG_TAG, "DroneCANIfaceMgr: Iface Null\n");
        return false;
    }
    if (ifaces[num_ifaces] != nullptr) {
        AP::can().log_text(AP_CANManager::LOG_ERROR, LOG_TAG, "DroneCANIfaceMgr: Iface already added\n");
        return false;
    }
    ifaces[num_ifaces] = can_iface;
    if (ifaces[num_ifaces] == nullptr) {
        AP::can().log_text(AP_CANManager::LOG_ERROR, LOG_TAG, "DroneCANIfaceMgr: Can't alloc uavcan::iface\n");
        return false;
    }
    if (!can_iface->set_event_handle(&_event_handle)) {
        AP::can().log_text(AP_CANManager::LOG_ERROR, LOG_TAG, "DroneCANIfaceMgr: Setting event handle failed\n");
        return false;
    }
    AP::can().log_text(AP_CANManager::LOG_INFO, LOG_TAG, "DroneCANIfaceMgr: Successfully added interface %d\n", int(num_ifaces));
    num_ifaces++;
    return true;
}
#endif // #if HAL_ENABLE_DRONECAN_DRIVERS
