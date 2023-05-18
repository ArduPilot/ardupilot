#include "AP_Canard_iface.h"
#include <AP_HAL/AP_HAL.h>
#include <AP_CANManager/AP_CANManager.h>
#if HAL_ENABLE_DRONECAN_DRIVERS
#include <canard/handler_list.h>
#include <canard/transfer_object.h>
#include <AP_Math/AP_Math.h>

extern const AP_HAL::HAL& hal;
#define LOG_TAG "DroneCANIface"

#define DEBUG_PKTS 0

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

CanardInterface::CanardInterface(uint8_t iface_index) :
Interface(iface_index) {
#if AP_TEST_DRONECAN_DRIVERS
    if (iface_index < 3) {
        canard_ifaces[iface_index] = this;
    }
    if (iface_index == 0) {
        test_iface.init(test_node_mem_area, sizeof(test_node_mem_area), 125);
    }
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
    WITH_SEMAPHORE(_sem);
#if AP_TEST_DRONECAN_DRIVERS
    if (this == &test_iface) {
        test_iface_sem.take_blocking();
    }
#endif

    // do canard broadcast
    bool success = canardBroadcast(&canard,
                            bcast_transfer.data_type_signature,
                            bcast_transfer.data_type_id,
                            bcast_transfer.inout_transfer_id,
                            bcast_transfer.priority,
                            bcast_transfer.payload,
                            bcast_transfer.payload_len,
                            AP_HAL::native_micros64() + (bcast_transfer.timeout_ms * 1000)
#if CANARD_MULTI_IFACE
                            , ((1<<num_ifaces) - 1) // send over all ifaces
#endif
#if HAL_CANFD_SUPPORTED
                            , bcast_transfer.canfd
#endif
            ) > 0;
#if AP_TEST_DRONECAN_DRIVERS
    if (this == &test_iface) {
        test_iface_sem.give();
    }
#endif
    return success;
}

bool CanardInterface::request(uint8_t destination_node_id, const Canard::Transfer &req_transfer) {
    if (!initialized) {
        return false;
    }
    WITH_SEMAPHORE(_sem);
    // do canard request
    return canardRequestOrRespond(&canard,
                                    destination_node_id,
                                    req_transfer.data_type_signature,
                                    req_transfer.data_type_id,
                                    req_transfer.inout_transfer_id,
                                    req_transfer.priority,
                                    CanardRequest,
                                    req_transfer.payload,
                                    req_transfer.payload_len,
                                    AP_HAL::native_micros64() + (req_transfer.timeout_ms * 1000)
#if CANARD_MULTI_IFACE
                    , ((1<<num_ifaces) - 1) // send over all ifaces
#endif
#if HAL_CANFD_SUPPORTED
                    , false
#endif
            ) > 0;
}

bool CanardInterface::respond(uint8_t destination_node_id, const Canard::Transfer &res_transfer) {
    if (!initialized) {
        return false;
    }
    WITH_SEMAPHORE(_sem);
    // do canard respond
    return canardRequestOrRespond(&canard,
                                    destination_node_id,
                                    res_transfer.data_type_signature,
                                    res_transfer.data_type_id,
                                    res_transfer.inout_transfer_id,
                                    res_transfer.priority,
                                    CanardResponse,
                                    res_transfer.payload,
                                    res_transfer.payload_len,
                                    AP_HAL::native_micros64() + (res_transfer.timeout_ms * 1000)
#if CANARD_MULTI_IFACE
                    , ((1<<num_ifaces) - 1) // send over all ifaces
#endif
#if HAL_CANFD_SUPPORTED
                    , false
#endif
            ) > 0;
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
            canardHandleRxFrame(&canard_ifaces[0]->canard, txf, AP_HAL::native_micros64());   
        }
        canardPopTxQueue(&test_iface.canard);
    }
}
#endif

void CanardInterface::processTx() {
    WITH_SEMAPHORE(_sem);

    for (uint8_t iface = 0; iface < num_ifaces; iface++) {
        if (ifaces[iface] == NULL) {
            continue;
        }
        auto txq = canard.tx_queue;
        if (txq == nullptr) {
            return;
        }
        AP_HAL::CANFrame txmsg {};
        // scan through list of pending transfers
        while (true) {
            auto txf = &txq->frame;
            txmsg.dlc = AP_HAL::CANFrame::dataLengthToDlc(txf->data_len);
            memcpy(txmsg.data, txf->data, txf->data_len);
            txmsg.id = (txf->id | AP_HAL::CANFrame::FlagEFF);
#if HAL_CANFD_SUPPORTED
            txmsg.canfd = txf->canfd;
#endif
            bool write = true;
            bool read = false;
            ifaces[iface]->select(read, write, &txmsg, 0);
            if ((AP_HAL::native_micros64() < txf->deadline_usec) && (txf->iface_mask & (1U<<iface)) && write) {
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

    // purge expired transfers
    for (const CanardCANFrame* txf = canardPeekTxQueue(&canard); txf != NULL; txf = canardPeekTxQueue(&canard)) {
        if ((AP_HAL::native_micros64() >= txf->deadline_usec) || (txf->iface_mask == 0)) {
            canardPopTxQueue(&canard);
        } else {
            break;
        }
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
                WITH_SEMAPHORE(_sem);

#if DEBUG_PKTS
                const int16_t res = 
#endif
                canardHandleRxFrame(&canard, &rx_frame, timestamp);
#if DEBUG_PKTS
                // hal.console->printf("DTID: %u\n", extractDataType(rx_frame.id));
                // hal.console->printf("Rx %d, IF%d %lx: ", res, i, rx_frame.id);
                if (res < 0 &&
                    res != -CANARD_ERROR_RX_NOT_WANTED &&
                    res != -CANARD_ERROR_RX_WRONG_ADDRESS) {
                    hal.console->printf("Rx error %d, IF%d %lx: \n", res, i, rx_frame.id);
                    // for (uint8_t index = 0; index < rx_frame.data_len; index++) {
                    //     hal.console->printf("%02x", rx_frame.data[index]);
                    // }
                    // hal.console->printf("\n");
                }
#endif
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
    const uint64_t deadline = AP_HAL::native_micros64() + duration_ms*1000;
    while (true) {
        processRx();
        processTx();
        uint64_t now = AP_HAL::native_micros64();
        if (now < deadline) {
            _event_handle.wait(MIN(UINT16_MAX - 2U, deadline - now));
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
