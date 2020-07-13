/*
   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */
/*
  CAN bootloader support
 */
#include <AP_HAL/AP_HAL.h>

#if HAL_USE_CAN == TRUE
#include <AP_Math/AP_Math.h>
#include <AP_Math/crc.h>
#include <canard.h>
#include "support.h"
#include <uavcan/protocol/dynamic_node_id/Allocation.h>
#include <uavcan/protocol/file/BeginFirmwareUpdate.h>
#include <uavcan/protocol/file/Read.h>
#include <uavcan/protocol/dynamic_node_id/Allocation.h>
#include <uavcan/protocol/NodeStatus.h>
#include <uavcan/protocol/RestartNode.h>
#include <uavcan/protocol/GetNodeInfo.h>
#include "can.h"
#include "bl_protocol.h"
#include <drivers/stm32/canard_stm32.h>
#include "app_comms.h"
#include <AP_HAL_ChibiOS/hwdef/common/watchdog.h>
#include <stdio.h>


static CanardInstance canard;
static uint32_t canard_memory_pool[4096/4];
#ifndef HAL_CAN_DEFAULT_NODE_ID
#define HAL_CAN_DEFAULT_NODE_ID CANARD_BROADCAST_NODE_ID
#endif
static uint8_t initial_node_id = HAL_CAN_DEFAULT_NODE_ID;

// can config for 1MBit
static uint32_t baudrate = 1000000U;

static CANConfig cancfg = {
    CAN_MCR_ABOM | CAN_MCR_AWUM | CAN_MCR_TXFP,
    0 // filled in below
};

#ifndef CAN_APP_VERSION_MAJOR
#define CAN_APP_VERSION_MAJOR                                           1
#endif
#ifndef CAN_APP_VERSION_MINOR
#define CAN_APP_VERSION_MINOR                                           0
#endif
#ifndef CAN_APP_NODE_NAME
#define CAN_APP_NODE_NAME                                               "org.ardupilot.ap_periph"
#endif

static uint8_t node_id_allocation_transfer_id;
static uavcan_protocol_NodeStatus node_status;
static uint32_t send_next_node_id_allocation_request_at_ms;
static uint8_t node_id_allocation_unique_id_offset;
static uint32_t app_first_word = 0xFFFFFFFF;

static struct {
    uint64_t ofs;
    uint32_t last_ms;
    uint8_t node_id;
    uint8_t transfer_id;
    uint8_t path[UAVCAN_PROTOCOL_FILE_PATH_PATH_MAX_LENGTH+1];
    uint8_t sector;
    uint32_t sector_ofs;
} fw_update;

enum {
    FAIL_REASON_NO_APP_SIG = 10,
    FAIL_REASON_BAD_LENGTH = 11,
    FAIL_REASON_BAD_BOARD_ID = 12,
    FAIL_REASON_BAD_CRC = 13,
    FAIL_REASON_IN_UPDATE = 14,
    FAIL_REASON_WATCHDOG = 15,
};

/*
  get cpu unique ID
 */
static void readUniqueID(uint8_t* out_uid)
{
    uint8_t len = UAVCAN_PROTOCOL_DYNAMIC_NODE_ID_ALLOCATION_UNIQUE_ID_MAX_LENGTH;
    memset(out_uid, 0, len);
    memcpy(out_uid, (const void *)UDID_START, MIN(len,12));
}

/*
  simple 16 bit random number generator
 */
static uint16_t get_randomu16(void)
{
    static uint32_t m_z = 1234;
    static uint32_t m_w = 76542;
    m_z = 36969 * (m_z & 0xFFFFu) + (m_z >> 16);
    m_w = 18000 * (m_w & 0xFFFFu) + (m_w >> 16);
    return ((m_z << 16) + m_w) & 0xFFFF;
}


/**
 * Returns a pseudo random integer in a given range
 */
static uint32_t get_random_range(uint16_t range)
{
    return get_randomu16() % range;
}

/*
  handle a GET_NODE_INFO request
 */
static void handle_get_node_info(CanardInstance* ins,
                                 CanardRxTransfer* transfer)
{
    uint8_t buffer[UAVCAN_PROTOCOL_GETNODEINFO_RESPONSE_MAX_SIZE] {};
    uavcan_protocol_GetNodeInfoResponse pkt {};

    node_status.uptime_sec = AP_HAL::millis() / 1000U;

    pkt.status = node_status;
    pkt.software_version.major = CAN_APP_VERSION_MAJOR;
    pkt.software_version.minor = CAN_APP_VERSION_MINOR;

    readUniqueID(pkt.hardware_version.unique_id);

    // use hw major/minor for APJ_BOARD_ID so we know what fw is
    // compatible with this hardware
    pkt.hardware_version.major = APJ_BOARD_ID >> 8;
    pkt.hardware_version.minor = APJ_BOARD_ID & 0xFF;

    char name[strlen(CAN_APP_NODE_NAME)+1];
    strcpy(name, CAN_APP_NODE_NAME);
    pkt.name.len = strlen(CAN_APP_NODE_NAME);
    pkt.name.data = (uint8_t *)name;

    uint16_t total_size = uavcan_protocol_GetNodeInfoResponse_encode(&pkt, buffer);

    canardRequestOrRespond(ins,
                           transfer->source_node_id,
                           UAVCAN_PROTOCOL_GETNODEINFO_SIGNATURE,
                           UAVCAN_PROTOCOL_GETNODEINFO_ID,
                           &transfer->transfer_id,
                           transfer->priority,
                           CanardResponse,
                           &buffer[0],
                           total_size);
}

/*
  send a read for a fw update
 */
static void send_fw_read(void)
{
    uint32_t now = AP_HAL::millis();
    if (now - fw_update.last_ms < 250) {
        // the server may still be responding
        return;
    }
    fw_update.last_ms = now;

    uint8_t buffer[UAVCAN_PROTOCOL_FILE_READ_REQUEST_MAX_SIZE];
    canardEncodeScalar(buffer, 0, 40, &fw_update.ofs);
    uint32_t offset = 40;
    uint8_t len = strlen((const char *)fw_update.path);
    for (uint8_t i=0; i<len; i++) {
        canardEncodeScalar(buffer, offset, 8, &fw_update.path[i]);
        offset += 8;
    }
    uint32_t total_size = (offset+7)/8;
    canardRequestOrRespond(&canard,
                           fw_update.node_id,
                           UAVCAN_PROTOCOL_FILE_READ_SIGNATURE,
                           UAVCAN_PROTOCOL_FILE_READ_ID,
                           &fw_update.transfer_id,
                           CANARD_TRANSFER_PRIORITY_HIGH,
                           CanardRequest,
                           &buffer[0],
                           total_size);
}

/*
  handle response to file read for fw update
 */
static void handle_file_read_response(CanardInstance* ins, CanardRxTransfer* transfer)
{
    if ((transfer->transfer_id+1)%256 != fw_update.transfer_id ||
        transfer->source_node_id != fw_update.node_id) {
        return;
    }
    int16_t error = 0;
    canardDecodeScalar(transfer, 0, 16, true, (void*)&error);
    uint16_t len = transfer->payload_len - 2;

    uint32_t offset = 16;
    uint32_t buf32[(len+3)/4];
    uint8_t *buf = (uint8_t *)&buf32[0];
    for (uint16_t i=0; i<len; i++) {
        canardDecodeScalar(transfer, offset, 8, false, (void*)&buf[i]);
        offset += 8;
    }

    const uint32_t sector_size = flash_func_sector_size(fw_update.sector);

    if (fw_update.sector_ofs == 0) {
        flash_func_erase_sector(fw_update.sector);
    }
    if (fw_update.sector_ofs+len > sector_size) {
        flash_func_erase_sector(fw_update.sector+1);
    }
    for (uint16_t i=0; i<len/4; i++) {
        if (i == 0 && fw_update.sector == 0 && fw_update.ofs == 0) {
            // keep first word aside, to be flashed last
            app_first_word = buf32[0];
        } else {
            flash_func_write_word(fw_update.ofs+i*4, buf32[i]);
        }
    }
    fw_update.ofs += len;
    fw_update.sector_ofs += len;
    if (fw_update.sector_ofs >= flash_func_sector_size(fw_update.sector)) {
        fw_update.sector++;
        fw_update.sector_ofs -= sector_size;
    }
    if (len < UAVCAN_PROTOCOL_FILE_READ_RESPONSE_DATA_MAX_LENGTH) {
        fw_update.node_id = 0;
        // now flash the first word
        flash_func_write_word(0, app_first_word);
        if (can_check_firmware()) {
            jump_to_app();
        }
    }

    // show offset number we are flashing in kbyte as crude progress indicator
    node_status.vendor_specific_status_code = 1 + (fw_update.ofs / 1024U);

    fw_update.last_ms = 0;
}

/*
  handle a begin firmware update request. We start pulling in the file data
 */
static void handle_begin_firmware_update(CanardInstance* ins, CanardRxTransfer* transfer)
{
    // manual decoding due to TAO bug in libcanard generated code
    if (transfer->payload_len < 1 || transfer->payload_len > sizeof(fw_update.path)+1) {
        return;
    }
    if (fw_update.node_id == 0) {
        uint32_t offset = 0;
        canardDecodeScalar(transfer, 0, 8, false, (void*)&fw_update.node_id);
        offset += 8;
        for (uint8_t i=0; i<transfer->payload_len-1; i++) {
            canardDecodeScalar(transfer, offset, 8, false, (void*)&fw_update.path[i]);
            offset += 8;
        }
        fw_update.ofs = 0;
        fw_update.last_ms = 0;
        fw_update.sector = 0;
        fw_update.sector_ofs = 0;
        if (fw_update.node_id == 0) {
            fw_update.node_id = transfer->source_node_id;
        }
    }

    uint8_t buffer[UAVCAN_PROTOCOL_FILE_BEGINFIRMWAREUPDATE_RESPONSE_MAX_SIZE];
    uavcan_protocol_file_BeginFirmwareUpdateResponse reply {};
    reply.error = UAVCAN_PROTOCOL_FILE_BEGINFIRMWAREUPDATE_RESPONSE_ERROR_OK;

    uint32_t total_size = uavcan_protocol_file_BeginFirmwareUpdateResponse_encode(&reply, buffer);
    canardRequestOrRespond(ins,
                           transfer->source_node_id,
                           UAVCAN_PROTOCOL_FILE_BEGINFIRMWAREUPDATE_SIGNATURE,
                           UAVCAN_PROTOCOL_FILE_BEGINFIRMWAREUPDATE_ID,
                           &transfer->transfer_id,
                           transfer->priority,
                           CanardResponse,
                           &buffer[0],
                           total_size);

    send_fw_read();
}

static void handle_allocation_response(CanardInstance* ins, CanardRxTransfer* transfer)
{
    // Rule C - updating the randomized time interval
    send_next_node_id_allocation_request_at_ms =
        AP_HAL::millis() + UAVCAN_PROTOCOL_DYNAMIC_NODE_ID_ALLOCATION_MIN_REQUEST_PERIOD_MS +
        get_random_range(UAVCAN_PROTOCOL_DYNAMIC_NODE_ID_ALLOCATION_MAX_FOLLOWUP_DELAY_MS);

    if (transfer->source_node_id == CANARD_BROADCAST_NODE_ID)
    {
        node_id_allocation_unique_id_offset = 0;
        return;
    }

    // Copying the unique ID from the message
    static const uint8_t UniqueIDBitOffset = 8;
    uint8_t received_unique_id[UAVCAN_PROTOCOL_DYNAMIC_NODE_ID_ALLOCATION_UNIQUE_ID_MAX_LENGTH];
    uint8_t received_unique_id_len = 0;
    for (; received_unique_id_len < (transfer->payload_len - (UniqueIDBitOffset / 8U)); received_unique_id_len++) {
        assert(received_unique_id_len < UAVCAN_PROTOCOL_DYNAMIC_NODE_ID_ALLOCATION_UNIQUE_ID_MAX_LENGTH);
        const uint8_t bit_offset = (uint8_t)(UniqueIDBitOffset + received_unique_id_len * 8U);
        (void) canardDecodeScalar(transfer, bit_offset, 8, false, &received_unique_id[received_unique_id_len]);
    }

    // Obtaining the local unique ID
    uint8_t my_unique_id[UAVCAN_PROTOCOL_DYNAMIC_NODE_ID_ALLOCATION_UNIQUE_ID_MAX_LENGTH];
    readUniqueID(my_unique_id);

    // Matching the received UID against the local one
    if (memcmp(received_unique_id, my_unique_id, received_unique_id_len) != 0) {
        node_id_allocation_unique_id_offset = 0;
        return;         // No match, return
    }

    if (received_unique_id_len < UAVCAN_PROTOCOL_DYNAMIC_NODE_ID_ALLOCATION_UNIQUE_ID_MAX_LENGTH) {
        // The allocator has confirmed part of unique ID, switching to the next stage and updating the timeout.
        node_id_allocation_unique_id_offset = received_unique_id_len;
        send_next_node_id_allocation_request_at_ms -= UAVCAN_PROTOCOL_DYNAMIC_NODE_ID_ALLOCATION_MIN_REQUEST_PERIOD_MS;
    } else {
        // Allocation complete - copying the allocated node ID from the message
        uint8_t allocated_node_id = 0;
        (void) canardDecodeScalar(transfer, 0, 7, false, &allocated_node_id);
        assert(allocated_node_id <= 127);

        canardSetLocalNodeID(ins, allocated_node_id);
    }
}

/**
 * This callback is invoked by the library when a new message or request or response is received.
 */
static void onTransferReceived(CanardInstance* ins,
                               CanardRxTransfer* transfer)
{
    /*
     * Dynamic node ID allocation protocol.
     * Taking this branch only if we don't have a node ID, ignoring otherwise.
     */
    if (canardGetLocalNodeID(ins) == CANARD_BROADCAST_NODE_ID) {
        if (transfer->transfer_type == CanardTransferTypeBroadcast &&
            transfer->data_type_id == UAVCAN_PROTOCOL_DYNAMIC_NODE_ID_ALLOCATION_ID) {
            handle_allocation_response(ins, transfer);
        }
        return;
    }

    switch (transfer->data_type_id) {
    case UAVCAN_PROTOCOL_GETNODEINFO_ID:
        handle_get_node_info(ins, transfer);
        break;

    case UAVCAN_PROTOCOL_FILE_BEGINFIRMWAREUPDATE_ID:
        handle_begin_firmware_update(ins, transfer);
        break;

    case UAVCAN_PROTOCOL_FILE_READ_ID:
        handle_file_read_response(ins, transfer);
        break;

    case UAVCAN_PROTOCOL_RESTARTNODE_ID:
        NVIC_SystemReset();
        break;
    }
}


/**
 * This callback is invoked by the library when it detects beginning of a new transfer on the bus that can be received
 * by the local node.
 * If the callback returns true, the library will receive the transfer.
 * If the callback returns false, the library will ignore the transfer.
 * All transfers that are addressed to other nodes are always ignored.
 */
static bool shouldAcceptTransfer(const CanardInstance* ins,
                                 uint64_t* out_data_type_signature,
                                 uint16_t data_type_id,
                                 CanardTransferType transfer_type,
                                 uint8_t source_node_id)
{
    (void)source_node_id;

    if (canardGetLocalNodeID(ins) == CANARD_BROADCAST_NODE_ID) {
        /*
         * If we're in the process of allocation of dynamic node ID, accept only relevant transfers.
         */
        if ((transfer_type == CanardTransferTypeBroadcast) &&
            (data_type_id == UAVCAN_PROTOCOL_DYNAMIC_NODE_ID_ALLOCATION_ID))
        {
            *out_data_type_signature = UAVCAN_PROTOCOL_DYNAMIC_NODE_ID_ALLOCATION_SIGNATURE;
            return true;
        }
        return false;
    }

    switch (data_type_id) {
    case UAVCAN_PROTOCOL_GETNODEINFO_ID:
        *out_data_type_signature = UAVCAN_PROTOCOL_GETNODEINFO_SIGNATURE;
        return true;
    case UAVCAN_PROTOCOL_FILE_BEGINFIRMWAREUPDATE_ID:
        *out_data_type_signature = UAVCAN_PROTOCOL_FILE_BEGINFIRMWAREUPDATE_SIGNATURE;
        return true;
    case UAVCAN_PROTOCOL_RESTARTNODE_ID:
        *out_data_type_signature = UAVCAN_PROTOCOL_RESTARTNODE_SIGNATURE;
        return true;
    case UAVCAN_PROTOCOL_FILE_READ_ID:
        *out_data_type_signature = UAVCAN_PROTOCOL_FILE_READ_SIGNATURE;
        return true;
    default:
        break;
    }

    return false;
}

static void processTx(void)
{
    static uint8_t fail_count;
    for (const CanardCANFrame* txf = NULL; (txf = canardPeekTxQueue(&canard)) != NULL;) {
        CANTxFrame txmsg {};
        txmsg.DLC = txf->data_len;
        memcpy(txmsg.data8, txf->data, 8);
        txmsg.EID = txf->id & CANARD_CAN_EXT_ID_MASK;
        txmsg.IDE = 1;
        txmsg.RTR = 0;
        if (canTransmit(&CAND1, CAN_ANY_MAILBOX, &txmsg, TIME_IMMEDIATE) == MSG_OK) {
            canardPopTxQueue(&canard);
            fail_count = 0;
        } else {
            // just exit and try again later. If we fail 8 times in a row
            // then start discarding to prevent the pool filling up
            if (fail_count < 8) {
                fail_count++;
            } else {
                canardPopTxQueue(&canard);
            }
            return;
        }
    }
}

static void processRx(void)
{
    CANRxFrame rxmsg {};
    while (canReceive(&CAND1, CAN_ANY_MAILBOX, &rxmsg, TIME_IMMEDIATE) == MSG_OK) {
        CanardCANFrame rx_frame {};

        palToggleLine(HAL_GPIO_PIN_LED_BOOTLOADER);

        const uint64_t timestamp = AP_HAL::micros64();
        memcpy(rx_frame.data, rxmsg.data8, 8);
        rx_frame.data_len = rxmsg.DLC;
        if(rxmsg.IDE) {
            rx_frame.id = CANARD_CAN_FRAME_EFF | rxmsg.EID;
        } else {
            rx_frame.id = rxmsg.SID;
        }
        canardHandleRxFrame(&canard, &rx_frame, timestamp);
    }
}

/*
  handle waiting for a node ID
 */
static void can_handle_DNA(void)
{
    if (canardGetLocalNodeID(&canard) != CANARD_BROADCAST_NODE_ID) {
        return;
    }

    if (AP_HAL::millis() < send_next_node_id_allocation_request_at_ms) {
        return;
    }

    send_next_node_id_allocation_request_at_ms =
        AP_HAL::millis() + UAVCAN_PROTOCOL_DYNAMIC_NODE_ID_ALLOCATION_MIN_REQUEST_PERIOD_MS +
        get_random_range(UAVCAN_PROTOCOL_DYNAMIC_NODE_ID_ALLOCATION_MAX_FOLLOWUP_DELAY_MS);
    
    // Structure of the request is documented in the DSDL definition
    // See http://uavcan.org/Specification/6._Application_level_functions/#dynamic-node-id-allocation
    uint8_t allocation_request[CANARD_CAN_FRAME_MAX_DATA_LEN - 1];
    allocation_request[0] = (uint8_t)(CANARD_BROADCAST_NODE_ID << 1U);

    if (node_id_allocation_unique_id_offset == 0) {
        allocation_request[0] |= 1;     // First part of unique ID
    }

    uint8_t my_unique_id[UAVCAN_PROTOCOL_DYNAMIC_NODE_ID_ALLOCATION_UNIQUE_ID_MAX_LENGTH];
    readUniqueID(my_unique_id);

    static const uint8_t MaxLenOfUniqueIDInRequest = 6;
    uint8_t uid_size = (uint8_t)(UAVCAN_PROTOCOL_DYNAMIC_NODE_ID_ALLOCATION_UNIQUE_ID_MAX_LENGTH - node_id_allocation_unique_id_offset);
    if (uid_size > MaxLenOfUniqueIDInRequest) {
        uid_size = MaxLenOfUniqueIDInRequest;
    }

    memmove(&allocation_request[1], &my_unique_id[node_id_allocation_unique_id_offset], uid_size);

    // Broadcasting the request
    canardBroadcast(&canard,
                    UAVCAN_PROTOCOL_DYNAMIC_NODE_ID_ALLOCATION_SIGNATURE,
                    UAVCAN_PROTOCOL_DYNAMIC_NODE_ID_ALLOCATION_ID,
                    &node_id_allocation_transfer_id,
                    CANARD_TRANSFER_PRIORITY_LOW,
                    &allocation_request[0],
                    (uint16_t) (uid_size + 1));

    // Preparing for timeout; if response is received, this value will be updated from the callback.
    node_id_allocation_unique_id_offset = 0;
}

static void send_node_status(void)
{
    uint8_t buffer[UAVCAN_PROTOCOL_NODESTATUS_MAX_SIZE];
    node_status.uptime_sec = AP_HAL::millis() / 1000U;

    uint32_t len = uavcan_protocol_NodeStatus_encode(&node_status, buffer);

    static uint8_t transfer_id;  // Note that the transfer ID variable MUST BE STATIC (or heap-allocated)!

    canardBroadcast(&canard,
                    UAVCAN_PROTOCOL_NODESTATUS_SIGNATURE,
                    UAVCAN_PROTOCOL_NODESTATUS_ID,
                    &transfer_id,
                    CANARD_TRANSFER_PRIORITY_LOW,
                    buffer,
                    len);
}


/**
 * This function is called at 1 Hz rate from the main loop.
 */
static void process1HzTasks(uint64_t timestamp_usec)
{
    canardCleanupStaleTransfers(&canard, timestamp_usec);

    if (canardGetLocalNodeID(&canard) != CANARD_BROADCAST_NODE_ID) {
        node_status.mode = fw_update.node_id?UAVCAN_PROTOCOL_NODESTATUS_MODE_SOFTWARE_UPDATE:UAVCAN_PROTOCOL_NODESTATUS_MODE_MAINTENANCE;
        send_node_status();
    }
}

void can_set_node_id(uint8_t node_id)
{
    initial_node_id = node_id;
}

/*
  check firmware CRC to see if it matches
 */
bool can_check_firmware(void)
{
    if (fw_update.node_id != 0) {
        // we're doing an update, don't boot this fw
        node_status.vendor_specific_status_code = FAIL_REASON_IN_UPDATE;
        return false;
    }
    const uint8_t sig[8] = { 0x40, 0xa2, 0xe4, 0xf1, 0x64, 0x68, 0x91, 0x06 };
    const uint8_t *flash = (const uint8_t *)(FLASH_LOAD_ADDRESS + FLASH_BOOTLOADER_LOAD_KB*1024);
    const uint32_t flash_size = (BOARD_FLASH_SIZE - FLASH_BOOTLOADER_LOAD_KB)*1024;
    const app_descriptor *ad = (const app_descriptor *)memmem(flash, flash_size-sizeof(app_descriptor), sig, sizeof(sig));
    if (ad == nullptr) {
        // no application signature
        node_status.vendor_specific_status_code = FAIL_REASON_NO_APP_SIG;
        printf("No app sig\n");
        return false;
    }
    // check length
    if (ad->image_size > flash_size) {
        node_status.vendor_specific_status_code = FAIL_REASON_BAD_LENGTH;
        printf("Bad fw length %u\n", ad->image_size);
        return false;
    }

    if (ad->board_id != APJ_BOARD_ID) {
        node_status.vendor_specific_status_code = FAIL_REASON_BAD_BOARD_ID;
        printf("Bad board_id %u should be %u\n", ad->board_id, APJ_BOARD_ID);
        return false;
    }

    const uint8_t desc_len = offsetof(app_descriptor, version_major) - offsetof(app_descriptor, image_crc1);
    uint32_t len1 = ((const uint8_t *)&ad->image_crc1) - flash;
    uint32_t len2 = ad->image_size - (len1 + desc_len);
    uint32_t crc1 = crc32_small(0, flash, len1);
    uint32_t crc2 = crc32_small(0, (const uint8_t *)&ad->version_major, len2);
    if (crc1 != ad->image_crc1 || crc2 != ad->image_crc2) {
        node_status.vendor_specific_status_code = FAIL_REASON_BAD_CRC;
        printf("Bad app CRC 0x%08x:0x%08x 0x%08x:0x%08x\n", ad->image_crc1, ad->image_crc2, crc1, crc2);
        return false;
    }
    printf("Good firmware\n");
    return true;
}

// check for a firmware update marker left by app
void can_check_update(void)
{
#if HAL_RAM_RESERVE_START >= 256
    struct app_bootloader_comms *comms = (struct app_bootloader_comms *)HAL_RAM0_START;
    if (comms->magic == APP_BOOTLOADER_COMMS_MAGIC) {
        can_set_node_id(comms->my_node_id);
        fw_update.node_id = comms->server_node_id;
        memcpy(fw_update.path, comms->path, UAVCAN_PROTOCOL_FILE_PATH_PATH_MAX_LENGTH+1);
    }
    // clear comms region
    memset(comms, 0, sizeof(struct app_bootloader_comms));
#endif
}

void can_start()
{
    node_status.mode = UAVCAN_PROTOCOL_NODESTATUS_MODE_MAINTENANCE;

    // calculate optimal CAN timings given PCLK1 and baudrate
    CanardSTM32CANTimings timings {};
    canardSTM32ComputeCANTimings(STM32_PCLK1, baudrate, &timings);
    cancfg.btr = CAN_BTR_SJW(0) |
        CAN_BTR_TS2(timings.bit_segment_2-1) |
        CAN_BTR_TS1(timings.bit_segment_1-1) |
        CAN_BTR_BRP(timings.bit_rate_prescaler-1);
    canStart(&CAND1, &cancfg);

    canardInit(&canard, (uint8_t *)canard_memory_pool, sizeof(canard_memory_pool),
               onTransferReceived, shouldAcceptTransfer, NULL);

    if (initial_node_id != CANARD_BROADCAST_NODE_ID) {
        canardSetLocalNodeID(&canard, initial_node_id);
    }

    send_next_node_id_allocation_request_at_ms =
        AP_HAL::millis() + UAVCAN_PROTOCOL_DYNAMIC_NODE_ID_ALLOCATION_MIN_REQUEST_PERIOD_MS +
        get_random_range(UAVCAN_PROTOCOL_DYNAMIC_NODE_ID_ALLOCATION_MAX_FOLLOWUP_DELAY_MS);

    if (stm32_was_watchdog_reset()) {
        node_status.vendor_specific_status_code = FAIL_REASON_WATCHDOG;
    }
}


void can_update()
{
    // do one loop of CAN support. If we are doing a few update then
    // loop until it is finished
    do {
        processTx();
        processRx();
        can_handle_DNA();
        static uint32_t last_1Hz_ms;
        uint32_t now = AP_HAL::millis();
        if (now - last_1Hz_ms >= 1000) {
            last_1Hz_ms = now;
            process1HzTasks(AP_HAL::micros64());
        }
        if (fw_update.node_id != 0) {
            send_fw_read();
        }
    } while (fw_update.node_id != 0);
}

#endif // HAL_USE_CAN
