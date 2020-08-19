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
  AP_Periph can support
 */
#include <AP_HAL/AP_HAL.h>
#include <AP_Math/AP_Math.h>
#include "AP_Periph.h"
#include "hal.h"
#include <canard.h>
#include <uavcan/protocol/dynamic_node_id/Allocation.h>
#include <uavcan/protocol/NodeStatus.h>
#include <uavcan/protocol/RestartNode.h>
#include <uavcan/protocol/GetNodeInfo.h>
#include <uavcan/protocol/file/BeginFirmwareUpdate.h>
#include <uavcan/protocol/param/GetSet.h>
#include <uavcan/protocol/param/ExecuteOpcode.h>
#include <uavcan/equipment/ahrs/MagneticFieldStrength.h>
#include <uavcan/equipment/gnss/Fix.h>
#include <uavcan/equipment/gnss/Fix2.h>
#include <uavcan/equipment/gnss/Auxiliary.h>
#include <uavcan/equipment/air_data/StaticPressure.h>
#include <uavcan/equipment/air_data/StaticTemperature.h>
#include <uavcan/equipment/air_data/RawAirData.h>
#include <uavcan/equipment/indication/BeepCommand.h>
#include <uavcan/equipment/indication/LightsCommand.h>
#include <uavcan/equipment/range_sensor/Measurement.h>
#include <uavcan/equipment/hardpoint/Command.h>
#include <uavcan/equipment/esc/Status.h>
#include <ardupilot/indication/SafetyState.h>
#include <ardupilot/indication/Button.h>
#include <ardupilot/equipment/trafficmonitor/TrafficReport.h>
#include <uavcan/equipment/gnss/RTCMStream.h>
#include <uavcan/protocol/debug/LogMessage.h>
#include <stdio.h>
#include <AP_HAL_ChibiOS/hwdef/common/stm32_util.h>
#include <AP_HAL_ChibiOS/hwdef/common/watchdog.h>
#include <drivers/stm32/canard_stm32.h>
#include <AP_HAL/I2CDevice.h>
#include "../AP_Bootloader/app_comms.h"
#include <AP_HAL/utility/RingBuffer.h>
#include <AP_Common/AP_FWVersion.h>

#include "i2c.h"
#include <utility>

extern const AP_HAL::HAL &hal;
extern AP_Periph_FW periph;

#ifndef HAL_CAN_POOL_SIZE
#define HAL_CAN_POOL_SIZE 4000
#endif

static CanardInstance canard;
static uint32_t canard_memory_pool[HAL_CAN_POOL_SIZE/sizeof(uint32_t)];
#ifndef HAL_CAN_DEFAULT_NODE_ID
#define HAL_CAN_DEFAULT_NODE_ID CANARD_BROADCAST_NODE_ID
#endif
static uint8_t PreferredNodeID = HAL_CAN_DEFAULT_NODE_ID;
static uint8_t transfer_id;

#ifndef CAN_APP_NODE_NAME
#define CAN_APP_NODE_NAME                                               "org.ardupilot.ap_periph"
#endif

#ifndef CAN_PROBE_CONTINUOUS
#define CAN_PROBE_CONTINUOUS 0
#endif

/*
 * Variables used for dynamic node ID allocation.
 * RTFM at http://uavcan.org/Specification/6._Application_level_functions/#dynamic-node-id-allocation
 */
static uint32_t send_next_node_id_allocation_request_at_ms; ///< When the next node ID allocation request should be sent
static uint8_t node_id_allocation_unique_id_offset;         ///< Depends on the stage of the next request

/*
 * Node status variables
 */
static uavcan_protocol_NodeStatus node_status;


/**
 * Returns a pseudo random integer in a given range
 */
static uint16_t get_random_range(uint16_t range)
{
    return get_random16() % range;
}


/*
  get cpu unique ID
 */
static void readUniqueID(uint8_t* out_uid)
{
    uint8_t len = UAVCAN_PROTOCOL_DYNAMIC_NODE_ID_ALLOCATION_UNIQUE_ID_MAX_LENGTH;
    memset(out_uid, 0, len);
    hal.util->get_system_id_unformatted(out_uid, len);
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
    pkt.software_version.major = AP::fwversion().major;
    pkt.software_version.minor = AP::fwversion().minor;
    pkt.software_version.optional_field_flags = UAVCAN_PROTOCOL_SOFTWAREVERSION_OPTIONAL_FIELD_FLAG_VCS_COMMIT | UAVCAN_PROTOCOL_SOFTWAREVERSION_OPTIONAL_FIELD_FLAG_IMAGE_CRC;
    pkt.software_version.vcs_commit = app_descriptor.git_hash;
    uint32_t *crc = (uint32_t *)&pkt.software_version.image_crc;
    crc[0] = app_descriptor.image_crc1;
    crc[1] = app_descriptor.image_crc2;

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

    const int16_t resp_res = canardRequestOrRespond(ins,
                                                    transfer->source_node_id,
                                                    UAVCAN_PROTOCOL_GETNODEINFO_SIGNATURE,
                                                    UAVCAN_PROTOCOL_GETNODEINFO_ID,
                                                    &transfer->transfer_id,
                                                    transfer->priority,
                                                    CanardResponse,
                                                    &buffer[0],
                                                    total_size);
    if (resp_res <= 0) {
        printf("Could not respond to GetNodeInfo: %d\n", resp_res);
    }
}

/*
  handle parameter GetSet request
 */
static void handle_param_getset(CanardInstance* ins, CanardRxTransfer* transfer)
{
    // param fetch all can take a long time, so pat watchdog
    stm32_watchdog_pat();

    uavcan_protocol_param_GetSetRequest req;
    uint8_t arraybuf[UAVCAN_PROTOCOL_PARAM_GETSET_REQUEST_NAME_MAX_LENGTH];
    uint8_t *arraybuf_ptr = arraybuf;
    if (uavcan_protocol_param_GetSetRequest_decode(transfer, transfer->payload_len, &req, &arraybuf_ptr) < 0) {
        return;
    }

    uavcan_protocol_param_GetSetResponse pkt {};

    uint8_t name[AP_MAX_NAME_SIZE+1] {};
    AP_Param *vp;
    enum ap_var_type ptype;

    if (req.name.len != 0 && req.name.len > AP_MAX_NAME_SIZE) {
        vp = nullptr;
    } else if (req.name.len != 0 && req.name.len <= AP_MAX_NAME_SIZE) {
        strncpy((char *)name, (char *)req.name.data, req.name.len);
        vp = AP_Param::find((char *)name, &ptype);
    } else {
        AP_Param::ParamToken token;
        vp = AP_Param::find_by_index(req.index, &ptype, &token);
        if (vp != nullptr) {
            vp->copy_name_token(token, (char *)name, AP_MAX_NAME_SIZE+1, true);
        }
    }
    if (vp != nullptr && req.name.len != 0 && req.value.union_tag != UAVCAN_PROTOCOL_PARAM_VALUE_EMPTY) {
        // param set
        switch (ptype) {
        case AP_PARAM_INT8:
            if (req.value.union_tag != UAVCAN_PROTOCOL_PARAM_VALUE_INTEGER_VALUE) {
                return;
            }
            ((AP_Int8 *)vp)->set_and_save_ifchanged(req.value.integer_value);
            break;
        case AP_PARAM_INT16:
            if (req.value.union_tag != UAVCAN_PROTOCOL_PARAM_VALUE_INTEGER_VALUE) {
                return;
            }
            ((AP_Int16 *)vp)->set_and_save_ifchanged(req.value.integer_value);
            break;
        case AP_PARAM_INT32:
            if (req.value.union_tag != UAVCAN_PROTOCOL_PARAM_VALUE_INTEGER_VALUE) {
                return;
            }
            ((AP_Int32 *)vp)->set_and_save_ifchanged(req.value.integer_value);
            break;
        case AP_PARAM_FLOAT:
            if (req.value.union_tag != UAVCAN_PROTOCOL_PARAM_VALUE_REAL_VALUE) {
                return;
            }
            ((AP_Float *)vp)->set_and_save_ifchanged(req.value.real_value);
            break;
        default:
            return;
        }
    }
    if (vp != nullptr) {
        switch (ptype) {
        case AP_PARAM_INT8:
            pkt.value.union_tag = UAVCAN_PROTOCOL_PARAM_VALUE_INTEGER_VALUE;
            pkt.value.integer_value = ((AP_Int8 *)vp)->get();
            break;
        case AP_PARAM_INT16:
            pkt.value.union_tag = UAVCAN_PROTOCOL_PARAM_VALUE_INTEGER_VALUE;
            pkt.value.integer_value = ((AP_Int16 *)vp)->get();
            break;
        case AP_PARAM_INT32:
            pkt.value.union_tag = UAVCAN_PROTOCOL_PARAM_VALUE_INTEGER_VALUE;
            pkt.value.integer_value = ((AP_Int32 *)vp)->get();
            break;
        case AP_PARAM_FLOAT:
            pkt.value.union_tag = UAVCAN_PROTOCOL_PARAM_VALUE_REAL_VALUE;
            pkt.value.real_value = ((AP_Float *)vp)->get();
            break;
        default:
            return;
        }
        pkt.name.len = strlen((char *)name);
        pkt.name.data = name;
    }

    uint8_t buffer[UAVCAN_PROTOCOL_PARAM_GETSET_RESPONSE_MAX_SIZE];
    uint16_t total_size = uavcan_protocol_param_GetSetResponse_encode(&pkt, buffer);

    canardRequestOrRespond(ins,
                           transfer->source_node_id,
                           UAVCAN_PROTOCOL_PARAM_GETSET_SIGNATURE,
                           UAVCAN_PROTOCOL_PARAM_GETSET_ID,
                           &transfer->transfer_id,
                           transfer->priority,
                           CanardResponse,
                           &buffer[0],
                           total_size);

}

/*
  handle parameter executeopcode request
 */
static void handle_param_executeopcode(CanardInstance* ins, CanardRxTransfer* transfer)
{
    uavcan_protocol_param_ExecuteOpcodeRequest req;
    if (uavcan_protocol_param_ExecuteOpcodeRequest_decode(transfer, transfer->payload_len, &req, nullptr) < 0) {
        return;
    }
    if (req.opcode == UAVCAN_PROTOCOL_PARAM_EXECUTEOPCODE_REQUEST_OPCODE_ERASE) {
        StorageManager::erase();
        AP_Param::erase_all();
        AP_Param::load_all();
        AP_Param::setup_sketch_defaults();
#ifdef HAL_PERIPH_ENABLE_GPS
        AP_Param::setup_object_defaults(&periph.gps, periph.gps.var_info);
#endif
#ifdef HAL_PERIPH_ENABLE_MAG
        AP_Param::setup_object_defaults(&periph.compass, periph.compass.var_info);
#endif
#ifdef HAL_PERIPH_ENABLE_BARO
        AP_Param::setup_object_defaults(&periph.baro, periph.baro.var_info);
#endif
#ifdef HAL_PERIPH_ENABLE_AIRSPEED
        AP_Param::setup_object_defaults(&periph.airspeed, periph.airspeed.var_info);
#endif
#ifdef HAL_PERIPH_ENABLE_RANGEFINDER
        AP_Param::setup_object_defaults(&periph.rangefinder, periph.rangefinder.var_info);
#endif
    }

    uavcan_protocol_param_ExecuteOpcodeResponse pkt {};

    pkt.ok = true;

    uint8_t buffer[UAVCAN_PROTOCOL_PARAM_EXECUTEOPCODE_RESPONSE_MAX_SIZE];
    uint16_t total_size = uavcan_protocol_param_ExecuteOpcodeResponse_encode(&pkt, buffer);

    canardRequestOrRespond(ins,
                           transfer->source_node_id,
                           UAVCAN_PROTOCOL_PARAM_EXECUTEOPCODE_SIGNATURE,
                           UAVCAN_PROTOCOL_PARAM_EXECUTEOPCODE_ID,
                           &transfer->transfer_id,
                           transfer->priority,
                           CanardResponse,
                           &buffer[0],
                           total_size);
}

static void processTx(void);
static void processRx(void);

static void handle_begin_firmware_update(CanardInstance* ins, CanardRxTransfer* transfer)
{
#if HAL_RAM_RESERVE_START >= 256
    // setup information on firmware request at start of ram
    struct app_bootloader_comms *comms = (struct app_bootloader_comms *)HAL_RAM0_START;
    memset(comms, 0, sizeof(struct app_bootloader_comms));
    comms->magic = APP_BOOTLOADER_COMMS_MAGIC;

    // manual decoding due to TAO bug in libcanard generated code
    if (transfer->payload_len < 1 || transfer->payload_len > sizeof(comms->path)+1) {
        return;
    }
    uint32_t offset = 0;
    canardDecodeScalar(transfer, 0, 8, false, (void*)&comms->server_node_id);
    offset += 8;
    for (uint8_t i=0; i<transfer->payload_len-1; i++) {
        canardDecodeScalar(transfer, offset, 8, false, (void*)&comms->path[i]);
        offset += 8;
    }
    if (comms->server_node_id == 0) {
        comms->server_node_id = transfer->source_node_id;
    }
    comms->my_node_id = canardGetLocalNodeID(ins);

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
    uint8_t count = 50;
    while (count--) {
        processTx();
        hal.scheduler->delay(1);
    }
#endif

    // instant reboot, with backup register used to give bootloader
    // the node_id
    set_fast_reboot((rtc_boot_magic)(RTC_BOOT_CANBL | canardGetLocalNodeID(ins)));
    NVIC_SystemReset();
}

static void handle_allocation_response(CanardInstance* ins, CanardRxTransfer* transfer)
{
    // Rule C - updating the randomized time interval
    send_next_node_id_allocation_request_at_ms =
        AP_HAL::millis() + UAVCAN_PROTOCOL_DYNAMIC_NODE_ID_ALLOCATION_MIN_REQUEST_PERIOD_MS +
        get_random_range(UAVCAN_PROTOCOL_DYNAMIC_NODE_ID_ALLOCATION_MAX_FOLLOWUP_DELAY_MS);

    if (transfer->source_node_id == CANARD_BROADCAST_NODE_ID)
    {
        printf("Allocation request from another allocatee\n");
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
        printf("Mismatching allocation response\n");
        node_id_allocation_unique_id_offset = 0;
        return;         // No match, return
    }

    if (received_unique_id_len < UAVCAN_PROTOCOL_DYNAMIC_NODE_ID_ALLOCATION_UNIQUE_ID_MAX_LENGTH) {
        // The allocator has confirmed part of unique ID, switching to the next stage and updating the timeout.
        node_id_allocation_unique_id_offset = received_unique_id_len;
        send_next_node_id_allocation_request_at_ms -= UAVCAN_PROTOCOL_DYNAMIC_NODE_ID_ALLOCATION_MIN_REQUEST_PERIOD_MS;

        printf("Matching allocation response: %d\n", received_unique_id_len);
    } else {
        // Allocation complete - copying the allocated node ID from the message
        uint8_t allocated_node_id = 0;
        (void) canardDecodeScalar(transfer, 0, 7, false, &allocated_node_id);
        assert(allocated_node_id <= 127);

        canardSetLocalNodeID(ins, allocated_node_id);
        printf("Node ID allocated: %d\n", allocated_node_id);
    }
}


/*
  fix value of a float for canard float16 format
 */
static void fix_float16(float &f)
{
    *(uint16_t *)&f = canardConvertNativeFloatToFloat16(f);
}


#ifdef HAL_PERIPH_ENABLE_BUZZER
static uint32_t buzzer_start_ms;
static uint32_t buzzer_len_ms;
/*
  handle BeepCommand
 */
static void handle_beep_command(CanardInstance* ins, CanardRxTransfer* transfer)
{
    uavcan_equipment_indication_BeepCommand req;
    if (uavcan_equipment_indication_BeepCommand_decode(transfer, transfer->payload_len, &req, nullptr) < 0) {
        return;
    }
    static bool initialised;
    if (!initialised) {
        initialised = true;
        hal.rcout->init();
        hal.util->toneAlarm_init();
    }
    fix_float16(req.frequency);
    fix_float16(req.duration);
    buzzer_start_ms = AP_HAL::millis();
    buzzer_len_ms = req.duration*1000;
    float volume = constrain_float(periph.g.buzz_volume/100.0, 0, 1);
    hal.util->toneAlarm_set_buzzer_tone(req.frequency, volume, uint32_t(req.duration*1000));
}

/*
  update buzzer
 */
static void can_buzzer_update(void)
{
    if (buzzer_start_ms != 0) {
        uint32_t now = AP_HAL::millis();
        if (now - buzzer_start_ms > buzzer_len_ms) {
            hal.util->toneAlarm_set_buzzer_tone(0, 0, 0);
            buzzer_start_ms = 0;
        }
    }
}
#endif // HAL_PERIPH_ENABLE_BUZZER

#ifdef HAL_GPIO_PIN_SAFE_LED
static uint8_t safety_state;

/*
  handle SafetyState
 */
static void handle_safety_state(CanardInstance* ins, CanardRxTransfer* transfer)
{
    ardupilot_indication_SafetyState req;
    if (ardupilot_indication_SafetyState_decode(transfer, transfer->payload_len, &req, nullptr) < 0) {
        return;
    }
    safety_state = req.status;
}
#endif // HAL_GPIO_PIN_SAFE_LED

#ifdef HAL_PERIPH_ENABLE_GPS
/*
  handle gnss::RTCMStream
 */
static void handle_RTCMStream(CanardInstance* ins, CanardRxTransfer* transfer)
{
    uavcan_equipment_gnss_RTCMStream req;
    uint8_t arraybuf[UAVCAN_EQUIPMENT_GNSS_RTCMSTREAM_DATA_MAX_LENGTH];
    uint8_t *arraybuf_ptr = arraybuf;
    if (uavcan_equipment_gnss_RTCMStream_decode(transfer, transfer->payload_len, &req, &arraybuf_ptr) < 0) {
        return;
    }
    periph.gps.handle_gps_rtcm_fragment(0, req.data.data, req.data.len);
}
#endif // HAL_PERIPH_ENABLE_GPS


#ifdef AP_PERIPH_HAVE_LED
static void set_rgb_led(uint8_t red, uint8_t green, uint8_t blue)
{
#ifdef HAL_PERIPH_NEOPIXEL_COUNT
    hal.rcout->set_serial_led_rgb_data(HAL_PERIPH_NEOPIXEL_CHAN, -1, red, green, blue);
    hal.rcout->serial_led_send(HAL_PERIPH_NEOPIXEL_CHAN);
#endif // HAL_PERIPH_NEOPIXEL_COUNT
#ifdef HAL_PERIPH_ENABLE_NCP5623_LED
    {
        const uint8_t i2c_address = 0x38;
        static AP_HAL::OwnPtr<AP_HAL::I2CDevice> dev;
        if (!dev) {
            dev = std::move(hal.i2c_mgr->get_device(0, i2c_address));
        }
        WITH_SEMAPHORE(dev->get_semaphore());
        dev->set_retries(0);
        uint8_t v = 0x3f; // enable LED
        dev->transfer(&v, 1, nullptr, 0);
        v = 0x40 | red >> 3; // red
        dev->transfer(&v, 1, nullptr, 0);
        v = 0x60 | green >> 3; // green
        dev->transfer(&v, 1, nullptr, 0);
        v = 0x80 | blue >> 3; // blue
        dev->transfer(&v, 1, nullptr, 0);
    }
#endif // HAL_PERIPH_ENABLE_NCP5623_LED
}

/*
  handle lightscommand
 */
static void handle_lightscommand(CanardInstance* ins, CanardRxTransfer* transfer)
{
    uavcan_equipment_indication_LightsCommand req;
    uint8_t arraybuf[UAVCAN_EQUIPMENT_INDICATION_LIGHTSCOMMAND_MAX_SIZE];
    uint8_t *arraybuf_ptr = arraybuf;
    if (uavcan_equipment_indication_LightsCommand_decode(transfer, transfer->payload_len, &req, &arraybuf_ptr) < 0) {
        return;
    }
    for (uint8_t i=0; i<req.commands.len; i++) {
        uavcan_equipment_indication_SingleLightCommand &cmd = req.commands.data[i];
        // to get the right color proportions we scale the green so that is uses the
        // same number of bits as red and blue
        uint8_t red = cmd.color.red<<3;
        uint8_t green = (cmd.color.green>>1)<<3;
        uint8_t blue = cmd.color.blue<<3;
        if (periph.g.led_brightness != 100 && periph.g.led_brightness >= 0) {
            float scale = periph.g.led_brightness * 0.01;
            red = constrain_int16(red * scale, 0, 255);
            green = constrain_int16(green * scale, 0, 255);
            blue = constrain_int16(blue * scale, 0, 255);
        }
        set_rgb_led(red, green, blue);
    }
}
#endif // AP_PERIPH_HAVE_LED

#ifdef HAL_GPIO_PIN_SAFE_LED
/*
  update safety LED
 */
static void can_safety_LED_update(void)
{
    static uint32_t last_update_ms;
    switch (safety_state) {
    case ARDUPILOT_INDICATION_SAFETYSTATE_STATUS_SAFETY_OFF:
        palWriteLine(HAL_GPIO_PIN_SAFE_LED, SAFE_LED_ON);
        break;
    case ARDUPILOT_INDICATION_SAFETYSTATE_STATUS_SAFETY_ON: {
        uint32_t now = AP_HAL::millis();
        if (now - last_update_ms > 100) {
            last_update_ms = now;
            static uint8_t led_counter;
            const uint16_t led_pattern = 0x5500;
            led_counter = (led_counter+1) % 16;
            palWriteLine(HAL_GPIO_PIN_SAFE_LED, (led_pattern & (1U << led_counter))?!SAFE_LED_ON:SAFE_LED_ON);
        }
        break;
    }
    default:
        palWriteLine(HAL_GPIO_PIN_SAFE_LED, !SAFE_LED_ON);
        break;
    }
}
#endif // HAL_GPIO_PIN_SAFE_LED


#ifdef HAL_GPIO_PIN_SAFE_BUTTON
#ifndef HAL_SAFE_BUTTON_ON
#define HAL_SAFE_BUTTON_ON 1
#endif
/*
  update safety button
 */
static void can_safety_button_update(void)
{
    static uint32_t last_update_ms;
    static uint8_t counter;
    uint32_t now = AP_HAL::millis();
    // send at 10Hz when pressed
    if (palReadLine(HAL_GPIO_PIN_SAFE_BUTTON) != HAL_SAFE_BUTTON_ON) {
        counter = 0;
        return;
    }
    if (now - last_update_ms < 100) {
        return;
    }
    if (counter < 255) {
        counter++;
    }

    last_update_ms = now;
    ardupilot_indication_Button pkt {};
    pkt.button = ARDUPILOT_INDICATION_BUTTON_BUTTON_SAFETY;
    pkt.press_time = counter;

    uint8_t buffer[ARDUPILOT_INDICATION_BUTTON_MAX_SIZE];
    uint16_t total_size = ardupilot_indication_Button_encode(&pkt, buffer);

    canardBroadcast(&canard,
                    ARDUPILOT_INDICATION_BUTTON_SIGNATURE,
                    ARDUPILOT_INDICATION_BUTTON_ID,
                    &transfer_id,
                    CANARD_TRANSFER_PRIORITY_LOW,
                    &buffer[0],
                    total_size);
}
#endif // HAL_GPIO_PIN_SAFE_BUTTON

/**
 * This callback is invoked by the library when a new message or request or response is received.
 */
static void onTransferReceived(CanardInstance* ins,
                               CanardRxTransfer* transfer)
{
#ifdef HAL_GPIO_PIN_LED_CAN1
    palToggleLine(HAL_GPIO_PIN_LED_CAN1);
#endif

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

    case UAVCAN_PROTOCOL_RESTARTNODE_ID:
        printf("RestartNode\n");
        hal.scheduler->delay(10);
        NVIC_SystemReset();
        break;

    case UAVCAN_PROTOCOL_PARAM_GETSET_ID:
        handle_param_getset(ins, transfer);
        break;

    case UAVCAN_PROTOCOL_PARAM_EXECUTEOPCODE_ID:
        handle_param_executeopcode(ins, transfer);
        break;

#ifdef HAL_PERIPH_ENABLE_BUZZER
    case UAVCAN_EQUIPMENT_INDICATION_BEEPCOMMAND_ID:
        handle_beep_command(ins, transfer);
        break;
#endif

#ifdef HAL_GPIO_PIN_SAFE_LED
    case ARDUPILOT_INDICATION_SAFETYSTATE_ID:
        handle_safety_state(ins, transfer);
        break;
#endif

#ifdef HAL_PERIPH_ENABLE_GPS
    case UAVCAN_EQUIPMENT_GNSS_RTCMSTREAM_ID:
        handle_RTCMStream(ins, transfer);
        break;
#endif
        
#ifdef AP_PERIPH_HAVE_LED
    case UAVCAN_EQUIPMENT_INDICATION_LIGHTSCOMMAND_ID:
        handle_lightscommand(ins, transfer);
        break;
#endif
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

    if (canardGetLocalNodeID(ins) == CANARD_BROADCAST_NODE_ID)
    {
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
    case UAVCAN_PROTOCOL_PARAM_GETSET_ID:
        *out_data_type_signature = UAVCAN_PROTOCOL_PARAM_GETSET_SIGNATURE;
        return true;
    case UAVCAN_PROTOCOL_PARAM_EXECUTEOPCODE_ID:
        *out_data_type_signature = UAVCAN_PROTOCOL_PARAM_EXECUTEOPCODE_SIGNATURE;
        return true;
#ifdef HAL_PERIPH_ENABLE_BUZZER
    case UAVCAN_EQUIPMENT_INDICATION_BEEPCOMMAND_ID:
        *out_data_type_signature = UAVCAN_EQUIPMENT_INDICATION_BEEPCOMMAND_SIGNATURE;
        return true;
#endif
#ifdef HAL_GPIO_PIN_SAFE_LED
    case ARDUPILOT_INDICATION_SAFETYSTATE_ID:
        *out_data_type_signature = ARDUPILOT_INDICATION_SAFETYSTATE_SIGNATURE;
        return true;
#endif
#ifdef AP_PERIPH_HAVE_LED
    case UAVCAN_EQUIPMENT_INDICATION_LIGHTSCOMMAND_ID:
        *out_data_type_signature = UAVCAN_EQUIPMENT_INDICATION_LIGHTSCOMMAND_SIGNATURE;
        return true;
#endif
#ifdef HAL_PERIPH_ENABLE_GPS
    case UAVCAN_EQUIPMENT_GNSS_RTCMSTREAM_ID:
        *out_data_type_signature = UAVCAN_EQUIPMENT_GNSS_RTCMSTREAM_SIGNATURE;
        return true;
#endif
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

static ObjectBuffer<CANRxFrame> rxbuffer{32};

static void can_rxfull_cb(CANDriver *canp, uint32_t flags)
{
    CANRxFrame rxmsg;
    chSysLockFromISR();
    while (canTryReceiveI(canp, CAN_ANY_MAILBOX, &rxmsg) == false) {
        rxbuffer.push_force(rxmsg);
    }
    chSysUnlockFromISR();
}

static void processRx(void)
{
    CANRxFrame rxmsg;
    while (true) {
        bool have_msg;
        chSysLock();
        have_msg = rxbuffer.pop(rxmsg);
        chSysUnlock();
        if (!have_msg) {
            break;
        }
        CanardCANFrame rx_frame {};

        //palToggleLine(HAL_GPIO_PIN_LED);

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

static uint16_t pool_peak_percent(void)
{
    const CanardPoolAllocatorStatistics stats = canardGetPoolAllocatorStatistics(&canard);
    const uint16_t peak_percent = (uint16_t)(100U * stats.peak_usage_blocks / stats.capacity_blocks);
    return peak_percent;
}

/**
 * This function is called at 1 Hz rate from the main loop.
 */
static void process1HzTasks(uint64_t timestamp_usec)
{
    /*
     * Purging transfers that are no longer transmitted. This will occasionally free up some memory.
     */
    canardCleanupStaleTransfers(&canard, timestamp_usec);

    /*
     * Printing the memory usage statistics.
     */
    {
        /*
         * The recommended way to establish the minimal size of the memory pool is to stress-test the application and
         * record the worst case memory usage.
         */
        if (pool_peak_percent() > 70) {
            printf("WARNING: ENLARGE MEMORY POOL\n");
        }
    }

    /*
     * Transmitting the node status message periodically.
     */
    {
        uint8_t buffer[UAVCAN_PROTOCOL_NODESTATUS_MAX_SIZE];
        node_status.uptime_sec = AP_HAL::millis() / 1000U;

        node_status.vendor_specific_status_code = hal.util->available_memory();

        uint32_t len = uavcan_protocol_NodeStatus_encode(&node_status, buffer);

        const int16_t bc_res = canardBroadcast(&canard,
                                               UAVCAN_PROTOCOL_NODESTATUS_SIGNATURE,
                                               UAVCAN_PROTOCOL_NODESTATUS_ID,
                                               &transfer_id,
                                               CANARD_TRANSFER_PRIORITY_LOW,
                                               buffer,
                                               len);
        if (bc_res <= 0) {
            printf("broadcast fail %d\n", bc_res);
        } else {
            //printf("broadcast node status OK\n");
        }
    }

#if !defined(HAL_NO_FLASH_SUPPORT) && !defined(HAL_NO_ROMFS_SUPPORT)
    if (periph.g.flash_bootloader.get()) {
        periph.g.flash_bootloader.set_and_save_ifchanged(0);
        hal.scheduler->delay(1000);
        AP_HAL::Util::FlashBootloader res = hal.util->flash_bootloader();
        switch (res) {
        case AP_HAL::Util::FlashBootloader::OK:
            can_printf("Flash bootloader OK\n");
            break;
        case AP_HAL::Util::FlashBootloader::NO_CHANGE:
            can_printf("Bootloader unchanged\n");
            break;
        default:
            can_printf("Flash bootloader FAILED\n");
            break;
        }
    }
#endif

    node_status.mode = UAVCAN_PROTOCOL_NODESTATUS_MODE_OPERATIONAL;

#if 0
    // test code for watchdog reset
    if (AP_HAL::millis() > 15000) {
        while (true) ;
    }
#endif

    if (AP_HAL::millis() > 30000) {
        // use RTC to mark that we have been running fine for
        // 30s. This is used along with watchdog resets to ensure the
        // user has a chance to load a fixed firmware
        set_fast_reboot(RTC_BOOT_FWOK);
    }
}

/*
  wait for dynamic allocation of node ID
 */
static void can_wait_node_id(void)
{
    uint8_t node_id_allocation_transfer_id = 0;
    const uint32_t led_pattern = 0xAAAA;
    uint8_t led_idx = 0;
    uint32_t last_led_change = AP_HAL::millis();
    const uint32_t led_change_period = 50;

    while (canardGetLocalNodeID(&canard) == CANARD_BROADCAST_NODE_ID)
    {
        printf("Waiting for dynamic node ID allocation... (pool %u)\n", pool_peak_percent());

        stm32_watchdog_pat();
        uint32_t now = AP_HAL::millis();

        send_next_node_id_allocation_request_at_ms =
            now + UAVCAN_PROTOCOL_DYNAMIC_NODE_ID_ALLOCATION_MIN_REQUEST_PERIOD_MS +
            get_random_range(UAVCAN_PROTOCOL_DYNAMIC_NODE_ID_ALLOCATION_MAX_FOLLOWUP_DELAY_MS);

        while (((now=AP_HAL::millis()) < send_next_node_id_allocation_request_at_ms) &&
               (canardGetLocalNodeID(&canard) == CANARD_BROADCAST_NODE_ID))
        {
            processTx();
            processRx();
            canardCleanupStaleTransfers(&canard, AP_HAL::micros64());
            stm32_watchdog_pat();

            if (now - last_led_change > led_change_period) {
                // blink LED in recognisable pattern while waiting for DNA
                palWriteLine(HAL_GPIO_PIN_LED, (led_pattern & (1U<<led_idx))?1:0);
                led_idx = (led_idx+1) % 32;
                last_led_change = now;
            }
        }


        if (canardGetLocalNodeID(&canard) != CANARD_BROADCAST_NODE_ID)
        {
            break;
        }

        // Structure of the request is documented in the DSDL definition
        // See http://uavcan.org/Specification/6._Application_level_functions/#dynamic-node-id-allocation
        uint8_t allocation_request[CANARD_CAN_FRAME_MAX_DATA_LEN - 1];
        allocation_request[0] = (uint8_t)(PreferredNodeID << 1U);

        if (node_id_allocation_unique_id_offset == 0)
        {
            allocation_request[0] |= 1;     // First part of unique ID
        }

        uint8_t my_unique_id[UAVCAN_PROTOCOL_DYNAMIC_NODE_ID_ALLOCATION_UNIQUE_ID_MAX_LENGTH];
        readUniqueID(my_unique_id);

        static const uint8_t MaxLenOfUniqueIDInRequest = 6;
        uint8_t uid_size = (uint8_t)(UAVCAN_PROTOCOL_DYNAMIC_NODE_ID_ALLOCATION_UNIQUE_ID_MAX_LENGTH - node_id_allocation_unique_id_offset);
        if (uid_size > MaxLenOfUniqueIDInRequest)
        {
            uid_size = MaxLenOfUniqueIDInRequest;
        }

        // Paranoia time
        assert(node_id_allocation_unique_id_offset < UAVCAN_PROTOCOL_DYNAMIC_NODE_ID_ALLOCATION_UNIQUE_ID_MAX_LENGTH);
        assert(uid_size <= MaxLenOfUniqueIDInRequest);
        assert(uid_size > 0);
        assert((uid_size + node_id_allocation_unique_id_offset) <= UAVCAN_PROTOCOL_DYNAMIC_NODE_ID_ALLOCATION_UNIQUE_ID_MAX_LENGTH);

        memmove(&allocation_request[1], &my_unique_id[node_id_allocation_unique_id_offset], uid_size);

        // Broadcasting the request
        const int16_t bcast_res = canardBroadcast(&canard,
                                                  UAVCAN_PROTOCOL_DYNAMIC_NODE_ID_ALLOCATION_SIGNATURE,
                                                  UAVCAN_PROTOCOL_DYNAMIC_NODE_ID_ALLOCATION_ID,
                                                  &node_id_allocation_transfer_id,
                                                  CANARD_TRANSFER_PRIORITY_LOW,
                                                  &allocation_request[0],
                                                  (uint16_t) (uid_size + 1));
        if (bcast_res < 0)
        {
            printf("Could not broadcast ID allocation req; error %d\n", bcast_res);
        }

        // Preparing for timeout; if response is received, this value will be updated from the callback.
        node_id_allocation_unique_id_offset = 0;
    }

    printf("Dynamic node ID allocation complete [%d]\n", canardGetLocalNodeID(&canard));
}

void AP_Periph_FW::can_start()
{
    node_status.health = UAVCAN_PROTOCOL_NODESTATUS_HEALTH_OK;
    node_status.mode = UAVCAN_PROTOCOL_NODESTATUS_MODE_INITIALIZATION;
    node_status.uptime_sec = AP_HAL::millis() / 1000U;

    static CANConfig cancfg = {
        CAN_MCR_ABOM | CAN_MCR_AWUM | CAN_MCR_TXFP,
        0
    };

    // calculate optimal CAN timings given PCLK1 and baudrate
    CanardSTM32CANTimings timings {};
    canardSTM32ComputeCANTimings(STM32_PCLK1, unsigned(g.can_baudrate), &timings);
    cancfg.btr = CAN_BTR_SJW(0) |
        CAN_BTR_TS2(timings.bit_segment_2-1) |
        CAN_BTR_TS1(timings.bit_segment_1-1) |
        CAN_BTR_BRP(timings.bit_rate_prescaler-1);
    
    if (g.can_node >= 0 && g.can_node < 128) {
        PreferredNodeID = g.can_node;
    }

    CAND1.rxfull_cb = can_rxfull_cb;

    canStart(&CAND1, &cancfg);

    canardInit(&canard, (uint8_t *)canard_memory_pool, sizeof(canard_memory_pool),
               onTransferReceived, shouldAcceptTransfer, NULL);

    if (PreferredNodeID != CANARD_BROADCAST_NODE_ID) {
        canardSetLocalNodeID(&canard, PreferredNodeID);
    }

    // wait for dynamic node ID allocation
    can_wait_node_id();
}

#ifdef HAL_PERIPH_ENABLE_PWM_HARDPOINT
void AP_Periph_FW::pwm_hardpoint_init()
{
    hal.gpio->attach_interrupt(
        PWM_HARDPOINT_PIN,
        FUNCTOR_BIND_MEMBER(&AP_Periph_FW::pwm_irq_handler, void, uint8_t, bool, uint32_t), AP_HAL::GPIO::INTERRUPT_BOTH);

}

/*
  called on PWM pin transition
 */
void AP_Periph_FW::pwm_irq_handler(uint8_t pin, bool pin_state, uint32_t timestamp)
{
    if (pin_state == 0 && pwm_hardpoint.last_state == 1 && pwm_hardpoint.last_ts_us != 0) {
        uint32_t width = timestamp - pwm_hardpoint.last_ts_us;
        if (width > 500 && width < 2500) {
            pwm_hardpoint.pwm_value = width;
            if (width > pwm_hardpoint.highest_pwm) {
                pwm_hardpoint.highest_pwm = width;
            }
        }
    }
    pwm_hardpoint.last_state = pin_state;
    pwm_hardpoint.last_ts_us = timestamp;
}

void AP_Periph_FW::pwm_hardpoint_update()
{
    uint32_t now = AP_HAL::millis();
    // send at 10Hz
    void *save = hal.scheduler->disable_interrupts_save();
    uint16_t value = pwm_hardpoint.highest_pwm;
    pwm_hardpoint.highest_pwm = 0;
    hal.scheduler->restore_interrupts(save);
    float rate = g.hardpoint_rate;
    rate = constrain_float(rate, 10, 100);
    if (value > 0 && now - pwm_hardpoint.last_send_ms >= 1000U/rate) {
        pwm_hardpoint.last_send_ms = now;
        uavcan_equipment_hardpoint_Command cmd {};
        cmd.hardpoint_id = g.hardpoint_id;
        cmd.command = value;

        uint8_t buffer[UAVCAN_EQUIPMENT_HARDPOINT_COMMAND_MAX_SIZE];
        uint16_t total_size = uavcan_equipment_hardpoint_Command_encode(&cmd, buffer);
        canardBroadcast(&canard,
                        UAVCAN_EQUIPMENT_HARDPOINT_COMMAND_SIGNATURE,
                        UAVCAN_EQUIPMENT_HARDPOINT_COMMAND_ID,
                        &transfer_id,
                        CANARD_TRANSFER_PRIORITY_LOW,
                        &buffer[0],
                        total_size);
    }
}
#endif // HAL_PERIPH_ENABLE_PWM_HARDPOINT

#ifdef HAL_PERIPH_ENABLE_HWESC
void AP_Periph_FW::hwesc_telem_update()
{
    if (!hwesc_telem.update()) {
        return;
    }
    const HWESC_Telem::HWESC &t = hwesc_telem.get_telem();

    uavcan_equipment_esc_Status pkt {};
    pkt.esc_index = g.esc_number;
    pkt.voltage = t.voltage;
    pkt.current = t.current;
    pkt.temperature = MAX(t.mos_temperature, t.cap_temperature);
    pkt.rpm = t.rpm;
    pkt.power_rating_pct = t.phase_current;
    pkt.error_count = t.error_count;

    fix_float16(pkt.voltage);
    fix_float16(pkt.current);
    fix_float16(pkt.temperature);

    uint8_t buffer[UAVCAN_EQUIPMENT_ESC_STATUS_MAX_SIZE];
    uint16_t total_size = uavcan_equipment_esc_Status_encode(&pkt, buffer);
    canardBroadcast(&canard,
                    UAVCAN_EQUIPMENT_ESC_STATUS_SIGNATURE,
                    UAVCAN_EQUIPMENT_ESC_STATUS_ID,
                    &transfer_id,
                    CANARD_TRANSFER_PRIORITY_LOW,
                    &buffer[0],
                    total_size);
}
#endif // HAL_PERIPH_ENABLE_HWESC


void AP_Periph_FW::can_update()
{
    static uint32_t last_1Hz_ms;
    uint32_t now = AP_HAL::millis();
    if (now - last_1Hz_ms >= 1000) {
        last_1Hz_ms = now;
        process1HzTasks(AP_HAL::micros64());
    }
    can_mag_update();
    can_gps_update();
    can_baro_update();
    can_airspeed_update();
    can_rangefinder_update();
#ifdef HAL_PERIPH_ENABLE_BUZZER
    can_buzzer_update();
#endif
#ifdef HAL_GPIO_PIN_SAFE_LED
    can_safety_LED_update();
#endif
#ifdef HAL_GPIO_PIN_SAFE_BUTTON
    can_safety_button_update();
#endif
#ifdef HAL_PERIPH_ENABLE_PWM_HARDPOINT
    pwm_hardpoint_update();
#endif
#ifdef HAL_PERIPH_ENABLE_HWESC
    hwesc_telem_update();
#endif

    processTx();
    processRx();
}

/*
  update CAN magnetometer
 */
void AP_Periph_FW::can_mag_update(void)
{
#ifdef HAL_PERIPH_ENABLE_MAG
    if (!compass.enabled()) {
        return;
    }
    compass.read();
#if CAN_PROBE_CONTINUOUS
    if (compass.get_count() == 0) {
        static uint32_t last_probe_ms;
        uint32_t now = AP_HAL::millis();
        if (now - last_probe_ms >= 1000) {
            last_probe_ms = now;
            compass.init();
        }
    }
#endif

    if (last_mag_update_ms == compass.last_update_ms()) {
        return;
    }

    last_mag_update_ms = compass.last_update_ms();
    const Vector3f &field = compass.get_field();
    uavcan_equipment_ahrs_MagneticFieldStrength pkt {};

    // the canard dsdl compiler doesn't understand float16
    for (uint8_t i=0; i<3; i++) {
        pkt.magnetic_field_ga[i] = field[i] * 0.001;
        fix_float16(pkt.magnetic_field_ga[i]);
    }

    uint8_t buffer[UAVCAN_EQUIPMENT_AHRS_MAGNETICFIELDSTRENGTH_MAX_SIZE];
    uint16_t total_size = uavcan_equipment_ahrs_MagneticFieldStrength_encode(&pkt, buffer);

    canardBroadcast(&canard,
                    UAVCAN_EQUIPMENT_AHRS_MAGNETICFIELDSTRENGTH_SIGNATURE,
                    UAVCAN_EQUIPMENT_AHRS_MAGNETICFIELDSTRENGTH_ID,
                    &transfer_id,
                    CANARD_TRANSFER_PRIORITY_LOW,
                    &buffer[0],
                    total_size);
#endif // HAL_PERIPH_ENABLE_MAG
}

/*
  update CAN GPS
 */
void AP_Periph_FW::can_gps_update(void)
{
#ifdef HAL_PERIPH_ENABLE_GPS
    if (gps.get_type(0) == AP_GPS::GPS_Type::GPS_TYPE_NONE) {
        return;
    }
    gps.update();
    if (last_gps_update_ms == gps.last_message_time_ms()) {
        return;
    }
    last_gps_update_ms = gps.last_message_time_ms();

    {
        /*
          send Fix packet
        */
        uavcan_equipment_gnss_Fix pkt {};
        const Location &loc = gps.location();
        const Vector3f &vel = gps.velocity();

        pkt.timestamp.usec = AP_HAL::micros64();
        pkt.gnss_timestamp.usec = gps.time_epoch_usec();
        if (pkt.gnss_timestamp.usec == 0) {
            pkt.gnss_time_standard = UAVCAN_EQUIPMENT_GNSS_FIX_GNSS_TIME_STANDARD_NONE;
        } else {
            pkt.gnss_time_standard = UAVCAN_EQUIPMENT_GNSS_FIX_GNSS_TIME_STANDARD_UTC;
        }
        pkt.longitude_deg_1e8 = uint64_t(loc.lng) * 10ULL;
        pkt.latitude_deg_1e8 = uint64_t(loc.lat) * 10ULL;
        pkt.height_ellipsoid_mm = loc.alt * 10;
        pkt.height_msl_mm = loc.alt * 10;
        for (uint8_t i=0; i<3; i++) {
            // the canard dsdl compiler doesn't understand float16
            pkt.ned_velocity[i] = vel[i];
            fix_float16(pkt.ned_velocity[i]);
        }
        pkt.sats_used = gps.num_sats();
        switch (gps.status()) {
        case AP_GPS::GPS_Status::NO_GPS:
        case AP_GPS::GPS_Status::NO_FIX:
            pkt.status = UAVCAN_EQUIPMENT_GNSS_FIX_STATUS_NO_FIX;
            break;
        case AP_GPS::GPS_Status::GPS_OK_FIX_2D:
            pkt.status = UAVCAN_EQUIPMENT_GNSS_FIX_STATUS_2D_FIX;
            break;
        case AP_GPS::GPS_Status::GPS_OK_FIX_3D:
        case AP_GPS::GPS_Status::GPS_OK_FIX_3D_DGPS:
        case AP_GPS::GPS_Status::GPS_OK_FIX_3D_RTK_FLOAT:
        case AP_GPS::GPS_Status::GPS_OK_FIX_3D_RTK_FIXED:
            pkt.status = UAVCAN_EQUIPMENT_GNSS_FIX_STATUS_3D_FIX;
            break;
        }

        float pos_cov[9] {};
        pkt.position_covariance.data = &pos_cov[0];
        pkt.position_covariance.len = 9;

        float vacc;
        if (gps.vertical_accuracy(vacc)) {
            pos_cov[8] = sq(vacc);
            fix_float16(pos_cov[8]);
        }

        float hacc;
        if (gps.horizontal_accuracy(hacc)) {
            pos_cov[0] = pos_cov[4] = sq(hacc);
            fix_float16(pos_cov[0]);
            fix_float16(pos_cov[4]);
        }
    
        float vel_cov[9] {};
        pkt.velocity_covariance.data = &pos_cov[0];
        pkt.velocity_covariance.len = 9;

        float sacc;
        if (gps.speed_accuracy(sacc)) {
            float vc3 = sq(sacc);
            vel_cov[0] = vel_cov[4] = vel_cov[8] = vc3;
            fix_float16(vel_cov[0]);
            fix_float16(vel_cov[4]);
            fix_float16(vel_cov[8]);
        }

        uint8_t buffer[UAVCAN_EQUIPMENT_GNSS_FIX_MAX_SIZE];
        uint16_t total_size = uavcan_equipment_gnss_Fix_encode(&pkt, buffer);

        canardBroadcast(&canard,
                        UAVCAN_EQUIPMENT_GNSS_FIX_SIGNATURE,
                        UAVCAN_EQUIPMENT_GNSS_FIX_ID,
                        &transfer_id,
                        CANARD_TRANSFER_PRIORITY_LOW,
                        &buffer[0],
                        total_size);
    }

    {
        /*
          send Fix2 packet
        */
        uavcan_equipment_gnss_Fix2 pkt {};
        const Location &loc = gps.location();
        const Vector3f &vel = gps.velocity();

        pkt.timestamp.usec = AP_HAL::micros64();
        pkt.gnss_timestamp.usec = gps.time_epoch_usec();
        if (pkt.gnss_timestamp.usec == 0) {
            pkt.gnss_time_standard = UAVCAN_EQUIPMENT_GNSS_FIX_GNSS_TIME_STANDARD_NONE;
        } else {
            pkt.gnss_time_standard = UAVCAN_EQUIPMENT_GNSS_FIX_GNSS_TIME_STANDARD_UTC;
        }
        pkt.longitude_deg_1e8 = uint64_t(loc.lng) * 10ULL;
        pkt.latitude_deg_1e8 = uint64_t(loc.lat) * 10ULL;
        pkt.height_ellipsoid_mm = loc.alt * 10;
        pkt.height_msl_mm = loc.alt * 10;
        for (uint8_t i=0; i<3; i++) {
            pkt.ned_velocity[i] = vel[i];
        }
        pkt.sats_used = gps.num_sats();
        switch (gps.status()) {
        case AP_GPS::GPS_Status::NO_GPS:
        case AP_GPS::GPS_Status::NO_FIX:
            pkt.status = UAVCAN_EQUIPMENT_GNSS_FIX2_STATUS_NO_FIX;
            pkt.mode = UAVCAN_EQUIPMENT_GNSS_FIX2_MODE_SINGLE;
            pkt.sub_mode = UAVCAN_EQUIPMENT_GNSS_FIX2_SUB_MODE_DGPS_OTHER;
            break;
        case AP_GPS::GPS_Status::GPS_OK_FIX_2D:
            pkt.status = UAVCAN_EQUIPMENT_GNSS_FIX2_STATUS_2D_FIX;
            pkt.mode = UAVCAN_EQUIPMENT_GNSS_FIX2_MODE_SINGLE;
            pkt.sub_mode = UAVCAN_EQUIPMENT_GNSS_FIX2_SUB_MODE_DGPS_OTHER;
            break;
        case AP_GPS::GPS_Status::GPS_OK_FIX_3D:
            pkt.status = UAVCAN_EQUIPMENT_GNSS_FIX2_STATUS_3D_FIX;
            pkt.mode = UAVCAN_EQUIPMENT_GNSS_FIX2_MODE_SINGLE;
            pkt.sub_mode = UAVCAN_EQUIPMENT_GNSS_FIX2_SUB_MODE_DGPS_OTHER;
            break;
        case AP_GPS::GPS_Status::GPS_OK_FIX_3D_DGPS:
            pkt.status = UAVCAN_EQUIPMENT_GNSS_FIX2_STATUS_3D_FIX;
            pkt.mode = UAVCAN_EQUIPMENT_GNSS_FIX2_MODE_DGPS;
            pkt.sub_mode = UAVCAN_EQUIPMENT_GNSS_FIX2_SUB_MODE_DGPS_SBAS;
            break;
        case AP_GPS::GPS_Status::GPS_OK_FIX_3D_RTK_FLOAT:
            pkt.status = UAVCAN_EQUIPMENT_GNSS_FIX2_STATUS_3D_FIX;
            pkt.mode = UAVCAN_EQUIPMENT_GNSS_FIX2_MODE_RTK;
            pkt.sub_mode = UAVCAN_EQUIPMENT_GNSS_FIX2_SUB_MODE_RTK_FLOAT;
            break;
        case AP_GPS::GPS_Status::GPS_OK_FIX_3D_RTK_FIXED:
            pkt.status = UAVCAN_EQUIPMENT_GNSS_FIX2_STATUS_3D_FIX;
            pkt.mode = UAVCAN_EQUIPMENT_GNSS_FIX2_MODE_RTK;
            pkt.sub_mode = UAVCAN_EQUIPMENT_GNSS_FIX2_SUB_MODE_RTK_FIXED;
            break;
        }

        float cov[6] {};
        pkt.covariance.data = &cov[0];
        pkt.covariance.len = 6;

        float hacc;
        if (gps.horizontal_accuracy(hacc)) {
            cov[0] = cov[1] = sq(hacc);
        }
    
        float vacc;
        if (gps.vertical_accuracy(vacc)) {
            cov[2] = sq(vacc);
        }

        float sacc;
        if (gps.speed_accuracy(sacc)) {
            float vc3 = sq(sacc);
            cov[3] = cov[4] = cov[5] = vc3;
        }

        for (uint8_t i=0; i<6; i++) {
            fix_float16(cov[i]);
        }

        uint8_t buffer[UAVCAN_EQUIPMENT_GNSS_FIX2_MAX_SIZE];
        uint16_t total_size = uavcan_equipment_gnss_Fix2_encode(&pkt, buffer);

        canardBroadcast(&canard,
                        UAVCAN_EQUIPMENT_GNSS_FIX2_SIGNATURE,
                        UAVCAN_EQUIPMENT_GNSS_FIX2_ID,
                        &transfer_id,
                        CANARD_TRANSFER_PRIORITY_LOW,
                        &buffer[0],
                        total_size);
    }
    
    /*
      send aux packet
     */
    {
        uavcan_equipment_gnss_Auxiliary aux {};
        aux.hdop = gps.get_hdop() * 0.01;
        aux.vdop = gps.get_vdop() * 0.01;
        fix_float16(aux.hdop);
        fix_float16(aux.vdop);

        uint8_t buffer[UAVCAN_EQUIPMENT_GNSS_AUXILIARY_MAX_SIZE];
        uint16_t total_size = uavcan_equipment_gnss_Auxiliary_encode(&aux, buffer);
        canardBroadcast(&canard,
                        UAVCAN_EQUIPMENT_GNSS_AUXILIARY_SIGNATURE,
                        UAVCAN_EQUIPMENT_GNSS_AUXILIARY_ID,
                        &transfer_id,
                        CANARD_TRANSFER_PRIORITY_LOW,
                        &buffer[0],
                        total_size);
    }
#endif // HAL_PERIPH_ENABLE_GPS
}

/*
  update CAN baro
 */
void AP_Periph_FW::can_baro_update(void)
{
#ifdef HAL_PERIPH_ENABLE_BARO
    if (!periph.g.baro_enable) {
        return;
    }
    baro.update();
    if (last_baro_update_ms == baro.get_last_update()) {
        return;
    }

    last_baro_update_ms = baro.get_last_update();
    if (!baro.healthy()) {
        // don't send any data
        return;
    }
    const float press = baro.get_pressure();
    const float temp = baro.get_temperature();

    {
        uavcan_equipment_air_data_StaticPressure pkt {};
        pkt.static_pressure = press;
        pkt.static_pressure_variance = 0; // should we make this a parameter?
        fix_float16(pkt.static_pressure_variance);

        uint8_t buffer[UAVCAN_EQUIPMENT_AIR_DATA_STATICPRESSURE_MAX_SIZE];
        uint16_t total_size = uavcan_equipment_air_data_StaticPressure_encode(&pkt, buffer);

        canardBroadcast(&canard,
                        UAVCAN_EQUIPMENT_AIR_DATA_STATICPRESSURE_SIGNATURE,
                        UAVCAN_EQUIPMENT_AIR_DATA_STATICPRESSURE_ID,
                        &transfer_id,
                        CANARD_TRANSFER_PRIORITY_LOW,
                        &buffer[0],
                        total_size);
    }

    {
        uavcan_equipment_air_data_StaticTemperature pkt {};
        pkt.static_temperature = temp + C_TO_KELVIN;
        pkt.static_temperature_variance = 0; // should we make this a parameter?

        fix_float16(pkt.static_temperature);
        fix_float16(pkt.static_temperature_variance);

        uint8_t buffer[UAVCAN_EQUIPMENT_AIR_DATA_STATICTEMPERATURE_MAX_SIZE];
        uint16_t total_size = uavcan_equipment_air_data_StaticTemperature_encode(&pkt, buffer);

        canardBroadcast(&canard,
                        UAVCAN_EQUIPMENT_AIR_DATA_STATICTEMPERATURE_SIGNATURE,
                        UAVCAN_EQUIPMENT_AIR_DATA_STATICTEMPERATURE_ID,
                        &transfer_id,
                        CANARD_TRANSFER_PRIORITY_LOW,
                        &buffer[0],
                        total_size);
    }
#endif // HAL_PERIPH_ENABLE_BARO
}


/*
  update CAN airspeed
 */
void AP_Periph_FW::can_airspeed_update(void)
{
#ifdef HAL_PERIPH_ENABLE_AIRSPEED
    if (!airspeed.enabled()) {
        return;
    }
#if CAN_PROBE_CONTINUOUS
    if (!airspeed.healthy()) {
        uint32_t now = AP_HAL::millis();
        static uint32_t last_probe_ms;
        if (now - last_probe_ms >= 1000) {
            last_probe_ms = now;
            airspeed.init();
        }
    }
#endif
    uint32_t now = AP_HAL::millis();
    if (now - last_airspeed_update_ms < 50) {
        // max 20Hz data
        return;
    }
    last_airspeed_update_ms = now;
    airspeed.update(false);
    if (!airspeed.healthy()) {
        // don't send any data
        return;
    }
    const float press = airspeed.get_corrected_pressure();
    float temp;
    if (!airspeed.get_temperature(temp)) {
        temp = nanf("");
    } else {
        temp += C_TO_KELVIN;
    }

    uavcan_equipment_air_data_RawAirData pkt {};
    pkt.differential_pressure = press;
    pkt.static_air_temperature = temp;
    fix_float16(pkt.differential_pressure);
    fix_float16(pkt.static_air_temperature);

    // unfilled elements are NaN
    pkt.static_pressure = nanf("");
    pkt.static_pressure_sensor_temperature = nanf("");
    pkt.differential_pressure_sensor_temperature = nanf("");
    pkt.pitot_temperature = nanf("");

    uint8_t buffer[UAVCAN_EQUIPMENT_AIR_DATA_RAWAIRDATA_MAX_SIZE];
    uint16_t total_size = uavcan_equipment_air_data_RawAirData_encode(&pkt, buffer);

    canardBroadcast(&canard,
                    UAVCAN_EQUIPMENT_AIR_DATA_RAWAIRDATA_SIGNATURE,
                    UAVCAN_EQUIPMENT_AIR_DATA_RAWAIRDATA_ID,
                    &transfer_id,
                    CANARD_TRANSFER_PRIORITY_LOW,
                    &buffer[0],
                    total_size);
#endif // HAL_PERIPH_ENABLE_AIRSPEED
}


/*
  update CAN rangefinder
 */
void AP_Periph_FW::can_rangefinder_update(void)
{
#ifdef HAL_PERIPH_ENABLE_RANGEFINDER
    if (rangefinder.get_type(0) == RangeFinder::Type::NONE) {
        return;
    }
#if CAN_PROBE_CONTINUOUS
    if (rangefinder.num_sensors() == 0) {
        uint32_t now = AP_HAL::millis();
        static uint32_t last_probe_ms;
        if (now - last_probe_ms >= 1000) {
            last_probe_ms = now;
            rangefinder.init(ROTATION_NONE);
        }
    }
#endif
    uint32_t now = AP_HAL::millis();
    static uint32_t last_update_ms;
    if (now - last_update_ms < 20) {
        // max 50Hz data
        return;
    }
    last_update_ms = now;
    rangefinder.update();
    RangeFinder::Status status = rangefinder.status_orient(ROTATION_NONE);
    if (status <= RangeFinder::Status::NoData) {
        // don't send any data
        return;
    }
    uint16_t dist_cm = rangefinder.distance_cm_orient(ROTATION_NONE);
    uavcan_equipment_range_sensor_Measurement pkt {};
    pkt.sensor_id = rangefinder.get_address(0);
    switch (status) {
    case RangeFinder::Status::OutOfRangeLow:
        pkt.reading_type = UAVCAN_EQUIPMENT_RANGE_SENSOR_MEASUREMENT_READING_TYPE_TOO_CLOSE;
        break;
    case RangeFinder::Status::OutOfRangeHigh:
        pkt.reading_type = UAVCAN_EQUIPMENT_RANGE_SENSOR_MEASUREMENT_READING_TYPE_TOO_FAR;
        break;
    case RangeFinder::Status::Good:
        pkt.reading_type = UAVCAN_EQUIPMENT_RANGE_SENSOR_MEASUREMENT_READING_TYPE_VALID_RANGE;
        break;
    default:
        pkt.reading_type = UAVCAN_EQUIPMENT_RANGE_SENSOR_MEASUREMENT_READING_TYPE_UNDEFINED;
        break;
    }
    switch (rangefinder.get_mav_distance_sensor_type_orient(ROTATION_NONE)) {
    case MAV_DISTANCE_SENSOR_LASER:
        pkt.sensor_type = UAVCAN_EQUIPMENT_RANGE_SENSOR_MEASUREMENT_SENSOR_TYPE_LIDAR;
        break;
    case MAV_DISTANCE_SENSOR_ULTRASOUND:
        pkt.sensor_type = UAVCAN_EQUIPMENT_RANGE_SENSOR_MEASUREMENT_SENSOR_TYPE_SONAR;
        break;
    case MAV_DISTANCE_SENSOR_RADAR:
        pkt.sensor_type = UAVCAN_EQUIPMENT_RANGE_SENSOR_MEASUREMENT_SENSOR_TYPE_RADAR;
        break;
    default:
        pkt.sensor_type = UAVCAN_EQUIPMENT_RANGE_SENSOR_MEASUREMENT_SENSOR_TYPE_UNDEFINED;
        break;
    }

    pkt.range = dist_cm * 0.01;
    fix_float16(pkt.range);

    uint8_t buffer[UAVCAN_EQUIPMENT_RANGE_SENSOR_MEASUREMENT_MAX_SIZE];
    uint16_t total_size = uavcan_equipment_range_sensor_Measurement_encode(&pkt, buffer);

    canardBroadcast(&canard,
                    UAVCAN_EQUIPMENT_RANGE_SENSOR_MEASUREMENT_SIGNATURE,
                    UAVCAN_EQUIPMENT_RANGE_SENSOR_MEASUREMENT_ID,
                    &transfer_id,
                    CANARD_TRANSFER_PRIORITY_LOW,
                    &buffer[0],
                    total_size);
#endif // HAL_PERIPH_ENABLE_RANGEFINDER
}


#ifdef HAL_PERIPH_ENABLE_ADSB
/*
  map an ADSB_VEHICLE MAVLink message to a UAVCAN TrafficReport message
 */
void AP_Periph_FW::can_send_ADSB(struct __mavlink_adsb_vehicle_t &msg)
{
    ardupilot_equipment_trafficmonitor_TrafficReport pkt {};
    pkt.timestamp.usec = 0;
    pkt.icao_address = msg.ICAO_address;
    pkt.tslc = msg.tslc;
    pkt.latitude_deg_1e7 = msg.lat;
    pkt.longitude_deg_1e7 = msg.lon;
    pkt.alt_m = msg.altitude * 1e-3;

    pkt.heading = radians(msg.heading * 1e-2);
    fix_float16(pkt.heading);

    pkt.velocity[0] = cosf(pkt.heading) * msg.hor_velocity * 1e-2;
    pkt.velocity[1] = sinf(pkt.heading) * msg.hor_velocity * 1e-2;
    pkt.velocity[2] = -msg.ver_velocity * 1e-2;
    fix_float16(pkt.velocity[0]);
    fix_float16(pkt.velocity[1]);
    fix_float16(pkt.velocity[2]);

    pkt.squawk = msg.squawk;
    memcpy(pkt.callsign, msg.callsign, MIN(sizeof(msg.callsign),sizeof(pkt.callsign)));
    if (msg.flags & 0x8000) {
        pkt.source = ARDUPILOT_EQUIPMENT_TRAFFICMONITOR_TRAFFICREPORT_SOURCE_ADSB_UAT;
    } else {
        pkt.source = ARDUPILOT_EQUIPMENT_TRAFFICMONITOR_TRAFFICREPORT_SOURCE_ADSB;
    }

    pkt.traffic_type = msg.emitter_type;

    if ((msg.flags & ADSB_FLAGS_VALID_ALTITUDE) != 0 && msg.altitude_type == 0) {
        pkt.alt_type = ARDUPILOT_EQUIPMENT_TRAFFICMONITOR_TRAFFICREPORT_ALT_TYPE_PRESSURE_AMSL;
    } else if ((msg.flags & ADSB_FLAGS_VALID_ALTITUDE) != 0 && msg.altitude_type == 1) {
        pkt.alt_type = ARDUPILOT_EQUIPMENT_TRAFFICMONITOR_TRAFFICREPORT_ALT_TYPE_WGS84;
    } else {
        pkt.alt_type = ARDUPILOT_EQUIPMENT_TRAFFICMONITOR_TRAFFICREPORT_ALT_TYPE_ALT_UNKNOWN;
    }

    pkt.lat_lon_valid = (msg.flags & ADSB_FLAGS_VALID_COORDS) != 0;
    pkt.heading_valid = (msg.flags & ADSB_FLAGS_VALID_HEADING) != 0;
    pkt.velocity_valid = (msg.flags & ADSB_FLAGS_VALID_VELOCITY) != 0;
    pkt.callsign_valid = (msg.flags & ADSB_FLAGS_VALID_CALLSIGN) != 0;
    pkt.ident_valid = (msg.flags & ADSB_FLAGS_VALID_SQUAWK) != 0;
    pkt.simulated_report = (msg.flags & ADSB_FLAGS_SIMULATED) != 0;

    // these flags are not in common.xml
    pkt.vertical_velocity_valid = (msg.flags & 0x0080) != 0;
    pkt.baro_valid = (msg.flags & 0x0100) != 0;

    uint8_t buffer[ARDUPILOT_EQUIPMENT_TRAFFICMONITOR_TRAFFICREPORT_MAX_SIZE];
    uint16_t total_size = ardupilot_equipment_trafficmonitor_TrafficReport_encode(&pkt, buffer);

    canardBroadcast(&canard,
                    ARDUPILOT_EQUIPMENT_TRAFFICMONITOR_TRAFFICREPORT_SIGNATURE,
                    ARDUPILOT_EQUIPMENT_TRAFFICMONITOR_TRAFFICREPORT_ID,
                    &transfer_id,
                    CANARD_TRANSFER_PRIORITY_LOW,
                    &buffer[0],
                    total_size);
}
#endif // HAL_PERIPH_ENABLE_ADSB

// printf to CAN LogMessage for debugging
void can_printf(const char *fmt, ...)
{
    uavcan_protocol_debug_LogMessage pkt {};
    uint8_t buffer[UAVCAN_PROTOCOL_DEBUG_LOGMESSAGE_MAX_SIZE];
    char tbuf[100];
    va_list ap;
    va_start(ap, fmt);
    uint32_t n = vsnprintf(tbuf, sizeof(tbuf), fmt, ap);
    va_end(ap);
    pkt.text.len = MIN(n, sizeof(tbuf));
    pkt.text.data = (uint8_t *)&tbuf[0];

    uint32_t len = uavcan_protocol_debug_LogMessage_encode(&pkt, buffer);

    canardBroadcast(&canard,
                    UAVCAN_PROTOCOL_DEBUG_LOGMESSAGE_SIGNATURE,
                    UAVCAN_PROTOCOL_DEBUG_LOGMESSAGE_ID,
                    &transfer_id,
                    CANARD_TRANSFER_PRIORITY_LOW,
                    buffer,
                    len);

}
