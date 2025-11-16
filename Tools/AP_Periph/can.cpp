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
#include <AP_HAL/AP_HAL_Boards.h>
#include "AP_Periph.h"
#include <stdio.h>
#include <drivers/stm32/canard_stm32.h>
#include <AP_HAL/I2CDevice.h>
#include <AP_HAL/utility/RingBuffer.h>
#include <AP_Common/AP_FWVersion.h>
#include <dronecan_msgs.h>

#if CONFIG_HAL_BOARD == HAL_BOARD_CHIBIOS
#include <hal.h>
#include <AP_HAL_ChibiOS/CANIface.h>
#include <AP_HAL_ChibiOS/hwdef/common/stm32_util.h>
#include <AP_HAL_ChibiOS/hwdef/common/watchdog.h>
#elif CONFIG_HAL_BOARD == HAL_BOARD_SITL
#include <AP_HAL_SITL/CANSocketIface.h>
#include <AP_HAL_SITL/AP_HAL_SITL.h>
#endif

#define IFACE_ALL ((1U<<(HAL_NUM_CAN_IFACES))-1U)

#include "i2c.h"
#include <utility>

#if HAL_NUM_CAN_IFACES >= 2
#include <AP_CANManager/AP_CANSensor.h>
#endif

#if CONFIG_HAL_BOARD == HAL_BOARD_SITL
extern const HAL_SITL &hal;
#else
extern const AP_HAL::HAL &hal;
#endif

extern AP_Periph_FW periph;

#ifndef HAL_PERIPH_LOOP_DELAY_US
// delay between can loop updates. This needs to be longer on F4
#if defined(STM32H7)
#define HAL_PERIPH_LOOP_DELAY_US 64
#else
#define HAL_PERIPH_LOOP_DELAY_US 1024
#endif
#endif

// timeout all frames at 1s
#define CAN_FRAME_TIMEOUT 1000000ULL

#define DEBUG_PKTS 0

#if HAL_PERIPH_CAN_MIRROR
  #ifndef HAL_PERIPH_CAN_MIRROR_QUEUE_SIZE
    #define HAL_PERIPH_CAN_MIRROR_QUEUE_SIZE 64
  #endif
#endif //HAL_PERIPH_CAN_MIRROR

#ifndef HAL_PERIPH_SUPPORT_LONG_CAN_PRINTF
    // When enabled, can_printf() strings longer than the droneCAN max text length (90 chars)
    // are split into multiple packets instead of truncating the string. This is
    // especially helpful with HAL_GCS_ENABLED where libraries use the mavlink
    // send_text() method where we support strings up to 256 chars by splitting them
    // up into multiple 50 char mavlink packets.
    #define HAL_PERIPH_SUPPORT_LONG_CAN_PRINTF (HAL_PROGRAM_SIZE_LIMIT_KB >= 1024)
#endif

static struct instance_t {
    uint8_t index;

#if CONFIG_HAL_BOARD == HAL_BOARD_CHIBIOS
    AP_HAL::CANIface* iface;
#elif CONFIG_HAL_BOARD == HAL_BOARD_SITL
    HALSITL::CANIface* iface;
#endif

#if HAL_PERIPH_CAN_MIRROR
  #if HAL_NUM_CAN_IFACES < 2
    #error "Can't use HAL_PERIPH_CAN_MIRROR if there are not at least 2 HAL_NUM_CAN_IFACES"
  #endif
    ObjectBuffer<AP_HAL::CANFrame> *mirror_queue;
    uint8_t mirror_fail_count;
#endif // HAL_PERIPH_CAN_MIRROR
} instances[HAL_NUM_CAN_IFACES];


#if CONFIG_HAL_BOARD == HAL_BOARD_CHIBIOS && defined(HAL_GPIO_PIN_TERMCAN1) && (HAL_NUM_CAN_IFACES >= 2)
static ioline_t can_term_lines[] = {
HAL_GPIO_PIN_TERMCAN1

#if HAL_NUM_CAN_IFACES > 2 
#ifdef HAL_GPIO_PIN_TERMCAN2
,HAL_GPIO_PIN_TERMCAN2
#else
#error "Only one Can Terminator defined with over two CAN Ifaces"
#endif
#endif

#if HAL_NUM_CAN_IFACES > 2 
#ifdef HAL_GPIO_PIN_TERMCAN3
,HAL_GPIO_PIN_TERMCAN3
#else
#error "Only two Can Terminator defined with three CAN Ifaces"
#endif
#endif

};
#endif // CONFIG_HAL_BOARD == HAL_BOARD_CHIBIOS && defined(HAL_GPIO_PIN_TERMCAN1)

uint8_t user_set_node_id = HAL_CAN_DEFAULT_NODE_ID;

#ifndef AP_PERIPH_PROBE_CONTINUOUS
#define AP_PERIPH_PROBE_CONTINUOUS 0
#endif

#ifndef AP_PERIPH_ENFORCE_AT_LEAST_ONE_PORT_IS_UAVCAN_1MHz
#define AP_PERIPH_ENFORCE_AT_LEAST_ONE_PORT_IS_UAVCAN_1MHz 1
#endif

#if CONFIG_HAL_BOARD == HAL_BOARD_CHIBIOS
ChibiOS::CANIface* AP_Periph_FW::can_iface_periph[HAL_NUM_CAN_IFACES];
#elif CONFIG_HAL_BOARD == HAL_BOARD_SITL
HALSITL::CANIface* AP_Periph_FW::can_iface_periph[HAL_NUM_CAN_IFACES];
#endif

#if AP_CAN_SLCAN_ENABLED
SLCAN::CANIface AP_Periph_FW::slcan_interface;
#endif

#ifdef EXT_FLASH_SIZE_MB
static_assert(EXT_FLASH_SIZE_MB == 0, "DroneCAN bootloader cannot support external flash");
#endif

/*
 * Node status variables
 */
static uavcan_protocol_NodeStatus node_status;
#if HAL_ENABLE_SENDING_STATS
static dronecan_protocol_Stats protocol_stats;
#endif
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
    uint8_t len = sizeof(uavcan_protocol_dynamic_node_id_Allocation::unique_id.data);
    memset(out_uid, 0, len);
    hal.util->get_system_id_unformatted(out_uid, len);
}


/*
  handle a GET_NODE_INFO request
 */
void AP_Periph_FW::handle_get_node_info(CanardInstance* canard_instance,
                                        CanardRxTransfer* transfer)
{
    uint8_t buffer[UAVCAN_PROTOCOL_GETNODEINFO_RESPONSE_MAX_SIZE];
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

    if (g.serial_number > 0) {
        hal.util->snprintf((char*)pkt.name.data, sizeof(pkt.name.data), "%s(%u)", CAN_APP_NODE_NAME, (unsigned)g.serial_number);
    } else {
        hal.util->snprintf((char*)pkt.name.data, sizeof(pkt.name.data), "%s", CAN_APP_NODE_NAME);
    }
    pkt.name.len = strnlen((char*)pkt.name.data, sizeof(pkt.name.data));

    uint16_t total_size = uavcan_protocol_GetNodeInfoResponse_encode(&pkt, buffer, !canfdout());

    canard_respond(canard_instance,
                   transfer,
                   UAVCAN_PROTOCOL_GETNODEINFO_SIGNATURE,
                   UAVCAN_PROTOCOL_GETNODEINFO_ID,
                   &buffer[0],
                   total_size);
}

// compatability code added Mar 2024 for 4.6:
#ifndef AP_PERIPH_GPS_TYPE_COMPATABILITY_ENABLED
#define AP_PERIPH_GPS_TYPE_COMPATABILITY_ENABLED 1
#endif

/*
  handle parameter GetSet request
 */
void AP_Periph_FW::handle_param_getset(CanardInstance* canard_instance, CanardRxTransfer* transfer)
{
    // param fetch all can take a long time, so pat watchdog
    stm32_watchdog_pat();

    uavcan_protocol_param_GetSetRequest req;
    if (uavcan_protocol_param_GetSetRequest_decode(transfer, &req)) {
        return;
    }

    uavcan_protocol_param_GetSetResponse pkt {};

    AP_Param *vp;
    enum ap_var_type ptype;

    if (req.name.len != 0 && req.name.len > AP_MAX_NAME_SIZE) {
        vp = nullptr;
    } else if (req.name.len != 0 && req.name.len <= AP_MAX_NAME_SIZE) {
        memcpy((char *)pkt.name.data, (char *)req.name.data, req.name.len);
#if AP_PERIPH_GPS_TYPE_COMPATABILITY_ENABLED
        // cope with older versions of ArduPilot attempting to
        // auto-configure AP_Periph using "GPS_TYPE" by
        // auto-converting to "GPS1_TYPE":
        if (strncmp((char*)req.name.data, "GPS_TYPE", req.name.len) == 0) {
            vp = AP_Param::find("GPS1_TYPE", &ptype);
        } else {
            vp = AP_Param::find((char *)pkt.name.data, &ptype);
        }
#else
        vp = AP_Param::find((char *)pkt.name.data, &ptype);
#endif
    } else {
        AP_Param::ParamToken token {};
        vp = AP_Param::find_by_index(req.index, &ptype, &token);
        if (vp != nullptr) {
            vp->copy_name_token(token, (char *)pkt.name.data, AP_MAX_NAME_SIZE+1, true);
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
        pkt.name.len = strnlen((char *)pkt.name.data, sizeof(pkt.name.data));
    }

    uint8_t buffer[UAVCAN_PROTOCOL_PARAM_GETSET_RESPONSE_MAX_SIZE];
    uint16_t total_size = uavcan_protocol_param_GetSetResponse_encode(&pkt, buffer, !canfdout());

    canard_respond(canard_instance,
                   transfer,
                   UAVCAN_PROTOCOL_PARAM_GETSET_SIGNATURE,
                   UAVCAN_PROTOCOL_PARAM_GETSET_ID,
                   &buffer[0],
                   total_size);
}

/*
  handle parameter executeopcode request
 */
void AP_Periph_FW::handle_param_executeopcode(CanardInstance* canard_instance, CanardRxTransfer* transfer)
{
    uavcan_protocol_param_ExecuteOpcodeRequest req;
    if (uavcan_protocol_param_ExecuteOpcodeRequest_decode(transfer, &req)) {
        return;
    }
    if (req.opcode == UAVCAN_PROTOCOL_PARAM_EXECUTEOPCODE_REQUEST_OPCODE_ERASE) {
        StorageManager::erase();
        AP_Param::erase_all();
        AP_Param::load_all();
        AP_Param::setup_sketch_defaults();
#if AP_PERIPH_GPS_ENABLED
        AP_Param::setup_object_defaults(&gps, gps.var_info);
#endif
#if AP_PERIPH_BATTERY_ENABLED
        AP_Param::setup_object_defaults(&battery, battery_lib.var_info);
#endif
#if AP_PERIPH_MAG_ENABLED
        AP_Param::setup_object_defaults(&compass, compass.var_info);
#endif
#if AP_PERIPH_BARO_ENABLED
        AP_Param::setup_object_defaults(&baro, baro.var_info);
#endif
#if AP_PERIPH_AIRSPEED_ENABLED
        AP_Param::setup_object_defaults(&airspeed, airspeed.var_info);
#endif
#if AP_PERIPH_RANGEFINDER_ENABLED
        AP_Param::setup_object_defaults(&rangefinder, rangefinder.var_info);
#endif
    }

    uavcan_protocol_param_ExecuteOpcodeResponse pkt {};

    pkt.ok = true;

    uint8_t buffer[UAVCAN_PROTOCOL_PARAM_EXECUTEOPCODE_RESPONSE_MAX_SIZE];
    uint16_t total_size = uavcan_protocol_param_ExecuteOpcodeResponse_encode(&pkt, buffer, !canfdout());

    canard_respond(canard_instance,
                   transfer,
                   UAVCAN_PROTOCOL_PARAM_EXECUTEOPCODE_SIGNATURE,
                   UAVCAN_PROTOCOL_PARAM_EXECUTEOPCODE_ID,
                   &buffer[0],
                   total_size);
}

void AP_Periph_FW::handle_begin_firmware_update(CanardInstance* canard_instance, CanardRxTransfer* transfer)
{
#if HAL_RAM_RESERVE_START >= 256
    // setup information on firmware request at start of ram
    auto *comms = (struct app_bootloader_comms *)HAL_RAM0_START;
    if (comms->magic != APP_BOOTLOADER_COMMS_MAGIC) {
        memset(comms, 0, sizeof(*comms));
    }
    comms->magic = APP_BOOTLOADER_COMMS_MAGIC;

    uavcan_protocol_file_BeginFirmwareUpdateRequest req;
    if (uavcan_protocol_file_BeginFirmwareUpdateRequest_decode(transfer, &req)) {
        return;
    }

    comms->server_node_id = req.source_node_id;
    if (comms->server_node_id == 0) {
        comms->server_node_id = transfer->source_node_id;
    }
    memcpy(comms->path, req.image_file_remote_path.path.data, req.image_file_remote_path.path.len);
    comms->my_node_id = canardGetLocalNodeID(canard_instance);

    uint8_t buffer[UAVCAN_PROTOCOL_FILE_BEGINFIRMWAREUPDATE_RESPONSE_MAX_SIZE];
    uavcan_protocol_file_BeginFirmwareUpdateResponse reply {};
    reply.error = UAVCAN_PROTOCOL_FILE_BEGINFIRMWAREUPDATE_RESPONSE_ERROR_OK;

    uint32_t total_size = uavcan_protocol_file_BeginFirmwareUpdateResponse_encode(&reply, buffer, !canfdout());
    canard_respond(canard_instance,
                   transfer,
                   UAVCAN_PROTOCOL_FILE_BEGINFIRMWAREUPDATE_SIGNATURE,
                   UAVCAN_PROTOCOL_FILE_BEGINFIRMWAREUPDATE_ID,
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
    prepare_reboot();
#if CONFIG_HAL_BOARD == HAL_BOARD_CHIBIOS
    set_fast_reboot((rtc_boot_magic)(RTC_BOOT_CANBL | canardGetLocalNodeID(canard_instance)));
    NVIC_SystemReset();
#endif
}

void AP_Periph_FW::handle_allocation_response(CanardInstance* canard_instance, CanardRxTransfer* transfer)
{
    // Rule C - updating the randomized time interval
    dronecan.send_next_node_id_allocation_request_at_ms =
        AP_HAL::millis() + UAVCAN_PROTOCOL_DYNAMIC_NODE_ID_ALLOCATION_MIN_REQUEST_PERIOD_MS +
        get_random_range(UAVCAN_PROTOCOL_DYNAMIC_NODE_ID_ALLOCATION_MAX_FOLLOWUP_DELAY_MS);

    if (transfer->source_node_id == CANARD_BROADCAST_NODE_ID)
    {
        printf("Allocation request from another allocatee\n");
        dronecan.node_id_allocation_unique_id_offset = 0;
        return;
    }

    // Copying the unique ID from the message
    uavcan_protocol_dynamic_node_id_Allocation msg;

    if (uavcan_protocol_dynamic_node_id_Allocation_decode(transfer, &msg)) {
        // failed decode
        return;
    }

    // Obtaining the local unique ID
    uint8_t my_unique_id[sizeof(msg.unique_id.data)];
    readUniqueID(my_unique_id);

    // Matching the received UID against the local one
    if (memcmp(msg.unique_id.data, my_unique_id, msg.unique_id.len) != 0) {
        printf("Mismatching allocation response\n");
        dronecan.node_id_allocation_unique_id_offset = 0;
        return;         // No match, return
    }

    if (msg.unique_id.len < sizeof(msg.unique_id.data)) {
        // The allocator has confirmed part of unique ID, switching to the next stage and updating the timeout.
        dronecan.node_id_allocation_unique_id_offset = msg.unique_id.len;
        dronecan.send_next_node_id_allocation_request_at_ms -= UAVCAN_PROTOCOL_DYNAMIC_NODE_ID_ALLOCATION_MIN_REQUEST_PERIOD_MS;

        printf("Matching allocation response: %d\n", msg.unique_id.len);
    } else if (msg.node_id != CANARD_BROADCAST_NODE_ID) { // new ID valid? (if not we will time out and start over)
        // Allocation complete - copying the allocated node ID from the message
        canardSetLocalNodeID(canard_instance, msg.node_id);
        printf("IF%d Node ID allocated: %d\n", dronecan.dna_interface, msg.node_id);

#if AP_PERIPH_GPS_ENABLED && (HAL_NUM_CAN_IFACES >= 2) && GPS_MOVING_BASELINE
        if (g.gps_mb_only_can_port) {
            // we need to assign the unallocated port to be used for Moving Baseline only
            gps_mb_can_port = (dronecan.dna_interface+1)%HAL_NUM_CAN_IFACES;
            if (canardGetLocalNodeID(&dronecan.canard) == CANARD_BROADCAST_NODE_ID) {
                // copy node id from the primary iface
                canardSetLocalNodeID(&dronecan.canard, msg.node_id);
#ifdef HAL_GPIO_PIN_TERMCAN1
                // also terminate the line as we don't have any other device on this port
                palWriteLine(can_term_lines[gps_mb_can_port], 1);
#endif
            }
        }
#endif
    }
}


#if defined(HAL_GPIO_PIN_SAFE_LED) || AP_PERIPH_RC_OUT_ENABLED
static uint8_t safety_state;

/*
  handle SafetyState
 */
void AP_Periph_FW::handle_safety_state(CanardInstance* canard_instance, CanardRxTransfer* transfer)
{
    ardupilot_indication_SafetyState req;
    if (ardupilot_indication_SafetyState_decode(transfer, &req)) {
        return;
    }
    safety_state = req.status;
#if AP_PERIPH_SAFETY_SWITCH_ENABLED
    rcout_handle_safety_state(safety_state);
#endif
}
#endif // HAL_GPIO_PIN_SAFE_LED

/*
  handle ArmingStatus
 */
void AP_Periph_FW::handle_arming_status(CanardInstance* canard_instance, CanardRxTransfer* transfer)
{
    uavcan_equipment_safety_ArmingStatus req;
    if (uavcan_equipment_safety_ArmingStatus_decode(transfer, &req)) {
        return;
    }
    hal.util->set_soft_armed(req.status == UAVCAN_EQUIPMENT_SAFETY_ARMINGSTATUS_STATUS_FULLY_ARMED);
}


#if AP_PERIPH_RTC_GLOBALTIME_ENABLED
/*
  handle GlobalTime
 */
void AP_Periph_FW::handle_globaltime(CanardInstance* canard_instance, CanardRxTransfer* transfer)
{
    dronecan_protocol_GlobalTime req;
    if (dronecan_protocol_GlobalTime_decode(transfer, &req)) {
        return;
    }
    AP::rtc().set_utc_usec(req.timestamp.usec, AP_RTC::source_type::SOURCE_GPS);
}
#endif // AP_PERIPH_RTC_GLOBALTIME_ENABLED



#if AP_PERIPH_HAVE_LED_WITHOUT_NOTIFY || AP_PERIPH_NOTIFY_ENABLED
void AP_Periph_FW::set_rgb_led(uint8_t red, uint8_t green, uint8_t blue)
{
#if AP_PERIPH_NOTIFY_ENABLED
    notify.handle_rgb(red, green, blue);
#if AP_PERIPH_RC_OUT_ENABLED
    rcout_has_new_data_to_update = true;
#endif // AP_PERIPH_RC_OUT_ENABLED
#endif // AP_PERIPH_NOTIFY_ENABLED

#ifdef HAL_PERIPH_NEOPIXEL_COUNT_WITHOUT_NOTIFY
    hal.rcout->set_serial_led_rgb_data(HAL_PERIPH_NEOPIXEL_CHAN_WITHOUT_NOTIFY, -1, red, green, blue);
    hal.rcout->serial_led_send(HAL_PERIPH_NEOPIXEL_CHAN_WITHOUT_NOTIFY);
#endif // HAL_PERIPH_NEOPIXEL_COUNT_WITHOUT_NOTIFY

#if AP_PERIPH_NCP5623_LED_WITHOUT_NOTIFY_ENABLED
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
#endif // AP_PERIPH_NCP5623_LED_WITHOUT_NOTIFY_ENABLED

#if AP_PERIPH_NCP5623_BGR_LED_WITHOUT_NOTIFY_ENABLED
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
        v = 0x40 | blue >> 3; // blue
        dev->transfer(&v, 1, nullptr, 0);
        v = 0x60 | green >> 3; // green
        dev->transfer(&v, 1, nullptr, 0);
        v = 0x80 | red >> 3; // red
        dev->transfer(&v, 1, nullptr, 0);
    }
#endif // AP_PERIPH_NCP5623_BGR_LED_WITHOUT_NOTIFY_ENABLED
#if AP_PERIPH_TOSHIBA_LED_WITHOUT_NOTIFY_ENABLED
    {
#define TOSHIBA_LED_PWM0    0x01    // pwm0 register
#define TOSHIBA_LED_ENABLE  0x04    // enable register
#define TOSHIBA_LED_I2C_ADDR 0x55   // default I2C bus address

        static AP_HAL::OwnPtr<AP_HAL::I2CDevice> dev_toshiba;
        if (!dev_toshiba) {
            dev_toshiba = std::move(hal.i2c_mgr->get_device(0, TOSHIBA_LED_I2C_ADDR));
        }
        WITH_SEMAPHORE(dev_toshiba->get_semaphore());
        dev_toshiba->set_retries(0); // use 0 because this is running on main thread.

        // enable the led
        dev_toshiba->write_register(TOSHIBA_LED_ENABLE, 0x03);

        /* 4-bit for each color */
        uint8_t val[4] = { 
            TOSHIBA_LED_PWM0, 
            (uint8_t)(blue >> 4),
            (uint8_t)(green / 16), 
            (uint8_t)(red / 16) 
        };
        dev_toshiba->transfer(val, sizeof(val), nullptr, 0);
    }
#endif // AP_PERIPH_TOSHIBA_LED_WITHOUT_NOTIFY_ENABLED
}

/*
  handle lightscommand
 */
void AP_Periph_FW::handle_lightscommand(CanardInstance* canard_instance, CanardRxTransfer* transfer)
{
    uavcan_equipment_indication_LightsCommand req;
    if (uavcan_equipment_indication_LightsCommand_decode(transfer, &req)) {
        return;
    }
    for (uint8_t i=0; i<req.commands.len; i++) {
        uavcan_equipment_indication_SingleLightCommand &cmd = req.commands.data[i];
        // to get the right color proportions we scale the green so that is uses the
        // same number of bits as red and blue
        uint8_t red = cmd.color.red<<3U;
        uint8_t green = (cmd.color.green>>1U)<<3U;
        uint8_t blue = cmd.color.blue<<3U;
#if AP_PERIPH_NOTIFY_ENABLED
        const int8_t brightness = notify.get_rgb_led_brightness_percent();
#elif AP_PERIPH_HAVE_LED_WITHOUT_NOTIFY
        const int8_t brightness = g.led_brightness;
#endif
        if (brightness != 100 && brightness >= 0) {
            const float scale = brightness * 0.01;
            red = constrain_int16(red * scale, 0, 255);
            green = constrain_int16(green * scale, 0, 255);
            blue = constrain_int16(blue * scale, 0, 255);
        }
        set_rgb_led(red, green, blue);
    }
}
#endif // AP_PERIPH_HAVE_LED_WITHOUT_NOTIFY

#if AP_PERIPH_RC_OUT_ENABLED
void AP_Periph_FW::handle_esc_rawcommand(CanardInstance* canard_instance, CanardRxTransfer* transfer)
{
    uavcan_equipment_esc_RawCommand cmd;
    if (uavcan_equipment_esc_RawCommand_decode(transfer, &cmd)) {
        return;
    }
    rcout_esc(cmd.cmd.data, cmd.cmd.len);

    // Update internal copy for disabling output to ESC when CAN packets are lost
    last_esc_num_channels = cmd.cmd.len;
    last_esc_raw_command_ms = AP_HAL::millis();
}

void AP_Periph_FW::handle_act_command(CanardInstance* canard_instance, CanardRxTransfer* transfer)
{
    uavcan_equipment_actuator_ArrayCommand cmd;
    if (uavcan_equipment_actuator_ArrayCommand_decode(transfer, &cmd)) {
        return;
    }

    bool valid_output = false;
    for (uint8_t i=0; i < cmd.commands.len; i++) {
        const auto &c = cmd.commands.data[i];
        switch (c.command_type) {
        case UAVCAN_EQUIPMENT_ACTUATOR_COMMAND_COMMAND_TYPE_UNITLESS:
            rcout_srv_unitless(c.actuator_id, c.command_value);
            valid_output = true;
            break;
        case UAVCAN_EQUIPMENT_ACTUATOR_COMMAND_COMMAND_TYPE_PWM:
            rcout_srv_PWM(c.actuator_id, c.command_value);
            valid_output = true;
            break;
        }
    }

    if (valid_output) {
        actuator.last_command_ms = AP_HAL::millis();
    }
}
#endif // AP_PERIPH_RC_OUT_ENABLED

#if AP_PERIPH_NOTIFY_ENABLED
void AP_Periph_FW::handle_notify_state(CanardInstance* canard_instance, CanardRxTransfer* transfer)
{
    ardupilot_indication_NotifyState msg;
    if (ardupilot_indication_NotifyState_decode(transfer, &msg)) {
        return;
    }
    if (msg.aux_data.len == 2 && msg.aux_data_type == ARDUPILOT_INDICATION_NOTIFYSTATE_VEHICLE_YAW_EARTH_CENTIDEGREES) {
        uint16_t tmp = 0;
        memcpy(&tmp, msg.aux_data.data, sizeof(tmp));
        yaw_earth = radians((float)tmp * 0.01f);
    }
    vehicle_state = msg.vehicle_state;
    last_vehicle_state_ms = AP_HAL::millis();
}
#endif // AP_PERIPH_NOTIFY_ENABLED

#ifdef HAL_GPIO_PIN_SAFE_LED
/*
  update safety LED
 */
void AP_Periph_FW::can_safety_LED_update(void)
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
void AP_Periph_FW::can_safety_button_update(void)
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
    uint16_t total_size = ardupilot_indication_Button_encode(&pkt, buffer, !canfdout());

    canard_broadcast(ARDUPILOT_INDICATION_BUTTON_SIGNATURE,
                            ARDUPILOT_INDICATION_BUTTON_ID,
                            CANARD_TRANSFER_PRIORITY_LOW,
                            &buffer[0],
                            total_size);
}
#endif // HAL_GPIO_PIN_SAFE_BUTTON

/**
 * This callback is invoked by the library when a new message or request or response is received.
 */
void AP_Periph_FW::onTransferReceived(CanardInstance* canard_instance,
                                      CanardRxTransfer* transfer)
{
#ifdef HAL_GPIO_PIN_LED_CAN1
    palToggleLine(HAL_GPIO_PIN_LED_CAN1);
#endif

#if HAL_CANFD_SUPPORTED
    // enable tao for decoding when not on CANFD
    transfer->tao = !transfer->canfd;
#endif

    /*
     * Dynamic node ID allocation protocol.
     * Taking this branch only if we don't have a node ID, ignoring otherwise.
     */
    if (canardGetLocalNodeID(canard_instance) == CANARD_BROADCAST_NODE_ID) {
        if (transfer->transfer_type == CanardTransferTypeBroadcast &&
            transfer->data_type_id == UAVCAN_PROTOCOL_DYNAMIC_NODE_ID_ALLOCATION_ID) {
            handle_allocation_response(canard_instance, transfer);
        }
        return;
    }

    switch (transfer->data_type_id) {
    case UAVCAN_PROTOCOL_GETNODEINFO_ID:
        handle_get_node_info(canard_instance, transfer);
        break;

    case UAVCAN_PROTOCOL_FILE_BEGINFIRMWAREUPDATE_ID:
        handle_begin_firmware_update(canard_instance, transfer);
        break;

    case UAVCAN_PROTOCOL_RESTARTNODE_ID: {
            printf("RestartNode\n");
            uavcan_protocol_RestartNodeResponse pkt {
                ok: true,
            };
            uint8_t buffer[UAVCAN_PROTOCOL_RESTARTNODE_RESPONSE_MAX_SIZE];
            uint16_t total_size = uavcan_protocol_RestartNodeResponse_encode(&pkt, buffer, !canfdout());
            canard_respond(canard_instance,
                    transfer,
                    UAVCAN_PROTOCOL_RESTARTNODE_SIGNATURE,
                    UAVCAN_PROTOCOL_RESTARTNODE_ID,
                    &buffer[0],
                    total_size);

            // schedule a reboot to occur
            reboot_request_ms = AP_HAL::millis();
        }
        break;

    case UAVCAN_PROTOCOL_PARAM_GETSET_ID:
        handle_param_getset(canard_instance, transfer);
        break;

    case UAVCAN_PROTOCOL_PARAM_EXECUTEOPCODE_ID:
        handle_param_executeopcode(canard_instance, transfer);
        break;

#if AP_PERIPH_BUZZER_WITHOUT_NOTIFY_ENABLED || AP_PERIPH_NOTIFY_ENABLED
    case UAVCAN_EQUIPMENT_INDICATION_BEEPCOMMAND_ID:
        handle_beep_command(canard_instance, transfer);
        break;
#endif

#if defined(HAL_GPIO_PIN_SAFE_LED) || AP_PERIPH_RC_OUT_ENABLED
    case ARDUPILOT_INDICATION_SAFETYSTATE_ID:
        handle_safety_state(canard_instance, transfer);
        break;
#endif

    case UAVCAN_EQUIPMENT_SAFETY_ARMINGSTATUS_ID:
        handle_arming_status(canard_instance, transfer);
        break;

#if AP_PERIPH_GPS_ENABLED
    case UAVCAN_EQUIPMENT_GNSS_RTCMSTREAM_ID:
        handle_RTCMStream(canard_instance, transfer);
        break;

#if GPS_MOVING_BASELINE
    case ARDUPILOT_GNSS_MOVINGBASELINEDATA_ID:
        handle_MovingBaselineData(canard_instance, transfer);
        break;
#endif
#endif // AP_PERIPH_GPS_ENABLED

#if AP_UART_MONITOR_ENABLED
    case UAVCAN_TUNNEL_TARGETTED_ID:
        handle_tunnel_Targetted(canard_instance, transfer);
        break;
#endif

#if AP_PERIPH_HAVE_LED_WITHOUT_NOTIFY || AP_PERIPH_NOTIFY_ENABLED
    case UAVCAN_EQUIPMENT_INDICATION_LIGHTSCOMMAND_ID:
        handle_lightscommand(canard_instance, transfer);
        break;
#endif

#if AP_PERIPH_RC_OUT_ENABLED
    case UAVCAN_EQUIPMENT_ESC_RAWCOMMAND_ID:
        handle_esc_rawcommand(canard_instance, transfer);
        break;

    case UAVCAN_EQUIPMENT_ACTUATOR_ARRAYCOMMAND_ID:
        handle_act_command(canard_instance, transfer);
        break;
#endif

#if AP_PERIPH_NOTIFY_ENABLED
    case ARDUPILOT_INDICATION_NOTIFYSTATE_ID:
        handle_notify_state(canard_instance, transfer);
        break;
#endif

#if AP_PERIPH_RELAY_ENABLED
    case UAVCAN_EQUIPMENT_HARDPOINT_COMMAND_ID:
        handle_hardpoint_command(canard_instance, transfer);
        break;
#endif
#if AP_PERIPH_RTC_GLOBALTIME_ENABLED
    case DRONECAN_PROTOCOL_GLOBALTIME_ID:
        handle_globaltime(canard_instance, transfer);
        break;
#endif

    }
}

/**
 * This callback is invoked by the library when a new message or request or response is received.
 */
static void onTransferReceived_trampoline(CanardInstance* canard_instance,
                                          CanardRxTransfer* transfer)
{
    AP_Periph_FW *fw = (AP_Periph_FW *)canard_instance->user_reference;
    fw->onTransferReceived(canard_instance, transfer);
}


/**
 * This callback is invoked by the library when it detects beginning of a new transfer on the bus that can be received
 * by the local node.
 * If the callback returns true, the library will receive the transfer.
 * If the callback returns false, the library will ignore the transfer.
 * All transfers that are addressed to other nodes are always ignored.
 */
bool AP_Periph_FW::shouldAcceptTransfer(const CanardInstance* canard_instance,
                                        uint64_t* out_data_type_signature,
                                        uint16_t data_type_id,
                                        CanardTransferType transfer_type,
                                        uint8_t source_node_id)
{
    (void)source_node_id;

    if (canardGetLocalNodeID(canard_instance) == CANARD_BROADCAST_NODE_ID)
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
#if AP_PERIPH_BUZZER_WITHOUT_NOTIFY_ENABLED || AP_PERIPH_NOTIFY_ENABLED
    case UAVCAN_EQUIPMENT_INDICATION_BEEPCOMMAND_ID:
        *out_data_type_signature = UAVCAN_EQUIPMENT_INDICATION_BEEPCOMMAND_SIGNATURE;
        return true;
#endif
#if defined(HAL_GPIO_PIN_SAFE_LED) || AP_PERIPH_RC_OUT_ENABLED
    case ARDUPILOT_INDICATION_SAFETYSTATE_ID:
        *out_data_type_signature = ARDUPILOT_INDICATION_SAFETYSTATE_SIGNATURE;
        return true;
#endif
    case UAVCAN_EQUIPMENT_SAFETY_ARMINGSTATUS_ID:
        *out_data_type_signature = UAVCAN_EQUIPMENT_SAFETY_ARMINGSTATUS_SIGNATURE;
        return true;
#if AP_PERIPH_HAVE_LED_WITHOUT_NOTIFY || AP_PERIPH_NOTIFY_ENABLED
    case UAVCAN_EQUIPMENT_INDICATION_LIGHTSCOMMAND_ID:
        *out_data_type_signature = UAVCAN_EQUIPMENT_INDICATION_LIGHTSCOMMAND_SIGNATURE;
        return true;
#endif
#if AP_PERIPH_GPS_ENABLED
    case UAVCAN_EQUIPMENT_GNSS_RTCMSTREAM_ID:
        *out_data_type_signature = UAVCAN_EQUIPMENT_GNSS_RTCMSTREAM_SIGNATURE;
        return true;

#if GPS_MOVING_BASELINE
    case ARDUPILOT_GNSS_MOVINGBASELINEDATA_ID:
        *out_data_type_signature = ARDUPILOT_GNSS_MOVINGBASELINEDATA_SIGNATURE;
        return true;
#endif
#endif // AP_PERIPH_GPS_ENABLED

#if AP_UART_MONITOR_ENABLED
    case UAVCAN_TUNNEL_TARGETTED_ID:
        *out_data_type_signature = UAVCAN_TUNNEL_TARGETTED_SIGNATURE;
        return true;
#endif

#if AP_PERIPH_RC_OUT_ENABLED
    case UAVCAN_EQUIPMENT_ESC_RAWCOMMAND_ID:
        *out_data_type_signature = UAVCAN_EQUIPMENT_ESC_RAWCOMMAND_SIGNATURE;
        return true;
    
    case UAVCAN_EQUIPMENT_ACTUATOR_ARRAYCOMMAND_ID:
        *out_data_type_signature = UAVCAN_EQUIPMENT_ACTUATOR_ARRAYCOMMAND_SIGNATURE;
        return true;
#endif
#if AP_PERIPH_NOTIFY_ENABLED
    case ARDUPILOT_INDICATION_NOTIFYSTATE_ID:
        *out_data_type_signature = ARDUPILOT_INDICATION_NOTIFYSTATE_SIGNATURE;
        return true;
#endif
#if AP_PERIPH_RELAY_ENABLED
    case UAVCAN_EQUIPMENT_HARDPOINT_COMMAND_ID:
        *out_data_type_signature = UAVCAN_EQUIPMENT_HARDPOINT_COMMAND_SIGNATURE;
        return true;
#endif
#if AP_PERIPH_RTC_GLOBALTIME_ENABLED
    case DRONECAN_PROTOCOL_GLOBALTIME_ID:
        *out_data_type_signature = DRONECAN_PROTOCOL_GLOBALTIME_SIGNATURE;
        return true;
#endif
    default:
        break;
    }

    return false;
}

static bool shouldAcceptTransfer_trampoline(const CanardInstance* canard_instance,
                                            uint64_t* out_data_type_signature,
                                            uint16_t data_type_id,
                                            CanardTransferType transfer_type,
                                            uint8_t source_node_id)
{
    AP_Periph_FW *fw = (AP_Periph_FW *)canard_instance->user_reference;
    return fw->shouldAcceptTransfer(canard_instance, out_data_type_signature, data_type_id, transfer_type, source_node_id);
}

void AP_Periph_FW::cleanup_stale_transactions(uint64_t timestamp_usec)
{
    canardCleanupStaleTransfers(&dronecan.canard, timestamp_usec);
}

uint8_t *AP_Periph_FW::get_tid_ptr(uint32_t transfer_desc)
{
    // check head
    if (!dronecan.tid_map_head) {
        dronecan.tid_map_head = (dronecan_protocol_t::tid_map*)calloc(1, sizeof(dronecan_protocol_t::tid_map));
        if (dronecan.tid_map_head == nullptr) {
            return nullptr;
        }
        dronecan.tid_map_head->transfer_desc = transfer_desc;
        dronecan.tid_map_head->next = nullptr;
        return &dronecan.tid_map_head->tid;
    } else if (dronecan.tid_map_head->transfer_desc == transfer_desc) {
        return &dronecan.tid_map_head->tid;
    }

    // search through the list for an existing entry
    dronecan_protocol_t::tid_map *tid_map_ptr = dronecan.tid_map_head;
    while(tid_map_ptr->next) {
        tid_map_ptr = tid_map_ptr->next;
        if (tid_map_ptr->transfer_desc == transfer_desc) {
            return &tid_map_ptr->tid;
        }
    }

    // create a new entry, if not found
    tid_map_ptr->next = (dronecan_protocol_t::tid_map*)calloc(1, sizeof(dronecan_protocol_t::tid_map));
    if (tid_map_ptr->next == nullptr) {
        return nullptr;
    }
    tid_map_ptr->next->transfer_desc = transfer_desc;
    tid_map_ptr->next->next = nullptr;
    return &tid_map_ptr->next->tid;
}

bool AP_Periph_FW::canard_broadcast(uint64_t data_type_signature,
                                    uint16_t data_type_id,
                                    uint8_t priority,
                                    const void* payload,
                                    uint16_t payload_len,
                                    uint8_t iface_mask)
{
    WITH_SEMAPHORE(canard_broadcast_semaphore);
    const bool is_dna = data_type_id == UAVCAN_PROTOCOL_DYNAMIC_NODE_ID_ALLOCATION_ID;
    if (!is_dna && canardGetLocalNodeID(&dronecan.canard) == CANARD_BROADCAST_NODE_ID) {
        return false;
    }

    uint8_t *tid_ptr = get_tid_ptr(MAKE_TRANSFER_DESCRIPTOR(data_type_signature, data_type_id, 0, CANARD_BROADCAST_NODE_ID));
    if (tid_ptr == nullptr) {
        return false;
    }

    // create transfer object
    CanardTxTransfer transfer_object = {
        .transfer_type = CanardTransferTypeBroadcast,
        .data_type_signature = data_type_signature,
        .data_type_id = data_type_id,
        .inout_transfer_id = tid_ptr,
        .priority = priority,
        .payload = (uint8_t*)payload,
        .payload_len = payload_len,
#if CANARD_ENABLE_CANFD
        .canfd = is_dna? false : canfdout(),
#endif
        .deadline_usec = AP_HAL::micros64()+CAN_FRAME_TIMEOUT,
#if CANARD_MULTI_IFACE
        .iface_mask = iface_mask==0 ? uint8_t(IFACE_ALL) : iface_mask,
#endif
    };
    const int16_t res = canardBroadcastObj(&dronecan.canard, &transfer_object);

#if DEBUG_PKTS
    if (res < 0) {
        can_printf("Tx error %d\n", res);
    }
#endif
#if HAL_ENABLE_SENDING_STATS
    if (res <= 0) {
        protocol_stats.tx_errors++;
    } else {
        protocol_stats.tx_frames += res;
    }
#endif
    return res > 0;
}

/*
  send a response
 */
bool AP_Periph_FW::canard_respond(CanardInstance* canard_instance,
                                  CanardRxTransfer *transfer,
                                  uint64_t data_type_signature,
                                  uint16_t data_type_id,
                                  const uint8_t *payload,
                                  uint16_t payload_len)
{
    CanardTxTransfer transfer_object = {
        .transfer_type = CanardTransferTypeResponse,
        .data_type_signature = data_type_signature,
        .data_type_id = data_type_id,
        .inout_transfer_id = &transfer->transfer_id,
        .priority = transfer->priority,
        .payload = payload,
        .payload_len = payload_len,
#if CANARD_ENABLE_CANFD
        .canfd = canfdout(),
#endif
        .deadline_usec = AP_HAL::micros64()+CAN_FRAME_TIMEOUT,
#if CANARD_MULTI_IFACE
        .iface_mask = IFACE_ALL,
#endif
    };
    const auto res = canardRequestOrRespondObj(canard_instance,
                                               transfer->source_node_id,
                                               &transfer_object);
#if DEBUG_PKTS
    if (res < 0) {
        can_printf("Tx error %d\n", res);
    }
#endif
#if HAL_ENABLE_SENDING_STATS
    if (res <= 0) {
        protocol_stats.tx_errors++;
    } else {
        protocol_stats.tx_frames += res;
    }
#endif
    return res > 0;
}

void AP_Periph_FW::processTx(void)
{
    for (CanardCANFrame* txf = NULL; (txf = canardPeekTxQueue(&dronecan.canard)) != NULL;) {
        AP_HAL::CANFrame txmsg {};
        txmsg.dlc = AP_HAL::CANFrame::dataLengthToDlc(txf->data_len);
        memcpy(txmsg.data, txf->data, txf->data_len);
        txmsg.id = (txf->id | AP_HAL::CANFrame::FlagEFF);
#if HAL_CANFD_SUPPORTED
        txmsg.canfd = txf->canfd;
#endif
        // push message with 1s timeout
        bool sent = true;
        const uint64_t now_us = AP_HAL::micros64();
        const uint64_t deadline = now_us + 1000000U;
        // try sending to all interfaces
        for (auto &_ins : instances) {
            if (_ins.iface == NULL) {
                continue;
            }
    #if CANARD_MULTI_IFACE
            if (!(txf->iface_mask & (1U<<_ins.index))) {
                continue;
            }
    #endif
    #if HAL_NUM_CAN_IFACES >= 2
            if (can_protocol_cached[_ins.index] != AP_CAN::Protocol::DroneCAN) {
                continue;
            }
    #endif
            if (_ins.iface->send(txmsg, deadline, 0) <= 0) {
                /*
                  We were not able to queue the frame for
                  sending. Only mark the send as failing if the
                  interface is active. We consider an interface as
                  active if it has had a successful transmit in the
                  last 2 seconds
                 */
                volatile const auto *stats = _ins.iface->get_statistics();
                uint64_t last_transmit_us = stats->last_transmit_us;
                if (stats == nullptr || AP_HAL::micros64() - last_transmit_us < 2000000UL) {
                    sent = false;
                }
            } else {
#if CANARD_MULTI_IFACE
                txf->iface_mask &= ~(1U<<_ins.index);
#endif
            }
        }
        if (sent) {
            canardPopTxQueue(&dronecan.canard);
            dronecan.tx_fail_count = 0;
        } else {
            // exit and try again later. If we fail 8 times in a row
            // then cleanup any stale transfers to keep the queue from
            // filling
            if (dronecan.tx_fail_count < 8) {
                dronecan.tx_fail_count++;
            } else {
#if HAL_ENABLE_SENDING_STATS
                protocol_stats.tx_errors++;
#endif
                dronecan.tx_fail_count = 0;
                cleanup_stale_transactions(now_us);
            }
            break;
        }
    }
}

#if HAL_ENABLE_SENDING_STATS
void AP_Periph_FW::update_rx_protocol_stats(int16_t res)
{
    switch (-res) {
    case CANARD_OK:
        protocol_stats.rx_frames++;
        break;
    case CANARD_ERROR_OUT_OF_MEMORY:
        protocol_stats.rx_error_oom++;
        break;
    case CANARD_ERROR_INTERNAL:
        protocol_stats.rx_error_internal++;
        break;
    case CANARD_ERROR_RX_INCOMPATIBLE_PACKET:
        protocol_stats.rx_ignored_not_wanted++;
        break;
    case CANARD_ERROR_RX_WRONG_ADDRESS:
        protocol_stats.rx_ignored_wrong_address++;
        break;
    case CANARD_ERROR_RX_NOT_WANTED:
        protocol_stats.rx_ignored_not_wanted++;
        break;
    case CANARD_ERROR_RX_MISSED_START:
        protocol_stats.rx_error_missed_start++;
        break;
    case CANARD_ERROR_RX_WRONG_TOGGLE:
        protocol_stats.rx_error_wrong_toggle++;
        break;
    case CANARD_ERROR_RX_UNEXPECTED_TID:
        protocol_stats.rx_ignored_unexpected_tid++;
        break;
    case CANARD_ERROR_RX_SHORT_FRAME:
        protocol_stats.rx_error_short_frame++;
        break;
    case CANARD_ERROR_RX_BAD_CRC:
        protocol_stats.rx_error_bad_crc++;
        break;
    default:
        // mark all other errors as internal
        protocol_stats.rx_error_internal++;
        break;
    }
}
#endif

void AP_Periph_FW::processRx(void)
{
    AP_HAL::CANFrame rxmsg;
    for (auto &instance : instances) {
        if (instance.iface == NULL) {
            continue;
        }
#if HAL_NUM_CAN_IFACES >= 2
        if (can_protocol_cached[instance.index] != AP_CAN::Protocol::DroneCAN) {
            continue;
        }
#endif
        while (true) {
            bool read_select = true;
            bool write_select = false;
            instance.iface->select(read_select, write_select, nullptr, 0);
            if (!read_select) { // No data pending
                break;
            }
            CanardCANFrame rx_frame {};

            //palToggleLine(HAL_GPIO_PIN_LED);
            uint64_t timestamp;
            AP_HAL::CANIface::CanIOFlags flags;
            if (instance.iface->receive(rxmsg, timestamp, flags) <= 0) {
                break;
            }
#if HAL_PERIPH_CAN_MIRROR
            for (auto &other_instance : instances) {
                if (other_instance.mirror_queue == nullptr) { // we aren't mirroring here, or failed on memory
                    continue;
                }
                if (other_instance.index == instance.index) { // don't self add
                    continue;
                }
                other_instance.mirror_queue->push(rxmsg);
            }
#endif // HAL_PERIPH_CAN_MIRROR
            rx_frame.data_len = AP_HAL::CANFrame::dlcToDataLength(rxmsg.dlc);
            memcpy(rx_frame.data, rxmsg.data, rx_frame.data_len);
#if HAL_CANFD_SUPPORTED
            rx_frame.canfd = rxmsg.canfd;
#endif
            rx_frame.id = rxmsg.id;
#if CANARD_MULTI_IFACE
            rx_frame.iface_id = instance.index;
#endif

            const int16_t res = canardHandleRxFrame(&dronecan.canard, &rx_frame, timestamp);
#if HAL_ENABLE_SENDING_STATS
            if (res == -CANARD_ERROR_RX_MISSED_START) {
                // this might remaining frames from a message that we don't accept, so check
                uint64_t dummy_signature;
                if (shouldAcceptTransfer(&dronecan.canard,
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
#else
            (void)res;
#endif
        }
    }
}

#if HAL_PERIPH_CAN_MIRROR
void AP_Periph_FW::processMirror(void)
{
    const uint64_t deadline = AP_HAL::micros64() + 1000000;

    for (auto &ins : instances) {
        if (ins.iface == nullptr || ins.mirror_queue == nullptr) { // can't send on a null interface
            continue;
        }

        const uint32_t pending = ins.mirror_queue->available();
        for (uint32_t i = 0; i < pending; i++) { // limit how long we can loop
            AP_HAL::CANFrame txmsg {};

            if (!ins.mirror_queue->peek(txmsg)) {
                break;
            }

            if (ins.iface->send(txmsg, deadline, 0) <= 0) {
                if (ins.mirror_fail_count < 8) {
                    ins.mirror_fail_count++;
                } else {
                    ins.mirror_queue->pop();
                }
                break;
            } else {
                ins.mirror_fail_count = 0;
                ins.mirror_queue->pop();
            }
        }
    }
}
#endif // HAL_PERIPH_CAN_MIRROR

uint16_t AP_Periph_FW::pool_peak_percent()
{
    const CanardPoolAllocatorStatistics stats = canardGetPoolAllocatorStatistics(&dronecan.canard);
    const uint16_t peak_percent = (uint16_t)(100U * stats.peak_usage_blocks / stats.capacity_blocks);
    return peak_percent;
}

void AP_Periph_FW::node_status_send(void)
{
    {
        uint8_t buffer[UAVCAN_PROTOCOL_NODESTATUS_MAX_SIZE];
        node_status.uptime_sec = AP_HAL::millis() / 1000U;

        node_status.vendor_specific_status_code = MIN(hal.util->available_memory(), unsigned(UINT16_MAX));

        uint32_t len = uavcan_protocol_NodeStatus_encode(&node_status, buffer, !canfdout());

        canard_broadcast(UAVCAN_PROTOCOL_NODESTATUS_SIGNATURE,
                                UAVCAN_PROTOCOL_NODESTATUS_ID,
                                CANARD_TRANSFER_PRIORITY_LOW,
                                buffer,
                                len);
    }
#if HAL_ENABLE_SENDING_STATS
    if (debug_option_is_set(AP_Periph_FW::DebugOptions::ENABLE_STATS)) {
        {
            uint8_t buffer[DRONECAN_PROTOCOL_STATS_MAX_SIZE];
            uint32_t len = dronecan_protocol_Stats_encode(&protocol_stats, buffer, !canfdout());
            canard_broadcast(DRONECAN_PROTOCOL_STATS_SIGNATURE,
                                    DRONECAN_PROTOCOL_STATS_ID,
                                    CANARD_TRANSFER_PRIORITY_LOWEST,
                                    buffer,
                                    len);
        }
        for (auto &instance : instances) {
            uint8_t buffer[DRONECAN_PROTOCOL_CANSTATS_MAX_SIZE];
            dronecan_protocol_CanStats can_stats;
            const AP_HAL::CANIface::bus_stats_t *bus_stats = instance.iface->get_statistics();
            if (bus_stats == nullptr) {
                return;
            }
            can_stats.interface = instance.index;
            can_stats.tx_requests = bus_stats->tx_requests;
            can_stats.tx_rejected = bus_stats->tx_rejected;
            can_stats.tx_overflow = bus_stats->tx_overflow;
            can_stats.tx_success = bus_stats->tx_success;
            can_stats.tx_timedout = bus_stats->tx_timedout;
            can_stats.tx_abort = bus_stats->tx_abort;
            can_stats.rx_received = bus_stats->rx_received;
            can_stats.rx_overflow = bus_stats->rx_overflow;
            can_stats.rx_errors = bus_stats->rx_errors;
            can_stats.busoff_errors = bus_stats->num_busoff_err;
            uint32_t len = dronecan_protocol_CanStats_encode(&can_stats, buffer, !canfdout());
            canard_broadcast(DRONECAN_PROTOCOL_CANSTATS_SIGNATURE,
                            DRONECAN_PROTOCOL_CANSTATS_ID,
                            CANARD_TRANSFER_PRIORITY_LOWEST,
                            buffer,
                            len);
        }
    }
#endif
}


/**
 * This function is called at 1 Hz rate from the main loop.
 */
void AP_Periph_FW::process1HzTasks(uint64_t timestamp_usec)
{
    /*
     * Purging transfers that are no longer transmitted. This will occasionally free up some memory.
     */
    cleanup_stale_transactions(timestamp_usec);

    /*
     * Printing the memory usage statistics.
     */
    {
        /*
         * The recommended way to establish the minimal size of the memory pool is to stress-test the application and
         * record the worst case memory usage.
         */

        const float pool_pct = pool_peak_percent();
        if (pool_pct > 70) {
            printf("WARNING: ENLARGE MEMORY POOL (peak=%f%%)\n", pool_pct);
        }
    }

    /*
     * Transmitting the node status message periodically.
     */
    node_status_send();

#if !defined(HAL_NO_FLASH_SUPPORT) && !defined(HAL_NO_ROMFS_SUPPORT)
    if (g.flash_bootloader.get()) {
        const uint8_t flash_bl = g.flash_bootloader.get();
        g.flash_bootloader.set_and_save_ifchanged(0);
        if (flash_bl == 42) {
            // magic developer value to test watchdog support with main loop lockup
            while (true) {
                can_printf("entering lockup\n");
                hal.scheduler->delay(100);
            }
        }
        if (flash_bl == 43) {
            // magic developer value to test watchdog support with hard fault
            can_printf("entering fault\n");
            void *foo = (void*)0xE000ED38;
            typedef void (*fptr)();
            fptr gptr = (fptr) (void *) foo;
            gptr();
        }
        EXPECT_DELAY_MS(2000);
        hal.scheduler->delay(1000);
        AP_HAL::Util::FlashBootloader res = hal.util->flash_bootloader();
        switch (res) {
        case AP_HAL::Util::FlashBootloader::OK:
            can_printf("Flash bootloader OK\n");
            break;
        case AP_HAL::Util::FlashBootloader::NO_CHANGE:
            can_printf("Bootloader unchanged\n");
            break;
#if AP_SIGNED_FIRMWARE
        case AP_HAL::Util::FlashBootloader::NOT_SIGNED:
            can_printf("Bootloader not signed\n");
            break;
#endif
        default:
            can_printf("Flash bootloader FAILED\n");
            break;
        }
    }
#endif

#if CONFIG_HAL_BOARD == HAL_BOARD_SITL
    if (hal.run_in_maintenance_mode()) {
        node_status.mode = UAVCAN_PROTOCOL_NODESTATUS_MODE_MAINTENANCE;
    } else
#endif
    {
        node_status.mode = UAVCAN_PROTOCOL_NODESTATUS_MODE_OPERATIONAL;
    }

#if 0
    // test code for watchdog reset
    if (AP_HAL::millis() > 15000) {
        while (true) ;
    }
#endif
#if CONFIG_HAL_BOARD == HAL_BOARD_CHIBIOS
    if (AP_HAL::millis() > 30000) {
        // use RTC to mark that we have been running fine for
        // 30s. This is used along with watchdog resets to ensure the
        // user has a chance to load a fixed firmware
        set_fast_reboot(RTC_BOOT_FWOK);
    }
#endif
}

/*
  wait for dynamic allocation of node ID
 */
bool AP_Periph_FW::no_iface_finished_dna = true;

bool AP_Periph_FW::can_do_dna()
{
    if (canardGetLocalNodeID(&dronecan.canard) != CANARD_BROADCAST_NODE_ID) {
        AP_Periph_FW::no_iface_finished_dna = false;
        return true;
    }

    const uint32_t now = AP_HAL::millis();

    if (AP_Periph_FW::no_iface_finished_dna) {
        printf("Waiting for dynamic node ID allocation %x... (pool %u)\n",  IFACE_ALL, pool_peak_percent());
    }

    dronecan.send_next_node_id_allocation_request_at_ms =
        now + UAVCAN_PROTOCOL_DYNAMIC_NODE_ID_ALLOCATION_MIN_REQUEST_PERIOD_MS +
        get_random_range(UAVCAN_PROTOCOL_DYNAMIC_NODE_ID_ALLOCATION_MAX_FOLLOWUP_DELAY_MS);

    // Structure of the request is documented in the DSDL definition
    // See http://uavcan.org/Specification/6._Application_level_functions/#dynamic-node-id-allocation
    uint8_t allocation_request[CANARD_CAN_FRAME_MAX_DATA_LEN - 1];
    allocation_request[0] = 0; // we are only called if the user has not set an ID, so request any ID

    if (dronecan.node_id_allocation_unique_id_offset == 0) {
        allocation_request[0] |= 1;     // First part of unique ID
        // set interface to try
        dronecan.dna_interface++;
        dronecan.dna_interface %= HAL_NUM_CAN_IFACES;
    }

    uint8_t my_unique_id[sizeof(uavcan_protocol_dynamic_node_id_Allocation::unique_id.data)];
    readUniqueID(my_unique_id);

    static const uint8_t MaxLenOfUniqueIDInRequest = 6;
    uint8_t uid_size = (uint8_t)(sizeof(uavcan_protocol_dynamic_node_id_Allocation::unique_id.data) - dronecan.node_id_allocation_unique_id_offset);
    
    if (uid_size > MaxLenOfUniqueIDInRequest) {
        uid_size = MaxLenOfUniqueIDInRequest;
    }

    memmove(&allocation_request[1], &my_unique_id[dronecan.node_id_allocation_unique_id_offset], uid_size);

    // Broadcasting the request
    canard_broadcast(UAVCAN_PROTOCOL_DYNAMIC_NODE_ID_ALLOCATION_SIGNATURE,
                     UAVCAN_PROTOCOL_DYNAMIC_NODE_ID_ALLOCATION_ID,
                     CANARD_TRANSFER_PRIORITY_LOW,
                     &allocation_request[0],
                     (uint16_t) (uid_size + 1));

    // Preparing for timeout; if response is received, this value will be updated from the callback.
    dronecan.node_id_allocation_unique_id_offset = 0;
    return false;
}

void AP_Periph_FW::can_start()
{
    node_status.health = UAVCAN_PROTOCOL_NODESTATUS_HEALTH_OK;
    node_status.mode = UAVCAN_PROTOCOL_NODESTATUS_MODE_INITIALIZATION;
    node_status.uptime_sec = AP_HAL::millis() / 1000U;

    if (g.can_node >= 0 && g.can_node < 128) {
        user_set_node_id = g.can_node;
    }

#if !defined(HAL_NO_FLASH_SUPPORT) && !defined(HAL_NO_ROMFS_SUPPORT)
    g.flash_bootloader.set_and_save_ifchanged(0);
#endif

#if AP_PERIPH_ENFORCE_AT_LEAST_ONE_PORT_IS_UAVCAN_1MHz && HAL_NUM_CAN_IFACES >= 2
    bool has_uavcan_at_1MHz = false;
    for (uint8_t i=0; i<HAL_NUM_CAN_IFACES; i++) {
        if (g.can_protocol[i] == AP_CAN::Protocol::DroneCAN && g.can_baudrate[i] == 1000000) {
            has_uavcan_at_1MHz = true;
        }
    }
    if (!has_uavcan_at_1MHz) {
        g.can_protocol[0].set_and_save(AP_CAN::Protocol::DroneCAN);
        g.can_baudrate[0].set_and_save(1000000);
    }
#endif // HAL_PERIPH_ENFORCE_AT_LEAST_ONE_PORT_IS_UAVCAN_1MHz

    {
        /*
          support termination parameters, and also a hardware switch
          to force termination and an LED to indicate if termination
          is active
         */
#ifdef HAL_GPIO_PIN_GPIO_CAN1_TERM
        bool can1_term = g.can_terminate[0];
# ifdef HAL_GPIO_PIN_GPIO_CAN1_TERM_SWITCH
        can1_term |= palReadLine(HAL_GPIO_PIN_GPIO_CAN1_TERM_SWITCH);
# endif
        palWriteLine(HAL_GPIO_PIN_GPIO_CAN1_TERM, can1_term);
# ifdef HAL_GPIO_PIN_GPIO_CAN1_TERM_LED
        palWriteLine(HAL_GPIO_PIN_GPIO_CAN1_TERM_LED, can1_term? HAL_LED_ON : !HAL_LED_ON);
# endif
#endif

#ifdef HAL_GPIO_PIN_GPIO_CAN2_TERM
        bool can2_term = g.can_terminate[1];
# ifdef HAL_GPIO_PIN_GPIO_CAN2_TERM_SWITCH
        can2_term |= palReadLine(HAL_GPIO_PIN_GPIO_CAN2_TERM_SWITCH);
# endif
        palWriteLine(HAL_GPIO_PIN_GPIO_CAN2_TERM, can2_term);
# ifdef HAL_GPIO_PIN_GPIO_CAN2_TERM_LED
        palWriteLine(HAL_GPIO_PIN_GPIO_CAN2_TERM_LED, can2_term? HAL_LED_ON : !HAL_LED_ON);
# endif
#endif

#ifdef HAL_GPIO_PIN_GPIO_CAN3_TERM
        bool can3_term = g.can_terminate[2];
# ifdef HAL_GPIO_PIN_GPIO_CAN3_TERM_SWITCH
        can3_term |= palReadLine(HAL_GPIO_PIN_GPIO_CAN3_TERM_SWITCH);
# endif
        palWriteLine(HAL_GPIO_PIN_GPIO_CAN3_TERM, can3_term);
# ifdef HAL_GPIO_PIN_GPIO_CAN3_TERM_LED
        palWriteLine(HAL_GPIO_PIN_GPIO_CAN3_TERM_LED, can3_term? HAL_LED_ON : !HAL_LED_ON);
# endif
#endif
    }

    for (uint8_t i=0; i<HAL_NUM_CAN_IFACES; i++) {
#if CONFIG_HAL_BOARD == HAL_BOARD_CHIBIOS
        can_iface_periph[i] = NEW_NOTHROW ChibiOS::CANIface();
#elif CONFIG_HAL_BOARD == HAL_BOARD_SITL
        can_iface_periph[i] = NEW_NOTHROW HALSITL::CANIface();
#endif
        instances[i].iface = can_iface_periph[i];
        instances[i].index = i;
#if HAL_PERIPH_CAN_MIRROR
        if ((g.can_mirror_ports & (1U << i)) != 0) {
            instances[i].mirror_queue = NEW_NOTHROW ObjectBuffer<AP_HAL::CANFrame> (HAL_PERIPH_CAN_MIRROR_QUEUE_SIZE);
        }
#endif //HAL_PERIPH_CAN_MIRROR
#if HAL_NUM_CAN_IFACES >= 2
        can_protocol_cached[i] = g.can_protocol[i];
        CANSensor::set_periph(i, can_protocol_cached[i], can_iface_periph[i]);
#endif
        if (can_iface_periph[i] != nullptr) {
#if HAL_CANFD_SUPPORTED
            can_iface_periph[i]->init(g.can_baudrate[i], g.can_fdbaudrate[i]*1000000U);
#else
            can_iface_periph[i]->init(g.can_baudrate[i]);
#endif
        }
    }

#if AP_CAN_SLCAN_ENABLED
    const uint8_t slcan_selected_index = g.can_slcan_cport - 1;
    if (slcan_selected_index < HAL_NUM_CAN_IFACES) {
        slcan_interface.set_can_iface(can_iface_periph[slcan_selected_index]);
        instances[slcan_selected_index].iface = (AP_HAL::CANIface*)&slcan_interface;

        // ensure there's a serial port mapped to SLCAN
        if (!serial_manager.have_serial(AP_SerialManager::SerialProtocol_SLCAN, 0)) {
            serial_manager.set_protocol_and_baud(SERIALMANAGER_NUM_PORTS-1, AP_SerialManager::SerialProtocol_SLCAN, 1500000);
        }
    }
#endif

    canardInit(&dronecan.canard, (uint8_t *)dronecan.canard_memory_pool, sizeof(dronecan.canard_memory_pool),
               onTransferReceived_trampoline, shouldAcceptTransfer_trampoline, this);

    if (user_set_node_id != CANARD_BROADCAST_NODE_ID) {
        canardSetLocalNodeID(&dronecan.canard, user_set_node_id);
    }
}


#if AP_PERIPH_RC_OUT_ENABLED
#if HAL_WITH_ESC_TELEM
// try to map the ESC number to a motor number. This is needed
// for when we have multiple CAN nodes, one for each ESC
uint8_t AP_Periph_FW::get_motor_number(const uint8_t esc_number) const
{
    const auto *channel = SRV_Channels::srv_channel(esc_number);
    // try to map the ESC number to a motor number. This is needed
    // for when we have multiple CAN nodes, one for each ESC
    if (channel == nullptr) {
        return esc_number;
    }
    const int8_t motor_num = channel->get_motor_num();
    return (motor_num == -1) ? esc_number : motor_num;
}

/*
  send ESC status packets based on AP_ESC_Telem
 */
void AP_Periph_FW::esc_telem_update()
{
    uint32_t mask = esc_telem.get_active_esc_mask();
    while (mask != 0) {
        int8_t i = __builtin_ffs(mask) - 1;
        mask &= ~(1U<<i);
        const float nan = nanf("");
        uavcan_equipment_esc_Status pkt {};
        pkt.esc_index = get_motor_number(i);

        if (!esc_telem.get_voltage(i, pkt.voltage)) {
            pkt.voltage = nan;
        }
        if (!esc_telem.get_current(i, pkt.current)) {
            pkt.current = nan;
        }
        int16_t temperature;
        if (esc_telem.get_motor_temperature(i, temperature)) {
            pkt.temperature = C_TO_KELVIN(temperature*0.01);
        } else if (esc_telem.get_temperature(i, temperature)) {
            pkt.temperature = C_TO_KELVIN(temperature*0.01);
        } else {
            pkt.temperature = nan;
        }
        float rpm;
        float error_rate;
        if (esc_telem.get_raw_rpm_and_error_rate(i, rpm, error_rate)) {
            pkt.rpm = rpm;
            pkt.error_count = error_rate;
        }

#if AP_EXTENDED_ESC_TELEM_ENABLED
        uint8_t power_rating_pct;
        if (esc_telem.get_power_percentage(i, power_rating_pct)) {
            pkt.power_rating_pct = power_rating_pct;
        }
#endif

        uint8_t buffer[UAVCAN_EQUIPMENT_ESC_STATUS_MAX_SIZE];
        uint16_t total_size = uavcan_equipment_esc_Status_encode(&pkt, buffer, !canfdout());
        canard_broadcast(UAVCAN_EQUIPMENT_ESC_STATUS_SIGNATURE,
                         UAVCAN_EQUIPMENT_ESC_STATUS_ID,
                         CANARD_TRANSFER_PRIORITY_LOW,
                         &buffer[0],
                         total_size);
    }
}
#endif // HAL_WITH_ESC_TELEM

#if AP_EXTENDED_ESC_TELEM_ENABLED
void AP_Periph_FW::esc_telem_extended_update(const uint32_t &now_ms)
{
    if (g.esc_extended_telem_rate <= 0) {
        // Not configured to send
        return;
    }

    uint32_t mask = esc_telem.get_active_esc_mask();
    if (mask == 0) {
        // No ESCs to report
        return;
    }

    // ESCs are sent in turn to minimise used bandwidth, to make the rate param match the status message we multiply
    // the period such that the param gives the per-esc rate
    const uint32_t update_period_ms = 1000 / constrain_int32(g.esc_extended_telem_rate.get() * __builtin_popcount(mask), 1, 1000);
    if (now_ms - last_esc_telem_extended_update < update_period_ms) {
        // Too soon!
        return;
    }
    last_esc_telem_extended_update = now_ms;

    for (uint8_t i = 0; i < ESC_TELEM_MAX_ESCS; i++) {
        // Send each ESC in turn
        const uint8_t index = (last_esc_telem_extended_sent_id + 1 + i) % ESC_TELEM_MAX_ESCS;

        if ((mask & (1U << index)) == 0) {
            // Not enabled
            continue;
        }

        uavcan_equipment_esc_StatusExtended pkt {};

        // Only send if we have data
        bool have_data = false;

        int16_t motor_temp_cdeg;
        if (esc_telem.get_motor_temperature(index, motor_temp_cdeg)) {
            // Convert from centi-degrees to degrees
            pkt.motor_temperature_degC = motor_temp_cdeg * 0.01;
            have_data = true;
        }

        have_data |= esc_telem.get_input_duty(index, pkt.input_pct);
        have_data |= esc_telem.get_output_duty(index, pkt.output_pct);
        have_data |= esc_telem.get_flags(index, pkt.status_flags);

        if (have_data) {
            pkt.esc_index = get_motor_number(index);

            uint8_t buffer[UAVCAN_EQUIPMENT_ESC_STATUSEXTENDED_MAX_SIZE];
            const uint16_t total_size = uavcan_equipment_esc_StatusExtended_encode(&pkt, buffer, !canfdout());

            canard_broadcast(UAVCAN_EQUIPMENT_ESC_STATUSEXTENDED_SIGNATURE,
                                UAVCAN_EQUIPMENT_ESC_STATUSEXTENDED_ID,
                                CANARD_TRANSFER_PRIORITY_LOW,
                                &buffer[0],
                                total_size);
        }

        last_esc_telem_extended_sent_id = index;
        break;
    }
}
#endif

#if AP_PERIPH_ESC_APD_ENABLED
void AP_Periph_FW::apd_esc_telem_update()
{
    for(uint8_t i = 0; i < ARRAY_SIZE(apd_esc_telem); i++) {
        if (apd_esc_telem[i] == nullptr) {
            continue;
        }
        ESC_APD_Telem &esc = *apd_esc_telem[i];

        if (esc.update()) {
            const ESC_APD_Telem::telem &t = esc.get_telem();

            uavcan_equipment_esc_Status pkt {};
            static_assert(APD_ESC_INSTANCES <= ARRAY_SIZE(g.esc_number), "There must be an ESC instance number for each APD ESC");
            pkt.esc_index = g.esc_number[i];
            pkt.voltage = t.voltage;
            pkt.current = t.current;
            pkt.temperature = t.temperature;
            pkt.rpm = t.rpm;
            pkt.power_rating_pct = t.power_rating_pct;
            pkt.error_count = t.error_count;

            uint8_t buffer[UAVCAN_EQUIPMENT_ESC_STATUS_MAX_SIZE];
            uint16_t total_size = uavcan_equipment_esc_Status_encode(&pkt, buffer, !canfdout());
            canard_broadcast(UAVCAN_EQUIPMENT_ESC_STATUS_SIGNATURE,
                            UAVCAN_EQUIPMENT_ESC_STATUS_ID,
                            CANARD_TRANSFER_PRIORITY_LOW,
                            &buffer[0],
                            total_size);
                }
    }

}
#endif // AP_PERIPH_ESC_APD_ENABLED
#endif // AP_PERIPH_RC_OUT_ENABLED

void AP_Periph_FW::can_update()
{
    const uint32_t now = AP_HAL::millis();
    const uint32_t led_pattern = 0xAAAA;
    const uint32_t led_change_period = 50;
    static uint8_t led_idx = 0;
    static uint32_t last_led_change;

    if ((now - last_led_change > led_change_period) && no_iface_finished_dna) {
        // blink LED in recognisable pattern while waiting for DNA
#ifdef HAL_GPIO_PIN_LED
        palWriteLine(HAL_GPIO_PIN_LED, (led_pattern & (1U<<led_idx))?1:0);
#elif defined(HAL_GPIO_PIN_SAFE_LED)
        // or use safety LED if defined
        palWriteLine(HAL_GPIO_PIN_SAFE_LED, (led_pattern & (1U<<led_idx))?1:0);
#else
        (void)led_pattern;
        (void)led_idx;
#endif
        led_idx = (led_idx+1) % 32;
        last_led_change = now;
    }

    if (AP_HAL::millis() > dronecan.send_next_node_id_allocation_request_at_ms) {
        can_do_dna();
    }
    
    static uint32_t last_1Hz_ms;
    if (now - last_1Hz_ms >= 1000) {
        last_1Hz_ms = now;
        process1HzTasks(AP_HAL::micros64());
    }

#if CONFIG_HAL_BOARD == HAL_BOARD_SITL
    if (!hal.run_in_maintenance_mode())
#endif
    {
#if AP_PERIPH_MAG_ENABLED
        can_mag_update();
#endif
#if AP_PERIPH_GPS_ENABLED
        can_gps_update();
#endif
#if AP_UART_MONITOR_ENABLED
        send_serial_monitor_data();
#endif
#if AP_PERIPH_BATTERY_ENABLED
        can_battery_update();
#endif
#if AP_PERIPH_BARO_ENABLED
        can_baro_update();
#endif
#if AP_PERIPH_AIRSPEED_ENABLED
        can_airspeed_update();
#endif
#if AP_PERIPH_RANGEFINDER_ENABLED
        can_rangefinder_update();
#endif
#if AP_PERIPH_PROXIMITY_ENABLED
        can_proximity_update();
#endif
    #if AP_PERIPH_BUZZER_WITHOUT_NOTIFY_ENABLED || AP_PERIPH_NOTIFY_ENABLED
        can_buzzer_update();
    #endif
    #ifdef HAL_GPIO_PIN_SAFE_LED
        can_safety_LED_update();
    #endif
    #ifdef HAL_GPIO_PIN_SAFE_BUTTON
        can_safety_button_update();
    #endif
    #if AP_PERIPH_PWM_HARDPOINT_ENABLED
        pwm_hardpoint_update();
    #endif
    #if AP_PERIPH_HOBBYWING_ESC_ENABLED
        hwesc_telem_update();
    #endif
#if AP_PERIPH_ESC_APD_ENABLED
        apd_esc_telem_update();
#endif
    #if AP_PERIPH_MSP_ENABLED
        msp_sensor_update();
    #endif
    #if AP_PERIPH_RC_OUT_ENABLED
        rcout_update();
    #endif
    #if AP_PERIPH_EFI_ENABLED
        can_efi_update();
    #endif
#if AP_PERIPH_DEVICE_TEMPERATURE_ENABLED
        temperature_sensor_update();
#endif
#if AP_PERIPH_RPM_STREAM_ENABLED
        rpm_sensor_send();
#endif
    }
    const uint32_t now_us = AP_HAL::micros();
    while ((AP_HAL::micros() - now_us) < 1000) {
        hal.scheduler->delay_microseconds(HAL_PERIPH_LOOP_DELAY_US);

#if HAL_CANFD_SUPPORTED
        // allow for user enabling/disabling CANFD at runtime
        dronecan.canard.tao_disabled = g.can_fdmode == 1;
#endif
        {
            WITH_SEMAPHORE(canard_broadcast_semaphore);
            processTx();
            processRx();
#if HAL_PERIPH_CAN_MIRROR
            processMirror();
#endif // HAL_PERIPH_CAN_MIRROR

        }
    }

    // if there is a reboot scheduled, do it 1 second after request to allow the acknowledgement to be sent
    if (reboot_request_ms != 0 && (AP_HAL::millis() - reboot_request_ms > 1000)) {
        prepare_reboot();
#if CONFIG_HAL_BOARD == HAL_BOARD_CHIBIOS
        NVIC_SystemReset();
#elif CONFIG_HAL_BOARD == HAL_BOARD_SITL
        HAL_SITL::actually_reboot();
#endif
    }
}

// printf to CAN LogMessage for debugging
void can_vprintf(uint8_t severity, const char *fmt, va_list ap)
{
    // map MAVLink levels to CAN levels
    uint8_t level = UAVCAN_PROTOCOL_DEBUG_LOGLEVEL_DEBUG;
    switch (severity) {
    case MAV_SEVERITY_DEBUG:
        level = UAVCAN_PROTOCOL_DEBUG_LOGLEVEL_DEBUG;
        break;
    case MAV_SEVERITY_INFO:
        level = UAVCAN_PROTOCOL_DEBUG_LOGLEVEL_INFO;
        break;
    case MAV_SEVERITY_NOTICE:
    case MAV_SEVERITY_WARNING:
        level = UAVCAN_PROTOCOL_DEBUG_LOGLEVEL_WARNING;
        break;
    case MAV_SEVERITY_ERROR:
    case MAV_SEVERITY_CRITICAL:
    case MAV_SEVERITY_ALERT:
    case MAV_SEVERITY_EMERGENCY:
        level = UAVCAN_PROTOCOL_DEBUG_LOGLEVEL_ERROR;
        break;
    }

#if HAL_PERIPH_SUPPORT_LONG_CAN_PRINTF
    const uint8_t packet_count_max = 4; // how many packets we're willing to break up an over-sized string into
    const uint8_t packet_data_max = 90; // max single debug string length = sizeof(uavcan_protocol_debug_LogMessage.text.data)
    uint8_t buffer_data[packet_count_max*packet_data_max] {};

    // strip off any negative return errors by treating result as 0
    uint32_t char_count = MAX(vsnprintf((char*)buffer_data, sizeof(buffer_data), fmt, ap), 0);

    // send multiple uavcan_protocol_debug_LogMessage packets if the fmt string is too long.
    uint16_t buffer_offset = 0;
    for (uint8_t i=0; i<packet_count_max && char_count > 0; i++) {
        uavcan_protocol_debug_LogMessage pkt {};
        pkt.level.value = level;
        pkt.text.len = MIN(char_count, sizeof(pkt.text.data));
        char_count -= pkt.text.len;

        memcpy(pkt.text.data, &buffer_data[buffer_offset], pkt.text.len);
        buffer_offset += pkt.text.len;

        uint8_t buffer_packet[UAVCAN_PROTOCOL_DEBUG_LOGMESSAGE_MAX_SIZE];
        const uint32_t len = uavcan_protocol_debug_LogMessage_encode(&pkt, buffer_packet, !periph.canfdout());

        periph.canard_broadcast(UAVCAN_PROTOCOL_DEBUG_LOGMESSAGE_SIGNATURE,
                                UAVCAN_PROTOCOL_DEBUG_LOGMESSAGE_ID,
                                CANARD_TRANSFER_PRIORITY_LOW,
                                buffer_packet,
                                len);
    }
    
#else
    uavcan_protocol_debug_LogMessage pkt {};
    uint8_t buffer[UAVCAN_PROTOCOL_DEBUG_LOGMESSAGE_MAX_SIZE];
    uint32_t n = vsnprintf((char*)pkt.text.data, sizeof(pkt.text.data), fmt, ap);
    pkt.level.value = level;
    pkt.text.len = MIN(n, sizeof(pkt.text.data));

    uint32_t len = uavcan_protocol_debug_LogMessage_encode(&pkt, buffer, !periph.canfdout());

    periph.canard_broadcast(UAVCAN_PROTOCOL_DEBUG_LOGMESSAGE_SIGNATURE,
                            UAVCAN_PROTOCOL_DEBUG_LOGMESSAGE_ID,
                            CANARD_TRANSFER_PRIORITY_LOW,
                            buffer,
                            len);

#endif
}

// printf to CAN LogMessage for debugging, with severity
void can_printf_severity(uint8_t severity, const char *fmt, ...)
{
    va_list ap;
    va_start(ap, fmt);
    can_vprintf(severity, fmt, ap);
    va_end(ap);
}

// printf to CAN LogMessage for debugging, with DEBUG level
void can_printf(const char *fmt, ...)
{
    va_list ap;
    va_start(ap, fmt);
    can_vprintf(MAV_SEVERITY_DEBUG, fmt, ap);
    va_end(ap);
}
