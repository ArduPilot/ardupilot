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

#ifndef HAL_PERIPH_SUPPORT_LONG_CAN_PRINTF
    // When enabled, can_printf() strings longer than the droneCAN max text length (90 chars)
    // are split into multiple packets instead of truncating the string. This is
    // especially helpful with HAL_GCS_ENABLED where libraries use the mavlink
    // send_text() method where we support strings up to 256 chars by splitting them
    // up into multiple 50 char mavlink packets.
    #define HAL_PERIPH_SUPPORT_LONG_CAN_PRINTF (BOARD_FLASH_SIZE >= 1024)
#endif

static struct instance_t {
    uint8_t index;

#if CONFIG_HAL_BOARD == HAL_BOARD_CHIBIOS
    AP_HAL::CANIface* iface;
#elif CONFIG_HAL_BOARD == HAL_BOARD_SITL
    HALSITL::CANIface* iface;
#endif
} instances[HAL_NUM_CAN_IFACES];

#ifndef HAL_CAN_DEFAULT_NODE_ID
#define HAL_CAN_DEFAULT_NODE_ID CANARD_BROADCAST_NODE_ID
#endif
uint8_t PreferredNodeID = HAL_CAN_DEFAULT_NODE_ID;

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
void AP_Periph_DroneCAN::handle_get_node_info(const CanardRxTransfer& transfer, const uavcan_protocol_GetNodeInfoRequest &req)
{
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

    if (periph.g.serial_number > 0) {
        hal.util->snprintf((char*)pkt.name.data, sizeof(pkt.name.data), "%s(%u)", CAN_APP_NODE_NAME, (unsigned)periph.g.serial_number);
    } else {
        hal.util->snprintf((char*)pkt.name.data, sizeof(pkt.name.data), "%s", CAN_APP_NODE_NAME);
    }
    pkt.name.len = strnlen((char*)pkt.name.data, sizeof(pkt.name.data));

    periph.dronecan->get_node_info_server.respond(transfer, pkt);
}

/*
  handle parameter GetSet request
 */
void AP_Periph_DroneCAN::handle_param_getset(const CanardRxTransfer& transfer, const uavcan_protocol_param_GetSetRequest &req)
{
    // param fetch all can take a long time, so pat watchdog
    stm32_watchdog_pat();

    uavcan_protocol_param_GetSetResponse pkt {};

    AP_Param *vp;
    enum ap_var_type ptype;

    if (req.name.len != 0 && req.name.len > AP_MAX_NAME_SIZE) {
        vp = nullptr;
    } else if (req.name.len != 0 && req.name.len <= AP_MAX_NAME_SIZE) {
        memcpy((char *)pkt.name.data, (char *)req.name.data, req.name.len);
        vp = AP_Param::find((char *)pkt.name.data, &ptype);
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
    periph.dronecan->param_getset_server.respond(transfer, pkt);
}

/*
  handle parameter executeopcode request
 */
void AP_Periph_DroneCAN::handle_param_executeopcode(const CanardRxTransfer& transfer, const uavcan_protocol_param_ExecuteOpcodeRequest &req)
{
    if (req.opcode == UAVCAN_PROTOCOL_PARAM_EXECUTEOPCODE_REQUEST_OPCODE_ERASE) {
        StorageManager::erase();
        AP_Param::erase_all();
        AP_Param::load_all();
        AP_Param::setup_sketch_defaults();
#ifdef HAL_PERIPH_ENABLE_GPS
        AP_Param::setup_object_defaults(&periph.gps, periph.gps.var_info);
#endif
#ifdef HAL_PERIPH_ENABLE_BATTERY
        AP_Param::setup_object_defaults(&periph.battery, periph.battery_lib.var_info);
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

    periph.dronecan->param_executeopcode_server.respond(transfer, pkt);
}

void AP_Periph_DroneCAN::handle_begin_firmware_update(const CanardRxTransfer& transfer, const uavcan_protocol_file_BeginFirmwareUpdateRequest &req)
{
#if HAL_RAM_RESERVE_START >= 256
    // setup information on firmware request at start of ram
    auto *comms = (struct app_bootloader_comms *)HAL_RAM0_START;
    if (comms->magic != APP_BOOTLOADER_COMMS_MAGIC) {
        memset(comms, 0, sizeof(*comms));
    }
    comms->magic = APP_BOOTLOADER_COMMS_MAGIC;
    comms->server_node_id = req.source_node_id;
    if (comms->server_node_id == 0) {
        comms->server_node_id = transfer.source_node_id;
    }
    memset(comms->path, 0, sizeof(comms->path));
    memcpy(comms->path, req.image_file_remote_path.path.data, req.image_file_remote_path.path.len);
    comms->my_node_id = periph.dronecan->canard_iface.get_node_id();

    uavcan_protocol_file_BeginFirmwareUpdateResponse reply {};
    reply.error = UAVCAN_PROTOCOL_FILE_BEGINFIRMWAREUPDATE_RESPONSE_ERROR_OK;

    periph.dronecan->begin_firmware_update_server.respond(transfer, reply);
    uint8_t count = 50;
    while (count--) {
        periph.dronecan->canard_iface.processTx(false);
        hal.scheduler->delay(1);
    }
#endif

    // instant reboot, with backup register used to give bootloader
    // the node_id
    periph.prepare_reboot();
#if CONFIG_HAL_BOARD == HAL_BOARD_CHIBIOS
    set_fast_reboot((rtc_boot_magic)(RTC_BOOT_CANBL | periph.dronecan->canard_iface.get_node_id()));
    NVIC_SystemReset();
#endif
}

void AP_Periph_DroneCAN::handle_allocation_response(const CanardRxTransfer& transfer, const uavcan_protocol_dynamic_node_id_Allocation &msg)
{
    // Rule C - updating the randomized time interval
    periph.send_next_node_id_allocation_request_at_ms =
        AP_HAL::millis() + UAVCAN_PROTOCOL_DYNAMIC_NODE_ID_ALLOCATION_MIN_REQUEST_PERIOD_MS +
        get_random_range(UAVCAN_PROTOCOL_DYNAMIC_NODE_ID_ALLOCATION_MAX_FOLLOWUP_DELAY_MS);

    if (transfer.source_node_id == CANARD_BROADCAST_NODE_ID)
    {
        printf("Allocation request from another allocatee\n");
        periph.node_id_allocation_unique_id_offset = 0;
        return;
    }

    // Obtaining the local unique ID
    uint8_t my_unique_id[sizeof(msg.unique_id.data)];
    readUniqueID(my_unique_id);

    // Matching the received UID against the local one
    if (memcmp(msg.unique_id.data, my_unique_id, msg.unique_id.len) != 0) {
        printf("Mismatching allocation response\n");
        periph.node_id_allocation_unique_id_offset = 0;
        return;         // No match, return
    }

    if (msg.unique_id.len < sizeof(msg.unique_id.data)) {
        // The allocator has confirmed part of unique ID, switching to the next stage and updating the timeout.
        periph.node_id_allocation_unique_id_offset = msg.unique_id.len;
        periph.send_next_node_id_allocation_request_at_ms -= UAVCAN_PROTOCOL_DYNAMIC_NODE_ID_ALLOCATION_MIN_REQUEST_PERIOD_MS;

        printf("Matching allocation response: %d\n", msg.unique_id.len);
    } else {
        // Allocation complete - copying the allocated node ID from the message
        periph.dronecan->canard_iface.set_node_id(msg.node_id);
        printf("Node ID allocated: %d\n", msg.node_id);
    }
}


#if defined(HAL_GPIO_PIN_SAFE_LED) || defined(HAL_PERIPH_ENABLE_RC_OUT)
static uint8_t safety_state;

/*
  handle SafetyState
 */
void AP_Periph_DroneCAN::handle_safety_state(const CanardRxTransfer& transfer, const ardupilot_indication_SafetyState &req)
{
    safety_state = req.status;
#if AP_PERIPH_SAFETY_SWITCH_ENABLED
    periph.rcout_handle_safety_state(safety_state);
#endif
}
#endif // HAL_GPIO_PIN_SAFE_LED

/*
  handle ArmingStatus
 */
#if AP_PERIPH_HANDLE_ARMING_STATUS
void AP_Periph_DroneCAN::handle_arming_status(const CanardRxTransfer& transfer, const uavcan_equipment_safety_ArmingStatus &msg)
{
    hal.util->set_soft_armed(msg.status == UAVCAN_EQUIPMENT_SAFETY_ARMINGSTATUS_STATUS_FULLY_ARMED);
}
#endif

#if defined(AP_PERIPH_HAVE_LED_WITHOUT_NOTIFY) || defined(HAL_PERIPH_ENABLE_NOTIFY)
void AP_Periph_FW::set_rgb_led(uint8_t red, uint8_t green, uint8_t blue)
{
#ifdef HAL_PERIPH_ENABLE_NOTIFY
    notify.handle_rgb(red, green, blue);
#ifdef HAL_PERIPH_ENABLE_RC_OUT
    rcout_has_new_data_to_update = true;
#endif // HAL_PERIPH_ENABLE_RC_OUT
#endif // HAL_PERIPH_ENABLE_NOTIFY

#ifdef HAL_PERIPH_NEOPIXEL_COUNT_WITHOUT_NOTIFY
    hal.rcout->set_serial_led_rgb_data(HAL_PERIPH_NEOPIXEL_CHAN_WITHOUT_NOTIFY, -1, red, green, blue);
    hal.rcout->serial_led_send(HAL_PERIPH_NEOPIXEL_CHAN_WITHOUT_NOTIFY);
#endif // HAL_PERIPH_NEOPIXEL_COUNT_WITHOUT_NOTIFY

#ifdef HAL_PERIPH_ENABLE_NCP5623_LED_WITHOUT_NOTIFY
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
#endif // HAL_PERIPH_ENABLE_NCP5623_LED_WITHOUT_NOTIFY

#ifdef HAL_PERIPH_ENABLE_NCP5623_BGR_LED_WITHOUT_NOTIFY
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
#endif // HAL_PERIPH_ENABLE_NCP5623_BGR_LED_WITHOUT_NOTIFY
#ifdef HAL_PERIPH_ENABLE_TOSHIBA_LED_WITHOUT_NOTIFY
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
#endif // HAL_PERIPH_ENABLE_TOSHIBA_LED_WITHOUT_NOTIFY
}

/*
  handle lightscommand
 */
void AP_Periph_DroneCAN::handle_lightscommand(const CanardRxTransfer& transfer, const uavcan_equipment_indication_LightsCommand &req)
{
    for (uint8_t i=0; i<req.commands.len; i++) {
        const uavcan_equipment_indication_SingleLightCommand &cmd = req.commands.data[i];
        // to get the right color proportions we scale the green so that is uses the
        // same number of bits as red and blue
        uint8_t red = cmd.color.red<<3U;
        uint8_t green = (cmd.color.green>>1U)<<3U;
        uint8_t blue = cmd.color.blue<<3U;
#ifdef HAL_PERIPH_ENABLE_NOTIFY
        const int8_t brightness = periph.notify.get_rgb_led_brightness_percent();
#elif defined(AP_PERIPH_HAVE_LED_WITHOUT_NOTIFY)
        const int8_t brightness = periph.g.led_brightness;
#endif
        if (brightness != 100 && brightness >= 0) {
            const float scale = brightness * 0.01;
            red = constrain_int16(red * scale, 0, 255);
            green = constrain_int16(green * scale, 0, 255);
            blue = constrain_int16(blue * scale, 0, 255);
        }
        periph.set_rgb_led(red, green, blue);
    }
}
#endif // AP_PERIPH_HAVE_LED_WITHOUT_NOTIFY

#ifdef HAL_PERIPH_ENABLE_RC_OUT
void AP_Periph_DroneCAN::handle_esc_rawcommand(const CanardRxTransfer& transfer, const uavcan_equipment_esc_RawCommand &cmd)
{
    periph.rcout_esc(cmd.cmd.data, cmd.cmd.len);

    // Update internal copy for disabling output to ESC when CAN packets are lost
    periph.last_esc_num_channels = cmd.cmd.len;
    periph.last_esc_raw_command_ms = AP_HAL::millis();
}

void AP_Periph_DroneCAN::handle_act_command(const CanardRxTransfer& transfer, const uavcan_equipment_actuator_ArrayCommand &cmd)
{
    for (uint8_t i=0; i < cmd.commands.len; i++) {
        const auto &c = cmd.commands.data[i];
        switch (c.command_type) {
        case UAVCAN_EQUIPMENT_ACTUATOR_COMMAND_COMMAND_TYPE_UNITLESS:
            periph.rcout_srv_unitless(c.actuator_id, c.command_value);
            break;
        case UAVCAN_EQUIPMENT_ACTUATOR_COMMAND_COMMAND_TYPE_PWM:
            periph.rcout_srv_PWM(c.actuator_id, c.command_value);
            break;
        }
    }
}
#endif // HAL_PERIPH_ENABLE_RC_OUT

#if defined(HAL_PERIPH_ENABLE_NOTIFY)
void AP_Periph_DroneCAN::handle_notify_state(const CanardRxTransfer& transfer, const ardupilot_indication_NotifyState &msg)
{
    if (msg.aux_data.len == 2 && msg.aux_data_type == ARDUPILOT_INDICATION_NOTIFYSTATE_VEHICLE_YAW_EARTH_CENTIDEGREES) {
        uint16_t tmp = 0;
        memcpy(&tmp, msg.aux_data.data, sizeof(tmp));
        periph.yaw_earth = radians((float)tmp * 0.01f);
    }
    periph.vehicle_state = msg.vehicle_state;
    periph.last_vehicle_state = AP_HAL::millis();
}
#endif // HAL_PERIPH_ENABLE_NOTIFY

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

    dronecan->button_pub.broadcast(pkt);
}
#endif // HAL_GPIO_PIN_SAFE_BUTTON

void AP_Periph_DroneCAN::handle_restart_node(const CanardRxTransfer& transfer, const uavcan_protocol_RestartNodeRequest &req)
{
    uavcan_protocol_RestartNodeResponse resp {};
    resp.ok = true;
    printf("RestartNode\n");
    periph.dronecan->restart_node_server.respond(transfer, resp);
    periph.dronecan->canard_iface.process(10);
    periph.prepare_reboot();
#if CONFIG_HAL_BOARD == HAL_BOARD_CHIBIOS
    NVIC_SystemReset();
#endif
}

void AP_Periph_FW::node_status_send(void)
{
    {
        node_status.uptime_sec = AP_HAL::millis() / 1000U;

        node_status.vendor_specific_status_code = MIN(hal.util->available_memory(), unsigned(UINT16_MAX));
        dronecan->node_status_pub.broadcast(node_status);
    }
#if HAL_ENABLE_SENDING_STATS
    if (debug_option_is_set(AP_Periph_FW::DebugOptions::ENABLE_STATS)) {
        {
            dronecan->stats_pub.broadcast(protocol_stats);
        }
        for (auto &instance : instances) {
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
            dronecan->can_stats_pub.broadcast(can_stats);
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
     * Printing the memory usage statistics.
     */
    {
        /*
         * The recommended way to establish the minimal size of the memory pool is to stress-test the application and
         * record the worst case memory usage.
         */

        if (dronecan->canard_iface.pool_peak_percent() > 70) {
            printf("WARNING: ENLARGE MEMORY POOL\n");
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
    if (dronecan->canard_iface.get_node_id() != CANARD_BROADCAST_NODE_ID) {
        AP_Periph_FW::no_iface_finished_dna = false;
        return true;
    }

    const uint32_t now = AP_HAL::millis();

    if (AP_Periph_FW::no_iface_finished_dna) {
        printf("Waiting for dynamic node ID allocation... (pool %u)\n", dronecan->canard_iface.pool_peak_percent());
    }

    send_next_node_id_allocation_request_at_ms =
        now + UAVCAN_PROTOCOL_DYNAMIC_NODE_ID_ALLOCATION_MIN_REQUEST_PERIOD_MS +
        get_random_range(UAVCAN_PROTOCOL_DYNAMIC_NODE_ID_ALLOCATION_MAX_FOLLOWUP_DELAY_MS);

    // Structure of the request is documented in the DSDL definition
    // See http://uavcan.org/Specification/6._Application_level_functions/#dynamic-node-id-allocation
    uavcan_protocol_dynamic_node_id_Allocation allocation_request {};

    allocation_request.node_id = PreferredNodeID;
    if (node_id_allocation_unique_id_offset == 0) {
        allocation_request.first_part_of_unique_id = true;
    }

    uint8_t my_unique_id[sizeof(uavcan_protocol_dynamic_node_id_Allocation::unique_id.data)] {};
    readUniqueID(my_unique_id);

    static const uint8_t MaxLenOfUniqueIDInRequest = 6;
    uint8_t uid_size = (uint8_t)(sizeof(uavcan_protocol_dynamic_node_id_Allocation::unique_id.data) - node_id_allocation_unique_id_offset);
    
    if (uid_size > MaxLenOfUniqueIDInRequest) {
        uid_size = MaxLenOfUniqueIDInRequest;
    }

    memmove(&allocation_request.unique_id.data, &my_unique_id[node_id_allocation_unique_id_offset], uid_size);
    allocation_request.unique_id.len = uid_size;

    // Broadcasting the request
    dronecan->dynamic_node_id_pub.broadcast(allocation_request);

    // Preparing for timeout; if response is received, this value will be updated from the callback.
    node_id_allocation_unique_id_offset = 0;
    return false;
}

void AP_Periph_FW::can_start()
{
    node_status.health = UAVCAN_PROTOCOL_NODESTATUS_HEALTH_OK;
    node_status.mode = UAVCAN_PROTOCOL_NODESTATUS_MODE_INITIALIZATION;
    node_status.uptime_sec = AP_HAL::millis() / 1000U;

    if (g.can_node >= 0 && g.can_node < 128) {
        PreferredNodeID = g.can_node;
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
        g.can_protocol[0].set_and_save(uint8_t(AP_CAN::Protocol::DroneCAN));
        g.can_baudrate[0].set_and_save(1000000);
    }
#endif // HAL_PERIPH_ENFORCE_AT_LEAST_ONE_PORT_IS_UAVCAN_1MHz

    dronecan = new AP_Periph_DroneCAN();

    if (dronecan == nullptr) {
        AP_HAL::panic("Failed to allocate dronecan");
    }

#ifdef HAL_GPIO_PIN_GPIO_CAN1_TERM
    palWriteLine(HAL_GPIO_PIN_GPIO_CAN1_TERM, g.can_terminate[0]);
#endif
#ifdef HAL_GPIO_PIN_GPIO_CAN2_TERM
    palWriteLine(HAL_GPIO_PIN_GPIO_CAN2_TERM, g.can_terminate[1]);
#endif
#ifdef HAL_GPIO_PIN_GPIO_CAN3_TERM
    palWriteLine(HAL_GPIO_PIN_GPIO_CAN3_TERM, g.can_terminate[2]);
#endif

    for (uint8_t i=0; i<HAL_NUM_CAN_IFACES; i++) {
#if CONFIG_HAL_BOARD == HAL_BOARD_CHIBIOS
        can_iface_periph[i] = new ChibiOS::CANIface();
#elif CONFIG_HAL_BOARD == HAL_BOARD_SITL
        can_iface_periph[i] = new HALSITL::CANIface();
#endif
        instances[i].iface = can_iface_periph[i];
        instances[i].index = i;
#if HAL_NUM_CAN_IFACES >= 2
        can_protocol_cached[i] = g.can_protocol[i];
        CANSensor::set_periph(i, can_protocol_cached[i], can_iface_periph[i]);
#endif
        if (can_iface_periph[i] != nullptr) {
#if HAL_CANFD_SUPPORTED
            can_iface_periph[i]->init(g.can_baudrate[i], g.can_fdbaudrate[i]*1000000U, AP_HAL::CANIface::NormalMode);
#else
            can_iface_periph[i]->init(g.can_baudrate[i], AP_HAL::CANIface::NormalMode);
#endif
            dronecan->canard_iface.add_interface(can_iface_periph[i]);
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
}

AP_Periph_DroneCAN::AP_Periph_DroneCAN()
{
    canard_iface.init(canard_memory_pool, sizeof(canard_memory_pool), PreferredNodeID);

    // setup message timeouts and priorities
    dynamic_node_id_pub.set_priority(CANARD_TRANSFER_PRIORITY_MEDIUM);
    dynamic_node_id_pub.set_timeout_ms(500);

    node_status_pub.set_priority(CANARD_TRANSFER_PRIORITY_LOW);
    node_status_pub.set_timeout_ms(1000);

#if HAL_ENABLE_SENDING_STATS
    stats_pub.set_priority(CANARD_TRANSFER_PRIORITY_LOW);
    stats_pub.set_timeout_ms(1000);

    can_stats_pub.set_priority(CANARD_TRANSFER_PRIORITY_LOW);
    can_stats_pub.set_timeout_ms(1000);
#endif

    mag_pub.set_priority(CANARD_TRANSFER_PRIORITY_MEDIUM);
    mag_pub.set_timeout_ms(10);

#ifdef HAL_PERIPH_ENABLE_GPS
    gnss_fix2_pub.set_priority(CANARD_TRANSFER_PRIORITY_MEDIUM);
    gnss_fix2_pub.set_timeout_ms(50);

    gnss_aux_pub.set_priority(CANARD_TRANSFER_PRIORITY_MEDIUM);
    gnss_aux_pub.set_timeout_ms(50);

    gnss_status_pub.set_priority(CANARD_TRANSFER_PRIORITY_MEDIUM);
    gnss_status_pub.set_timeout_ms(50);
#if GPS_MOVING_BASELINE
    moving_baseline_pub.set_priority(CANARD_TRANSFER_PRIORITY_MEDIUM);
    moving_baseline_pub.set_timeout_ms(50);

    relposheading_pub.set_priority(CANARD_TRANSFER_PRIORITY_MEDIUM);
    relposheading_pub.set_timeout_ms(50);
#endif
#endif

#ifdef HAL_PERIPH_ENABLE_BARO
    static_pressure_pub.set_priority(CANARD_TRANSFER_PRIORITY_MEDIUM);
    static_pressure_pub.set_timeout_ms(20);

    static_temperature_pub.set_priority(CANARD_TRANSFER_PRIORITY_MEDIUM);
    static_temperature_pub.set_timeout_ms(20);
#endif

    log_pub.set_priority(CANARD_TRANSFER_PRIORITY_LOWEST);
    log_pub.set_timeout_ms(10);

#if AP_UART_MONITOR_ENABLED
    tunnel_pub.set_priority(CANARD_TRANSFER_PRIORITY_HIGH);
    tunnel_pub.set_timeout_ms(5);
#endif

#ifdef HAL_GPIO_PIN_SAFE_BUTTON
    button_pub.set_priority(CANARD_TRANSFER_PRIORITY_LOW);
    button_pub.set_timeout_ms(200);
#endif

#if defined(HAL_PERIPH_ENABLE_HWESC) || HAL_WITH_ESC_TELEM
    esc_status_pub.set_priority(CANARD_TRANSFER_PRIORITY_LOW);
    esc_status_pub.set_timeout_ms(200);
#endif

#ifdef HAL_PERIPH_ENABLE_ADSB
    traffic_report_pub.set_priority(CANARD_TRANSFER_PRIORITY_LOWEST);
    traffic_report_pub.set_timeout_ms(200);
#endif

#ifdef HAL_PERIPH_ENABLE_AIRSPEED
    raw_air_data_pub.set_priority(CANARD_TRANSFER_PRIORITY_MEDIUM);
    raw_air_data_pub.set_timeout_ms(50);
#endif

#if defined(HAL_PERIPH_ENABLE_BATTERY) || defined(HAL_PERIPH_ENABLE_BATTERY_BALANCE)
    battery_info_aux_pub.set_priority(CANARD_TRANSFER_PRIORITY_LOW);
    battery_info_aux_pub.set_timeout_ms(200);

    battery_info_pub.set_priority(CANARD_TRANSFER_PRIORITY_LOW);
    battery_info_pub.set_timeout_ms(200);
#endif

#ifdef HAL_PERIPH_ENABLE_EFI
    reciprocating_engine_status_pub.set_priority(CANARD_TRANSFER_PRIORITY_LOW);
    reciprocating_engine_status_pub.set_timeout_ms(200);
#endif

#ifdef HAL_PERIPH_ENABLE_PWM_HARDPOINT
    hardpoint_command_pub.set_priority(CANARD_TRANSFER_PRIORITY_LOW);
    hardpoint_command_pub.set_timeout_ms(200);
#endif

#ifdef HAL_PERIPH_ENABLE_PROXIMITY
    proximity_pub.set_priority(CANARD_TRANSFER_PRIORITY_LOW);
    proximity_pub.set_timeout_ms(200);
#endif

#ifdef HAL_PERIPH_ENABLE_RANGEFINDER
    range_sensor_measurement_pub.set_priority(CANARD_TRANSFER_PRIORITY_LOW);
    range_sensor_measurement_pub.set_timeout_ms(200);
#endif

#ifdef HAL_PERIPH_ENABLE_RCIN
    rc_input_pub.set_priority(CANARD_TRANSFER_PRIORITY_LOW);
    rc_input_pub.set_timeout_ms(200);
#endif

    param_getset_server.set_timeout_ms(200);
    param_executeopcode_server.set_timeout_ms(200);
    begin_firmware_update_server.set_timeout_ms(200);
    restart_node_server.set_timeout_ms(200);
}

#ifdef HAL_PERIPH_ENABLE_RC_OUT
#if HAL_WITH_ESC_TELEM
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
        const auto *channel = SRV_Channels::srv_channel(i);
        // try to map the ESC number to a motor number. This is needed
        // for when we have multiple CAN nodes, one for each ESC
        if (channel == nullptr) {
            pkt.esc_index = i;
        } else {
            const int8_t motor_num = channel->get_motor_num();
            pkt.esc_index = motor_num==-1? i:motor_num;
        }
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
        if (esc_telem.get_raw_rpm(i, rpm)) {
            pkt.rpm = rpm;
        }
        pkt.power_rating_pct = 0;
        pkt.error_count = 0;

        dronecan->esc_status_pub.broadcast(pkt);
    }
}
#endif // HAL_WITH_ESC_TELEM

#ifdef HAL_PERIPH_ENABLE_ESC_APD
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

            uint8_t buffer[UAVCAN_EQUIPMENT_ESC_STATUS_MAX_SIZE] {};
            uint16_t total_size = uavcan_equipment_esc_Status_encode(&pkt, buffer, !canfdout());
            canard_broadcast(UAVCAN_EQUIPMENT_ESC_STATUS_SIGNATURE,
                            UAVCAN_EQUIPMENT_ESC_STATUS_ID,
                            CANARD_TRANSFER_PRIORITY_LOW,
                            &buffer[0],
                            total_size);
                }
    }

}
#endif // HAL_PERIPH_ENABLE_ESC_APD
#endif // HAL_PERIPH_ENABLE_RC_OUT

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

    if (AP_HAL::millis() > send_next_node_id_allocation_request_at_ms) {
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
#ifdef HAL_PERIPH_ENABLE_MAG
        can_mag_update();
#endif
#ifdef HAL_PERIPH_ENABLE_GPS
        can_gps_update();
#endif
#if AP_UART_MONITOR_ENABLED
        send_serial_monitor_data();
#endif
#ifdef HAL_PERIPH_ENABLE_BATTERY
        can_battery_update();
#endif
#ifdef HAL_PERIPH_ENABLE_BARO
        can_baro_update();
#endif
#ifdef HAL_PERIPH_ENABLE_AIRSPEED
        can_airspeed_update();
#endif
#ifdef HAL_PERIPH_ENABLE_RANGEFINDER
        can_rangefinder_update();
#endif
#ifdef HAL_PERIPH_ENABLE_PROXIMITY
        can_proximity_update();
#endif
    #if defined(HAL_PERIPH_ENABLE_BUZZER_WITHOUT_NOTIFY) || defined (HAL_PERIPH_ENABLE_NOTIFY)
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
#ifdef HAL_PERIPH_ENABLE_ESC_APD
        apd_esc_telem_update();
#endif
    #ifdef HAL_PERIPH_ENABLE_MSP
        msp_sensor_update();
    #endif
    #ifdef HAL_PERIPH_ENABLE_RC_OUT
        rcout_update();
    #endif
    #ifdef HAL_PERIPH_ENABLE_EFI
        can_efi_update();
    #endif
    }
    dronecan->canard_iface.process(1);
}

// printf to CAN LogMessage for debugging
void can_printf(const char *fmt, ...)
{
#if HAL_PERIPH_SUPPORT_LONG_CAN_PRINTF
    const uint8_t packet_count_max = 4; // how many packets we're willing to break up an over-sized string into
    const uint8_t packet_data_max = 90; // max single debug string length = sizeof(uavcan_protocol_debug_LogMessage.text.data)
    uint8_t buffer_data[packet_count_max*packet_data_max] {};

    va_list ap;
    va_start(ap, fmt);
    // strip off any negative return errors by treating result as 0
    uint32_t char_count = MAX(vsnprintf((char*)buffer_data, sizeof(buffer_data), fmt, ap), 0);
    va_end(ap);

    // send multiple uavcan_protocol_debug_LogMessage packets if the fmt string is too long.
    uint16_t buffer_offset = 0;
    for (uint8_t i=0; i<packet_count_max && char_count > 0; i++) {
        uavcan_protocol_debug_LogMessage pkt {};
        pkt.text.len = MIN(char_count, sizeof(pkt.text.data));
        char_count -= pkt.text.len;

        memcpy(pkt.text.data, &buffer_data[buffer_offset], pkt.text.len);
        buffer_offset += pkt.text.len;
        periph.dronecan->log_pub.broadcast(pkt);
    }
    
#else
    uavcan_protocol_debug_LogMessage pkt {};
    va_list ap;
    va_start(ap, fmt);
    uint32_t n = vsnprintf((char*)pkt.text.data, sizeof(pkt.text.data), fmt, ap);
    va_end(ap);
    pkt.text.len = MIN(n, sizeof(pkt.text.data));

    periph.dronecan->log_pub.broadcast(pkt);
#endif
}
