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

#include "AP_ESC_Telem.h"

#if HAL_WITH_ESC_TELEM

#include <AP_HAL/AP_HAL.h>
#include <GCS_MAVLink/GCS.h>
#include <AP_Logger/AP_Logger.h>
#include <AP_BoardConfig/AP_BoardConfig.h>
#include <AP_TemperatureSensor/AP_TemperatureSensor_config.h>

#include <AP_Math/AP_Math.h>
#include <AP_SerialManager/AP_SerialManager.h>

#include "AP_HobbyWing_ESC.h"
#include "AP_HobbyWing_ESC_Platinum_PRO_v3.h"
#include "AP_HobbyWing_ESC_Platinum_v4.h"
#include "AP_HobbyWing_ESC_XRotor_v4.h"
#include "AP_HobbyWing_DataLink.h"

//#define ESC_TELEM_DEBUG

#define ESC_RPM_CHECK_TIMEOUT_US 210000UL   // timeout for motor running validity

extern const AP_HAL::HAL& hal;

// table of user settable parameters
#if AP_HOBBYWING_ESC_ENABLED
const AP_Param::GroupInfo AP_ESC_Telem_MotorGroup::var_info[] = {
    AP_GROUPINFO_FLAGS("_PROT", 1, AP_ESC_Telem_MotorGroup, protocol, 0, AP_PARAM_FLAG_ENABLE),

    AP_GROUPINFO("_POLE", 2, AP_ESC_Telem_MotorGroup, poles, 14),

    AP_GROUPINFO("_MASK", 3, AP_ESC_Telem_MotorGroup, mask, 0),

    AP_GROUPEND
};
#endif  // AP_HOBBYWING_ESC_ENABLED

// table of user settable parameters
const AP_Param::GroupInfo AP_ESC_Telem::var_info[] = {

    // @Param: _MAV_OFS
    // @DisplayName: ESC Telemetry mavlink offset
    // @Description: Offset to apply to ESC numbers when reporting as ESC_TELEMETRY packets over MAVLink. This allows high numbered motors to be displayed as low numbered ESCs for convenience on GCS displays. A value of 4 would send ESC on output 5 as ESC number 1 in ESC_TELEMETRY packets
    // @Increment: 1
    // @Range: 0 31
    // @User: Standard
    AP_GROUPINFO("_MAV_OFS", 1, AP_ESC_Telem, mavlink_offset, 0),

#if AP_ESC_TELEM_MAX_PROTOCOL_GROUPS > 0
    // @Param: _G1_PROT
    // @DisplayName: ESC telemetry protocol for Motor Group1
    // @Description: ESC telemetry protocol used for this motor group
    // @Values: 0:None, 1:HobbyWing Platinum Pro v3, 2:HobbyWing Platinum v4, 3: HobbyWing XRotorv4
    // @Increment: 1
    // @User: Standard

    // @Param: _G1_POLE
    // @DisplayName: Number of Poles for Motor Group1
    // @Description: When converting from ESC telemetry stream, assume this number of motor poles in connected motors (this is an eRPM to RPM conversion)
    // @Increment: 1
    // @User: Standard

    // @Param: _G1_MASK
    // @DisplayName: Mask of which motor outputs correspond to configured telemetry inputs.  For each bit set in this mask there must be a serial port configured with the matching protocol.
    // @Description: ESC telemetry protocol used for this motor group
    // @Values: 0:None, 1:HobbyWing Platinum Pro v3, 2:HobbyWing Platinum v4, 3: HobbyWing XRotorv4
    // @Increment: 1
    // @User: Standard

    AP_SUBGROUPINFO(motor_group[0], "_G1", 2, AP_ESC_Telem, AP_ESC_Telem_MotorGroup),
#endif  // AP_ESC_TELEM_MAX_PROTOCOL_GROUPS > 0

#if AP_ESC_TELEM_MAX_PROTOCOL_GROUPS > 1
    // @Param: _G2_PROT
    // @CopyFieldsFrom: ESC_TLM_G1_PROT
    // @DisplayName: ESC telemetry protocol for Motor Group1

    // @Param: _G2_POLE
    // @CopyFieldsFrom: ESC_TLM_G1_POLE
    // @DisplayName: Number of Poles for Motor Group1

    // @Param: _G2_MASK
    // @CopyFieldsFrom: ESC_TLM_G1_MASK
    AP_SUBGROUPINFO(motor_group[1], "_G2", 3, AP_ESC_Telem, AP_ESC_Telem_MotorGroup),
#endif  // AP_ESC_TELEM_MAX_PROTOCOL_GROUPS > 1

#if AP_HOBBYWING_DATALINK_ENABLED
    // @Group: _DL
    // @Path: AP_HobbyWing_DataLink.cpp
    AP_SUBGROUPINFO(datalink, "_DL", 10, AP_ESC_Telem, AP_HobbyWing_DataLink),
#endif

    AP_GROUPEND
};

AP_ESC_Telem::AP_ESC_Telem()
{
    if (_singleton) {
        AP_HAL::panic("Too many AP_ESC_Telem instances");
    }
    _singleton = this;
#if !defined(IOMCU_FW)
    AP_Param::setup_object_defaults(this, var_info);
#endif
}

AP_SerialManager::SerialProtocol AP_ESC_Telem::serial_protocol_for_esc_telem_protocol(AP_ESC_Telem_Protocol protocol)
{
    switch (protocol) {
    case AP_ESC_Telem_Protocol::HOBBYWING_PLATINUM_PRO_V3:
        return AP_SerialManager::SerialProtocol::SerialProtocol_HobbyWing_PlatinumProV3;
    case AP_ESC_Telem_Protocol::HOBBYWING_PLATINUM_V4:
        return AP_SerialManager::SerialProtocol::SerialProtocol_HobbyWing_PlatinumV4;
    case AP_ESC_Telem_Protocol::HOBBYWING_XROTOR_V4:
        return AP_SerialManager::SerialProtocol::SerialProtocol_HobbyWing_XRotorV4;
    default:
        // this is a user-supplied parameter, so could be anything:
        break;
    }

    return AP_SerialManager::SerialProtocol::SerialProtocol_None;
}

AP_HobbyWing_ESC *AP_ESC_Telem::new_esc_backend_for_type(AP_ESC_Telem_Protocol protocol, AP_HAL::UARTDriver &uart, uint8_t servo_channel, uint8_t poles)
{
    switch (protocol) {
#if AP_HOBBYWING_PLATINUM_PRO_V3_ENABLED
    case AP_ESC_Telem_Protocol::HOBBYWING_PLATINUM_PRO_V3:
        return new AP_HobbyWing_Platinum_PRO_v3(uart, servo_channel, poles);
#endif
#if AP_HOBBYWING_PLATINUM_V4_ENABLED
    case AP_ESC_Telem_Protocol::HOBBYWING_PLATINUM_V4:
        return new AP_HobbyWing_Platinum_v4(uart, servo_channel, poles);
#endif
#if AP_HOBBYWING_XROTOR_V4_ENABLED
    case AP_ESC_Telem_Protocol::HOBBYWING_XROTOR_V4:
        return new AP_HobbyWing_XRotor_v4(uart, servo_channel, poles);
#endif
    default:
        break;
    }

    return nullptr;
}


void AP_ESC_Telem::init()
{
#if AP_HOBBYWING_ESC_ENABLED
    for (auto &group : motor_group) {
        if (group.protocol == AP_ESC_Telem_Protocol::NONE) {
            continue;
        }
        for (uint8_t i=0; i<32; i++) {  // n.b. 32 is ambitious given old mavlink transfer protocol
            if (!BIT_IS_SET(group.mask, i)) {
                continue;
            }

            // find a UART for this group protocol:
            const auto serial_protocol = serial_protocol_for_esc_telem_protocol(group.protocol);
            if (serial_protocol == AP_SerialManager::SerialProtocol::SerialProtocol_None) {
                break;
            }

            // note we rely on group.protocol being constrained by the
            // return of a valid serial protocol!  If that goes wrong
            // this will lead to an out-of-bounds array access here:
            const uint8_t group_protocol = group.protocol;
            AP_HAL::UARTDriver *uart { AP::serialmanager().find_serial(serial_protocol, num_escs_for_protocol[group_protocol]) };
            if (uart == nullptr) {
                break;
            }

            // instantiate a backend for the type:
            escs[num_escs] = new_esc_backend_for_type(group.protocol, *uart, i+1, group.poles);
            if (escs[num_escs] == nullptr) {
                break;
            }

            num_escs_for_protocol[group_protocol]++;
            num_escs++;
        }
    }

#if AP_HOBBYWING_DATALINK_ENABLED
    {
        AP_HAL::UARTDriver *uart { AP::serialmanager().find_serial(AP_SerialManager::SerialProtocol_HobbyWing_DataLink, 0) };
        if (uart != nullptr) {
            datalink = new AP_HobbyWing_DataLink(*uart);
        }
    }
#endif

    bool create_thread = false;

    if (num_escs != 0) {
        create_thread = true;
    }

#if AP_HOBBYWING_DATALINK_ENABLED
    if (datalink != nullptr) {
        create_thread = true;
    }
#endif

    if (!create_thread) {
        return;
    }

    // start a thread to handle the reading from all of those UARTs:
    if (!hal.scheduler->thread_create(
            FUNCTOR_BIND_MEMBER(&AP_ESC_Telem::thread_main, void),
            "HobbyWing",
            512, AP_HAL::Scheduler::PRIORITY_BOOST, 1)) {
        DEV_PRINTF("Failed to create HobbyWing thread\n");
    }
#endif // AP_HOBBYWING_ESC_ENABLED
}

#if AP_HOBBYWING_ESC_ENABLED
void AP_ESC_Telem::thread_main()
{
    // initialise all escs; this will set the UART up in the thread
    for (uint8_t i=0; i<num_escs; i++) {
        escs[i]->init();
    }
#if AP_HOBBYWING_DATALINK_ENABLED
    if (datalink != nullptr) {
        datalink->init();
    }
#endif

    while (true) {
        hal.scheduler->delay_microseconds(1500);
        for (uint8_t i=0; i<num_escs; i++) {
            escs[i]->update();
        }
#if AP_HOBBYWING_DATALINK_ENABLED
        if (datalink != nullptr) {
            datalink->update();
        }
#endif
    }
}
#endif  // AP_HOBBYWING_ESC_ENABLED

void AP_ESC_Telem::update_telemetry()
{
#if AP_HOBBYWING_ESC_ENABLED
    for (uint8_t i=0; i<num_escs; i++) {
        AP_HobbyWing_ESC &esc = *escs[i];
        esc.update_telemetry();
    }

#if AP_HOBBYWING_DATALINK_ENABLED
    if (datalink != nullptr) {
        datalink->update_telemetry();
    }
#endif  // AP_HOBBYWING_DATALINK_ENABLED
#endif  // AP_HOBBYWING_ESC_ENABLED
}

// return the average motor RPM
float AP_ESC_Telem::get_average_motor_rpm(uint32_t servo_channel_mask) const
{
    float rpm_avg = 0.0f;
    uint8_t valid_escs = 0;

    // average the rpm of each motor
    for (uint8_t i = 0; i < ESC_TELEM_MAX_ESCS; i++) {
        if (BIT_IS_SET(servo_channel_mask,i)) {
            float rpm;
            if (get_rpm(i, rpm)) {
                rpm_avg += rpm;
                valid_escs++;
            }
        }
    }

    if (valid_escs > 0) {
        rpm_avg /= valid_escs;
    }

    return rpm_avg;
}

// return all the motor frequencies in Hz for dynamic filtering
uint8_t AP_ESC_Telem::get_motor_frequencies_hz(uint8_t nfreqs, float* freqs) const
{
    uint8_t valid_escs = 0;

    // average the rpm of each motor as reported by BLHeli and convert to Hz
    for (uint8_t i = 0; i < ESC_TELEM_MAX_ESCS && valid_escs < nfreqs; i++) {
        float rpm;
        if (get_rpm(i, rpm)) {
            freqs[valid_escs++] = rpm * (1.0f / 60.0f);
        } else if (was_rpm_data_ever_reported(_rpm_data[i])) {
            // if we have ever received data on an ESC, mark it as valid but with no data
            // this prevents large frequency shifts when ESCs disappear
            freqs[valid_escs++] = 0.0f;
        }
    }

    return MIN(valid_escs, nfreqs);
}

// get mask of ESCs that sent valid telemetry and/or rpm data in the last
// ESC_TELEM_DATA_TIMEOUT_MS/ESC_RPM_DATA_TIMEOUT_US
uint32_t AP_ESC_Telem::get_active_esc_mask() const {
    uint32_t ret = 0;
    const uint32_t now = AP_HAL::millis();
    uint32_t now_us = AP_HAL::micros();
    for (uint8_t i = 0; i < ESC_TELEM_MAX_ESCS; i++) {
        if (_telem_data[i].last_update_ms == 0 && !was_rpm_data_ever_reported(_rpm_data[i])) {
            // have never seen telem from this ESC
            continue;
        }
        if (_telem_data[i].stale(now)
            && !rpm_data_within_timeout(_rpm_data[i], now_us, ESC_RPM_DATA_TIMEOUT_US)) {
            continue;
        }
        ret |= (1U << i);
    }
    return ret;
}

// return number of active ESCs present
uint8_t AP_ESC_Telem::get_num_active_escs() const {
    uint32_t active = get_active_esc_mask();
    return __builtin_popcount(active);
}

// return the whether all the motors in servo_channel_mask are running
bool AP_ESC_Telem::are_motors_running(uint32_t servo_channel_mask, float min_rpm, float max_rpm) const
{
    const uint32_t now = AP_HAL::micros();

    for (uint8_t i = 0; i < ESC_TELEM_MAX_ESCS; i++) {
        if (BIT_IS_SET(servo_channel_mask, i)) {
            const volatile AP_ESC_Telem_Backend::RpmData& rpmdata = _rpm_data[i];
            // we choose a relatively strict measure of health so that failsafe actions can rely on the results
            if (!rpm_data_within_timeout(rpmdata, now, ESC_RPM_CHECK_TIMEOUT_US)) {
                return false;
            }
            if (rpmdata.rpm < min_rpm) {
                return false;
            }
            if ((max_rpm > 0) && (rpmdata.rpm > max_rpm)) {
                return false;
            }
        }
    }
    return true;
}

// is telemetry active for the provided channel mask
bool AP_ESC_Telem::is_telemetry_active(uint32_t servo_channel_mask) const
{
    for (uint8_t i = 0; i < ESC_TELEM_MAX_ESCS; i++) {
        if (BIT_IS_SET(servo_channel_mask, i)) {
            // no data received
            if (get_last_telem_data_ms(i) == 0 && !was_rpm_data_ever_reported(_rpm_data[i])) {
                return false;
            }
        }
    }
    return true;
}

// get an individual ESC's slewed rpm if available, returns true on success
bool AP_ESC_Telem::get_rpm(uint8_t esc_index, float& rpm) const
{
    if (esc_index >= ESC_TELEM_MAX_ESCS) {
        return false;
    }

    const volatile AP_ESC_Telem_Backend::RpmData& rpmdata = _rpm_data[esc_index];

    if (is_zero(rpmdata.update_rate_hz)) {
        return false;
    }

    const uint32_t now = AP_HAL::micros();
    if (rpm_data_within_timeout(rpmdata, now, ESC_RPM_DATA_TIMEOUT_US)) {
        const float slew = MIN(1.0f, (now - rpmdata.last_update_us) * rpmdata.update_rate_hz * (1.0f / 1e6f));
        rpm = (rpmdata.prev_rpm + (rpmdata.rpm - rpmdata.prev_rpm) * slew);

#if AP_SCRIPTING_ENABLED
        if ((1U<<esc_index) & rpm_scale_mask) {
            rpm *= rpm_scale_factor[esc_index];
        }
#endif

        return true;
    }
    return false;
}

// get an individual ESC's unfiltered rpm if available, returns true on success
bool AP_ESC_Telem::get_raw_rpm(uint8_t esc_index, float& rpm) const
{
    if (esc_index >= ESC_TELEM_MAX_ESCS) {
        return false;
    }

    const volatile AP_ESC_Telem_Backend::RpmData& rpmdata = _rpm_data[esc_index];

    const uint32_t now = AP_HAL::micros();

    if (!rpm_data_within_timeout(rpmdata, now, ESC_RPM_DATA_TIMEOUT_US)) {
        return false;
    }

    rpm = rpmdata.rpm;
    return true;
}

// get an individual ESC's temperature in centi-degrees if available, returns true on success
bool AP_ESC_Telem::get_temperature(uint8_t esc_index, int16_t& temp) const
{
    const volatile AP_ESC_Telem_Backend::TelemetryData& telemdata = _telem_data[esc_index];
    if (esc_index >= ESC_TELEM_MAX_ESCS
        || telemdata.stale()
        || !(telemdata.types & (AP_ESC_Telem_Backend::TelemetryType::TEMPERATURE | AP_ESC_Telem_Backend::TelemetryType::TEMPERATURE_EXTERNAL))) {
        return false;
    }
    temp = telemdata.temperature_cdeg;
    return true;
}

// get an individual motor's temperature in centi-degrees if available, returns true on success
bool AP_ESC_Telem::get_motor_temperature(uint8_t esc_index, int16_t& temp) const
{
    const volatile AP_ESC_Telem_Backend::TelemetryData& telemdata = _telem_data[esc_index];
    if (esc_index >= ESC_TELEM_MAX_ESCS
        || telemdata.stale()
        || !(telemdata.types & (AP_ESC_Telem_Backend::TelemetryType::MOTOR_TEMPERATURE | AP_ESC_Telem_Backend::TelemetryType::MOTOR_TEMPERATURE_EXTERNAL))) {
        return false;
    }
    temp = telemdata.motor_temp_cdeg;
    return true;
}

// get the highest ESC temperature in centi-degrees if available, returns true if there is valid data for at least one ESC
bool AP_ESC_Telem::get_highest_motor_temperature(int16_t& temp) const
{
    uint8_t valid_escs = 0;

    for (uint8_t i = 0; i < ESC_TELEM_MAX_ESCS; i++) {
        int16_t temp_temp;
        if (get_motor_temperature(i, temp_temp)) {
            temp = MAX(temp, temp_temp);
            valid_escs++;
        }
    }

    return valid_escs > 0;
}

// get an individual ESC's current in Ampere if available, returns true on success
bool AP_ESC_Telem::get_current(uint8_t esc_index, float& amps) const
{
    const volatile AP_ESC_Telem_Backend::TelemetryData& telemdata = _telem_data[esc_index];
    if (esc_index >= ESC_TELEM_MAX_ESCS
        || telemdata.stale()
        || !(telemdata.types & AP_ESC_Telem_Backend::TelemetryType::CURRENT)) {
        return false;
    }
    amps = telemdata.current;
    return true;
}

// get an individual ESC's voltage in Volt if available, returns true on success
bool AP_ESC_Telem::get_voltage(uint8_t esc_index, float& volts) const
{
    const volatile AP_ESC_Telem_Backend::TelemetryData& telemdata = _telem_data[esc_index];
    if (esc_index >= ESC_TELEM_MAX_ESCS
        || telemdata.stale()
        || !(telemdata.types & AP_ESC_Telem_Backend::TelemetryType::VOLTAGE)) {
        return false;
    }
    volts = telemdata.voltage;
    return true;
}

// get an individual ESC's energy consumption in milli-Ampere.hour if available, returns true on success
bool AP_ESC_Telem::get_consumption_mah(uint8_t esc_index, float& consumption_mah) const
{
    const volatile AP_ESC_Telem_Backend::TelemetryData& telemdata = _telem_data[esc_index];
    if (esc_index >= ESC_TELEM_MAX_ESCS
        || telemdata.stale()
        || !(telemdata.types & AP_ESC_Telem_Backend::TelemetryType::CONSUMPTION)) {
        return false;
    }
    consumption_mah = telemdata.consumption_mah;
    return true;
}

// get an individual ESC's usage time in seconds if available, returns true on success
bool AP_ESC_Telem::get_usage_seconds(uint8_t esc_index, uint32_t& usage_s) const
{
    const volatile AP_ESC_Telem_Backend::TelemetryData& telemdata = _telem_data[esc_index];
    if (esc_index >= ESC_TELEM_MAX_ESCS
        || telemdata.stale()
        || !(telemdata.types & AP_ESC_Telem_Backend::TelemetryType::USAGE)) {
        return false;
    }
    usage_s = telemdata.usage_s;
    return true;
}

// send ESC telemetry messages over MAVLink
void AP_ESC_Telem::send_esc_telemetry_mavlink(uint8_t mav_chan)
{
#if HAL_GCS_ENABLED
    if (!_have_data) {
        // we've never had any data
        return;
    }

    const uint32_t now = AP_HAL::millis();
    const uint32_t now_us = AP_HAL::micros();

    // loop through groups of 4 ESCs
    const uint8_t esc_offset = constrain_int16(mavlink_offset, 0, ESC_TELEM_MAX_ESCS - 1);
    const uint8_t num_idx = ESC_TELEM_MAX_ESCS / 4;
    for (uint8_t idx = 0; idx < num_idx; idx++) {
        const uint8_t i = (next_idx + idx) % num_idx;

        // return if no space in output buffer to send mavlink messages
        if (!HAVE_PAYLOAD_SPACE((mavlink_channel_t)mav_chan, ESC_TELEMETRY_1_TO_4)) {
            // not enough mavlink buffer space, start at this index next time
            next_idx = i;
            return;
        }

        bool all_stale = true;
        for (uint8_t j=0; j<4; j++) {
            const uint8_t esc_id = (i * 4 + j) + esc_offset;
            if (esc_id < ESC_TELEM_MAX_ESCS &&
                (!_telem_data[esc_id].stale(now) ||
                 rpm_data_within_timeout(_rpm_data[esc_id], now_us, ESC_RPM_DATA_TIMEOUT_US))) {
                all_stale = false;
                break;
            }
        }
        if (all_stale) {
            // skip this group of ESCs if no data to send
            continue;
        }


        // arrays to hold output
        mavlink_esc_telemetry_1_to_4_t s {};

        // fill in output arrays
        for (uint8_t j = 0; j < 4; j++) {
            const uint8_t esc_id = (i * 4 + j) + esc_offset;
            if (esc_id >= ESC_TELEM_MAX_ESCS) {
                continue;
            }
            volatile AP_ESC_Telem_Backend::TelemetryData const &telemdata = _telem_data[esc_id];

            s.temperature[j] = telemdata.temperature_cdeg / 100;
            s.voltage[j] = constrain_float(telemdata.voltage * 100.0f, 0, UINT16_MAX);
            s.current[j] = constrain_float(telemdata.current * 100.0f, 0, UINT16_MAX);
            s.totalcurrent[j] = constrain_float(telemdata.consumption_mah, 0, UINT16_MAX);
            float rpmf;
            if (get_rpm(esc_id, rpmf)) {
                s.rpm[j] = constrain_float(rpmf, 0, UINT16_MAX);
            }
            s.count[j] = telemdata.count;
        }

        // make sure a msg hasn't been extended
        static_assert(MAVLINK_MSG_ID_ESC_TELEMETRY_1_TO_4_LEN == MAVLINK_MSG_ID_ESC_TELEMETRY_5_TO_8_LEN &&
                      MAVLINK_MSG_ID_ESC_TELEMETRY_1_TO_4_LEN == MAVLINK_MSG_ID_ESC_TELEMETRY_9_TO_12_LEN &&
                      MAVLINK_MSG_ID_ESC_TELEMETRY_1_TO_4_LEN == MAVLINK_MSG_ID_ESC_TELEMETRY_13_TO_16_LEN &&
                      MAVLINK_MSG_ID_ESC_TELEMETRY_1_TO_4_LEN == MAVLINK_MSG_ID_ESC_TELEMETRY_17_TO_20_LEN &&
                      MAVLINK_MSG_ID_ESC_TELEMETRY_1_TO_4_LEN == MAVLINK_MSG_ID_ESC_TELEMETRY_21_TO_24_LEN &&
                      MAVLINK_MSG_ID_ESC_TELEMETRY_1_TO_4_LEN == MAVLINK_MSG_ID_ESC_TELEMETRY_21_TO_24_LEN &&
                      MAVLINK_MSG_ID_ESC_TELEMETRY_1_TO_4_LEN == MAVLINK_MSG_ID_ESC_TELEMETRY_25_TO_28_LEN &&
                      MAVLINK_MSG_ID_ESC_TELEMETRY_1_TO_4_LEN == MAVLINK_MSG_ID_ESC_TELEMETRY_29_TO_32_LEN,
                      "telem messages not compatible");

        const mavlink_channel_t chan = (mavlink_channel_t)mav_chan;
        // send messages
        switch (i) {
            case 0:
                mavlink_msg_esc_telemetry_1_to_4_send_struct(chan, &s);
                break;
            case 1:
                mavlink_msg_esc_telemetry_5_to_8_send_struct(chan, (const mavlink_esc_telemetry_5_to_8_t *)&s);
                break;
            case 2:
                mavlink_msg_esc_telemetry_9_to_12_send_struct(chan, (const mavlink_esc_telemetry_9_to_12_t *)&s);
                break;
            case 3:
                mavlink_msg_esc_telemetry_13_to_16_send_struct(chan, (const mavlink_esc_telemetry_13_to_16_t *)&s);
                break;
#if ESC_TELEM_MAX_ESCS > 16
            case 4:
                mavlink_msg_esc_telemetry_17_to_20_send_struct(chan, (const mavlink_esc_telemetry_17_to_20_t *)&s);
                break;
            case 5:
                mavlink_msg_esc_telemetry_21_to_24_send_struct(chan, (const mavlink_esc_telemetry_21_to_24_t *)&s);
                break;
            case 6:
                mavlink_msg_esc_telemetry_25_to_28_send_struct(chan, (const mavlink_esc_telemetry_25_to_28_t *)&s);
                break;
            case 7:
                mavlink_msg_esc_telemetry_29_to_32_send_struct(chan, (const mavlink_esc_telemetry_29_to_32_t *)&s);
                break;
#endif
        }
    }
    // we checked for all sends without running out of buffer space,
    // start at zero next time
    next_idx = 0;

#endif // HAL_GCS_ENABLED
}

// record an update to the telemetry data together with timestamp
// this should be called by backends when new telemetry values are available
void AP_ESC_Telem::update_telem_data(const uint8_t esc_index, const AP_ESC_Telem_Backend::TelemetryData& new_data, const uint16_t data_mask)
{
    // rpm and telemetry data are not protected by a semaphore even though updated from different threads
    // all data is per-ESC and only written from the update thread and read by the user thread
    // each element is a primitive type and the timestamp is only updated at the end, thus a caller
    // can only get slightly more up-to-date information that perhaps they were expecting or might
    // read data that has just gone stale - both of these are safe and avoid the overhead of locking

    if (esc_index >= ESC_TELEM_MAX_ESCS || data_mask == 0) {
        return;
    }

    _have_data = true;
    volatile AP_ESC_Telem_Backend::TelemetryData &telemdata = _telem_data[esc_index];

#if AP_TEMPERATURE_SENSOR_ENABLED
    // always allow external data. Block "internal" if external has ever its ever been set externally then ignore normal "internal" updates
    const bool has_temperature = (data_mask & AP_ESC_Telem_Backend::TelemetryType::TEMPERATURE_EXTERNAL) ||
        ((data_mask & AP_ESC_Telem_Backend::TelemetryType::TEMPERATURE) && !(telemdata.types & AP_ESC_Telem_Backend::TelemetryType::TEMPERATURE_EXTERNAL));

    const bool has_motor_temperature = (data_mask & AP_ESC_Telem_Backend::TelemetryType::MOTOR_TEMPERATURE_EXTERNAL) ||
        ((data_mask & AP_ESC_Telem_Backend::TelemetryType::MOTOR_TEMPERATURE) && !(telemdata.types & AP_ESC_Telem_Backend::TelemetryType::MOTOR_TEMPERATURE_EXTERNAL));
#else
    const bool has_temperature = (data_mask & AP_ESC_Telem_Backend::TelemetryType::TEMPERATURE);
    const bool has_motor_temperature = (data_mask & AP_ESC_Telem_Backend::TelemetryType::MOTOR_TEMPERATURE);
#endif

    if (has_temperature) {
        telemdata.temperature_cdeg = new_data.temperature_cdeg;
    }
    if (has_motor_temperature) {
        telemdata.motor_temp_cdeg = new_data.motor_temp_cdeg;
    }
    if (data_mask & AP_ESC_Telem_Backend::TelemetryType::VOLTAGE) {
        telemdata.voltage = new_data.voltage;
    }
    if (data_mask & AP_ESC_Telem_Backend::TelemetryType::CURRENT) {
        telemdata.current = new_data.current;
    }
    if (data_mask & AP_ESC_Telem_Backend::TelemetryType::CONSUMPTION) {
        telemdata.consumption_mah = new_data.consumption_mah;
    }
    if (data_mask & AP_ESC_Telem_Backend::TelemetryType::USAGE) {
        telemdata.usage_s = new_data.usage_s;
    }

    telemdata.count++;
    telemdata.types |= data_mask;
    telemdata.last_update_ms = AP_HAL::millis();
}

// record an update to the RPM together with timestamp, this allows the notch values to be slewed
// this should be called by backends when new telemetry values are available
void AP_ESC_Telem::update_rpm(const uint8_t esc_index, const float new_rpm, const float error_rate)
{
    if (isnan(new_rpm)) {
        abort();
    }

    if (esc_index >= ESC_TELEM_MAX_ESCS) {
        return;
    }

    _have_data = true;

    const uint32_t now = MAX(1U ,AP_HAL::micros()); // don't allow a value of 0 in, as we use this as a flag in places
    volatile AP_ESC_Telem_Backend::RpmData& rpmdata = _rpm_data[esc_index];
    const auto last_update_us = rpmdata.last_update_us;

    rpmdata.prev_rpm = rpmdata.rpm;
    rpmdata.rpm = new_rpm;
    rpmdata.update_rate_hz = 1.0e6f / constrain_uint32((now - last_update_us), 100, 1000000U*10U); // limit the update rate 0.1Hz to 10KHz 
    rpmdata.last_update_us = now;
    rpmdata.error_rate = error_rate;
    rpmdata.data_valid = true;

#ifdef ESC_TELEM_DEBUG
    hal.console->printf("RPM: rate=%.1fhz, rpm=%f)\n", rpmdata.update_rate_hz, new_rpm);
#endif
}

void AP_ESC_Telem::update()
{
    update_telemetry();
    update_logging();
}

#if HAL_LOGGING_ENABLED
void AP_ESC_Telem::update_logging()
{
    AP_Logger *logger = AP_Logger::get_singleton();
    const uint64_t now_us64 = AP_HAL::micros64();

    for (uint8_t i = 0; i < ESC_TELEM_MAX_ESCS; i++) {
        const volatile AP_ESC_Telem_Backend::RpmData &rpmdata = _rpm_data[i];
        const volatile AP_ESC_Telem_Backend::TelemetryData &telemdata = _telem_data[i];
        // Push received telemetry data into the logging system
        if (logger && logger->logging_enabled()) {
            if (telemdata.last_update_ms != _last_telem_log_ms[i]
                || rpmdata.last_update_us != _last_rpm_log_us[i]) {

                float rpm = AP::logger().quiet_nanf();
                get_rpm(i, rpm);
                float raw_rpm = AP::logger().quiet_nanf();
                get_raw_rpm(i, raw_rpm);

                // Write ESC status messages
                //   id starts from 0
                //   rpm, raw_rpm is eRPM (in RPM units)
                //   voltage is in Volt
                //   current is in Ampere
                //   esc_temp is in centi-degrees Celsius
                //   current_tot is in milli-Ampere hours
                //   motor_temp is in centi-degrees Celsius
                //   error_rate is in percentage
                const struct log_Esc pkt{
                    LOG_PACKET_HEADER_INIT(uint8_t(LOG_ESC_MSG)),
                    time_us     : now_us64,
                    instance    : i,
                    rpm         : rpm,
                    raw_rpm     : raw_rpm,
                    voltage     : telemdata.voltage,
                    current     : telemdata.current,
                    esc_temp    : telemdata.temperature_cdeg,
                    current_tot : telemdata.consumption_mah,
                    motor_temp  : telemdata.motor_temp_cdeg,
                    error_rate  : rpmdata.error_rate
                };
                AP::logger().WriteBlock(&pkt, sizeof(pkt));
                _last_telem_log_ms[i] = telemdata.last_update_ms;
                _last_rpm_log_us[i] = rpmdata.last_update_us;
            }
        }
    }
#endif  // HAL_LOGGING_ENABLED

    const uint32_t now_us = AP_HAL::micros();
    for (uint8_t i = 0; i < ESC_TELEM_MAX_ESCS; i++) {
        // Invalidate RPM data if not received for too long
        if ((now_us - _rpm_data[i].last_update_us) > ESC_RPM_DATA_TIMEOUT_US) {
            _rpm_data[i].data_valid = false;
        }
    }
}

bool AP_ESC_Telem::rpm_data_within_timeout(const volatile AP_ESC_Telem_Backend::RpmData &instance, const uint32_t now_us, const uint32_t timeout_us)
{
    // easy case, has the time window been crossed so it's invalid
    if ((now_us - instance.last_update_us) > timeout_us) {
        return false;
    }
    // we never got a valid data, to it's invalid
    if (instance.last_update_us == 0) {
        return false;
    }
    // check if things generally expired on us, this is done to handle time wrapping
    return instance.data_valid;
}

bool AP_ESC_Telem::was_rpm_data_ever_reported(const volatile AP_ESC_Telem_Backend::RpmData &instance)
{
    return instance.last_update_us > 0;
}

#if AP_SCRIPTING_ENABLED
/*
  set RPM scale factor from script
*/
void AP_ESC_Telem::set_rpm_scale(const uint8_t esc_index, const float scale_factor)
{
    if (esc_index < ESC_TELEM_MAX_ESCS) {
        rpm_scale_factor[esc_index] = scale_factor;
        rpm_scale_mask |= (1U<<esc_index);
    }
}
#endif

AP_ESC_Telem *AP_ESC_Telem::_singleton = nullptr;

/*
 * Get the AP_ESC_Telem singleton
 */
AP_ESC_Telem *AP_ESC_Telem::get_singleton()
{
    return AP_ESC_Telem::_singleton;
}

namespace AP {

AP_ESC_Telem &esc_telem()
{
    return *AP_ESC_Telem::get_singleton();
}

};

#endif
