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
#include <AP_HAL/AP_HAL.h>
#include <GCS_MAVLink/GCS.h>
#include <AP_Logger/AP_Logger.h>

#if HAL_WITH_ESC_TELEM

#include <AP_BoardConfig/AP_BoardConfig.h>
#include <AP_TemperatureSensor/AP_TemperatureSensor_config.h>

#include <AP_Math/AP_Math.h>

//#define ESC_TELEM_DEBUG

#define ESC_RPM_CHECK_TIMEOUT_US 210000UL   // timeout for motor running validity

extern const AP_HAL::HAL& hal;

// table of user settable parameters
const AP_Param::GroupInfo AP_ESC_Telem::var_info[] = {

    // @Param: _MAV_OFS
    // @DisplayName: ESC Telemetry mavlink offset
    // @Description: Offset to apply to ESC numbers when reporting as ESC_TELEMETRY packets over MAVLink. This allows high numbered motors to be displayed as low numbered ESCs for convenience on GCS displays. A value of 4 would send ESC on output 5 as ESC number 1 in ESC_TELEMETRY packets
    // @Increment: 1
    // @Range: 0 31
    // @User: Standard
    AP_GROUPINFO("_MAV_OFS", 1, AP_ESC_Telem, mavlink_offset, 0),
    
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
    for (uint8_t i = 0; i < ESC_TELEM_MAX_ESCS; i++) {
        if (_telem_data[i].last_update_ms == 0 && !was_rpm_data_ever_reported(_rpm_data[i])) {
            // have never seen telem from this ESC
            continue;
        }
        if (_telem_data[i].stale() && !_rpm_data[i].data_valid) {
            continue;
        }
        ret |= (1U << i);
    }
    return ret;
}

// return an active ESC for the purposes of reporting (e.g. in the OSD)
uint8_t AP_ESC_Telem::get_max_rpm_esc() const
{
    uint32_t ret = 0;
    float max_rpm = 0;
    for (uint8_t i = 0; i < ESC_TELEM_MAX_ESCS; i++) {
        if (_telem_data[i].last_update_ms == 0 && !was_rpm_data_ever_reported(_rpm_data[i])) {
            // have never seen telem from this ESC
            continue;
        }
        if (_telem_data[i].stale() && !_rpm_data[i].data_valid) {
            continue;
        }
        if (_rpm_data[i].rpm > max_rpm) {
            max_rpm = _rpm_data[i].rpm;
            ret = i;
        }
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

    for (uint8_t i = 0; i < ESC_TELEM_MAX_ESCS; i++) {
        if (BIT_IS_SET(servo_channel_mask, i)) {
            const volatile AP_ESC_Telem_Backend::RpmData& rpmdata = _rpm_data[i];
            // we choose a relatively strict measure of health so that failsafe actions can rely on the results
            if (!rpm_data_within_timeout(rpmdata, ESC_RPM_CHECK_TIMEOUT_US)) {
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
    if (rpmdata.data_valid) {
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

// get an individual ESC's raw rpm if available, returns true on success
bool AP_ESC_Telem::get_raw_rpm_and_error_rate(uint8_t esc_index, float& rpm, float& error_rate) const
{
    if (esc_index >= ESC_TELEM_MAX_ESCS) {
        return false;
    }

    const volatile AP_ESC_Telem_Backend::RpmData& rpmdata = _rpm_data[esc_index];

    if (!rpmdata.data_valid) {
        return false;
    }

    rpm = rpmdata.rpm;
    error_rate = rpmdata.error_rate;
    return true;
}

// get an individual ESC's temperature in centi-degrees if available, returns true on success
bool AP_ESC_Telem::get_temperature(uint8_t esc_index, int16_t& temp) const
{
    if (esc_index >= ESC_TELEM_MAX_ESCS) {
        return false;
    }

    const volatile AP_ESC_Telem_Backend::TelemetryData& telemdata = _telem_data[esc_index];
    if (!telemdata.valid(AP_ESC_Telem_Backend::TelemetryType::TEMPERATURE | AP_ESC_Telem_Backend::TelemetryType::TEMPERATURE_EXTERNAL)) {
        return false;
    }
    temp = telemdata.temperature_cdeg;
    return true;
}

// get an individual motor's temperature in centi-degrees if available, returns true on success
bool AP_ESC_Telem::get_motor_temperature(uint8_t esc_index, int16_t& temp) const
{
    if (esc_index >= ESC_TELEM_MAX_ESCS) {
        return false;
    }

    const volatile AP_ESC_Telem_Backend::TelemetryData& telemdata = _telem_data[esc_index];
    if (!telemdata.valid(AP_ESC_Telem_Backend::TelemetryType::MOTOR_TEMPERATURE | AP_ESC_Telem_Backend::TelemetryType::MOTOR_TEMPERATURE_EXTERNAL)) {
        return false;
    }
    temp = telemdata.motor_temp_cdeg;
    return true;
}

// get the highest ESC temperature in centi-degrees if available, returns true if there is valid data for at least one ESC
bool AP_ESC_Telem::get_highest_temperature(int16_t& temp) const
{
    uint8_t valid_escs = 0;

    for (uint8_t i = 0; i < ESC_TELEM_MAX_ESCS; i++) {
        int16_t temp_temp;
        if (get_temperature(i, temp_temp)) {
            temp = MAX(temp, temp_temp);
            valid_escs++;
        }
    }

    return valid_escs > 0;
}

// get an individual ESC's current in Ampere if available, returns true on success
bool AP_ESC_Telem::get_current(uint8_t esc_index, float& amps) const
{
    if (esc_index >= ESC_TELEM_MAX_ESCS) {
        return false;
    }

    const volatile AP_ESC_Telem_Backend::TelemetryData& telemdata = _telem_data[esc_index];
    if (!telemdata.valid(AP_ESC_Telem_Backend::TelemetryType::CURRENT)) {
        return false;
    }
    amps = telemdata.current;
    return true;
}

// get an individual ESC's voltage in Volt if available, returns true on success
bool AP_ESC_Telem::get_voltage(uint8_t esc_index, float& volts) const
{
    if (esc_index >= ESC_TELEM_MAX_ESCS) {
        return false;
    }

    const volatile AP_ESC_Telem_Backend::TelemetryData& telemdata = _telem_data[esc_index];
    if (!telemdata.valid(AP_ESC_Telem_Backend::TelemetryType::VOLTAGE)) {
        return false;
    }
    volts = telemdata.voltage;
    return true;
}

// get an individual ESC's energy consumption in milli-Ampere.hour if available, returns true on success
bool AP_ESC_Telem::get_consumption_mah(uint8_t esc_index, float& consumption_mah) const
{
    if (esc_index >= ESC_TELEM_MAX_ESCS) {
        return false;
    }

    const volatile AP_ESC_Telem_Backend::TelemetryData& telemdata = _telem_data[esc_index];
    if (!telemdata.valid(AP_ESC_Telem_Backend::TelemetryType::CONSUMPTION)) {
        return false;
    }
    consumption_mah = telemdata.consumption_mah;
    return true;
}

// get an individual ESC's usage time in seconds if available, returns true on success
bool AP_ESC_Telem::get_usage_seconds(uint8_t esc_index, uint32_t& usage_s) const
{
    if (esc_index >= ESC_TELEM_MAX_ESCS) {
        return false;
    }

    const volatile AP_ESC_Telem_Backend::TelemetryData& telemdata = _telem_data[esc_index];
    if (!telemdata.valid(AP_ESC_Telem_Backend::TelemetryType::USAGE)) {
        return false;
    }
    usage_s = telemdata.usage_s;
    return true;
}

#if AP_EXTENDED_ESC_TELEM_ENABLED
// get an individual ESC's input duty cycle if available, returns true on success
bool AP_ESC_Telem::get_input_duty(uint8_t esc_index, uint8_t& input_duty) const
{
    if (esc_index >= ESC_TELEM_MAX_ESCS) {
        return false;
    }

    const volatile AP_ESC_Telem_Backend::TelemetryData& telemdata = _telem_data[esc_index];
    if (!telemdata.valid(AP_ESC_Telem_Backend::TelemetryType::INPUT_DUTY)) {
        return false;
    }
    input_duty = telemdata.input_duty;
    return true;
}

// get an individual ESC's output duty cycle if available, returns true on success
bool AP_ESC_Telem::get_output_duty(uint8_t esc_index, uint8_t& output_duty) const
{
    if (esc_index >= ESC_TELEM_MAX_ESCS) {
        return false;
    }

    const volatile AP_ESC_Telem_Backend::TelemetryData& telemdata = _telem_data[esc_index];
    if (!telemdata.valid(AP_ESC_Telem_Backend::TelemetryType::OUTPUT_DUTY)) {
        return false;
    }
    output_duty = telemdata.output_duty;
    return true;
}

// get an individual ESC's status flags if available, returns true on success
bool AP_ESC_Telem::get_flags(uint8_t esc_index, uint32_t& flags) const
{
    if (esc_index >= ESC_TELEM_MAX_ESCS) {
        return false;
    }

    const volatile AP_ESC_Telem_Backend::TelemetryData& telemdata = _telem_data[esc_index];
    if (!telemdata.valid(AP_ESC_Telem_Backend::TelemetryType::FLAGS)) {
        return false;
    }
    flags = telemdata.flags;
    return true;
}

// get an individual ESC's percentage of output power if available, returns true on success
bool AP_ESC_Telem::get_power_percentage(uint8_t esc_index, uint8_t& power_percentage) const
{
    if (esc_index >= ESC_TELEM_MAX_ESCS) {
        return false;
    }

    const volatile AP_ESC_Telem_Backend::TelemetryData& telemdata = _telem_data[esc_index];
    if (!telemdata.valid(AP_ESC_Telem_Backend::TelemetryType::POWER_PERCENTAGE)) {
        return false;
    }
    power_percentage = telemdata.power_percentage;
    return true;
}
#endif // AP_EXTENDED_ESC_TELEM_ENABLED

// send ESC telemetry messages over MAVLink
void AP_ESC_Telem::send_esc_telemetry_mavlink(uint8_t mav_chan)
{
#if HAL_GCS_ENABLED
    if (!_have_data) {
        // we've never had any data
        return;
    }

    // loop through groups of 4 ESCs
    const uint8_t esc_offset = constrain_int16(mavlink_offset, 0, ESC_TELEM_MAX_ESCS-1);

    // ensure we send out partially-full groups:
    const uint8_t num_idx = (ESC_TELEM_MAX_ESCS + 3) / 4;

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
                (!_telem_data[esc_id].stale() || _rpm_data[esc_id].data_valid)) {
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
                // rpm can be negative
                s.rpm[j] = constrain_float(fabsf(rpmf), 0, UINT16_MAX);
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

#if AP_EXTENDED_ESC_TELEM_ENABLED
    if (data_mask & AP_ESC_Telem_Backend::TelemetryType::INPUT_DUTY) {
        telemdata.input_duty = new_data.input_duty;
    }
    if (data_mask & AP_ESC_Telem_Backend::TelemetryType::OUTPUT_DUTY) {
        telemdata.output_duty = new_data.output_duty;
    }
    if (data_mask & AP_ESC_Telem_Backend::TelemetryType::FLAGS) {
        telemdata.flags = new_data.flags;
    }
    if (data_mask & AP_ESC_Telem_Backend::TelemetryType::POWER_PERCENTAGE) {
        telemdata.power_percentage = new_data.power_percentage;
    }
#endif //AP_EXTENDED_ESC_TELEM_ENABLED

#if AP_EXTENDED_DSHOT_TELEM_V2_ENABLED
    if (data_mask & AP_ESC_Telem_Backend::TelemetryType::EDT2_STATUS) {
        telemdata.edt2_status = merge_edt2_status(telemdata.edt2_status, new_data.edt2_status);
    }
    if (data_mask & AP_ESC_Telem_Backend::TelemetryType::EDT2_STRESS) {
        telemdata.edt2_stress = merge_edt2_stress(telemdata.edt2_stress, new_data.edt2_stress);
    }
#endif

    telemdata.count++;
    telemdata.types |= data_mask;
    telemdata.last_update_ms = AP_HAL::millis();
    telemdata.any_data_valid = true;
}

// record an update to the RPM together with timestamp, this allows the notch values to be slewed
// this should be called by backends when new telemetry values are available
void AP_ESC_Telem::update_rpm(const uint8_t esc_index, const float new_rpm, const float error_rate)
{
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

#if AP_EXTENDED_DSHOT_TELEM_V2_ENABLED

// The following is based on https://github.com/bird-sanctuary/extended-dshot-telemetry.
// For the following part we explain the bits of Extended DShot Telemetry v2 status telemetry:
// - bits 0-3: the "stress level"
// - bit 5: the "error" bit   (e.g. the stall event in Bluejay)
// - bit 6: the "warning" bit (e.g. the desync event in Bluejay)
// - bit 7: the "alert" bit   (e.g. the demag event in Bluejay)

// Since logger can read out telemetry values less frequently than they are updated,
// it makes sense to aggregate these status bits, and to collect the maximum observed stress level.
// To reduce the logging rate of the EDT2 messages, we will try to log them only once a new frame comes.
// To track this, we are going to (ab)use bit 15 of the field: 1 means there is something to write.

// EDTv2 also features separate "stress" messages.
// These come more frequently, and are scaled differently (the allowed range is from 0 to 255),
// so we have to log them separately.

constexpr uint16_t EDT2_TELEM_UPDATED = 0x8000U;
constexpr uint16_t EDT2_STRESS_0F_MASK = 0xfU;
constexpr uint16_t EDT2_STRESS_FF_MASK = 0xffU;
constexpr uint16_t EDT2_ERROR_MASK = 0x20U;
constexpr uint16_t EDT2_WARNING_MASK = 0x40U;
constexpr uint16_t EDT2_ALERT_MASK = 0x80U;
constexpr uint16_t EDT2_ALL_BITS = EDT2_ERROR_MASK | EDT2_WARNING_MASK | EDT2_ALERT_MASK;

#define EDT2_HAS_NEW_DATA(status) bool((status) & EDT2_TELEM_UPDATED)
#define EDT2_STRESS_FROM_STATUS(status) uint8_t((status) & EDT2_STRESS_0F_MASK)
#define EDT2_ERROR_BIT_FROM_STATUS(status) bool((status) & EDT2_ERROR_MASK)
#define EDT2_WARNING_BIT_FROM_STATUS(status) bool((status) & EDT2_WARNING_MASK)
#define EDT2_ALERT_BIT_FROM_STATUS(status) bool((status) & EDT2_ALERT_MASK)
#define EDT2_STRESS_FROM_STRESS(stress) uint8_t((stress) & EDT2_STRESS_FF_MASK)

uint16_t AP_ESC_Telem::merge_edt2_status(uint16_t old_status, uint16_t new_status)
{
    if (EDT2_HAS_NEW_DATA(old_status)) {
        new_status = uint16_t(
            (old_status & ~EDT2_STRESS_0F_MASK) | // old status except for the stress is preserved
            (new_status & EDT2_ALL_BITS) | // all new status bits are included
            MAX(old_status & EDT2_STRESS_0F_MASK, new_status & EDT2_STRESS_0F_MASK) // the stress is maxed out
        );
    }
    return EDT2_TELEM_UPDATED | new_status;
}

uint16_t AP_ESC_Telem::merge_edt2_stress(uint16_t old_stress, uint16_t new_stress)
{
    if (EDT2_HAS_NEW_DATA(old_stress)) {
        new_stress = uint16_t(
            MAX(old_stress & EDT2_STRESS_FF_MASK, new_stress & EDT2_STRESS_FF_MASK) // the stress is maxed out
        );
    }
    return EDT2_TELEM_UPDATED | new_stress;
}

#endif // AP_EXTENDED_DSHOT_TELEM_V2_ENABLED

void AP_ESC_Telem::update()
{
#if HAL_LOGGING_ENABLED
    AP_Logger *logger = AP_Logger::get_singleton();
    const uint64_t now_us64 = AP_HAL::micros64();

    for (uint8_t i = 0; i < ESC_TELEM_MAX_ESCS; i++) {
        const volatile AP_ESC_Telem_Backend::RpmData &rpmdata = _rpm_data[i];
        volatile AP_ESC_Telem_Backend::TelemetryData &telemdata = _telem_data[i];
        // Push received telemetry data into the logging system
        if (logger && logger->logging_enabled()) {
            if (telemdata.last_update_ms != _last_telem_log_ms[i]
                || rpmdata.last_update_us != _last_rpm_log_us[i]) {

                // Update last log timestamps
                _last_telem_log_ms[i] = telemdata.last_update_ms;
                _last_rpm_log_us[i] = rpmdata.last_update_us;

                float rpm = AP_Logger::quiet_nanf();
                get_rpm(i, rpm);
                float raw_rpm = AP_Logger::quiet_nanf();
                float rpm_error_rate = AP_Logger::quiet_nanf();
                get_raw_rpm_and_error_rate(i, raw_rpm, rpm_error_rate);

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
                    error_rate  : rpm_error_rate
                };
                AP::logger().WriteBlock(&pkt, sizeof(pkt));

#if AP_EXTENDED_ESC_TELEM_ENABLED
                // Write ESC extended status messages
                //   id: starts from 0
                //   input duty: duty cycle input to the ESC in percent
                //   output duty: duty cycle output to the motor in percent
                //   status flags: manufacurer-specific status flags
                const bool has_ext_data = telemdata.types & 
                        (AP_ESC_Telem_Backend::TelemetryType::INPUT_DUTY |
                         AP_ESC_Telem_Backend::TelemetryType::OUTPUT_DUTY |
                         AP_ESC_Telem_Backend::TelemetryType::FLAGS |
                         AP_ESC_Telem_Backend::TelemetryType::POWER_PERCENTAGE);
                if (has_ext_data) {
                    // @LoggerMessage: ESCX
                    // @Description: ESC extended telemetry data
                    // @Field: TimeUS: Time since system startup
                    // @Field: Instance: starts from 0
                    // @Field: inpct: input duty cycle in percent
                    // @Field: outpct: output duty cycle in percent
                    // @Field: flags: manufacturer-specific status flags
                    // @Field: Pwr: Power percentage
                    AP::logger().WriteStreaming("ESCX",
                                                "TimeUS,Instance,inpct,outpct,flags,Pwr",
                                                "s"     "#"      "%"   "%"    "-"   "%",
                                                "F"     "-"      "-"   "-"    "-"   "-",
                                                "Q"     "B"      "B"   "B"    "I"   "B",
                                                AP_HAL::micros64(),
                                                i,
                                                telemdata.input_duty,
                                                telemdata.output_duty,
                                                telemdata.flags,
                                                telemdata.power_percentage);
                }
#endif //AP_EXTENDED_ESC_TELEM_ENABLED

#if AP_EXTENDED_DSHOT_TELEM_V2_ENABLED
                // Write an EDTv2 message, if there is any update
                uint16_t edt2_status = telemdata.edt2_status;
                uint16_t edt2_stress = telemdata.edt2_stress;
                if (EDT2_HAS_NEW_DATA(edt2_status | edt2_stress)) {
                    // Could probably be faster/smaller with bitmasking, but not sure
                    uint8_t status = 0;
                    if (EDT2_HAS_NEW_DATA(edt2_stress)) {
                        status |= uint8_t(log_Edt2_Status::HAS_STRESS_DATA);
                    }
                    if (EDT2_HAS_NEW_DATA(edt2_status)) {
                        status |= uint8_t(log_Edt2_Status::HAS_STATUS_DATA);
                    }
                    if (EDT2_ALERT_BIT_FROM_STATUS(edt2_status)) {
                        status |= uint8_t(log_Edt2_Status::ALERT_BIT);
                    }
                    if (EDT2_WARNING_BIT_FROM_STATUS(edt2_status)) {
                        status |= uint8_t(log_Edt2_Status::WARNING_BIT);
                    }
                    if (EDT2_ERROR_BIT_FROM_STATUS(edt2_status)) {
                        status |= uint8_t(log_Edt2_Status::ERROR_BIT);
                    }
                    // An EDT2 status message is:
                    //   id: starts from 0
                    //   stress: the current stress which comes from edt2_stress
                    //   max_stress: the maximum stress which comes from edt2_status
                    //   status: the status bits which come from both
                    const struct log_Edt2 pkt_edt2{
                        LOG_PACKET_HEADER_INIT(uint8_t(LOG_EDT2_MSG)),
                        time_us     : now_us64,
                        instance    : i,
                        stress      : EDT2_STRESS_FROM_STRESS(edt2_stress),
                        max_stress  : EDT2_STRESS_FROM_STATUS(edt2_status),
                        status      : status,
                    };
                    if (AP::logger().WriteBlock_first_succeed(&pkt_edt2, sizeof(pkt_edt2))) {
                        // Only clean the telem_updated bits if the write succeeded.
                        // This is important because, if rate limiting is enabled,
                        // the log-on-change behavior may lose a lot of entries
                        telemdata.edt2_status &= ~EDT2_TELEM_UPDATED;
                        telemdata.edt2_stress &= ~EDT2_TELEM_UPDATED;
                    }
                }
#endif // AP_EXTENDED_DSHOT_TELEM_V2_ENABLED
            }
        }
    }
#endif  // HAL_LOGGING_ENABLED

    for (uint8_t i = 0; i < ESC_TELEM_MAX_ESCS; i++) {
        // copy the last_updated_us timestamp to avoid any race issues
        const uint32_t last_updated_us = _rpm_data[i].last_update_us;
        const uint32_t now_us = AP_HAL::micros();
        // Invalidate RPM data if not received for too long
        if (AP_HAL::timeout_expired(last_updated_us, now_us, ESC_RPM_DATA_TIMEOUT_US)) {
            _rpm_data[i].data_valid = false;
        }
        const uint32_t last_telem_data_ms = _telem_data[i].last_update_ms;
        const uint32_t now_ms = AP_HAL::millis();
        // Invalidate telemetry data if not received for too long
        if (AP_HAL::timeout_expired(last_telem_data_ms, now_ms, ESC_TELEM_DATA_TIMEOUT_MS)) {
            _telem_data[i].any_data_valid = false;
        }
    }
}

// NOTE: This function should only be used to check timeouts other than 
// ESC_RPM_DATA_TIMEOUT_US. Timeouts equal to ESC_RPM_DATA_TIMEOUT_US should
// use RpmData::data_valid, which is cheaper and achieves the same result.
bool AP_ESC_Telem::rpm_data_within_timeout(const volatile AP_ESC_Telem_Backend::RpmData &instance, const uint32_t timeout_us)
{
    // copy the last_update_us timestamp to avoid any race issues
    const uint32_t last_update_us = instance.last_update_us;
    const uint32_t now_us = AP_HAL::micros();
    // easy case, has the time window been crossed so it's invalid
    if (AP_HAL::timeout_expired(last_update_us, now_us, timeout_us)) {
        return false;
    }
    // we never got a valid data, to it's invalid
    if (last_update_us == 0) {
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

#endif // HAL_WITH_ESC_TELEM
