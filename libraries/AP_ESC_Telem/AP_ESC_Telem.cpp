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

//#define ESC_TELEM_DEBUG

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
    AP_Param::setup_object_defaults(this, var_info);
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
    for (uint8_t i = 0; i < ESC_TELEM_MAX_ESCS && i < nfreqs; i++) {
        float rpm;
        if (get_rpm(i, rpm)) {
            freqs[valid_escs++] = rpm * (1.0f / 60.0f);
        }
    }

    return MIN(valid_escs, nfreqs);
}

// get mask of ESCs that sent valid telemetry data in the last
// ESC_TELEM_DATA_TIMEOUT_MS
uint32_t AP_ESC_Telem::get_active_esc_mask() const {
    uint32_t ret = 0;
    const uint32_t now = AP_HAL::millis();
    for (uint8_t i = 0; i < ESC_TELEM_MAX_ESCS; i++) {
        if (now - _telem_data[i].last_update_ms >= ESC_TELEM_DATA_TIMEOUT_MS) {
            continue;
        }
        if (_telem_data[i].last_update_ms == 0) {
            // have never seen telem from this ESC
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
    if (rpmdata.last_update_us > 0 && (now >= rpmdata.last_update_us)
        && (now - rpmdata.last_update_us < ESC_RPM_DATA_TIMEOUT_US)) {
        const float slew = MIN(1.0f, (now - rpmdata.last_update_us) * rpmdata.update_rate_hz * (1.0f / 1e6f));
        rpm = (rpmdata.prev_rpm + (rpmdata.rpm - rpmdata.prev_rpm) * slew);
        return true;
    }
    return false;
}

// get an individual ESC's raw rpm if available, returns true on success
bool AP_ESC_Telem::get_raw_rpm(uint8_t esc_index, float& rpm) const
{
    if (esc_index >= ESC_TELEM_MAX_ESCS) {
        return false;
    }

    const volatile AP_ESC_Telem_Backend::RpmData& rpmdata = _rpm_data[esc_index];

    const uint32_t now = AP_HAL::micros();

    if (now < rpmdata.last_update_us || now - rpmdata.last_update_us > ESC_RPM_DATA_TIMEOUT_US) {
        return false;
    }

    rpm = rpmdata.rpm;
    return true;
}

// get an individual ESC's temperature in centi-degrees if available, returns true on success
bool AP_ESC_Telem::get_temperature(uint8_t esc_index, int16_t& temp) const
{
    if (esc_index >= ESC_TELEM_MAX_ESCS
        || AP_HAL::millis() - _telem_data[esc_index].last_update_ms > ESC_TELEM_DATA_TIMEOUT_MS
        || !(_telem_data[esc_index].types & AP_ESC_Telem_Backend::TelemetryType::TEMPERATURE)) {
        return false;
    }
    temp = _telem_data[esc_index].temperature_cdeg;
    return true;
}

// get an individual motor's temperature in centi-degrees if available, returns true on success
bool AP_ESC_Telem::get_motor_temperature(uint8_t esc_index, int16_t& temp) const
{
    if (esc_index >= ESC_TELEM_MAX_ESCS
        || AP_HAL::millis() - _telem_data[esc_index].last_update_ms > ESC_TELEM_DATA_TIMEOUT_MS
        || !(_telem_data[esc_index].types & AP_ESC_Telem_Backend::TelemetryType::MOTOR_TEMPERATURE)) {
        return false;
    }
    temp = _telem_data[esc_index].motor_temp_cdeg;
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
    if (esc_index >= ESC_TELEM_MAX_ESCS
        || AP_HAL::millis() - _telem_data[esc_index].last_update_ms > ESC_TELEM_DATA_TIMEOUT_MS
        || !(_telem_data[esc_index].types & AP_ESC_Telem_Backend::TelemetryType::CURRENT)) {
        return false;
    }
    amps = _telem_data[esc_index].current;
    return true;
}

// get an individual ESC's voltage in Volt if available, returns true on success
bool AP_ESC_Telem::get_voltage(uint8_t esc_index, float& volts) const
{
    if (esc_index >= ESC_TELEM_MAX_ESCS
        || AP_HAL::millis() - _telem_data[esc_index].last_update_ms > ESC_TELEM_DATA_TIMEOUT_MS
        || !(_telem_data[esc_index].types & AP_ESC_Telem_Backend::TelemetryType::VOLTAGE)) {
        return false;
    }
    volts = _telem_data[esc_index].voltage;
    return true;
}

// get an individual ESC's energy consumption in milli-Ampere.hour if available, returns true on success
bool AP_ESC_Telem::get_consumption_mah(uint8_t esc_index, float& consumption_mah) const
{
    if (esc_index >= ESC_TELEM_MAX_ESCS
        || AP_HAL::millis() - _telem_data[esc_index].last_update_ms > ESC_TELEM_DATA_TIMEOUT_MS
        || !(_telem_data[esc_index].types & AP_ESC_Telem_Backend::TelemetryType::CONSUMPTION)) {
        return false;
    }
    consumption_mah = _telem_data[esc_index].consumption_mah;
    return true;
}

// get an individual ESC's usage time in seconds if available, returns true on success
bool AP_ESC_Telem::get_usage_seconds(uint8_t esc_index, uint32_t& usage_s) const
{
    if (esc_index >= ESC_TELEM_MAX_ESCS
        || AP_HAL::millis() - _telem_data[esc_index].last_update_ms > ESC_TELEM_DATA_TIMEOUT_MS
        || !(_telem_data[esc_index].types & AP_ESC_Telem_Backend::TelemetryType::USAGE)) {
        return false;
    }
    usage_s = _telem_data[esc_index].usage_s;
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

    uint32_t now = AP_HAL::millis();
    uint32_t now_us = AP_HAL::micros();

    // loop through groups of 4 ESCs
    const uint8_t esc_offset = constrain_int16(mavlink_offset, 0, ESC_TELEM_MAX_ESCS-1);
    const uint8_t num_idx = ESC_TELEM_MAX_ESCS/4;
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
                (now - _telem_data[esc_id].last_update_ms <= ESC_TELEM_DATA_TIMEOUT_MS ||
                 now_us - _rpm_data[esc_id].last_update_us <= ESC_RPM_DATA_TIMEOUT_US)) {
                all_stale = false;
                break;
            }
        }
        if (all_stale) {
            // skip this group of ESCs if no data to send
            continue;
        }


        // arrays to hold output
        uint8_t temperature[4] {};
        uint16_t voltage[4] {};
        uint16_t current[4] {};
        uint16_t current_tot[4] {};
        uint16_t rpm[4] {};
        uint16_t count[4] {};

        // fill in output arrays
        for (uint8_t j = 0; j < 4; j++) {
            const uint8_t esc_id = (i * 4 + j) + esc_offset;
            if (esc_id >= ESC_TELEM_MAX_ESCS) {
                continue;
            }

            temperature[j] = _telem_data[esc_id].temperature_cdeg / 100;
            voltage[j] = constrain_float(_telem_data[esc_id].voltage * 100.0f, 0, UINT16_MAX);
            current[j] = constrain_float(_telem_data[esc_id].current * 100.0f, 0, UINT16_MAX);
            current_tot[j] = constrain_float(_telem_data[esc_id].consumption_mah, 0, UINT16_MAX);
            float rpmf = 0.0f;
            if (get_rpm(esc_id, rpmf)) {
                rpm[j] = constrain_float(rpmf, 0, UINT16_MAX);
            } else {
                rpm[j] = 0;
            }
            count[j] = _telem_data[esc_id].count;
        }

        // use a function pointer to reduce flash space
        typedef void (*esc_telem_send_fn_t)(mavlink_channel_t, const uint8_t *, const uint16_t *, const uint16_t *, const uint16_t *, const uint16_t *, const uint16_t *);
        esc_telem_send_fn_t fn = nullptr;

        // send messages
        switch (i) {
            case 0:
                fn = mavlink_msg_esc_telemetry_1_to_4_send;
                break;
            case 1:
                fn = mavlink_msg_esc_telemetry_5_to_8_send;
                break;
            case 2:
                fn = mavlink_msg_esc_telemetry_9_to_12_send;
                break;
            case 3:
                fn = mavlink_msg_esc_telemetry_13_to_16_send;
                break;
#if ESC_TELEM_MAX_ESCS > 16
            case 4:
                fn = mavlink_msg_esc_telemetry_17_to_20_send;
                break;
            case 5:
                fn = mavlink_msg_esc_telemetry_21_to_24_send;
                break;
            case 6:
                fn = mavlink_msg_esc_telemetry_25_to_28_send;
                break;
            case 7:
                fn = mavlink_msg_esc_telemetry_29_to_32_send;
                break;
#endif
        }
        if (fn != nullptr) {
            fn((mavlink_channel_t)mav_chan, temperature, voltage, current, current_tot, rpm, count);
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

    if (esc_index >= ESC_TELEM_MAX_ESCS) {
        return;
    }

    _have_data = true;

    if (data_mask & AP_ESC_Telem_Backend::TelemetryType::TEMPERATURE) {
        _telem_data[esc_index].temperature_cdeg = new_data.temperature_cdeg;
    }
    if (data_mask & AP_ESC_Telem_Backend::TelemetryType::MOTOR_TEMPERATURE) {
        _telem_data[esc_index].motor_temp_cdeg = new_data.motor_temp_cdeg;
    }
    if (data_mask & AP_ESC_Telem_Backend::TelemetryType::VOLTAGE) {
        _telem_data[esc_index].voltage = new_data.voltage;
    }
    if (data_mask & AP_ESC_Telem_Backend::TelemetryType::CURRENT) {
        _telem_data[esc_index].current = new_data.current;
    }
    if (data_mask & AP_ESC_Telem_Backend::TelemetryType::CONSUMPTION) {
        _telem_data[esc_index].consumption_mah = new_data.consumption_mah;
    }
    if (data_mask & AP_ESC_Telem_Backend::TelemetryType::USAGE) {
        _telem_data[esc_index].usage_s = new_data.usage_s;
    }

    _telem_data[esc_index].count++;
    _telem_data[esc_index].types |= data_mask;
    _telem_data[esc_index].last_update_ms = AP_HAL::millis();
}

// record an update to the RPM together with timestamp, this allows the notch values to be slewed
// this should be called by backends when new telemetry values are available
void AP_ESC_Telem::update_rpm(const uint8_t esc_index, const uint16_t new_rpm, const float error_rate)
{
    if (esc_index >= ESC_TELEM_MAX_ESCS) {
        return;
    }

    _have_data = true;

    const uint32_t now = AP_HAL::micros();
    volatile AP_ESC_Telem_Backend::RpmData& rpmdata = _rpm_data[esc_index];

    rpmdata.prev_rpm = rpmdata.rpm;
    rpmdata.rpm = new_rpm;
    if (now > rpmdata.last_update_us) { // cope with wrapping
        rpmdata.update_rate_hz = 1.0e6f / (now - rpmdata.last_update_us);
    }
    rpmdata.last_update_us = now;
    rpmdata.error_rate = error_rate;

#ifdef ESC_TELEM_DEBUG
    hal.console->printf("RPM: rate=%.1fhz, rpm=%d)\n", rpmdata.update_rate_hz, new_rpm);
#endif
}

void AP_ESC_Telem::update()
{
    AP_Logger *logger = AP_Logger::get_singleton();

    // Push received telemetry data into the logging system
    if (logger && logger->logging_enabled()) {

        for (uint8_t i = 0; i < ESC_TELEM_MAX_ESCS; i++) {
            if (_telem_data[i].last_update_ms != _last_telem_log_ms[i]
                || _rpm_data[i].last_update_us != _last_rpm_log_us[i]) {

                float rpm = 0.0f;
                get_rpm(i, rpm);
                float rawrpm = 0.0f;
                get_raw_rpm(i, rawrpm);

                // Write ESC status messages
                //   id starts from 0
                //   rpm is eRPM (rpm * 100)
                //   voltage is in Volt
                //   current is in Ampere
                //   esc_temp is in centi-degrees Celsius
                //   current_tot is in milli-Ampere hours
                //   motor_temp is in centi-degrees Celsius
                //   error_rate is in percentage
                const struct log_Esc pkt{
                    LOG_PACKET_HEADER_INIT(uint8_t(LOG_ESC_MSG)),
                    time_us     : AP_HAL::micros64(),
                    instance    : i,
                    rpm         : (int32_t) rpm * 100,
                    raw_rpm     : (int32_t) rawrpm * 100,
                    voltage     : _telem_data[i].voltage,
                    current     : _telem_data[i].current,
                    esc_temp    : _telem_data[i].temperature_cdeg,
                    current_tot : _telem_data[i].consumption_mah,
                    motor_temp  : _telem_data[i].motor_temp_cdeg,
                    error_rate  : _rpm_data[i].error_rate
                };
                AP::logger().WriteBlock(&pkt, sizeof(pkt));
                _last_telem_log_ms[i] = _telem_data[i].last_update_ms;
                _last_rpm_log_us[i] = _rpm_data[i].last_update_us;
            }
        }
    }
}

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
