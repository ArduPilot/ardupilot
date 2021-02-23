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

#if HAL_MAX_CAN_PROTOCOL_DRIVERS
  #include <AP_CANManager/AP_CANManager.h>
  #include <AP_Common/AP_Common.h>
  #include <AP_Vehicle/AP_Vehicle.h>
  #include <AP_UAVCAN/AP_UAVCAN.h>
  #include <AP_KDECAN/AP_KDECAN.h>
  #include <AP_ToshibaCAN/AP_ToshibaCAN.h>
#endif

extern const AP_HAL::HAL& hal;

AP_ESC_Telem::AP_ESC_Telem()
{
    if (_singleton) {
        AP_HAL::panic("Too many AP_ESC_Telem instances");
    }
    _singleton = this;
}

// return the average motor frequency in Hz for dynamic filtering
float AP_ESC_Telem::get_average_motor_frequency_hz() const
{
    float motor_freq = 0.0f;
    uint8_t valid_escs = 0;

    // average the rpm of each motor and convert to Hz
    for (uint8_t i = 0; i < ESC_TELEM_MAX_ESCS; i++) {
        float rpm;
        if (get_rpm(i, rpm)) {
            motor_freq += rpm * (1.0f / 60.0f);
            valid_escs++;
        }
    }
    if (valid_escs > 0) {
        motor_freq /= valid_escs;
    }

    return motor_freq;
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

// return number of active ESCs present
uint8_t AP_ESC_Telem::get_num_active_escs() const {
    uint8_t nmotors = 0;
    uint16_t motor_mask = _active_escs_mask;
    while (motor_mask) {
        nmotors += motor_mask & 1;
        motor_mask >>= 1;
    }
    return nmotors;
}

// get an individual ESC's slewed rpm if available, returns true on success
bool AP_ESC_Telem::get_rpm(uint8_t esc_index, float& rpm) const
{
    if (esc_index > ESC_TELEM_MAX_ESCS
        || !(_active_escs_mask & (1 << esc_index))
        || is_zero(_update_rate_hz[esc_index])) {
        return false;
    }

    const uint32_t now = AP_HAL::micros();
    if (_last_rpm_update_us[esc_index] > 0 && (now - _last_rpm_update_us[esc_index] < 1000000)) {
        const float slew = MIN(1.0f, (now - _last_rpm_update_us[esc_index]) * _update_rate_hz[esc_index] * (1.0f / 1e6f));
        rpm = (_prev_rpm[esc_index] + (_curr_rpm[esc_index] - _prev_rpm[esc_index]) * slew);
        return true;
    }
    return false;
}

// get an individual ESC's temperature in degrees if available, returns true on success
bool AP_ESC_Telem::get_temperature(uint8_t esc_index, int16_t& temp) const
{
    if (esc_index >= ESC_TELEM_MAX_ESCS
        || AP_HAL::millis() - _last_telem_data_ms[esc_index] > ESC_TELEM_DATA_TIMEOUT_MS
        || !(_telem_data_mask[esc_index] & AP_ESC_Telem_Backend::TelemetryType::TEMPERATURE)) {
        return false;
    }
    temp = _telem_data[esc_index].temperature_deg;
    return true;
}

// get an individual motor's temperature in degrees if available, returns true on success
bool AP_ESC_Telem::get_motor_temperature(uint8_t esc_index, int16_t& temp) const
{
    if (esc_index >= ESC_TELEM_MAX_ESCS
        || AP_HAL::millis() - _last_telem_data_ms[esc_index] > ESC_TELEM_DATA_TIMEOUT_MS
        || !(_telem_data_mask[esc_index] & AP_ESC_Telem_Backend::TelemetryType::MOTOR_TEMPERATURE)) {
        return false;
    }
    temp = _telem_data[esc_index].motor_temp_deg;
    return true;
}

// get an individual ESC's current if available, returns true on success
bool AP_ESC_Telem::get_current_ca(uint8_t esc_index, uint16_t& amps_ca) const
{
    if (esc_index >= ESC_TELEM_MAX_ESCS
        || AP_HAL::millis() - _last_telem_data_ms[esc_index] > ESC_TELEM_DATA_TIMEOUT_MS
        || !(_telem_data_mask[esc_index] & AP_ESC_Telem_Backend::TelemetryType::CURRENT)) {
        return false;
    }
    amps_ca = _telem_data[esc_index].current_ca;
    return true;
}

// get an individual ESC's current if available, returns true on success
bool AP_ESC_Telem::get_voltage_cv(uint8_t esc_index, uint16_t& volts_cv) const
{
    if (esc_index >= ESC_TELEM_MAX_ESCS
        || AP_HAL::millis() - _last_telem_data_ms[esc_index] > ESC_TELEM_DATA_TIMEOUT_MS
        || !(_telem_data_mask[esc_index] & AP_ESC_Telem_Backend::TelemetryType::VOLTAGE)) {
        return false;
    }
    volts_cv = _telem_data[esc_index].voltage_cv;
    return true;
}

// get an individual ESC's current if available, returns true on success
bool AP_ESC_Telem::get_consumption_mah(uint8_t esc_index, uint16_t& consumption_mah) const
{
    if (esc_index >= ESC_TELEM_MAX_ESCS
        || AP_HAL::millis() - _last_telem_data_ms[esc_index] > ESC_TELEM_DATA_TIMEOUT_MS
        || !(_telem_data_mask[esc_index] & AP_ESC_Telem_Backend::TelemetryType::CONSUMPTION)) {
        return false;
    }
    consumption_mah = _telem_data[esc_index].consumption_mah;
    return true;
}

// get an individual ESC's data if available, returns true on success
bool AP_ESC_Telem::get_telem_data(uint8_t esc_index, AP_ESC_Telem_Backend::TelemetryData& data) const
{
    if (esc_index >= ESC_TELEM_MAX_ESCS
        || AP_HAL::millis() - _last_telem_data_ms[esc_index] > ESC_TELEM_DATA_TIMEOUT_MS) {
        return false;
    }
    data = _telem_data[esc_index];
    return true;
}

// get an individual ESC's current if available, returns true on success
bool AP_ESC_Telem::get_usage_seconds(uint8_t esc_index, uint32_t& usage_s) const
{
    if (esc_index >= ESC_TELEM_MAX_ESCS
        || AP_HAL::millis() - _last_telem_data_ms[esc_index] > ESC_TELEM_DATA_TIMEOUT_MS
        || !(_telem_data_mask[esc_index] & AP_ESC_Telem_Backend::TelemetryType::USAGE)) {
        return false;
    }
    usage_s = _telem_data[esc_index].usage_s;
    return true;
}

// send ESC telemetry messages over MAVLink
void AP_ESC_Telem::send_esc_telemetry_mavlink(uint8_t mav_chan)
{
    // return immediately if no ESCs have been found
    if (_active_escs_mask == 0) {
        return;
    }

    // loop through 3 groups of 4 ESCs
    for (uint8_t i = 0; i < 3; i++) {

        // return if no space in output buffer to send mavlink messages
        if (!HAVE_PAYLOAD_SPACE((mavlink_channel_t)mav_chan, ESC_TELEMETRY_1_TO_4)) {
            return;
        }

        // skip this group of ESCs if no data to send
        if ((_active_escs_mask & ((uint32_t)0x0F << i*4)) == 0) {
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
            const uint8_t esc_id = i * 4 + j;
            temperature[j] = _telem_data[esc_id].temperature_deg;
            voltage[j] = _telem_data[esc_id].voltage_cv;
            current[j] = _telem_data[esc_id].current_ca;
            current_tot[j] = constrain_float(_telem_data[esc_id].consumption_mah, 0, UINT16_MAX);
            float rpmf;
            if (get_rpm(esc_id, rpmf)) {
                rpm[j] = constrain_float(rpmf, 0, UINT16_MAX);
            } else {
                rpm[j] = 0;
            }
            count[j] = _telem_data[esc_id].count;
        }

        // send messages
        switch (i) {
            case 0:
                mavlink_msg_esc_telemetry_1_to_4_send((mavlink_channel_t)mav_chan, temperature, voltage, current, current_tot, rpm, count);
                break;
            case 1:
                mavlink_msg_esc_telemetry_5_to_8_send((mavlink_channel_t)mav_chan, temperature, voltage, current, current_tot, rpm, count);
                break;
            case 2:
                mavlink_msg_esc_telemetry_9_to_12_send((mavlink_channel_t)mav_chan, temperature, voltage, current, current_tot, rpm, count);
                break;
            default:
                break;
        }
    }
}

// record an update to the telemetry data together with timestamp
// this should be called by backends when new telemetry values are available
void AP_ESC_Telem::update_telem_data(uint8_t esc_index, const AP_ESC_Telem_Backend::TelemetryData& new_data, uint8_t data_mask)
{
    if (esc_index > ESC_TELEM_MAX_ESCS) {
        return;
    }

    _last_telem_data_ms[esc_index] = AP_HAL::millis();

    if (data_mask & AP_ESC_Telem_Backend::TelemetryType::TEMPERATURE) {
        _telem_data[esc_index].temperature_deg = new_data.temperature_deg;
    }
    if (data_mask & AP_ESC_Telem_Backend::TelemetryType::MOTOR_TEMPERATURE) {
        _telem_data[esc_index].motor_temp_deg = new_data.motor_temp_deg;
    }
    if (data_mask & AP_ESC_Telem_Backend::TelemetryType::VOLTAGE) {
        _telem_data[esc_index].voltage_cv = new_data.voltage_cv;
    }
    if (data_mask & AP_ESC_Telem_Backend::TelemetryType::CURRENT) {
        _telem_data[esc_index].current_ca = new_data.current_ca;
    }
    if (data_mask & AP_ESC_Telem_Backend::TelemetryType::CONSUMPTION) {
        _telem_data[esc_index].consumption_mah = new_data.consumption_mah;
    }
    if (data_mask & AP_ESC_Telem_Backend::TelemetryType::USAGE) {
        _telem_data[esc_index].usage_s = new_data.usage_s;
    }
    if (data_mask & AP_ESC_Telem_Backend::TelemetryType::ERROR_RATE) {
        _telem_data[esc_index].error_rate = new_data.error_rate;
    }

    _telem_data[esc_index].count++;
    _telem_data_mask[esc_index] |= data_mask;
    _active_escs_mask |= (1 << esc_index);
}

// record an update to the RPM together with timestamp, this allows the notch values to be slewed
// this should be called by backends when new telemetry values are available
void AP_ESC_Telem::update_rpm(uint8_t esc_index, uint16_t new_rpm)
{
    if (esc_index > ESC_TELEM_MAX_ESCS) {
        return;
    }
    uint32_t now = AP_HAL::micros();

    _prev_rpm[esc_index] = _curr_rpm[esc_index];
    _curr_rpm[esc_index] = new_rpm;
    _update_rate_hz[esc_index] = 1.0e6f / (now -_last_rpm_update_us[esc_index]);
    _last_rpm_update_us[esc_index] = now;
    _active_escs_mask |= (1 << esc_index);

#ifdef ESC_TELEM_DEBUG
    hal.console->printf("RPM: rate=%.1fhz, rpm=%d)\n", _update_rate_hz[esc_index], new_rpm);
#endif
}

void AP_ESC_Telem::init(void)
{
    if (_initialised) {
        return;
    }

#if HAL_MAX_CAN_PROTOCOL_DRIVERS
    const uint8_t num_drivers = AP::can().get_num_drivers();
    for (uint8_t i = 0; i < num_drivers; i++) {
        if (AP::can().get_driver_type(i) == AP_CANManager::Driver_Type_ToshibaCAN) {
            AP_ToshibaCAN *tcan = AP_ToshibaCAN::get_tcan(i);
            if (tcan != nullptr) {
                add_backend(tcan);
                break;
            }
        }
    }
#endif

    _initialised = true;
}

// log ESC telemetry at 10Hz
void AP_ESC_Telem::update()
{
    AP_Logger *logger = AP_Logger::get_singleton();

    // Push received telemtry data into the logging system
    if (logger && logger->logging_enabled()) {

        for (uint8_t i = 0; i < ESC_TELEM_MAX_ESCS; i++) {
            if ((_active_escs_mask & (1<<i)) &&  (_last_telem_data_ms[i] >  _last_telem_log_ms[i]
                || _last_rpm_update_us[i] > _last_telem_log_ms[i] * 1000)) {

                float rpm = 0.0f;
                get_rpm(i, rpm);

                logger->Write_ESC(i, AP_HAL::micros64(),
                                (int32_t) rpm * 100,
                                _telem_data[i].voltage_cv,
                                _telem_data[i].current_ca,
                                _telem_data[i].temperature_deg,
                                _telem_data[i].consumption_mah,
                                _telem_data[i].motor_temp_deg,
                                _telem_data[i].error_rate);
                _last_telem_log_ms[i] = AP_HAL::millis();
            }
        }
    }
}

bool AP_ESC_Telem::add_backend(AP_ESC_Telem_Backend *backend)
{
    if (!backend) {
        return false;
    }
    if (_backend_count == ESC_MAX_BACKENDS) {
        AP_HAL::panic("Too many ESC backends");
    }
    _backends[_backend_count++] = backend;
    return true;
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
