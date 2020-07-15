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

#include "AP_CRSF_Telem.h"
#include "AP_VideoTX.h"
#include <AP_HAL/utility/sparse-endian.h>
#include <AP_BattMonitor/AP_BattMonitor.h>
#include <AP_Common/AP_FWVersion.h>
#include <AP_GPS/AP_GPS.h>
#include <GCS_MAVLink/GCS.h>
#include <AP_RCProtocol/AP_RCProtocol_CRSF.h>
#include <AP_SerialManager/AP_SerialManager.h>
#include <AP_AHRS/AP_AHRS.h>
#include <AP_Notify/AP_Notify.h>
#include <math.h>
#include <stdio.h>

#if HAL_CRSF_TELEM_ENABLED

// #define CRSF_DEBUG
#ifdef CRSF_DEBUG
# define debug(fmt, args...)	hal.console->printf("CRSF: " fmt "\n", ##args)
#else
# define debug(fmt, args...)	do {} while(0)
#endif

extern const AP_HAL::HAL& hal;

AP_CRSF_Telem *AP_CRSF_Telem::singleton;

AP_CRSF_Telem::AP_CRSF_Telem() : AP_RCTelemetry(0)
{
    singleton = this;
}

AP_CRSF_Telem::~AP_CRSF_Telem(void)
{
    singleton = nullptr;
}

bool AP_CRSF_Telem::init(void)
{
    // sanity check that we are using a UART for RC input
    if (!AP::serialmanager().find_serial(AP_SerialManager::SerialProtocol_RCIN, 0)
        && !AP::serialmanager().find_serial(AP_SerialManager::SerialProtocol_CRSF, 0)) {
        return false;
    }
    return AP_RCTelemetry::init();
}

/*
  setup ready for passthrough telem
 */
void AP_CRSF_Telem::setup_wfq_scheduler(void)
{
    // initialize packet weights for the WFQ scheduler
    // priority[i] = 1/_scheduler.packet_weight[i]
    // rate[i] = LinkRate * ( priority[i] / (sum(priority[1-n])) )

    // CSRF telemetry rate is 150Hz (4ms) max, so these rates must fit
    add_scheduler_entry(50, 100);   // heartbeat        10Hz
    add_scheduler_entry(50, 120);   // Attitude and compass 8Hz
    add_scheduler_entry(200, 1000); // parameters        1Hz
    add_scheduler_entry(1300, 500); // battery           2Hz
    add_scheduler_entry(550, 280);  // GPS               3Hz
    add_scheduler_entry(550, 500);  // flight mode       2Hz
}

void AP_CRSF_Telem::adjust_packet_weight(bool queue_empty)
{
}

// WFQ scheduler
bool AP_CRSF_Telem::is_packet_ready(uint8_t idx, bool queue_empty)
{
    switch (idx) {
    case PARAMETERS:
        return AP::vtx().have_params_changed() ||_vtx_power_change_pending || _vtx_freq_change_pending || _vtx_options_change_pending;
    default:
        return _enable_telemetry;
    }
}

// WFQ scheduler
void AP_CRSF_Telem::process_packet(uint8_t idx)
{
    // send packet
    switch (idx) {
        case HEARTBEAT: // HEARTBEAT
            calc_heartbeat();
            break;
        case ATTITUDE:
            calc_attitude();
            break;
        case PARAMETERS: // update various parameters
            update_params();
            break;
        case BATTERY: // BATTERY
            calc_battery();
            break;
        case GPS: // GPS
            calc_gps();
            break;
        case FLIGHT_MODE: // GPS
            calc_flight_mode();
            break;
        default:
            break;
    }
}

// Process a frame from the CRSF protocol decoder
bool AP_CRSF_Telem::_process_frame(AP_RCProtocol_CRSF::FrameType frame_type, void* data) {
    switch (frame_type) {
    // this means we are connected to an RC receiver and can send telemetry
    case AP_RCProtocol_CRSF::CRSF_FRAMETYPE_RC_CHANNELS_PACKED:
    // the EVO sends battery frames and we should send telemetry back to populate the OSD
    case AP_RCProtocol_CRSF::CRSF_FRAMETYPE_BATTERY_SENSOR:
        _enable_telemetry = true;
        break;

    case AP_RCProtocol_CRSF::CRSF_FRAMETYPE_VTX:
        process_vtx_frame((VTXFrame*)data);
        break;

    case AP_RCProtocol_CRSF::CRSF_FRAMETYPE_VTX_TELEM:
        process_vtx_telem_frame((VTXTelemetryFrame*)data);
        break;

    default:
        break;
    }
    return true;
}

void AP_CRSF_Telem::process_vtx_frame(VTXFrame* vtx) {
    vtx->user_frequency = be16toh(vtx->user_frequency);

    debug("VTX: SmartAudio: %d, Avail: %d, FreqMode: %d, Band: %d, Channel: %d, Freq: %d, PitMode: %d, Pwr: %d, Pit: %d",
        vtx->smart_audio_ver, vtx->is_vtx_available, vtx->is_in_user_frequency_mode,
        vtx->band, vtx->channel, vtx->is_in_user_frequency_mode ? vtx->user_frequency : AP_VideoTX::get_frequency_mhz(vtx->band, vtx->channel),
        vtx->is_in_pitmode, vtx->power, vtx->pitmode);
    AP_VideoTX& apvtx = AP::vtx();

    apvtx.set_enabled(vtx->is_vtx_available);
    apvtx.set_band(vtx->band);
    apvtx.set_channel(vtx->channel);
    if (vtx->is_in_user_frequency_mode) {
        apvtx.set_frequency_mhz(vtx->user_frequency);
    } else {
        apvtx.set_frequency_mhz(AP_VideoTX::get_frequency_mhz(vtx->band, vtx->channel));
    }
    // 14dBm (25mW), 20dBm (100mW), 26dBm (400mW), 29dBm (800mW)
    switch (vtx->power) {
        case 0:
            apvtx.set_power_mw(25);
            break;
        case 1:
            apvtx.set_power_mw(200);
            break;
        case 2:
            apvtx.set_power_mw(500);
            break;
        case 3:
            apvtx.set_power_mw(800);
            break;
    }
    if (vtx->is_in_pitmode) {
        apvtx.set_options(apvtx.get_options() | uint8_t(AP_VideoTX::VideoOptions::VTX_PITMODE));
    } else {
        apvtx.set_options(apvtx.get_options() & ~uint8_t(AP_VideoTX::VideoOptions::VTX_PITMODE));
    }
    // make sure the configured values now reflect reality
    apvtx.set_defaults();

    _vtx_power_change_pending = _vtx_freq_change_pending = _vtx_options_change_pending = false;
}

void AP_CRSF_Telem::process_vtx_telem_frame(VTXTelemetryFrame* vtx) {
    vtx->frequency = be16toh(vtx->frequency);
    debug("VTXTelemetry: Freq: %d, PitMode: %d, Power: %d", vtx->frequency, vtx->pitmode, vtx->power);

    AP_VideoTX& apvtx = AP::vtx();

    apvtx.set_frequency_mhz(vtx->frequency);

    AP_VideoTX::VideoBand band;
    uint8_t channel;
    if (AP_VideoTX::get_band_and_channel(vtx->frequency, band, channel)) {
        apvtx.set_band(uint8_t(band));
        apvtx.set_channel(channel);
    }

    apvtx.set_power_dbm(vtx->power);

    if (vtx->pitmode) {
        apvtx.set_options(apvtx.get_options() | uint8_t(AP_VideoTX::VideoOptions::VTX_PITMODE));
    } else {
        apvtx.set_options(apvtx.get_options() & ~uint8_t(AP_VideoTX::VideoOptions::VTX_PITMODE));
    }
    // make sure the configured values now reflect reality
    apvtx.set_defaults();

    _vtx_power_change_pending = _vtx_freq_change_pending = _vtx_options_change_pending = false;
}

// process any changed settings and schedule for transmission
void AP_CRSF_Telem::update()
{
}

void AP_CRSF_Telem::update_params()
{
    AP_VideoTX& vtx = AP::vtx();

    _vtx_freq_change_pending = vtx.update_band() || vtx.update_channel() || _vtx_freq_change_pending;
    _vtx_power_change_pending = vtx.update_power() || _vtx_power_change_pending;
    _vtx_options_change_pending = vtx.update_options() || _vtx_options_change_pending;

    if (_vtx_freq_change_pending || _vtx_power_change_pending || _vtx_options_change_pending) {
        debug("update_params(): freq %d->%d, chan: %d->%d, band: %d->%d, pwr: %d->%d, opts: %d->%d",
            vtx.get_frequency_mhz(),
            AP_VideoTX::get_frequency_mhz(vtx.get_configured_band(), vtx.get_configured_channel()),
            vtx.get_channel(), vtx.get_configured_channel(),
            vtx.get_band(), vtx.get_configured_band(),
            vtx.get_power_mw(), vtx.get_configured_power_mw(),
            vtx.get_options(), vtx.get_configured_options());

        _telem_type = AP_RCProtocol_CRSF::CRSF_FRAMETYPE_COMMAND;
        _telem.ext.command.destination = AP_RCProtocol_CRSF::CRSF_ADDRESS_VTX;
        _telem.ext.command.origin = AP_RCProtocol_CRSF::CRSF_ADDRESS_FLIGHT_CONTROLLER;
        _telem.ext.command.command_id = AP_RCProtocol_CRSF::CRSF_COMMAND_VTX;

        // make the desired frequency match the desired band and channel
        if (_vtx_freq_change_pending) {
            vtx.set_frequency_mhz(AP_VideoTX::get_frequency_mhz(vtx.get_configured_band(), vtx.get_configured_channel()));
        }

        uint8_t len = 5;
        if (_vtx_freq_change_pending && _vtx_freq_update) {
            _telem.ext.command.payload[0] = AP_RCProtocol_CRSF::CRSF_COMMAND_VTX_FREQ;
            _telem.ext.command.payload[1] = (vtx.get_frequency_mhz() & 0xFF00) >> 8;
            _telem.ext.command.payload[2] = (vtx.get_frequency_mhz() & 0xFF);
            _vtx_freq_update = false;
            len++;
        } else if (_vtx_freq_change_pending) {
            _telem.ext.command.payload[0] = AP_RCProtocol_CRSF::CRSF_COMMAND_VTX_CHANNEL;
            _telem.ext.command.payload[1] = vtx.get_configured_band() * VTX_MAX_CHANNELS + vtx.get_configured_channel();
            _vtx_freq_update = true;
        } else if (_vtx_power_change_pending && _vtx_dbm_update) {
            _telem.ext.command.payload[0] = AP_RCProtocol_CRSF::CRSF_COMMAND_VTX_POWER_DBM;
            _telem.ext.command.payload[1] = vtx.get_configured_power_dbm();
            _vtx_dbm_update = false;
        } else if (_vtx_power_change_pending) {
            _telem.ext.command.payload[0] = AP_RCProtocol_CRSF::CRSF_COMMAND_VTX_POWER;
            if (vtx.get_configured_power_mw() < 26) {
                vtx.set_configured_power_mw(25);
                _telem.ext.command.payload[1] = 0;
            } else if (vtx.get_configured_power_mw() < 201) {
                if (vtx.get_configured_power_mw() < 101) {
                    vtx.set_configured_power_mw(100);
                } else {
                    vtx.set_configured_power_mw(200);
                }
                _telem.ext.command.payload[1] = 1;
            } else if (vtx.get_configured_power_mw() < 501) {
                if (vtx.get_configured_power_mw() < 401) {
                    vtx.set_configured_power_mw(400);
                } else {
                    vtx.set_configured_power_mw(500);
                }
                _telem.ext.command.payload[1] = 2;
            } else {
                vtx.set_configured_power_mw(800);
                _telem.ext.command.payload[1] = 3;
            }
            _vtx_dbm_update = true;
        } else if (_vtx_options_change_pending) {
            _telem.ext.command.payload[0] = AP_RCProtocol_CRSF::CRSF_COMMAND_VTX_PITMODE;
            if (vtx.get_configured_options() & uint8_t(AP_VideoTX::VideoOptions::VTX_PITMODE)) {
                _telem.ext.command.payload[1] = 1;
            } else {
                _telem.ext.command.payload[1] = 0;
            }
        }
        _telem_pending = true;
        // calculate command crc
        uint8_t* crcptr = &_telem.ext.command.destination;
        uint8_t crc = crc8_dvb(0, AP_RCProtocol_CRSF::CRSF_FRAMETYPE_COMMAND, 0xBA);
        for (uint8_t i = 0; i < len; i++) {
            crc = crc8_dvb(crc, crcptr[i], 0xBA);
        }
        crcptr[len] = crc;
        _telem_size = len + 1;
    }
}

// prepare parameter ping data
void AP_CRSF_Telem::calc_parameter_ping()
{
    _telem_type = AP_RCProtocol_CRSF::CRSF_FRAMETYPE_PARAM_DEVICE_PING;
    _telem.ext.ping.destination = AP_RCProtocol_CRSF::CRSF_ADDRESS_VTX;
    _telem.ext.ping.origin = AP_RCProtocol_CRSF::CRSF_ADDRESS_FLIGHT_CONTROLLER;
    _telem_size = sizeof(ParameterPingFrame);
    _telem_pending = true;
}

// prepare qos data - mandatory frame that must be sent periodically
void AP_CRSF_Telem::calc_heartbeat()
{
    _telem.bcast.heartbeat.origin = AP_RCProtocol_CRSF::CRSF_ADDRESS_FLIGHT_CONTROLLER;
    _telem_size = sizeof(HeartbeatFrame);
    _telem_type = AP_RCProtocol_CRSF::CRSF_FRAMETYPE_HEARTBEAT;
    _telem_pending = true;
}

// prepare battery data
void AP_CRSF_Telem::calc_battery()
{
    const AP_BattMonitor &_battery = AP::battery();

    _telem.bcast.battery.voltage = htobe16(uint16_t(roundf(_battery.voltage(0) * 10.0f)));

    float current;
    if (!_battery.current_amps(current, 0)) {
        current = 0;
    }
    _telem.bcast.battery.current = htobe16(int16_t(roundf(current * 10.0f)));

    float used_mah;
    if (!_battery.consumed_mah(used_mah, 0)) {
        used_mah = 0;
    }

    _telem.bcast.battery.remaining = _battery.capacity_remaining_pct(0);

    const int32_t capacity = used_mah;
    _telem.bcast.battery.capacity[0] = (capacity & 0xFF0000) >> 16;
    _telem.bcast.battery.capacity[1] = (capacity & 0xFF00) >> 8;
    _telem.bcast.battery.capacity[2] = (capacity & 0xFF);

    _telem_size = sizeof(BatteryFrame);
    _telem_type = AP_RCProtocol_CRSF::CRSF_FRAMETYPE_BATTERY_SENSOR;

    _telem_pending = true;
}

// prepare gps data
void AP_CRSF_Telem::calc_gps()
{
    const Location &loc = AP::gps().location(0); // use the first gps instance (same as in send_mavlink_gps_raw)

    _telem.bcast.gps.latitude = htobe32(loc.lat);
    _telem.bcast.gps.longitude = htobe32(loc.lng);
    _telem.bcast.gps.groundspeed = htobe16(roundf(AP::gps().ground_speed() * 100000 / 3600));
    _telem.bcast.gps.altitude = htobe16(constrain_int16(loc.alt / 100, 0, 5000) + 1000);
    _telem.bcast.gps.gps_heading = htobe16(roundf(AP::gps().ground_course() * 100.0f));
    _telem.bcast.gps.satellites = AP::gps().num_sats();

    _telem_size = sizeof(AP_CRSF_Telem::GPSFrame);
    _telem_type = AP_RCProtocol_CRSF::CRSF_FRAMETYPE_GPS;

    _telem_pending = true;
}

// prepare attitude data
void AP_CRSF_Telem::calc_attitude()
{
    AP_AHRS &_ahrs = AP::ahrs();
    WITH_SEMAPHORE(_ahrs.get_semaphore());

    const int16_t INT_PI = 31415;
    // units are radians * 10000
    _telem.bcast.attitude.roll_angle = htobe16(constrain_int16(roundf(wrap_PI(_ahrs.roll) * 10000.0f), -INT_PI, INT_PI));
    _telem.bcast.attitude.pitch_angle = htobe16(constrain_int16(roundf(wrap_PI(_ahrs.pitch) * 10000.0f), -INT_PI, INT_PI));
    _telem.bcast.attitude.yaw_angle = htobe16(constrain_int16(roundf(wrap_PI(_ahrs.yaw) * 10000.0f), -INT_PI, INT_PI));

    _telem_size = sizeof(AP_CRSF_Telem::AttitudeFrame);
    _telem_type = AP_RCProtocol_CRSF::CRSF_FRAMETYPE_ATTITUDE;

    _telem_pending = true;
}

// prepare flight mode data
void AP_CRSF_Telem::calc_flight_mode()
{
    AP_Notify * notify = AP_Notify::get_singleton();
    if (notify) {
        hal.util->snprintf(_telem.bcast.flightmode.flight_mode, 16, "%s", notify->get_flight_mode_str());

        _telem_size = sizeof(AP_CRSF_Telem::FlightModeFrame);
        _telem_type = AP_RCProtocol_CRSF::CRSF_FRAMETYPE_FLIGHT_MODE;
        _telem_pending = true;
    }
}

/*
  fetch CRSF frame data
 */
bool AP_CRSF_Telem::_get_telem_data(AP_RCProtocol_CRSF::Frame* data)
{
    memset(&_telem, 0, sizeof(TelemetryPayload));
    run_wfq_scheduler();
    if (!_telem_pending) {
        return false;
    }
    memcpy(data->payload, &_telem, _telem_size);
    data->device_address = AP_RCProtocol_CRSF::CRSF_ADDRESS_FLIGHT_CONTROLLER;  // sync byte
    data->length = _telem_size + 2;
    data->type = _telem_type;

    _telem_pending = false;

    return true;
}

/*
  fetch data for an external transport, such as CRSF
 */
bool AP_CRSF_Telem::process_frame(AP_RCProtocol_CRSF::FrameType frame_type, void* data)
{
    if (!get_singleton()) {
        return false;
    }
    return singleton->_process_frame(frame_type, data);
}

/*
  fetch data for an external transport, such as CRSF
 */
bool AP_CRSF_Telem::get_telem_data(AP_RCProtocol_CRSF::Frame* data)
{
    if (!get_singleton()) {
        return false;
    }
    return singleton->_get_telem_data(data);
}

AP_CRSF_Telem *AP_CRSF_Telem::get_singleton(void) {
    if (!singleton && !hal.util->get_soft_armed()) {
        // if telem data is requested when we are disarmed and don't
        // yet have a AP_CRSF_Telem object then try to allocate one
        new AP_CRSF_Telem();
        // initialize the passthrough scheduler
        if (singleton) {
            singleton->init();
        }
    }
    return singleton;
}

namespace AP {
    AP_CRSF_Telem *crsf_telem() {
        return AP_CRSF_Telem::get_singleton();
    }
};

#endif
