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
#include <AP_Common/AP_FWVersion.h>
#include <AP_OSD/AP_OSD.h>
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
    add_scheduler_entry(200, 50);   // parameters       20Hz (generally not active unless requested by the TX)
    add_scheduler_entry(50, 120);   // Attitude and compass 8Hz
    add_scheduler_entry(200, 1000); // VTX parameters    1Hz
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
        return _request_pending > 0;
    case VTX_PARAMETERS:
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
        case PARAMETERS: // update parameter settings
            update_params();
            break;
        case ATTITUDE:
            calc_attitude();
            break;
        case VTX_PARAMETERS: // update various VTX parameters
            update_vtx_params();
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

    case AP_RCProtocol_CRSF::CRSF_FRAMETYPE_PARAM_DEVICE_PING:
        process_ping_frame((ParameterPingFrame*)data);
        break;

    case AP_RCProtocol_CRSF::CRSF_FRAMETYPE_PARAMETER_READ:
        process_param_read_frame((ParameterSettingsReadFrame*)data);
        break;

    case AP_RCProtocol_CRSF::CRSF_FRAMETYPE_PARAMETER_WRITE:
        process_param_write_frame((ParameterSettingsWriteFrame*)data);
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

// request for device info
void AP_CRSF_Telem::process_ping_frame(ParameterPingFrame* ping)
{
    debug("process_ping_frame: %d -> %d", ping->origin, ping->destination);
    if (ping->destination != 0 && ping->destination != AP_RCProtocol_CRSF::CRSF_ADDRESS_FLIGHT_CONTROLLER) {
        return; // request was not for us
    }

    _param_request.origin = ping->origin;
    _request_pending = AP_RCProtocol_CRSF::CRSF_FRAMETYPE_PARAM_DEVICE_PING;
}

void AP_CRSF_Telem::process_param_read_frame(ParameterSettingsReadFrame* read_frame)
{
    //debug("process_param_read_frame: %d -> %d for %d[%d]", read_frame->origin, read_frame->destination,
    //    read_frame->param_number, read_frame->param_chunk);
    if (read_frame->destination != 0 && read_frame->destination != AP_RCProtocol_CRSF::CRSF_ADDRESS_FLIGHT_CONTROLLER) {
        return; // request was not for us
    }

    _param_request = *read_frame;
    _request_pending = AP_RCProtocol_CRSF::CRSF_FRAMETYPE_PARAMETER_READ;
}

// process any changed settings and schedule for transmission
void AP_CRSF_Telem::update()
{
}

void AP_CRSF_Telem::update_params()
{
    // handle general parameter requests
    switch (_request_pending) {
    case AP_RCProtocol_CRSF::CRSF_FRAMETYPE_PARAM_DEVICE_PING:
        calc_device_info();
        break;
    case AP_RCProtocol_CRSF::CRSF_FRAMETYPE_PARAMETER_READ:
        calc_parameter();
        break;
    default:
        break;
    }
}

void AP_CRSF_Telem::update_vtx_params()
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

// return device information about ArduPilot
void AP_CRSF_Telem::calc_device_info() {
#if !APM_BUILD_TYPE(APM_BUILD_UNKNOWN)
    _telem.ext.info.destination = _param_request.origin;
    _telem.ext.info.origin = AP_RCProtocol_CRSF::CRSF_ADDRESS_FLIGHT_CONTROLLER;

    const AP_FWVersion &fwver = AP::fwversion();
    // write out the name with version, max width is 60 - 18 = the meaning of life
    int32_t n = strlen(fwver.fw_string);
    strncpy((char*)_telem.ext.info.payload, fwver.fw_string, 41);
    n = MIN(n + 1, 42);

    put_be32_ptr(&_telem.ext.info.payload[n], // serial number
        uint32_t(fwver.major) << 24 | uint32_t(fwver.minor) << 16 | uint32_t(fwver.patch) << 8 | uint32_t(fwver.fw_type));
    n += 4;
    put_be32_ptr(&_telem.ext.info.payload[n], // hardware id
        uint32_t(fwver.vehicle_type) << 24 | uint32_t(fwver.board_type) << 16 | uint32_t(fwver.board_subtype));
    n += 4;
    put_be32_ptr(&_telem.ext.info.payload[n], fwver.os_sw_version);   // software id
    n += 4;
#if OSD_PARAM_ENABLED
    _telem.ext.info.payload[n++] = AP_OSD_ParamScreen::NUM_PARAMS * AP_OSD_NUM_PARAM_SCREENS; // param count
#else
    _telem.ext.info.payload[n++] = 0; // param count
#endif
    _telem.ext.info.payload[n++] = 0;   // param version

    _telem_size = n + 2;
    _telem_type = AP_RCProtocol_CRSF::CRSF_FRAMETYPE_PARAM_DEVICE_INFO;

    _request_pending = 0;
    _telem_pending = true;
#endif
}

// return parameter information
void AP_CRSF_Telem::calc_parameter() {
#if OSD_PARAM_ENABLED
    AP_OSD* osd = AP::osd();

    if (osd == nullptr) {
        return;
    }

    _telem.ext.param_entry.header.destination = _param_request.origin;
    _telem.ext.param_entry.header.origin = AP_RCProtocol_CRSF::CRSF_ADDRESS_FLIGHT_CONTROLLER;

    AP_OSD_ParamSetting* param = osd->get_setting((_param_request.param_num - 1) / AP_OSD_ParamScreen::NUM_PARAMS,
        (_param_request.param_num - 1) % AP_OSD_ParamScreen::NUM_PARAMS);

    if (param == nullptr) {
        return;
    }

    _telem.ext.param_entry.header.param_num = _param_request.param_num;
#if HAL_CRSF_TELEM_TEXT_SELECTION_ENABLED
    if (param->get_custom_metadata() != nullptr) {
        calc_text_selection(param, _param_request.param_chunk);
        return;
    }
#endif
    size_t idx = 0;
    _telem.ext.param_entry.header.chunks_left = 0;
    _telem.ext.param_entry.payload[idx++] = 0; // parent folder
    idx++;  // leave a gap for the type
    param->copy_name_camel_case((char*)&_telem.ext.param_entry.payload[idx], 17);
    idx += strnlen((char*)&_telem.ext.param_entry.payload[idx], 16) + 1;

    switch (param->_param_type) {
    case AP_PARAM_INT8: {
        AP_Int8* p = (AP_Int8*)param->_param;
        _telem.ext.param_entry.payload[1] = ParameterType::INT8;
        _telem.ext.param_entry.payload[idx] = p->get();  // value
        _telem.ext.param_entry.payload[idx+1] = int8_t(param->_param_min);  // min
        _telem.ext.param_entry.payload[idx+2] = int8_t(param->_param_max); // max
        _telem.ext.param_entry.payload[idx+3] = int8_t(0);  // default
        idx += 4;
        break;
    }
    case AP_PARAM_INT16: {
        AP_Int16* p = (AP_Int16*)param->_param;
        _telem.ext.param_entry.payload[1] = ParameterType::INT16;
        put_be16_ptr(&_telem.ext.param_entry.payload[idx], p->get());  // value
        put_be16_ptr(&_telem.ext.param_entry.payload[idx+2], param->_param_min);  // min
        put_be16_ptr(&_telem.ext.param_entry.payload[idx+4], param->_param_max); // max
        put_be16_ptr(&_telem.ext.param_entry.payload[idx+6], 0);  // default
        idx += 8;
        break;
    }
    case AP_PARAM_INT32: {
        AP_Int32* p = (AP_Int32*)param->_param;
        _telem.ext.param_entry.payload[1] = ParameterType::FLOAT;
#define FLOAT_ENCODE(f) (int32_t(roundf(f)))
        put_be32_ptr(&_telem.ext.param_entry.payload[idx], p->get());  // value
        put_be32_ptr(&_telem.ext.param_entry.payload[idx+4], FLOAT_ENCODE(param->_param_min));  // min
        put_be32_ptr(&_telem.ext.param_entry.payload[idx+8], FLOAT_ENCODE(param->_param_max)); // max
        put_be32_ptr(&_telem.ext.param_entry.payload[idx+12], FLOAT_ENCODE(0.0f));  // default
#undef FLOAT_ENCODE
        _telem.ext.param_entry.payload[idx+16] = 0; // decimal point
        put_be32_ptr(&_telem.ext.param_entry.payload[idx+17], 1);  // step size
        idx += 21;
        break;
    }
    case AP_PARAM_FLOAT: {
        AP_Float* p = (AP_Float*)param->_param;
        _telem.ext.param_entry.payload[1] = ParameterType::FLOAT;
        uint8_t digits = 0;
        const float incr = MAX(0.001f, param->_param_incr); // a bug in OpenTX prevents this going any smaller

        for (float floatp = incr; floatp < 1.0f; floatp *= 10) {
            digits++;
        }
        const float mult = powf(10, digits);
#define FLOAT_ENCODE(f) (int32_t(roundf(mult * f)))
        put_be32_ptr(&_telem.ext.param_entry.payload[idx], FLOAT_ENCODE(p->get()));  // value
        put_be32_ptr(&_telem.ext.param_entry.payload[idx+4], FLOAT_ENCODE(param->_param_min));  // min
        put_be32_ptr(&_telem.ext.param_entry.payload[idx+8], FLOAT_ENCODE(param->_param_max)); // max
        put_be32_ptr(&_telem.ext.param_entry.payload[idx+12], FLOAT_ENCODE(0.0f));  // default
        _telem.ext.param_entry.payload[idx+16] = digits; // decimal point
        put_be32_ptr(&_telem.ext.param_entry.payload[idx+17], FLOAT_ENCODE(incr));  // step size
#undef FLOAT_ENCODE
        //debug("Encoding param %f(%f -> %f, %f) as %d(%d) (%d -> %d, %d)", p->get(),
        //    param->_param_min.get(), param->_param_max.get(), param->_param_incr.get(),
        //    int(FLOAT_ENCODE(p->get())), digits, int(FLOAT_ENCODE(param->_param_min)),
        //    int(FLOAT_ENCODE(param->_param_max)), int(FLOAT_ENCODE(param->_param_incr)));
        idx += 21;
        break;
    }
    default:
        return;
    }
    _telem.ext.param_entry.payload[idx] = 0; // units

    _telem_size = sizeof(AP_CRSF_Telem::ParameterSettingsEntryHeader) + 1 + idx;
    _telem_type = AP_RCProtocol_CRSF::CRSF_FRAMETYPE_PARAMETER_SETTINGS_ENTRY;

    _request_pending = 0;
    _telem_pending = true;
#endif
}

#if HAL_CRSF_TELEM_TEXT_SELECTION_ENABLED
// class that spits out a chunk of data from a larger stream of contiguous chunks
// the caller describes which chunk it needs and provides this class with all of the data
// data is not written until the start position is reached and after a whole chunk
// is accumulated the rest of the data is skipped in order to determine how many chunks
// are left to be sent
class BufferChunker {
public:
    BufferChunker(uint8_t* buf, uint16_t chunk_size, uint16_t start_chunk) :
        _buf(buf), _idx(0), _start_chunk(start_chunk), _chunk_size(chunk_size), _chunk(0), _bytes(0) {
    }

    // accumulate a string, writing to the underlying buffer as required
    void put_string(const char* str, uint16_t str_len) {
        // skip over data we have already written or have yet to write
        if (_chunk != _start_chunk) {
            if (skip_bytes(str_len)) {
                // partial write
                strncpy((char*)_buf, &str[str_len - _idx], _idx);
                _bytes += _idx;
            }
            return;
        }

        uint16_t rem = remaining();
        if (rem > str_len) {
            strncpy_noterm((char*)&_buf[_idx], str, str_len);
            _idx += str_len;
            _bytes += str_len;
        } else {
            strncpy_noterm((char*)&_buf[_idx], str, rem);
            _chunk++;
            _idx += str_len;
            _bytes += rem;
            _idx %= _chunk_size;
        }
    }

    // accumulate a byte, writing to the underlying buffer as required
    void put_byte(uint8_t b) {
        if (_chunk != _start_chunk) {
            if (skip_bytes(1)) {
                _buf[0] = b;
                _bytes++;
            }
            return;
        }
        if (remaining() > 0) {
            _buf[_idx++] = b;
            _bytes++;
        } else {
            _chunk++;
            _idx = 0;
        }
    }

    uint8_t chunks_remaining() const { return _chunk - _start_chunk; }
    uint8_t bytes_written() const { return _bytes; }

private:
    uint16_t remaining() const { return _chunk_size - _bytes; }

    // skip over the requested number of bytes
    // returns true if we overflow into a chunk that needs to be written
    bool skip_bytes(uint16_t len) {
        _idx += len;
        if (_idx >= _chunk_size) {
            _chunk++;
            _idx %= _chunk_size;
            // partial write
            if (_chunk == _start_chunk && _idx > 0) {
                return true;
            }
        }
        return false;
    }

    uint8_t* _buf;
    uint16_t _idx;
    uint16_t _bytes;
    uint8_t _chunk;
    const uint16_t _start_chunk;
    const uint16_t _chunk_size;
};

// provide information about a text selection, possibly over multiple chunks
void AP_CRSF_Telem::calc_text_selection(AP_OSD_ParamSetting* param, uint8_t chunk)
{
    const uint8_t CHUNK_SIZE = 56;
    const AP_OSD_ParamSetting::ParamMetadata* metadata = param->get_custom_metadata();

    // chunk the output
    BufferChunker chunker(_telem.ext.param_entry.payload, CHUNK_SIZE, chunk);

    chunker.put_byte(0);  // parent folder
    chunker.put_byte(ParameterType::TEXT_SELECTION);  // parameter type

    char name[17];
    param->copy_name_camel_case(name, 17);
    chunker.put_string(name, strnlen(name, 16)); // parameter name
    chunker.put_byte(0);  // trailing null

    for (uint8_t i = 0; i < metadata->values_max; i++) {
        uint8_t len = strnlen(metadata->values[i], 16);
        if (len == 0) {
            chunker.put_string("---", 3);
        } else {
            chunker.put_string(metadata->values[i], len);
        }
        if (i == metadata->values_max - 1) {
            chunker.put_byte(0);
        } else {
            chunker.put_byte(';');
        }
    }

    int32_t val = -1;
    switch (param->_param_type) {
    case AP_PARAM_INT8:
        val = ((AP_Int8*)param->_param)->get();
        break;
    case AP_PARAM_INT16:
        val = ((AP_Int16*)param->_param)->get();
        break;
    case AP_PARAM_INT32:
        val = ((AP_Int32*)param->_param)->get();
        break;
    default:
        return;
    }

    // out of range values really confuse the TX
    val = constrain_int16(val, 0, metadata->values_max - 1);
    chunker.put_byte(val);  // value
    chunker.put_byte(0);  // min
    chunker.put_byte(metadata->values_max); // max
    chunker.put_byte(0);  // default
    chunker.put_byte(0); // units

    _telem.ext.param_entry.header.chunks_left = chunker.chunks_remaining();

    _telem_size = sizeof(AP_CRSF_Telem::ParameterSettingsEntryHeader) + chunker.bytes_written();
    _telem_type = AP_RCProtocol_CRSF::CRSF_FRAMETYPE_PARAMETER_SETTINGS_ENTRY;

    _request_pending = 0;
    _telem_pending = true;
}
#endif

// write parameter information back into AP - assumes we already know the encoding for floats
void AP_CRSF_Telem::process_param_write_frame(ParameterSettingsWriteFrame* write_frame)
{
    debug("process_param_write_frame: %d -> %d", write_frame->origin, write_frame->destination);
    if (write_frame->destination != AP_RCProtocol_CRSF::CRSF_ADDRESS_FLIGHT_CONTROLLER) {
        return; // request was not for us
    }
#if OSD_PARAM_ENABLED
    AP_OSD* osd = AP::osd();

    if (osd == nullptr) {
        return;
    }

    AP_OSD_ParamSetting* param = osd->get_setting((write_frame->param_num - 1) / AP_OSD_ParamScreen::NUM_PARAMS,
        (write_frame->param_num - 1) % AP_OSD_ParamScreen::NUM_PARAMS);

    if (param == nullptr) {
        return;
    }

#if HAL_CRSF_TELEM_TEXT_SELECTION_ENABLED
    bool text_selection = param->get_custom_metadata() != nullptr;
#else
    bool text_selection = false;
#endif

    switch (param->_param_type) {
    case AP_PARAM_INT8: {
        AP_Int8* p = (AP_Int8*)param->_param;
        p->set_and_save(write_frame->payload[0]);
        break;
    }
    case AP_PARAM_INT16: {
        AP_Int16* p = (AP_Int16*)param->_param;
        if (text_selection) {
            // if we have custom metadata then the parameter is a text selection
            p->set_and_save(write_frame->payload[0]);
        } else {
            p->set_and_save(be16toh_ptr(write_frame->payload));
        }
        break;
    }
    case AP_PARAM_INT32: {
        AP_Int32* p = (AP_Int32*)param->_param;
        if (text_selection) {
            // if we have custom metadata then the parameter is a text selection
            p->set_and_save(write_frame->payload[0]);
        } else {
            p->set_and_save(be32toh_ptr(write_frame->payload));
        }
        break;
    }
    case AP_PARAM_FLOAT: {
        AP_Float* p = (AP_Float*)param->_param;
        const int32_t val = be32toh_ptr(write_frame->payload);
        uint8_t digits = 0;
        const float incr = MAX(0.001f, param->_param_incr); // a bug in OpenTX prevents this going any smaller

        for (float floatp = incr; floatp < 1.0f; floatp *= 10) {
            digits++;
        }
        p->set_and_save(float(val) / powf(10, digits));
        break;
    }
    default:
        break;
    }
#endif
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
