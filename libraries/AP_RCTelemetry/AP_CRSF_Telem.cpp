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
#include <AP_VideoTX/AP_VideoTX.h>
#include <AP_HAL/utility/sparse-endian.h>
#include <AP_BattMonitor/AP_BattMonitor.h>
#include <AP_Common/AP_FWVersion.h>
#include <AP_GPS/AP_GPS.h>
#include <GCS_MAVLink/GCS.h>
#include <AP_RCProtocol/AP_RCProtocol_CRSF.h>
#include <AP_SerialManager/AP_SerialManager.h>
#include <AP_AHRS/AP_AHRS.h>
#include <AP_Notify/AP_Notify.h>
#include <AP_OSD/AP_OSD.h>
#include <AP_Frsky_Telem/AP_Frsky_SPort_Passthrough.h>
#include <math.h>
#include <stdio.h>
#include <AP_HAL/AP_HAL.h>

#if HAL_CRSF_TELEM_ENABLED

//#define CRSF_DEBUG
//#define CRSF_DEBUG_VTX
#ifdef CRSF_DEBUG
# define debug(fmt, args...)	hal.console->printf("CRSF: " fmt "\n", ##args)
#else
# define debug(fmt, args...)	do {} while(0)
#endif
#if defined(CRSF_DEBUG_VTX) || defined(CRSF_DEBUG)
# define debug_vtx(fmt, args...)	hal.console->printf("CRSF: " fmt "\n", ##args)
#else
# define debug_vtx(fmt, args...)	do {} while(0)
#endif

extern const AP_HAL::HAL& hal;
const uint8_t AP_CRSF_Telem::PASSTHROUGH_STATUS_TEXT_FRAME_MAX_SIZE;
const uint8_t AP_CRSF_Telem::PASSTHROUGH_MULTI_PACKET_FRAME_MAX_SIZE;
const uint8_t AP_CRSF_Telem::CRSF_RX_DEVICE_PING_MAX_RETRY;

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
    if (!AP::serialmanager().have_serial(AP_SerialManager::SerialProtocol_RCIN, 0)
        && !AP::serialmanager().have_serial(AP_SerialManager::SerialProtocol_CRSF, 0)) {
        return false;
    }

    // if we are flying blind force an update for everything
    if (ignore_vtx_status()) {
        _vtx_freq_change_pending = _vtx_power_change_pending = _vtx_options_change_pending = VTX_SETTINGS_RETRIES;
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
    add_scheduler_entry(5, 20);     // parameters       50Hz (generally not active unless requested by the TX)
    add_scheduler_entry(50, 120);   // Attitude and compass 8Hz
    add_scheduler_entry(200, 1000); // VTX parameters    1Hz
    add_scheduler_entry(1300, 500); // battery           2Hz
    add_scheduler_entry(550, 280);  // GPS               3Hz
    add_scheduler_entry(550, 500);  // flight mode       2Hz
    add_scheduler_entry(5000, 100); // passthrough       max 10Hz
    add_scheduler_entry(5000, 500); // status text       max 2Hz
    add_scheduler_entry(5, 20);     // command          50Hz (generally not active unless requested by the TX)
    add_scheduler_entry(5, 500);    // version ping      2Hz (only active at startup)
    add_scheduler_entry(5, 100);    // device ping      10Hz (only active during TX loss, also see CRSF_RX_TIMEOUT)
    disable_scheduler_entry(DEVICE_PING);
}

void AP_CRSF_Telem::setup_custom_telemetry()
{
    if (_custom_telem.init_done) {
        return;
    }

    if (!rc().crsf_custom_telemetry()) {
        return;
    }

    // check if passthru already assigned
    const int8_t frsky_port = AP::serialmanager().find_portnum(AP_SerialManager::SerialProtocol_FrSky_SPort_Passthrough,0);
    if (frsky_port != -1) {
        gcs().send_text(MAV_SEVERITY_CRITICAL, "%s: passthrough telemetry conflict on SERIAL%d", get_protocol_string(), frsky_port);
       _custom_telem.init_done = true;
       return;
    }

    // we need crossfire firmware version
    if (_crsf_version.pending) {
        return;
    }

    AP_Frsky_SPort_Passthrough* passthrough = AP::frsky_passthrough_telem();
    if (passthrough == nullptr) {
        return;
    }

    // setup the frsky scheduler for crossfire and elrs
    passthrough->disable_scheduler_entry(AP_Frsky_SPort_Passthrough::GPS_LAT);
    passthrough->disable_scheduler_entry(AP_Frsky_SPort_Passthrough::GPS_LON);
    passthrough->disable_scheduler_entry(AP_Frsky_SPort_Passthrough::TEXT);
    passthrough->set_scheduler_entry_min_period(AP_Frsky_SPort_Passthrough::ATTITUDE, 350); // 3Hz

    // setup the crossfire scheduler for custom telemetry
    set_scheduler_entry(FLIGHT_MODE, 1200, 2000);   // 0.5Hz
    set_scheduler_entry(HEARTBEAT, 2000, 5000);     // 0.2Hz

    _telem_rf_mode = get_rf_mode();
    // setup custom telemetry for current rf_mode
    update_custom_telemetry_rates(_telem_rf_mode);

    gcs().send_text(MAV_SEVERITY_DEBUG,"%s: custom telem init done, fw %d.%02d", get_protocol_string(), _crsf_version.major, _crsf_version.minor);

    _custom_telem.init_done = true;
}

void AP_CRSF_Telem::update_custom_telemetry_rates(AP_RCProtocol_CRSF::RFMode rf_mode)
{
    // ignore rf mode changes if we are processing parameter packets
    if (_custom_telem.disable_telemetry_timeout_ms > 0) {
        return;
    }

    if (is_high_speed_telemetry(rf_mode)) {
        // standard telemetry for high data rates
        set_scheduler_entry(BATTERY, 1000, 1000);       // 1Hz
        set_scheduler_entry(ATTITUDE, 1000, 1000);      // 1Hz
        // custom telemetry for high data rates
        set_scheduler_entry(GPS, 550, 500);            // 2.0Hz
        set_scheduler_entry(PASSTHROUGH, 100, 100);    // 8Hz
        set_scheduler_entry(STATUS_TEXT, 200, 750);    // 1.5Hz
    } else {
        // standard telemetry for low data rates
        set_scheduler_entry(BATTERY, 1000, 2000);       // 0.5Hz
        set_scheduler_entry(ATTITUDE, 1000, 3000);      // 0.33Hz
        if (_crsf_version.is_elrs) {
            // ELRS custom telemetry for low data rates
            set_scheduler_entry(GPS, 550, 1000);            // 1.0Hz
            set_scheduler_entry(PASSTHROUGH, 350, 500);     // 2.0Hz
            set_scheduler_entry(STATUS_TEXT, 500, 2000);    // 0.5Hz
        } else {
            // CRSF custom telemetry for low data rates
            set_scheduler_entry(GPS, 550, 1000);              // 1.0Hz
            set_scheduler_entry(PASSTHROUGH, 500, 3000);      // 0.3Hz
            set_scheduler_entry(STATUS_TEXT, 600, 2000);      // 0.5Hz
        }
    }
}

bool AP_CRSF_Telem::process_rf_mode_changes()
{
    const AP_RCProtocol_CRSF::RFMode current_rf_mode = get_rf_mode();
    uint32_t now = AP_HAL::millis();

    // the presence of a uart indicates that we are using CRSF for RC control
    AP_RCProtocol_CRSF* crsf = AP::crsf();
    AP_HAL::UARTDriver* uart = nullptr;
    if (crsf != nullptr) {
        uart = crsf->get_UART();
    }
    if (uart == nullptr) {
        return true;
    }
    // not ready yet
    if (!uart->is_initialized()) {
        return false;
    }

    // warn the user if their setup is sub-optimal
    if (_telem_bootstrap_msg_pending && !uart->is_dma_enabled()) {
        gcs().send_text(MAV_SEVERITY_WARNING, "%s: running on non-DMA serial port", get_protocol_string());
    }
    // note if option was set to show LQ in place of RSSI
    bool current_lq_as_rssi_active = bool(rc().use_crsf_lq_as_rssi());
    if(_telem_bootstrap_msg_pending || _noted_lq_as_rssi_active != current_lq_as_rssi_active){
        _noted_lq_as_rssi_active = current_lq_as_rssi_active;
        gcs().send_text(MAV_SEVERITY_INFO, "%s: RSSI now displays %s", get_protocol_string(), current_lq_as_rssi_active ? " as LQ" : "normally");
    }
    _telem_bootstrap_msg_pending = false;

    const bool is_high_speed = is_high_speed_telemetry(current_rf_mode);
    if ((now - _telem_last_report_ms > 5000)) {
        // report an RF mode change or a change in telemetry rate if we haven't done so in the last 5s
        if (!rc().suppress_crsf_message() && (_telem_rf_mode != current_rf_mode || abs(int16_t(_telem_last_avg_rate) - int16_t(_scheduler.avg_packet_rate)) > 25)) {
            gcs().send_text(MAV_SEVERITY_INFO, "%s: RF Mode %d, telemetry rate is %dHz", get_protocol_string(), uint8_t(current_rf_mode) - (_crsf_version.is_elrs ? uint8_t(AP_RCProtocol_CRSF::RFMode::ELRS_RF_MODE_4HZ) : 0), get_telemetry_rate());
        }
        // tune the scheduler based on telemetry speed high/low transitions
        if (_telem_is_high_speed != is_high_speed) {
            update_custom_telemetry_rates(current_rf_mode);
        }
        _telem_is_high_speed = is_high_speed;
        _telem_rf_mode = current_rf_mode;
        _telem_last_avg_rate = _scheduler.avg_packet_rate;
        if (_telem_last_report_ms == 0) {   // only want to show bootstrap messages once
            _telem_bootstrap_msg_pending = true;
        }
        _telem_last_report_ms = now;
    }
    return true;
}

// return custom frame id based on fw version
uint8_t AP_CRSF_Telem::get_custom_telem_frame_id() const
{
    if (!_crsf_version.pending &&
        ((_crsf_version.major > 4 || (_crsf_version.major == 4 && _crsf_version.minor >= 6)) || _crsf_version.is_elrs)) {
        return AP_RCProtocol_CRSF::CRSF_FRAMETYPE_AP_CUSTOM_TELEM;
    }
    return AP_RCProtocol_CRSF::CRSF_FRAMETYPE_AP_CUSTOM_TELEM_LEGACY;
}

AP_RCProtocol_CRSF::RFMode AP_CRSF_Telem::get_rf_mode() const
{
    AP_RCProtocol_CRSF* crsf = AP::crsf();
    if (crsf == nullptr) {
        return AP_RCProtocol_CRSF::RFMode::RF_MODE_UNKNOWN;
    }

    if (!_crsf_version.pending && _crsf_version.use_rf_mode) {
        if (_crsf_version.is_elrs) {
            return static_cast<AP_RCProtocol_CRSF::RFMode>(uint8_t(AP_RCProtocol_CRSF::RFMode::ELRS_RF_MODE_4HZ) + crsf->get_link_status().rf_mode);
        }
        return static_cast<AP_RCProtocol_CRSF::RFMode>(crsf->get_link_status().rf_mode);
    } else if (_crsf_version.is_tracer) {
        return AP_RCProtocol_CRSF::RFMode::CRSF_RF_MODE_250HZ;
    }

    /*
     Note:
     - CRSF rf mode 2 on UARTS with DMA runs @160Hz
     - CRSF rf mode 2 on UARTS with no DMA runs @70Hz
    */
    if (get_avg_packet_rate() < 40U) {
        // no DMA CRSF rf mode 1
        return AP_RCProtocol_CRSF::RFMode::CRSF_RF_MODE_50HZ;
    }
    if (get_avg_packet_rate() > 120U) {
        // DMA CRSF rf mode 2
        return AP_RCProtocol_CRSF::RFMode::CRSF_RF_MODE_150HZ;
    }
    if (get_max_packet_rate() < 120U) {
        // no CRSF DMA rf mode 2
        return AP_RCProtocol_CRSF::RFMode::CRSF_RF_MODE_150HZ;
    }
    return AP_RCProtocol_CRSF::RFMode::CRSF_RF_MODE_50HZ;
}

bool AP_CRSF_Telem::is_high_speed_telemetry(const AP_RCProtocol_CRSF::RFMode rf_mode) const
{
    if (!_crsf_version.is_elrs) {
        return rf_mode == AP_RCProtocol_CRSF::RFMode::CRSF_RF_MODE_150HZ || rf_mode == AP_RCProtocol_CRSF::RFMode::CRSF_RF_MODE_250HZ;
    }
    return get_telemetry_rate() > 30;
}

uint16_t AP_CRSF_Telem::get_telemetry_rate() const
{
    if (!_crsf_version.is_elrs) {
        return get_avg_packet_rate();
    }
    AP_RCProtocol_CRSF* crsf = AP::crsf();
    if (crsf == nullptr) {
        return get_avg_packet_rate();
    }
    // ELRS sends 1 telemetry frame every n RC frames
    // the 1:n ratio is user selected
    // RC rate is measured by get_avg_packet_rate()
    // telemetry rate = air rate - RC rate
    return uint16_t(AP_RCProtocol_CRSF::elrs_air_rates[MIN(crsf->get_link_status().rf_mode, 7U)] - get_avg_packet_rate());
}

void AP_CRSF_Telem::queue_message(MAV_SEVERITY severity, const char *text)
{
    // no need to queue status text messages when crossfire
    // custom telemetry is not enabled
    if (!rc().crsf_custom_telemetry()) {
        return;
    }
    AP_RCTelemetry::queue_message(severity, text);
}

/*
  disable telemetry entries that require a transmitter to be present
 */
void AP_CRSF_Telem::disable_tx_entries()
{
    disable_scheduler_entry(ATTITUDE);
    disable_scheduler_entry(BATTERY);
    disable_scheduler_entry(GPS);
    disable_scheduler_entry(FLIGHT_MODE);
    disable_scheduler_entry(PASSTHROUGH);
    disable_scheduler_entry(STATUS_TEXT);
    // GENERAL_COMMAND and PARAMETERS will only be sent under very specific circumstances
}

/*
  enable telemetry entries that require a transmitter to be present
 */
void AP_CRSF_Telem::enable_tx_entries()
{
    enable_scheduler_entry(ATTITUDE);
    enable_scheduler_entry(BATTERY);
    enable_scheduler_entry(GPS);
    enable_scheduler_entry(FLIGHT_MODE);
    enable_scheduler_entry(PASSTHROUGH);
    enable_scheduler_entry(STATUS_TEXT);

    update_custom_telemetry_rates(_telem_rf_mode);
}

void AP_CRSF_Telem::disable_telemetry(uint32_t enable_at_ms)
{
    debug("disable telemetry");
    _custom_telem.disable_telemetry_timeout_ms = enable_at_ms;
    set_scheduler_entry(HEARTBEAT, 50, 200);            // heartbeat        5Hz
    disable_tx_entries();
}

void AP_CRSF_Telem::enable_telemetry()
{
    debug("enable telemetry");
    _custom_telem.disable_telemetry_timeout_ms = 0;
    // setup the crossfire scheduler for custom telemetry
    set_scheduler_entry(HEARTBEAT, 2000, 5000);     // 0.2Hz
    enable_tx_entries();
}

void AP_CRSF_Telem::adjust_packet_weight(bool queue_empty)
{
    uint32_t now_ms = AP_HAL::millis();
    setup_custom_telemetry();

    /*
     whenever we detect a pending request we configure the scheduler
     to allow faster parameters processing.
     We start a "fast parameter window" that we close after 5sec
    */
    bool expired = _custom_telem.disable_telemetry_timeout_ms > 0
        && now_ms > _custom_telem.disable_telemetry_timeout_ms;
    if (_custom_telem.disable_telemetry_timeout_ms == 0
        && _pending_request.frame_type > 0
        && _pending_request.frame_type != AP_RCProtocol_CRSF::CRSF_FRAMETYPE_PARAM_DEVICE_INFO
        && !hal.util->get_soft_armed()) {
        // fast window start
        disable_telemetry(now_ms + 5000);
    } else if (expired) {
        // fast window stop
        enable_telemetry();
    }
}

// WFQ scheduler
bool AP_CRSF_Telem::is_packet_ready(uint8_t idx, bool queue_empty)
{
    if (!process_rf_mode_changes()) {
        return false;
    }

    switch (idx) {
    case PARAMETERS:
        return _pending_request.frame_type > 0;
    case VTX_PARAMETERS:
        return AP::vtx().have_params_changed() || is_vtx_change_pending();
    case PASSTHROUGH:
        return rc().crsf_custom_telemetry();
    case STATUS_TEXT:
        return rc().crsf_custom_telemetry() && !queue_empty;
    case GENERAL_COMMAND:
        return _baud_rate_request.pending;
    case VERSION_PING:
        return _crsf_version.pending;
    case HEARTBEAT:
        return true; // always send heartbeat if enabled
    case DEVICE_PING:
        return !_crsf_version.pending; // only send pings if version has been negotiated
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
            process_pending_requests();
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
        case PASSTHROUGH:
            if (is_high_speed_telemetry(_telem_rf_mode)) {
                // on fast links we have 1:1 ratio between
                // passthrough frames and crossfire frames
                get_single_packet_passthrough_telem_data();
            } else {
                // on slower links we pack many passthrough
                // frames in a single crossfire one (up to 9)
                const uint8_t size = _crsf_version.is_elrs ? 3 : AP_CRSF_Telem::PASSTHROUGH_MULTI_PACKET_FRAME_MAX_SIZE;
                get_multi_packet_passthrough_telem_data(size);
            }
            break;
        case STATUS_TEXT:
            calc_status_text();
            break;
        case GENERAL_COMMAND:
            calc_command_response();
            break;
        case VERSION_PING:
            // to get crossfire firmware version we send an RX device ping
            if (_crsf_version.retry_count++ > CRSF_RX_DEVICE_PING_MAX_RETRY) {
                _crsf_version.pending = false;
                _crsf_version.minor = 0;
                _crsf_version.major = 0;
                disable_scheduler_entry(VERSION_PING);
                gcs().send_text(MAV_SEVERITY_DEBUG,"%s: RX device ping failed", get_protocol_string());
            } else {
                calc_device_ping(AP_RCProtocol_CRSF::CRSF_ADDRESS_CRSF_RECEIVER);
                gcs().send_text(MAV_SEVERITY_DEBUG,"%s: requesting RX device info", get_protocol_string());
            }
            break;
        case DEVICE_PING:
            calc_device_ping(AP_RCProtocol_CRSF::CRSF_ADDRESS_CRSF_RECEIVER);
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

    case AP_RCProtocol_CRSF::CRSF_FRAMETYPE_PARAM_DEVICE_INFO:
        process_device_info_frame((ParameterDeviceInfoFrame*)data);
        break;

    case AP_RCProtocol_CRSF::CRSF_FRAMETYPE_COMMAND:
        process_command_frame((CommandFrame*)data);
        break;

    default:
        break;
    }
    return true;
}

bool AP_CRSF_Telem::is_vtx_change_pending() const {
    return _vtx_power_change_pending || _vtx_freq_change_pending || _vtx_options_change_pending;
}

void AP_CRSF_Telem::process_vtx_telem_frame(VTXTelemetryFrame* vtx)
{
    vtx->frequency = be16toh(vtx->frequency);

    // start processing configuration changes
    vtx_is_alive = true;

    // the VTX rewrite sends many fake frames that can be spotted through zeros
    if (_crsf_version.major == 6 && _crsf_version.minor <= 19
        && vtx->power == 0 && vtx->pitmode == 0 && vtx->frequency == 5865) {
        return;
    }

    debug_vtx("VTXTelemetry: Freq: %d, PitMode: %d, Power: %d", vtx->frequency, vtx->pitmode, vtx->power);

    // if we can't trust the status message, simply bail
    if (ignore_vtx_status()) {
        return;
    }

    AP_VideoTX& apvtx = AP::vtx();

    if (!apvtx.get_enabled()) {
        return;
    }

    apvtx.set_frequency_mhz(vtx->frequency);

    AP_VideoTX::VideoBand band;
    uint8_t channel;
    if (AP_VideoTX::get_band_and_channel(vtx->frequency, band, channel)) {
        apvtx.set_band(uint8_t(band));
        apvtx.set_channel(channel);
    }

    apvtx.set_power_dbm(vtx->power);
    // no change pending and whatever we configured did not take
    if (_vtx_power_change_pending && apvtx.get_configured_power_dbm() != vtx->power) {
        _vtx_power_change_pending = 0;
        apvtx.update_power_dbm(apvtx.get_configured_power_dbm(), AP_VideoTX::PowerActive::Inactive);
    }
    process_vtx_frame(vtx->pitmode);
}

void AP_CRSF_Telem::process_vtx_frame(bool pitmode)
{
    AP_VideoTX& vtx = AP::vtx();

    if (pitmode) {
        vtx.set_options(vtx.get_options() | uint8_t(AP_VideoTX::VideoOptions::VTX_PITMODE));
    } else {
        vtx.set_options(vtx.get_options() & ~uint8_t(AP_VideoTX::VideoOptions::VTX_PITMODE));
    }

    const bool update_pending = is_vtx_change_pending();

    if (!vtx.update_power()) {
        _vtx_power_change_pending = 0;
    }

    if (!vtx.update_band() && !vtx.update_channel() && !vtx.update_frequency()) {
        _vtx_freq_change_pending = 0;
    }

    if (!vtx.update_options()) {
        _vtx_options_change_pending = 0;
    }
    // make sure the configured values now reflect reality
    if (!vtx.set_defaults() && update_pending && !is_vtx_change_pending()) {
        vtx.announce_vtx_settings();
    }
}

// request for device info
void AP_CRSF_Telem::process_ping_frame(ParameterPingFrame* ping)
{
    debug("process_ping_frame: %d -> %d", ping->origin, ping->destination);
    if (ping->destination != 0 && ping->destination != AP_RCProtocol_CRSF::CRSF_ADDRESS_FLIGHT_CONTROLLER) {
        return; // request was not for us
    }

    _param_request.origin = ping->origin;
    _pending_request.frame_type = AP_RCProtocol_CRSF::CRSF_FRAMETYPE_PARAM_DEVICE_INFO;
    _pending_request.destination = ping->origin;
}

// request for device info
void AP_CRSF_Telem::process_device_info_frame(ParameterDeviceInfoFrame* info)
{
    debug("process_device_info_frame: 0x%x -> 0x%x", info->origin, info->destination);
    if (info->destination != 0 && info->destination != AP_RCProtocol_CRSF::CRSF_ADDRESS_FLIGHT_CONTROLLER) {
        return; // request was not for us
    }

    // we are only interested in RC device info for firmware version detection
    if (info->origin != 0 && info->origin != AP_RCProtocol_CRSF::CRSF_ADDRESS_CRSF_RECEIVER) {
        return;
    }
    /*
        Payload size is 58:
        char[] Device name ( Null-terminated string, max len is 42 )
        uint32_t Serial number
        uint32_t Hardware ID
        uint32_t Firmware ID (0x00:0x00:0xAA:0xBB AA=major, BB=minor)
        uint8_t Parameters count
        uint8_t Parameter version number
    */
    // get the terminator of the device name string
    const uint8_t offset = strnlen((char*)info->payload,42U);
    if (strncmp((char*)info->payload, "Tracer", 6) == 0) {
        _crsf_version.is_tracer = true;
    } else if (strncmp((char*)&info->payload[offset+1], "ELRS", 4) == 0) {
        // ELRS magic number is ELRS encoded in the serial number
        // 0x45 'E' 0x4C 'L' 0x52 'R' 0x53 'S'
        _crsf_version.is_elrs = true;
    }
    /*
        fw major ver = offset + terminator (8bits) + serial (32bits) + hw id (32bits) + 3rd byte of sw id = 11bytes
        fw minor ver = offset + terminator (8bits) + serial (32bits) + hw id (32bits) + 4th byte of sw id = 12bytes
    */
    _crsf_version.major = info->payload[offset+11];
    _crsf_version.minor = info->payload[offset+12];

    // should we use rf_mode reported by link statistics?
    if (_crsf_version.is_elrs || (!_crsf_version.is_tracer && (_crsf_version.major > 3 || (_crsf_version.major == 3 && _crsf_version.minor >= 72)))) {
        _crsf_version.use_rf_mode = true;
    }

    _crsf_version.pending = false;
    disable_scheduler_entry(VERSION_PING);
}

// request for a general command
void AP_CRSF_Telem::process_command_frame(CommandFrame* command)
{
    debug("process_command_frame: 0x%x -> 0x%x: 0x%x", command->origin, command->destination, command->payload[0]);
    if (command->destination != 0 && command->destination != AP_RCProtocol_CRSF::CRSF_ADDRESS_FLIGHT_CONTROLLER) {
        return; // request was not for us
    }

    if (command->payload[0] == AP_RCProtocol_CRSF::CRSF_COMMAND_ACK) {
        debug("command ACK: %d %d %d from %d", command->payload[1], command->payload[2], command->payload[3], command->origin);
    }

    // we are only interested in commands from the RX
    if (command->origin != 0 && command->origin != AP_RCProtocol_CRSF::CRSF_ADDRESS_CRSF_RECEIVER) {
        return;
    }

    switch (command->payload[0]) {
        case AP_RCProtocol_CRSF::CRSF_COMMAND_GENERAL_CRSF_SPEED_PROPOSAL: {
            uint32_t baud_rate = command->payload[2] << 24 | command->payload[3] << 16
                | command->payload[4] << 8 | command->payload[5];
            _baud_rate_request.port_id = command->payload[1];
            _baud_rate_request.valid = AP::crsf()->change_baud_rate(baud_rate);
            _baud_rate_request.pending = true;
            debug("requested baud rate change %lu", baud_rate);
            break;
        }
        default:
            break; // do nothing
    }
}

void AP_CRSF_Telem::process_param_read_frame(ParameterSettingsReadFrame* read_frame)
{
    debug("process_param_read_frame: %d -> %d for %d[%d]", read_frame->origin, read_frame->destination,
        read_frame->param_num, read_frame->param_chunk);
    if (read_frame->destination != 0 && read_frame->destination != AP_RCProtocol_CRSF::CRSF_ADDRESS_FLIGHT_CONTROLLER) {
        return; // request was not for us
    }

    _param_request = *read_frame;
    _pending_request.frame_type = AP_RCProtocol_CRSF::CRSF_FRAMETYPE_PARAMETER_READ;
}

// process any changed settings and schedule for transmission
void AP_CRSF_Telem::update()
{
}

void AP_CRSF_Telem::process_pending_requests()
{
    // handle general parameter requests
    switch (_pending_request.frame_type) {
    // construct a response to a ping frame
    case AP_RCProtocol_CRSF::CRSF_FRAMETYPE_PARAM_DEVICE_INFO:
        _custom_telem.disable_telemetry_timeout_ms = AP_HAL::millis() + 5000;
        calc_device_info();
        break;
    // construct a ping frame originating here
    case AP_RCProtocol_CRSF::CRSF_FRAMETYPE_PARAM_DEVICE_PING:
        calc_device_ping(_pending_request.destination);
        break;
    case AP_RCProtocol_CRSF::CRSF_FRAMETYPE_PARAMETER_READ:
        // reset parameter passthrough timeout
        _custom_telem.disable_telemetry_timeout_ms = AP_HAL::millis() + 5000;
        calc_parameter();
        break;
    default:
        break;
    }

    _pending_request.frame_type = 0;
}

void AP_CRSF_Telem::update_vtx_params()
{
    AP_VideoTX& vtx = AP::vtx();

    if (!vtx.get_enabled() || !vtx_is_alive) {
        return;
    }

    // disable other telemetry to allow requests to get through
    if (_custom_telem.disable_telemetry_timeout_ms == 0) {
        disable_telemetry(AP_HAL::millis() + 1000);
    }

    if (!_vtx_freq_change_pending && (vtx.update_band() || vtx.update_channel() || vtx.update_frequency())) {
        _vtx_freq_change_pending = VTX_SETTINGS_RETRIES;
    }

    // don't update the power if we are supposed to be in pitmode as this will take us out of pitmode
    if (!_vtx_options_change_pending && vtx.update_options()) {
        if (vtx.get_configured_pitmode()) {
            _vtx_pitmode_change = PitmodeChangeState::PitmodeOn;
        } else {
            _vtx_pitmode_change = PitmodeChangeState::PitmodeOff;
        }
        _vtx_options_change_pending = VTX_SETTINGS_RETRIES;
    }

    if (!vtx.get_configured_pitmode() && vtx.update_power()
        && !_vtx_freq_change_pending && !_vtx_options_change_pending && !_vtx_power_change_pending) {
        _vtx_power_change_pending = VTX_SETTINGS_RETRIES;
    }

    if (is_vtx_change_pending()) {
        // make the desired frequency match the desired band and channel
        if (_vtx_freq_change_pending) {
            if (vtx.update_band() || vtx.update_channel()) {
                vtx.update_configured_frequency();
            } else {
                vtx.update_configured_channel_and_band();
            }
        }

        debug_vtx("update_vtx(): freq %d->%d, chan: %d->%d, band: %d->%d, pwr: %d->%d, opts: %d->%d",
            vtx.get_frequency_mhz(),  vtx.get_configured_frequency_mhz(),
            vtx.get_channel(), vtx.get_configured_channel(),
            vtx.get_band(), vtx.get_configured_band(),
            vtx.get_power_mw(), vtx.get_configured_power_mw(),
            vtx.get_options(), vtx.get_configured_options());

        _telem_type = AP_RCProtocol_CRSF::CRSF_FRAMETYPE_COMMAND;
        _telem.ext.command.destination = AP_RCProtocol_CRSF::CRSF_ADDRESS_VTX;
        _telem.ext.command.origin = AP_RCProtocol_CRSF::CRSF_ADDRESS_FLIGHT_CONTROLLER;
        _telem.ext.command.command_id = AP_RCProtocol_CRSF::CRSF_COMMAND_VTX;

        uint8_t len = 5;
        // prioritize option changes so that the pilot can get in and out of pitmode
        if (_vtx_options_change_pending) {
            if (_vtx_pitmode_change == PitmodeChangeState::PowerupFromPitmode) {
                _telem.ext.command.payload[0] = AP_RCProtocol_CRSF::CRSF_COMMAND_VTX_PITMODE_POWERUP;
                debug_vtx("pitmode power up");
                len--;
            } else {
                _telem.ext.command.payload[0] = AP_RCProtocol_CRSF::CRSF_COMMAND_VTX_PITMODE;
                if (vtx.get_configured_pitmode()) {
                    _telem.ext.command.payload[1] = 1;
                    debug_vtx("pitmode on");
                } else {
                    debug_vtx("pitmode off");
                    _telem.ext.command.payload[1] = 0;
                    //_vtx_pitmode_change = PitmodeChangeState::PowerupFromPitmode; // try power up next time
                }
            }
            if (ignore_vtx_status() && --_vtx_options_change_pending == 0) {
                vtx.set_options_are_current();
            }
        } else if (_vtx_freq_change_pending) {
            _telem.ext.command.payload[0] = AP_RCProtocol_CRSF::CRSF_COMMAND_VTX_FREQ;
            _telem.ext.command.payload[1] = (vtx.get_configured_frequency_mhz() & 0xFF00) >> 8;
            _telem.ext.command.payload[2] = (vtx.get_configured_frequency_mhz() & 0xFF);
            if (ignore_vtx_status() && --_vtx_freq_change_pending == 0) {
                vtx.set_freq_is_current();
            }
            len++;
        } else if (_vtx_power_change_pending) {
            debug_vtx("power %ddbm", vtx.get_configured_power_dbm());
            _telem.ext.command.payload[0] = AP_RCProtocol_CRSF::CRSF_COMMAND_VTX_POWER_DBM;
            _telem.ext.command.payload[1] = vtx.get_configured_power_dbm();
            if (ignore_vtx_status() && --_vtx_power_change_pending == 0) {
                vtx.set_power_is_current();
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

        // if we have sent settings enough times, declare victory
        if (ignore_vtx_status() && !is_vtx_change_pending()) {
            if (!vtx.set_defaults()) {
                vtx.announce_vtx_settings();
            }
        }
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

    uint8_t percentage = 0;
    IGNORE_RETURN(_battery.capacity_remaining_pct(percentage, 0));

    _telem.bcast.battery.remaining = percentage;

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
        // Note: snprintf() always terminates the string
        hal.util->snprintf(_telem.bcast.flightmode.flight_mode, sizeof(AP_CRSF_Telem::FlightModeFrame), "%s", notify->get_flight_mode_str());
        // Note: strlen(_telem.bcast.flightmode.flight_mode) is safe because called on a guaranteed null terminated string
        _telem_size = strlen(_telem.bcast.flightmode.flight_mode) + 1; //send the terminator as well
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
    int32_t n = strlen(fwver.fw_short_string);
    strncpy((char*)_telem.ext.info.payload, fwver.fw_short_string, 41);
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

    _pending_request.frame_type = 0;
    _telem_pending = true;
#endif
}

// send a device ping
void AP_CRSF_Telem::calc_device_ping(uint8_t destination) {
    _telem.ext.ping.destination = destination;
    _telem.ext.ping.origin = AP_RCProtocol_CRSF::CRSF_ADDRESS_FLIGHT_CONTROLLER;
    _telem_size = 2;
    _telem_type = AP_RCProtocol_CRSF::CRSF_FRAMETYPE_PARAM_DEVICE_PING;
    _pending_request.frame_type = 0;

    _telem_pending = true;
}

// send a command response
void AP_CRSF_Telem::calc_command_response() {
    _telem.ext.command.destination = AP_RCProtocol_CRSF::CRSF_ADDRESS_CRSF_RECEIVER;
    _telem.ext.command.origin = AP_RCProtocol_CRSF::CRSF_ADDRESS_FLIGHT_CONTROLLER;
    _telem.ext.command.command_id = AP_RCProtocol_CRSF::CRSF_COMMAND_GENERAL;
    _telem.ext.command.payload[0] = AP_RCProtocol_CRSF::CRSF_COMMAND_GENERAL_CRSF_SPEED_RESPONSE;
    _telem.ext.command.payload[1] = _baud_rate_request.port_id;
    _telem.ext.command.payload[2] = _baud_rate_request.valid;
    _telem_type = AP_RCProtocol_CRSF::CRSF_FRAMETYPE_COMMAND;

    // calculate command crc
    uint8_t len = 6;
    uint8_t* crcptr = &_telem.ext.command.destination;
    uint8_t crc = crc8_dvb(0, AP_RCProtocol_CRSF::CRSF_FRAMETYPE_COMMAND, 0xBA);
    for (uint8_t i = 0; i < len; i++) {
        crc = crc8_dvb(crc, crcptr[i], 0xBA);
    }
    crcptr[len] = crc;
    _telem_size = len + 1;

    _pending_request.frame_type = 0;
    _baud_rate_request.pending = false;

    debug("sent baud rate response: %u", _baud_rate_request.valid);
    _telem_pending = true;
}

// return parameter information
void AP_CRSF_Telem::calc_parameter() {
#if OSD_PARAM_ENABLED
    _telem.ext.param_entry.header.destination = _param_request.origin;
    _telem.ext.param_entry.header.origin = AP_RCProtocol_CRSF::CRSF_ADDRESS_FLIGHT_CONTROLLER;
    size_t idx = 0;

    // root folder request
    if (_param_request.param_num == 0) {
        _telem.ext.param_entry.header.param_num = 0;
        _telem.ext.param_entry.header.chunks_left = 0;
        _telem.ext.param_entry.payload[idx++] = 0; // parent folder
        _telem.ext.param_entry.payload[idx++] = ParameterType::FOLDER; // type
        _telem.ext.param_entry.payload[idx++] = 'r'; // "root" name
        _telem.ext.param_entry.payload[idx++] = 'o';
        _telem.ext.param_entry.payload[idx++] = 'o';
        _telem.ext.param_entry.payload[idx++] = 't';
        _telem.ext.param_entry.payload[idx++] = 0; // null terminator

        // write out all of the ids we are going to send
        for (uint8_t i = 0; i < AP_OSD_ParamScreen::NUM_PARAMS * AP_OSD_NUM_PARAM_SCREENS; i++) {
            _telem.ext.param_entry.payload[idx++] = i + 1;
        }
        _telem.ext.param_entry.payload[idx] = 0xFF; // terminator

        _telem_size = sizeof(AP_CRSF_Telem::ParameterSettingsEntryHeader) + 1 + idx;
        _telem_type = AP_RCProtocol_CRSF::CRSF_FRAMETYPE_PARAMETER_SETTINGS_ENTRY;
        _pending_request.frame_type = 0;
        _telem_pending = true;
        return;
    }

    AP_OSD* osd = AP::osd();

    if (osd == nullptr) {
        return;
    }

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

    _pending_request.frame_type = 0;
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

    _pending_request.frame_type = 0;
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

// get status text data
void AP_CRSF_Telem::calc_status_text()
{
    if (!_statustext.available) {
        WITH_SEMAPHORE(_statustext.sem);
        // check link speed
        if (!_crsf_version.is_elrs && !is_high_speed_telemetry(_telem_rf_mode)) {
            // keep only warning/error/critical/alert/emergency status text messages
            bool got_message = false;
            while (_statustext.queue.pop(_statustext.next)) {
                if (_statustext.next.severity <= MAV_SEVERITY_WARNING) {
                    got_message = true;
                    break;
                }
            }
            if (!got_message) {
                return;
            }
        } else if (!_statustext.queue.pop(_statustext.next)) {
            return;
        }
        _statustext.available = true;
    }

    _telem_type = get_custom_telem_frame_id();
    _telem.bcast.custom_telem.status_text.sub_type = AP_RCProtocol_CRSF::CustomTelemSubTypeID::CRSF_AP_CUSTOM_TELEM_STATUS_TEXT;
    _telem.bcast.custom_telem.status_text.severity = _statustext.next.severity;
    // Note: snprintf() always terminates the string
    hal.util->snprintf(_telem.bcast.custom_telem.status_text.text, AP_CRSF_Telem::PASSTHROUGH_STATUS_TEXT_FRAME_MAX_SIZE, "%s", _statustext.next.text);
    // frame size = sub_type(1) + severity(1) + strlen(text) + terminator
    // Note: strlen(_telem.bcast.custom_telem.status_text.text) is safe because called on a guaranteed null terminated string
    _telem_size = 2 + strlen(_telem.bcast.custom_telem.status_text.text) + 1;
    _telem_pending = true;
    _statustext.available = false;
}

/*
 Get 1 packet of passthrough telemetry data
*/
void AP_CRSF_Telem::get_single_packet_passthrough_telem_data()
{
    _telem_pending = false;
    uint8_t packet_count;
    AP_Frsky_SPort::sport_packet_t packet;
    if (!AP_Frsky_Telem::get_telem_data(&packet, packet_count, 1)) {
        return;
    }
    _telem.bcast.custom_telem.single_packet_passthrough.sub_type = AP_RCProtocol_CRSF::CustomTelemSubTypeID::CRSF_AP_CUSTOM_TELEM_SINGLE_PACKET_PASSTHROUGH;
    _telem.bcast.custom_telem.single_packet_passthrough.appid = packet.appid;
    _telem.bcast.custom_telem.single_packet_passthrough.data = packet.data;
    _telem_size = sizeof(AP_CRSF_Telem::PassthroughSinglePacketFrame);
    _telem_type = get_custom_telem_frame_id();
    _telem_pending = true;
}

/*
 Get up to PASSTHROUGH_MULTI_PACKET_FRAME_MAX_SIZE packets of passthrough telemetry data (for slow links)
 Note: we have 2 distinct frame types (single packet vs multi packet) because
 whenever possible we use smaller frames for they have a higher "chance"
 of being transmitted by the crossfire RX scheduler.
*/
void AP_CRSF_Telem::get_multi_packet_passthrough_telem_data(uint8_t size)
{
    size = MIN(size, AP_CRSF_Telem::PASSTHROUGH_MULTI_PACKET_FRAME_MAX_SIZE);
    _telem_pending = false;
    uint8_t count = 0;
    AP_Frsky_SPort::sport_packet_t buffer[AP_CRSF_Telem::PASSTHROUGH_MULTI_PACKET_FRAME_MAX_SIZE] {};
    // we request a PASSTHROUGH_MULTI_PACKET_FRAME_MAX_SIZE packet array, i.e. 9 packets
    if (!AP_Frsky_Telem::get_telem_data(buffer, count, size)) {
        return;
    }
    _telem.bcast.custom_telem.multi_packet_passthrough.sub_type = AP_RCProtocol_CRSF::CustomTelemSubTypeID::CRSF_AP_CUSTOM_TELEM_MULTI_PACKET_PASSTHROUGH;
    for (uint8_t idx=0; idx<count; idx++) {
        _telem.bcast.custom_telem.multi_packet_passthrough.packets[idx].appid = buffer[idx].appid;
        _telem.bcast.custom_telem.multi_packet_passthrough.packets[idx].data = buffer[idx].data;
    }
    _telem.bcast.custom_telem.multi_packet_passthrough.size = count;
    _telem_size = 2 + sizeof(AP_CRSF_Telem::PassthroughMultiPacketFrame::PassthroughTelemetryPacket)*count; //subtype + size + 6*count
    _telem_type = get_custom_telem_frame_id();
    _telem_pending = true;
}

/*
  fetch CRSF frame data
  if is_tx_active is true then this will be a request for telemetry after receiving an RC frame
 */
bool AP_CRSF_Telem::_get_telem_data(AP_RCProtocol_CRSF::Frame* data, bool is_tx_active)
{
    memset(&_telem, 0, sizeof(TelemetryPayload));
    // update telemetry tasks if we either lost or regained the transmitter
    if (_is_tx_active != is_tx_active) {
        if (is_tx_active) {
            disable_scheduler_entry(DEVICE_PING);
            enable_tx_entries();
        } else {
            disable_tx_entries();
            enable_scheduler_entry(DEVICE_PING);
        }
        _is_tx_active = is_tx_active;
    }

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
bool AP_CRSF_Telem::get_telem_data(AP_RCProtocol_CRSF::Frame* data, bool is_tx_active)
{
    if (!get_singleton()) {
        return false;
    }
    return singleton->_get_telem_data(data, is_tx_active);
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
