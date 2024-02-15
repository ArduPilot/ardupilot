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

#include "AP_RCTelemetry_config.h"

#if AP_GHST_TELEM_ENABLED

#include "AP_GHST_Telem.h"
#include <AP_VideoTX/AP_VideoTX.h>
#include <AP_HAL/utility/sparse-endian.h>
#include <AP_BattMonitor/AP_BattMonitor.h>
#include <AP_GPS/AP_GPS.h>
#include <GCS_MAVLink/GCS.h>
#include <AP_RCProtocol/AP_RCProtocol_GHST.h>
#include <AP_SerialManager/AP_SerialManager.h>
#include <AP_Compass/AP_Compass.h>
#include <AP_Baro/AP_Baro.h>
#include <AP_AHRS/AP_AHRS.h>
#include <AP_Notify/AP_Notify.h>
#include <math.h>
#include <stdio.h>
#include <AP_HAL/AP_HAL.h>
#include <AP_Vehicle/AP_Vehicle_Type.h>

//#define GHST_DEBUG
#ifdef GHST_DEBUG
# define debug(fmt, args...)	hal.console->printf("GHST: " fmt "\n", ##args)
#else
# define debug(fmt, args...)	do {} while(0)
#endif

extern const AP_HAL::HAL& hal;

AP_GHST_Telem *AP_GHST_Telem::singleton;

AP_GHST_Telem::AP_GHST_Telem() : AP_RCTelemetry(0)
{
    singleton = this;
}

AP_GHST_Telem::~AP_GHST_Telem(void)
{
    singleton = nullptr;
}

bool AP_GHST_Telem::init(void)
{
    // sanity check that we are using a UART for RC input
    if (!AP::serialmanager().have_serial(AP_SerialManager::SerialProtocol_RCIN, 0)) {
        return false;
    }

    return AP_RCTelemetry::init();
}

/*
  setup ready for passthrough telem
 */
void AP_GHST_Telem::setup_wfq_scheduler(void)
{
    // initialize packet weights for the WFQ scheduler
    // priority[i] = 1/_scheduler.packet_weight[i]
    // rate[i] = LinkRate * ( priority[i] / (sum(priority[1-n])) )

    // CSRF telemetry rate is 150Hz (4ms) max, so these rates must fit
    add_scheduler_entry(50, 120);   // Attitude and compass 8Hz
    add_scheduler_entry(200, 1000); // VTX parameters    1Hz
    add_scheduler_entry(1300, 500); // battery           2Hz
    add_scheduler_entry(550, 280);  // GPS               3Hz
    add_scheduler_entry(550, 280);  // GPS2              3Hz
}

void AP_GHST_Telem::update_custom_telemetry_rates(AP_RCProtocol_GHST::RFMode rf_mode)
{
    if (is_high_speed_telemetry(rf_mode)) {
        // standard telemetry for high data rates
        set_scheduler_entry(BATTERY, 1000, 1000);       // 1Hz
        set_scheduler_entry(ATTITUDE, 1000, 1000);      // 1Hz
        // custom telemetry for high data rates
        set_scheduler_entry(GPS, 550, 500);            // 2.0Hz
        set_scheduler_entry(GPS2, 550, 500);            // 2.0Hz
    } else {
        // standard telemetry for low data rates
        set_scheduler_entry(BATTERY, 1000, 2000);       // 0.5Hz
        set_scheduler_entry(ATTITUDE, 1000, 3000);      // 0.33Hz
        // GHST custom telemetry for low data rates
        set_scheduler_entry(GPS, 550, 1000);              // 1.0Hz
        set_scheduler_entry(GPS2, 550, 1000);              // 1.0Hz
    }
}

bool AP_GHST_Telem::process_rf_mode_changes()
{
    const AP_RCProtocol_GHST::RFMode current_rf_mode = get_rf_mode();
    uint32_t now = AP_HAL::millis();

    AP_RCProtocol_GHST* ghost = AP::ghost();
    AP_HAL::UARTDriver* uart = nullptr;
    if (ghost != nullptr) {
        uart = ghost->get_UART();
    }

    if (uart == nullptr) {
        return true;
    }

    if (!ghost->is_detected()) {
        return false;
    }
    // not ready yet
    if (!uart->is_initialized()) {
        return false;
    }
#if !defined (STM32H7)
    // warn the user if their setup is sub-optimal, H7 does not need DMA on serial port
    if (_telem_bootstrap_msg_pending && !uart->is_dma_enabled()) {
        GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "%s: running on non-DMA serial port", get_protocol_string());
    }
#endif
    // note if option was set to show LQ in place of RSSI
    bool current_lq_as_rssi_active = rc().option_is_enabled(RC_Channels::Option::USE_CRSF_LQ_AS_RSSI);
    if(_telem_bootstrap_msg_pending || _noted_lq_as_rssi_active != current_lq_as_rssi_active){
        _noted_lq_as_rssi_active = current_lq_as_rssi_active;
        GCS_SEND_TEXT(MAV_SEVERITY_INFO, "%s: RSSI now displays %s", get_protocol_string(), current_lq_as_rssi_active ? " as LQ" : "normally");
    }
    _telem_bootstrap_msg_pending = false;

    const bool is_high_speed = is_high_speed_telemetry(current_rf_mode);
    if ((now - _telem_last_report_ms > 5000)) {
        // report an RF mode change or a change in telemetry rate if we haven't done so in the last 5s
        if (!rc().option_is_enabled(RC_Channels::Option::SUPPRESS_CRSF_MESSAGE) && (_rf_mode != current_rf_mode)) {
            GCS_SEND_TEXT(MAV_SEVERITY_INFO, "%s: Link rate %dHz, Telemetry %s",
                get_protocol_string(), ghost->get_link_rate(), _enable_telemetry ? "ON" : "OFF");
        }
        // tune the scheduler based on telemetry speed high/low transitions
        if (_telem_is_high_speed != is_high_speed) {
            update_custom_telemetry_rates(current_rf_mode);
        }
        _telem_is_high_speed = is_high_speed;
        _rf_mode = current_rf_mode;
        _telem_last_avg_rate = _scheduler.avg_packet_rate;
        if (_telem_last_report_ms == 0) {   // only want to show bootstrap messages once
            _telem_bootstrap_msg_pending = true;
        }
        _telem_last_report_ms = now;
    }
    return true;
}

AP_RCProtocol_GHST::RFMode AP_GHST_Telem::get_rf_mode() const
{
    AP_RCProtocol_GHST* ghost = AP::ghost();
    if (ghost == nullptr) {
        return AP_RCProtocol_GHST::RFMode::RF_MODE_UNKNOWN;
    }

    return static_cast<AP_RCProtocol_GHST::RFMode>(ghost->get_link_status().rf_mode);
}

bool AP_GHST_Telem::is_high_speed_telemetry(const AP_RCProtocol_GHST::RFMode rf_mode) const
{
    return rf_mode == AP_RCProtocol_GHST::RFMode::GHST_RF_MODE_RACE || rf_mode == AP_RCProtocol_GHST::RFMode::GHST_RF_MODE_RACE250;
}

uint16_t AP_GHST_Telem::get_telemetry_rate() const
{
    return get_avg_packet_rate();
}

// WFQ scheduler
bool AP_GHST_Telem::is_packet_ready(uint8_t idx, bool queue_empty)
{
    return _enable_telemetry;
}

// WFQ scheduler
void AP_GHST_Telem::process_packet(uint8_t idx)
{
    // send packet
    switch (idx) {
        case ATTITUDE:
            calc_attitude();
            break;
        case BATTERY: // BATTERY
            calc_battery();
            break;
        case GPS: // GPS
            calc_gps();
            break;
        case GPS2: // GPS secondary info
            calc_gps2();
            break;
        default:
            break;
    }
}

// Process a frame from the GHST protocol decoder
bool AP_GHST_Telem::_process_frame(AP_RCProtocol_GHST::FrameType frame_type, void* data) {
    switch (frame_type) {
    // this means we are connected to an RC receiver and can send telemetry
    case AP_RCProtocol_GHST::GHST_UL_RC_CHANS_RSSI: {
        process_rf_mode_changes();
        _enable_telemetry = AP::ghost()->is_telemetry_supported();
        break;
    }
    default:
        break;
    }
    return true;
}

// process any changed settings and schedule for transmission
void AP_GHST_Telem::update()
{
}

void AP_GHST_Telem::process_pending_requests()
{
    _pending_request.frame_type = 0;
}

// prepare battery data
void AP_GHST_Telem::calc_battery()
{
    debug("BATTERY");
    const AP_BattMonitor &_battery = AP::battery();

    _telem.battery.voltage = htole16(uint16_t(roundf(_battery.voltage(0) * 100.0f)));

    float current;
    if (!_battery.current_amps(current, 0)) {
        current = 0;
    }
    _telem.battery.current = htole16(uint16_t(roundf(current * 100.0f)));

    float used_mah;
    if (!_battery.consumed_mah(used_mah, 0)) {
        used_mah = 0;
    }

    _telem.battery.consumed = htole16(uint16_t(roundf(used_mah * 100.0f)));;
    _telem.battery.rx_voltage = htole16(uint16_t(roundf(hal.analogin->board_voltage() * 10)));

    _telem_size = sizeof(BatteryFrame);
    _telem_type = AP_RCProtocol_GHST::GHST_DL_PACK_STAT;

    _telem_pending = true;
}

// prepare gps data
void AP_GHST_Telem::calc_gps()
{
    debug("GPS");
    const Location &loc = AP::gps().location(0); // use the first gps instance (same as in send_mavlink_gps_raw)

    _telem.gps.latitude = htole32(loc.lat);
    _telem.gps.longitude = htole32(loc.lng);
    _telem.gps.altitude = htole16(constrain_int16(loc.alt / 100, 0, 5000) + 1000);

    _telem_size = sizeof(AP_GHST_Telem::GPSFrame);
    _telem_type = AP_RCProtocol_GHST::GHST_DL_GPS_PRIMARY;

    _telem_pending = true;
}

void AP_GHST_Telem::calc_gps2()
{
    debug("GPS2");
    _telem.gps2.groundspeed = htole16(roundf(AP::gps().ground_speed() * 100000 / 3600));
    _telem.gps2.gps_heading = htole16(roundf(AP::gps().ground_course() * 100.0f));
    _telem.gps2.satellites = AP::gps().num_sats();

    AP_AHRS &_ahrs = AP::ahrs();
    WITH_SEMAPHORE(_ahrs.get_semaphore());
    Location loc;

    if (_ahrs.get_location(loc) && _ahrs.home_is_set()) {
        const Location &home_loc = _ahrs.get_home();
        _telem.gps2.home_dist = home_loc.get_distance(loc) / 10; // 10m
        _telem.gps2.home_heading = loc.get_bearing_to(home_loc) / 10; // deci-degrees
    } else {
        _telem.gps2.home_dist = 0;
        _telem.gps2.home_heading = 0;
    }

    AP_GPS::GPS_Status status = AP::gps().status();
    _telem.gps2.flags = status >= AP_GPS::GPS_OK_FIX_2D ? 0x1 : 0;

    _telem_size = sizeof(AP_GHST_Telem::GPSSecondaryFrame);
    _telem_type = AP_RCProtocol_GHST::GHST_DL_GPS_SECONDARY;

    _telem_pending = true;
}

// prepare attitude data
void AP_GHST_Telem::calc_attitude()
{
    debug("MAGBARO");
    AP_AHRS &_ahrs = AP::ahrs();
    WITH_SEMAPHORE(_ahrs.get_semaphore());

    float heading = AP::compass().calculate_heading(_ahrs.get_rotation_body_to_ned());
    _telem.sensor.compass_heading = htole16(degrees(wrap_PI(heading)));

    float alt = AP::baro().get_altitude();
    _telem.sensor.baro_alt = htole16(roundf(alt));
    _telem.sensor.vario = 0;
    _telem.sensor.flags = 3;
    _telem_size = sizeof(AP_GHST_Telem::SensorFrame);
    _telem_type = AP_RCProtocol_GHST::GHST_DL_MAGBARO;

    _telem_pending = true;
}

/*
  fetch GHST frame data
  if is_tx_active is true then this will be a request for telemetry after receiving an RC frame
 */
bool AP_GHST_Telem::_get_telem_data(AP_RCProtocol_GHST::Frame* data, bool is_tx_active)
{
    memset(&_telem, 0, sizeof(TelemetryPayload));
    _is_tx_active = is_tx_active;

    run_wfq_scheduler();
    if (!_telem_pending) {
        return false;
    }
    memcpy(data->payload, &_telem, _telem_size);
    data->device_address = AP_RCProtocol_GHST::GHST_ADDRESS_GHST_RECEIVER;
    data->length = _telem_size + 2;
    data->type = _telem_type;

    _telem_pending = false;
    return true;
}

/*
  fetch data for an external transport, such as GHST
 */
bool AP_GHST_Telem::process_frame(AP_RCProtocol_GHST::FrameType frame_type, void* data)
{
    if (!get_singleton()) {
        return false;
    }
    return singleton->_process_frame(frame_type, data);
}

/*
  fetch data for an external transport, such as GHST
 */
bool AP_GHST_Telem::get_telem_data(AP_RCProtocol_GHST::Frame* data, bool is_tx_active)
{
    if (!get_singleton()) {
        return false;
    }
    return singleton->_get_telem_data(data, is_tx_active);
}

AP_GHST_Telem *AP_GHST_Telem::get_singleton(void) {
    if (!singleton && !hal.util->get_soft_armed()) {
        // if telem data is requested when we are disarmed and don't
        // yet have a AP_GHST_Telem object then try to allocate one
        new AP_GHST_Telem();
        // initialize the passthrough scheduler
        if (singleton) {
            singleton->init();
        }
    }
    return singleton;
}

namespace AP {
    AP_GHST_Telem *ghost_telem() {
        return AP_GHST_Telem::get_singleton();
    }
};

#endif // AP_GHST_TELEM_ENABLED
