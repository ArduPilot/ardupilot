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

#include <AP_Common/AP_FWVersion.h>
#include <AP_Vehicle/AP_Vehicle_Type.h>
#include <AP_Stats/AP_Stats.h>
#include <AP_RSSI/AP_RSSI.h>
#include <AP_Notify/AP_Notify.h>

#include "AP_MSP.h"
#include "AP_MSP_Telem_DJI.h"

#include <stdio.h>

#if HAL_MSP_ENABLED
extern const AP_HAL::HAL& hal;

using namespace MSP;

bool AP_MSP_Telem_DJI::init_uart()
{
    if (_msp_port.uart != nullptr)  {
        _msp_port.uart->set_flow_control(AP_HAL::UARTDriver::FLOW_CONTROL_DISABLE);
        _msp_port.uart->begin(AP_SERIALMANAGER_MSP_BAUD, AP_SERIALMANAGER_MSP_BUFSIZE_RX, AP_SERIALMANAGER_MSP_BUFSIZE_TX);
        return true;
    }
    return false;
}

bool AP_MSP_Telem_DJI::is_scheduler_enabled() const
{
    const AP_MSP *msp = AP::msp();
    return msp && msp->is_option_enabled(AP_MSP::Option::TELEMETRY_MODE);
}

void AP_MSP_Telem_DJI::hide_osd_items(void)
{
    const AP_MSP *msp = AP::msp();
    if (msp == nullptr) {
        return;
    }
    // apply base class defaults
    AP_MSP_Telem_Backend::hide_osd_items();

    // apply DJI OSD specific rules
    const AP_Notify& notify = AP::notify();
    // default is hide the DJI flightmode widget
    BIT_SET(osd_hidden_items_bitmask, OSD_FLYMODE);

    if (msp->_msp_status.flashing_on) {
        // flash flightmode on failsafe
        if (notify.flags.failsafe_battery || notify.flags.failsafe_gcs || notify.flags.failsafe_radio || notify.flags.ekf_bad) {
            BIT_CLEAR(osd_hidden_items_bitmask, OSD_FLYMODE);
        }
    }
}

uint32_t AP_MSP_Telem_DJI::get_osd_flight_mode_bitmask(void)
{
    uint32_t mode_mask = AP_MSP_Telem_Backend::get_osd_flight_mode_bitmask();
    const AP_Notify& notify = AP::notify();

    // check failsafe
    if (notify.flags.failsafe_battery || notify.flags.failsafe_gcs || notify.flags.failsafe_radio || notify.flags.ekf_bad ) {
        BIT_SET(mode_mask, DJI_FLAG_FS);
    }
    return mode_mask;
}

MSPCommandResult AP_MSP_Telem_DJI::msp_process_out_fc_variant(sbuf_t *dst)
{
    sbuf_write_data(dst, "BTFL", FLIGHT_CONTROLLER_IDENTIFIER_LENGTH);
    return MSP_RESULT_ACK;
}

MSPCommandResult AP_MSP_Telem_DJI::msp_process_out_esc_sensor_data(sbuf_t *dst)
{
#if HAL_WITH_ESC_TELEM
    int16_t highest_temperature = 0;
    AP_ESC_Telem& telem = AP::esc_telem();
    if (!displaying_stats_screen()) {
        telem.get_highest_motor_temperature(highest_temperature);
    } else {
#if OSD_ENABLED
        AP_OSD *osd = AP::osd();
        if (osd == nullptr) {
            return MSP_RESULT_ERROR;
        }
        WITH_SEMAPHORE(osd->get_semaphore());
        highest_temperature = osd->get_stats_info().max_esc_temp;
#endif
    }

    const struct PACKED {
        uint8_t temp;
        uint16_t rpm;
    } esc_sensor_data {
        temp : uint8_t(highest_temperature * 0.01f),            // deg, report max temperature
        rpm : uint16_t(telem.get_average_motor_rpm() * 0.1f)    // rpm, report average RPM across all motors
    };

    sbuf_write_data(dst, &esc_sensor_data, sizeof(esc_sensor_data));
#endif
    return MSP_RESULT_ACK;
}

void AP_MSP_Telem_DJI::update_home_pos(home_state_t &home_state)
{
    AP_MSP_Telem_Backend::update_home_pos(home_state);
#if OSD_ENABLED
    if (!displaying_stats_screen()) {
        return;
    }
    AP_MSP *msp = AP::msp();
    if (msp == nullptr) {
        return;
    }
    AP_OSD *osd = AP::osd();
    if (osd == nullptr) {
        return;
    }
    WITH_SEMAPHORE(osd->get_semaphore());
    // override telemetry with max distance and altitude info
    // alternate max distance with traveled distance every 2 seconds
    if (msp->_msp_status.slow_flashing_on) {
        home_state.home_distance_m = osd->get_stats_info().max_dist_m;
    } else {
        home_state.home_distance_m = osd->get_stats_info().last_distance_m;
    }
    home_state.rel_altitude_cm = osd->get_stats_info().max_alt_m * 100;
#endif
}

void AP_MSP_Telem_DJI::update_battery_state(battery_state_t &_battery_state)
{
    AP_MSP_Telem_Backend::update_battery_state(_battery_state);
#if OSD_ENABLED
    if (!displaying_stats_screen()) {
        return;
    }
    AP_OSD *osd = AP::osd();
    if (osd == nullptr) {
        return;
    }
    WITH_SEMAPHORE(osd->get_semaphore());
    // override telemetry with max current and voltage info
    _battery_state.batt_current_a = osd->get_stats_info().max_current_a;
    _battery_state.batt_voltage_v = osd->get_stats_info().min_voltage_v;
#endif
}

void AP_MSP_Telem_DJI::update_gps_state(gps_state_t &gps_state)
{
    AP_MSP_Telem_Backend::update_gps_state(gps_state);
#if OSD_ENABLED
    if (!displaying_stats_screen()) {
        return;
    }
    AP_OSD *osd = AP::osd();
    if (osd == nullptr) {
        return;
    }
    WITH_SEMAPHORE(osd->get_semaphore());
    // override telemetry with max speed info
    gps_state.speed_cms = osd->get_stats_info().max_speed_mps * 100;
#endif
}

void AP_MSP_Telem_DJI::update_airspeed(airspeed_state_t &airspeed_state)
{
    AP_MSP_Telem_Backend::update_airspeed(airspeed_state);
#if OSD_ENABLED
    if (!displaying_stats_screen()) {
        return;
    }
    AP_OSD *osd = AP::osd();
    if (osd == nullptr) {
        return;
    }
    WITH_SEMAPHORE(osd->get_semaphore());
    // override telemetry with max speed info
    airspeed_state.airspeed_estimate_ms = osd->get_stats_info().max_airspeed_mps;
#endif
}

void AP_MSP_Telem_DJI::update_flight_mode_str(char *flight_mode_str, uint8_t size, bool wind_enabled)
{
    AP_MSP_Telem_Backend::update_flight_mode_str(flight_mode_str, size, wind_enabled);
#if OSD_ENABLED
    if (!displaying_stats_screen()) {
        return;
    }
    AP_Stats *stats = AP::stats();
    if (stats != nullptr) {
        uint32_t t = stats->get_flight_time_s();
        // need to check snprintf return value to prevent format-truncation warning in GCC because of -Werror=format-truncation
        if (snprintf(flight_mode_str, size, "%s %3u:%02u", "STATS", unsigned(t/60), unsigned(t%60)) < 0) {
            snprintf(flight_mode_str, size, "%s", "STATS --:--");
        }
    } else {
        snprintf(flight_mode_str, size, "%s", "STATS");
    }
#endif
}

bool AP_MSP_Telem_DJI::get_rssi(float &rssi) const
{
    if (!AP_MSP_Telem_Backend::get_rssi(rssi)) {
        return false;
    }
#if OSD_ENABLED
    if (!displaying_stats_screen()) {
        return true;
    }
#if AP_RSSI_ENABLED
    AP_RSSI* ap_rssi = AP::rssi();
    if (ap_rssi == nullptr) {
        return false;
    }
    if (!ap_rssi->enabled()) {
        return false;
    }
#else
    return false;
#endif
    AP_OSD *osd = AP::osd();
    if (osd == nullptr) {
        return false;
    }
    WITH_SEMAPHORE(osd->get_semaphore());
    // override telemetry with min rssi info
    // Note: return false when min_rssi has not been updated yet by AP_OSD::update_stats()
    if (is_equal(osd->get_stats_info().min_rssi, FLT_MAX)) {
        return false;
    }
    rssi = osd->get_stats_info().min_rssi; // range is [0-1]
#endif
    return true;
}
#endif //HAL_MSP_ENABLED
