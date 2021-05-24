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

#include <AP_AHRS/AP_AHRS.h>
#include <AP_Baro/AP_Baro.h>
#include <AP_Airspeed/AP_Airspeed.h>
#include <AP_BattMonitor/AP_BattMonitor.h>
#include <AP_ESC_Telem/AP_ESC_Telem.h>
#include <RC_Channel/RC_Channel.h>
#include <AP_Common/AP_FWVersion.h>
#include <AP_GPS/AP_GPS.h>
#include <AP_OpticalFlow/AP_OpticalFlow.h>
#include <AP_RangeFinder/AP_RangeFinder.h>
#include <AP_RCMapper/AP_RCMapper.h>
#include <AP_RSSI/AP_RSSI.h>
#include <AP_RTC/AP_RTC.h>
#include <GCS_MAVLink/GCS.h>

#include "AP_MSP.h"
#include "AP_MSP_Telem_Backend.h"

#include <ctype.h>
#include <stdio.h>

#if HAL_MSP_ENABLED

extern const AP_HAL::HAL& hal;
constexpr uint8_t AP_MSP_Telem_Backend::arrows[8];

using namespace MSP;

AP_MSP_Telem_Backend::AP_MSP_Telem_Backend(AP_HAL::UARTDriver *uart) : AP_RCTelemetry(MSP_TIME_SLOT_MAX)
{
    _msp_port.uart = uart;
}

/*
  Scheduler helper
 */
void AP_MSP_Telem_Backend::setup_wfq_scheduler(void)
{
    // initialize packet weights for the WFQ scheduler
    // priority[i] = 1/_scheduler.packet_weight[i]
    // rate[i] = LinkRate * ( priority[i] / (sum(priority[1-n])) )

    set_scheduler_entry(EMPTY_SLOT, 50, 50);          // nothing to send
    set_scheduler_entry(NAME, 200, 200);              // 5Hz  12 chars string used for general purpose text messages
    set_scheduler_entry(STATUS, 500, 500);            // 2Hz  flightmode
    set_scheduler_entry(CONFIG, 200, 200);            // 5Hz  OSD item positions
    set_scheduler_entry(RAW_GPS, 250, 250);           // 4Hz  GPS lat/lon
    set_scheduler_entry(COMP_GPS, 250, 250);          // 4Hz  home direction and distance
    set_scheduler_entry(ATTITUDE, 200, 200);          // 5Hz  attitude
    set_scheduler_entry(ALTITUDE, 250, 250);          // 4Hz  altitude(cm) and velocity(cm/s)
    set_scheduler_entry(ANALOG, 250, 250);            // 4Hz  rssi + batt
    set_scheduler_entry(BATTERY_STATE, 500, 500);     // 2Hz  battery
#if HAL_WITH_ESC_TELEM
    set_scheduler_entry(ESC_SENSOR_DATA, 500, 500);   // 2Hz  ESC telemetry
#endif
    set_scheduler_entry(RTC_DATETIME, 1000, 1000);    // 1Hz  RTC
}

/*
 * init - perform required initialisation
 */
bool AP_MSP_Telem_Backend::init()
{
    enable_warnings();
    return AP_RCTelemetry::init();
}

bool AP_MSP_Telem_Backend::init_uart()
{
    if (_msp_port.uart != nullptr)  {
        // re-init port here for use in this thread
        _msp_port.uart->begin(0);
        return true;
    }
    return false;
}

void AP_MSP_Telem_Backend::process_outgoing_data()
{
    if (is_scheduler_enabled()) {
        AP_RCTelemetry::run_wfq_scheduler();
    }
}

/*
  Scheduler helper
 */
bool AP_MSP_Telem_Backend::is_packet_ready(uint8_t idx, bool queue_empty)
{
    switch (idx) {
    case EMPTY_SLOT:        // empty slot
    case NAME:              // used for status_text messages
    case STATUS:            // flightmode
    case CONFIG:            // OSD config
    case RAW_GPS:           // lat,lon, speed
    case COMP_GPS:          // home dir,dist
    case ATTITUDE:          // Attitude
    case ALTITUDE:          // Altitude and Vario
    case ANALOG:            // Rssi, Battery, mAh, Current
    case BATTERY_STATE:     // voltage, capacity, current, mAh
#if HAL_WITH_ESC_TELEM
    case ESC_SENSOR_DATA:   // esc temp + rpm
#endif
    case RTC_DATETIME:      // RTC
        return true;
    default:
        return false;
    }
}

/*
  Invoked at each scheduler step
 */
void AP_MSP_Telem_Backend::process_packet(uint8_t idx)
{
    if (idx == EMPTY_SLOT) {
        return;
    }

    uint8_t out_buf[MSP_PORT_OUTBUF_SIZE] {};

    msp_packet_t reply = {
        .buf = { .ptr = out_buf, .end = MSP_ARRAYEND(out_buf), },
        .cmd = (int16_t)msp_packet_type_map[idx],
        .flags = 0,
        .result = 0,
    };
    uint8_t *out_buf_head = reply.buf.ptr;

    msp_process_out_command(msp_packet_type_map[idx], &reply.buf);
    uint32_t len = reply.buf.ptr - &out_buf[0];
    sbuf_switch_to_reader(&reply.buf, out_buf_head); // change streambuf direction
    if (len > 0) {
        // don't send zero length packets
        msp_serial_encode(&_msp_port, &reply, _msp_port.msp_version);
    }

    _msp_port.c_state = MSP_IDLE;
}

uint8_t AP_MSP_Telem_Backend::calc_cell_count(const float battery_voltage)
{
    return floorf((battery_voltage / CELLFULL) + 1);
}

float AP_MSP_Telem_Backend::get_vspeed_ms(void)
{
    {
        // release semaphore as soon as possible
        AP_AHRS &_ahrs = AP::ahrs();
        Vector3f v {};
        WITH_SEMAPHORE(_ahrs.get_semaphore());
        if (_ahrs.get_velocity_NED(v)) {
            return -v.z;
        }
    }
    AP_Baro &_baro = AP::baro();
    WITH_SEMAPHORE(_baro.get_semaphore());
    return _baro.get_climb_rate();
}

void AP_MSP_Telem_Backend::update_home_pos(home_state_t &home_state)
{
    AP_AHRS &_ahrs = AP::ahrs();
    WITH_SEMAPHORE(_ahrs.get_semaphore());
    Location loc;
    float alt;
    if (_ahrs.get_position(loc) && _ahrs.home_is_set()) {
        const Location &home_loc = _ahrs.get_home();
        home_state.home_distance_m = home_loc.get_distance(loc);
        home_state.home_bearing_cd = loc.get_bearing_to(home_loc);
    } else {
        home_state.home_distance_m = 0;
        home_state.home_bearing_cd = 0;
    }
    _ahrs.get_relative_position_D_home(alt);
    home_state.rel_altitude_cm = -alt * 100;
    home_state.home_is_set = _ahrs.home_is_set();
}

void AP_MSP_Telem_Backend::update_gps_state(gps_state_t &gps_state)
{
    AP_GPS& gps = AP::gps();

    memset(&gps_state, 0, sizeof(gps_state));

    WITH_SEMAPHORE(gps.get_semaphore());
    gps_state.fix_type = gps.status() >= AP_GPS::GPS_Status::GPS_OK_FIX_3D? 2:0;
    gps_state.num_sats = gps.num_sats();

    if (gps_state.fix_type > 0) {
        const Location &loc = AP::gps().location(); //get gps instance 0
        gps_state.lat = loc.lat;
        gps_state.lon = loc.lng;
        gps_state.alt_m = loc.alt/100; // 1m resolution
        gps_state.speed_cms = gps.ground_speed() * 100;
        gps_state.ground_course_cd = gps.ground_course_cd();
    }
}

void AP_MSP_Telem_Backend::update_battery_state(battery_state_t &battery_state)
{
    memset(&battery_state, 0, sizeof(battery_state));

    const AP_BattMonitor &_battery = AP::battery();
    if (!_battery.current_amps(battery_state.batt_current_a)) {
        battery_state.batt_current_a = 0;
    }
    if (!_battery.consumed_mah(battery_state.batt_consumed_mah)) {
        battery_state.batt_consumed_mah = 0;
    }
    battery_state.batt_voltage_v =_battery.voltage();
    battery_state.batt_capacity_mah = _battery.pack_capacity_mah();

    const AP_Notify& notify = AP::notify();
    if (notify.flags.failsafe_battery) {
        battery_state.batt_state = MSP_BATTERY_CRITICAL;
    } else {
        battery_state.batt_state = MSP_BATTERY_OK;
    }
    // detect cellcount and update only if we get a higher values, we do not want to update it while discharging
    uint8_t cc = calc_cell_count(battery_state.batt_voltage_v);
    if (cc > battery_state.batt_cellcount) {
        battery_state.batt_cellcount = cc;
    }
}

void AP_MSP_Telem_Backend::update_airspeed(airspeed_state_t &airspeed_state)
{
    AP_AHRS &ahrs = AP::ahrs();
    WITH_SEMAPHORE(ahrs.get_semaphore());
    airspeed_state.airspeed_have_estimate = ahrs.airspeed_estimate(airspeed_state.airspeed_estimate_ms);
    if (!airspeed_state.airspeed_have_estimate) {
        airspeed_state.airspeed_estimate_ms = 0.0;
    }
}

/*
    MSP OSDs can display up to MSP_TXT_VISIBLE_CHARS chars (UTF8 characters are supported)
    We display the flight mode string either with or without wind state
*/
void AP_MSP_Telem_Backend::update_flight_mode_str(char *flight_mode_str, bool wind_enabled)
{
#if OSD_ENABLED
    AP_OSD *osd = AP::osd();
    if (osd == nullptr) {
        return;
    }
#endif
    AP_Notify *notify = AP_Notify::get_singleton();
    if (notify == nullptr) {
        return;
    }
    // clear
    memset(flight_mode_str, 0, MSP_TXT_BUFFER_SIZE);

    if (wind_enabled) {
        /*
          Wind is rendered next to the current flight mode, for the direction we use an UTF8 arrow (bytes 0xE286[nn])
          example: MANU 4m/s ↗
        */
        AP_AHRS &ahrs = AP::ahrs();
        Vector3f v;
        {
            WITH_SEMAPHORE(ahrs.get_semaphore());
            v = ahrs.wind_estimate();
        }
        bool invert_wind = false;
#if OSD_ENABLED
        invert_wind = osd->screen[0].check_option(AP_OSD::OPTION_INVERTED_WIND);
#endif
        if (invert_wind) {
            v = -v;
        }
        uint8_t units = OSD_UNIT_METRIC;
#if OSD_ENABLED
        units = osd->units == AP_OSD::UNITS_IMPERIAL ?  OSD_UNIT_IMPERIAL : OSD_UNIT_METRIC;
#endif
        // if needed convert m/s to ft/s
        const float v_length = (units == OSD_UNIT_METRIC) ? v.length() : v.length() * 3.28084;
        const char* unit = (units == OSD_UNIT_METRIC) ? "m/s" : "f/s";

        if (v_length > 1.0f) {
            const int32_t angle = wrap_360_cd(DEGX100 * atan2f(v.y, v.x) - ahrs.yaw_sensor);
            const int32_t interval = 36000 / ARRAY_SIZE(arrows);
            uint8_t arrow = arrows[((angle + interval / 2) / interval) % ARRAY_SIZE(arrows)];
            snprintf(flight_mode_str, MSP_TXT_BUFFER_SIZE, "%s %d%s%c%c%c", notify->get_flight_mode_str(),  (uint8_t)roundf(v_length), unit, 0xE2, 0x86, arrow);
        } else {
            snprintf(flight_mode_str, MSP_TXT_BUFFER_SIZE, "%s ---%s", notify->get_flight_mode_str(), unit);
        }
    } else {
        /*
            Flight mode is rendered with simple mode flags
            examples:
                MANU
                MANU [S]
                MANU [SS]
        */
#ifndef HAL_NO_GCS
        const bool simple_mode = gcs().simple_input_active();
        const bool supersimple_mode = gcs().supersimple_input_active();
        const char* simple_mode_str = simple_mode ? " [S]" : (supersimple_mode ? " [SS]" : "");

        char buffer[MSP_TXT_BUFFER_SIZE] {};
        // flightmode
        const uint8_t used = snprintf(buffer, ARRAY_SIZE(buffer), "%s%s", notify->get_flight_mode_str(), simple_mode_str);
        // left pad
        uint8_t left_padded_len = MSP_TXT_VISIBLE_CHARS - (MSP_TXT_VISIBLE_CHARS - used)/2;
        snprintf(flight_mode_str, MSP_TXT_BUFFER_SIZE, "%*s", left_padded_len, buffer);
#endif
    }
}

void AP_MSP_Telem_Backend::enable_warnings()
{
    AP_MSP *msp = AP::msp();
    if (msp == nullptr) {
        return;
    }
    BIT_SET(msp->_osd_config.enabled_warnings, OSD_WARNING_FAIL_SAFE);
    BIT_SET(msp->_osd_config.enabled_warnings, OSD_WARNING_BATTERY_CRITICAL);
}

void AP_MSP_Telem_Backend::process_incoming_data()
{
    if (_msp_port.uart == nullptr) {
        return;
    }

    uint32_t numc = MIN(_msp_port.uart->available(), 1024U);

    if (numc > 0) {
        // Process incoming bytes
        while (numc-- > 0) {
            const uint8_t c = _msp_port.uart->read();
            msp_parse_received_data(&_msp_port, c);

            if (_msp_port.c_state == MSP_COMMAND_RECEIVED) {
                msp_process_received_command();
            }
        }
    }
}

/*
  ported from betaflight/src/main/msp/msp_serial.c
 */
void AP_MSP_Telem_Backend::msp_process_received_command()
{
    uint8_t out_buf[MSP_PORT_OUTBUF_SIZE];

    msp_packet_t reply = {
        .buf = { .ptr = out_buf, .end = MSP_ARRAYEND(out_buf), },
        .cmd = -1,
        .flags = 0,
        .result = 0,
    };
    uint8_t *out_buf_head = reply.buf.ptr;

    msp_packet_t command = {
        .buf = { .ptr = _msp_port.in_buf, .end = _msp_port.in_buf + _msp_port.data_size, },
        .cmd = (int16_t)_msp_port.cmd_msp,
        .flags = _msp_port.cmd_flags,
        .result = 0,
    };

    const MSPCommandResult status = msp_process_command(&command, &reply);

    if (status != MSP_RESULT_NO_REPLY) {
        sbuf_switch_to_reader(&reply.buf, out_buf_head); // change streambuf direction
        msp_serial_encode(&_msp_port, &reply, _msp_port.msp_version);
    }

    _msp_port.c_state = MSP_IDLE;
}

/*
  ported from inav/src/main/fc/fc_msp.c
 */
MSPCommandResult AP_MSP_Telem_Backend::msp_process_command(msp_packet_t *cmd, msp_packet_t *reply)
{
    MSPCommandResult ret = MSP_RESULT_ACK;
    sbuf_t *dst = &reply->buf;
    sbuf_t *src = &cmd->buf;
    const uint16_t cmd_msp = cmd->cmd;
    // initialize reply by default
    reply->cmd = cmd->cmd;

    if (MSP2_IS_SENSOR_MESSAGE(cmd_msp)) {
        ret = msp_process_sensor_command(cmd_msp, src);
    } else {
        ret = msp_process_out_command(cmd_msp, dst);
    }

    // Process DONT_REPLY flag
    if (cmd->flags & MSP_FLAG_DONT_REPLY) {
        ret = MSP_RESULT_NO_REPLY;
    }

    reply->result = ret;
    return ret;
}

MSPCommandResult AP_MSP_Telem_Backend::msp_process_out_command(uint16_t cmd_msp, sbuf_t *dst)
{
    switch (cmd_msp) {
    case MSP_API_VERSION:
        return msp_process_out_api_version(dst);
    case MSP_FC_VARIANT:
        return msp_process_out_fc_variant(dst);
    case MSP_FC_VERSION:
        return msp_process_out_fc_version(dst);
    case MSP_BOARD_INFO:
        return msp_process_out_board_info(dst);
    case MSP_BUILD_INFO:
        return msp_process_out_build_info(dst);
    case MSP_NAME:
        return msp_process_out_name(dst);
    case MSP_OSD_CONFIG:
        return msp_process_out_osd_config(dst);
    case MSP_STATUS:
    case MSP_STATUS_EX:
        return msp_process_out_status(dst);
    case MSP_RAW_GPS:
        return msp_process_out_raw_gps(dst);
    case MSP_COMP_GPS:
        return msp_process_out_comp_gps(dst);
    case MSP_ATTITUDE:
        return msp_process_out_attitude(dst);
    case MSP_ALTITUDE:
        return msp_process_out_altitude(dst);
    case MSP_ANALOG:
        return msp_process_out_analog(dst);
    case MSP_BATTERY_STATE:
        return msp_process_out_battery_state(dst);
    case MSP_UID:
        return msp_process_out_uid(dst);
#if HAL_WITH_ESC_TELEM
    case MSP_ESC_SENSOR_DATA:
        return msp_process_out_esc_sensor_data(dst);
#endif
    case MSP_RTC:
        return msp_process_out_rtc(dst);
    case MSP_RC:
        return msp_process_out_rc(dst);
    default:
        // MSP always requires an ACK even for unsupported messages
        return MSP_RESULT_ACK;
    }
}

MSPCommandResult AP_MSP_Telem_Backend::msp_process_sensor_command(uint16_t cmd_msp, sbuf_t *src)
{
    MSP_UNUSED(src);

    switch (cmd_msp) {
    case MSP2_SENSOR_RANGEFINDER: {
        const MSP::msp_rangefinder_data_message_t *pkt = (const MSP::msp_rangefinder_data_message_t *)src->ptr;
        msp_handle_rangefinder(*pkt);
    }
    break;
    case MSP2_SENSOR_OPTIC_FLOW: {
        const MSP::msp_opflow_data_message_t *pkt = (const MSP::msp_opflow_data_message_t *)src->ptr;
        msp_handle_opflow(*pkt);
    }
    break;
    case MSP2_SENSOR_GPS: {
        const MSP::msp_gps_data_message_t *pkt = (const MSP::msp_gps_data_message_t *)src->ptr;
        msp_handle_gps(*pkt);
    }
    break;
    case MSP2_SENSOR_COMPASS: {
        const MSP::msp_compass_data_message_t *pkt = (const MSP::msp_compass_data_message_t *)src->ptr;
        msp_handle_compass(*pkt);
    }
    break;
    case MSP2_SENSOR_BAROMETER: {
        const MSP::msp_baro_data_message_t *pkt = (const MSP::msp_baro_data_message_t *)src->ptr;
        msp_handle_baro(*pkt);
    }
    break;
    case MSP2_SENSOR_AIRSPEED: {
        const MSP::msp_airspeed_data_message_t *pkt = (const MSP::msp_airspeed_data_message_t *)src->ptr;
        msp_handle_airspeed(*pkt);
    }
    break;
    }

    return MSP_RESULT_NO_REPLY;
}

void AP_MSP_Telem_Backend::msp_handle_opflow(const MSP::msp_opflow_data_message_t &pkt)
{
#if HAL_MSP_OPTICALFLOW_ENABLED
    OpticalFlow *optflow = AP::opticalflow();
    if (optflow == nullptr) {
        return;
    }
    optflow->handle_msp(pkt);
#endif
}

void AP_MSP_Telem_Backend::msp_handle_rangefinder(const MSP::msp_rangefinder_data_message_t &pkt)
{
#if HAL_MSP_RANGEFINDER_ENABLED
    RangeFinder *rangefinder = AP::rangefinder();
    if (rangefinder == nullptr) {
        return;
    }
    rangefinder->handle_msp(pkt);
#endif
}

void AP_MSP_Telem_Backend::msp_handle_gps(const MSP::msp_gps_data_message_t &pkt)
{
#if HAL_MSP_GPS_ENABLED
    AP::gps().handle_msp(pkt);
#endif
}

void AP_MSP_Telem_Backend::msp_handle_compass(const MSP::msp_compass_data_message_t &pkt)
{
#if HAL_MSP_COMPASS_ENABLED
    AP::compass().handle_msp(pkt);
#endif
}

void AP_MSP_Telem_Backend::msp_handle_baro(const MSP::msp_baro_data_message_t &pkt)
{
#if HAL_MSP_BARO_ENABLED
    AP::baro().handle_msp(pkt);
#endif
}

void AP_MSP_Telem_Backend::msp_handle_airspeed(const MSP::msp_airspeed_data_message_t &pkt)
{
#if HAL_MSP_AIRSPEED_ENABLED
    auto *airspeed = AP::airspeed();
    if (airspeed) {
        airspeed->handle_msp(pkt);
    }
#endif
}

MSPCommandResult AP_MSP_Telem_Backend::msp_process_out_raw_gps(sbuf_t *dst)
{
#if OSD_ENABLED
    AP_OSD *osd = AP::osd();
    if (osd == nullptr) {
        return MSP_RESULT_ERROR;
    }
#endif
    gps_state_t gps_state;
    update_gps_state(gps_state);

    // handle airspeed override
    bool airspeed_en = false;
#if OSD_ENABLED
    airspeed_en = osd->screen[0].aspeed.enabled;
#endif
    if (airspeed_en) {
        airspeed_state_t airspeed_state;
        update_airspeed(airspeed_state);
        gps_state.speed_cms = airspeed_state.airspeed_estimate_ms * 100;           // airspeed in cm/s
    }

    sbuf_write_data(dst, &gps_state, sizeof(gps_state));
    return MSP_RESULT_ACK;
}

MSPCommandResult AP_MSP_Telem_Backend::msp_process_out_comp_gps(sbuf_t *dst)
{
    home_state_t home_state;
    update_home_pos(home_state);

    // no need to apply yaw compensation, the DJI air unit will do it for us :-)
    int32_t home_angle_deg = home_state.home_bearing_cd * 0.01;
    if (home_state.home_distance_m < 2) {
        //avoid fast rotating arrow at small distances
        home_angle_deg = 0;
    }

    struct PACKED {
        uint16_t dist_home_m;
        uint16_t home_angle_deg;
        uint8_t toggle_gps;
    } p;

    p.dist_home_m = home_state.home_distance_m;
    p.home_angle_deg = home_angle_deg;
    p.toggle_gps = 1;

    sbuf_write_data(dst, &p, sizeof(p));

    return MSP_RESULT_ACK;
}

// Autoscroll message is the same as in minimosd-extra.
// Thanks to night-ghost for the approach.
MSPCommandResult AP_MSP_Telem_Backend::msp_process_out_name(sbuf_t *dst)
{
#if OSD_ENABLED
    AP_OSD *osd = AP::osd();
    if (osd == nullptr) {
        return MSP_RESULT_ERROR;
    }
#endif
    AP_MSP *msp = AP::msp();
    if (msp == nullptr) {
        return MSP_RESULT_ERROR;
    }
    AP_Notify * notify = AP_Notify::get_singleton();
    if (notify) {
        uint16_t msgtime_ms = 10000; //default is 10 secs
#if OSD_ENABLED
        msgtime_ms = AP::osd()->msgtime_s * 1000;
#endif
        // text message is visible for _msp.msgtime_s but only if
        // a flight mode change did not steal focus
        const uint32_t visible_time_ms = AP_HAL::millis() - notify->get_text_updated_millis();
        if (visible_time_ms < msgtime_ms && !msp->_msp_status.flight_mode_focus) {
            char buffer[NOTIFY_TEXT_BUFFER_SIZE];
            strncpy(buffer, notify->get_text(), ARRAY_SIZE(buffer));
            const uint8_t len = strnlen(buffer, ARRAY_SIZE(buffer));

            for (uint8_t i=0; i<len; i++) {
                //normalize whitespace
                if (isspace(buffer[i])) {
                    buffer[i] = ' ';
                } else {
                    //converted to uppercase,
                    buffer[i] = toupper(buffer[i]);
                }
            }

            int8_t start_position = 0;
            //scroll if required
            //scroll pattern: wait, scroll to the left, wait, scroll to the right
            if (len > MSP_TXT_VISIBLE_CHARS) {
                const uint8_t chars_to_scroll = len - MSP_TXT_VISIBLE_CHARS;
                const uint8_t total_cycles = 2*message_scroll_delay + 2*chars_to_scroll;
                const uint8_t current_cycle = (visible_time_ms / message_scroll_time_ms) % total_cycles;

                //calculate scroll start_position
                if (current_cycle < total_cycles/2) {
                    //move to the left
                    start_position = current_cycle - message_scroll_delay;
                } else {
                    //move to the right
                    start_position = total_cycles - current_cycle;
                }
                start_position = constrain_int16(start_position, 0, chars_to_scroll);
                uint8_t end_position = start_position + MSP_TXT_VISIBLE_CHARS;

                //ensure array boundaries
                start_position = MIN(start_position, int8_t(ARRAY_SIZE(buffer)-1));
                end_position = MIN(end_position, int8_t(ARRAY_SIZE(buffer)-1));

                //trim invisible part
                buffer[end_position] = 0;
            }

            sbuf_write_data(dst, buffer + start_position, strlen(buffer + start_position));  // max MSP_TXT_VISIBLE_CHARS chars general text...
        } else {
            bool wind_en = false;
            char flight_mode_str[MSP_TXT_BUFFER_SIZE];
#if OSD_ENABLED
            wind_en = osd->screen[0].wind.enabled;
#endif
            update_flight_mode_str(flight_mode_str, wind_en);
            sbuf_write_data(dst, flight_mode_str, ARRAY_SIZE(flight_mode_str));  // rendered as up to MSP_TXT_VISIBLE_CHARS chars with UTF8 support
        }
    }
    return MSP_RESULT_ACK;
}

MSPCommandResult AP_MSP_Telem_Backend::msp_process_out_status(sbuf_t *dst)
{
    const uint32_t mode_bitmask = get_osd_flight_mode_bitmask();
    sbuf_write_u16(dst, 0);                     // task delta time
    sbuf_write_u16(dst, 0);                     // I2C error count
    sbuf_write_u16(dst, 0);                     // sensor status
    sbuf_write_data(dst, &mode_bitmask, 4);     // unconditional part of flags, first 32 bits
    sbuf_write_u8(dst, 0);

    sbuf_write_u16(dst, constrain_int16(0, 0, 100));  //system load
    sbuf_write_u16(dst, 0);                     // gyro cycle time

    // Cap BoxModeFlags to 32 bits
    sbuf_write_u8(dst, 0);

    // Write arming disable flags
    sbuf_write_u8(dst, 1);
    sbuf_write_u32(dst, !AP::notify().flags.armed);

    // Extra flags
    sbuf_write_u8(dst, 0);
    return MSP_RESULT_ACK;
}

MSPCommandResult AP_MSP_Telem_Backend::msp_process_out_osd_config(sbuf_t *dst)
{
#if OSD_ENABLED
    AP_OSD *osd = AP::osd();
    if (osd == nullptr) {
        return MSP_RESULT_ERROR;
    }
#endif
    const AP_MSP *msp = AP::msp();
    if (msp == nullptr) {
        return MSP_RESULT_ERROR;
    }
    sbuf_write_u8(dst, OSD_FLAGS_OSD_FEATURE);                      // flags
    sbuf_write_u8(dst, 0);                                          // video system
    // Configuration
    uint8_t units = OSD_UNIT_METRIC;
#if OSD_ENABLED
    units = osd->units == AP_OSD::UNITS_METRIC ? OSD_UNIT_METRIC : OSD_UNIT_IMPERIAL;
#endif

    sbuf_write_u8(dst, units);                                 // units
    // Alarms
    sbuf_write_u8(dst, msp->_osd_config.rssi_alarm);                 // rssi alarm
    sbuf_write_u16(dst, msp->_osd_config.cap_alarm);                 // capacity alarm
    // Reuse old timer alarm (U16) as OSD_ITEM_COUNT
    sbuf_write_u8(dst, 0);
    sbuf_write_u8(dst, OSD_ITEM_COUNT);                             // osd items count

    sbuf_write_u16(dst, msp->_osd_config.alt_alarm);                 // altitude alarm

    // element position and visibility
    uint16_t pos = 0;   // default is hide this element
    for (uint8_t i = 0; i < OSD_ITEM_COUNT; i++) {
        pos = 0;    // 0 is hide this item
        if (msp->_osd_item_settings[i] != nullptr) {      // ok supported
            if (msp->_osd_item_settings[i]->enabled) {    // ok enabled
                // let's check if we need to hide this dynamically
                if (!BIT_IS_SET(osd_hidden_items_bitmask, i)) {
                    pos = MSP_OSD_POS(msp->_osd_item_settings[i]);
                }
            }
        }
        sbuf_write_u16(dst, pos);
    }

    // post flight statistics
    sbuf_write_u8(dst, OSD_STAT_COUNT);                         // stats items count
    for (uint8_t i = 0; i < OSD_STAT_COUNT; i++ ) {
        sbuf_write_u16(dst, 0);                                 // stats not supported
    }

    // timers
    sbuf_write_u8(dst, OSD_TIMER_COUNT);                      // timers
    for (uint8_t i = 0; i < OSD_TIMER_COUNT; i++) {
        // no timer support
        sbuf_write_u16(dst, 0);
    }

    // Enabled warnings
    // API < 1.41
    // Send low word first for backwards compatibility
    sbuf_write_u16(dst, (uint16_t)(msp->_osd_config.enabled_warnings & 0xFFFF)); // Enabled warnings
    // API >= 1.41
    // Send the warnings count and 32bit enabled warnings flags.
    // Add currently active OSD profile (0 indicates OSD profiles not available).
    // Add OSD stick overlay mode (0 indicates OSD stick overlay not available).
    sbuf_write_u8(dst, OSD_WARNING_COUNT);            // warning count
    sbuf_write_u32(dst, msp->_osd_config.enabled_warnings);  // enabled warning

    // If the feature is not available there is only 1 profile and it's always selected
    sbuf_write_u8(dst, 1);    // available profiles
    sbuf_write_u8(dst, 1);    // selected profile

    sbuf_write_u8(dst, 0);    // OSD stick overlay

    // API >= 1.43
    // Add the camera frame element width/height
    //sbuf_write_u8(dst, osdConfig()->camera_frame_width);
    //sbuf_write_u8(dst, osdConfig()->camera_frame_height);
    return MSP_RESULT_ACK;
}

MSPCommandResult AP_MSP_Telem_Backend::msp_process_out_attitude(sbuf_t *dst)
{
    AP_AHRS &ahrs = AP::ahrs();
    WITH_SEMAPHORE(ahrs.get_semaphore());

    struct PACKED {
        int16_t roll;
        int16_t pitch;
        int16_t yaw;
    } attitude;

    attitude.roll = ahrs.roll_sensor * 0.1;     // centidegress to decidegrees
    attitude.pitch = ahrs.pitch_sensor * 0.1;   // centidegress to decidegrees
    attitude.yaw = ahrs.yaw_sensor * 0.01;      // centidegress to degrees

    sbuf_write_data(dst, &attitude, sizeof(attitude));
    return MSP_RESULT_ACK;
}

MSPCommandResult AP_MSP_Telem_Backend::msp_process_out_altitude(sbuf_t *dst)
{
    home_state_t home_state;
    update_home_pos(home_state);

    sbuf_write_u32(dst, home_state.rel_altitude_cm);                // relative altitude cm
    sbuf_write_u16(dst, int16_t(get_vspeed_ms() * 100));            // climb rate cm/s
    return MSP_RESULT_ACK;
}

MSPCommandResult AP_MSP_Telem_Backend::msp_process_out_analog(sbuf_t *dst)
{
    AP_RSSI* rssi = AP::rssi();
    if (rssi == nullptr) {
        return MSP_RESULT_ERROR;
    }
    battery_state_t battery_state;
    update_battery_state(battery_state);

    struct PACKED {
        uint8_t voltage_dv;
        uint16_t mah;
        uint16_t rssi;
        int16_t current_ca;
        uint16_t voltage_cv;
    } battery;

    battery.voltage_dv = constrain_int16(battery_state.batt_voltage_v * 10, 0, 255);            // battery voltage V to dV
    battery.mah = constrain_int32(battery_state.batt_consumed_mah, 0, 0xFFFF);                  // milliamp hours drawn from battery
    battery.rssi = rssi->enabled() ? rssi->read_receiver_rssi() * 1023 : 0;                     // rssi 0-1 to 0-1023
    battery.current_ca = constrain_int32(battery_state.batt_current_a * 100, -0x8000, 0x7FFF);  // current A to cA (0.01 steps, range is -320A to 320A)
    battery.voltage_cv = constrain_int32(battery_state.batt_voltage_v * 100,0,0xFFFF);          // battery voltage in 0.01V steps

    sbuf_write_data(dst, &battery, sizeof(battery));
    return MSP_RESULT_ACK;
}

MSPCommandResult AP_MSP_Telem_Backend::msp_process_out_battery_state(sbuf_t *dst)
{
    const AP_MSP *msp = AP::msp();
    if (msp == nullptr) {
        return MSP_RESULT_ERROR;
    }
    battery_state_t battery_state;
    update_battery_state(battery_state);

    struct PACKED {
        uint8_t cellcount;
        uint16_t capacity_mah;
        uint8_t voltage_dv;
        uint16_t mah;
        int16_t current_ca;
        uint8_t state;
        uint16_t voltage_cv;
    } battery;

    battery.cellcount = constrain_int16((msp->_cellcount > 0 ? msp->_cellcount : battery_state.batt_cellcount), 0, 255);    // cell count 0 indicates battery not detected.
    battery.capacity_mah = battery_state.batt_capacity_mah;                                                                          // in mAh
    battery.voltage_dv = constrain_int16(battery_state.batt_voltage_v * 10, 0, 255);                                        // battery voltage V to dV
    battery.mah = MIN(battery_state.batt_consumed_mah, 0xFFFF);                                                             // milliamp hours drawn from battery
    battery.current_ca = constrain_int32(battery_state.batt_current_a * 100, -0x8000, 0x7FFF);                              // current A to cA (0.01 steps, range is -320A to 320A)
    battery.state = battery_state.batt_state;                                                                               // BATTERY: OK=0, CRITICAL=2
    battery.voltage_cv = constrain_int32(battery_state.batt_voltage_v * 100, 0, 0x7FFF);                                    // battery voltage in 0.01V steps

    sbuf_write_data(dst, &battery, sizeof(battery));
    return MSP_RESULT_ACK;
}

MSPCommandResult AP_MSP_Telem_Backend::msp_process_out_esc_sensor_data(sbuf_t *dst)
{
#if HAL_WITH_ESC_TELEM
    AP_ESC_Telem& telem = AP::esc_telem();
    if (telem.get_last_telem_data_ms(0)) {
        const uint8_t num_motors = telem.get_num_active_escs();
        sbuf_write_u8(dst, num_motors);
        for (uint8_t i = 0; i < num_motors; i++) {
            int16_t temp = 0;
            float rpm = 0.0f;
            telem.get_rpm(i, rpm);
            telem.get_temperature(i, temp);
            sbuf_write_u8(dst, uint8_t(temp / 100));        // deg
            sbuf_write_u16(dst, uint16_t(rpm * 0.1));
        }
    }
#endif
    return MSP_RESULT_ACK;
}

MSPCommandResult AP_MSP_Telem_Backend::msp_process_out_rtc(sbuf_t *dst)
{
    tm localtime_tm {}; // year is relative to 1900
    uint64_t time_usec = 0;
    if (AP::rtc().get_utc_usec(time_usec)) { // may fail, leaving time_unix at 0
        const time_t time_sec = time_usec / 1000000;
        localtime_tm = *gmtime(&time_sec);
    }
    struct PACKED {
        uint16_t year;
        uint8_t mon;
        uint8_t mday;
        uint8_t hour;
        uint8_t min;
        uint8_t sec;
        uint16_t millis;
    } rtc;

    rtc.year = localtime_tm.tm_year + 1900;   // tm_year is relative to year 1900
    rtc.mon = localtime_tm.tm_mon + 1;        // MSP requires [1-12] months
    rtc.mday = localtime_tm.tm_mday;
    rtc.hour = localtime_tm.tm_hour;
    rtc.min = localtime_tm.tm_min;
    rtc.sec = localtime_tm.tm_sec;
    rtc.millis = (time_usec / 1000U) % 1000U;

    sbuf_write_data(dst, &rtc, sizeof(rtc));
    return MSP_RESULT_ACK;
}

MSPCommandResult AP_MSP_Telem_Backend::msp_process_out_rc(sbuf_t *dst)
{
    const RCMapper* rcmap = AP::rcmap();
    if (rcmap == nullptr) {
        return MSP_RESULT_ERROR;
    }
    uint16_t values[16] = {};
    rc().get_radio_in(values, ARRAY_SIZE(values));

    struct PACKED {
        uint16_t a;
        uint16_t e;
        uint16_t r;
        uint16_t t;
    } rc;

    // send only 4 channels, MSP order is AERT
    rc.a = values[rcmap->roll()];       // A
    rc.e = values[rcmap->pitch()];      // E
    rc.r = values[rcmap->yaw()];        // R
    rc.t = values[rcmap->throttle()];   // T

    sbuf_write_data(dst, &rc, sizeof(rc));
    return MSP_RESULT_ACK;
}

MSPCommandResult AP_MSP_Telem_Backend::msp_process_out_board_info(sbuf_t *dst)
{
    const AP_FWVersion &fwver = AP::fwversion();

    sbuf_write_data(dst, "ARDU", BOARD_IDENTIFIER_LENGTH);
    sbuf_write_u16(dst, 0);
    sbuf_write_u8(dst, 0);
    sbuf_write_u8(dst, 0);
    sbuf_write_u8(dst, strlen(fwver.fw_string));
    sbuf_write_data(dst, fwver.fw_string, strlen(fwver.fw_string));
    return MSP_RESULT_ACK;
}

MSPCommandResult AP_MSP_Telem_Backend::msp_process_out_build_info(sbuf_t *dst)
{
    const AP_FWVersion &fwver = AP::fwversion();

    // we don't use real dates here as that would mean we don't get
    // consistent builds. Being able to reproduce the exact build at a
    // later date is a valuable property of the code
    sbuf_write_data(dst, "Jan 01 1980", BUILD_DATE_LENGTH);
    sbuf_write_data(dst, "00:00:00", BUILD_TIME_LENGTH);
    sbuf_write_data(dst, fwver.fw_hash_str, GIT_SHORT_REVISION_LENGTH);
    return MSP_RESULT_ACK;
}

MSPCommandResult AP_MSP_Telem_Backend::msp_process_out_uid(sbuf_t *dst)
{
    uint8_t id[12] {};
    uint8_t len = sizeof(id);
    hal.util->get_system_id_unformatted(id, len);
    sbuf_write_data(dst, id, sizeof(id));
    return MSP_RESULT_ACK;
}

void AP_MSP_Telem_Backend::hide_osd_items(void)
{
#if OSD_ENABLED
    AP_OSD *osd = AP::osd();
    if (osd == nullptr) {
        return;
    }
#endif
    AP_MSP *msp = AP::msp();
    if (msp == nullptr) {
        return;
    }
    const AP_Notify &notify = AP::notify();
    // clear all and only set the flashing ones
    BIT_CLEAR(osd_hidden_items_bitmask, OSD_GPS_SATS);
    BIT_CLEAR(osd_hidden_items_bitmask, OSD_HOME_DIR);
    BIT_CLEAR(osd_hidden_items_bitmask, OSD_HOME_DIST);
    BIT_CLEAR(osd_hidden_items_bitmask, OSD_GPS_SPEED);
    BIT_CLEAR(osd_hidden_items_bitmask, OSD_CRAFT_NAME);
    BIT_CLEAR(osd_hidden_items_bitmask, OSD_AVG_CELL_VOLTAGE);
    BIT_CLEAR(osd_hidden_items_bitmask, OSD_MAIN_BATT_VOLTAGE);
    BIT_CLEAR(osd_hidden_items_bitmask, OSD_RTC_DATETIME);

    if (msp->_msp_status.flashing_on) {
        // flash satcount when no 3D Fix
        gps_state_t gps_state;
        update_gps_state(gps_state);
        if (gps_state.fix_type == 0) {
            BIT_SET(osd_hidden_items_bitmask, OSD_GPS_SATS);
        }
        // flash home dir and distance if home is not set
        home_state_t home_state;
        update_home_pos(home_state);
        if (!home_state.home_is_set) {
            BIT_SET(osd_hidden_items_bitmask, OSD_HOME_DIR);
            BIT_SET(osd_hidden_items_bitmask, OSD_HOME_DIST);
        }
        // flash airspeed if there's no estimate
        bool airspeed_en = false;
#if OSD_ENABLED
        airspeed_en = osd->screen[0].aspeed.enabled;
#endif
        if (airspeed_en) {
            airspeed_state_t airspeed_state;
            update_airspeed(airspeed_state);
            if (!airspeed_state.airspeed_have_estimate) {
                BIT_SET(osd_hidden_items_bitmask, OSD_GPS_SPEED);
            }
        }
        // flash text flightmode for 3secs after each change
        if (msp->_msp_status.flight_mode_focus) {
            BIT_SET(osd_hidden_items_bitmask, OSD_CRAFT_NAME);
        }
        // flash battery on failsafe
        if (notify.flags.failsafe_battery) {
            BIT_SET(osd_hidden_items_bitmask, OSD_AVG_CELL_VOLTAGE);
            BIT_SET(osd_hidden_items_bitmask, OSD_MAIN_BATT_VOLTAGE);
        }
        // flash rtc if no time available
        uint64_t time_usec;
        if (!AP::rtc().get_utc_usec(time_usec)) {
            BIT_SET(osd_hidden_items_bitmask, OSD_RTC_DATETIME);
        }
    }
}

#endif //HAL_MSP_ENABLED
