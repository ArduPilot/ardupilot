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

   MSP telemetry library backend base class
*/
#pragma once

#include "AP_MSP_config.h"

#if HAL_MSP_ENABLED

#include <AP_RCTelemetry/AP_RCTelemetry.h>
#include <AP_SerialManager/AP_SerialManager.h>
#include <AP_OSD/AP_OSD.h>

#include "msp.h"

#include <time.h>

#define MSP_TIME_SLOT_MAX 12
#define CELLFULL 4.35
#define MSP_TXT_BUFFER_SIZE     15U // 11 + 3 utf8 chars + terminator
#define MSP_TXT_VISIBLE_CHARS   11U

class AP_MSP;

class AP_MSP_Telem_Backend : AP_RCTelemetry
{
friend AP_MSP;
public:
    AP_MSP_Telem_Backend(AP_HAL::UARTDriver *uart);

    typedef struct battery_state_s {
        float batt_current_a;
        float batt_consumed_mah;
        float batt_voltage_v;
        int32_t batt_capacity_mah;
        uint8_t batt_cellcount;
        MSP::battery_state_e batt_state;
    } battery_state_t;

    typedef struct PACKED gps_state_s {
        uint8_t fix_type;
        uint8_t num_sats;
        int32_t lat;
        int32_t lon;
        uint16_t alt_m;
        uint16_t speed_cms;
        uint16_t ground_course_dd;
    } gps_state_t;

    typedef struct airspeed_state_s {
        float airspeed_estimate_ms;
        bool  airspeed_have_estimate;
    } airspeed_state_t;

    typedef struct home_state_s {
        bool home_is_set;
        float home_bearing_cd;
        uint32_t home_distance_m;
        int32_t rel_altitude_cm;
    } home_state_t;

    // init - perform required initialisation
    virtual bool init() override;
    virtual bool init_uart();
    virtual void enable_warnings();
    virtual void hide_osd_items(void);

    // MSP tx/rx processors
    void process_incoming_data();     // incoming data
    void process_outgoing_data();     // push outgoing data

#if HAL_WITH_MSP_DISPLAYPORT
    // displayport commands
    // betaflight/src/main/io/displayport_msp.c
    virtual void msp_displayport_heartbeat();
    virtual void msp_displayport_grab();
    virtual void msp_displayport_release();
    virtual void msp_displayport_clear_screen();
    virtual void msp_displayport_draw_screen();
    virtual void msp_displayport_write_string(uint8_t col, uint8_t row, bool blink, const char *string, const uint8_t font_table);
    virtual void msp_displayport_set_options(const uint8_t font_index, const uint8_t screen_resolution);
#endif
protected:
    enum msp_packet_type : uint8_t {
        EMPTY_SLOT = 0,
        NAME,
        STATUS,
        CONFIG,
        RAW_GPS,
        COMP_GPS,
        ATTITUDE,
        ALTITUDE,
        ANALOG,
        BATTERY_STATE,
#if HAL_WITH_ESC_TELEM
        ESC_SENSOR_DATA,
#endif
        RTC_DATETIME,
    };

    const uint16_t msp_packet_type_map[MSP_TIME_SLOT_MAX] = {
        0,
        MSP_NAME,
        MSP_STATUS,
        MSP_OSD_CONFIG,
        MSP_RAW_GPS,
        MSP_COMP_GPS,
        MSP_ATTITUDE,
        MSP_ALTITUDE,
        MSP_ANALOG,
        MSP_BATTERY_STATE,
#if HAL_WITH_ESC_TELEM
        MSP_ESC_SENSOR_DATA,
#endif
        MSP_RTC
    };

    /* UTF-8 encodings
        U+2191 ↑       e2 86 91        UPWARDS ARROW
        U+2197 ↗       e2 86 97        NORTH EAST ARROW
        U+2192 →       e2 86 92        RIGHTWARDS ARROW
        U+2198 ↘       e2 86 98        SOUTH EAST ARROW
        U+2193 ↓       e2 86 93        DOWNWARDS ARROW
        U+2199 ↙       e2 86 99        SOUTH WEST ARROW
        U+2190 ←       e2 86 90        LEFTWARDS ARROW
        U+2196 ↖       e2 86 96        NORTH WEST ARROW
    */
    static constexpr uint8_t arrows[8] = {0x91, 0x97, 0x92, 0x98, 0x93, 0x99, 0x90, 0x96};

    static const uint8_t message_scroll_time_ms = 200;
    static const uint8_t message_scroll_delay = 5;

    // each backend can hide/unhide items dynamically
    uint64_t osd_hidden_items_bitmask;

    // MSP decoder status
    MSP::msp_port_t _msp_port;

    // passthrough WFQ scheduler
    bool is_packet_ready(uint8_t idx, bool queue_empty) override;
    void process_packet(uint8_t idx) override;
    void adjust_packet_weight(bool queue_empty) override {}
    void setup_wfq_scheduler(void) override;
    bool get_next_msg_chunk(void) override
    {
        return true;
    }

    // telemetry helpers
    uint8_t calc_cell_count(float battery_voltage);
    virtual float get_vspeed_ms(void) const;
    virtual bool get_rssi(float &rssi) const;
    virtual void update_home_pos(home_state_t &home_state);
    virtual void update_battery_state(battery_state_t &_battery_state);
    virtual void update_gps_state(gps_state_t &gps_state);
    virtual void update_airspeed(airspeed_state_t &airspeed_state);
    virtual void update_flight_mode_str(char *flight_mode_str, uint8_t size, bool wind_enabled);

    // MSP parsing
    void msp_process_received_command();
    MSP::MSPCommandResult msp_process_command(MSP::msp_packet_t *cmd, MSP::msp_packet_t *reply);
    MSP::MSPCommandResult msp_process_sensor_command(uint16_t cmd_msp, MSP::sbuf_t *src);
    MSP::MSPCommandResult msp_process_out_command(uint16_t cmd_msp, MSP::sbuf_t *dst);

    // MSP send
    void msp_send_packet(uint16_t cmd, MSP::msp_version_e msp_version, const void *p, uint16_t size, bool is_request);

    // MSP sensor command processing
    void msp_handle_opflow(const MSP::msp_opflow_data_message_t &pkt);
    void msp_handle_rangefinder(const MSP::msp_rangefinder_data_message_t &pkt);
    void msp_handle_gps(const MSP::msp_gps_data_message_t &pkt);
    void msp_handle_compass(const MSP::msp_compass_data_message_t &pkt);
    void msp_handle_baro(const MSP::msp_baro_data_message_t &pkt);
    void msp_handle_airspeed(const MSP::msp_airspeed_data_message_t &pkt);

    // implementation specific helpers
    // we only set arming status
    // custom masks are needed for vendor specific settings
    virtual uint32_t get_osd_flight_mode_bitmask(void);

    virtual bool is_scheduler_enabled() const = 0;                            // only osd backends should allow a push type telemetry
    virtual bool use_msp_thread() const {return true;};                       // is this backend hanlded by the MSP thread?
    virtual AP_SerialManager::SerialProtocol get_serial_protocol() const = 0;
    virtual bool displaying_stats_screen() const;

    // implementation specific MSP out command processing
    virtual MSP::MSPCommandResult msp_process_out_api_version(MSP::sbuf_t *dst);
    virtual MSP::MSPCommandResult msp_process_out_fc_version(MSP::sbuf_t *dst);
    virtual MSP::MSPCommandResult msp_process_out_fc_variant(MSP::sbuf_t *dst);
    virtual MSP::MSPCommandResult msp_process_out_uid(MSP::sbuf_t *dst);
    virtual MSP::MSPCommandResult msp_process_out_board_info(MSP::sbuf_t *dst);
    virtual MSP::MSPCommandResult msp_process_out_build_info(MSP::sbuf_t *dst);
    virtual MSP::MSPCommandResult msp_process_out_name(MSP::sbuf_t *dst);
    virtual MSP::MSPCommandResult msp_process_out_status(MSP::sbuf_t *dst);
    virtual MSP::MSPCommandResult msp_process_out_osd_config(MSP::sbuf_t *dst);
    virtual MSP::MSPCommandResult msp_process_out_raw_gps(MSP::sbuf_t *dst);
    virtual MSP::MSPCommandResult msp_process_out_comp_gps(MSP::sbuf_t *dst);
    virtual MSP::MSPCommandResult msp_process_out_attitude(MSP::sbuf_t *dst);
    virtual MSP::MSPCommandResult msp_process_out_altitude(MSP::sbuf_t *dst);
    virtual MSP::MSPCommandResult msp_process_out_analog(MSP::sbuf_t *dst);
    virtual MSP::MSPCommandResult msp_process_out_battery_state(MSP::sbuf_t *dst);
    virtual MSP::MSPCommandResult msp_process_out_esc_sensor_data(MSP::sbuf_t *dst);
    virtual MSP::MSPCommandResult msp_process_out_rtc(MSP::sbuf_t *dst);
    virtual MSP::MSPCommandResult msp_process_out_rc(MSP::sbuf_t *dst);
};

#endif  //HAL_MSP_ENABLED
