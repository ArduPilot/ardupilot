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
#pragma once

#include "AP_AIS_config.h"

#if AP_AIS_ENABLED
// 0 fully disabled and compiled out
// 1 compiled in and enabled
// 2 compiled in with dummy methods, none functional, except rover which never uses dummy methods functionality

#include <AP_Param/AP_Param.h>
#include <AP_Common/AP_ExpandingArray.h>
#include <GCS_MAVLink/GCS_MAVLink.h>

#define AIVDM_BUFFER_SIZE 10
#define AIVDM_PAYLOAD_SIZE 65

#include <AP_NMEA_Input/AP_NMEA_Input.h>

class AP_AIS : AP_NMEA_Input
{
public:
    AP_AIS();

    CLASS_NO_COPY(AP_AIS);

    // get singleton instance
    static AP_AIS *get_singleton();

    // return true if AIS is enabled
    bool enabled() const;

    // Initialize the AIS object and prepare it for use
    void init();

    // update AIS, expected to be called at 20hz
    void update();

    // send mavlink AIS message
    void send(mavlink_channel_t chan);

    // parameter block
    static const struct AP_Param::GroupInfo var_info[];

private:

    // methods required to be a AP_NMEA_Input
    void handle_decode_success() override;
    bool start_sentence_type(const char *term_type) override;
    bool handle_term(uint8_t term_number, const char *term) override;

    // parameters
    AP_Int8 _type;             // type of AIS receiver
    AP_Int16 _max_list;        // maximum number of vessels to track at once
    AP_Int16 _time_out;        // time in seconds that a vessel will be dropped from the list
    AP_Int16 _log_options;     // logging options bitmask

    enum class AISType {
        NONE   = 0,
        NMEA   = 1,
    };

    enum options {
        AIS_OPTIONS_LOG_ALL_RAW         = 1<<0,
        AIS_OPTIONS_LOG_UNSUPPORTED_RAW = 1<<1,
        AIS_OPTIONS_LOG_DECODED         = 1<<2,
    };

    struct AIVDM {
        uint8_t num;
        uint8_t total;
        uint8_t ID;
        char payload[AIVDM_PAYLOAD_SIZE];
    };
    AIVDM _incoming;
    AIVDM _AIVDM_buffer[AIVDM_BUFFER_SIZE];

    struct ais_vehicle_t {
        mavlink_ais_vessel_t info;
        uint32_t last_update_ms; // last time this was refreshed, allows timeouts
        uint32_t last_send_ms; // last time this message was sent via mavlink, stops us spamming the link
    };

    // list of the vessels that are being tracked
    AP_ExpandingArray<ais_vehicle_t> _list {8};

    uint16_t _send_index; // index of the last vessel send over mavlink

    // removed the given index from the AIVDM buffer shift following elements
    void buffer_shift(uint8_t i);

    // find vessel in existing list, if not then return NEW_NOTHROW index if possible
    bool get_vessel_index(uint32_t mmsi, uint16_t &index, uint32_t lat = 0, uint32_t lon = 0) WARN_IF_UNUSED;
    void clear_list_item(uint16_t index);

    // decode the payload
    bool payload_decode(const char *payload) WARN_IF_UNUSED;

    // decode specific message types
    bool decode_position_report(const char *payload, uint8_t type) WARN_IF_UNUSED;
    bool decode_base_station_report(const char *payload) WARN_IF_UNUSED;
    bool decode_static_and_voyage_data(const char *payload) WARN_IF_UNUSED;

    // read the specified bits from the char array each char giving 6 bits
    void get_char(const char *payload, char *array, uint16_t low, uint16_t high);
    uint32_t get_bits(const char *payload, uint16_t low, uint16_t high);
    int32_t get_bits_signed(const char *payload, uint16_t low, uint16_t high);
    // un-encode the ASCII payload armoring
    uint8_t payload_char_decode(const char c);

    // log a raw AIVDM message
    void log_raw(const AIVDM *msg);

    static AP_AIS *_singleton;
};

#endif  // AP_AIS_ENABLED
