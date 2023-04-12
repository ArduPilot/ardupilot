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

// Automatic Identification System, https://gpsd.gitlab.io/gpsd/AIVDM.html

// ToDo: enable receiving of the Mavlink AIS message, type bitmask?

#include "AP_AIS.h"

#if AP_AIS_ENABLED

#include <AP_Vehicle/AP_Vehicle_Type.h>

#define AP_AIS_DUMMY_METHODS_ENABLED ((AP_AIS_ENABLED == 2) && !APM_BUILD_TYPE(APM_BUILD_Rover))

#if !AP_AIS_DUMMY_METHODS_ENABLED

#include <AP_Logger/AP_Logger.h>
#include <AP_SerialManager/AP_SerialManager.h>
#include <GCS_MAVLink/GCS_MAVLink.h>
#include <GCS_MAVLink/GCS.h>

const AP_Param::GroupInfo AP_AIS::var_info[] = {

    // @Param: TYPE
    // @DisplayName: AIS receiver type
    // @Description: AIS receiver type
    // @Values: 0:None,1:NMEA AIVDM message
    // @User: Standard
    // @RebootRequired: True
    AP_GROUPINFO_FLAGS("TYPE", 1, AP_AIS, _type, 0, AP_PARAM_FLAG_ENABLE),

    // @Param: LIST_MAX
    // @DisplayName: AIS vessel list size
    // @Description: AIS list size of nearest vessels. Longer lists take longer to refresh with lower SRx_ADSB values.
    // @Range: 1 100
    // @User: Advanced
    AP_GROUPINFO("LIST_MAX", 2, AP_AIS, _max_list, 25),

    // @Param: TIME_OUT
    // @DisplayName: AIS vessel time out
    // @Description: if no updates are received in this time a vessel will be removed from the list
    // @Units: s
    // @Range: 1 2000
    // @User: Advanced
    AP_GROUPINFO("TIME_OUT", 3, AP_AIS, _time_out, 600),

    // @Param: LOGGING
    // @DisplayName: AIS logging options
    // @Description: Bitmask of AIS logging options
    // @Bitmask: 0:Log all AIVDM messages,1:Log only unsupported AIVDM messages,2:Log decoded messages
    // @User: Advanced
    AP_GROUPINFO("LOGGING", 4, AP_AIS, _log_options, AIS_OPTIONS_LOG_UNSUPPORTED_RAW | AIS_OPTIONS_LOG_DECODED),

    AP_GROUPEND
};

// constructor
AP_AIS::AP_AIS()
{
    if (_singleton != nullptr) {
        AP_HAL::panic("AIS must be singleton");
    }
    _singleton = this;

    AP_Param::setup_object_defaults(this, var_info);
}

// return true if AIS is enabled
bool AP_AIS::enabled() const
{ 
    return AISType(_type.get()) != AISType::NONE;
}

// Initialize the AIS object and prepare it for use
void AP_AIS::init()
{
    if (!enabled()) {
        return;
    }

    _uart = AP::serialmanager().find_serial(AP_SerialManager::SerialProtocol_AIS, 0);
    if (_uart == nullptr) {
        return;
    }

    _uart->begin(AP::serialmanager().find_baudrate(AP_SerialManager::SerialProtocol_AIS, 0));
}

// update AIS, expected to be called at 20hz
void AP_AIS::update()
{
    if (!_uart || !enabled()) {
        return;
    }

    // read any available lines
    uint32_t nbytes = MIN(_uart->available(),1024U);
    while (nbytes-- > 0) {
        const int16_t byte = _uart->read();
        if (byte == -1) {
            break;
        }
        const char c = byte;
        if (decode(c)) {
            const bool log_all = (_log_options & AIS_OPTIONS_LOG_ALL_RAW) != 0;
            const bool log_unsupported = ((_log_options & AIS_OPTIONS_LOG_UNSUPPORTED_RAW) != 0) && !log_all; // only log unsupported if not logging all

            if (_incoming.total  > AIVDM_BUFFER_SIZE)  {
                // no point in trying to decode it wont fit
                if (log_all || log_unsupported) {
                    log_raw(&_incoming);
                }
                break;
            }
            if (log_all) {
                log_raw(&_incoming);
            }

            if (_incoming.num == 1 && _incoming.total == 1) {
                // single part message
                if (!payload_decode(_incoming.payload) && log_unsupported) {
                    // could not decode so log
                    log_raw(&_incoming);
                }
            } else if (_incoming.num == _incoming.total) {
                // last part of a multi part message
                uint8_t index = 0;
                uint8_t msg_parts[_incoming.num - 1];
                for  (uint8_t i = 0; i < AIVDM_BUFFER_SIZE; i++) {
                    // look for the rest of the message from the start of the buffer
                    // we assume the mesage has be received in the correct order
                    if (_AIVDM_buffer[i].num == (index + 1) && _AIVDM_buffer[i].total == _incoming.total && _AIVDM_buffer[i].ID == _incoming.ID) {
                        msg_parts[index] = i;
                        index++;
                        if (index >= _incoming.num) {
                            break;
                        }
                    }
                }

                // did we find the right number?
                if (_incoming.num != index) {
                    // could not find all of the message, save messages
                    if (log_unsupported) {
                        for (uint8_t i = 0; i < index; i++) {
                            log_raw(&_AIVDM_buffer[msg_parts[i]]);
                        }
                        log_raw(&_incoming);
                    }
                    // remove
                    for (uint8_t i = 0; i < index; i++) {
                        buffer_shift(msg_parts[i]);
                    }
                    break;
                }

                // combine packets
                char multi[AIVDM_PAYLOAD_SIZE*_incoming.total];
                strncpy(multi,_AIVDM_buffer[msg_parts[0]].payload,AIVDM_PAYLOAD_SIZE);
                for (uint8_t i = 1; i < _incoming.total - 1; i++) {
                    strncat(multi,_AIVDM_buffer[msg_parts[i]].payload,sizeof(multi));
                }
                strncat(multi,_incoming.payload,sizeof(multi));
                const bool decoded = payload_decode(multi);
                for (uint8_t i = 0; i < _incoming.total; i++) {
                    // unsupported type, log and discard
                    if (!decoded && log_unsupported) {
                        log_raw(&_AIVDM_buffer[msg_parts[i]]);
                    }
                    buffer_shift(msg_parts[i]);
                }
                if (!decoded && log_unsupported) {
                    log_raw(&_incoming);
                }
            } else {
                // multi part message, store in buffer
                bool fits_in = false;
                for  (uint8_t i = 0; i < AIVDM_BUFFER_SIZE; i++) {
                    // find the first free spot
                    if (_AIVDM_buffer[i].num == 0 && _AIVDM_buffer[i].total == 0 && _AIVDM_buffer[i].ID == 0) {
                        _AIVDM_buffer[i] = _incoming;
                        fits_in = true;
                        break;
                    }
                }
                if (!fits_in) {
                    // remove the oldest message
                    if (log_unsupported) {
                        // log the unused message before removing it
                        log_raw(&_AIVDM_buffer[0]);
                    }
                    buffer_shift(0);
                    _AIVDM_buffer[AIVDM_BUFFER_SIZE - 1] = _incoming;
                }
            }
        }
    }

    // remove expired items from the list
    const uint32_t now =  AP_HAL::millis();
    const uint32_t timeout = _time_out * 1000;
    if (now < timeout) {
        return;
    }
    const uint32_t deadline = now - timeout;
    for (uint16_t i = 0; i < _list.max_items(); i++) {
        if (_list[i].last_update_ms < deadline && _list[i].last_update_ms != 0) {
            clear_list_item(i);
        }
    }
}

// Send a AIS mavlink message
void AP_AIS::send(mavlink_channel_t chan)
{
    if (!enabled()) {
        return;
    }

    const uint16_t list_size = _list.max_items();
    const uint32_t now =  AP_HAL::millis();
    uint16_t search_length = 0;
    while (search_length < list_size) {
        _send_index++;
        search_length++;
        if (_send_index == list_size) {
            _send_index = 0;
        }
        if (_list[_send_index].last_update_ms != 0 &&
            (_list[_send_index].last_send_ms < _list[_send_index].last_update_ms || now -_list[_send_index].last_send_ms > 30000)) {
                // only re-send if there has been a change or the resend time has expired
                _list[_send_index].last_send_ms = now;
                _list[_send_index].info.tslc = (now - _list[_send_index].last_update_ms) * 0.001;
                mavlink_msg_ais_vessel_send_struct(chan,&_list[_send_index].info);
                return;
        }
    }
}

// remove the given index from the AIVDM buffer and shift following elements up
void AP_AIS::buffer_shift(uint8_t i)
{
    for (uint8_t n = i;  n < (AIVDM_BUFFER_SIZE - 1); n++) {
        _AIVDM_buffer[n].ID = _AIVDM_buffer[n+1].ID;
        _AIVDM_buffer[n].num = _AIVDM_buffer[n+1].num;
        _AIVDM_buffer[n].total = _AIVDM_buffer[n+1].total;
        strncpy(_AIVDM_buffer[n].payload,_AIVDM_buffer[n+1].payload,AIVDM_PAYLOAD_SIZE);
    }
    _AIVDM_buffer[AIVDM_BUFFER_SIZE - 1].ID = 0;
    _AIVDM_buffer[AIVDM_BUFFER_SIZE - 1].num = 0;
    _AIVDM_buffer[AIVDM_BUFFER_SIZE - 1].total = 0;
    _AIVDM_buffer[AIVDM_BUFFER_SIZE - 1].payload[0] = 0;
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Functions related to the vessel list

// find vessel index in existing list, if not then return new index if possible
bool AP_AIS::get_vessel_index(uint32_t mmsi, uint16_t &index, uint32_t lat, uint32_t lon)
{
    const uint16_t list_size = _list.max_items();

    uint16_t empty = 0;
    bool found_empty = false;
    for (uint16_t i = 0; i < list_size; i++) {
        if (_list[i].info.MMSI == mmsi) {
            index = i;
            return true;
        }
        if (_list[i].last_update_ms == 0 && !found_empty) {
            found_empty = true;
            empty = i;
        }
    }

    // got through the list without a match
    if (found_empty) {
        index = empty;
        _list[index].info.MMSI = mmsi;
        return true;
    }

    // no space in the list
    if (list_size < _max_list) {
        // if we can try and expand
        if (_list.expand(1)) {
            index = list_size;
            _list[index].info.MMSI = mmsi;
            return true;
        }
    }

    // could not expand list, either because of memory or max list param
    // if we have a valid incoming location we can bump a further item from the list
    if (lat == 0 && lon == 0) {
        return false;
    }

    Location current_loc;
    if (!AP::ahrs().get_location(current_loc)) {
        return false;
    }

    Location loc;
    float dist;
    float max_dist = 0;
    for (uint16_t i = 0; i < list_size; i++) {
        loc.lat = _list[i].info.lat;
        loc.lng = _list[i].info.lon;
        dist = loc.get_distance(current_loc);
        if (dist > max_dist) {
            max_dist = dist;
            index = i;
        }
    }

    // find the current distance
    loc.lat = lat;
    loc.lng = lon;
    dist = loc.get_distance(current_loc);

    if (dist < max_dist) {
        clear_list_item(index);
        _list[index].info.MMSI = mmsi;
        return true;
    }

    return false;
}

void AP_AIS::clear_list_item(uint16_t index)
{
    if (index < _list.max_items()) {
        memset(&_list[index],0,sizeof(ais_vehicle_t));
    }
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Functions for decoding AIVDM payload mesages

bool AP_AIS::payload_decode(const char *payload)
{
    // the mesage type is defined by the first character
    const uint8_t type = payload_char_decode(payload[0]);

    switch (type) {
        case 1: // Position Report Class A
        case 2: // Position Report Class A (Assigned schedule)
        case 3: // Position Report Class A (Response to interrogation)
            return decode_position_report(payload, type);
        case 4: // Base Station Report
            return decode_base_station_report(payload);
        case 5: // Static and Voyage Related Data
            return decode_static_and_voyage_data(payload);

        default:
            return false;
    }
}

bool AP_AIS::decode_position_report(const char *payload, uint8_t type)
{
    if (strlen(payload) != 28) {
        return false;
    }

    uint8_t repeat     = get_bits(payload, 6, 7);
    uint32_t mmsi      = get_bits(payload, 8, 37);
    uint8_t nav        = get_bits(payload, 38, 41);
    int8_t rot  = get_bits_signed(payload, 42, 49);
    uint16_t sog       = get_bits(payload, 50, 59);
    bool pos_acc       = get_bits(payload, 60, 60);
    int32_t lon = get_bits_signed(payload, 61, 88)  * ((1.0f / 600000.0f)*1e7);
    int32_t lat = get_bits_signed(payload, 89, 115) * ((1.0f / 600000.0f)*1e7);
    uint16_t cog       = get_bits(payload, 116, 127) * 10;
    uint16_t head      = get_bits(payload, 128, 136) * 100;
    uint8_t sec_utc    = get_bits(payload, 137, 142);
    uint8_t maneuver   = get_bits(payload, 143, 144);
    // 145 - 147: spare
    bool raim = get_bits(payload, 148, 148);
    uint32_t radio = get_bits(payload, 149, 167);

    // log the raw infomation
    if ((_log_options & AIS_OPTIONS_LOG_DECODED) != 0) {
        const struct log_AIS_msg1 pkt{
            LOG_PACKET_HEADER_INIT(LOG_AIS_MSG1),
            time_us      : AP_HAL::micros64(),
            type         : type,
            repeat       : repeat,
            mmsi         : mmsi,
            nav          : nav,
            rot          : rot,
            sog          : sog,
            pos_acc      : pos_acc,
            lon          : lon,
            lat          : lat,
            cog          : cog,
            head         : head,
            sec_utc      : sec_utc,
            maneuver     : maneuver,
            raim         : raim,
            radio        : radio
        };
        AP::logger().WriteBlock(&pkt, sizeof(pkt));
    }

    uint16_t index;
    if (!get_vessel_index(mmsi, index, lat, lon)) {
        // no room in the vessel list
        return true;
    }

    // mask of flags that we receive in this message
    const uint16_t mask = ~(AIS_FLAGS_POSITION_ACCURACY | AIS_FLAGS_VALID_COG | AIS_FLAGS_VALID_VELOCITY | AIS_FLAGS_VALID_TURN_RATE | AIS_FLAGS_TURN_RATE_SIGN_ONLY);
    uint16_t flags = _list[index].info.flags & mask; // clear all flags that will be updated
    if (pos_acc) {
        flags |= AIS_FLAGS_POSITION_ACCURACY;
    }
    if (cog < 36000) {
        flags |= AIS_FLAGS_VALID_COG;
    }
    if (sog < 1023) {
        flags |= AIS_FLAGS_VALID_VELOCITY;
    }
    if (sog == 1022) {
        flags |= AIS_FLAGS_HIGH_VELOCITY;
    }
    if (rot > -128) {
        flags |= AIS_FLAGS_VALID_TURN_RATE;
    }
    if (rot == 127 || rot == -127) {
        flags |= AIS_FLAGS_TURN_RATE_SIGN_ONLY;
    } else {
        rot = powf((rot / 4.733f),2.0f) / 6.0f;
    }

    _list[index].info.lat = lat; // int32_t [degE7] Latitude
    _list[index].info.lon = lon; // int32_t [degE7] Longitude
    _list[index].info.COG = cog; // uint16_t [cdeg] Course over ground
    _list[index].info.heading = head; // uint16_t [cdeg] True heading
    _list[index].info.velocity = sog; // uint16_t [cm/s] Speed over ground
    _list[index].info.flags = flags; // uint16_t Bitmask to indicate various statuses including valid data fields
    _list[index].info.turn_rate = rot; // int8_t [cdeg/s] Turn rate
    _list[index].info.navigational_status = nav; // uint8_t Navigational status
    _list[index].last_update_ms = AP_HAL::millis();

    return true;
}

bool AP_AIS::decode_base_station_report(const char *payload)
{
    if (strlen(payload) != 28) {
        return false;
    }

    uint8_t repeat     = get_bits(payload, 6, 7);
    uint32_t mmsi      = get_bits(payload, 8, 37);
    uint16_t year      = get_bits(payload, 38, 51);
    uint8_t month      = get_bits(payload, 52, 55);
    uint8_t day        = get_bits(payload, 56, 60);
    uint8_t hour       = get_bits(payload, 61, 65);
    uint8_t minute     = get_bits(payload, 66, 71);
    uint8_t second     = get_bits(payload, 72, 77);
    bool fix           = get_bits(payload, 78, 78);
    int32_t lon = get_bits_signed(payload, 79, 106)  * ((1.0f / 600000.0f)*1e7);
    int32_t lat = get_bits_signed(payload, 107, 133) * ((1.0f / 600000.0f)*1e7);
    uint8_t epfd       = get_bits(payload, 134, 137);
    // 138 - 147: spare
    bool raim          = get_bits(payload, 148, 148);
    uint32_t radio     = get_bits(payload, 149, 167);

    // log the raw infomation
    if ((_log_options & AIS_OPTIONS_LOG_DECODED) != 0) {
        struct log_AIS_msg4 pkt {
            LOG_PACKET_HEADER_INIT(LOG_AIS_MSG4),
            time_us     : AP_HAL::micros64(),
            repeat      : repeat,
            mmsi        : mmsi,
            year        : year,
            month       : month,
            day         : day,
            hour        : hour,
            minute      : minute,
            second      : second,
            fix         : fix,
            lon         : lon,
            lat         : lat,
            epfd        : epfd,
            raim        : raim,
            radio       : radio
        };
        AP::logger().WriteBlock(&pkt, sizeof(pkt));
    }

    uint16_t index;
    if (!get_vessel_index(mmsi, index)) {
        return true;
    }

    _list[index].info.lat = lat; // int32_t [degE7] Latitude
    _list[index].info.lon = lon; // int32_t [degE7] Longitude
    _list[index].last_update_ms = AP_HAL::millis();

    return true;
}

bool AP_AIS::decode_static_and_voyage_data(const char *payload)
{
    if (strlen(payload) != 71) {
        return false;
    }

    char call_sign[8];
    char name[21];
    char dest[21];

    uint8_t repeat      = get_bits(payload, 6, 7);
    uint32_t mmsi       = get_bits(payload, 8, 37);
    uint8_t ver         = get_bits(payload, 38, 39);
    uint32_t imo        = get_bits(payload, 40, 69);
               get_char(payload, call_sign, 70, 111);
                    get_char(payload, name, 112, 231);
    uint8_t vessel_type = get_bits(payload, 232, 239);
    uint16_t bow_dim    = get_bits(payload, 240, 248);
    uint16_t stern_dim  = get_bits(payload, 249, 257);
    uint8_t port_dim    = get_bits(payload, 258, 263);
    uint8_t star_dim    = get_bits(payload, 264, 269);
    uint8_t fix         = get_bits(payload, 270, 273);
    //uint8_t month     = get_bits(payload, 274, 277); // too much for a single log
    //uint8_t day       = get_bits(payload, 278, 282);
    //uint8_t hour      = get_bits(payload, 283, 287);
    //uint8_t minute    = get_bits(payload, 288, 293);
    uint8_t draught     = get_bits(payload, 294, 301);
                    get_char(payload, dest, 302, 421);
    bool dte            = get_bits(payload, 422, 422);
    // 423 - 426: spare

    // log the raw infomation
    if ((_log_options & AIS_OPTIONS_LOG_DECODED) != 0) {
        struct log_AIS_msg5 pkt {
            LOG_PACKET_HEADER_INIT(LOG_AIS_MSG5),
            time_us     : AP_HAL::micros64(),
            repeat      : repeat,
            mmsi        : mmsi,
            ver         : ver,
            imo         : imo,
            call_sign   : {},
            name        : {},
            vessel_type : vessel_type,
            bow_dim     : bow_dim,
            stern_dim   : stern_dim,
            port_dim    : port_dim,
            star_dim    : star_dim,
            fix         : fix,
            draught     : draught,
            dest        : {},
            dte         : dte
        };
        strncpy(pkt.call_sign, call_sign, sizeof(pkt.call_sign));
        strncpy(pkt.name, name, sizeof(pkt.name));
        strncpy(pkt.dest, dest, sizeof(pkt.dest));
        AP::logger().WriteBlock(&pkt, sizeof(pkt));
    }

    uint16_t index;
    if (!get_vessel_index(mmsi, index)) {
        return true;
    }

    // mask of flags that we receive in this message
    const uint16_t mask = ~(AIS_FLAGS_VALID_DIMENSIONS | AIS_FLAGS_LARGE_BOW_DIMENSION | AIS_FLAGS_LARGE_STERN_DIMENSION | AIS_FLAGS_LARGE_STARBOARD_DIMENSION | AIS_FLAGS_VALID_CALLSIGN | AIS_FLAGS_VALID_NAME);
    uint16_t flags = _list[index].info.flags & mask; // clear all flags that will be updated
    if (bow_dim != 0 && stern_dim != 0 && port_dim != 0 && star_dim != 0) {
        flags |= AIS_FLAGS_VALID_DIMENSIONS;
        if (bow_dim == 511) {
            flags |= AIS_FLAGS_LARGE_BOW_DIMENSION;
        }
        if (stern_dim == 511) {
            flags |= AIS_FLAGS_LARGE_STERN_DIMENSION;
        }
        if (port_dim == 63) {
            flags |= AIS_FLAGS_LARGE_PORT_DIMENSION;
        }
        if (star_dim == 63) {
            flags |= AIS_FLAGS_LARGE_STARBOARD_DIMENSION;
        }
    }
    if (strlen(call_sign) != 0) {
        flags |= AIS_FLAGS_VALID_CALLSIGN;
    }
    if (strlen(name) != 0) {
        flags |= AIS_FLAGS_VALID_NAME;
    }

    _list[index].info.dimension_bow = bow_dim; // uint16_t [m] Distance from lat/lon location to bow
    _list[index].info.dimension_stern = stern_dim; // uint16_t [m] Distance from lat/lon location to stern
    _list[index].info.flags = flags; // uint16_t Bitmask to indicate various statuses including valid data fields
    _list[index].info.type = vessel_type; // uint8_t Type of vessels
    _list[index].info.dimension_port = port_dim; // uint8_t [m] Distance from lat/lon location to port side
    _list[index].info.dimension_starboard = star_dim; // uint8_t [m] Distance from lat/lon location to starboard side
    memcpy(_list[index].info.callsign,call_sign,sizeof(_list[index].info.callsign)); // char The vessel callsign
    memcpy(_list[index].info.name,name,sizeof(_list[index].info.name)); // char The vessel name

    // note that the last contact time is not updated, this message does not provide a location for a valid vessel a location must be received

    return true;
}


//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Functions for decoding AIVDM payload bits

// decode bits to a char array
void AP_AIS::get_char(const char *payload, char *array, uint16_t low, uint16_t high)
{
    bool found_char = false;
    uint8_t length = ((high - low) + 1)/6;
    for (uint8_t i = length; i > 0; i--) {
        uint8_t ascii = get_bits(payload, low + (i-1)*6, (low + (i*6)) - 1);
        if (ascii < 32) {
            ascii += 64;
        }
        if (ascii == 64 || (ascii == 32 && !found_char)) { // '@' marks end of string, remove trailing spaces
            array[i-1] = 0;
        } else {
            found_char = true;
            array[i-1] = ascii;
        }
    }
    array[length] = 0; // always null terminate
}

// read the specified bits from the char array each char giving 6 bits
uint32_t AP_AIS::get_bits(const char *payload, uint16_t low, uint16_t high)
{
    uint8_t char_low = low / 6;
    uint8_t bit_low = low % 6;

    uint8_t char_high = high / 6;
    uint8_t bit_high = (high % 6) + 1;

    uint32_t val = 0;
    for (uint8_t index = 0; index <= char_high - char_low; index++) {
        uint8_t value = payload_char_decode(payload[char_low + index]);
        uint8_t mask = 0b111111;
        if (index == 0) {
            mask = mask >> bit_low;
        }
        value &= mask;
        if (index == char_high - char_low) {
            value = value >> (6 - bit_high);
            val = val << bit_high;
        } else {
            val = val << 6;
        }

        val |= value;
    }

    return val;
}

// read the specified bits from the char array each char giving 6 bits
// As the values are a arbitrary length the sign bit is in the wrong place for standard length varables
int32_t AP_AIS::get_bits_signed(const char *payload, uint16_t low, uint16_t high)
{
    uint32_t value = get_bits(payload, low, high);
    if (get_bits(payload, low, low)) { // check sign bit
        // negative number
        return value | (UINT32_MAX << (high - low));
    }
    return value;
}

// Convert payload chars to bits
uint8_t AP_AIS::payload_char_decode(const char c)
{
    uint8_t value = c;
    value -= 48;
    if (value > 40) {
        value -= 8;
    }
    return value;
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Functions for decoding and logging AIVDM NMEA sentence

// log a raw AIVDM a message
void AP_AIS::log_raw(const AIVDM *msg)
{
    struct log_AIS_raw pkt{
        LOG_PACKET_HEADER_INIT(LOG_AIS_RAW_MSG),
        time_us      : AP_HAL::micros64(),
        num          : msg->num,
        total        : msg->total,
        ID           : msg->ID,
        payload      : {}
    };
    memcpy(pkt.payload, msg->payload, sizeof(pkt.payload));
    AP::logger().WriteBlock(&pkt, sizeof(pkt));
}

// add a single character to the buffer and attempt to decode
// returns true if a complete sentence was successfully decoded
bool AP_AIS::decode(char c)
{
    switch (c) {
    case ',':
        // end of a term, add to checksum
        _checksum ^= c;
        FALLTHROUGH;
    case '\r':
    case '\n':
    case '*':
    {
        if (_sentence_done) {
            return false;
        }

        // null terminate and decode latest term
        _term[_term_offset] = 0;
        bool valid_sentence = decode_latest_term();

        // move onto next term
        _term_number++;
        _term_offset = 0;
        _term_is_checksum = (c == '*');
        return valid_sentence;
    }

    case '!': // sentence begin
        _sentence_valid = false;
        _term_number = 0;
        _term_offset = 0;
        _checksum = 0;
        _term_is_checksum = false;
        _sentence_done = false;
        return false;
    }

    // ordinary characters are added to term
    if (_term_offset < sizeof(_term) - 1) {
        _term[_term_offset++] = c;
    }
    if (!_term_is_checksum) {
        _checksum ^= c;
    }

    return false;
}

// decode the most recently consumed term
// returns true if new sentence has just passed checksum test and is validated
bool AP_AIS::decode_latest_term()
{
    // handle the last term in a message
    if (_term_is_checksum) {
        _sentence_done = true;
        uint8_t checksum = 16 * char_to_hex(_term[0]) + char_to_hex(_term[1]);
        return ((checksum == _checksum) && _sentence_valid);
    }

    // the first term determines the sentence type
    if (_term_number == 0) {
        if (strcmp(_term, "AIVDM") == 0) {
            // we found the sentence type for AIS
            _sentence_valid = true;
        }
        return false;
    }

    // if this is not the sentence we want then wait for another
    if (!_sentence_valid) {
        return false;
    }

    switch (_term_number) {
        case 1:
            _incoming.total = strtol(_term, NULL, 10);
            break;

        case 2:
            _incoming.num = strtol(_term, NULL, 10);
            break;

        case 3:
            _incoming.ID = 0;
            if (strlen(_term) > 0) {
                _incoming.ID = strtol(_term, NULL, 10);
             } else if (_incoming.num != 1 || _incoming.total != 1) {
                // only allow no ID if this is a single part message
                _sentence_valid = false;
            }
            break;

        // case 4, chanel, either A or B, discarded

        case 5:
            if (strlen(_term) == 0) {
                _sentence_valid = false;
            } else {
                strcpy(_incoming.payload,_term);
            }
            break;

        //case 5, number of fill bits, discarded
    }
    return false;
}

// get singleton instance
AP_AIS *AP_AIS::get_singleton() {
    return _singleton;
}

#else
// Dummy methods are required to allow functionality to be enabled for Rover.
// It is not posible to compile in or out the full code based on vehicle type due to limitations
// of the handling of `APM_BUILD_TYPE` define.
// These dummy methods minimise flash cost in that case.

const AP_Param::GroupInfo AP_AIS::var_info[] = { AP_GROUPEND };
AP_AIS::AP_AIS() {};

bool AP_AIS::enabled() const { return false; }

void AP_AIS::init() {};
void AP_AIS::update() {};
void AP_AIS::send(mavlink_channel_t chan) {};

AP_AIS *AP_AIS::get_singleton() { return nullptr; }

#endif // AP_AIS_DUMMY_METHODS_ENABLED

AP_AIS *AP_AIS::_singleton;

#endif  // AP_AIS_ENABLED
