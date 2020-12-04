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

//
//  Swift Navigation SBP GPS driver for ArduPilot.
//	Code by Niels Joubert
//
//  Swift Binary Protocol format: http://docs.swift-nav.com/
//

#include "AP_GPS.h"
#include "AP_GPS_SBP.h"
#include <AP_Logger/AP_Logger.h>

extern const AP_HAL::HAL& hal;

#define SBP_DEBUGGING 1
#define SBP_HW_LOGGING 1

#define SBP_TIMEOUT_HEATBEAT  4000
#define SBP_TIMEOUT_PVT       500

#if SBP_DEBUGGING
 # define Debug(fmt, args ...)                  \
do {                                            \
    hal.console->printf("%s:%d: " fmt "\n",     \
                        __FUNCTION__, __LINE__, \
                        ## args);               \
    hal.scheduler->delay(1);                    \
} while(0)
#else
 # define Debug(fmt, args ...)
#endif

AP_GPS_SBP::AP_GPS_SBP(AP_GPS &_gps, AP_GPS::GPS_State &_state,
                       AP_HAL::UARTDriver *_port) :
    AP_GPS_Backend(_gps, _state, _port)
{

    Debug("SBP Driver Initialized");

    parser_state.state = sbp_parser_state_t::WAITING;

    //Externally visible state
    state.status = AP_GPS::NO_FIX;
    state.last_gps_time_ms = last_heatbeat_received_ms = AP_HAL::millis();

}

// Process all bytes available from the stream
//
bool
AP_GPS_SBP::read(void)
{

    //Invariant: Calling this function processes *all* data current in the UART buffer.
    //
    //IMPORTANT NOTICE: This function is NOT CALLED for several seconds
    // during arming. That should not cause the driver to die. Process *all* waiting messages

    _sbp_process();

    return _attempt_state_update();

}

void
AP_GPS_SBP::inject_data(const uint8_t *data, uint16_t len)
{

    if (port->txspace() > len) {
        last_injected_data_ms = AP_HAL::millis();
        port->write(data, len);
    } else {
        Debug("PIKSI: Not enough TXSPACE");
    }

}

//This attempts to reads all SBP messages from the incoming port.
//Returns true if a new message was read, false if we failed to read a message.
void
AP_GPS_SBP::_sbp_process()
{

    while (port->available() > 0) {
        uint8_t temp = port->read();
        uint16_t crc;


        //This switch reads one character at a time,
        //parsing it into buffers until a full message is dispatched
        switch (parser_state.state) {
            case sbp_parser_state_t::WAITING:
                if (temp == SBP_PREAMBLE) {
                    parser_state.n_read = 0;
                    parser_state.state = sbp_parser_state_t::GET_TYPE;
                }
                break;

            case sbp_parser_state_t::GET_TYPE:
                *((uint8_t*)&(parser_state.msg_type) + parser_state.n_read) = temp;
                parser_state.n_read += 1;
                if (parser_state.n_read >= 2) {
                    parser_state.n_read = 0;
                    parser_state.state = sbp_parser_state_t::GET_SENDER;
                }
                break;

            case sbp_parser_state_t::GET_SENDER:
                *((uint8_t*)&(parser_state.sender_id) + parser_state.n_read) = temp;
                parser_state.n_read += 1;
                if (parser_state.n_read >= 2) {
                    parser_state.n_read = 0;
                    parser_state.state = sbp_parser_state_t::GET_LEN;
                }
                break;

            case sbp_parser_state_t::GET_LEN:
                parser_state.msg_len = temp;
                parser_state.n_read = 0;
                parser_state.state = sbp_parser_state_t::GET_MSG;
                break;

            case sbp_parser_state_t::GET_MSG:
                *((uint8_t*)&(parser_state.msg_buff) + parser_state.n_read) = temp;
                parser_state.n_read += 1;
                if (parser_state.n_read >= parser_state.msg_len) {
                    parser_state.n_read = 0;
                    parser_state.state = sbp_parser_state_t::GET_CRC;
                }
                break;

            case sbp_parser_state_t::GET_CRC:
                *((uint8_t*)&(parser_state.crc) + parser_state.n_read) = temp;
                parser_state.n_read += 1;
                if (parser_state.n_read >= 2) {
                    parser_state.state = sbp_parser_state_t::WAITING;

                    crc = crc16_ccitt((uint8_t*)&(parser_state.msg_type), 2, 0);
                    crc = crc16_ccitt((uint8_t*)&(parser_state.sender_id), 2, crc);
                    crc = crc16_ccitt(&(parser_state.msg_len), 1, crc);
                    crc = crc16_ccitt(parser_state.msg_buff, parser_state.msg_len, crc);
                    if (parser_state.crc == crc) {
                        _sbp_process_message();
                    } else {
                        Debug("CRC Error Occurred!");
                        crc_error_counter += 1;
                    }

                    parser_state.state = sbp_parser_state_t::WAITING;
                }
                break;

            default:
                parser_state.state = sbp_parser_state_t::WAITING;
                break;
            }
    }
}


//INVARIANT: A fully received message with correct CRC is currently in parser_state
void
AP_GPS_SBP::_sbp_process_message() {
    switch (parser_state.msg_type) {
        case SBP_HEARTBEAT_MSGTYPE:
            last_heatbeat_received_ms = AP_HAL::millis();
            break;

        case SBP_GPS_TIME_MSGTYPE:
            memcpy(&last_gps_time, parser_state.msg_buff, sizeof(last_gps_time));
            check_new_itow(last_gps_time.tow, parser_state.msg_len);
            break;

        case SBP_VEL_NED_MSGTYPE:
            memcpy(&last_vel_ned, parser_state.msg_buff, sizeof(last_vel_ned));
            check_new_itow(last_vel_ned.tow, parser_state.msg_len);
            break;

        case SBP_POS_LLH_MSGTYPE: {
            struct sbp_pos_llh_t *pos_llh = (struct sbp_pos_llh_t*)parser_state.msg_buff;
            check_new_itow(pos_llh->tow, parser_state.msg_len);
            // Check if this is a single point or RTK solution
            // flags = 0 -> single point
            if (pos_llh->flags == 0) {
                last_pos_llh_spp = *pos_llh;
            } else if (pos_llh->flags == 1 || pos_llh->flags == 2) {
                last_pos_llh_rtk = *pos_llh;
            }
            break;
        }

        case SBP_DOPS_MSGTYPE:
            memcpy(&last_dops, parser_state.msg_buff, sizeof(last_dops));
            check_new_itow(last_dops.tow, parser_state.msg_len);
            break;

        case SBP_TRACKING_STATE_MSGTYPE:
            //INTENTIONALLY BLANK
            //Currently unhandled, but logged after switch statement.
            break;

        case SBP_IAR_STATE_MSGTYPE: {
            sbp_iar_state_t *iar = (struct sbp_iar_state_t*)parser_state.msg_buff;
            last_iar_num_hypotheses = iar->num_hypotheses;
            break;
        }

        default:
            // log anyway if it's an unsupported message.
            // The log mask will be used to adjust or suppress logging
            break;
    }

    logging_log_raw_sbp(parser_state.msg_type, parser_state.sender_id, parser_state.msg_len, parser_state.msg_buff);
}

bool
AP_GPS_SBP::_attempt_state_update()
{

    // If we currently have heartbeats
    //    - NO FIX
    //
    // If we have a full update available, save it
    //
    uint32_t now = AP_HAL::millis();
    bool ret = false;

    if (now - last_heatbeat_received_ms > SBP_TIMEOUT_HEATBEAT) {

        state.status = AP_GPS::NO_FIX;
        Debug("No Heartbeats from Piksi! Driver Ready to Die!");

    } else if (last_pos_llh_rtk.tow == last_vel_ned.tow
            && abs((int32_t) (last_gps_time.tow - last_vel_ned.tow)) < 10000
            && abs((int32_t) (last_dops.tow - last_vel_ned.tow)) < 60000
            && last_vel_ned.tow > last_full_update_tow) {

        // Use the RTK position
        sbp_pos_llh_t *pos_llh = &last_pos_llh_rtk;

        // Update time state
        state.time_week         = last_gps_time.wn;
        state.time_week_ms      = last_vel_ned.tow;

        state.hdop              = last_dops.hdop;

        // Update velocity state
        state.velocity[0]       = (float)(last_vel_ned.n * 1.0e-3);
        state.velocity[1]       = (float)(last_vel_ned.e * 1.0e-3);
        state.velocity[2]       = (float)(last_vel_ned.d * 1.0e-3);
        state.have_vertical_velocity = true;

        float ground_vector_sq = state.velocity[0]*state.velocity[0] + state.velocity[1]*state.velocity[1];
        state.ground_speed = safe_sqrt(ground_vector_sq);

        state.ground_course = wrap_360(degrees(atan2f(state.velocity[1], state.velocity[0])));

        // Update position state

        state.location.lat      = (int32_t) (pos_llh->lat * (double)1e7);
        state.location.lng      = (int32_t) (pos_llh->lon * (double)1e7);
        state.location.alt      = (int32_t) (pos_llh->height * 100);
        state.num_sats          = pos_llh->n_sats;

        if (pos_llh->flags == 0) {
            state.status = AP_GPS::GPS_OK_FIX_3D;
        } else if (pos_llh->flags == 2) {
            state.status = AP_GPS::GPS_OK_FIX_3D_RTK_FLOAT;
        } else if (pos_llh->flags == 1) {
            state.status = AP_GPS::GPS_OK_FIX_3D_RTK_FIXED;
        }

        last_full_update_tow = last_vel_ned.tow;
        last_full_update_cpu_ms = now;
        state.rtk_iar_num_hypotheses = last_iar_num_hypotheses;

        logging_log_full_update();
        ret = true;

    } else if (now - last_full_update_cpu_ms > SBP_TIMEOUT_PVT) {

        //INVARIANT: If we currently have a fix, ONLY return true after a full update.

        state.status = AP_GPS::NO_FIX;
        ret = true;

    } else {

        //No timeouts yet, no data yet, nothing has happened.

    }

    return ret;

}



bool
AP_GPS_SBP::_detect(struct SBP_detect_state &state, uint8_t data)
{
    // This switch reads one character at a time, if we find something that
    // looks like our preamble we'll try to read the full message length,
    // calculating the CRC. If the CRC matches, we have an SBP GPS!

    switch (state.state) {
        case SBP_detect_state::WAITING:
            if (data == SBP_PREAMBLE) {
                state.n_read = 0;
                state.crc_so_far = 0;
                state.state = SBP_detect_state::GET_TYPE;
            }
            break;

        case SBP_detect_state::GET_TYPE:
            *((uint8_t*)&(state.msg_type) + state.n_read) = data;
            state.crc_so_far = crc16_ccitt(&data, 1, state.crc_so_far);
            state.n_read += 1;
            if (state.n_read >= 2) {
                state.n_read = 0;
                state.state = SBP_detect_state::GET_SENDER;
            }
            break;

        case SBP_detect_state::GET_SENDER:
            state.crc_so_far = crc16_ccitt(&data, 1, state.crc_so_far);
            state.n_read += 1;
            if (state.n_read >= 2) {
                state.n_read = 0;
                state.state = SBP_detect_state::GET_LEN;
            }
            break;

        case SBP_detect_state::GET_LEN:
            state.crc_so_far = crc16_ccitt(&data, 1, state.crc_so_far);
            state.msg_len = data;
            state.n_read = 0;
            state.state = SBP_detect_state::GET_MSG;
            break;

        case SBP_detect_state::GET_MSG:
            if (state.msg_type == SBP_HEARTBEAT_MSGTYPE && state.n_read < 4) {
                *((uint8_t*)&(state.heartbeat_buff) + state.n_read) = data;
            }
            state.crc_so_far = crc16_ccitt(&data, 1, state.crc_so_far);
            state.n_read += 1;
            if (state.n_read >= state.msg_len) {
                state.n_read = 0;
                state.state = SBP_detect_state::GET_CRC;
            }
            break;

        case SBP_detect_state::GET_CRC:
            *((uint8_t*)&(state.crc) + state.n_read) = data;
            state.n_read += 1;
            if (state.n_read >= 2) {
                state.state = SBP_detect_state::WAITING;
                if (state.crc == state.crc_so_far
                        && state.msg_type == SBP_HEARTBEAT_MSGTYPE) {
                    struct sbp_heartbeat_t* heartbeat = ((struct sbp_heartbeat_t*)state.heartbeat_buff);
                    return heartbeat->protocol_major == 0;
                }
                return false;
            }
            break;

        default:
            state.state = SBP_detect_state::WAITING;
            break;
    }
    return false;
}

#if SBP_HW_LOGGING

void
AP_GPS_SBP::logging_log_full_update()
{

    if (!should_log()) {
        return;
    }

    struct log_SbpHealth pkt = {
        LOG_PACKET_HEADER_INIT(LOG_MSG_SBPHEALTH),
        time_us                    : AP_HAL::micros64(),
        crc_error_counter          : crc_error_counter,
        last_injected_data_ms      : last_injected_data_ms,
        last_iar_num_hypotheses    : last_iar_num_hypotheses,
    };

    AP::logger().WriteBlock(&pkt, sizeof(pkt));
};

void
AP_GPS_SBP::logging_log_raw_sbp(uint16_t msg_type,
        uint16_t sender_id,
        uint8_t msg_len,
        uint8_t *msg_buff) {
    if (!should_log()) {
        return;
    }

    //MASK OUT MESSAGES WE DON'T WANT TO LOG
    if (( ((uint16_t) gps._sbp_logmask) & msg_type) == 0) {
        return;
    }

    uint64_t time_us = AP_HAL::micros64();
    uint8_t pages = 1;

    if (msg_len > 48) {
        pages += (msg_len - 48) / 104 + 1;
    }

    struct log_SbpRAWH pkt = {
        LOG_PACKET_HEADER_INIT(LOG_MSG_SBPRAWH),
        time_us         : time_us,
        msg_type        : msg_type,
        sender_id       : sender_id,
        index           : 1,
        pages           : pages,
        msg_len         : msg_len,
    };
    memcpy(pkt.data, msg_buff, MIN(msg_len, 48));
    AP::logger().WriteBlock(&pkt, sizeof(pkt));

    for (uint8_t i = 0; i < pages - 1; i++) {
        struct log_SbpRAWM pkt2 = {
            LOG_PACKET_HEADER_INIT(LOG_MSG_SBPRAWM),
            time_us         : time_us,
            msg_type        : msg_type,
            sender_id       : sender_id,
            index           : uint8_t(i + 2),
            pages           : pages,
            msg_len         : msg_len,
        };
        memcpy(pkt2.data, &msg_buff[48 + i * 104], MIN(msg_len - (48 + i * 104), 104));
        AP::logger().WriteBlock(&pkt2, sizeof(pkt2));
    }
};

#endif // SBP_HW_LOGGING
