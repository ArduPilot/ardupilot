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
#include "AP_GPS_SBP2.h"
#include <AP_Logger/AP_Logger.h>
#include <GCS_MAVLink/GCS_MAVLink.h>
#include <GCS_MAVLink/GCS.h>

extern const AP_HAL::HAL& hal;

#define SBP_DEBUGGING 0
#define SBP_INFOREPORTING 1

//INVARIANT: We expect SBP to give us a heartbeat in less than 2 seconds.
//  This is more lax than the default Piksi settings,
//  and we assume the user hasn't reconfigured their Piksi to longer heartbeat intervals
#define SBP_TIMEOUT_HEARTBEAT  2000

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

#if SBP_INFOREPORTING
 # define Info(fmt, args ...)                                               \
do {                                                                        \
    gcs().send_text(MAV_SEVERITY_INFO, fmt "\n", ## args); \
} while(0) 
#else
 # define Info(fmt, args ...)
#endif


AP_GPS_SBP2::AP_GPS_SBP2(AP_GPS &_gps, AP_GPS::GPS_State &_state,
                       AP_HAL::UARTDriver *_port) :
    AP_GPS_Backend(_gps, _state, _port)
{
    Debug("SBP Driver Initialized");
    parser_state.state = sbp_parser_state_t::WAITING;
}

// Process all bytes available from the stream
//
bool
AP_GPS_SBP2::read(void)
{
    //Invariant: Calling this function processes *all* data current in the UART buffer.
    //
    //IMPORTANT NOTICE: This function is NOT CALLED for several seconds
    // during arming. That should not cause the driver to die. Process *all* waiting messages

    _sbp_process();
    return _attempt_state_update();
}

void
AP_GPS_SBP2::inject_data(const uint8_t *data, uint16_t len)
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
AP_GPS_SBP2::_sbp_process()
{
    uint32_t nleft = port->available(); 
    while (nleft > 0) {
        nleft--;
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
AP_GPS_SBP2::_sbp_process_message() {
    //Here, we copy messages into local structs.
    switch (parser_state.msg_type) {
        case SBP_HEARTBEAT_MSGTYPE:
            memcpy(&last_heartbeat, parser_state.msg_buff, sizeof(struct sbp_heartbeat_t));
            last_heartbeat_received_ms = AP_HAL::millis();
            break;

        case SBP_GPS_TIME_MSGTYPE:
            memcpy(&last_gps_time, parser_state.msg_buff, sizeof(struct sbp_gps_time_t));
            check_new_itow(last_gps_time.tow, parser_state.msg_len);
            break;

        case SBP_VEL_NED_MSGTYPE:
            memcpy(&last_vel_ned, parser_state.msg_buff, sizeof(struct sbp_vel_ned_t));
            check_new_itow(last_vel_ned.tow, parser_state.msg_len);
            break;

        case SBP_POS_LLH_MSGTYPE:
            memcpy(&last_pos_llh, parser_state.msg_buff, sizeof(struct sbp_pos_llh_t));
            check_new_itow(last_pos_llh.tow, parser_state.msg_len);
            break;

        case SBP_DOPS_MSGTYPE:
            memcpy(&last_dops, parser_state.msg_buff, sizeof(struct sbp_dops_t));
            check_new_itow(last_dops.tow, parser_state.msg_len);
            break;

        case SBP_EXT_EVENT_MSGTYPE:
            memcpy(&last_event, parser_state.msg_buff, sizeof(struct sbp_ext_event_t));
            check_new_itow(last_event.tow, parser_state.msg_len);
            logging_ext_event();
            break;

        default:
            break;
    }

    // send all messages we receive to log, even if it's an unsupported message,
    // so we can do additional post-processing from Dataflash logs.
    // The log mask will be used to adjust or suppress logging
    logging_log_raw_sbp(parser_state.msg_type, parser_state.sender_id, parser_state.msg_len, parser_state.msg_buff);
}

int32_t 
AP_GPS_SBP2::distMod(int32_t tow1_ms, int32_t tow2_ms, int32_t mod) {
    return MIN(abs(tow1_ms - tow2_ms), mod - abs(tow1_ms - tow2_ms));
}

bool
AP_GPS_SBP2::_attempt_state_update()
{
    if (last_heartbeat_received_ms == 0)
        return false;

    uint32_t now = AP_HAL::millis();

    if (now - last_heartbeat_received_ms > SBP_TIMEOUT_HEARTBEAT) {

        state.status = AP_GPS::NO_FIX;
        Info("No Heartbeats from Piksi! Status to NO_FIX.");
        return false;

    } else if (last_heartbeat.protocol_major != 2) {

        state.status = AP_GPS::NO_FIX;
        Info("Received a heartbeat from non-SBPv2 device. Current driver only supports SBPv2. Status to NO_FIX.");
        return false;

    } else if (last_heartbeat.nap_error   == 1 ||
               last_heartbeat.io_error    == 1 ||
               last_heartbeat.sys_error   == 1) {

        state.status = AP_GPS::NO_FIX;

        Info("Piksi reported an error. Status to NO_FIX.");
        Debug("   ext_antenna: %d", last_heartbeat.ext_antenna);
        Debug("   res2: %d", last_heartbeat.res2);
        Debug("   protocol_major: %d", last_heartbeat.protocol_major);
        Debug("   protocol_minor: %d", last_heartbeat.protocol_minor);
        Debug("   res: %d", last_heartbeat.res);
        Debug("   nap_error: %d", last_heartbeat.nap_error);
        Debug("   io_error: %d", last_heartbeat.io_error);
        Debug("   sys_error: %d", last_heartbeat.sys_error);

        return false;

    } else if (last_pos_llh.tow == last_vel_ned.tow
            && (distMod(last_gps_time.tow, last_vel_ned.tow, AP_MSEC_PER_WEEK) < 10000)
            && (distMod(last_dops.tow, last_vel_ned.tow, AP_MSEC_PER_WEEK) < 60000)
            && (last_vel_ned.tow > last_full_update_tow || (last_gps_time.wn > last_full_update_wn && last_vel_ned.tow < last_full_update_tow))) {

        //We have an aligned VEL and LLH, and a recent DOPS and TIME.

        //
        // Check Flags for Valid Messages
        //
        if (last_gps_time.flags.time_src    == 0 ||
            last_vel_ned.flags.vel_mode     == 0 ||
            last_pos_llh.flags.fix_mode     == 0 ||
            last_dops.flags.fix_mode        == 0) {

            Debug("Message Marked as Invalid. NO FIX! Flags: {GPS_TIME: %d, VEL_NED: %d, POS_LLH: %d, DOPS: %d}",
                   last_gps_time.flags.time_src,
                   last_vel_ned.flags.vel_mode,
                   last_pos_llh.flags.fix_mode,
                   last_dops.flags.fix_mode);

            state.status = AP_GPS::NO_FIX;
            return false;
        }

        //
        // Update external time and accuracy state
        //
        state.time_week         = last_gps_time.wn;
        state.time_week_ms      = last_vel_ned.tow;
        state.hdop              = last_dops.hdop;
        state.vdop              = last_dops.vdop;
        state.last_gps_time_ms = now;

        //
        // Update velocity state
        //
        state.velocity[0]       = (float)(last_vel_ned.n * 1.0e-3);
        state.velocity[1]       = (float)(last_vel_ned.e * 1.0e-3);
        state.velocity[2]       = (float)(last_vel_ned.d * 1.0e-3);

        float ground_vector_sq = state.velocity[0]*state.velocity[0] + state.velocity[1]*state.velocity[1];
        state.ground_speed = safe_sqrt(ground_vector_sq);

        state.ground_course = wrap_360(degrees(atan2f(state.velocity[1], state.velocity[0])));

        state.speed_accuracy        = safe_sqrt(
                                        powf((float)last_vel_ned.h_accuracy * 1.0e-3f, 2) + 
                                        powf((float)last_vel_ned.v_accuracy * 1.0e-3f, 2));
        state.horizontal_accuracy   = (float) last_pos_llh.h_accuracy * 1.0e-3f;
        state.vertical_accuracy     = (float) last_pos_llh.v_accuracy * 1.0e-3f;

        //
        // Set flags appropriately
        //
        state.have_vertical_velocity   = true;
        state.have_speed_accuracy      = !is_zero(state.speed_accuracy);
        state.have_horizontal_accuracy = !is_zero(state.horizontal_accuracy);
        state.have_vertical_accuracy   = !is_zero(state.vertical_accuracy);

        //
        // Update position state
        //
        state.location.lat      = (int32_t) (last_pos_llh.lat * (double)1e7);
        state.location.lng      = (int32_t) (last_pos_llh.lon * (double)1e7);
        state.location.alt      = (int32_t) (last_pos_llh.height * 100);
        state.num_sats          = last_pos_llh.n_sats;

        switch (last_pos_llh.flags.fix_mode) {
            case 1:
                state.status = AP_GPS::GPS_OK_FIX_3D;
                break;
            case 2:
                state.status = AP_GPS::GPS_OK_FIX_3D_DGPS;
                break;
            case 3:
                state.status = AP_GPS::GPS_OK_FIX_3D_RTK_FLOAT;
                break;
            case 4:
                state.status = AP_GPS::GPS_OK_FIX_3D_RTK_FIXED;
                break;
            case 6:
                state.status = AP_GPS::GPS_OK_FIX_3D_DGPS;
                break;
            default:
                state.status = AP_GPS::NO_FIX;
                break;
        }

        //
        // Update Internal Timing
        //
        last_full_update_tow = last_vel_ned.tow;
        last_full_update_wn  = last_gps_time.wn;

        return true;
    }
    return false;
}



bool
AP_GPS_SBP2::_detect(struct SBP2_detect_state &state, uint8_t data)
{
    // This switch reads one character at a time, if we find something that
    // looks like our preamble we'll try to read the full message length,
    // calculating the CRC. If the CRC matches, we have an SBP GPS!
    switch (state.state) {
        case SBP2_detect_state::WAITING:
            if (data == SBP_PREAMBLE) {
                state.n_read = 0;
                state.crc_so_far = 0;
                state.state = SBP2_detect_state::GET_TYPE;
            }
            break;

        case SBP2_detect_state::GET_TYPE:
            *((uint8_t*)&(state.msg_type) + state.n_read) = data;
            state.crc_so_far = crc16_ccitt(&data, 1, state.crc_so_far);
            state.n_read += 1;
            if (state.n_read >= 2) {
                state.n_read = 0;
                state.state = SBP2_detect_state::GET_SENDER;
            }
            break;

        case SBP2_detect_state::GET_SENDER:
            state.crc_so_far = crc16_ccitt(&data, 1, state.crc_so_far);
            state.n_read += 1;
            if (state.n_read >= 2) {
                state.n_read = 0;
                state.state = SBP2_detect_state::GET_LEN;
            }
            break;

        case SBP2_detect_state::GET_LEN:
            state.crc_so_far = crc16_ccitt(&data, 1, state.crc_so_far);
            state.msg_len = data;
            state.n_read = 0;
            state.state = SBP2_detect_state::GET_MSG;
            break;

        case SBP2_detect_state::GET_MSG:
            if (state.msg_type == SBP_HEARTBEAT_MSGTYPE && state.n_read < 4) {
                *((uint8_t*)&(state.heartbeat_buff) + state.n_read) = data;
            }
            state.crc_so_far = crc16_ccitt(&data, 1, state.crc_so_far);
            state.n_read += 1;
            if (state.n_read >= state.msg_len) {
                state.n_read = 0;
                state.state = SBP2_detect_state::GET_CRC;
            }
            break;

        case SBP2_detect_state::GET_CRC:
            *((uint8_t*)&(state.crc) + state.n_read) = data;
            state.n_read += 1;
            if (state.n_read >= 2) {
                state.state = SBP2_detect_state::WAITING;
                if (state.crc == state.crc_so_far
                        && state.msg_type == SBP_HEARTBEAT_MSGTYPE) {
                    struct sbp_heartbeat_t* heartbeat = ((struct sbp_heartbeat_t*)state.heartbeat_buff);
                    return heartbeat->protocol_major == 2;
                }
                return false;
            }
            break;

        default:
            state.state = SBP2_detect_state::WAITING;
            break;
    }
    return false;
}

void
AP_GPS_SBP2::logging_log_full_update()
{
    if (!should_df_log()) {
      return;
    }

    //TODO: Expand with heartbeat info.
    //TODO: Get rid of IAR NUM HYPO

    struct log_SbpHealth pkt = {
        LOG_PACKET_HEADER_INIT(LOG_MSG_SBPHEALTH),
        time_us                    : AP_HAL::micros64(),
        crc_error_counter          : crc_error_counter,
        last_injected_data_ms      : last_injected_data_ms,
        last_iar_num_hypotheses    : 0,
    };
    AP::logger().WriteBlock(&pkt, sizeof(pkt));
};

void
AP_GPS_SBP2::logging_log_raw_sbp(uint16_t msg_type,
        uint16_t sender_id,
        uint8_t msg_len,
        uint8_t *msg_buff) {
    if (!should_df_log()) {
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

void
AP_GPS_SBP2::logging_ext_event() {
    if (!should_df_log()) {
      return;
    }

    struct log_SbpEvent pkt = {
        LOG_PACKET_HEADER_INIT(LOG_MSG_SBPEVENT),
        time_us            : AP_HAL::micros64(),
        wn                 : last_event.wn,
        tow                : last_event.tow,
        ns_residual        : last_event.ns_residual,
        level              : last_event.flags.level,
        quality            : last_event.flags.quality,
    };
    AP::logger().WriteBlock(&pkt, sizeof(pkt));
};
