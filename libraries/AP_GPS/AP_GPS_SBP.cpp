// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-
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
//  Swift Navigation GPS driver for ArduPilot
//	Origin code by Niels Joubert njoubert.com
//


#include <AP_GPS.h>
#include "AP_GPS_SBP.h"
#include <DataFlash.h>

#if GPS_RTK_AVAILABLE

#define SBP_DEBUGGING 0
#define SBP_FAKE_3DLOCK 0

extern const AP_HAL::HAL& hal;

#define SBP_MILLIS_BETWEEN_HEALTHCHECKS 2000U
#define SBP_BASELINE_TIMEOUT_MS 1000U
#define SBP_FIX_TIMEOUT_MS 1000U
#define SBP_HEARTBEAT_TIMEOUT_MS 5000U

#define SBP_MILLIS_BETWEEN_TRACKING_LOG 1800U

#define SBP_DEBUGGING 0
#if SBP_DEBUGGING
 # define Debug(fmt, args ...)  do {hal.console->printf("%s:%d: " fmt "\n", __FUNCTION__, __LINE__, ## args); hal.scheduler->delay(1); } while(0)
#else
 # define Debug(fmt, args ...)
 #endif

/*
  only do detailed hardware logging on boards likely to have more log
  storage space
 */
#if GPS_RTK_AVAILABLE
#define SBP_HW_LOGGING 1
#else
#define SBP_HW_LOGGING 0
#endif

bool AP_GPS_SBP::logging_started = false;

AP_GPS_SBP::AP_GPS_SBP(AP_GPS &_gps, AP_GPS::GPS_State &_state, AP_HAL::UARTDriver *_port) :
    AP_GPS_Backend(_gps, _state, _port),

    last_baseline_received_ms(0),
    last_heatbeat_received_ms(0),
    last_tracking_state_ms(0),
    iar_num_hypotheses(-1),
    baseline_recv_rate(0),

    dgps_corrections_incoming(false),
    rtk_corrections_incoming(false),

    has_new_pos_llh(false),
    has_new_vel_ned(false),
    has_new_baseline_ecef(false),

    has_rtk_base_pos(false),

    pos_msg_counter(0),
    vel_msg_counter(0),
    baseline_msg_counter(0),
    full_update_counter(0),
    crc_error_counter(0),
    last_healthcheck_millis(0)
{

    parser_state.state = sbp_parser_state_t::WAITING;

    state.status = AP_GPS::NO_FIX;
    state.have_vertical_velocity = true;

    state.last_gps_time_ms = last_heatbeat_received_ms = last_healthcheck_millis = hal.scheduler->millis();

}

bool 
AP_GPS_SBP::can_calculate_base_pos(void) 
{
    return (rtk_corrections_incoming && !has_rtk_base_pos);
};


void 
AP_GPS_SBP::calculate_base_pos(void)
{
    //INVARIANT:
    //  Only ever capture home with motors not armed!
    //  External driver checks whether can_raise_fix_level becomes true
    //  and only if it can, AND motors are not armed, will be capture home!
    if (state.status < AP_GPS::GPS_OK_FIX_3D) {
        Debug("Attempting to capture home without GPS Fix available. Can't do RTK without home lat-lon.");
        return;
    }

    if (!rtk_corrections_incoming) {
        Debug("Attempting to capture home baseline without rtk corrections being received.");
        return;
    }

    Vector3d current_llh;
    Vector3d current_ecef;
    Vector3d current_baseline_ecef;

    current_llh[0] = last_sbp_pos_llh_msg.lat * DEG_TO_RAD_DOUBLE;
    current_llh[1] = last_sbp_pos_llh_msg.lon * DEG_TO_RAD_DOUBLE;
    current_llh[2] = last_sbp_pos_llh_msg.height;

    wgsllh2ecef(current_llh, current_ecef);

    current_baseline_ecef[0] = ((double)last_sbp_baseline_ecef_msg.x) / 1000.0;
    current_baseline_ecef[1] = ((double)last_sbp_baseline_ecef_msg.y) / 1000.0;
    current_baseline_ecef[2] = ((double)last_sbp_baseline_ecef_msg.z) / 1000.0;

    base_pos_ecef = current_ecef - current_baseline_ecef;
    has_rtk_base_pos = true;
    
    Debug("SBP Got Base Position! has_rtk_base_pos=%d, (%.2f, %.2f, %.2f)", has_rtk_base_pos,
        base_pos_ecef[0],
        base_pos_ecef[1],
        base_pos_ecef[2]);
}
void
AP_GPS_SBP::invalidate_base_pos()
{
    has_rtk_base_pos = false;
}

bool 
AP_GPS_SBP::read(void)
{
    //Invariant: Calling this function processes *all* data current in the UART buffer.
    //
    //IMPORTANT NOTICE: This function is NOT CALLED for several seconds
    // during arming. That should not cause the driver to die. Process *all* waiting messages

    bool full_update = false;
    do {
        //Attempt to process one message at a time
        bool new_message = sbp_process();

        //Attempt to update our internal state with this new message.
        if (update_state(new_message)) {
            full_update = true;
            full_update_counter += 1;
        }

    } while (port->available() > 0);

    uint32_t now = hal.scheduler->millis();
    uint32_t elapsed = now - last_healthcheck_millis;
    if (elapsed > SBP_MILLIS_BETWEEN_HEALTHCHECKS) {
        last_healthcheck_millis = now;

        float pos_msg_hz      = pos_msg_counter / (float) elapsed * 1000;
        float vel_msg_hz      = vel_msg_counter / (float) elapsed * 1000;
        float baseline_msg_hz = baseline_msg_counter / (float) elapsed * 1000;
        float full_update_hz  = full_update_counter / (float) elapsed * 1000;

        baseline_recv_rate = uint8_t (baseline_msg_hz * 10);

        pos_msg_counter = 0;
        vel_msg_counter = 0;
        baseline_msg_counter = 0;
        full_update_counter = 0;

        Debug("SBP GPS perf: Fix=(%d) CRC=(%d) Pos=(%.2fHz) Vel=(%.2fHz) Baseline=(%.2fHz) Update=(%.2fHz) DGPS=(%d) RTK=(%d) RTK_HOME=(%d) IAR=(%d)", 
            state.status,
            crc_error_counter,
            pos_msg_hz,
            vel_msg_hz,
            baseline_msg_hz,
            full_update_hz,
            dgps_corrections_incoming,
            rtk_corrections_incoming,
            has_rtk_base_pos,
            iar_num_hypotheses);

#if SBP_HW_LOGGING
        logging_log_health(pos_msg_hz,
                           vel_msg_hz,
                           baseline_msg_hz,
                           full_update_hz);
#endif

    }

    return full_update;
}

//This consolidates all the latest messages,
//and the current mode the driver is in
//
// INVARIANT: 
//   If in a fix mode >= 3, 
//     returns true only if a full position and velocity update happened.
//   If in fix mode 0 or 1,
//     returns true if messages are being received or we haven't timed out
bool
AP_GPS_SBP::update_state(bool has_new_message)
{

    uint32_t now = hal.scheduler->millis();

    //Determine the current mode the GPS is in: DGPS or plain
    //Notice that this is sticky.
    if (has_new_baseline_ecef && (now - last_baseline_received_ms < SBP_BASELINE_TIMEOUT_MS)) {
        dgps_corrections_incoming = true;
        if (gps._min_dgps >= 100) { 
            //Allow only IntegerRTK baselines
            rtk_corrections_incoming  = dgps_corrections_incoming && (last_sbp_baseline_ecef_msg.flags & 0x1);
        } else { 
            //Allow floatRTK baselines
            rtk_corrections_incoming  = dgps_corrections_incoming;
        }
    }

    //Currently we only use relative positioning if we have RTK-level fixes,
    //we ignore float-level fixes
    bool using_relative_positioning = rtk_corrections_incoming && has_rtk_base_pos;

    //Drop out of RTK mode if we haven't seen a baseline for a while...
    if (using_relative_positioning && (now - last_baseline_received_ms > SBP_BASELINE_TIMEOUT_MS)) {
        dgps_corrections_incoming = false;
        rtk_corrections_incoming = false;
        using_relative_positioning = false;
    }

    //UPDATE POSITION AND VELOCITY
    if (!using_relative_positioning && 
            (has_new_pos_llh && has_new_vel_ned) && 
            (last_sbp_pos_llh_msg.tow == last_sbp_vel_ned_msg.tow)) {

        state.last_gps_time_ms  = hal.scheduler->millis();

        state.time_week_ms      = last_sbp_pos_llh_msg.tow;
        state.location.lat      = (int32_t) (last_sbp_pos_llh_msg.lat*1e7);
        state.location.lng      = (int32_t) (last_sbp_pos_llh_msg.lon*1e7);
        state.location.alt      = (int32_t) (last_sbp_pos_llh_msg.height*1e2);
        state.num_sats          = last_sbp_pos_llh_msg.n_sats;

        update_state_velocity();

        has_new_pos_llh = false;

        state.status = AP_GPS::GPS_OK_FIX_3D;

        return true;

    } else if (using_relative_positioning &&
            (has_new_baseline_ecef && has_new_vel_ned) &&
            (last_sbp_baseline_ecef_msg.tow == last_sbp_vel_ned_msg.tow)) {

        state.last_gps_time_ms  = hal.scheduler->millis();

        //Generate a new lat-lon from baseline

        //Grab the current baseline
        Vector3d current_baseline_ecef; //units are currently in mm
        current_baseline_ecef[0] = ((double)last_sbp_baseline_ecef_msg.x) / 1000.0;
        current_baseline_ecef[1] = ((double)last_sbp_baseline_ecef_msg.y) / 1000.0;
        current_baseline_ecef[2] = ((double)last_sbp_baseline_ecef_msg.z) / 1000.0;

        //Offset the reference point from that
        Vector3d current_pos_ecef;
        current_pos_ecef = base_pos_ecef + current_baseline_ecef;

        Vector3d current_pos_llh;
        wgsecef2llh(current_pos_ecef, current_pos_llh);
    
        current_pos_llh[0] *= RAD_TO_DEG_DOUBLE;
        current_pos_llh[1] *= RAD_TO_DEG_DOUBLE;

        state.time_week_ms = last_sbp_baseline_ecef_msg.tow;
        state.location.lat = (int32_t) (current_pos_llh[0] * 1e7);
        state.location.lng = (int32_t) (current_pos_llh[1] * 1e7);
        state.location.alt = (int32_t) (current_pos_llh[2] * 1e3);
        state.num_sats     = last_sbp_baseline_ecef_msg.n_sats;

        update_state_velocity();

        has_new_baseline_ecef = false;

        state.status = AP_GPS::GPS_OK_FIX_3D_RTK;
        
        return true;
    }


    //If we get here,
    //We have not been able to update the GPS state yet for this process call.
    //Check whether the GPS is still alive and processing messages!

    if (!using_relative_positioning && (now - state.last_gps_time_ms > SBP_FIX_TIMEOUT_MS)) {
        state.status = AP_GPS::NO_FIX;
        return (now - last_heatbeat_received_ms < SBP_HEARTBEAT_TIMEOUT_MS);
    }

    if (now - last_heatbeat_received_ms > SBP_HEARTBEAT_TIMEOUT_MS) {
        state.status = AP_GPS::NO_GPS;
        return false;
    }

    if (state.status < AP_GPS::GPS_OK_FIX_3D) {
        //If we are receiving messages, but dont have a fix yet, thats okay.
        return has_new_message;
    } else {
        //If we have a fix and we got here, then we're in between message synchronizations
        return false;
    }
    return true;

}

void 
AP_GPS_SBP::update_state_velocity(void)
{

    state.time_week_ms      = last_sbp_vel_ned_msg.tow;
    state.velocity[0]       = (float)(last_sbp_vel_ned_msg.n / 1000.0);
    state.velocity[1]       = (float)(last_sbp_vel_ned_msg.e / 1000.0);
    state.velocity[2]       = (float)(last_sbp_vel_ned_msg.d / 1000.0);

    float ground_vector_sq = state.velocity[0]*state.velocity[0] + state.velocity[1]*state.velocity[1];
    state.ground_speed = safe_sqrt(ground_vector_sq);

    state.ground_course_cd = (int32_t) 100*ToDeg(atan2f(state.velocity[1], state.velocity[0]));
    if (state.ground_course_cd < 0) {
      state.ground_course_cd += 36000;
    }

    has_new_vel_ned = false;

}

//This attempts to read a SINGLE SBP messages from the incoming port.
//Returns true if a new message was read, false if we failed to read a message.
bool
AP_GPS_SBP::sbp_process() 
{
    while (port->available() > 0) {
        uint8_t temp = port->read();
        uint16_t crc;


        //This switch reads one character at a time,
        //parsing it into buffers until a full message is dispatched
        switch(parser_state.state) {
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
                        //OK, we have a valid message. Dispatch the appropriate function:
                        switch(parser_state.msg_type) {
                            case SBP_POS_ECEF_MSGTYPE:
                                sbp_process_pos_ecef(parser_state.msg_buff);
                                break;
                            case SBP_POS_LLH_MSGTYPE:
                                sbp_process_pos_llh(parser_state.msg_buff);
                                break;
                            case SBP_BASELINE_ECEF_MSGTYPE:
                                sbp_process_baseline_ecef(parser_state.msg_buff);
                                break;
                            case SBP_BASELINE_NED_MSGTYPE:
                                sbp_process_baseline_ned(parser_state.msg_buff);
                                break;
                            case SBP_VEL_ECEF_MSGTYPE:
                                sbp_process_vel_ecef(parser_state.msg_buff);
                                break;
                            case SBP_VEL_NED_MSGTYPE:
                                sbp_process_vel_ned(parser_state.msg_buff);
                                break;
                            case SBP_GPS_TIME_MSGTYPE:
                                sbp_process_gpstime(parser_state.msg_buff);
                                break;
                            case SBP_DOPS_MSGTYPE:
                                sbp_process_dops(parser_state.msg_buff);
                                break;
                            case SBP_TRACKING_STATE_MSGTYPE:
                                sbp_process_tracking_state(parser_state.msg_buff, parser_state.msg_len);
                                break;
                            case SBP_IAR_STATE_MSGTYPE:
                                sbp_process_iar_state(parser_state.msg_buff);
                                break;
                            case SBP_HEARTBEAT_MSGTYPE:
                                sbp_process_heartbeat(parser_state.msg_buff);
                                break;
                            case SBP_STARTUP_MSGTYPE:
                                sbp_process_startup(parser_state.msg_buff);
                                break;
                        }
                        return true;

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
    //We have parsed all the waiting messages
    return false;
}

void
AP_GPS_SBP::sbp_process_heartbeat(uint8_t* msg)
{
    last_heatbeat_received_ms = hal.scheduler->millis();
}

void 
AP_GPS_SBP::sbp_process_gpstime(uint8_t* msg) 
{
    struct sbp_gps_time_t* t = (struct sbp_gps_time_t*)msg;
    state.time_week         = t->wn;
    state.time_week_ms      = t->tow;
}

void 
AP_GPS_SBP::sbp_process_dops(uint8_t* msg) 
{
    struct sbp_dops_t* d = (struct sbp_dops_t*) msg;
    state.time_week_ms      = d->tow;
    state.hdop              = d->hdop;
}

void 
AP_GPS_SBP::sbp_process_pos_ecef(uint8_t* msg) 
{
    //Using LLH, not ECEF
}

void 
AP_GPS_SBP::sbp_process_pos_llh(uint8_t* msg) 
{
    struct sbp_pos_llh_t* pos = (struct sbp_pos_llh_t*)msg;
    last_sbp_pos_llh_msg = *pos;

    has_new_pos_llh = true;

#if SBP_DEBUGGING || SBP_HW_LOGGING
    pos_msg_counter += 1;
#endif

#if SBP_HW_LOGGING
    logging_log_llh(pos);
#endif
}

void 
AP_GPS_SBP::sbp_process_baseline_ecef(uint8_t* msg) 
{
    struct sbp_baseline_ecef_t* b = (struct sbp_baseline_ecef_t*)msg;
    last_sbp_baseline_ecef_msg = *b;

    last_baseline_received_ms = hal.scheduler->millis();
    has_new_baseline_ecef = true;

#if SBP_DEBUGGING || SBP_HW_LOGGING
    baseline_msg_counter += 1;
#endif

#if SBP_HW_LOGGING
    logging_log_baseline_ecef(b);
#endif
}

void 
AP_GPS_SBP::sbp_process_baseline_ned(uint8_t* msg) 
{
    //Currently we use ECEF baselines.
    //This is just for logging purposes.
    struct sbp_baseline_ned_t* b = (struct sbp_baseline_ned_t*)msg;
    last_sbp_baseline_ned_msg = *b;

}

void 
AP_GPS_SBP::sbp_process_vel_ecef(uint8_t* msg) 
{
    //Currently we use NED velocity.
}

void 
AP_GPS_SBP::sbp_process_vel_ned(uint8_t* msg) 
{
    struct sbp_vel_ned_t* vel = (struct sbp_vel_ned_t*)msg;
    last_sbp_vel_ned_msg = *vel;

    has_new_vel_ned = true;

#if SBP_DEBUGGING || SBP_HW_LOGGING
    vel_msg_counter += 1;
#endif
}

void
AP_GPS_SBP::sbp_process_tracking_state(uint8_t* msg, uint8_t len)
{
    uint32_t now = hal.scheduler->millis();

    struct sbp_tracking_state_t* tracking_state = (struct sbp_tracking_state_t*)msg;
    last_sbp_tracking_state_msg = *tracking_state;

    uint8_t num = len / sizeof(sbp_tracking_state_t);
    last_sbp_tracking_state_msg_num = num;

    //Rate-limit the tracking state messages to no more than 1.8 seconds
    if (now - last_tracking_state_ms > SBP_MILLIS_BETWEEN_TRACKING_LOG) {
        last_tracking_state_ms = now;

#ifdef SBP_HW_LOGGING
        logging_log_tracking_state(tracking_state, num);
#endif

    }

}

void
AP_GPS_SBP::sbp_process_iar_state(uint8_t* msg)
{
    struct sbp_iar_state_t* iar_state = (struct sbp_iar_state_t*)msg;
    iar_num_hypotheses = (int32_t) iar_state->num_hypotheses;
}

void 
AP_GPS_SBP::sbp_process_startup(uint8_t* msg)
{
    invalidate_base_pos();
}

bool
AP_GPS_SBP::_detect(struct SBP_detect_state &state, uint8_t data) 
{
    //This switch reads one character at a time,
    //if we find something that looks like our preamble
    //we'll try to read the full message length, calculating the CRC.
    //If the CRC matches, we have a SBP GPS!
    switch(state.state) {
        case SBP_detect_state::WAITING:
            if (data == SBP_PREAMBLE) {
                state.n_read = 0;
                state.crc_so_far = 0;
                state.state = SBP_detect_state::GET_TYPE;
            }
            break;

        case SBP_detect_state::GET_TYPE:
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
                return state.crc == state.crc_so_far;
            }
            break;

        default:
            state.state = SBP_detect_state::WAITING;
            break;
    }
    return false;
}

void 
AP_GPS_SBP::send_mavlink_gps_rtk(mavlink_channel_t chan)
{

    uint8_t health = dgps_corrections_incoming |
        (rtk_corrections_incoming << 1) |
        (has_rtk_base_pos << 2);

        mavlink_msg_gps_rtk_send(
            chan,
            last_baseline_received_ms,                 // Time since boot of last baseline message received in ms.
            AP_GPS::GPS_TYPE_SBP,                       // Identification of connected RTK receiver.
            state.time_week,                        // GPS Week Number of last baseline
            last_sbp_baseline_ned_msg.tow,          // GPS Time of Week of last baseline
            health,                                 // GPS-specific health report for RTK data.
            baseline_recv_rate,                     // Rate of baseline messages being received by GPS, in HZ*10
            last_sbp_baseline_ned_msg.n_sats,       // Current number of sats used for RTK calculation.
            1,                                      // Coordinate system of baseline. 0 == ECEF, 1 == NED
            last_sbp_baseline_ned_msg.n,            // Current baseline in ECEF x or NED north component in mm
            last_sbp_baseline_ned_msg.e,            // Current baseline in ECEF y or NED east component in mm
            last_sbp_baseline_ned_msg.d,            // Current baseline in ECEF z or NED down component in mm
            last_sbp_baseline_ned_msg.h_accuracy,   // Current estimate of baseline accuracy.
            iar_num_hypotheses                      // Current number of integer ambiguity hypotheses.
        );

}

#if GPS_MAX_INSTANCES > 1
void 
AP_GPS_SBP::send_mavlink_gps2_rtk(mavlink_channel_t chan)
{

    uint8_t health = dgps_corrections_incoming |
        (rtk_corrections_incoming << 1) |
        (has_rtk_base_pos << 2);

        mavlink_msg_gps2_rtk_send(
            chan,
            last_baseline_received_ms,                 // Time since boot of last baseline message received in ms.
            AP_GPS::GPS_TYPE_SBP,                       // Identification of connected RTK receiver.
            state.time_week,                        // GPS Week Number of last baseline
            last_sbp_baseline_ned_msg.tow,          // GPS Time of Week of last baseline
            health,                                 // GPS-specific health report for RTK data.
            baseline_recv_rate,                     // Rate of baseline messages being received by GPS, in HZ*10
            last_sbp_baseline_ned_msg.n_sats,       // Current number of sats used for RTK calculation.
            1,                                      // Coordinate system of baseline. 0 == ECEF, 1 == NED
            last_sbp_baseline_ned_msg.n,            // Current baseline in ECEF x or NED north component in mm
            last_sbp_baseline_ned_msg.e,            // Current baseline in ECEF y or NED east component in mm
            last_sbp_baseline_ned_msg.d,            // Current baseline in ECEF z or NED down component in mm
            last_sbp_baseline_ned_msg.h_accuracy,   // Current estimate of baseline accuracy.
            iar_num_hypotheses                      // Current number of integer ambiguity hypotheses.
        );
}
#endif

#if SBP_HW_LOGGING

#define LOG_MSG_SBPHEALTH 202
#define LOG_MSG_SBPLLH 203
#define LOG_MSG_SBPBASELINE 204
#define LOG_MSG_SBPTRACKING1 205
#define LOG_MSG_SBPTRACKING2 206

struct PACKED log_SbpHealth {
    LOG_PACKET_HEADER;
    uint32_t timestamp;
    float pos_msg_hz;
    float vel_msg_hz;
    float baseline_msg_hz;
    float full_update_hz;
    uint32_t crc_error_counter;
    uint8_t dgps_corrections_incoming;
    uint8_t rtk_corrections_incoming;
    uint8_t has_rtk_base_pos;
    int32_t iar_num_hypotheses;
};

struct PACKED log_SbpLLH {
    LOG_PACKET_HEADER;
    uint32_t timestamp;
    uint32_t tow;
    int32_t  lat;
    int32_t  lon;
    int32_t  alt;
    uint8_t  n_sats;
};

struct PACKED log_SbpBaseline {
    LOG_PACKET_HEADER;
    uint32_t timestamp;
    uint32_t tow;          //< GPS Time of Week of ECEF Baseline (unit: ms)
    int32_t x;             //< Baseline ECEF X coordinate
    int32_t y;             //< Baseline ECEF Y coordinate
    int32_t z;             //< Baseline ECEF Z coordinate
    int32_t length;        //< Baseline length
    uint16_t accuracy;     //< Horizontal position accuracy estimate (unit: mm)
    uint8_t n_sats;        //< Number of satellites used in solution
    uint8_t flags;         //< Status flags (reserved)    
};

struct PACKED log_SbpTracking1 {
    LOG_PACKET_HEADER;
    uint32_t timestamp;
    uint8_t ch1_prn;
    float   ch1_cn0;
    uint8_t ch2_prn;
    float   ch2_cn0;
    uint8_t ch3_prn;
    float   ch3_cn0;
    uint8_t ch4_prn;
    float   ch4_cn0;
    uint8_t ch5_prn;
    float   ch5_cn0;
    uint8_t ch6_prn;
    float   ch6_cn0;
    uint8_t ch7_prn;
    float   ch7_cn0;
};

struct PACKED log_SbpTracking2 {
    LOG_PACKET_HEADER;
    uint32_t timestamp;
    uint8_t ch8_prn;
    float   ch8_cn0;
    uint8_t ch9_prn;
    float   ch9_cn0;
    uint8_t ch10_prn;
    float   ch10_cn0;
    uint8_t ch11_prn;
    float   ch11_cn0;
    uint8_t ch12_prn;
    float   ch12_cn0;
    uint8_t ch13_prn;
    float   ch13_cn0;
    uint8_t ch14_prn;
    float   ch14_cn0;
};

static const struct LogStructure sbp_log_structures[] PROGMEM = {
    { LOG_MSG_SBPHEALTH, sizeof(log_SbpHealth),
      "SBPH", "IffffIBBBi",  "TimeMS,PHz,VHz,BHz,UpHz,CrcError,dgpsOn,rtkOn,hasRtkBase,IAR" },
    { LOG_MSG_SBPLLH, sizeof(log_SbpLLH),
      "SBPL", "IIiiiB",      "TimeMS,tow,lat,lon,alt,num_sats" },
    { LOG_MSG_SBPBASELINE, sizeof(log_SbpBaseline),
      "SBPB", "IIiiiiHBB",  "TimeMS,tow,x,y,z,len,acc,num_sats,flags" },
    { LOG_MSG_SBPTRACKING1, sizeof(log_SbpTracking1),
      "SBT1", "IBfBfBfBfBfBfBf",  "TimeMS,s1,c1,s2,c2,s3,c3,s4,c4,s5,c5,s6,c6,s7,c7" },
    { LOG_MSG_SBPTRACKING2, sizeof(log_SbpTracking2),      
      "SBT2", "IBfBfBfBfBfBfBf",  "TimeMS,s8,c8,s9,c9,s10,c10,s11,c11,s12,c12,s13,c13,s14,c14" }

};

void 
AP_GPS_SBP::logging_write_headers(void)
{
    if (!logging_started) {
        logging_started = true;
        gps._DataFlash->AddLogFormats(sbp_log_structures, sizeof(sbp_log_structures) / sizeof(LogStructure));
    }
}

void 
AP_GPS_SBP::logging_log_health(float pos_msg_hz, float vel_msg_hz, float baseline_msg_hz, float full_update_hz)
{

    if (gps._DataFlash == NULL || !gps._DataFlash->logging_started()) {
      return;
    }

    logging_write_headers();

    struct log_SbpHealth pkt = {
        LOG_PACKET_HEADER_INIT(LOG_MSG_SBPHEALTH),
        timestamp                  : hal.scheduler->millis(),
        pos_msg_hz                 : pos_msg_hz,
        vel_msg_hz                 : vel_msg_hz,
        baseline_msg_hz            : baseline_msg_hz,
        full_update_hz             : full_update_hz,
        crc_error_counter          : crc_error_counter,
        dgps_corrections_incoming  : dgps_corrections_incoming,
        rtk_corrections_incoming   : rtk_corrections_incoming,
        has_rtk_base_pos           : has_rtk_base_pos,
        iar_num_hypotheses         : iar_num_hypotheses
    };
    gps._DataFlash->WriteBlock(&pkt, sizeof(pkt));    

};

void
AP_GPS_SBP::logging_log_llh(struct sbp_pos_llh_t* p)
{

    if (gps._DataFlash == NULL || !gps._DataFlash->logging_started()) {
      return;
    }

    logging_write_headers();

    struct log_SbpLLH pkt = {
        LOG_PACKET_HEADER_INIT(LOG_MSG_SBPLLH),
        timestamp       : hal.scheduler->millis(),
        tow             : p->tow,
        lat             : (int32_t) (p->lat*1e7),
        lon             : (int32_t) (p->lon*1e7),
        alt             : (int32_t) (p->height*1e2),
        n_sats          : p->n_sats,
    };
    gps._DataFlash->WriteBlock(&pkt, sizeof(pkt));    

};

void 
AP_GPS_SBP::logging_log_baseline_ecef(struct sbp_baseline_ecef_t* b)
{

    if (gps._DataFlash == NULL || !gps._DataFlash->logging_started()) {
      return;
    }

    logging_write_headers();

    float x = b->x / 1000.0;
    float y = b->y / 1000.0;
    float z = b->z / 1000.0;
    int32_t len = (int32_t) (safe_sqrt(x*x+y*y+z*z) * 1000.0f);

    struct log_SbpBaseline pkt = {
        LOG_PACKET_HEADER_INIT(LOG_MSG_SBPBASELINE),
        timestamp     : hal.scheduler->millis(),
        tow           : b->tow,
        x             : b->x,
        y             : b->y,
        z             : b->z,
        length        : len,
        accuracy      : b->accuracy,
        n_sats        : b->n_sats,
        flags         : b->flags
    };

    gps._DataFlash->WriteBlock(&pkt, sizeof(pkt));    

};


void 
AP_GPS_SBP::logging_log_tracking_state(struct sbp_tracking_state_t* tstate, uint8_t num)
{
    if (gps._DataFlash == NULL || !gps._DataFlash->logging_started()) {
      return;
    }

    logging_write_headers();

    struct log_SbpTracking1 pkt = {
        LOG_PACKET_HEADER_INIT(LOG_MSG_SBPTRACKING1),
        timestamp         : hal.scheduler->millis(),
        ch1_prn           : tstate[0].prn,
        ch1_cn0           : tstate[0].cn0,
        ch2_prn           : (uint8_t)(num < 1 ? 0 : tstate[1].prn),
        ch2_cn0           : num < 1 ? 0 : tstate[1].cn0,
        ch3_prn           : (uint8_t)(num < 2 ? 0 : tstate[2].prn),
        ch3_cn0           : num < 2 ? 0 : tstate[2].cn0,
        ch4_prn           : (uint8_t)(num < 3 ? 0 : tstate[3].prn),
        ch4_cn0           : num < 3 ? 0 : tstate[3].cn0,
        ch5_prn           : (uint8_t)(num < 4 ? 0 : tstate[4].prn),
        ch5_cn0           : num < 4 ? 0 : tstate[4].cn0,
        ch6_prn           : (uint8_t)(num < 5 ? 0 : tstate[5].prn),
        ch6_cn0           : num < 5 ? 0 : tstate[5].cn0,
        ch7_prn           : (uint8_t)(num < 6 ? 0 : tstate[6].prn),
        ch7_cn0           : num < 6 ? 0 : tstate[6].cn0,
    };
    gps._DataFlash->WriteBlock(&pkt, sizeof(pkt));    

    if (num > 6) {

        struct log_SbpTracking2 pkt2 = {
            LOG_PACKET_HEADER_INIT(LOG_MSG_SBPTRACKING2),
            timestamp         : hal.scheduler->millis(),
            ch8_prn           : (uint8_t)(num < 7  ? 0 : tstate[7].prn),
            ch8_cn0           : num < 7  ? 0 : tstate[7].cn0,
            ch9_prn           : (uint8_t)(num < 8  ? 0 : tstate[8].prn),
            ch9_cn0           : num < 8  ? 0 : tstate[8].cn0,
            ch10_prn          : (uint8_t)(num < 9  ? 0 : tstate[9].prn),
            ch10_cn0          : num < 9  ? 0 : tstate[9].cn0,
            ch11_prn          : (uint8_t)(num < 10 ? 0 : tstate[10].prn),
            ch11_cn0          : num < 10 ? 0 : tstate[10].cn0,
            ch12_prn          : (uint8_t)(num < 11 ? 0 : tstate[11].prn),
            ch12_cn0          : num < 11 ? 0 : tstate[11].cn0,
            ch13_prn          : (uint8_t)(num < 12 ? 0 : tstate[12].prn),
            ch13_cn0          : num < 12 ? 0 : tstate[12].cn0,
            ch14_prn          : (uint8_t)(num < 13 ? 0 : tstate[13].prn),
            ch14_cn0          : num < 13 ? 0 : tstate[13].cn0,
        };
        gps._DataFlash->WriteBlock(&pkt2, sizeof(pkt));

    };


};

#endif // SBP_HW_LOGGING

#endif // GPS_RTK_AVAILABLE
