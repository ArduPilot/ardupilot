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

#define SBP_DEBUGGING 0
#define SBP_FAKE_3DLOCK 0

extern const AP_HAL::HAL& hal;

#define SBP_MILLIS_BETWEEN_HEALTHCHECKS 1500


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
#if HAL_CPU_CLASS >= HAL_CPU_CLASS_75
#define SBP_HW_LOGGING 1
#else
#define SBP_HW_LOGGING 0
#endif

bool AP_GPS_SBP::logging_started = false;

AP_GPS_SBP::AP_GPS_SBP(AP_GPS &_gps, AP_GPS::GPS_State &_state, AP_HAL::UARTDriver *_port) :
    AP_GPS_Backend(_gps, _state, _port),
    has_updated_pos(false),
    has_updated_vel(false),
    pos_msg_counter(0),
    vel_msg_counter(0),
    dops_msg_counter(0),
    baseline_msg_counter(0),
    crc_error_counter(0),
    last_healthcheck_millis(0)
{

    Debug("Initializing SBP Driver");

    port->begin(115200, 256, 16);
    port->flush();

    parser_state.state = sbp_parser_state_t::WAITING;

    state.status = AP_GPS::NO_FIX;
    state.have_vertical_velocity = true;

}

bool 
AP_GPS_SBP::read(void)
{

    //First we process all data waiting for the queue.
    sbp_process();

    uint32_t now = hal.scheduler->millis();
    uint32_t elapsed = now - last_healthcheck_millis;
    if (elapsed > SBP_MILLIS_BETWEEN_HEALTHCHECKS) {
        last_healthcheck_millis = now;

#if SBP_DEBUGGING || SBP_HW_LOGGING
        float pos_msg_hz      = pos_msg_counter / (float) elapsed * 1000.0;
        float vel_msg_hz      = vel_msg_counter / (float) elapsed * 1000.0;
        float dops_msg_hz     = dops_msg_counter / (float) elapsed * 1000.0;
        float baseline_msg_hz = baseline_msg_counter / (float) elapsed * 1000.0;
        float crc_error_hz    = crc_error_counter / (float) elapsed * 1000.0;

        pos_msg_counter = 0;
        vel_msg_counter = 0;
        dops_msg_counter = 0;
        baseline_msg_counter = 0;
        crc_error_counter = 0;

        Debug("SBP GPS perf: CRC=(%.2fHz) Pos=(%.2fHz) Vel=(%.2fHz) Dops=(%.2fHz) Baseline=(%.2fHz)\n", 
            crc_error_hz,
            pos_msg_hz,
            vel_msg_hz,
            dops_msg_hz,
            baseline_msg_hz);

#if SBP_HW_LOGGING
        logging_log_health(pos_msg_hz,
            vel_msg_hz,
            dops_msg_hz,
            baseline_msg_hz,
            crc_error_hz);

#endif
#endif
    }

    //Now we check whether we've done a full update - is all the sticky bits set?
    if (has_updated_pos && has_updated_vel) {
        state.status = AP_GPS::GPS_OK_FIX_3D;
        has_updated_pos  = false;
        has_updated_vel  = false;
        return true;
    }
    return false;    
}

//This attempts to read all the SBP messages from the incoming port.
void
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
                            case SBP_GPS_TIME_MSGTYPE:
                                sbp_process_gpstime(parser_state.msg_buff);
                                break;
                            case SBP_DOPS_MSGTYPE:
                                sbp_process_dops(parser_state.msg_buff);
                                break;
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
                            default:
                                Debug("Unknown message received: msg_type=0x%x", parser_state.msg_type);
                        }

                    } else {
                        Debug("CRC Error Occurred!\n");
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
    return;
}

void 
AP_GPS_SBP::sbp_process_gpstime(uint8_t* msg) 
{
    struct sbp_gps_time_t* t = (struct sbp_gps_time_t*)msg;
    state.time_week         = t->wn;
    state.time_week_ms      = t->tow;
    state.last_gps_time_ms  = hal.scheduler->millis();
}

void 
AP_GPS_SBP::sbp_process_dops(uint8_t* msg) 
{
    struct sbp_dops_t* d = (struct sbp_dops_t*) msg;
    state.time_week_ms      = d->tow;
    state.last_gps_time_ms  = hal.scheduler->millis();
    state.hdop              = d->hdop;
    dops_msg_counter += 1;
}

void 
AP_GPS_SBP::sbp_process_pos_ecef(uint8_t* msg) 
{
    //Ideally we'd like this data in LLH format, not ECEF
}

void 
AP_GPS_SBP::sbp_process_pos_llh(uint8_t* msg) 
{
    struct sbp_pos_llh_t* pos = (struct sbp_pos_llh_t*)msg;
    state.time_week_ms      = pos->tow;
    state.last_gps_time_ms  = hal.scheduler->millis();
    state.location.lat      = (int32_t) (pos->lat*1e7);
    state.location.lng      = (int32_t) (pos->lon*1e7);
    state.location.alt      = (int32_t) (pos->height*1e2);
    state.num_sats          = pos->n_sats;
    pos_msg_counter += 1;
    has_updated_pos = true;
}

void 
AP_GPS_SBP::sbp_process_baseline_ecef(uint8_t* msg) 
{
    struct sbp_baseline_ecef_t* b = (struct sbp_baseline_ecef_t*)msg;

    baseline_msg_counter += 1;

#if SBP_HW_LOGGING
    logging_log_baseline(b);
#endif
}

void 
AP_GPS_SBP::sbp_process_baseline_ned(uint8_t* msg) 
{
    //Ideally we'd like this data in ECEF format, not NED
}

void 
AP_GPS_SBP::sbp_process_vel_ecef(uint8_t* msg) 
{
    //Ideally we'd like this data in NED format, not ECEF
}

void 
AP_GPS_SBP::sbp_process_vel_ned(uint8_t* msg) 
{
    struct sbp_vel_ned_t* vel = (struct sbp_vel_ned_t*)msg;
    state.time_week_ms      = vel->tow;
    state.last_gps_time_ms  = hal.scheduler->millis();
    state.velocity[0]       = (float)vel->n / 1000.0;
    state.velocity[1]       = (float)vel->e / 1000.0;
    state.velocity[2]       = (float)vel->d / 1000.0;
    state.num_sats          = vel->n_sats;

    float ground_vector_sq = state.velocity[0]*state.velocity[0] + state.velocity[1]*state.velocity[1];
    state.ground_speed = safe_sqrt(ground_vector_sq);

    state.ground_course_cd = (int32_t) 100*ToDeg(atan2f(state.velocity[1], state.velocity[0]));
    if (state.ground_course_cd < 0) {
      state.ground_course_cd += 36000;
    }

    vel_msg_counter += 1;
    has_updated_vel = true;
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

#if SBP_HW_LOGGING

#define LOG_MSG_SBPHEALTH 202
#define LOG_MSG_SBPBASELINE 203

struct PACKED log_SbpHealth {
    LOG_PACKET_HEADER;
    uint32_t timestamp;
    float pos_msg_hz;
    float vel_msg_hz;
    float dops_msg_hz;
    float baseline_msg_hz;
    float crc_error_hz;
};


struct PACKED log_SbpBaseline {
    LOG_PACKET_HEADER;
    uint32_t timestamp;
    uint32_t tow;
    int32_t baseline_x;
    int32_t baseline_y;
    int32_t baseline_z;
    uint16_t baseline_accuracy;
    uint8_t num_sats;
    uint8_t flags;
};

static const struct LogStructure sbp_log_structures[] PROGMEM = {
    { LOG_MSG_SBPHEALTH, sizeof(log_SbpHealth),
      "SBPH", "Ifffff",  "TimeMS,PosHz,VelHz,DopsHz,BaseHz,CrcHz" },
    { LOG_MSG_SBPBASELINE, sizeof(log_SbpBaseline),
      "SBPB", "IIiiiHBB",  "TimeMS,tow,bx,by,bz,bacc,num_sats,flags" }
};

void AP_GPS_SBP::logging_write_headers(void)
{
    if (!logging_started) {
        logging_started = true;
        gps._DataFlash->AddLogFormats(sbp_log_structures, sizeof(sbp_log_structures) / sizeof(LogStructure));
    }
}

void AP_GPS_SBP::logging_log_health(float pos_msg_hz, float vel_msg_hz, float dops_msg_hz, float baseline_msg_hz, float crc_error_hz)
{

    if (gps._DataFlash == NULL || !gps._DataFlash->logging_started()) {
      return;
    }

    logging_write_headers();

    struct log_SbpHealth pkt = {
        LOG_PACKET_HEADER_INIT(LOG_MSG_SBPHEALTH),
        timestamp       : hal.scheduler->millis(),
        pos_msg_hz      : pos_msg_hz,
        vel_msg_hz      : vel_msg_hz,
        dops_msg_hz     : dops_msg_hz,
        baseline_msg_hz : baseline_msg_hz,
        crc_error_hz    : crc_error_hz
    };
    gps._DataFlash->WriteBlock(&pkt, sizeof(pkt));    

}

void AP_GPS_SBP::logging_log_baseline(struct sbp_baseline_ecef_t* b)
{

    if (gps._DataFlash == NULL || !gps._DataFlash->logging_started()) {
      return;
    }

    logging_write_headers();

    struct log_SbpBaseline pkt = {
        LOG_PACKET_HEADER_INIT(LOG_MSG_SBPBASELINE),
        timestamp         : hal.scheduler->millis(),
        tow               : b->tow,
        baseline_x        : b->x,
        baseline_y        : b->y,
        baseline_z        : b->z,
        baseline_accuracy : b->accuracy,
        num_sats          : b->n_sats,
        flags             : b->flags
    };
    gps._DataFlash->WriteBlock(&pkt, sizeof(pkt));    

}

#endif // SBP_HW_LOGGING
