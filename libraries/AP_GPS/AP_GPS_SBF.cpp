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
//  Septentrio GPS driver for ArduPilot.
//  Code by Michael Oborne
//

#include "AP_GPS.h"
#include "AP_GPS_SBF.h"
#include <DataFlash/DataFlash.h>
#include <GCS_MAVLink/GCS.h>

extern const AP_HAL::HAL& hal;

#define SBF_DEBUGGING 0

#if SBF_DEBUGGING
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

#define SBF_EXCESS_COMMAND_BYTES 5 // 2 start bytes + validity byte + space byte + endline byte

#define RX_ERROR_MASK (SOFTWARE      | \
                       CONGESTION    | \
                       MISSEDEVENT   | \
                       CPUOVERLOAD   | \
                       INVALIDCONFIG | \
                       OUTOFGEOFENCE)


AP_GPS_SBF::AP_GPS_SBF(AP_GPS &_gps, AP_GPS::GPS_State &_state,
                       AP_HAL::UARTDriver *_port) :
    AP_GPS_Backend(_gps, _state, _port)
{
    sbf_msg.sbf_state = sbf_msg_parser_t::PREAMBLE1;

    port->write((const uint8_t*)_port_enable, strlen(_port_enable));
    _config_last_ack_time = AP_HAL::millis();
}

// Process all bytes available from the stream
//
bool
AP_GPS_SBF::read(void)
{
    uint32_t now = AP_HAL::millis();

    if (gps._auto_config != AP_GPS::GPS_AUTO_CONFIG_DISABLE &&
        _init_blob_index < ARRAY_SIZE(_initialisation_blob)) {
        const char *init_str = _initialisation_blob[_init_blob_index];

        if (now > _init_blob_time) {
            if (now > _config_last_ack_time + 2500) {
                // try to enable input on the GPS port if we have not made progress on configuring it
                Debug("SBF Sending port enable");
                port->write((const uint8_t*)_port_enable, strlen(_port_enable));
                _config_last_ack_time = now;
            } else {
                Debug("SBF sending init string: %s", init_str);
                port->write((const uint8_t*)init_str, strlen(init_str));
            }
            _init_blob_time = now + 1000;
        }
    }

    bool ret = false;
    while (port->available() > 0) {
        uint8_t temp = port->read();
        ret |= parse(temp);
    }

    return ret;
}

bool
AP_GPS_SBF::parse(uint8_t temp)
{
    switch (sbf_msg.sbf_state)
    {
        default:
        case sbf_msg_parser_t::PREAMBLE1:
            if (temp == SBF_PREAMBLE1) {
                sbf_msg.sbf_state = sbf_msg_parser_t::PREAMBLE2;
                sbf_msg.read = 0;
            }
            break;
        case sbf_msg_parser_t::PREAMBLE2:
            if (temp == SBF_PREAMBLE2) {
                sbf_msg.sbf_state = sbf_msg_parser_t::CRC1;
            } else if (temp == 'R') {
                Debug("SBF got a response\n");
                sbf_msg.sbf_state = sbf_msg_parser_t::COMMAND_LINE;
            }
            else
            {
                sbf_msg.sbf_state = sbf_msg_parser_t::PREAMBLE1;
            }
            break;
        case sbf_msg_parser_t::CRC1:
            sbf_msg.crc = temp;
            sbf_msg.sbf_state = sbf_msg_parser_t::CRC2;
            break;
        case sbf_msg_parser_t::CRC2:
            sbf_msg.crc += (uint16_t)(temp << 8);
            sbf_msg.sbf_state = sbf_msg_parser_t::BLOCKID1;
            break;
        case sbf_msg_parser_t::BLOCKID1:
            sbf_msg.blockid = temp;
            sbf_msg.sbf_state = sbf_msg_parser_t::BLOCKID2;
            break;
        case sbf_msg_parser_t::BLOCKID2:
            sbf_msg.blockid += (uint16_t)(temp << 8);
            sbf_msg.sbf_state = sbf_msg_parser_t::LENGTH1;
            break;
        case sbf_msg_parser_t::LENGTH1:
            sbf_msg.length = temp;
            sbf_msg.sbf_state = sbf_msg_parser_t::LENGTH2;
            break;
        case sbf_msg_parser_t::LENGTH2:
            sbf_msg.length += (uint16_t)(temp << 8);
            sbf_msg.sbf_state = sbf_msg_parser_t::DATA;
            if (sbf_msg.length % 4 != 0) {
                sbf_msg.sbf_state = sbf_msg_parser_t::PREAMBLE1;
                Debug("bad packet length=%u\n", (unsigned)sbf_msg.length);
            }
            if (sbf_msg.length < 8) {
                Debug("bad packet length=%u\n", (unsigned)sbf_msg.length);
                sbf_msg.sbf_state = sbf_msg_parser_t::PREAMBLE1;
                crc_error_counter++; // this is a probable buffer overflow, but this
                                     // indicates not enough bytes to do a crc
                break;
            }
            break;
        case sbf_msg_parser_t::DATA:
            if (sbf_msg.read < sizeof(sbf_msg.data)) {
                sbf_msg.data.bytes[sbf_msg.read] = temp;
            }
            sbf_msg.read++;
            if (sbf_msg.read >= (sbf_msg.length - 8)) {
                if (sbf_msg.read > sizeof(sbf_msg.data)) {
                    // not interested in these large messages
                    sbf_msg.sbf_state = sbf_msg_parser_t::PREAMBLE1;
                    break;
                }
                uint16_t crc = crc16_ccitt((uint8_t*)&sbf_msg.blockid, 2, 0);
                crc = crc16_ccitt((uint8_t*)&sbf_msg.length, 2, crc);
                crc = crc16_ccitt((uint8_t*)&sbf_msg.data, sbf_msg.length - 8, crc);

                sbf_msg.sbf_state = sbf_msg_parser_t::PREAMBLE1;

                if (sbf_msg.crc == crc) {
                    return process_message();
                } else {
                    Debug("crc fail\n");
                    crc_error_counter++;
                }
            }
            break;
        case sbf_msg_parser_t::COMMAND_LINE:
            if (sbf_msg.read < (sizeof(sbf_msg.data) - 1)) {
                sbf_msg.data.bytes[sbf_msg.read] = temp;
            } else {
                // we don't have enough buffer to compare the commands
                // most probable cause is that a user injected a longer command then
                // we have buffer for, or it could be a corruption, either way we
                // simply ignore the result
                sbf_msg.sbf_state = sbf_msg_parser_t::PREAMBLE1;
                break;
            }
            sbf_msg.read++;
            if (temp == '\n') {
                sbf_msg.data.bytes[sbf_msg.read] = 0;

                // received the result, lets assess it
                if (sbf_msg.data.bytes[0] == ':') {
                    // valid command, determine if it was the one we were trying
                    // to send in the configuration sequence
                    if (_init_blob_index < ARRAY_SIZE(_initialisation_blob)) {
                        if (!strncmp(_initialisation_blob[_init_blob_index], (char *)(sbf_msg.data.bytes + 2),
                                     sbf_msg.read - SBF_EXCESS_COMMAND_BYTES)) {
                            Debug("SBF Ack Command: %s\n", sbf_msg.data.bytes);
                            _init_blob_index++;
                            _config_last_ack_time = AP_HAL::millis();
                        } else {
                            Debug("SBF Ack command (unexpected): %s\n", sbf_msg.data.bytes);
                        }
                    }
                } else {
                    // rejected command, send it out as a debug
                    Debug("SBF NACK Command: %s\n", sbf_msg.data.bytes);
                }
                // resume normal parsing
                sbf_msg.sbf_state = sbf_msg_parser_t::PREAMBLE1;
                break;
            }
            break;
    }

    return false;
}

void
AP_GPS_SBF::log_ExtEventPVTGeodetic(const msg4007 &temp)
{
    if (!should_df_log()) {
        return;
    }

    uint64_t now = AP_HAL::micros64();

    struct log_GPS_SBF_EVENT header = {
        LOG_PACKET_HEADER_INIT(LOG_GPS_SBF_EVENT_MSG),
        time_us:now,
        TOW:temp.TOW,
        WNc:temp.WNc,
        Mode:temp.Mode,
        Error:temp.Error,
        Latitude:ToDeg(temp.Latitude),
        Longitude:ToDeg(temp.Longitude),
        Height:temp.Height,
        Undulation:temp.Undulation,
        Vn:temp.Vn,
        Ve:temp.Ve,
        Vu:temp.Vu,
        COG:temp.COG
    };

    DataFlash_Class::instance()->WriteBlock(&header, sizeof(header));
}

bool
AP_GPS_SBF::process_message(void)
{
    uint16_t blockid = (sbf_msg.blockid & 8191u);

    Debug("BlockID %d", blockid);

    switch (blockid) {
    case ExtEventPVTGeodetic:
        log_ExtEventPVTGeodetic(sbf_msg.data.msg4007u);
        break;
    case PVTGeodetic:
    {
        const msg4007 &temp = sbf_msg.data.msg4007u;

        // Update time state
        if (temp.WNc != 65535) {
            state.time_week = temp.WNc;
            state.time_week_ms = (uint32_t)(temp.TOW);
        }

        state.last_gps_time_ms = AP_HAL::millis();

        // Update velocity state (don't use −2·10^10)
        if (temp.Vn > -200000) {
            state.velocity.x = (float)(temp.Vn);
            state.velocity.y = (float)(temp.Ve);
            state.velocity.z = (float)(-temp.Vu);

            state.have_vertical_velocity = true;

            float ground_vector_sq = state.velocity[0] * state.velocity[0] + state.velocity[1] * state.velocity[1];
            state.ground_speed = (float)safe_sqrt(ground_vector_sq);

            state.ground_course = wrap_360(degrees(atan2f(state.velocity[1], state.velocity[0])));
            state.rtk_age_ms = temp.MeanCorrAge * 10;

            // value is expressed as twice the rms error = int16 * 0.01/2
            state.horizontal_accuracy = (float)temp.HAccuracy * 0.005f;
            state.vertical_accuracy = (float)temp.VAccuracy * 0.005f;
            state.have_horizontal_accuracy = true;
            state.have_vertical_accuracy = true;
        }

        // Update position state (don't use −2·10^10)
        if (temp.Latitude > -200000) {
            state.location.lat = (int32_t)(temp.Latitude * RAD_TO_DEG_DOUBLE * (double)1e7);
            state.location.lng = (int32_t)(temp.Longitude * RAD_TO_DEG_DOUBLE * (double)1e7);
            state.location.alt = (int32_t)(((float)temp.Height - temp.Undulation) * 1e2f);
        }

        if (temp.NrSV != 255) {
            state.num_sats = temp.NrSV;
        }

        Debug("temp.Mode=0x%02x\n", (unsigned)temp.Mode);
        switch (temp.Mode & 15) {
            case 0: // no pvt
                state.status = AP_GPS::NO_FIX;
                break;
            case 1: // standalone
                state.status = AP_GPS::GPS_OK_FIX_3D;
                break;
            case 2: // dgps
                state.status = AP_GPS::GPS_OK_FIX_3D_DGPS;
                break;
            case 3: // fixed location
                state.status = AP_GPS::GPS_OK_FIX_3D;
                break;
            case 4: // rtk fixed
                state.status = AP_GPS::GPS_OK_FIX_3D_RTK_FIXED;
                break;
            case 5: // rtk float
                state.status = AP_GPS::GPS_OK_FIX_3D_RTK_FLOAT;
                break;
            case 6: // sbas
                state.status = AP_GPS::GPS_OK_FIX_3D_DGPS;
                break;
            case 7: // moving rtk fixed
                state.status = AP_GPS::GPS_OK_FIX_3D_RTK_FIXED;
                break;
            case 8: // moving rtk float
                state.status = AP_GPS::GPS_OK_FIX_3D_RTK_FLOAT;
                break;
        }
        
        if ((temp.Mode & 64) > 0) { // gps is in base mode
            state.status = AP_GPS::NO_FIX;
        } else if ((temp.Mode & 128) > 0) { // gps only has 2d fix
            state.status = AP_GPS::GPS_OK_FIX_2D;
        }
                    
        return true;
    }
    case DOP:
    {
        const msg4001 &temp = sbf_msg.data.msg4001u;

        state.hdop = temp.HDOP;
        state.vdop = temp.VDOP;
        break;
    }
    case ReceiverStatus:
    {
        const msg4014 &temp = sbf_msg.data.msg4014u;
        RxState = temp.RxState;
        RxError = temp.RxError;
        break;
    }
    case VelCovGeodetic:
    {
        const msg5908 &temp = sbf_msg.data.msg5908u;

        // select the maximum variance, as the EKF will apply it to all the columnds in it's estimate
        // FIXME: Support returning the covariance matric to the EKF
        float max_variance_squared = MAX(temp.Cov_VnVn, MAX(temp.Cov_VeVe, temp.Cov_VuVu));
        if (is_positive(max_variance_squared)) {
            state.have_speed_accuracy = true;
            state.speed_accuracy = sqrt(max_variance_squared);
        } else {
            state.have_speed_accuracy = false;
        }
        break;
    }
    }

    return false;
}

void AP_GPS_SBF::broadcast_configuration_failure_reason(void) const
{
    if (gps._raw_data) {
        if (!(RxState & SBF_DISK_MOUNTED)){
            gcs().send_text(MAV_SEVERITY_INFO, "GPS %d: SBF disk is not mounted", state.instance + 1);
        }
        else if (RxState & SBF_DISK_FULL) {
            gcs().send_text(MAV_SEVERITY_INFO, "GPS %d: SBF disk is full", state.instance + 1);
        }
        else if (!(RxState & SBF_DISK_ACTIVITY)) {
            gcs().send_text(MAV_SEVERITY_INFO, "GPS %d: SBF is not currently logging", state.instance + 1);
        }
    }

    if (gps._auto_config != AP_GPS::GPS_AUTO_CONFIG_DISABLE &&
        _init_blob_index < ARRAY_SIZE(_initialisation_blob)) {
        gcs().send_text(MAV_SEVERITY_INFO, "GPS %d: SBF is not fully configured (%d/%d)", state.instance + 1,
                        _init_blob_index, ARRAY_SIZE(_initialisation_blob));
    }
}

bool AP_GPS_SBF::is_configured (void) {
    return (!gps._raw_data || (RxState & SBF_DISK_ACTIVITY)) &&
            (gps._auto_config == AP_GPS::GPS_AUTO_CONFIG_DISABLE ||
             _init_blob_index >= ARRAY_SIZE(_initialisation_blob));
}

bool AP_GPS_SBF::is_healthy (void) const {
    return (RxError & RX_ERROR_MASK) == 0;
}
