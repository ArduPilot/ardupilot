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
#include <GCS_MAVLink/GCS.h>
#include <AP_InternalError/AP_InternalError.h>
#include <stdio.h>
#include <ctype.h>

#if AP_GPS_SBF_ENABLED
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

#ifndef GPS_SBF_STREAM_NUMBER
  #define GPS_SBF_STREAM_NUMBER 1
#endif

#define SBF_EXCESS_COMMAND_BYTES 5 // 2 start bytes + validity byte + space byte + endline byte

#define RX_ERROR_MASK (CONGESTION    | \
                       MISSEDEVENT   | \
                       CPUOVERLOAD   | \
                       INVALIDCONFIG | \
                       OUTOFGEOFENCE)

constexpr const char *AP_GPS_SBF::portIdentifiers[];
constexpr const char* AP_GPS_SBF::_initialisation_blob[];
constexpr const char* AP_GPS_SBF::sbas_on_blob[];

AP_GPS_SBF::AP_GPS_SBF(AP_GPS &_gps, AP_GPS::GPS_State &_state,
                       AP_HAL::UARTDriver *_port) :
    AP_GPS_Backend(_gps, _state, _port)
{
    sbf_msg.sbf_state = sbf_msg_parser_t::PREAMBLE1;

    _config_last_ack_time = AP_HAL::millis();

    // if we ever parse RTK observations it will always be of type NED, so set it once
    state.rtk_baseline_coords_type = RTK_BASELINE_COORDINATE_SYSTEM_NED;

    // yaw available when option bit set or using dual antenna
    if (option_set(AP_GPS::DriverOptions::SBF_UseBaseForYaw) ||
        (get_type() == AP_GPS::GPS_Type::GPS_TYPE_SBF_DUAL_ANTENNA)) {
        state.gps_yaw_configured = true;
    }
}

AP_GPS_SBF::~AP_GPS_SBF (void) {
    free(config_string);
}

// Process all bytes available from the stream
//
bool
AP_GPS_SBF::read(void)
{
    bool ret = false;
    uint32_t available_bytes = port->available();
    for (uint32_t i = 0; i < available_bytes; i++) {
        uint8_t temp = port->read();
#if AP_GPS_DEBUG_LOGGING_ENABLED
        log_data(&temp, 1);
#endif
        ret |= parse(temp);
    }

    const uint32_t now = AP_HAL::millis();
    if (gps._auto_config != AP_GPS::GPS_AUTO_CONFIG_DISABLE) {
        if (config_step != Config_State::Complete) {
            if (now > _init_blob_time) {
                if (now > _config_last_ack_time + 2000) {
                    const size_t port_enable_len = strlen(_port_enable);
                    if (port_enable_len <= port->txspace()) {
                        // try to enable input on the GPS port if we have not made progress on configuring it
                        Debug("SBF Sending port enable");
                        port->write((const uint8_t*)_port_enable, port_enable_len);
                        _config_last_ack_time = now;
                    }
                } else if (readyForCommand) {
                    if (config_string == nullptr) {
                        switch (config_step) {
                            case Config_State::Baud_Rate:
                                if (asprintf(&config_string, "scs,COM%d,baud%d,bits8,No,bit1,%s\n",
                                             (int)gps._com_port[state.instance],
                                             230400,
                                             port->get_flow_control() != AP_HAL::UARTDriver::flow_control::FLOW_CONTROL_ENABLE ? "none" : "RTS|CTS") == -1) {
                                    config_string = nullptr;
                                }
                                break;
                            case Config_State::SSO:
                                const char *extra_config;
                                switch (get_type()) {
                                    case AP_GPS::GPS_Type::GPS_TYPE_SBF_DUAL_ANTENNA:
                                        extra_config = "+AttCovEuler+AuxAntPositions";
                                        break;
                                    case AP_GPS::GPS_Type::GPS_TYPE_SBF:
                                    default:
                                        extra_config = "";
                                        break;
                                }
                                if (asprintf(&config_string, "sso,Stream%d,COM%d,PVTGeodetic+DOP+ReceiverStatus+VelCovGeodetic+BaseVectorGeod%s,msec100\n",
                                             (int)GPS_SBF_STREAM_NUMBER,
                                             (int)gps._com_port[state.instance],
                                             extra_config) == -1) {
                                    config_string = nullptr;
                                }
                                break;
                            case Config_State::Blob:
                                if (asprintf(&config_string, "%s\n", _initialisation_blob[_init_blob_index]) == -1) {
                                    config_string = nullptr;
                                }
                                break;
                            case Config_State::SBAS:
                                switch ((AP_GPS::SBAS_Mode)gps._sbas_mode) {
                                    case AP_GPS::SBAS_Mode::Disabled:
                                        if (asprintf(&config_string, "%s\n", sbas_off) == -1) {
                                            config_string = nullptr;
                                        }
                                        break;
                                    case AP_GPS::SBAS_Mode::Enabled:
                                        if (asprintf(&config_string, "%s\n", sbas_on_blob[_init_blob_index]) == -1) {
                                            config_string = nullptr;
                                        }
                                        break;
                                    case AP_GPS::SBAS_Mode::DoNotChange:
                                        config_string = nullptr;
                                        config_step = Config_State::Complete;
                                        break;
                                }
                                break;
                            case Config_State::SGA:
                            {
                                const char *targetGA = "none";
                                if (get_type() == AP_GPS::GPS_Type::GPS_TYPE_SBF_DUAL_ANTENNA) {
                                    targetGA = "MultiAntenna";
                                }
                                if (asprintf(&config_string, "sga, %s\n", targetGA)) {
                                  config_string = nullptr;
                                }
                                break;
                            }
                            case Config_State::Complete:
                              // should never reach here, why search for a config if we have fully configured already
                              INTERNAL_ERROR(AP_InternalError::error_t::flow_of_control);
                              break;
                        }
                    }

                    if (config_string != nullptr) {
                        const size_t config_length = strlen(config_string);
                        if (config_length <= port->txspace()) {
                            Debug("SBF sending init string: %s", config_string);
                            port->write((const uint8_t*)config_string, config_length);
                            readyForCommand = false;
                        }
                    }
                }
            }
        } else if (gps._raw_data == 2) { // only manage disarm/rearms when the user opts into it
            if (hal.util->get_soft_armed()) {
                _has_been_armed = true;
            } else if (_has_been_armed && (RxState & SBF_DISK_MOUNTED)) {
                // since init is done at this point and unmounting should be rate limited,
                // take over the _init_blob_time variable
                if (now > _init_blob_time) {
                    unmount_disk();
                    _init_blob_time = now + 1000;
                }
            }
        }
    }

    // yaw timeout after 300 milliseconds
    if ((now - state.gps_yaw_time_ms) > 300) {
        state.have_gps_yaw = false;
        state.have_gps_yaw_accuracy = false;
    }

    return ret;
}

bool AP_GPS_SBF::logging_healthy(void) const
{
    switch (gps._raw_data) {
        case 1:
        default:
            return (RxState & SBF_DISK_MOUNTED) && (RxState & SBF_DISK_ACTIVITY);
        case 2:
            return ((RxState & SBF_DISK_MOUNTED) && (RxState & SBF_DISK_ACTIVITY)) || (!hal.util->get_soft_armed() && _has_been_armed);
    }
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
            } else {
                // attempt to detect command prompt
                portIdentifier[portLength++] = (char)temp;
                bool foundPossiblePort = false;
                for (const char *portId : portIdentifiers) {
                    if (strncmp(portId, portIdentifier, MIN(portLength, 3)) == 0) {
                        // we found one of the COM/USB/IP related ports
                        if (portLength == 4) {
                            // validate that we have an ascii number
                            if (isdigit((char)temp)) {
                                foundPossiblePort = true;
                                break;
                            }
                        } else if (portLength >= sizeof(portIdentifier)) {
                            if ((char)temp == '>') {
                                readyForCommand = true;
                                Debug("SBF: Ready for command");
                            }
                        } else {
                            foundPossiblePort = true;
                        }
                        break;
                    }
                }
                if (!foundPossiblePort) {
                    portLength = 0;
                }
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
                    if (config_string != nullptr) {
                        if (!strncmp(config_string, (char *)(sbf_msg.data.bytes + 2),
                                     sbf_msg.read - SBF_EXCESS_COMMAND_BYTES)) {
                            Debug("SBF Ack Command: %s\n", sbf_msg.data.bytes);
                            free(config_string);
                            config_string = nullptr;
                            switch (config_step) {
                                case Config_State::Baud_Rate:
                                    config_step = Config_State::SSO;
                                    break;
                                case Config_State::SSO:
                                    config_step = Config_State::Blob;
                                    break;
                                case Config_State::Blob:
                                    _init_blob_index++;
                                    if (_init_blob_index >= ARRAY_SIZE(_initialisation_blob)) {
                                        config_step = Config_State::SBAS;
                                        _init_blob_index = 0;
                                    }
                                    break;
                                case Config_State::SBAS:
                                    _init_blob_index++;
                                    if ((gps._sbas_mode == AP_GPS::SBAS_Mode::Disabled)
                                        ||_init_blob_index >= ARRAY_SIZE(sbas_on_blob)) {
                                        config_step = Config_State::SGA;
                                    }
                                    break;
                                case Config_State::SGA:
                                    config_step = Config_State::Complete;
                                    break;
                                case Config_State::Complete:
                                    // should never reach here, this implies that we validated a config string when we hadn't sent any
                                    INTERNAL_ERROR(AP_InternalError::error_t::flow_of_control);
                                    break;
                            }
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

bool
AP_GPS_SBF::process_message(void)
{
    uint16_t blockid = (sbf_msg.blockid & 8191u);

    Debug("BlockID %d", blockid);

    switch (blockid) {
    case PVTGeodetic:
    {
        const msg4007 &temp = sbf_msg.data.msg4007u;

        // Update time state
        if (temp.WNc != 65535) {
            state.time_week = temp.WNc;
            state.time_week_ms = (uint32_t)(temp.TOW);
        }

        check_new_itow(temp.TOW, sbf_msg.length);
        state.last_gps_time_ms = AP_HAL::millis();

        // Update velocity state (don't use −2·10^10)
        if (temp.Vn > -200000) {
            state.velocity.x = (float)(temp.Vn);
            state.velocity.y = (float)(temp.Ve);
            state.velocity.z = (float)(-temp.Vu);

            state.have_vertical_velocity = true;

            velocity_to_speed_course(state);
            state.rtk_age_ms = temp.MeanCorrAge * 10;

            // value is expressed as twice the rms error = int16 * 0.01/2
            state.horizontal_accuracy = (float)temp.HAccuracy * 0.005f;
            state.vertical_accuracy = (float)temp.VAccuracy * 0.005f;
            state.have_horizontal_accuracy = true;
            state.have_vertical_accuracy = true;
        }

        // Update position state (don't use -2·10^10)
        if (temp.Latitude > -200000) {
            state.location.lat = (int32_t)(temp.Latitude * RAD_TO_DEG_DOUBLE * (double)1e7);
            state.location.lng = (int32_t)(temp.Longitude * RAD_TO_DEG_DOUBLE * (double)1e7);
            state.have_undulation = true;
            state.undulation = -temp.Undulation;
            set_alt_amsl_cm(state, ((float)temp.Height - temp.Undulation) * 1e2f);
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
        check_new_itow(temp.TOW, sbf_msg.length);

        state.hdop = temp.HDOP;
        state.vdop = temp.VDOP;
        break;
    }
    case ReceiverStatus:
    {
        const msg4014 &temp = sbf_msg.data.msg4014u;
        check_new_itow(temp.TOW, sbf_msg.length);
        RxState = temp.RxState;
        if ((RxError & RX_ERROR_MASK) != (temp.RxError & RX_ERROR_MASK)) {
            GCS_SEND_TEXT(MAV_SEVERITY_INFO, "GPS %u: SBF error changed (0x%08x/0x%08x)", (unsigned int)(state.instance + 1),
                            (unsigned int)(RxError & RX_ERROR_MASK), (unsigned int)(temp.RxError & RX_ERROR_MASK));
        }
        RxError = temp.RxError;
        break;
    }
    case VelCovGeodetic:
    {
        const msg5908 &temp = sbf_msg.data.msg5908u;

        check_new_itow(temp.TOW, sbf_msg.length);
        // select the maximum variance, as the EKF will apply it to all the columns in it's estimate
        // FIXME: Support returning the covariance matrix to the EKF
        float max_variance_squared = MAX(temp.Cov_VnVn, MAX(temp.Cov_VeVe, temp.Cov_VuVu));
        if (is_positive(max_variance_squared)) {
            state.have_speed_accuracy = true;
            state.speed_accuracy = sqrt(max_variance_squared);
        } else {
            state.have_speed_accuracy = false;
        }
        break;
    }
    case AttEulerCov:
    {
        // yaw accuracy is taken from this message even though we actually calculate the yaw ourself (see AuxAntPositions below)
        // this is OK based on the assumption that the calculation methods are similar and that inaccuracy arises from the sensor readings
        if (get_type() == AP_GPS::GPS_Type::GPS_TYPE_SBF_DUAL_ANTENNA) {
            const msg5939 &temp = sbf_msg.data.msg5939u;

            check_new_itow(temp.TOW, sbf_msg.length);

            constexpr double floatDNU = -2e-10f;
            constexpr uint8_t errorBits = 0x8F; // Bits 0-1 are aux 1 baseline
                                                // Bits 2-3 are aux 2 baseline
                                                // Bit 7 is attitude not requested
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wfloat-equal" // suppress -Wfloat-equal as it's false positive when testing for DNU values
            if (((temp.Error & errorBits) == 0)
                && (temp.Cov_HeadHead != floatDNU)) {
#pragma GCC diagnostic pop
                state.gps_yaw_accuracy = sqrtf(temp.Cov_HeadHead);
                state.have_gps_yaw_accuracy = true;
            } else {
                state.gps_yaw_accuracy = false;
            }
        }
        break;
    }
    case AuxAntPositions:
    {
#if GPS_MOVING_BASELINE
        if (get_type() == AP_GPS::GPS_Type::GPS_TYPE_SBF_DUAL_ANTENNA) {
            // calculate yaw using reported antenna positions in earth-frame
            // note that this calculation does not correct for the vehicle's roll and pitch meaning it is inaccurate at very high lean angles
            const msg5942 &temp = sbf_msg.data.msg5942u;
            check_new_itow(temp.TOW, sbf_msg.length);
            if (temp.N > 0 && temp.ant1.Error == 0 && temp.ant1.AmbiguityType == 0) {
                // valid RTK integer fix
                const float rel_heading_deg = degrees(atan2f(temp.ant1.DeltaEast, temp.ant1.DeltaNorth));
                calculate_moving_base_yaw(rel_heading_deg,
                                          Vector3f(temp.ant1.DeltaNorth, temp.ant1.DeltaEast, temp.ant1.DeltaUp).length(),
                                          -temp.ant1.DeltaUp);
            }
        }
#endif
        break;
    }
    case BaseVectorGeod:
    {
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wfloat-equal" // suppress -Wfloat-equal as it's false positive when testing for DNU values
        const msg4028 &temp = sbf_msg.data.msg4028u;

        // just breakout any consts we need for Do Not Use (DNU) reasons
        constexpr double doubleDNU = -2e-10;
        constexpr uint16_t uint16DNU = 65535;

        check_new_itow(temp.TOW, sbf_msg.length);

        if (temp.N == 0) { // no sub blocks so just bail, we can't do anything useful here
            state.rtk_num_sats = 0;
            state.rtk_age_ms = 0;
            state.rtk_baseline_y_mm = 0;
            state.rtk_baseline_x_mm = 0;
            state.rtk_baseline_z_mm = 0;
            break;
        }

        state.rtk_num_sats = temp.info.NrSV;

        state.rtk_age_ms = (temp.info.CorrAge != 65535) ? ((uint32_t)temp.info.CorrAge) * 10 : 0;

        // copy the position as long as the data isn't DNU, we require NED, and heading before accepting any of it
        if ((temp.info.DeltaEast != doubleDNU) && (temp.info.DeltaNorth != doubleDNU) && (temp.info.DeltaUp != doubleDNU) &&
            (temp.info.Azimuth != uint16DNU)) {

            state.rtk_baseline_y_mm = temp.info.DeltaEast * 1e3;
            state.rtk_baseline_x_mm = temp.info.DeltaNorth * 1e3;
            state.rtk_baseline_z_mm = temp.info.DeltaUp * -1e3;

#if GPS_MOVING_BASELINE
            // copy the baseline data as a yaw source
            if (option_set(AP_GPS::DriverOptions::SBF_UseBaseForYaw)) {
                calculate_moving_base_yaw(temp.info.Azimuth * 0.01f + 180.0f,
                                          Vector3f(temp.info.DeltaNorth, temp.info.DeltaEast, temp.info.DeltaUp).length(),
                                          -temp.info.DeltaUp);
            }
#endif // GPS_MOVING_BASELINE

        } else if (option_set(AP_GPS::DriverOptions::SBF_UseBaseForYaw)) {
            state.rtk_baseline_y_mm = 0;
            state.rtk_baseline_x_mm = 0;
            state.rtk_baseline_z_mm = 0;
            state.have_gps_yaw = false;
        }

#pragma GCC diagnostic pop
        break;
    }
    }

    return false;
}

void AP_GPS_SBF::broadcast_configuration_failure_reason(void) const
{
    if (gps._auto_config != AP_GPS::GPS_AUTO_CONFIG_DISABLE &&
        config_step != Config_State::Complete) {
        GCS_SEND_TEXT(MAV_SEVERITY_INFO, "GPS %u: SBF is not fully configured (%u/%u/%u/%u)",
                                         state.instance + 1,
                                         (unsigned)config_step,
                                         _init_blob_index,
                                         (unsigned)ARRAY_SIZE(_initialisation_blob),
                                         (unsigned)ARRAY_SIZE(sbas_on_blob));
    }
}

bool AP_GPS_SBF::is_configured (void) const {
    return ((gps._auto_config == AP_GPS::GPS_AUTO_CONFIG_DISABLE) ||
            (config_step == Config_State::Complete));
}

bool AP_GPS_SBF::is_healthy (void) const {
    return (RxError & RX_ERROR_MASK) == 0;
}

void AP_GPS_SBF::mount_disk (void) const {
    const char* command = "emd, DSK1, Mount\n";
    Debug("Mounting disk");
    port->write((const uint8_t*)command, strlen(command));
}

void AP_GPS_SBF::unmount_disk (void) const {
    const char* command = "emd, DSK1, Unmount\n";
    GCS_SEND_TEXT(MAV_SEVERITY_DEBUG, "SBF unmounting disk");
    port->write((const uint8_t*)command, strlen(command));
}

bool AP_GPS_SBF::prepare_for_arming(void) {
    bool is_logging = true; // assume that its logging until proven otherwise
    if (gps._raw_data) {
        if (!(RxState & SBF_DISK_MOUNTED)){
            is_logging = false;
            GCS_SEND_TEXT(MAV_SEVERITY_INFO, "GPS %d: SBF disk is not mounted", state.instance + 1);

            // simply attempt to mount the disk, no need to check if the command was
            // ACK/NACK'd as we don't continuously attempt to remount the disk
            GCS_SEND_TEXT(MAV_SEVERITY_INFO, "GPS %d: Attempting to mount disk", state.instance + 1);
            mount_disk();
            // reset the flag to indicate if we should be logging
            _has_been_armed = false;
        }
        else if (RxState & SBF_DISK_FULL) {
            is_logging = false;
            GCS_SEND_TEXT(MAV_SEVERITY_INFO, "GPS %d: SBF disk is full", state.instance + 1);
        }
        else if (!(RxState & SBF_DISK_ACTIVITY)) {
            is_logging = false;
            GCS_SEND_TEXT(MAV_SEVERITY_INFO, "GPS %d: SBF is not currently logging", state.instance + 1);
        }
    }

    return is_logging;
}
#endif
