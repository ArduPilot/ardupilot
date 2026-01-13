/*
   Copyright (C) 2021  Kraus Hamdani Aerospace Inc. All rights reserved.

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

   Author: Tom Pittenger
 */

#include "AP_ADSB_uAvionix_UCP.h"

// This driver implements the UCP protocol from uAvionix which is a variant of the GDL90 protocol by Garmin
// https://uavionix.com/downloads/ping200X/uAvionix-UCP-Transponder-ICD-Rev-Q.pdf

#if HAL_ADSB_UCP_ENABLED

#include <AP_SerialManager/AP_SerialManager.h>
#include <GCS_MAVLink/GCS.h>
#include <AP_HAL/AP_HAL.h>
#include <AP_Math/AP_Math.h>
#include <AP_Math/crc.h>
#include <ctype.h>
#include <AP_Notify/AP_Notify.h>

#include <AP_GPS/AP_GPS.h>

extern const AP_HAL::HAL &hal;

#define AP_ADSB_UAVIONIX_HEALTH_TIMEOUT_MS                     (5000UL)

#define AP_ADSB_UAVIONIX_GCS_LOST_COMMS_LONG_TIMEOUT_MINUTES    (15UL)
#define AP_ADSB_UAVIONIX_GCS_LOST_COMMS_LONG_TIMEOUT_MS         (1000UL * 60UL * AP_ADSB_UAVIONIX_GCS_LOST_COMMS_LONG_TIMEOUT_MINUTES)

#define AP_ADSB_UAVIONIX_DETECT_GROUNDSTATE                     0
#define AP_ADSB_UAVIONIX_EMERGENCY_STATUS_ON_LOST_LINK          0
#define AP_ADSB_UAVIONIX_UCP_SET_CONFIG_RETIRES                 5
#define AP_ADSB_UAVIONIX_UCP_SET_CONFIG_DEBUG                   0

// detect if any port is configured as uAvionix_UCP
bool AP_ADSB_uAvionix_UCP::detect()
{
    return AP::serialmanager().have_serial(AP_SerialManager::SerialProtocol_ADSB, 0);
}


// Init, called once after class is constructed
bool AP_ADSB_uAvionix_UCP::init()
{
    _port = AP::serialmanager().find_serial(AP_SerialManager::SerialProtocol_ADSB, 0);
    if (_port == nullptr) {
        return false;
    }

    _frontend.out_state.ctrl.squawkCode = 1200;
    _frontend.out_state.tx_status.squawk = 1200;
    _frontend.out_state.tx_status.fault |= UAVIONIX_ADSB_OUT_STATUS_FAULT_STATUS_MESSAGE_UNAVAIL;

    return true;
}


void AP_ADSB_uAvionix_UCP::update()
{
    if (_port == nullptr) {
        return;
    }

    const uint32_t now_ms = AP_HAL::millis();

    // -----------------------------
    // read any available data on serial port
    // -----------------------------
    uint32_t nbytes = MIN(_port->available(), 10UL * GDL90_RX_MAX_PACKET_LENGTH);
    while (nbytes-- > 0) {
        uint8_t data;
        if (!_port->read(data)) {
            break;
        }
        if (parseByte(data, rx.msg, rx.status)) {
            rx.last_msg_ms = now_ms;
            handle_msg(rx.msg);
        }
    } // while nbytes


    if (run_state.last_packet_Transponder_Id_ms == 0 && run_state.request_Transponder_Id_tries < 5) {
        if (now_ms - run_state.last_packet_Request_Transponder_Id_ms >= 1000)
        {
            request_msg(GDL90_ID_IDENTIFICATION);
            run_state.request_Transponder_Id_tries++;
        }
    }

#if HAL_ADSB_UCP_SET_CONFIG
    if (transponder_config_set.icao_backup != _frontend.out_state.cfg.ICAO_id_param.get()) {
        // allow for in-flight changes to the ICAO ID
        transponder_config_set.icao_backup = _frontend.out_state.cfg.ICAO_id_param.get();
        run_state.request_Transponder_Config_tries = 0;
    }
    if ((run_state.request_Transponder_Config_tries < AP_ADSB_UAVIONIX_UCP_SET_CONFIG_RETIRES) &&
        (now_ms > 20000 || (_frontend.out_state.cfg.maxAircraftSpeed_knots > 0)) && // this boot delay is needed to give time for set_max_speed() and set_stall_speed_cm() to get populated before we try to set the config or else it will detect it as different and re-set the config
        (now_ms - transponder_config_set.last_send_ms >= 1000)) {
        transponder_config_set.last_send_ms = now_ms;
        update_Transponder_Config();
    }
#else
    if (run_state.last_packet_Transponder_Config_ms == 0 && run_state.request_Transponder_Config_tries < 5) {
        if (now_ms - run_state.last_packet_Request_Transponder_Config_ms >= 1000)
        {
            request_msg(GDL90_ID_TRANSPONDER_CONFIG);
            run_state.request_Transponder_Config_tries++;
        }
    }
#endif

   if (now_ms - run_state.last_packet_Transponder_Control_ms >= 1000) {
        run_state.last_packet_Transponder_Control_ms = now_ms;

        // We want to use the defaults stored on the ping200X, if possible.
        // Until we get the config message (or we've tried requesting it several times),
        // don't send the control message.
        if (run_state.last_packet_Transponder_Config_ms != 0 || run_state.request_Transponder_Config_tries >= 5) {
            send_Transponder_Control();
        }
    }

    if ((now_ms - run_state.last_packet_GPS_ms >= 200) && (_frontend._options & uint32_t(AP_ADSB::AdsbOption::Ping200X_Send_GPS)) != 0) {
        run_state.last_packet_GPS_ms = now_ms;
        send_GPS_Data();
    }

    // if the transponder has stopped giving us the data needed to
    // fill the transponder status mavlink message, indicate status unavailable.
    if ((now_ms - run_state.last_packet_Transponder_Status_ms >= 10000)
        && (now_ms - run_state.last_packet_Transponder_Heartbeat_ms >= 10000)
        && (now_ms - run_state.last_packet_Transponder_Ownship_ms >= 10000)) {
        _frontend.out_state.tx_status.fault |= UAVIONIX_ADSB_OUT_STATUS_FAULT_STATUS_MESSAGE_UNAVAIL;
    }
}


void AP_ADSB_uAvionix_UCP::handle_msg(const GDL90_RX_MESSAGE &msg)
{
    switch(msg.messageId) {
    case GDL90_ID_HEARTBEAT: {
        // The Heartbeat message provides real-time indications of the status and operation of the
        // transponder. The message will be transmitted with a period of one second for the UCP
        // protocol.
        memcpy(&rx.decoded.heartbeat, msg.raw, sizeof(rx.decoded.heartbeat));
        run_state.last_packet_Transponder_Heartbeat_ms = AP_HAL::millis();
        _frontend.out_state.tx_status.fault &= ~UAVIONIX_ADSB_OUT_STATUS_FAULT_STATUS_MESSAGE_UNAVAIL;

        if (rx.decoded.heartbeat.status.one.maintenanceRequired) {
            _frontend.out_state.tx_status.fault |= UAVIONIX_ADSB_OUT_STATUS_FAULT_MAINT_REQ;
        } else {
            _frontend.out_state.tx_status.fault &= ~UAVIONIX_ADSB_OUT_STATUS_FAULT_MAINT_REQ;
        }

        if (rx.decoded.heartbeat.status.two.functionFailureGnssUnavailable) {
            _frontend.out_state.tx_status.fault |= UAVIONIX_ADSB_OUT_STATUS_FAULT_GPS_UNAVAIL;
        } else {
            _frontend.out_state.tx_status.fault &= ~UAVIONIX_ADSB_OUT_STATUS_FAULT_GPS_UNAVAIL;
        }

        if (rx.decoded.heartbeat.status.two.functionFailureGnssNo3dFix) {
            _frontend.out_state.tx_status.fault |= UAVIONIX_ADSB_OUT_STATUS_FAULT_GPS_NO_POS;
        } else {
            _frontend.out_state.tx_status.fault &= ~UAVIONIX_ADSB_OUT_STATUS_FAULT_GPS_NO_POS;
        }

        if (rx.decoded.heartbeat.status.two.functionFailureTransmitSystem) {
            _frontend.out_state.tx_status.fault |= UAVIONIX_ADSB_OUT_STATUS_FAULT_TX_SYSTEM_FAIL;
        } else {
            _frontend.out_state.tx_status.fault &= ~UAVIONIX_ADSB_OUT_STATUS_FAULT_TX_SYSTEM_FAIL;
        }
        }
        break;

    case GDL90_ID_IDENTIFICATION:
        run_state.last_packet_Transponder_Id_ms = AP_HAL::millis();
        // The Identification message contains information used to identify the connected device. The
        // Identification message will be transmitted with a period of one second regardless of data status
        // or update for the UCP protocol and will be transmitted upon request for the UCP-HD protocol.
        if (memcmp(&rx.decoded.identification, msg.raw, sizeof(rx.decoded.identification)) != 0) {
            memcpy(&rx.decoded.identification, msg.raw, sizeof(rx.decoded.identification));

            // Firmware Part Number (not null terminated, but null padded if part number is less than 15 characters).
            // Copy into a temporary string that is 1 char longer so we ensure it's null terminated
            const uint8_t str_len = sizeof(rx.decoded.identification.primaryFwPartNumber);
            char primaryFwPartNumber[str_len+1];
            memcpy(&primaryFwPartNumber, rx.decoded.identification.primaryFwPartNumber, str_len);
            primaryFwPartNumber[str_len] = 0;

            GCS_SEND_TEXT(MAV_SEVERITY_DEBUG,"ADSB:Detected %s v%u.%u.%u SN:%u %s",
                get_hardware_name(rx.decoded.identification.primary.hwId),
                (unsigned)rx.decoded.identification.primary.fwMajorVersion,
                (unsigned)rx.decoded.identification.primary.fwMinorVersion,
                (unsigned)rx.decoded.identification.primary.fwBuildVersion,
                (unsigned)rx.decoded.identification.primary.serialNumber,
                primaryFwPartNumber);

#if HAL_ADSB_UCP_SET_CONFIG
            run_state.request_Transponder_Config_tries = 0;
#endif
        }
        break;

    case GDL90_ID_TRANSPONDER_CONFIG:
        run_state.last_packet_Transponder_Config_ms = AP_HAL::millis();
        memcpy(&rx.decoded.transponder_config, msg.raw, sizeof(rx.decoded.transponder_config));
        break;

#if AP_ADSB_UAVIONIX_UCP_CAPTURE_ALL_RX_PACKETS
    case GDL90_ID_OWNSHIP_REPORT:
        _frontend.out_state.tx_status.fault &= ~UAVIONIX_ADSB_OUT_STATUS_FAULT_STATUS_MESSAGE_UNAVAIL;
        // The Ownship message contains information on the GNSS position. If the Ownship GNSS
        // position fix is invalid, the Latitude, Longitude, and NIC fields will all have the ZERO value. The
        // Ownship message will be transmitted with a period of one second regardless of data status or
        // update for the UCP protocol. All fields in the ownship message are transmitted MSB first
        memcpy(&rx.decoded.ownship_report, msg.raw, sizeof(rx.decoded.ownship_report));
        run_state.last_packet_Transponder_Ownship_ms = AP_HAL::millis();
        _frontend.out_state.tx_status.NIC_NACp = rx.decoded.ownship_report.report.NIC | (rx.decoded.ownship_report.report.NACp << 4);
        memcpy(_frontend.out_state.tx_status.flight_id, rx.decoded.ownship_report.report.callsign, sizeof(_frontend.out_state.tx_status.flight_id));
        break;

    case GDL90_ID_OWNSHIP_GEOMETRIC_ALTITUDE:
        // An Ownship Geometric Altitude message will be transmitted with a period of one second when
        // the GNSS fix is valid for the UCP protocol. All fields in the Geometric Ownship Altitude
        // message are transmitted MSB first.
        memcpy(&rx.decoded.ownship_geometric_altitude, msg.raw, sizeof(rx.decoded.ownship_geometric_altitude));
        break;

    case GDL90_ID_SENSOR_MESSAGE:
        memcpy(&rx.decoded.sensor_message, msg.raw, sizeof(rx.decoded.sensor_message));
        break;

    case GDL90_ID_TRANSPONDER_STATUS:
    {
        _frontend.out_state.tx_status.fault &= ~UAVIONIX_ADSB_OUT_STATUS_FAULT_STATUS_MESSAGE_UNAVAIL;
        switch (msg.payload[0]) {
        case 1: {
            // version 1 of the transponder status message is sent at 1 Hz (if UCP protocol out is enabled on the transponder)
            memcpy(&rx.decoded.transponder_status, msg.raw, sizeof(rx.decoded.transponder_status));
            if (rx.decoded.transponder_status.identActive) {
                _frontend.out_state.tx_status.state |= UAVIONIX_ADSB_OUT_STATUS_STATE_IDENT_ACTIVE;
            } else {
                _frontend.out_state.tx_status.state &= ~UAVIONIX_ADSB_OUT_STATUS_STATE_IDENT_ACTIVE;
            }

            if (rx.decoded.transponder_status.modeAEnabled) {
                _frontend.out_state.tx_status.state |= UAVIONIX_ADSB_OUT_STATUS_STATE_MODE_A_ENABLED;
            } else {
                _frontend.out_state.tx_status.state &= ~UAVIONIX_ADSB_OUT_STATUS_STATE_MODE_A_ENABLED;
            }

            if (rx.decoded.transponder_status.modeCEnabled) {
                _frontend.out_state.tx_status.state |= UAVIONIX_ADSB_OUT_STATUS_STATE_MODE_C_ENABLED;
            } else {
                _frontend.out_state.tx_status.state &= ~UAVIONIX_ADSB_OUT_STATUS_STATE_MODE_C_ENABLED;
            }

            if (rx.decoded.transponder_status.modeSEnabled) {
                _frontend.out_state.tx_status.state |= UAVIONIX_ADSB_OUT_STATUS_STATE_MODE_S_ENABLED;
            } else {
                _frontend.out_state.tx_status.state &= ~UAVIONIX_ADSB_OUT_STATUS_STATE_MODE_S_ENABLED;
            }

            if (rx.decoded.transponder_status.es1090TxEnabled) {
                _frontend.out_state.tx_status.state |= UAVIONIX_ADSB_OUT_STATUS_STATE_1090ES_TX_ENABLED;
            } else {
                _frontend.out_state.tx_status.state &= ~UAVIONIX_ADSB_OUT_STATUS_STATE_1090ES_TX_ENABLED;
            }

            if (rx.decoded.transponder_status.x_bit) {
                _frontend.out_state.tx_status.state |= UAVIONIX_ADSB_OUT_STATUS_STATE_XBIT_ENABLED;
            } else {
                _frontend.out_state.tx_status.state &= ~UAVIONIX_ADSB_OUT_STATUS_STATE_XBIT_ENABLED;
            }

            _frontend.out_state.tx_status.squawk = rx.decoded.transponder_status.squawkCode;

#if !HAL_ADSB_UCP_SET_CONFIG
            if (run_state.last_packet_Transponder_Status_ms == 0 && run_state.last_packet_Transponder_Config_ms == 0) {
                // If this is the first time we've seen a status message,
                // and we haven't initialized the control message from the config message,
                // set initial control message contents to match transponder's current behavior.
                _frontend.out_state.ctrl.modeAEnabled = rx.decoded.transponder_status.modeAEnabled;
                _frontend.out_state.ctrl.modeCEnabled = rx.decoded.transponder_status.modeCEnabled;
                _frontend.out_state.ctrl.modeSEnabled = rx.decoded.transponder_status.modeSEnabled;
                _frontend.out_state.ctrl.es1090TxEnabled = rx.decoded.transponder_status.es1090TxEnabled;
                _frontend.out_state.ctrl.squawkCode = rx.decoded.transponder_status.squawkCode;
                _frontend.out_state.ctrl.x_bit = rx.decoded.transponder_status.x_bit;
            }
#endif
            run_state.last_packet_Transponder_Status_ms = AP_HAL::millis();
#if AP_MAVLINK_MSG_UAVIONIX_ADSB_OUT_STATUS_ENABLED
            GCS_SEND_MESSAGE(MSG_UAVIONIX_ADSB_OUT_STATUS);
            run_state.last_gcs_send_message_Transponder_Status_ms = AP_HAL::millis();
#endif
            break;
        }
        case 2:
            // deprecated
            break;
        case 3: {
            // Version 3 of the transponder status message is sent in response to the transponder control message (if UCP-HD protocol out is enabled on the transponder)
            memcpy(&rx.decoded.transponder_status_v3, msg.raw, sizeof(rx.decoded.transponder_status_v3));

            if (rx.decoded.transponder_status_v3.indicatingOnGround) {
                _frontend.out_state.tx_status.state |= UAVIONIX_ADSB_OUT_STATUS_STATE_ON_GROUND;
            } else {
                _frontend.out_state.tx_status.state &= ~UAVIONIX_ADSB_OUT_STATUS_STATE_ON_GROUND;
            }

            if (rx.decoded.transponder_status_v3.fault) {
                // unsure what fault is indicated, query heartbeat for more info
                request_msg(GDL90_ID_HEARTBEAT);
            }

            if (rx.decoded.transponder_status_v3.identActive) {
                _frontend.out_state.tx_status.state |= UAVIONIX_ADSB_OUT_STATUS_STATE_IDENT_ACTIVE;
            } else {
                _frontend.out_state.tx_status.state &= ~UAVIONIX_ADSB_OUT_STATUS_STATE_IDENT_ACTIVE;
            }

            if (rx.decoded.transponder_status_v3.modeAEnabled) {
                _frontend.out_state.tx_status.state |= UAVIONIX_ADSB_OUT_STATUS_STATE_MODE_A_ENABLED;
            } else {
                _frontend.out_state.tx_status.state &= ~UAVIONIX_ADSB_OUT_STATUS_STATE_MODE_A_ENABLED;
            }

            if (rx.decoded.transponder_status_v3.modeCEnabled) {
                _frontend.out_state.tx_status.state |= UAVIONIX_ADSB_OUT_STATUS_STATE_MODE_C_ENABLED;
            } else {
                _frontend.out_state.tx_status.state &= ~UAVIONIX_ADSB_OUT_STATUS_STATE_MODE_C_ENABLED;
            }

            if (rx.decoded.transponder_status_v3.modeSEnabled) {
                _frontend.out_state.tx_status.state |= UAVIONIX_ADSB_OUT_STATUS_STATE_MODE_S_ENABLED;
            } else {
                _frontend.out_state.tx_status.state &= ~UAVIONIX_ADSB_OUT_STATUS_STATE_MODE_S_ENABLED;
            }

            if (rx.decoded.transponder_status_v3.es1090TxEnabled) {
                _frontend.out_state.tx_status.state |= UAVIONIX_ADSB_OUT_STATUS_STATE_1090ES_TX_ENABLED;
            } else {
                _frontend.out_state.tx_status.state &= ~UAVIONIX_ADSB_OUT_STATUS_STATE_1090ES_TX_ENABLED;
            }

            _frontend.out_state.tx_status.squawk = rx.decoded.transponder_status_v3.squawkCode;
            _frontend.out_state.tx_status.NIC_NACp = rx.decoded.transponder_status_v3.NIC | (rx.decoded.transponder_status_v3.NACp << 4);
            _frontend.out_state.tx_status.boardTemp = rx.decoded.transponder_status_v3.temperature;

            if (run_state.last_packet_Transponder_Status_ms == 0 && run_state.last_packet_Transponder_Config_ms == 0) {
                // If this is the first time we've seen a status message,
                // and we haven't initialized the control message from the config message,
                // set initial control message contents to match transponder's current behavior.
                _frontend.out_state.ctrl.modeAEnabled = rx.decoded.transponder_status_v3.modeAEnabled;
                _frontend.out_state.ctrl.modeCEnabled = rx.decoded.transponder_status_v3.modeCEnabled;
                _frontend.out_state.ctrl.modeSEnabled = rx.decoded.transponder_status_v3.modeSEnabled;
                _frontend.out_state.ctrl.es1090TxEnabled = rx.decoded.transponder_status_v3.es1090TxEnabled;
                _frontend.out_state.ctrl.squawkCode = rx.decoded.transponder_status_v3.squawkCode;
            }
            run_state.last_packet_Transponder_Status_ms = AP_HAL::millis();
#if AP_MAVLINK_MSG_UAVIONIX_ADSB_OUT_STATUS_ENABLED
            GCS_SEND_MESSAGE(MSG_UAVIONIX_ADSB_OUT_STATUS);
            run_state.last_gcs_send_message_Transponder_Status_ms = AP_HAL::millis();
#endif
            break;
        }
        default:
            break;
        }
        break;
    }
#endif // AP_ADSB_UAVIONIX_UCP_CAPTURE_ALL_RX_PACKETS

    case GDL90_ID_TRANSPONDER_CONTROL:
    case GDL90_ID_GPS_DATA:
    case GDL90_ID_MESSAGE_REQUEST:
        // not handled, outbound only
        break;
    default:
        //GCS_SEND_TEXT(MAV_SEVERITY_DEBUG,"ADSB:Unknown msg %d", (int)msg.messageId);
        break;
    }
}


const char* AP_ADSB_uAvionix_UCP::get_hardware_name(const uint8_t hwId)
{
    switch(hwId) {
        case 0x09: return "Ping200s";
        case 0x0A: return "Ping20s";
        case 0x18: return "Ping200C";
        case 0x27: return "Ping20Z";
        case 0x2D: return "SkyBeaconX";             // (certified)
        case 0x26: //return "Ping200Z/Ping200X";    // (uncertified). Let's fallthrough and use Ping200X
        case 0x2F: return "Ping200X";               // (certified)
        case 0x30: return "TailBeaconX";            // (certified)
    } // switch hwId
    return "Unknown HW";
}

#if HAL_ADSB_UCP_SET_CONFIG
// get the transponder config amd compare it with what we want it to be and set it if it's different
void AP_ADSB_uAvionix_UCP::update_Transponder_Config()
{
    run_state.request_Transponder_Config_tries++;
    if (run_state.request_Transponder_Config_tries >= AP_ADSB_UAVIONIX_UCP_SET_CONFIG_RETIRES) {
        // we've retried too many times, stop trying
        GCS_SEND_TEXT(MAV_SEVERITY_CRITICAL, "ADSB: %s config timeout", get_hardware_name(rx.decoded.identification.primary.hwId));
        return;
    }

    const GDL90_TRANSPONDER_CONFIG_MSG_V4_V5 desired_config_msg = populate_Transponder_Config();

    if (rx.decoded.transponder_config.messageId == 0 || !compare_Transponder_Config(rx.decoded.transponder_config, desired_config_msg)) {
        // we've either never received a config so we blindly send in case we only have 1-way comms or it's wrong and we're trying to set it.
        gdl90Transmit((GDL90_TX_MESSAGE&)desired_config_msg, sizeof(desired_config_msg));
        request_msg(GDL90_ID_TRANSPONDER_CONFIG);

    } else {
        run_state.request_Transponder_Config_tries = AP_ADSB_UAVIONIX_UCP_SET_CONFIG_RETIRES; // no more retires, we're done!
        GCS_SEND_TEXT(MAV_SEVERITY_DEBUG,"ADSB: %s Ready", get_hardware_name(rx.decoded.identification.primary.hwId));
    }
}

bool AP_ADSB_uAvionix_UCP::compare_Transponder_Config(const GDL90_TRANSPONDER_CONFIG_MSG_V4_V5 &msg1, const GDL90_TRANSPONDER_CONFIG_MSG_V4_V5 &msg2)
{
#if AP_ADSB_UAVIONIX_UCP_SET_CONFIG_DEBUG
    if (msg1.version           != msg2.version)
        GCS_SEND_TEXT(MAV_SEVERITY_CRITICAL,"ADSB: mismatch version: %d, %d", (int)msg1.version, (int)msg2.version);
    if (msg1.icaoAddress[0]    != msg2.icaoAddress[0])
        GCS_SEND_TEXT(MAV_SEVERITY_CRITICAL,"ADSB: mismatch icaoAddress[0]: %d, %d", (int)msg1.icaoAddress[0], (int)msg2.icaoAddress[0]);
    if (msg1.icaoAddress[1]    != msg2.icaoAddress[1])
        GCS_SEND_TEXT(MAV_SEVERITY_CRITICAL,"ADSB: mismatch icaoAddress[1]: %d, %d", (int)msg1.icaoAddress[1], (int)msg2.icaoAddress[1]);
    if (msg1.icaoAddress[2]    != msg2.icaoAddress[2])
        GCS_SEND_TEXT(MAV_SEVERITY_CRITICAL,"ADSB: mismatch icaoAddress[2]: %d, %d", (int)msg1.icaoAddress[2], (int)msg2.icaoAddress[2]);
    if (msg1.maxSpeed          != msg2.maxSpeed)
        GCS_SEND_TEXT(MAV_SEVERITY_CRITICAL,"ADSB: mismatch maxSpeed: %d, %d", (int)msg1.maxSpeed, (int)msg2.maxSpeed);
    if (msg1.baroAltSource     != msg2.baroAltSource)
        GCS_SEND_TEXT(MAV_SEVERITY_CRITICAL,"ADSB: mismatch baroAltSource: %d, %d", (int)msg1.baroAltSource, (int)msg2.baroAltSource);
    if (msg1.SDA               != msg2.SDA)
        GCS_SEND_TEXT(MAV_SEVERITY_CRITICAL,"ADSB: mismatch SDA: %d, %d", (int)msg1.SDA, (int)msg2.SDA);
    if (msg1.SIL               != msg2.SIL)
        GCS_SEND_TEXT(MAV_SEVERITY_CRITICAL,"ADSB: mismatch SIL: %d, %d", (int)msg1.SIL, (int)msg2.SIL);
    if (msg1.lengthWidth       != msg2.lengthWidth)
        GCS_SEND_TEXT(MAV_SEVERITY_CRITICAL,"ADSB: mismatch lengthWidth: %d, %d", (int)msg1.lengthWidth, (int)msg2.lengthWidth);
    if (msg1.es1090InCapable   != msg2.es1090InCapable)
        GCS_SEND_TEXT(MAV_SEVERITY_CRITICAL,"ADSB: mismatch es1090InCapable: %d, %d", (int)msg1.es1090InCapable, (int)msg2.es1090InCapable);
    if (msg1.uatInCapable      != msg2.uatInCapable)
        GCS_SEND_TEXT(MAV_SEVERITY_CRITICAL,"ADSB: mismatch uatInCapable: %d, %d", (int)msg1.uatInCapable, (int)msg2.uatInCapable);
    if (msg1.longitudinalOffset!= msg2.longitudinalOffset)
        GCS_SEND_TEXT(MAV_SEVERITY_CRITICAL,"ADSB: mismatch longitudinalOffset: %d, %d", (int)msg1.longitudinalOffset, (int)msg2.longitudinalOffset);
    if (msg1.lateralOffset     != msg2.lateralOffset)
        GCS_SEND_TEXT(MAV_SEVERITY_CRITICAL,"ADSB: mismatch lateralOffset: %d, %d", (int)msg1.lateralOffset, (int)msg2.lateralOffset);
    if (msg1.stallSpeed_cmps   != msg2.stallSpeed_cmps)
        GCS_SEND_TEXT(MAV_SEVERITY_CRITICAL,"ADSB: mismatch stallSpeed_cmps: %d, %d", (int)msg1.stallSpeed_cmps, (int)msg2.stallSpeed_cmps);
    if (msg1.emitterType       != msg2.emitterType)
        GCS_SEND_TEXT(MAV_SEVERITY_CRITICAL,"ADSB: mismatch emitterType: %d, %d", (int)msg1.emitterType, (int)msg2.emitterType);
    if (msg1.baudRate          != msg2.baudRate)
        GCS_SEND_TEXT(MAV_SEVERITY_CRITICAL,"ADSB: mismatch baudRate: %d, %d", (int)msg1.baudRate, (int)msg2.baudRate);
    if (msg1.modeAEnabled      != msg2.modeAEnabled)
        GCS_SEND_TEXT(MAV_SEVERITY_CRITICAL,"ADSB: mismatch modeAEnabled: %d, %d", (int)msg1.modeAEnabled, (int)msg2.modeAEnabled);
    if (msg1.modeCEnabled      != msg2.modeCEnabled)
        GCS_SEND_TEXT(MAV_SEVERITY_CRITICAL,"ADSB: mismatch modeCEnabled: %d, %d", (int)msg1.modeCEnabled, (int)msg2.modeCEnabled);
    if (msg1.modeSEnabled      != msg2.modeSEnabled)
        GCS_SEND_TEXT(MAV_SEVERITY_CRITICAL,"ADSB: mismatch modeSEnabled: %d, %d", (int)msg1.modeSEnabled, (int)msg2.modeSEnabled);
    if (msg1.es1090TxEnabled   != msg2.es1090TxEnabled)
        GCS_SEND_TEXT(MAV_SEVERITY_CRITICAL,"ADSB: mismatch es1090TxEnabled: %d, %d", (int)msg1.es1090TxEnabled, (int)msg2.es1090TxEnabled);
    if (msg1.defaultSquawk     != msg2.defaultSquawk)
        GCS_SEND_TEXT(MAV_SEVERITY_CRITICAL,"ADSB: mismatch defaultSquawk: %d, %d", (int)msg1.defaultSquawk, (int)msg2.defaultSquawk);
    if (msg1.baro100           != msg2.baro100)
        GCS_SEND_TEXT(MAV_SEVERITY_CRITICAL,"ADSB: mismatch baro100: %d, %d", (int)msg1.baro100, (int)msg2.baro100);
#endif // AP_ADSB_UAVIONIX_UCP_SET_CONFIG_DEBUG

    // Don't compare the whole struct because we only care about certain values in it

    if (msg1.version           != msg2.version ||
        msg1.icaoAddress[0]    != msg2.icaoAddress[0] ||
        msg1.icaoAddress[1]    != msg2.icaoAddress[1] ||
        msg1.icaoAddress[2]    != msg2.icaoAddress[2] ||
        msg1.maxSpeed          != msg2.maxSpeed ||
        msg1.baroAltSource     != msg2.baroAltSource ||
        // msg1.SDA               != msg2.SDA ||        // These could be fixed in HW from the factory
        // msg1.SIL               != msg2.SIL ||        // These could be fixed in HW from the factory
        msg1.lengthWidth       != msg2.lengthWidth ||
        msg1.es1090InCapable   != msg2.es1090InCapable ||
        msg1.uatInCapable      != msg2.uatInCapable ||
        msg1.longitudinalOffset!= msg2.longitudinalOffset ||
        msg1.lateralOffset     != msg2.lateralOffset ||
        msg1.stallSpeed_cmps   != msg2.stallSpeed_cmps ||
        msg1.emitterType       != msg2.emitterType ||
        msg1.baudRate          != msg2.baudRate ||
        msg1.modeAEnabled      != msg2.modeAEnabled ||
        msg1.modeCEnabled      != msg2.modeCEnabled ||
        msg1.modeSEnabled      != msg2.modeSEnabled ||
        msg1.es1090TxEnabled   != msg2.es1090TxEnabled ||
        msg1.defaultSquawk     != msg2.defaultSquawk ||
        msg1.baro100           != msg2.baro100)
    {
        return false;
    }

    if (memcmp(msg1.registration, msg2.registration, sizeof(msg2.registration)) != 0) {
        // Aircraft Registration string mismatch. Make sure to use memcmp and not strncmp for this since it might not be null terminated
#if AP_ADSB_UAVIONIX_UCP_SET_CONFIG_DEBUG
        GCS_SEND_TEXT(MAV_SEVERITY_CRITICAL,"ADSB: mismatch callsign %s, %s", msg1.registration, msg2.registration);
#endif
        return false;
    }

    return true;
}

GDL90_TRANSPONDER_CONFIG_MSG_V4_V5 AP_ADSB_uAvionix_UCP::populate_Transponder_Config() const
{
    GDL90_TRANSPONDER_CONFIG_MSG_V4_V5 msg {};

    msg.messageId = GDL90_ID_TRANSPONDER_CONFIG;
    msg.version = 5;

    uint32_t icao_id = _frontend.out_state.cfg.ICAO_id_param.get();
    msg.icaoAddress[0] = (icao_id >> 16) & 0xFF;
    msg.icaoAddress[1] = (icao_id >> 8) & 0xFF;
    msg.icaoAddress[2] = (icao_id & 0x0FF);

    msg.maxSpeed = AP_ADSB::convert_maxknots_to_enum(_frontend.out_state.cfg.maxAircraftSpeed_knots);
    msg.baroAltSource = GDL90_BARO_DATA_SOURCE_INTERNAL;
    msg.SDA = rx.decoded.transponder_config.SDA ? rx.decoded.transponder_config.SDA: ADSB_SDA_10_NEG5;
    msg.SIL = rx.decoded.transponder_config.SIL ? rx.decoded.transponder_config.SIL : ADSB_SIL_10_NEG7;
    msg.lengthWidth = (ADSB_AIRCRAFT_LENGTH_WIDTH)_frontend.out_state.cfg.lengthWidth.get();
    msg.es1090InCapable = (_frontend.out_state.cfg.rf_capable.get() & ADSB_BITBASK_RF_CAPABILITIES_1090ES_IN) != 0 ? ADSB_1090ES_IN_CAPABLE : ADSB_NOT_1090ES_IN_CAPABLE;
    msg.uatInCapable = (_frontend.out_state.cfg.rf_capable.get() & ADSB_BITBASK_RF_CAPABILITIES_UAT_IN) != 0 ? ADSB_UAT_IN_CAPABLE : ADSB_NOT_UAT_IN_CAPABLE;
    msg.testMode =  0;
    msg.longitudinalOffset = (ADSB_GPS_LONGITUDINAL_OFFSET)_frontend.out_state.cfg.gpsOffsetLon.get();
    msg.lateralOffset = (ADSB_GPS_LATERAL_OFFSET)_frontend.out_state.cfg.gpsOffsetLat.get();

    memcpy(msg.registration, _frontend.out_state.cfg.callsign, sizeof(msg.registration));

    msg.stallSpeed_cmps = (_frontend.out_state.cfg.stall_speed_cm > 0) ? _frontend.out_state.cfg.stall_speed_cm : 1000;
    msg.emitterType = (ADSB_EMITTER)_frontend.out_state.cfg.emitterType.get();
    msg.baudRate = PING_COM_57600_BAUD;
    msg.modeAEnabled = 0;
    msg.modeCEnabled = 0;
    msg.modeSEnabled = 0;
    msg.es1090TxEnabled = 0;
    msg.defaultSquawk = _frontend.out_state.cfg.squawk_octal;
    msg.baro100 = 0; // 0 = 25 foot, 1 == 100 foot

    // CONFIG_VALIDITY_BITMASK
    msg.valdityBitmask.icaoValid = 1;
    msg.valdityBitmask.silValid = 0;
    msg.valdityBitmask.sdaValid = 0;
    msg.valdityBitmask.baroAltSourceValid = 1;
    msg.valdityBitmask.aircraftMaxSpeedValid = 1;
    msg.valdityBitmask.adsbInCapValid = 1;
    msg.valdityBitmask.aircraftLenWidthValid = 1;
    msg.valdityBitmask.aircraftLatOffsetValid = 1;
    msg.valdityBitmask.aircraftLongOffsetValid = 1;
    msg.valdityBitmask.aircraftRegValid = 1;
    msg.valdityBitmask.aircraftStallSpeedValid = 1;
    msg.valdityBitmask.aircraftEmitterCatValid = 1;
    msg.valdityBitmask.default1090ExTxModeValid = 1;
    msg.valdityBitmask.defaultModeSReplyModeValid = 1;
    msg.valdityBitmask.defaultModeCReplyModeValid = 1;
    msg.valdityBitmask.defaultModeAReplyModeValid = 1;
    msg.valdityBitmask.defaultModeASquawkValid = 1;
    msg.valdityBitmask.baro100Valid = 1;
    msg.valdityBitmask.serialBaudRateValid = 1;

    return msg;
}
#endif // AP_ADSB_UAVIONIX_UCP_SET_CONFIG

void AP_ADSB_uAvionix_UCP::send_Transponder_Control()
{
    GDL90_TRANSPONDER_CONTROL_MSG msg {};
    msg.messageId = GDL90_ID_TRANSPONDER_CONTROL;
    msg.version = GDL90_TRANSPONDER_CONTROL_VERSION;

#if CONFIG_HAL_BOARD == HAL_BOARD_SITL
    // when using the simulator, always declare we're on the ground to help
    // inhibit chaos if this ias actually being broadcasted on real hardware
    msg.airGroundState =  ADSB_ON_GROUND;
#elif AP_ADSB_UAVIONIX_DETECT_GROUNDSTATE
    msg.airGroundState =  _frontend.out_state.is_flying ? ADSB_AIRBORNE_SUBSONIC : ADSB_ON_GROUND;
#else
    msg.airGroundState = ADSB_AIRBORNE_SUBSONIC;
#endif

    msg.baroCrossChecked = ADSB_NIC_BARO_UNVERIFIED;
    msg.identActive = _frontend.out_state.ctrl.identActive;
    _frontend.out_state.ctrl.identActive = false; // only send identButtonActive once per request
    msg.modeAEnabled = _frontend.out_state.ctrl.modeAEnabled;
    msg.modeCEnabled = _frontend.out_state.ctrl.modeCEnabled;
    msg.modeSEnabled = (_frontend._options & uint32_t(AP_ADSB::AdsbOption::Mode3_Only)) ? 0 : _frontend.out_state.ctrl.modeSEnabled;
    msg.es1090TxEnabled = (_frontend._options & uint32_t(AP_ADSB::AdsbOption::Mode3_Only)) ? 0 : _frontend.out_state.ctrl.es1090TxEnabled;

    // if enabled via param ADSB_OPTIONS, use squawk 7400 while in any Loss-Comms related failsafe
    // https://www.faa.gov/documentLibrary/media/Notice/N_JO_7110.724_5-2-9_UAS_Lost_Link_2.pdf
    const AP_Notify& notify = AP::notify();
    if (((_frontend._options & uint32_t(AP_ADSB::AdsbOption::Squawk_7400_FS_RC)) && notify.flags.failsafe_radio) ||
        ((_frontend._options & uint32_t(AP_ADSB::AdsbOption::Squawk_7400_FS_GCS)) && notify.flags.failsafe_gcs)) {
        msg.squawkCode = 7400;
    } else {
        msg.squawkCode = _frontend.out_state.ctrl.squawkCode;
    }

#if AP_ADSB_UAVIONIX_EMERGENCY_STATUS_ON_LOST_LINK
    const uint32_t last_gcs_ms = gcs().sysid_mygcs_last_seen_time_ms();
    const bool gcs_lost_comms = (last_gcs_ms != 0) && (AP_HAL::millis() - last_gcs_ms > AP_ADSB_UAVIONIX_GCS_LOST_COMMS_LONG_TIMEOUT_MS);
    msg.emergencyState = gcs_lost_comms ? ADSB_EMERGENCY_STATUS::ADSB_EMERGENCY_UAS_LOST_LINK : ADSB_EMERGENCY_STATUS::ADSB_EMERGENCY_NONE;
#else
    msg.emergencyState = ADSB_EMERGENCY_STATUS::ADSB_EMERGENCY_NONE;
#endif

#if GDL90_TRANSPONDER_CONTROL_VERSION == 2
    msg.x_bit = 0;
#endif

    memcpy(msg.callsign, _frontend.out_state.ctrl.callsign, sizeof(msg.callsign));

    gdl90Transmit((GDL90_TX_MESSAGE&)msg, sizeof(msg));
}

void AP_ADSB_uAvionix_UCP::send_GPS_Data()
{
    GDL90_GPS_DATA_V2 msg {};
    msg.messageId = GDL90_ID_GPS_DATA;
    msg.version = 2;

    const AP_ADSB::Loc &gps { _frontend._my_loc };

    const GPS_FIX fix = (GPS_FIX)gps.status();
    const bool fix_is_good = (fix >= GPS_FIX_3D);
    const Vector3f velocity = fix_is_good ? gps.velocity() : Vector3f();

    msg.utcTime_s = gps.time_epoch_usec() * 1E-6;
    msg.latitude_ddE7 = fix_is_good ? _frontend._my_loc.lat : INT32_MAX;
    msg.longitude_ddE7 = fix_is_good ? _frontend._my_loc.lng : INT32_MAX;
    msg.altitudeGnss_mm = fix_is_good ? (_frontend._my_loc.alt * 10): INT32_MAX;

    // Protection Limits. FD or SBAS-based depending on state bits
    // Estimate HPL based on horizontal accuracy/HFOM to a probability of 10^-7:
    //  Using the central limit theorem for a Gaussian distribution,
    //  this is 5.326724*stdDev.
    //  Conservatively round up to 6 as a scaling factor,
    //  and asssume HFOM of 95% was calculated as 2*stdDev*HDOP.
    //  This yields a factor of 3 to estimate HPL from horizontal accuracy.
    float accHoriz;
    bool gotAccHoriz = gps.horizontal_accuracy(accHoriz);
    msg.HPL_mm = gotAccHoriz ? 3 * accHoriz * 1E3 : UINT32_MAX; // required to calculate NIC
    msg.VPL_cm = UINT32_MAX; // unused by ping200X

    // Figure of Merits
    msg.horizontalFOM_mm = gotAccHoriz ? accHoriz * 1E3 : UINT32_MAX;
    float accVert;
    msg.verticalFOM_cm = gps.vertical_accuracy(accVert) ? accVert * 1E2 : UINT16_MAX;
    float accVel;
    msg.horizontalVelocityFOM_mmps = gps.speed_accuracy(accVel) ? accVel * 1E3 : UINT16_MAX;
    msg.verticalVelocityFOM_mmps = msg.horizontalVelocityFOM_mmps;

    // Velocities
    msg.verticalVelocity_cmps = fix_is_good ? -1.0f * velocity.z * 1E2 : INT16_MAX;
    msg.northVelocity_mmps = fix_is_good ? velocity.x * 1E3 : INT32_MAX;
    msg.eastVelocity_mmps = fix_is_good ? velocity.y * 1E3 : INT32_MAX;

    // State
    msg.fixType = fix;

    GDL90_GPS_NAV_STATE nav_state {};
    nav_state.HPLfdeActive = 1;
    nav_state.fault = 0;
    nav_state.HrdMagNorth = 0;  // 1 means "north" is magnetic north

    msg.navState = nav_state;
    msg.satsUsed = gps.num_sats();

    gdl90Transmit((GDL90_TX_MESSAGE&)msg, sizeof(msg));
}


bool AP_ADSB_uAvionix_UCP::hostTransmit(uint8_t *buffer, uint16_t length)
{
    if (_port == nullptr || _port->txspace() < length) {
      return false;
    }
    _port->write(buffer, length);
    return true;
}


bool AP_ADSB_uAvionix_UCP::request_msg(const GDL90_MESSAGE_ID msg_id)
{
    const GDL90_TRANSPONDER_MESSAGE_REQUEST_V2 msg = {
      messageId : GDL90_ID_MESSAGE_REQUEST,
      version   : 2,
      reqMsgId  : msg_id
    };
    return gdl90Transmit((GDL90_TX_MESSAGE&)msg, sizeof(msg)) != 0;
}


uint16_t AP_ADSB_uAvionix_UCP::gdl90Transmit(const GDL90_TX_MESSAGE &message, const uint16_t length)
{
    uint8_t gdl90FrameBuffer[GDL90_TX_MAX_FRAME_LENGTH] {};

    const uint16_t frameCrc = crc16_ccitt_GDL90((uint8_t*)&message.raw, length, 0);

    // Set flag byte in frame buffer
    gdl90FrameBuffer[0] = GDL90_FLAG_BYTE;
    uint16_t frameIndex = 1;

    // Copy and stuff all payload bytes into frame buffer
    for (uint16_t i = 0; i < length+2; i++) {
        // Check for overflow of frame buffer
        if (frameIndex >= GDL90_TX_MAX_FRAME_LENGTH) {
            return 0;
        }

        uint8_t data;
        // Append CRC to payload
        if (i == length) {
            data = LOWBYTE(frameCrc);
        } else if (i == length+1) {
            data = HIGHBYTE(frameCrc);
        } else {
            data = message.raw[i];
        }

        if (data == GDL90_FLAG_BYTE || data == GDL90_CONTROL_ESCAPE_BYTE) {
            // Check for frame buffer overflow on stuffed byte
            if (frameIndex + 2 > GDL90_TX_MAX_FRAME_LENGTH) {
              return 0;
            }

            // Set control break and stuff this byte
            gdl90FrameBuffer[frameIndex++] = GDL90_CONTROL_ESCAPE_BYTE;
            gdl90FrameBuffer[frameIndex++] = data ^ GDL90_STUFF_BYTE;
        } else {
            gdl90FrameBuffer[frameIndex++] = data;
        }
    }

    // Add end of frame indication
    gdl90FrameBuffer[frameIndex++] = GDL90_FLAG_BYTE;

    // Push packet to UART
    if (hostTransmit(gdl90FrameBuffer, frameIndex)) {
        return frameIndex;
    }

    return 0;
}


bool AP_ADSB_uAvionix_UCP::parseByte(const uint8_t data, GDL90_RX_MESSAGE &msg, GDL90_RX_STATUS &status)
{
    switch (status.state)
    {
    case GDL90_RX_IDLE:
        if (data == GDL90_FLAG_BYTE && status.prev_data == GDL90_FLAG_BYTE) {
            status.length = 0;
            status.state = GDL90_RX_IN_PACKET;
        }
        break;

    case GDL90_RX_IN_PACKET:
        if (data == GDL90_CONTROL_ESCAPE_BYTE) {
            status.state = GDL90_RX_UNSTUFF;

        } else if (data == GDL90_FLAG_BYTE) {
            // packet complete! Check CRC and restart packet cycle on all pass or fail scenarios
            status.state = GDL90_RX_IDLE;

            if (status.length < GDL90_OVERHEAD_LENGTH) {
                // something is wrong, there's no actual data
                return false;
            }

            const uint8_t crc_LSB = msg.raw[status.length - 2];
            const uint8_t crc_MSB = msg.raw[status.length - 1];

            // NOTE: status.length contains messageId, payload and CRC16. So status.length-3 is effective payload length
            msg.crc = (uint16_t)crc_LSB | ((uint16_t)crc_MSB << 8);
            const uint16_t crc = crc16_ccitt_GDL90((uint8_t*)&msg.raw, status.length-2, 0);
            if (crc == msg.crc) {
                status.prev_data = data;
                // NOTE: this is the only path that returns true
                return true;
            }

        } else if (status.length < GDL90_RX_MAX_PACKET_LENGTH) {
            msg.raw[status.length++] = data;

        } else {
            status.state = GDL90_RX_IDLE;
        }
        break;

    case GDL90_RX_UNSTUFF:
        msg.raw[status.length++] = data ^ GDL90_STUFF_BYTE;
        status.state = GDL90_RX_IN_PACKET;
        break;
    }
    status.prev_data = data;
    return false;
}

#endif // HAL_ADSB_UCP_ENABLED

