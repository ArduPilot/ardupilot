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

extern const AP_HAL::HAL &hal;

#define AP_ADSB_UAVIONIX_HEALTH_TIMEOUT_MS                     (5000UL)

#define AP_ADSB_UAVIONIX_GCS_LOST_COMMS_LONG_TIMEOUT_MINUTES    (15UL)
#define AP_ADSB_UAVIONIX_GCS_LOST_COMMS_LONG_TIMEOUT_MS         (1000UL * 60UL * AP_ADSB_UAVIONIX_GCS_LOST_COMMS_LONG_TIMEOUT_MINUTES)

#define AP_ADSB_UAVIONIX_DETECT_GROUNDSTATE                     0
#define AP_ADSB_UAVIONIX_EMERGENCY_STATUS_ON_LOST_LINK          0

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

    request_msg(GDL90_ID_IDENTIFICATION);
    request_msg(GDL90_ID_TRANSPONDER_CONFIG);
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

   if (now_ms - run_state.last_packet_Transponder_Control_ms >= 1000) {
        run_state.last_packet_Transponder_Control_ms = now_ms;
        send_Transponder_Control();
    }

    if ((now_ms - run_state.last_packet_GPS_ms >= 200) && (_frontend._options & uint32_t(AP_ADSB::AdsbOption::Ping200X_Send_GPS)) != 0) {
        run_state.last_packet_GPS_ms = now_ms;
        send_GPS_Data();
    }

    // if the transponder has stopped giving us the data needed to 
    // fill the transponder status mavlink message, reset that data.
    if ((now_ms - run_state.last_packet_Transponder_Status_ms >= 10000 && run_state.last_packet_Transponder_Status_ms != 0)
        && (now_ms - run_state.last_packet_Transponder_Heartbeat_ms >= 10000 && run_state.last_packet_Transponder_Heartbeat_ms != 0)
        && (now_ms - run_state.last_packet_Transponder_Ownship_ms >= 10000 && run_state.last_packet_Transponder_Ownship_ms != 0))
    {
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

        // this is always true. The "ground/air bit place" is set meaning we're always in the air
        _frontend.out_state.tx_status.state |= UAVIONIX_ADSB_OUT_STATUS_STATE_ON_GROUND;

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

        _frontend.out_state.tx_status.fault &= ~UAVIONIX_ADSB_OUT_STATUS_FAULT_STATUS_MESSAGE_UNAVAIL;

        }
        break;

    case GDL90_ID_IDENTIFICATION:
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
        }
        break;

    case GDL90_ID_TRANSPONDER_CONFIG:
        memcpy(&rx.decoded.transponder_config, msg.raw, sizeof(rx.decoded.transponder_config));
        break;

#if AP_ADSB_UAVIONIX_UCP_CAPTURE_ALL_RX_PACKETS
    case GDL90_ID_OWNSHIP_REPORT:
        // The Ownship message contains information on the GNSS position. If the Ownship GNSS
        // position fix is invalid, the Latitude, Longitude, and NIC fields will all have the ZERO value. The
        // Ownship message will be transmitted with a period of one second regardless of data status or
        // update for the UCP protocol. All fields in the ownship message are transmitted MSB first
        memcpy(&rx.decoded.ownship_report, msg.raw, sizeof(rx.decoded.ownship_report));
        run_state.last_packet_Transponder_Ownship_ms = AP_HAL::millis();
        _frontend.out_state.tx_status.NIC_NACp = rx.decoded.ownship_report.report.NIC | (rx.decoded.ownship_report.report.NACp << 4);
        memcpy(_frontend.out_state.tx_status.flight_id, rx.decoded.ownship_report.report.callsign, sizeof(_frontend.out_state.tx_status.flight_id));
        //_frontend.out_state.tx_status.temperature = rx.decoded.ownship_report.report.temperature; // there is no message in the vocabulary of the 200x that has board temperature
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

        _frontend.out_state.tx_status.fault &= ~UAVIONIX_ADSB_OUT_STATUS_FAULT_STATUS_MESSAGE_UNAVAIL;

        if (run_state.last_packet_Transponder_Status_ms == 0) {
            // set initial control message contents to transponder defaults
            _frontend.out_state.ctrl.modeAEnabled = rx.decoded.transponder_status.modeAEnabled;
            _frontend.out_state.ctrl.modeCEnabled = rx.decoded.transponder_status.modeCEnabled;
            _frontend.out_state.ctrl.modeSEnabled = rx.decoded.transponder_status.modeSEnabled;
            _frontend.out_state.ctrl.es1090TxEnabled = rx.decoded.transponder_status.es1090TxEnabled;
            _frontend.out_state.ctrl.squawkCode = rx.decoded.transponder_status.squawkCode;
            _frontend.out_state.ctrl.x_bit = rx.decoded.transponder_status.x_bit;
        }
        run_state.last_packet_Transponder_Status_ms = AP_HAL::millis();
#if AP_MAVLINK_MSG_UAVIONIX_ADSB_OUT_STATUS_ENABLED
        GCS_SEND_MESSAGE(MSG_UAVIONIX_ADSB_OUT_STATUS);
#endif
        break;
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
    msg.modeSEnabled = _frontend.out_state.ctrl.modeSEnabled;
    msg.es1090TxEnabled = _frontend.out_state.ctrl.es1090TxEnabled;

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
    const uint32_t last_gcs_ms = gcs().sysid_myggcs_last_seen_time_ms();
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
    msg.HPL_mm = UINT32_MAX;
    msg.VPL_cm = UINT32_MAX;

    // Figure of Merits
    float accHoriz;
    msg.horizontalFOM_mm = gps.horizontal_accuracy(accHoriz) ? accHoriz * 1E3 : UINT32_MAX;
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


uint16_t AP_ADSB_uAvionix_UCP::gdl90Transmit(GDL90_TX_MESSAGE &message, const uint16_t length)
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

