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
 */

#include "AP_ADSB_uAvionix_UCP.h"

#if HAL_ADSB_UCP_ENABLED

#include <AP_SerialManager/AP_SerialManager.h>
#include <GCS_MAVLink/GCS.h>
#include <AP_HAL/AP_HAL.h>
#include <AP_Math/AP_Math.h>
#include <AP_Math/crc.h>
#include <ctype.h>
#include <AP_Notify/AP_Notify.h>

extern const AP_HAL::HAL &hal;

#define AP_ADSB_GCS_LOST_COMMS_LONG_TIMEOUT_MINUTES     (15UL)
#define AP_ADSB_GCS_LOST_COMMS_LONG_TIMEOUT_MS          (1000UL * 60UL * AP_ADSB_GCS_LOST_COMMS_LONG_TIMEOUT_MINUTES)
#define AP_ADSB_MSG_TIMEOUT_MS                          (5000)


// detect if any port is configured as uAvionix_UCP
bool AP_ADSB_uAvionix_UCP::detect()
{
    return (AP::serialmanager().find_serial(AP_SerialManager::SerialProtocol_ADSB, 0) != nullptr);
}


// Init, called once after class is constructed
bool AP_ADSB_uAvionix_UCP::init()
{
    _port = AP::serialmanager().find_serial(AP_SerialManager::SerialProtocol_ADSB, 0);

    return (_port != nullptr);
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
        const int16_t data = (uint8_t)_port->read();
        if (data < 0) {
            break;
        }

        if (parseByte((uint8_t)data, rx.msg, rx.status)) {
            rx.last_msg_ms = now_ms;
            handle_msg(rx.msg);
        }
    } // while nbytes

    if (now_ms - rx.last_msg_ms > AP_ADSB_MSG_TIMEOUT_MS) {
        // detect disconnected device and re-init
        rx.last_msg_ms = now_ms;
        if (init_state.stage != INIT_STAGE::WAIT_BOOT_DELAY) {
            init_state.timer_ms = 0;
            init_state.stage = INIT_STAGE::WAIT_BOOT_DELAY;
        }
        return;
    } else if (init_state.stage != INIT_STAGE::DONE)  {
        init_device();
        return;
    }

    run_device();
}


void AP_ADSB_uAvionix_UCP::run_device()
{
    const uint32_t now_ms = AP_HAL::millis();

    if (now_ms - run_state.last_packet_Transponder_Control_ms >= 1000) {
        run_state.last_packet_Transponder_Control_ms = now_ms;
        send_Transponder_Control();
    }

    bool const inhibitSendGps = (_frontend._options & uint32_t(AP_ADSB::AdsbOption::Inhibit_GPS_tx)) != 0;
    if (!inhibitSendGps && now_ms - run_state.last_packet_GPS_ms >= 200) {
        run_state.last_packet_GPS_ms = now_ms;
        send_GPS_Data();
    }


    bool const checkConfig_1s = (_frontend._options & uint32_t(AP_ADSB::AdsbOption::CheckConfig_1s)) != 0;
    bool const checkConfig_10s = (_frontend._options & uint32_t(AP_ADSB::AdsbOption::CheckConfig_10s)) != 0;
    if ((checkConfig_1s && now_ms - run_state.last_config_check_ms >= 1000) ||
        (checkConfig_10s && now_ms - run_state.last_config_check_ms >= 10000)) {
        run_state.last_config_check_ms = now_ms;

        // this checks the config but if we're not doing the full init then this will not change anythign and not spam the GCS
        if (!check_config()) {
            // something has changed since init, lets re-init
            init_state.stage = INIT_STAGE::WAIT_FOR_HW_ID;
        }
    }
}


void AP_ADSB_uAvionix_UCP::init_device()
{
    const uint32_t now_ms = AP_HAL::millis();

    switch (init_state.stage) {
    case INIT_STAGE::WAIT_BOOT_DELAY:
        if (init_state.timer_ms == 0) {
            init_state.timer_ms = now_ms;
            // invalidate the current cached copies to ensure we get fresh ones
            rx.decoded.identification.messageId = (GDL90_MESSAGE_ID)0;      // this should get auto-populated at 1Hz
            rx.decoded.transponder_config.messageId = (GDL90_MESSAGE_ID)0;  // this gets populated once I request it
        } else if (now_ms - init_state.timer_ms >= 2000) {
            // this gives us time for the 1 Hz loops to update params and settle
            init_state.stage = INIT_STAGE::WAIT_FOR_HW_ID;
        }
        break;

    case INIT_STAGE::WAIT_FOR_HW_ID:
        if (rx.decoded.identification.messageId != 0) {
            // messageId will populate with the message id once it receives that packet
            init_state.stage = INIT_STAGE::REQUEST_CONFIG;
        }
        break;

    case INIT_STAGE::REQUEST_CONFIG:
        if (rx.decoded.transponder_config.messageId != 0) {
            // messageId will populate with the message id once it receives that packet
            init_state.stage = INIT_STAGE::COMPARE_CONFIG;
        } else if (now_ms - init_state.timer_ms >= 1000) {
            init_state.timer_ms = now_ms;
            request_msg(GDL90_ID_TRANSPONDER_CONFIG);
        }
        break;

    case INIT_STAGE::COMPARE_CONFIG: {
        if (check_config()) {
            // confing looks OK!
            init_state.stage = INIT_STAGE::DONE;
        } else {
            // invalidate the structure and re-request
            init_state.timer_ms = 0;
            init_state.stage = INIT_STAGE::UPDATE_CONFIG;
        }
        } break;

    case INIT_STAGE::UPDATE_CONFIG:
        if (init_state.timer_ms == 0) {
            init_state.timer_ms = now_ms;
            gdl90Transmit((GDL90_TX_MESSAGE&)rx.decoded.transponder_config, sizeof(rx.decoded.transponder_config));
        } else if (now_ms - init_state.timer_ms >= 500) {
            init_state.stage = INIT_STAGE::REQUEST_CONFIG;
        }
        break;

    case INIT_STAGE::DONE:
        // nothing to do, we shouldn't ever reach this state
        return;
    } // init_state.stage

    if (init_state.stage == INIT_STAGE::DONE) {
        gcs().send_text(MAV_SEVERITY_DEBUG, "ADSB: %s Initialized", get_hardware_name(rx.decoded.identification.primary.hwId));
    }
}


bool AP_ADSB_uAvionix_UCP::check_config()
{
    // returns true if all the fields we care about match
    // asume true. If anything needs updating then set cfg_OK to false and update
    // the value and then the next state will set that value in the device and then
    // we'll verify by looping around again
    bool cfg_OK = true;
    
    const bool check_only = (init_state.stage == INIT_STAGE::DONE);

    const uint32_t device_icao = ((uint32_t)rx.decoded.transponder_config.icaoAddress[2]) |
                                 ((uint32_t)rx.decoded.transponder_config.icaoAddress[1] << 8) |
                                 ((uint32_t)rx.decoded.transponder_config.icaoAddress[0] << 16);

    uint32_t my_icao;
    if (_frontend.out_state.cfg.was_set_externally) {
        my_icao = (uint32_t)_frontend.out_state.cfg.ICAO_id;
    } else {
        my_icao = (uint32_t)_frontend.out_state.cfg.ICAO_id_param;
    }
    my_icao &= 0x00FFFFFF;
    
    if ((device_icao != my_icao) && (my_icao != 0)) {
        if (check_only) {
            return false;
        }
        gcs().send_text(MAV_SEVERITY_DEBUG, "ADSB: Setting Cfg: IcaoAddress %X -> %X",
            (unsigned)device_icao,
            (unsigned)my_icao);

        cfg_OK = false;
        rx.decoded.transponder_config.icaoAddress[2] = my_icao       & 0xFF;
        rx.decoded.transponder_config.icaoAddress[1] = (my_icao>>8)  & 0xFF;
        rx.decoded.transponder_config.icaoAddress[0] = (my_icao>>16) & 0xFF;

        rx.decoded.transponder_config.valdityBitmask.icaoValid = 1;
    }

    const ADSB_AIRCRAFT_LENGTH_WIDTH lengthWidth = (ADSB_AIRCRAFT_LENGTH_WIDTH)_frontend.out_state.cfg.lengthWidth.get();
    if (rx.decoded.transponder_config.lengthWidth != lengthWidth) {
        if (check_only) {
            return false;
        }
        gcs().send_text(MAV_SEVERITY_DEBUG, "ADSB: Setting Cfg: lengthWidth %u -> %u",
            (unsigned)rx.decoded.transponder_config.lengthWidth,
            (unsigned)lengthWidth);

        cfg_OK = false;
        rx.decoded.transponder_config.lengthWidth = lengthWidth;
        rx.decoded.transponder_config.valdityBitmask.aircraftLenWidthValid = 1;
    }

    const ADSB_UAT_IN_CAPABILITY supports_UAT_in = (_frontend.out_state.cfg.rf_capable & ADSB_BITBASK_RF_CAPABILITIES_UAT_IN) != 0 ? ADSB_UAT_IN_CAPABLE : ADSB_NOT_UAT_IN_CAPABLE;
    const ADSB_1090ES_IN_CAPABILITY supports_1090ES_in = (_frontend.out_state.cfg.rf_capable & ADSB_BITBASK_RF_CAPABILITIES_1090ES_IN) != 0 ? ADSB_1090ES_IN_CAPABLE :ADSB_NOT_1090ES_IN_CAPABLE;
    if (rx.decoded.transponder_config.uatInCapable != supports_UAT_in ||
        rx.decoded.transponder_config.es1090InCapable != supports_1090ES_in) {
        if (check_only) {
            return false;
        }
        gcs().send_text(MAV_SEVERITY_DEBUG, "ADSB: Setting Cfg: rf_capable UAT(%u -> %u) 1090ES(%u -> %u)",
            (unsigned)rx.decoded.transponder_config.uatInCapable,
            (unsigned)supports_UAT_in,
            (unsigned)rx.decoded.transponder_config.es1090InCapable,
            (unsigned)supports_1090ES_in);

        cfg_OK = false;
        rx.decoded.transponder_config.uatInCapable = supports_UAT_in;
        rx.decoded.transponder_config.es1090InCapable = supports_1090ES_in;
        rx.decoded.transponder_config.valdityBitmask.adsbInCapValid = 1;
    }

    if (rx.decoded.transponder_config.stallSpeed_cmps != _frontend.out_state.cfg.stall_speed_cm) {
        if (check_only) {
            return false;
        }
        gcs().send_text(MAV_SEVERITY_DEBUG, "ADSB: Setting Cfg: stall speed %u -> %u",
            (unsigned)rx.decoded.transponder_config.stallSpeed_cmps,
            (unsigned)_frontend.out_state.cfg.stall_speed_cm);

        cfg_OK = false;
        rx.decoded.transponder_config.stallSpeed_cmps = _frontend.out_state.cfg.stall_speed_cm;
        rx.decoded.transponder_config.valdityBitmask.aircraftStallSpeedValid = 1;
    }

    const GDL90_BARO_DATA_SOURCE baroAltSource_desired = (_frontend._options & uint32_t(AP_ADSB::AdsbOption::Autopilot_is_alt_baro)) ? GDL90_BARO_DATA_SOURCE_EXTERNAL : GDL90_BARO_DATA_SOURCE_INTERNAL;
    if (rx.decoded.transponder_config.baroAltSource != baroAltSource_desired ||
        rx.decoded.transponder_config.baro100 != 0 ||
        rx.decoded.transponder_config.testMode != 0) {
        if (check_only) {
            return false;
        }
        gcs().send_text(MAV_SEVERITY_DEBUG, "ADSB: Setting Cfg: BaroAltSrc(%u->%u), Baro(%u->0), Test(%u->0)",
            (unsigned)rx.decoded.transponder_config.baroAltSource,
            (unsigned)baroAltSource_desired,
            (unsigned)rx.decoded.transponder_config.baro100,
            (unsigned)rx.decoded.transponder_config.testMode);

        cfg_OK = false;

        rx.decoded.transponder_config.baro100 = 0;
        rx.decoded.transponder_config.testMode = 0;
        rx.decoded.transponder_config.baroAltSource = baroAltSource_desired;

        rx.decoded.transponder_config.valdityBitmask.baro100Valid = 1;
        rx.decoded.transponder_config.valdityBitmask.defaultModeSReplyModeValid = 1;
        rx.decoded.transponder_config.valdityBitmask.defaultModeCReplyModeValid = 1;
        rx.decoded.transponder_config.valdityBitmask.defaultModeAReplyModeValid = 1;
    }

    const ADSB_GPS_LONGITUDINAL_OFFSET long_offset = (ADSB_GPS_LONGITUDINAL_OFFSET)_frontend.out_state.cfg.gpsOffsetLon.get();
    const ADSB_GPS_LATERAL_OFFSET lat_offset = (ADSB_GPS_LATERAL_OFFSET)_frontend.out_state.cfg.gpsOffsetLat.get();
    if (rx.decoded.transponder_config.longitudinalOffset != long_offset ||
        rx.decoded.transponder_config.lateralOffset != lat_offset) {
        if (check_only) {
            return false;
        }
        gcs().send_text(MAV_SEVERITY_DEBUG, "ADSB: Setting Cfg: GPS Offset lat(%u -> %u) long(%u -> %u)",
            (unsigned)rx.decoded.transponder_config.longitudinalOffset,
            (unsigned)long_offset,
            (unsigned)rx.decoded.transponder_config.lateralOffset,
            (unsigned)lat_offset);

        cfg_OK = false;
        rx.decoded.transponder_config.longitudinalOffset = long_offset;
        rx.decoded.transponder_config.lateralOffset = lat_offset;
        rx.decoded.transponder_config.valdityBitmask.aircraftLatOffsetValid = 1;
        rx.decoded.transponder_config.valdityBitmask.aircraftLongOffsetValid = 1;
    }
    
    const ADSB_EMITTER emitterType = (ADSB_EMITTER)_frontend.out_state.cfg.emitterType.get();
    if (rx.decoded.transponder_config.emitterType != emitterType) {
        if (check_only) {
            return false;
        }
        gcs().send_text(MAV_SEVERITY_DEBUG, "ADSB: Setting Cfg: emitter Type %u -> %u",
            (unsigned)rx.decoded.transponder_config.emitterType,
            (unsigned)emitterType);

        cfg_OK = false;
        rx.decoded.transponder_config.emitterType = emitterType;
        rx.decoded.transponder_config.valdityBitmask.aircraftEmitterCatValid = 1;
    }

    const bool es1090TxEnabled = (_frontend.out_state.cfg.rfSelect & UAVIONIX_ADSB_OUT_RF_SELECT_TX_ENABLED) != 0;
    if (rx.decoded.transponder_config.es1090TxEnabled != es1090TxEnabled) {
        if (check_only) {
            return false;
        }
        gcs().send_text(MAV_SEVERITY_DEBUG, "ADSB: Setting Cfg: TX Enabled at power-up %u -> %u",
            (unsigned)rx.decoded.transponder_config.es1090TxEnabled,
            (unsigned)es1090TxEnabled);

        cfg_OK = false;
        rx.decoded.transponder_config.es1090TxEnabled = es1090TxEnabled;
        rx.decoded.transponder_config.valdityBitmask.default1090ExTxModeValid = 1;
    }

    const uint16_t defaultSquawk = _frontend.out_state.cfg.squawk_octal;
    if (rx.decoded.transponder_config.defaultSquawk != defaultSquawk) {
        if (check_only) {
            return false;
        }
        gcs().send_text(MAV_SEVERITY_DEBUG, "ADSB: Setting Cfg: Squawk %u -> %u",
            (unsigned)rx.decoded.transponder_config.defaultSquawk,
            (unsigned)defaultSquawk);

        cfg_OK = false;
        rx.decoded.transponder_config.defaultSquawk = defaultSquawk;
        rx.decoded.transponder_config.valdityBitmask.defaultModeASquawkValid = 1;
    }

    const uint8_t registration_len = sizeof(rx.decoded.transponder_config.registration); // this is 8
    if (_frontend.out_state.cfg.was_set_externally &&
        memcmp(rx.decoded.transponder_config.registration, _frontend.out_state.cfg.callsign, registration_len) != 0) {
        // this is very very hard to have set at boot time. To take advantage of this, set ADSB.TYPE=0
        // then reboot, then after it boots up, send mavlink_uavionix_adsb_out_cfg_t and then enable ADSB.
        // When ADSB starts up, that flag will be set and _frontend.out_state.cfg.callsign will be available

        if (check_only) {
            return false;
        }

        gcs().send_text(MAV_SEVERITY_DEBUG, "ADSB: Setting Cfg: Callsign %s -> %s",
            rx.decoded.transponder_config.registration,
            _frontend.out_state.cfg.callsign);
        
        cfg_OK = false;
        memcpy(rx.decoded.transponder_config.registration, _frontend.out_state.cfg.callsign, registration_len);
        bool has_seen_space = false;
        for (uint8_t i=0; i<registration_len; i++) {
            if (has_seen_space) {
                // once there's a space, it must be trailing whitespace
                rx.decoded.transponder_config.registration[i] = ' ';
            }

            const uint8_t value = rx.decoded.transponder_config.registration[i];
            if (isupper(value) || isdigit(value) || value == ' ') {
                //Aircraft Registration (ASCII string A-Z, 0-9 only), e.g. “N8644B “. Trailing spaces (0x20) only.
                continue;
            }

            if (islower(value)) {
                rx.decoded.transponder_config.registration[i] = toupper(value);
            } else {
                rx.decoded.transponder_config.registration[i] = ' ';
                has_seen_space = true;
            }
        }
    }

    // Don't Care.. or we do't store them in params to be able to veryify and set them
    // maxSpeed
    // SIL
    // SDA
    // inProtocol - we're already talking..
    // outProtocol - we're already talking..
    // baudRate    - we're already talking to it, won't hurt to just always forever use this baud

    return cfg_OK;
}

void AP_ADSB_uAvionix_UCP::handle_msg(const GDL90_RX_MESSAGE &msg)
{
    switch(msg.messageId) {
    case GDL90_ID_HEARTBEAT:
        // The Heartbeat message provides real-time indications of the status and operation of the
        // transponder. The message will be transmitted with a period of one second for the UCP
        // protocol.
        memcpy(&rx.decoded.heartbeat, msg.raw, sizeof(rx.decoded.heartbeat));
        
        _frontend.out_state.ident_isActive = rx.decoded.heartbeat.status.one.ident;
        if (rx.decoded.heartbeat.status.one.ident) {
            // if we're identing, clear the pending send request
            _frontend.out_state.ident_pending = false;
        }

        break;
    case GDL90_ID_OWNSHIP_REPORT:
        // The Ownship message contains information on the GNSS position. If the Ownship GNSS
        // position fix is invalid, the Latitude, Longitude, and NIC fields will all have the ZERO value. The
        // Ownship message will be transmitted with a period of one second regardless of data status or
        // update for the UCP protocol. All fields in the ownship message are transmitted MSB first
        memcpy(&rx.decoded.ownship_report, msg.raw, sizeof(rx.decoded.ownship_report));
        break;
    case GDL90_ID_OWNSHIP_GEOMETRIC_ALTITUDE:
        // An Ownship Geometric Altitude message will be transmitted with a period of one second when
        // the GNSS fix is valid for the UCP protocol. All fields in the Geometric Ownship Altitude
        // message are transmitted MSB first.
        memcpy(&rx.decoded.ownship_geometric_altitude, msg.raw, sizeof(rx.decoded.ownship_geometric_altitude));
        break;
    case GDL90_ID_IDENTIFICATION:
        // The Identification message contains information used to identify the connected device. The
        // Identification message will be transmitted with a period of one second regardless of data status
        // or update for the UCP protocol and will be transmitted upon request for the UCP-HD protocol.
        if (rx.decoded.identification.messageId == 0) {
            memcpy(&rx.decoded.identification, msg.raw, sizeof(rx.decoded.identification));

            // Firmware Part Number (not null terminated, but null padded if part number is less than 15 characters)
            // copy into a temporary string that is 1 char longer so we ensure it's null terminated
            const uint8_t str_len = sizeof(rx.decoded.identification.primaryFwPartNumber);
            char primaryFwPartNumber[str_len+1];
            memcpy(&primaryFwPartNumber, rx.decoded.identification.primaryFwPartNumber, str_len);
            primaryFwPartNumber[str_len] = 0;
            
            gcs().send_text(MAV_SEVERITY_DEBUG,"ADSB:Detected %s v%u.%u.%u SN:%u %s",
            get_hardware_name(rx.decoded.identification.primary.hwId),
            (unsigned)rx.decoded.identification.primary.fwMajorVersion,
            (unsigned)rx.decoded.identification.primary.fwMinorVersion,
            (unsigned)rx.decoded.identification.primary.fwBuildVersion,
            (unsigned)rx.decoded.identification.primary.serialNumber,
            primaryFwPartNumber);
        }
        break;
    case GDL90_ID_SENSOR_MESSAGE:
        memcpy(&rx.decoded.sensor_message, msg.raw, sizeof(rx.decoded.sensor_message));
        break;
    case GDL90_ID_TRANSPONDER_CONFIG:
        memcpy(&rx.decoded.transponder_config, msg.raw, sizeof(rx.decoded.transponder_config));
        break;
    case GDL90_ID_TRANSPONDER_STATUS:
        memcpy(&rx.decoded.transponder_status, msg.raw, sizeof(rx.decoded.transponder_status));
        break;

    case GDL90_ID_TRANSPONDER_CONTROL:
    case GDL90_ID_GPS_DATA:
    case GDL90_ID_MESSAGE_REQUEST:
        // not handled, outbound only
        break;
    default:
        //gcs().send_text(MAV_SEVERITY_DEBUG,"ADSB:Unknown msg %d", (int)msg.messageId);
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
        case 0x26: //return "Ping200Z/Ping200X";    // (uncertified). Let's fallthrough and use use Ping200X
        case 0x2F: return "Ping200X";               // (certified)
        case 0x30: return "TailBeaconX";            // (certified)
    } // switch hwId
    return "Unknown HW";    
}


void AP_ADSB_uAvionix_UCP::send_Transponder_Control()
{
    GDL90_TRANSPONDER_CONTROL_MSG msg {};
    msg.messageId = GDL90_ID_TRANSPONDER_CONTROL;
    msg.version = 1;

#if CONFIG_HAL_BOARD == HAL_BOARD_SITL
    // when using the simulator, always declare we'r eon the ground to help
    // inhibit chaos if this ias actually being broadcasted on real hardware
    msg.airGroundState =  ADSB_ON_GROUND;
#else
    msg.airGroundState =  _frontend.out_state.is_flying ? ADSB_AIRBORNE_SUBSONIC : ADSB_ON_GROUND;
#endif

    msg.baroCrossChecked = ADSB_NIC_BARO_UNVERIFIED;
    msg.identActive = _frontend.out_state.ident_pending;
    msg.modeAEnabled = true;
    msg.modeCEnabled = true;
    msg.modeSEnabled = true;
    msg.es1090TxEnabled = (_frontend.out_state.cfg.rfSelect & UAVIONIX_ADSB_OUT_RF_SELECT_TX_ENABLED) != 0;

    if ((rx.decoded.transponder_config.baroAltSource == GDL90_BARO_DATA_SOURCE_EXTERNAL) && AP::baro().healthy()) {
        msg.externalBaroAltitude_mm = AP::baro().get_altitude() * 0.001; // convert m to mm
    } else {
        msg.externalBaroAltitude_mm = INT32_MAX;
    }

    // use squawk 7400 while in any Loss-Comms related failsafe
    // https://www.faa.gov/documentLibrary/media/Notice/N_JO_7110.724_5-2-9_UAS_Lost_Link_2.pdf
    const AP_Notify& notify = AP::notify();
    if (((_frontend._options & uint32_t(AP_ADSB::AdsbOption::Squawk_7400_FS_RC)) && notify.flags.failsafe_radio) ||
        ((_frontend._options & uint32_t(AP_ADSB::AdsbOption::Squawk_7400_FS_GCS)) && notify.flags.failsafe_gcs)) {
        msg.squawkCode = 7400;
    } else {
        msg.squawkCode = _frontend.out_state.cfg.squawk_octal;
    }

    const uint32_t last_gcs_ms = gcs().sysid_myggcs_last_seen_time_ms();
    const bool gcs_lost_comms = (last_gcs_ms != 0) && (AP_HAL::millis() - last_gcs_ms < AP_ADSB_GCS_LOST_COMMS_LONG_TIMEOUT_MS);
    msg.emergencyState = gcs_lost_comms ? ADSB_EMERGENCY_STATUS::ADSB_EMERGENCY_UAS_LOST_LINK : ADSB_EMERGENCY_STATUS::ADSB_EMERGENCY_NONE;
    
    memcpy(msg.callsign, _frontend.out_state.cfg.callsign, sizeof(msg.callsign));
}


void AP_ADSB_uAvionix_UCP::send_GPS_Data()
{
    GDL90_GPS_DATA_V2 msg {};
    msg.messageId = GDL90_ID_GPS_DATA;
    msg.version = 2;
    
    const AP_GPS &gps = AP::gps();
    const GPS_FIX fix = (GPS_FIX)gps.status();
    const bool fix_is_good = (fix >= GPS_FIX_3D);
    const Vector3f &velocity = fix_is_good ? gps.velocity() : Vector3f();

    msg.utcTime_s = gps.time_epoch_usec() * 1E-3;
    msg.latitude_ddE7 = fix_is_good ? _frontend._my_loc.lat : INT32_MAX;
    msg.longitude_ddE7 = fix_is_good ? _frontend._my_loc.lng : INT32_MAX;
    msg.altitudeGnss_mm = fix_is_good ? _frontend._my_loc.alt : INT32_MAX;

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
    msg.verticalVelocity_cmps = fix_is_good ? velocity.z * 1E2 : INT16_MAX;
    msg.northVelocity_mmps = fix_is_good ? velocity.x * 1E3 : INT32_MAX;
    msg.eastVelocity_mmps = fix_is_good ? velocity.y * 1E3 : INT32_MAX;

    // State
    msg.fixType = fix;
    GDL90_GPS_NAV_STATE nav_state {};
      nav_state.HPLfdeActive = 1;
      nav_state.fault = 0;
      nav_state.HrdMagNorth = 1;  // 1 means "north" is magnetic north
    msg.navState = nav_state;
    msg.satsUsed = AP::gps().num_sats();

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

