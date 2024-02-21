/*
 * Copyright (C)  2022 Sagetech Avionics Inc. All rights reserved.
 *
 * This file is free software: you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the
 * Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This file is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program.  If not, see <http://www.gnu.org/licenses/>.
 * 
 * SDK specification
 * https://github.com/Sagetech-Avionics/sagetech-mxs-sdk/blob/main/doc/pdf/ICD02373_MXS_Host_ICD.pdf
 * 
 * Authors: Chuck Faber, Tom Pittenger
*/


#include "AP_ADSB_Sagetech_MXS.h"

#if HAL_ADSB_SAGETECH_MXS_ENABLED
#include <GCS_MAVLink/GCS.h>
#include <AP_SerialManager/AP_SerialManager.h>
#include <stdio.h>
#include <time.h>
#include <AP_Vehicle/AP_Vehicle_Type.h>

#define SAGETECH_USE_MXS_SDK        !APM_BUILD_TYPE(APM_BUILD_UNKNOWN)

#define MXS_INIT_TIMEOUT                            20000

#define SAGETECH_SCALE_CM_TO_FEET                   (0.0328084f)
#define SAGETECH_SCALE_FEET_TO_MM                   (304.8f)
#define SAGETECH_SCALE_KNOTS_TO_CM_PER_SEC          (51.4444f)
#define SAGETECH_SCALE_FT_PER_MIN_TO_CM_PER_SEC     (0.508f)
#define SAGETECH_SCALE_M_PER_SEC_TO_FT_PER_MIN      (196.85f)
#define CLIMB_RATE_LIMIT                            16448

#define SAGETECH_INSTALL_MSG_RATE                   5000
#define SAGETECH_OPERATING_MSG_RATE                 1000
#define SAGETECH_FLIGHT_ID_MSG_RATE                 8200
#define SAGETECH_GPS_MSG_RATE_FLYING                200
#define SAGETECH_GPS_MSG_RATE_GROUNDED              1000
#define SAGETECH_TARGETREQ_MSG_RATE                 1000
#define SAGETECH_HFOM_UNKNOWN                       (19000.0f) 
#define SAGETECH_VFOM_UNKNOWN                       (151.0f)
#define SAGETECH_HPL_UNKNOWN                        (38000.0f)

bool AP_ADSB_Sagetech_MXS::detect() 
{
    return AP::serialmanager().have_serial(AP_SerialManager::SerialProtocol_ADSB, 0);
}


bool AP_ADSB_Sagetech_MXS::init() 
{
    _port = AP::serialmanager().find_serial(AP_SerialManager::SerialProtocol_ADSB, 0);
    if (_port == nullptr) {
        return false;
    }
    return true;
}

void AP_ADSB_Sagetech_MXS::update() 
{
    if (_port == nullptr) {
        return;
    }

    // -----------------------------
    // read any available data on serial port
    // -----------------------------
    uint32_t nbytes = MIN(_port->available(), 10 * PAYLOAD_MXS_MAX_SIZE);
    while (nbytes-- > 0) {
        uint8_t data;
        if (!_port->read(data)) {
            break;
        }
        parse_byte(data);
    }

    const uint32_t now_ms = AP_HAL::millis();
    
    // -----------------------------
    // handle timers for generating data
    // -----------------------------
    if (!mxs_state.init) {
        if (!last.packet_initialize_ms || (now_ms - last.packet_initialize_ms >= SAGETECH_INSTALL_MSG_RATE)) {
            last.packet_initialize_ms = now_ms;

            if (_frontend._options & uint32_t(AP_ADSB::AdsbOption::SagteTech_MXS_External_Config)) {
                // request the device's configuration
                send_data_req(dataInstall);

            } else {
                // auto configure based on autopilot's saved parameters
                auto_config_operating();
                auto_config_installation();
                auto_config_flightid();
                send_targetreq_msg();
                _frontend.out_state.cfg.rf_capable.set_and_notify(rf_capable_flags_default);      // Set the RF Capability to 1090ES TX and RX 
                mxs_state.init = true;
            }

        } else if (last.packet_initialize_ms > MXS_INIT_TIMEOUT && !mxs_state.init_failed) {
            GCS_SEND_TEXT(MAV_SEVERITY_CRITICAL, "ADSB Sagetech MXS: Initialization Timeout. Failed to initialize.");
            mxs_state.init_failed = true;
        }

        // before init is done, don't run any other update() tasks
        return;
    } 

    if ((now_ms - last.packet_initialize_ms >= SAGETECH_INSTALL_MSG_RATE) &&
            (mxs_state.inst.icao != (uint32_t)_frontend.out_state.cfg.ICAO_id_param.get() ||
            mxs_state.inst.emitter != convert_emitter_type_to_sg(_frontend.out_state.cfg.emitterType.get()) ||
            mxs_state.inst.size != _frontend.out_state.cfg.lengthWidth.get() ||
            mxs_state.inst.maxSpeed != convert_airspeed_knots_to_sg(_frontend.out_state.cfg.maxAircraftSpeed_knots)
            )) {
        last.packet_initialize_ms = now_ms;
        send_install_msg();

    } else if (!last.packet_PreFlight_ms || (now_ms - last.packet_PreFlight_ms >= SAGETECH_FLIGHT_ID_MSG_RATE)) {
        last.packet_PreFlight_ms = now_ms;
        send_flight_id_msg();

    } else if (!last.packet_Operating_ms ||                                                   // Send once at boot
            now_ms - last.packet_Operating_ms >= SAGETECH_OPERATING_MSG_RATE ||             // Send Operating Message every second
            last.operating_squawk != _frontend.out_state.ctrl.squawkCode ||                 // Or anytime Operating data changes
            last.operating_squawk != _frontend.out_state.cfg.squawk_octal ||
            abs(last.operating_alt - _frontend._my_loc.alt) > 1555 ||                       // 1493cm == 49ft. The output resolution is 100ft per bit
            last.operating_rf_select != _frontend.out_state.cfg.rfSelect ||                 // The following booleans control the MXS OpMode
            last.modeAEnabled != _frontend.out_state.ctrl.modeAEnabled ||
            last.modeCEnabled != _frontend.out_state.ctrl.modeCEnabled ||
            last.modeSEnabled != _frontend.out_state.ctrl.modeSEnabled
            ) {

        if (last.operating_squawk != _frontend.out_state.ctrl.squawkCode) {
            last.operating_squawk = _frontend.out_state.ctrl.squawkCode;
            _frontend.out_state.cfg.squawk_octal_param.set_and_notify(last.operating_squawk);
        } else if (last.operating_squawk != _frontend.out_state.cfg.squawk_octal) {
            last.operating_squawk = _frontend.out_state.cfg.squawk_octal;
            _frontend.out_state.ctrl.squawkCode = last.operating_squawk;
        }
        last.operating_rf_select = _frontend.out_state.cfg.rfSelect;
        last.modeAEnabled = _frontend.out_state.ctrl.modeAEnabled;
        last.modeCEnabled = _frontend.out_state.ctrl.modeCEnabled;
        last.modeSEnabled = _frontend.out_state.ctrl.modeSEnabled;

        last.operating_alt = _frontend._my_loc.alt;
        last.packet_Operating_ms = now_ms;
        send_operating_msg();
    
    } else if (now_ms - last.packet_GPS_ms >= (_frontend.out_state.is_flying ? SAGETECH_GPS_MSG_RATE_FLYING : SAGETECH_GPS_MSG_RATE_GROUNDED)) {
        last.packet_GPS_ms = now_ms;
        send_gps_msg();

    } else if ((now_ms - last.packet_targetReq >= SAGETECH_TARGETREQ_MSG_RATE) && 
            ((mxs_state.treq.icao != (uint32_t)_frontend._special_ICAO_target) || (mxs_state.treq.maxTargets != (uint16_t)_frontend.in_state.list_size_param))) {
        send_targetreq_msg();
    }
}

void AP_ADSB_Sagetech_MXS::handle_packet(const Packet &msg) 
{
#if SAGETECH_USE_MXS_SDK
    switch (msg.type) {
        case MsgType::ACK:
            if(sgDecodeAck((uint8_t*) &msg, &mxs_state.ack)) {
                handle_ack(mxs_state.ack);
            }
            break;

        case MsgType::Installation_Response:
            if (!mxs_state.init && sgDecodeInstall((uint8_t*)&msg, &mxs_state.inst)) {
                // If not doing auto-config, get the current configuration of the MXS
                // Fill out configuration parameters with preconfigured values
                if (mxs_state.ack.opMode == modeOff) {                        // If the last ACK showed an OFF state, turn off all rfSelect bits.
                    _frontend.out_state.cfg.rfSelect.set_and_notify(0);
                } else if (mxs_state.ack.opMode == modeStby) {
                    _frontend.out_state.ctrl.modeAEnabled = false;
                    _frontend.out_state.ctrl.modeCEnabled = false;
                    _frontend.out_state.ctrl.modeSEnabled = false;
                    _frontend.out_state.ctrl.es1090TxEnabled = false;
                } else if (mxs_state.ack.opMode == modeOn) {
                    _frontend.out_state.ctrl.modeAEnabled = true;
                    _frontend.out_state.ctrl.modeCEnabled = false;
                    _frontend.out_state.ctrl.modeSEnabled = true;
                    _frontend.out_state.ctrl.es1090TxEnabled = true;
                } else if (mxs_state.ack.opMode == modeAlt) {
                    _frontend.out_state.ctrl.modeAEnabled = true;
                    _frontend.out_state.ctrl.modeCEnabled = true;
                    _frontend.out_state.ctrl.modeSEnabled = true;
                    _frontend.out_state.ctrl.es1090TxEnabled = true;
                }
                _frontend.out_state.cfg.ICAO_id_param.set_and_notify(mxs_state.inst.icao);
                _frontend.out_state.cfg.lengthWidth.set_and_notify(mxs_state.inst.size);
                _frontend.out_state.cfg.emitterType.set_and_notify(convert_sg_emitter_type_to_adsb(mxs_state.inst.emitter));

                _frontend.out_state.cfg.rf_capable.set_and_notify(rf_capable_flags_default);      // Set the RF Capability to UAT and 1090ES TX and RX 

                mxs_state.init = true;
            }
            break;

        case MsgType::FlightID_Response: {
            sg_flightid_t flightId {};
            if (sgDecodeFlightId((uint8_t*) &msg, &flightId)) {
                _frontend.set_callsign(flightId.flightId, false);
            }
            break;
        }

        // ADSB Messages
        case MsgType::ADSB_StateVector_Report: {
            sg_svr_t svr {};
            if (sgDecodeSVR((uint8_t*) &msg, &svr)) {
                handle_svr(svr);
            }
            break;
        }

        case MsgType::ADSB_ModeStatus_Report: {
            sg_msr_t msr {};
            if (sgDecodeMSR((uint8_t*) &msg, &msr)) {
                handle_msr(msr);
            }
            break;
        }

        case MsgType::Data_Request:
        case MsgType::Target_Request:
        case MsgType::Mode:
        case MsgType::Installation:
        case MsgType::FlightID:
        case MsgType::Operating:
        case MsgType::GPS_Data:
        case MsgType::Status_Response:
        case MsgType::Version_Response:
        case MsgType::Serial_Number_Response:
        case MsgType::Mode_Settings:
        case MsgType::Target_Summary_Report:
        case MsgType::RESERVED_0x84:
        case MsgType::RESERVED_0x85:
        case MsgType::RESERVED_0x8D:
        case MsgType::ADSB_Target_State_Report:
        case MsgType::ADSB_Air_Ref_Vel_Report:
            // unhandled or intended for out-bound only
            break;
    }
#endif // SAGETECH_USE_MXS_SDK
}

bool AP_ADSB_Sagetech_MXS::parse_byte(const uint8_t data) 
{
    switch (message_in.state) {
        default:
        case ParseState::WaitingFor_Start:
            if (data == START_BYTE) {
                message_in.checksum = data; // initialize checksum here
                message_in.state = ParseState::WaitingFor_MsgType;
            }
            break;
        case ParseState::WaitingFor_MsgType:
            message_in.checksum += data;
            message_in.packet.type = static_cast<MsgType>(data);
            message_in.state = ParseState::WaitingFor_MsgId;
            break;
        case ParseState::WaitingFor_MsgId:
            message_in.checksum += data;
            message_in.packet.id = data;
            message_in.state = ParseState::WaitingFor_PayloadLen;
            break;
        case ParseState::WaitingFor_PayloadLen: 
            message_in.checksum += data;
            message_in.packet.payload_length = data;
            message_in.index = 0;
            message_in.state = (data == 0) ? ParseState::WaitingFor_Checksum : ParseState::WaitingFor_PayloadContents;
            break;
        case ParseState::WaitingFor_PayloadContents:
            message_in.checksum += data;
            message_in.packet.payload[message_in.index++] = data;
            if (message_in.index >= message_in.packet.payload_length) {
                message_in.state = ParseState::WaitingFor_Checksum;
            }
            break;
        case ParseState::WaitingFor_Checksum:
            message_in.state = ParseState::WaitingFor_Start;
            if (message_in.checksum == data) {
                // append the checksum to the payload and zero out the payload index
                message_in.packet.payload[message_in.index] = data;
                message_in.index = 0;
                handle_packet(message_in.packet);
            }
            break;
    }
    return false;
}

void AP_ADSB_Sagetech_MXS::msg_write(const uint8_t *data, const uint16_t len) const
{
    if (_port != nullptr) {
        _port->write(data, len);
    }
}

void AP_ADSB_Sagetech_MXS::auto_config_operating()
{
    // Configure the Default Operation Message Data
    mxs_state.op.squawk = AP_ADSB::convert_base_to_decimal(8, _frontend.out_state.cfg.squawk_octal);
    mxs_state.op.opMode = sg_op_mode_t::modeOff;                                      // MXS needs to start in OFF mode to accept installation message
    mxs_state.op.savePowerUp = true;                                                  // Save power-up state in non-volatile
    mxs_state.op.enableSqt = true;                                                    // Enable extended squitters
    mxs_state.op.enableXBit = false;                                                  // Enable the x-bit
    mxs_state.op.milEmergency = false;                                                // Broadcast a military emergency
    mxs_state.op.emergcType = (sg_emergc_t)_frontend.out_state.ctrl.emergencyState;  // Enumerated civilian emergency type

    mxs_state.op.altUseIntrnl = true;                                                 // True = Report altitude from internal pressure sensor (will ignore other bits in the field)
    mxs_state.op.altHostAvlbl = false;
    mxs_state.op.altRes25 = !mxs_state.inst.altRes100;               // Host Altitude Resolution from install

    mxs_state.op.identOn = false;

    const auto &my_loc = _frontend._my_loc;

    populate_op_altitude(my_loc);
    populate_op_climbrate(my_loc);
    populate_op_airspeed_and_heading(my_loc);

    last.msg.type = SG_MSG_TYPE_HOST_OPMSG;

#if SAGETECH_USE_MXS_SDK
    uint8_t txComBuffer[SG_MSG_LEN_OPMSG] {};
    sgEncodeOperating(txComBuffer, &mxs_state.op, ++last.msg.id);
    msg_write(txComBuffer, SG_MSG_LEN_OPMSG);
#endif
}

void AP_ADSB_Sagetech_MXS::auto_config_installation()
{
    // Configure the Default Installation Message Data
    mxs_state.inst.icao = (uint32_t) _frontend.out_state.cfg.ICAO_id_param;
    snprintf(mxs_state.inst.reg, 8, "%-7s", "TEST01Z");

    mxs_state.inst.com0 = sg_baud_t::baud230400;
    mxs_state.inst.com1 = sg_baud_t::baud57600;

    mxs_state.inst.eth.ipAddress = 0;
    mxs_state.inst.eth.subnetMask = 0;
    mxs_state.inst.eth.portNumber = 0;

    mxs_state.inst.sil = sg_sil_t::silUnknown;
    mxs_state.inst.sda = sg_sda_t::sdaUnknown;
    mxs_state.inst.emitter = convert_emitter_type_to_sg(_frontend.out_state.cfg.emitterType.get());
    mxs_state.inst.size = (sg_size_t)_frontend.out_state.cfg.lengthWidth.get();
    mxs_state.inst.maxSpeed = convert_airspeed_knots_to_sg(_frontend.out_state.cfg.maxAircraftSpeed_knots);
    mxs_state.inst.altOffset = 0;         // Alt encoder offset is legacy field that should always be 0.
    mxs_state.inst.antenna = sg_antenna_t::antBottom;

    mxs_state.inst.altRes100 = true;
    mxs_state.inst.hdgTrueNorth = false;
    mxs_state.inst.airspeedTrue = false;
    mxs_state.inst.heater = true;         // Heater should always be connected.
    mxs_state.inst.wowConnected = true;

    last.msg.type = SG_MSG_TYPE_HOST_INSTALL;

#if SAGETECH_USE_MXS_SDK
    uint8_t txComBuffer[SG_MSG_LEN_INSTALL] {};
    sgEncodeInstall(txComBuffer, &mxs_state.inst, ++last.msg.id);
    msg_write(txComBuffer, SG_MSG_LEN_INSTALL);
#endif
}

void AP_ADSB_Sagetech_MXS::auto_config_flightid()
{
    if (!strlen(_frontend.out_state.cfg.callsign)) {
        snprintf(mxs_state.fid.flightId, sizeof(mxs_state.fid.flightId), "%-8s", "TESTMXS0");
    } else {
        snprintf(mxs_state.fid.flightId, sizeof(mxs_state.fid.flightId), "%-8s", _frontend.out_state.cfg.callsign);
    }
    last.msg.type = SG_MSG_TYPE_HOST_FLIGHT;

#if SAGETECH_USE_MXS_SDK
    uint8_t txComBuffer[SG_MSG_LEN_FLIGHT] {};
    sgEncodeFlightId(txComBuffer, &mxs_state.fid, ++last.msg.id);
    msg_write(txComBuffer, SG_MSG_LEN_FLIGHT);
#endif
}

void AP_ADSB_Sagetech_MXS::handle_ack(const sg_ack_t ack)
{
    if ((ack.ackId != last.msg.id) || (ack.ackType != last.msg.type)) {
        // GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "ADSB Sagetech MXS: ACK: Message %d of type %02x not acknowledged.", last.msg.id, last.msg.type);
    }
    // System health
    if (ack.failXpdr && !last.failXpdr) {
        GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "ADSB Sagetech MXS: Transponder Failure");
        _frontend.out_state.tx_status.fault |= UAVIONIX_ADSB_OUT_STATUS_FAULT_TX_SYSTEM_FAIL;
    }
    if (ack.failSystem && !last.failSystem) {
        GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "ADSB Sagetech MXS: System Failure");
        _frontend.out_state.tx_status.fault |= UAVIONIX_ADSB_OUT_STATUS_FAULT_TX_SYSTEM_FAIL;
    }
    last.failXpdr = ack.failXpdr;
    last.failSystem = ack.failSystem;
}

void AP_ADSB_Sagetech_MXS::handle_svr(const sg_svr_t svr) 
{
    if (svr.addrType != svrAdrIcaoUnknown && svr.addrType != svrAdrIcao && svr.addrType != svrAdrIcaoSurface) {
        // invalid icao
        return;
    }

    AP_ADSB::adsb_vehicle_t vehicle;
    if (!_frontend.get_vehicle_by_ICAO(svr.addr, vehicle)) {
        // new vehicle
        memset(&vehicle, 0, sizeof(vehicle));
        vehicle.info.ICAO_address = svr.addr;
    }

    vehicle.info.flags &= ~ADSB_FLAGS_VALID_SQUAWK;

    if (svr.validity.position) {
        vehicle.info.lat = (int32_t) (svr.lat * 1e7);
        vehicle.info.lon = (int32_t) (svr.lon * 1e7);
        vehicle.info.flags |= ADSB_FLAGS_VALID_COORDS;
    }
    if (svr.validity.geoAlt) {
        vehicle.info.altitude_type = ADSB_ALTITUDE_TYPE_GEOMETRIC;
        vehicle.info.altitude = svr.airborne.geoAlt * SAGETECH_SCALE_FEET_TO_MM;    // Convert from feet to mm
        vehicle.info.flags |= ADSB_FLAGS_VALID_ALTITUDE;
    }
    if (svr.validity.baroAlt) {
        vehicle.info.altitude_type = ADSB_ALTITUDE_TYPE_PRESSURE_QNH;
        vehicle.info.altitude = svr.airborne.baroAlt * SAGETECH_SCALE_FEET_TO_MM;    // Convert from feet to mm
        vehicle.info.flags |= ADSB_FLAGS_VALID_ALTITUDE;
    }
    if (svr.validity.surfSpeed) {
        vehicle.info.hor_velocity = svr.surface.speed * SAGETECH_SCALE_KNOTS_TO_CM_PER_SEC;   // Convert from knots to cm/s
        vehicle.info.flags |= ADSB_FLAGS_VALID_VELOCITY;
    }
    if (svr.validity.surfHeading) {
        vehicle.info.heading = svr.surface.heading * 100;
        vehicle.info.flags |= ADSB_FLAGS_VALID_HEADING;
    }
    if (svr.validity.airSpeed) {
        vehicle.info.hor_velocity = svr.airborne.speed * SAGETECH_SCALE_KNOTS_TO_CM_PER_SEC;   // Convert from knots to cm/s
        vehicle.info.heading = svr.airborne.heading * 100;
        vehicle.info.flags |= ADSB_FLAGS_VALID_VELOCITY;
        vehicle.info.flags |= ADSB_FLAGS_VALID_HEADING;
    }
    if (svr.validity.geoVRate || svr.validity.baroVRate) {
        vehicle.info.ver_velocity = svr.airborne.vrate * SAGETECH_SCALE_FT_PER_MIN_TO_CM_PER_SEC; // Convert from ft/min to cm/s
        vehicle.info.flags |= ADSB_FLAGS_VERTICAL_VELOCITY_VALID;
    }

    vehicle.last_update_ms = AP_HAL::millis();
    _frontend.handle_adsb_vehicle(vehicle);
}

void AP_ADSB_Sagetech_MXS::handle_msr(const sg_msr_t msr) 
{
    AP_ADSB::adsb_vehicle_t vehicle;
    if (!_frontend.get_vehicle_by_ICAO(msr.addr, vehicle)) {
        // new vehicle is not allowed here because we don't know the lat/lng
        // yet and we don't allow lat/lng of (0,0) so it will get rejected anyway
        return;
    }

    if (strlen(msr.callsign)) {
        snprintf(vehicle.info.callsign, sizeof(vehicle.info.callsign), "%-8s", msr.callsign);
        vehicle.info.flags |= ADSB_FLAGS_VALID_CALLSIGN;
    } else {
        vehicle.info.flags &= ~ADSB_FLAGS_VALID_CALLSIGN;
    }

    vehicle.last_update_ms = AP_HAL::millis();
    _frontend.handle_adsb_vehicle(vehicle);
}

void AP_ADSB_Sagetech_MXS::send_data_req(const sg_datatype_t dataReqType) 
{
    sg_datareq_t dataReq {};
    dataReq.reqType = dataReqType;
    last.msg.type = SG_MSG_TYPE_HOST_DATAREQ;

#if SAGETECH_USE_MXS_SDK
    uint8_t txComBuffer[SG_MSG_LEN_DATAREQ] {};
    sgEncodeDataReq(txComBuffer, &dataReq, ++last.msg.id);
    msg_write(txComBuffer, SG_MSG_LEN_DATAREQ);
#else
    (void)dataReq;
#endif
}

void AP_ADSB_Sagetech_MXS::send_install_msg()
{
    // MXS must be in OFF mode to change ICAO or Registration
    if (mxs_state.op.opMode != modeOff) {
        // GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "ADSB Sagetech MXS: unable to send installation data while not in OFF mode.");
        return;
    }

    mxs_state.inst.icao = (uint32_t)_frontend.out_state.cfg.ICAO_id_param.get();
    mxs_state.inst.emitter = convert_emitter_type_to_sg(_frontend.out_state.cfg.emitterType.get());
    mxs_state.inst.size = (sg_size_t)_frontend.out_state.cfg.lengthWidth.get();
    mxs_state.inst.maxSpeed = convert_airspeed_knots_to_sg(_frontend.out_state.cfg.maxAircraftSpeed_knots);
    mxs_state.inst.antenna = sg_antenna_t::antBottom;

    last.msg.type = SG_MSG_TYPE_HOST_INSTALL;

#if SAGETECH_USE_MXS_SDK
    uint8_t txComBuffer[SG_MSG_LEN_INSTALL] {};
    sgEncodeInstall(txComBuffer, &mxs_state.inst, ++last.msg.id);
    msg_write(txComBuffer, SG_MSG_LEN_INSTALL);
#endif
}

void AP_ADSB_Sagetech_MXS::send_flight_id_msg()
{
    if (!strlen((char*) _frontend.out_state.ctrl.callsign)) {
        return;
    }
    snprintf(mxs_state.fid.flightId, sizeof(mxs_state.fid.flightId), "%-8s", (char*) _frontend.out_state.ctrl.callsign);

    last.msg.type = SG_MSG_TYPE_HOST_FLIGHT;

#if SAGETECH_USE_MXS_SDK
    uint8_t txComBuffer[SG_MSG_LEN_FLIGHT] {};
    sgEncodeFlightId(txComBuffer, &mxs_state.fid, ++last.msg.id);
    msg_write(txComBuffer, SG_MSG_LEN_FLIGHT);
#endif
}

void AP_ADSB_Sagetech_MXS::send_operating_msg()
{
    if (!_frontend.out_state.ctrl.modeAEnabled && !_frontend.out_state.ctrl.modeCEnabled && 
            !_frontend.out_state.ctrl.modeSEnabled && !_frontend.out_state.ctrl.es1090TxEnabled) {
        mxs_state.op.opMode = modeStby;
    }
    if (_frontend.out_state.ctrl.modeAEnabled && !_frontend.out_state.ctrl.modeCEnabled && 
            _frontend.out_state.ctrl.modeSEnabled && _frontend.out_state.ctrl.es1090TxEnabled) {
        mxs_state.op.opMode = modeOn;
    }
    if (_frontend.out_state.ctrl.modeAEnabled && _frontend.out_state.ctrl.modeCEnabled && 
            _frontend.out_state.ctrl.modeSEnabled && _frontend.out_state.ctrl.es1090TxEnabled) {
        mxs_state.op.opMode = modeAlt;
    }
    if ((_frontend.out_state.cfg.rfSelect & 1) == 0) {
        mxs_state.op.opMode = modeOff;
    }

    mxs_state.op.squawk = AP_ADSB::convert_base_to_decimal(8, last.operating_squawk);
    mxs_state.op.emergcType = (sg_emergc_t) _frontend.out_state.ctrl.emergencyState;

    const auto &my_loc = _frontend._my_loc;

    populate_op_altitude(my_loc);
    populate_op_climbrate(my_loc);
    populate_op_airspeed_and_heading(my_loc);

    mxs_state.op.identOn = _frontend.out_state.ctrl.identActive;
    _frontend.out_state.ctrl.identActive = false;                           // only send identButtonActive once per request

    last.msg.type = SG_MSG_TYPE_HOST_OPMSG;

#if SAGETECH_USE_MXS_SDK
    uint8_t txComBuffer[SG_MSG_LEN_OPMSG] {};
    sgEncodeOperating(txComBuffer, &mxs_state.op, ++last.msg.id);
    msg_write(txComBuffer, SG_MSG_LEN_OPMSG);
#endif
}

void AP_ADSB_Sagetech_MXS::send_gps_msg()
{
    sg_gps_t gps {};
    const AP_ADSB::Loc &ap_gps { _frontend._my_loc };
    float hAcc, vAcc, velAcc;

    gps.hpl = SAGETECH_HPL_UNKNOWN;                                                     // HPL over 37,040m means unknown
    gps.hfom = ap_gps.horizontal_accuracy(hAcc) ? hAcc : SAGETECH_HFOM_UNKNOWN;         // HFOM over 18,520 specifies unknown
    gps.vfom = ap_gps.vertical_accuracy(vAcc) ? vAcc : SAGETECH_VFOM_UNKNOWN;           // VFOM over 150 specifies unknown
    gps.nacv = sg_nacv_t::nacvUnknown;
    if (ap_gps.speed_accuracy(velAcc)) {
        if (velAcc >= 10.0 || velAcc < 0) {
            gps.nacv = sg_nacv_t::nacvUnknown;
        }
        else if (velAcc >= 3.0) {
            gps.nacv = sg_nacv_t::nacv10dot0;
        }
        else if (velAcc >= 1.0) {
            gps.nacv = sg_nacv_t::nacv3dot0;
        }
        else if (velAcc >= 0.3) {
            gps.nacv = sg_nacv_t::nacv1dot0;
        }
        else { //if (velAcc >= 0.0)
            gps.nacv = sg_nacv_t::nacv0dot3;
        }
    }

    // Get Vehicle Longitude and Latitude and Convert to string
    const int32_t longitude = _frontend._my_loc.lng;
    const int32_t latitude =  _frontend._my_loc.lat;
    const double lon_deg = longitude * (double)1.0e-7 * (longitude < 0 ? -1 : 1);
    const double lon_minutes = (lon_deg - int(lon_deg)) * 60;
    snprintf((char*)&gps.longitude, 12, "%03u%02u.%05u", (unsigned)lon_deg, (unsigned)lon_minutes, unsigned((lon_minutes - (int)lon_minutes) * 1.0E5));

    const double lat_deg = latitude * (double)1.0e-7 * (latitude < 0 ? -1 : 1);
    const double lat_minutes = (lat_deg - int(lat_deg)) * 60;
    snprintf((char*)&gps.latitude, 11, "%02u%02u.%05u", (unsigned)lat_deg, (unsigned)lat_minutes, unsigned((lat_minutes - (int)lat_minutes) * 1.0E5));

    const Vector2f speed = _frontend._my_loc.groundspeed_vector();
    const float speed_knots = speed.length() * M_PER_SEC_TO_KNOTS;
    snprintf((char*)&gps.grdSpeed, 7, "%03u.%02u", (unsigned)speed_knots, unsigned((speed_knots - (int)speed_knots) * 1.0E2));

    const float heading = wrap_360(degrees(speed.angle()));
    snprintf((char*)&gps.grdTrack, 9, "%03u.%04u", unsigned(heading), unsigned((heading - (int)heading) * 1.0E4));


    gps.latNorth = (latitude >= 0 ? true: false);
    gps.lngEast = (longitude >= 0 ? true: false);

    gps.gpsValid = ap_gps.status() >=  AP_GPS_FixType::FIX_2D;

    uint64_t time_usec = ap_gps.epoch_from_rtc_us;
    if (ap_gps.have_epoch_from_rtc_us) {
        const time_t time_sec = time_usec * 1E-6;
        struct tm tmd {};
        struct tm* tm = gmtime_r(&time_sec, &tmd);

        snprintf((char*)&gps.timeOfFix, 11, "%02u%02u%06.3f", tm->tm_hour, tm->tm_min, tm->tm_sec + (time_usec % 1000000) * 1.0e-6);
    } else {
        strncpy(gps.timeOfFix, "      .   ", 11);
    }

    int32_t height;
    if (_frontend._my_loc.initialised() && _frontend._my_loc.get_alt_cm(Location::AltFrame::ABSOLUTE, height)) {
        gps.height = height * 0.01;
    } else {
        gps.height = 0.0;
    }

    last.msg.type = SG_MSG_TYPE_HOST_GPS;

#if SAGETECH_USE_MXS_SDK
    uint8_t txComBuffer[SG_MSG_LEN_GPS] {};
    sgEncodeGPS(txComBuffer, &gps, ++last.msg.id);
    msg_write(txComBuffer, SG_MSG_LEN_GPS);
#else
    (void)gps;
#endif
}

void AP_ADSB_Sagetech_MXS::send_targetreq_msg() 
{
    mxs_state.treq.reqType = sg_reporttype_t::reportAuto;
    mxs_state.treq.transmitPort = sg_transmitport_t::transmitCom1;
    mxs_state.treq.maxTargets = _frontend.in_state.list_size_param;
    mxs_state.treq.icao = _frontend._special_ICAO_target.get();
    mxs_state.treq.stateVector = true;
    mxs_state.treq.modeStatus = true;
    mxs_state.treq.targetState = false;
    mxs_state.treq.airRefVel = false;
    mxs_state.treq.tisb = false;
    mxs_state.treq.military = false;
    mxs_state.treq.commA = false;
    mxs_state.treq.ownship = true;

    last.msg.type = SG_MSG_TYPE_HOST_TARGETREQ;

#if SAGETECH_USE_MXS_SDK
    uint8_t txComBuffer[SG_MSG_LEN_TARGETREQ] {};
    sgEncodeTargetReq(txComBuffer, &mxs_state.treq, ++last.msg.id);
    msg_write(txComBuffer, SG_MSG_LEN_TARGETREQ);
#endif
}

sg_emitter_t AP_ADSB_Sagetech_MXS::convert_emitter_type_to_sg(const uint8_t emitterType) const
{
    uint8_t result = SG_EMIT_OFFSET_D;

    if (emitterType < 8) {
        result = emitterType;
    } else if (emitterType < 13 && emitterType >= 8) {
        result = (emitterType - 8) + SG_EMIT_OFFSET_B;
    } else if (emitterType < 16 && emitterType >= 14) {
        result = (emitterType - 14) + (SG_EMIT_OFFSET_B + 6);      // Subtracting 14 because SG emitter types don't include the reserved state at value 13.
    } else if (emitterType < 21 && emitterType >= 16) {
        result = (emitterType - 16) + SG_EMIT_OFFSET_C;
    }
    return (sg_emitter_t)result;
}

uint8_t AP_ADSB_Sagetech_MXS::convert_sg_emitter_type_to_adsb(const sg_emitter_t sgEmitterType) const
{
    if (sgEmitterType < SG_EMIT_OFFSET_B) {
        return sgEmitterType;
    } else if ((sgEmitterType < (SG_EMIT_OFFSET_B + 6)) && (sgEmitterType >= SG_EMIT_OFFSET_B)) {
        return (sgEmitterType - SG_EMIT_OFFSET_B) + 8;
    } else if ((sgEmitterType < SG_EMIT_OFFSET_C) && (sgEmitterType >= SG_EMIT_OFFSET_B + 6)) {
        return (sgEmitterType - (SG_EMIT_OFFSET_B + 6)) + 14;   // Starts at UAV = 14
    } else if ((sgEmitterType < SG_EMIT_OFFSET_D) && (sgEmitterType >= SG_EMIT_OFFSET_C)) {
        return (sgEmitterType - SG_EMIT_OFFSET_C) + 16;
    } else {
        return 0;
    }
}

sg_airspeed_t AP_ADSB_Sagetech_MXS::convert_airspeed_knots_to_sg(const float maxAirSpeed) const
{
    const int32_t airspeed = (int) maxAirSpeed;

    if (airspeed < 0) {
        return sg_airspeed_t::speedUnknown;
    } else if (airspeed < 75) {
        return sg_airspeed_t::speed75kt;
    } else if (airspeed < 150) {
        return sg_airspeed_t::speed150kt;
    } else if (airspeed < 300) {
        return sg_airspeed_t::speed300kt;
    } else if (airspeed < 600) {
        return sg_airspeed_t::speed600kt;
    } else if (airspeed < 1200) {
        return sg_airspeed_t::speed1200kt;
    } else { //if (airspeed >= 1200)
        return sg_airspeed_t::speedGreater;
    }
}

void AP_ADSB_Sagetech_MXS::populate_op_altitude(const AP_ADSB::Loc &loc)
{
    int32_t height;
    if (loc.initialised() && loc.get_alt_cm(Location::AltFrame::ABSOLUTE, height)) {
        mxs_state.op.altitude = height * SAGETECH_SCALE_CM_TO_FEET;         // Height above sealevel in feet
    } else {
        mxs_state.op.altitude = 0;
    }
}

void AP_ADSB_Sagetech_MXS::populate_op_climbrate(const AP_ADSB::Loc &my_loc)
{
    float vertRateD;
    if (my_loc.get_vert_pos_rate_D(vertRateD)) {
        // convert from down to up, and scale appropriately:
        mxs_state.op.climbRate = -1 * vertRateD * SAGETECH_SCALE_M_PER_SEC_TO_FT_PER_MIN;
        mxs_state.op.climbValid = true;
    } else {
        mxs_state.op.climbValid = false;
        mxs_state.op.climbRate = -CLIMB_RATE_LIMIT;
    }
}

void AP_ADSB_Sagetech_MXS::populate_op_airspeed_and_heading(const AP_ADSB::Loc &my_loc)
{
    const Vector2f speed = my_loc.groundspeed_vector();
    if (!speed.is_nan() && !speed.is_zero()) {
        mxs_state.op.headingValid = true;
        mxs_state.op.airspdValid = true;
    } else {
        mxs_state.op.headingValid = false;
        mxs_state.op.airspdValid = false;
    }
    const uint16_t speed_knots = speed.length() * M_PER_SEC_TO_KNOTS;
    double heading = wrap_360(degrees(speed.angle()));
    mxs_state.op.airspd = speed_knots;
    mxs_state.op.heading = heading;
}

#endif // HAL_ADSB_SAGETECH_MXS_ENABLED

