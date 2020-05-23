#include "AP_Mount_Trillium.h"

#if AP_MOUNT_TRILLIUM_ENABLED

#include <GCS_MAVLink/GCS.h>
#include <AP_SerialManager/AP_SerialManager.h>
#include <SRV_Channel/SRV_Channel.h>

extern const AP_HAL::HAL& hal;

#define AP_MOUNT_TRILLIUM_DEBUG_RX_ALL_MSGS                 1
#define AP_MOUNT_TRILLIUM_DEBUG_RX_UNHANDLED_MSGS           1
#define AP_MOUNT_TRILLIUM_DEBUG_TX_NOT_AUTOPILOT_MSGS       1
#define AP_MOUNT_TRILLIUM_DEBUG_TX_CMD_CHANGE_ONLY_MSGS     1
#define AP_MOUNT_TRILLIUM_DEBUG_TX_ALL_MSGS                 1
#define AP_MOUNT_TRILLIUM_DEBUG_TX_PASSTHROUGH_DROPS        1

#define AP_MOUNT_TRILLIUM_MAVLINK_PASSTHROUGH_DEVICE        13
#define AP_MOUNT_TRILLIUM_MAVLINK_PASSTHROUGH_ENABLE        1

#define AP_MOUNT_TRILLIUM_SET_ETHERNET_SETTINGS             0

#define AP_MOUNT_TRILLIUM_SITL_USE_IP                       1
#define AP_MOUNT_TRILLIUM_SITL_IP                           "172.20.114.45"
#define AP_MOUNT_TRILLIUM_SITL_PORT                         8748


void AP_Mount_Trillium::init()
{
    // check for Trillium Gimbal protocol
    _port = AP::serialmanager().find_serial(AP_SerialManager::SerialProtocol_Trillium, 0);

    // reset boot state
    memset(&_booting, 0, sizeof(_booting));

    if (_port != nullptr) {
        _port->set_unbuffered_writes(true);
        _port->set_flow_control(AP_HAL::UARTDriver::FLOW_CONTROL_DISABLE);

        set_mode((enum MAV_MOUNT_MODE)_state._default_mode.get());
    }
}

void AP_Mount_Trillium::init_hw()
{
    if (_booting.done) {
        return;
    }

#if AP_MOUNT_TRILLIUM_SITL_USE_IP && (CONFIG_HAL_BOARD == HAL_BOARD_SITL)
    if (!sock_connected) {
        sock_connected = sock.connect(AP_MOUNT_TRILLIUM_SITL_IP, AP_MOUNT_TRILLIUM_SITL_PORT);
        if (!sock_connected) {
            return;
        }
    }
#endif


    OrionPkt_t PktOut;
    (void)PktOut; // touch this so we don't get a unused variable error if all #defined features are disabled

    switch (_booting.step++) {
    case 0:
        break;

    case 1:
#if AP_MOUNT_TRILLIUM_SET_ETHERNET_SETTINGS
        // set network settings
        encodeOrionNetworkByteSettingsPacketStructure(&PktOut, &_network_settings_desired);
        OrionCommSend(&PktOut);
#endif
        break;

    case 2:
        // Request version indo
        requestOrionMessageByID(ORION_PKT_CAMERAS);
        break;
    case 3:
        requestOrionMessageByID(ORION_PKT_CLEVIS_VERSION);
        break;
    case 4:
        requestOrionMessageByID(ORION_PKT_CROWN_VERSION);
        break;
    case 5:
        requestOrionMessageByID(ORION_PKT_PAYLOAD_VERSION);
        break;
    case 6:
        requestOrionMessageByID(ORION_PKT_TRACKER_VERSION);
        break;
    case 7:
        requestOrionMessageByID(ORION_PKT_LENSCTL_VERSION);
        break;
    case 8:
        requestOrionMessageByID(ORION_PKT_BOARD);
        break;

    case 9:
    case 10:
        // TODO: add more hw init commands to complete the hw boot-up process without a factory-reseted device ever needing to connect to the company-provided software
        break;

    default:
        _booting.done = true;
        _booting.step = 0;
        break;
    }
}

// update mount position - should be called periodically
void AP_Mount_Trillium::update()
{
    read_incoming(); // read the incoming messages from the gimbal. This must be done before _booting.done is checked

    if (!_booting.done) {
        init_hw();
        return;
    }

    // flag to trigger sending target angles to gimbal
    bool resend_now = false;
    OrionPkt_t PktOut;
    const uint32_t now_ms = AP_HAL::millis();
    //const SRV_Channel* deplyServo = SRV_Channels::get_channel_for(SRV_Channel::k_mount_open);
    bool notifyGcs = false;

    // update based on mount mode
    switch(get_mode()) {
            // move mount to a "retracted" position.  we do not implement a separate servo based retract mechanism
        case MAV_MOUNT_MODE_RETRACT:
            _angle_ef_target_rad = _state._retract_angles.get() * DEG_TO_RAD;
            if (false) {
//            if (deplyServo != nullptr) {
//                notifyGcs = deplyServo->get_output_pwm() != deplyServo->get_output_min();
//                SRV_Channels::set_output_to_min(SRV_Channel::k_mount_open);

            } else if ((now_ms - _deploy_command_last_ms > 2000) && (_retract_status.State == RETRACT_STATE_DEPLOYED || _retract_status.State == RETRACT_STATE_DEPLOYING)) {
                _deploy_command_last_ms = now_ms;
                notifyGcs = true;
                encodeOrionRetractCommandPacket(&PktOut, RETRACT_CMD_RETRACT);
                OrionCommSend(&PktOut);
            }
            if (notifyGcs) {
                gcs().send_text(MAV_SEVERITY_DEBUG, "%sRetracting", _trilliumGcsHeader);
            }
            break;

        // move mount to a neutral position, typically pointing forward
        case MAV_MOUNT_MODE_NEUTRAL:
            _angle_ef_target_rad = _state._neutral_angles.get() * DEG_TO_RAD;
            if (false) {
//            if (deplyServo != nullptr) {
//                notifyGcs = deplyServo->get_output_pwm() != deplyServo->get_output_max();
//                SRV_Channels::set_output_to_max(SRV_Channel::k_mount_open);

            } else if ((now_ms - _deploy_command_last_ms > 2000) && (_retract_status.State == RETRACT_STATE_RETRACTED || _retract_status.State == RETRACT_STATE_RETRACTING)) {
                _deploy_command_last_ms = now_ms;
                notifyGcs = true;
                encodeOrionRetractCommandPacket(&PktOut, RETRACT_CMD_DEPLOY);
                OrionCommSend(&PktOut);
            }
            if (notifyGcs) {
                gcs().send_text(MAV_SEVERITY_DEBUG, "%sDeploying", _trilliumGcsHeader);
            }
            break;

        // point to the angles given by a mavlink message
        case MAV_MOUNT_MODE_MAVLINK_TARGETING:
            // do nothing because earth-frame angle targets (i.e. _angle_ef_target_rad) should have already been set by a MOUNT_CONTROL message from GCS
            resend_now = true;
            break;

        // RC radio manual angle control, but with stabilization from the AHRS
        case MAV_MOUNT_MODE_RC_TARGETING:
            // update targets using pilot's rc inputs
            update_targets_from_rc();
            resend_now = true;
            break;

        // point mount to a GPS point given by the mission planner
        case MAV_MOUNT_MODE_GPS_POINT:
            if (_state._roi_target_set) {
                double targetLat = deg2rad(_state._roi_target.lat * 1.0e-7f);
                double targetLon = deg2rad(_state._roi_target.lng * 1.0e-7f);
                double targetAlt = _state._roi_target.alt * 0.01f;    // cm -> m
                float targetVelNed[] = { 0.0, 0.0, 0.0 };

                encodeGeopointCmdPacket(&PktOut, targetLat, targetLon, targetAlt, targetVelNed, 0, geopointOptions::geopointNone);
                OrionCommSend(&PktOut);

            } else if (calc_angle_to_roi_target(_angle_ef_target_rad, true, true)) {
                resend_now = true;
            }
            break;

        case MAV_MOUNT_MODE_SYSID_TARGET:
            if (_state._target_sysid_location_set && _state._target_sysid != 0) {
                SendGeopointCmd(_state._target_sysid_location, Vector3f(), 0, geopointOptions::geopointNone);

            } else if (calc_angle_to_sysid_target(_angle_ef_target_rad, true, true)) {
                resend_now = true;
            }
            break;

        case MAV_MOUNT_MODE_TRACK:
            // do nothing, roi_target is getting auto-populated by incoming gimbal telemetry
            break;

        default:
            // we do not know this mode so do nothing
            break;
    }


    if (resend_now) {
        send_target_angles(_angle_ef_target_rad, false);
    }

    if (now_ms - _last_send_isFlying_ms > 1000) {
        _last_send_isFlying_ms = now_ms;

        OrionAutopilotData_t packet {};
        packet.IsFlying = AP_Notify::flags.flying;
        packet.CommGood = !AP_Notify::flags.failsafe_radio;
        packet.Agl = -1; // negative means ignore

        encodeOrionAutopilotDataPacketStructure(&PktOut, &packet);
        OrionCommSend(&PktOut);
    }
}

void AP_Mount_Trillium::SendGeopointCmd(const Location targetLoc, const Vector3f targetVelNed_vector, const float joystickRange, const geopointOptions options)
{
    OrionPkt_t PktOut;

    const double targetLat = deg2rad(_state._target_sysid_location.lat * 1.0e-7f);
    const double targetLon = deg2rad(_state._target_sysid_location.lng * 1.0e-7f);
    const double targetAlt = _state._target_sysid_location.alt * 0.01f;    // cm -> m
    float targetVelNed[] = { targetVelNed_vector.x, targetVelNed_vector.y, targetVelNed_vector.z };

    encodeGeopointCmdPacket(&PktOut, targetLat, targetLon, targetAlt, targetVelNed, joystickRange, options);
    OrionCommSend(&PktOut);

}

/*
 * detect and read the header of the incoming message from the gimbal
 */
void AP_Mount_Trillium::read_incoming()
{
    if (_port == nullptr) {
        return;
    }

#if AP_MOUNT_TRILLIUM_SITL_USE_IP && (CONFIG_HAL_BOARD == HAL_BOARD_SITL)
    if (!sock.pollin(0)) {
        // input buffer is empty
        return;
    }
    // limit our reads so we're not here forever
    int16_t num_available = 10*1024;
#else
    int16_t num_available = _port->available();
#endif

#if AP_MOUNT_TRILLIUM_MAVLINK_PASSTHROUGH_ENABLE
    if (num_available <= 0) {
        // this early return saves us from the local variable init work
        return;
    }

    uint8_t passthrough_payload[MAVLINK_MSG_PASSTHROUGH_FIELD_PAYLOAD_LEN];
    uint8_t passthrough_index = 0;
    const bool passthrough_enabled = _passthrough.last_MAVLink_to_gimbal_ms != 0;

    if (passthrough_enabled) {
        _passthrough.last_MAVLink_to_gimbal_ms = AP_HAL::millis();
    }
#endif // passthrough

    while (num_available-- > 0) {
        // Process bytes received
#if AP_MOUNT_TRILLIUM_SITL_USE_IP && (CONFIG_HAL_BOARD == HAL_BOARD_SITL)
        uint8_t buf[1];
        if (sock.recv(&buf, 1, 0) != 1) {
            break;
        }
        const uint8_t rxByte = buf[0];
#else
        const uint8_t rxByte = _port->read();
#endif

        if (LookForOrionPacketInByte(&_PktIn, rxByte)) {
            handle_packet(_PktIn);
        }

#if AP_MOUNT_TRILLIUM_MAVLINK_PASSTHROUGH_ENABLE
        if (passthrough_enabled) {
            passthrough_payload[passthrough_index++] = rxByte;
            if (num_available == 0 || passthrough_index >= sizeof(passthrough_payload)) {
                // end of bytes read or packet is full
                mavlink_msg_passthrough_send(_passthrough.chan, AP_MOUNT_TRILLIUM_MAVLINK_PASSTHROUGH_DEVICE, passthrough_index, passthrough_payload);
                passthrough_index = 0;
            }
        }
#endif // passthrough
    }
}

// send_target_angles in degrees or radians
void AP_Mount_Trillium::send_target_angles(Vector3f angle, bool target_in_degrees)
{
    // Form a command that will tell the gimbal to move at 10 deg/s in pan for one second
    if (target_in_degrees) {
        _cmd_last.Target[GIMBAL_AXIS_PAN]  = deg2radf(angle.x);
        _cmd_last.Target[GIMBAL_AXIS_TILT] = deg2radf(angle.y);
    } else {
        _cmd_last.Target[GIMBAL_AXIS_PAN]  = angle.x;
        _cmd_last.Target[GIMBAL_AXIS_TILT] = angle.y;
    }

    _cmd_last.Mode = _desiredMode;
    _cmd_last.ImpulseTime = 1.0f;
    _cmd_last.Stabilized = TRUE;

    OrionPkt_t pPkt;
    encodeOrionCmdPacket(&pPkt, &_cmd_last);
    OrionCommSend(&pPkt);
}

// send_target_angles in degrees
void AP_Mount_Trillium::send_target_angles(float pitch_deg, float roll_deg, float yaw_deg)
{
    send_target_angles(Vector3f(roll_deg, pitch_deg, yaw_deg), true);
}


void AP_Mount_Trillium::handle_packet(OrionPkt_t &packet)
{

#if AP_MOUNT_TRILLIUM_DEBUG_RX_ALL_MSGS
    gcs().send_text(MAV_SEVERITY_DEBUG, "%sRx 0x%02x,%3u:%s", _trilliumGcsHeader, packet.ID, packet.ID, get_packet_name(packet.ID));
#endif

    const uint8_t len = packet.Length;
    uint16_t index = 0;

    switch (packet.ID) {
    case ORION_PKT_DEBUG_STRING:
        {
        DebugString_t msg;
        decodeDebugStringPacketStructure(&packet, &msg);

        while (index <= len) {
            gcs().send_text(MAV_SEVERITY_DEBUG, "%s%s", _trilliumGcsHeader, (char*)&msg.description[index]);
            index += (MAVLINK_MSG_STATUSTEXT_FIELD_TEXT_LEN - sizeof(_trilliumGcsHeader));
        };
        }
        break;

    case ORION_PKT_RETRACT_STATUS:
        decodeOrionRetractStatusPacket(&packet, &_retract_status.Cmd, &_retract_status.State, &_retract_status.Pos, &_retract_status.Flags);
        break;

    case ORION_PKT_NETWORK_SETTINGS:
        decodeOrionNetworkByteSettingsPacketStructure(&packet, &_network_settings_current);
        break;

    case ORION_PKT_DIAGNOSTICS:
        // Received once every 3 seconds
        decodeOrionDiagnosticsPacketStructure(&packet, &_diagnostics);
        break;

    case ORION_PKT_PERFORMANCE:
        // Received at 4Hz
        decodeOrionPerformancePacketStructure(&packet, &_performance);
        break;

    case ORION_PKT_UART_CONFIG:
        decodeOrionUartConfigPacketStructure(&packet, &_uart_config);
        break;

    case ORION_PKT_GEOLOCATE_TELEMETRY:
        // Received at 10Hz
        decodeGeolocateTelemetryCorePacketStructure(&packet, &_telemetry_core);
//        if (get_mode() == MAV_MOUNT_MODE_TRACK) {
//            _state._roi_target.lat = Location(_telemetry_core.posLat, _telemetry_core.posLon, _telemetry_core.posAlt, Location::AltFrame::ABSOLUTE);
//        }
        break;

    case ORION_PKT_CAMERAS:
        break;

    case ORION_PKT_STARE_START:
        decodeStareStartPacketStructure(&packet, &_stare_start);
        break;

        // TODO: Implement either storing or debug printing these
    case ORION_PKT_CLEVIS_VERSION:
    case ORION_PKT_CROWN_VERSION:
    case ORION_PKT_PAYLOAD_VERSION:
    case ORION_PKT_TRACKER_VERSION:
    case ORION_PKT_LENSCTL_VERSION:
    case ORION_PKT_BOARD:
    case ORION_PKT_SOFTWARE_DIAGNOSTICS:
    case ORION_PKT_VIBRATION:
    case ORION_PKT_NETWORK_DIAGNOSTICS:
    case ORION_PKT_INITIALIZE:
    case ORION_PKT_CMD:
    case ORION_PKT_STARTUP_CMD:
    case ORION_PKT_AUTOPILOT_DATA:
    case ORION_PKT_STARE_ACK:
    case ORION_PKT_RANGE_DATA:


        // FULL LIST
    case ORION_PKT_LASER_CMD:
    case ORION_PKT_RESET:
    case ORION_PKT_PRIVATE_05:
    case ORION_PKT_LASER_STATES:
    case ORION_PKT_PRIVATE_08:
    case ORION_PKT_PRIVATE_09:
    case ORION_PKT_PRIVATE_1F:
    case ORION_PKT_PRIVATE_20:
    case ORION_PKT_PRIVATE_21:
    case ORION_PKT_LIMITS:
    case ORION_PKT_PRIVATE_23:
    case ORION_PKT_PRIVATE_24:
    case ORION_PKT_RESET_SOURCE:
    case ORION_PKT_PRIVATE_2A:
    case ORION_PKT_PRIVATE_2B:
    case ORION_PKT_PRIVATE_2D:
    case ORION_PKT_PRIVATE_40:
    case ORION_PKT_FAULTS:
    case ORION_PKT_PRIVATE_47:
    case ORION_PKT_PRIVATE_48:
    case ORION_PKT_PRIVATE_49:
    case ORION_PKT_CAMERA_SWITCH:
    case ORION_PKT_CAMERA_STATE:
    case ORION_PKT_NETWORK_VIDEO:
    case ORION_PKT_PRIVATE_64:
    case ORION_PKT_PRIVATE_65:
    case ORION_PKT_PRIVATE_66:
    case ORION_PKT_FLIR_SETTINGS:
    case ORION_PKT_APTINA_SETTINGS:
    case ORION_PKT_ZAFIRO_SETTINGS:
    case ORION_PKT_HITACHI_SETTINGS:
    case ORION_PKT_BAE_SETTINGS:
    case ORION_PKT_SONY_SETTINGS:
    case ORION_PKT_KTNC_SETTINGS:
    case ORION_PKT_PRIVATE_70:
    case ORION_PKT_PRIVATE_71:
    case ORION_PKT_PRIVATE_90:
    case ORION_PKT_PRIVATE_91:
    case ORION_PKT_PRIVATE_92:
    case ORION_PKT_RETRACT_CMD:
    case ORION_PKT_PRIVATE_B0:
    case ORION_PKT_USER_DATA:
    case ORION_PKT_KLV_USER_DATA:
    case ORION_PKT_PRIVATE_B4:
    case ORION_PKT_PRIVATE_C0:
    case ORION_PKT_PRIVATE_C1:
    case ORION_PKT_PRIVATE_C2:
    case ORION_PKT_PRIVATE_C3:
    case ORION_PKT_PRIVATE_CE:
    case ORION_PKT_PRIVATE_CF:
    case ORION_PKT_PRIVATE_D0:
    case ORION_PKT_GPS_DATA:
    case ORION_PKT_EXT_HEADING_DATA:
    case ORION_PKT_INS_QUALITY:
    case ORION_PKT_GEOPOINT_CMD:
    case ORION_PKT_PATH:
    case ORION_PKT_INS_OPTIONS:
    case ORION_PKT_PRIVATE_DB:
    case ORION_PKT_PRIVATE_DE:
    case ORION_PKT_PRIVATE_DF:
    case ORION_PKT_PRIVATE_E0:
    case ORION_PKT_PRIVATE_E1:
    case ORION_PKT_PRIVATE_E2:
    case ORION_PKT_PRIVATE_E3:
    case ORION_PKT_PRIVATE_E5:
    case ORION_PKT_PRIVATE_E6:
    case ORION_PKT_PRIVATE_E7:
    case ORION_PKT_PRIVATE_E8:
    case ORION_PKT_PRIVATE_E9:
    case ORION_PKT_PRIVATE_EA:
    case ORION_PKT_PRIVATE_F0:
    case ORION_PKT_PRIVATE_F1:
    case ORION_PKT_PRIVATE_F2:
    case ORION_PKT_PRIVATE_F3:
    case ORION_PKT_PRIVATE_F4:
    case ORION_PKT_PRIVATE_F5:
    case ORION_PKT_PRIVATE_F6:
    case ORION_PKT_PRIVATE_F7:
    default:
        // unhandled
#if AP_MOUNT_TRILLIUM_DEBUG_RX_UNHANDLED_MSGS
        gcs().send_text(MAV_SEVERITY_DEBUG, "%sunhandled 0x%02x,%3u:%s", _trilliumGcsHeader, packet.ID, packet.ID, get_packet_name(packet.ID));
#endif
        break;
    }
}

// send_mount_status - called to allow mounts to send their status to GCS using the MOUNT_STATUS message
void AP_Mount_Trillium::send_mount_status(mavlink_channel_t chan)
{
    if (!_booting.done) {
        return;
    }

    // return target angles as gimbal's actual attitude.
    mavlink_msg_mount_status_send(chan, 0, 0, _current_angle_deg.y, _current_angle_deg.x, _current_angle_deg.z);
}

void AP_Mount_Trillium::requestOrionMessageByID(uint8_t id)
{
    OrionPkt_t PktOut = {};
    MakeOrionPacket(&PktOut, id, 0);
    OrionCommSend(&PktOut);

}

/*
 pass a raw packet from mavlink to Trillium Serial interface
*/
void AP_Mount_Trillium::handle_passthrough(const mavlink_channel_t chan, const mavlink_passthrough_t &packet)
{
    if (packet.device != AP_MOUNT_TRILLIUM_MAVLINK_PASSTHROUGH_DEVICE) {
        return;
    }

    if (_port == nullptr || _port->txspace() < packet.size) {
        return;
    }

    const uint32_t sent = _port->write(packet.payload, packet.size);

    if (sent != packet.size) {
#if AP_MOUNT_TRILLIUM_DEBUG_TX_PASSTHROUGH_DROPS
        gcs().send_text(MAV_SEVERITY_DEBUG, "%spassthrough truncated", _trilliumGcsHeader);
#endif
    }

    // store time of send
    const uint32_t now_ms = AP_HAL::millis();
    _last_send_ms = now_ms;
    _passthrough.last_MAVLink_to_gimbal_ms = now_ms;
    _passthrough.chan = chan;
}

size_t AP_Mount_Trillium::OrionCommSend(const OrionPkt_t *pPkt)
{
    bool debug = AP_MOUNT_TRILLIUM_DEBUG_TX_ALL_MSGS;

#if AP_MOUNT_TRILLIUM_DEBUG_TX_CMD_CHANGE_ONLY_MSGS
    if (pPkt->ID == ORION_PKT_CMD) {
        debug = false;
        OrionCmd_t orionCmd_new {};
        decodeOrionCmdPacket(&pPkt, &orionCmd_new);
        if (orionCmd_new.Mode != _telemetry_core.mode) {
            gcs().send_text(MAV_SEVERITY_DEBUG, "%s Changing mode %s to %s", _trilliumGcsHeader, get_mode_name(_telemetry_core.mode), get_mode_name(orionCmd_new.Mode));
            _telemetry_core.mode = orionCmd_new.Mode;
        }
    }
#endif

#if AP_MOUNT_TRILLIUM_DEBUG_TX_NOT_AUTOPILOT_MSGS
    if (pPkt->ID == ORION_PKT_AUTOPILOT_DATA) {
        debug = false;
    }
#endif

    if (debug) {
        gcs().send_text(MAV_SEVERITY_DEBUG, "%sTx (0x%02x,%3u): %s", _trilliumGcsHeader, pPkt->ID, pPkt->ID, get_packet_name(pPkt->ID));
    }

    const uint32_t len = pPkt->Length + ORION_PKT_OVERHEAD;

#if AP_MOUNT_TRILLIUM_SITL_USE_IP && (CONFIG_HAL_BOARD == HAL_BOARD_SITL)
    if (sock_connected) {
        return sock.send((uint8_t *)pPkt, len);
    } else {
        return 0;
    }
#else
    return _port->write((uint8_t *)pPkt, len);
#endif
}

const char* AP_Mount_Trillium::get_mode_name(uint8_t mode)
{
    switch (mode) {
    case ORION_MODE_DISABLED:       return "DISABLED";

    case ORION_MODE_FAULT:          return "FAULT";
    case ORION_MODE_RATE:           return "RATE";
    case ORION_MODE_GEO_RATE:       return "GEO_RATE";
    case ORION_MODE_FFC_AUTO:       return "FFC_AUTO";
    //case ORION_MODE_FFC:            return "FFC";
    case ORION_MODE_FFC_MANUAL:     return "FFC_MANUAL";
    case ORION_MODE_SCENE:          return "SCENE";
    case ORION_MODE_TRACK:          return "TRACK";
    case ORION_MODE_CALIBRATION:    return "CALIBRATION";
    case ORION_MODE_POSITION:       return "POSITION";
    case ORION_MODE_POSITION_NO_LIMITS: return "POS_NO_LIMITS";
    case ORION_MODE_GEOPOINT:       return "GEOPOINT";
    case ORION_MODE_PATH:           return "PATH";
    case ORION_MODE_DOWN:           return "DOWN";
    case ORION_MODE_UNKNOWN:        return "UNKNOWN";

    default:
        return "Unknown";
    }

}

const char* AP_Mount_Trillium::get_packet_name(uint8_t id)
{
    switch (id) {
    case ORION_PKT_DEBUG_STRING:        return "DEBUG_STRING";
    case ORION_PKT_RETRACT_STATUS:      return "RETRACT_STATUS";
    case ORION_PKT_NETWORK_SETTINGS:    return "NETWORK_SETTINGS";
    case ORION_PKT_DIAGNOSTICS:         return "DIAGNOSTICS";
    case ORION_PKT_PERFORMANCE:         return "PERFORMANCE";
    case ORION_PKT_UART_CONFIG:         return "UART_CONFIG";
    case ORION_PKT_GEOLOCATE_TELEMETRY: return "GEOLOCATE_TELEMETRY";
    case ORION_PKT_CAMERAS:             return "CAMERAS";
    case ORION_PKT_CLEVIS_VERSION:      return "CLEVIS_VERSION";
    case ORION_PKT_CROWN_VERSION:       return "CROWN_VERSION";
    case ORION_PKT_PAYLOAD_VERSION:     return "PAYLOAD_VERSION";
    case ORION_PKT_TRACKER_VERSION:     return "TRACKER_VERSION";
    case ORION_PKT_LENSCTL_VERSION:     return "LENSCTL_VERSION";
    case ORION_PKT_BOARD:               return "BOARD";
    case ORION_PKT_SOFTWARE_DIAGNOSTICS:return "SOFTWARE_DIAGNOSTICS";
    case ORION_PKT_VIBRATION:           return "VIBRATION";
    case ORION_PKT_NETWORK_DIAGNOSTICS: return "NETWORK_DIAGNOSTICS";
    case ORION_PKT_INITIALIZE:          return "INITIALIZE";
    case ORION_PKT_CMD:                 return "CMD";
    case ORION_PKT_STARTUP_CMD:         return "STARTUP_CMD";
    case ORION_PKT_AUTOPILOT_DATA:      return "AUTOPILOT_DATA";
    case ORION_PKT_STARE_START:         return "STARE_START";
    case ORION_PKT_STARE_ACK:           return "STARE_ACK";
    case ORION_PKT_RANGE_DATA:          return "RANGE_DATA";
    case ORION_PKT_LASER_CMD:           return "LASER_CMD";
    case ORION_PKT_RESET:               return "RESET";
    case ORION_PKT_LASER_STATES:        return "LASER_STATES";
    case ORION_PKT_LIMITS:              return "LIMITS";
    case ORION_PKT_RESET_SOURCE:        return "RESET_SOURCE";
    case ORION_PKT_FAULTS:              return "FAULTS";
    case ORION_PKT_CAMERA_SWITCH:       return "CAMERA_SWITCH";
    case ORION_PKT_CAMERA_STATE:        return "CAMERA_STATE";
    case ORION_PKT_NETWORK_VIDEO:       return "NETWORK_VIDEO";
    case ORION_PKT_FLIR_SETTINGS:       return "FLIR_SETTINGS";
    case ORION_PKT_APTINA_SETTINGS:     return "APTINA_SETTINGS";
    case ORION_PKT_ZAFIRO_SETTINGS:     return "ZAFIRO_SETTINGS";
    case ORION_PKT_HITACHI_SETTINGS:    return "HITACHI_SETTINGS";
    case ORION_PKT_BAE_SETTINGS:        return "BAE_SETTINGS";
    case ORION_PKT_SONY_SETTINGS:       return "SONY_SETTINGS";
    case ORION_PKT_KTNC_SETTINGS:       return "KTNC_SETTINGS";
    case ORION_PKT_RETRACT_CMD:         return "RETRACT_CMD";
    case ORION_PKT_USER_DATA:           return "USER_DATA";
    case ORION_PKT_KLV_USER_DATA:       return "KLV_USER_DATA";
    case ORION_PKT_GPS_DATA:            return "GPS_DATA";
    case ORION_PKT_EXT_HEADING_DATA:    return "EXT_HEADING_DATA";
    case ORION_PKT_INS_QUALITY:         return "INS_QUALITY";
    case ORION_PKT_GEOPOINT_CMD:        return "GEOPOINT_CMD";
    case ORION_PKT_PATH:                return "PATH";
    case ORION_PKT_INS_OPTIONS:         return "INS_OPTIONS";

    case ORION_PKT_PRIVATE_05:
    case ORION_PKT_PRIVATE_08:
    case ORION_PKT_PRIVATE_09:
    case ORION_PKT_PRIVATE_1F:
    case ORION_PKT_PRIVATE_20:
    case ORION_PKT_PRIVATE_21:
    case ORION_PKT_PRIVATE_23:
    case ORION_PKT_PRIVATE_24:
    case ORION_PKT_PRIVATE_2A:
    case ORION_PKT_PRIVATE_2B:
    case ORION_PKT_PRIVATE_2D:
    case ORION_PKT_PRIVATE_40:
    case ORION_PKT_PRIVATE_47:
    case ORION_PKT_PRIVATE_48:
    case ORION_PKT_PRIVATE_49:
    case ORION_PKT_PRIVATE_64:
    case ORION_PKT_PRIVATE_65:
    case ORION_PKT_PRIVATE_66:
    case ORION_PKT_PRIVATE_70:
    case ORION_PKT_PRIVATE_71:
    case ORION_PKT_PRIVATE_90:
    case ORION_PKT_PRIVATE_91:
    case ORION_PKT_PRIVATE_92:
    case ORION_PKT_PRIVATE_B0:
    case ORION_PKT_PRIVATE_B4:
    case ORION_PKT_PRIVATE_C0:
    case ORION_PKT_PRIVATE_C1:
    case ORION_PKT_PRIVATE_C2:
    case ORION_PKT_PRIVATE_C3:
    case ORION_PKT_PRIVATE_CE:
    case ORION_PKT_PRIVATE_CF:
    case ORION_PKT_PRIVATE_D0:
    case ORION_PKT_PRIVATE_DB:
    case ORION_PKT_PRIVATE_DE:
    case ORION_PKT_PRIVATE_DF:
    case ORION_PKT_PRIVATE_E0:
    case ORION_PKT_PRIVATE_E1:
    case ORION_PKT_PRIVATE_E2:
    case ORION_PKT_PRIVATE_E3:
    case ORION_PKT_PRIVATE_E5:
    case ORION_PKT_PRIVATE_E6:
    case ORION_PKT_PRIVATE_E7:
    case ORION_PKT_PRIVATE_E8:
    case ORION_PKT_PRIVATE_E9:
    case ORION_PKT_PRIVATE_EA:
    case ORION_PKT_PRIVATE_F0:
    case ORION_PKT_PRIVATE_F1:
    case ORION_PKT_PRIVATE_F2:
    case ORION_PKT_PRIVATE_F3:
    case ORION_PKT_PRIVATE_F4:
    case ORION_PKT_PRIVATE_F5:
    case ORION_PKT_PRIVATE_F6:
    case ORION_PKT_PRIVATE_F7:
        return "PRIVATE";

    default:
        return "Unknown";
    }
}

MAV_RESULT AP_Mount_Trillium::custom(const mavlink_command_long_t &packet)
{
    if (packet.command != SPECIAL_MAVLINK_LONG_ID_MOUNT_CUSTOM) {
        return MAV_RESULT_UNSUPPORTED;
    }
    // param1 = bitmask of valid params where bit 1 is param2
    // param2 = mode
    // param3 = zoom
    // param4 = Sensor type
    // param5 = pan
    // param6 = tilt
    // param7 = unused

    if (isnan(packet.param1)) {
        return MAV_RESULT_FAILED;
    }

    const uint8_t bitmask = packet.param1;
    OrionPkt_t PktOut;

    if (!isnan(packet.param2) && (bitmask & (1 << 0))) {
        _desiredMode = (OrionMode_t)packet.param2;
        gcs().send_text(MAV_SEVERITY_DEBUG, "%sGCS Set to mode %s", _trilliumGcsHeader, get_mode_name(_desiredMode));

        _cmd_last.Mode = _desiredMode;

        encodeOrionCmdPacket(&PktOut, &_cmd_last);
        OrionCommSend(&PktOut);
    }

    if (!isnan(packet.param3) && (bitmask & (1 << 1))) {
        // zoom (+1, -1)
        _zoom[_selected_camera] = constrain_int32(_zoom[_selected_camera] + (packet.param3 * 10), -32767, 32767);
        gcs().send_text(MAV_SEVERITY_DEBUG, "%sGCS Set zoom %d", _trilliumGcsHeader, _zoom[_selected_camera]);

        encodeOrionCameraStatePacket(&PktOut, _zoom[_selected_camera], -1, _selected_camera);
        OrionCommSend(&PktOut);
    }

    if (!isnan(packet.param4) && (bitmask & (1 << 2))) {
        // sensor type
        // 0 - optical
        // 1 - IR
        _selected_camera = constrain_int16(packet.param4, 0, 1);
        gcs().send_text(MAV_SEVERITY_DEBUG, "%sGCS Set sensor: %s", _trilliumGcsHeader, (_selected_camera == 0) ? "Optical" : "IR");

        encodeOrionCameraSwitchPacket(&PktOut, _selected_camera);
        OrionCommSend(&PktOut);
    }

    if (!isnan(packet.param5) && !isnan(packet.param6) && (bitmask & (1 << 3)) && (bitmask & (1 << 4))) {
        // pan, tilt
        gcs().send_text(MAV_SEVERITY_DEBUG, "%sGCS Set pan, tilt: %.1f, %.1f", _trilliumGcsHeader, packet.param5, packet.param6);
    }
    if (!isnan(packet.param7) && (bitmask & (1 << 5))) {
    }

    return MAV_RESULT_ACCEPTED;
}
#endif // MOUNT_TRILLIUM_ENABLE

