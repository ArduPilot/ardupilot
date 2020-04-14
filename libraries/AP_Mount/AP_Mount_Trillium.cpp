#include "AP_Mount_Trillium.h"

#if AP_MOUNT_TRILLIUM_ENABLED

#include <GCS_MAVLink/GCS.h>
#include <AP_SerialManager/AP_SerialManager.h>
#include <SRV_Channel/SRV_Channel.h>

extern const AP_HAL::HAL& hal;

#define AP_MOUNT_TRILLIUM_DEBUG_RX_ALL_MSGS                 0
#define AP_MOUNT_TRILLIUM_DEBUG_RX_UNHANDLED_MSGS           0
#define AP_MOUNT_TRILLIUM_DEBUG_TX_ALL_MSGS                 0
#define AP_MOUNT_TRILLIUM_DEBUG_TX_PASSTHROUGH_DROPS        0

#define AP_MOUNT_TRILLIUM_MAVLINK_PASSTHROUGH_DEVICE        13
#define AP_MOUNT_TRILLIUM_MAVLINK_PASSTHROUGH_ENABLE        1

#define AP_MOUNT_TRILLIUM_SET_ETHERNET_SETTINGS             0

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

    // update based on mount mode
    switch(get_mode()) {
            // move mount to a "retracted" position.  we do not implement a separate servo based retract mechanism
        case MAV_MOUNT_MODE_RETRACT:
            _angle_ef_target_rad = _state._retract_angles.get() * DEG_TO_RAD;
            if (SRV_Channels::function_assigned(SRV_Channel::k_mount_open)) {
                const bool gimbal_is_pointed_in_stow_orientation = true;
                if (gimbal_is_pointed_in_stow_orientation) {
                    SRV_Channels::set_output_to_min(SRV_Channel::k_mount_open);
                }

            } else if (_retract_status.State == RETRACT_STATE_DEPLOYED || _retract_status.State == RETRACT_STATE_DEPLOYING) {
                encodeOrionRetractCommandPacket(&PktOut, RETRACT_CMD_RETRACT);
                OrionCommSend(&PktOut);
            }
            break;

        // move mount to a neutral position, typically pointing forward
        case MAV_MOUNT_MODE_NEUTRAL:
            _angle_ef_target_rad = _state._neutral_angles.get() * DEG_TO_RAD;
            if (SRV_Channels::function_assigned(SRV_Channel::k_mount_open)) {
                SRV_Channels::set_output_to_max(SRV_Channel::k_mount_open);

            } else if (_retract_status.State == RETRACT_STATE_RETRACTED || _retract_status.State == RETRACT_STATE_RETRACTING) {
                encodeOrionRetractCommandPacket(&PktOut, RETRACT_CMD_DEPLOY);
                OrionCommSend(&PktOut);
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
                double targetLat = deg2rad(_state._target_sysid_location.lat * 1.0e-7f);
                double targetLon = deg2rad(_state._target_sysid_location.lng * 1.0e-7f);
                double targetAlt = _state._target_sysid_location.alt * 0.01f;    // cm -> m
                float targetVelNed[] = { 0.0, 0.0, 0.0 };

                encodeGeopointCmdPacket(&PktOut, targetLat, targetLon, targetAlt, targetVelNed, 0, geopointOptions::geopointNone);
                OrionCommSend(&PktOut);

            } else if (calc_angle_to_sysid_target(_angle_ef_target_rad, true, true)) {
                resend_now = true;
            }
            break;

        default:
            // we do not know this mode so do nothing
            break;
    }


    if (resend_now) {
        send_target_angles(_angle_ef_target_rad, false);
    }
}

/*
 * detect and read the header of the incoming message from the gimbal
 */
void AP_Mount_Trillium::read_incoming()
{
    if (_port == nullptr) {
        return;
    }

    int16_t num_available = _port->available();

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
        const uint8_t rxByte = _port->read();

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
    OrionPkt_t pPkt;
    OrionCmd_t Cmd = {};

    // Form a command that will tell the gimbal to move at 10 deg/s in pan for one second
    if (target_in_degrees) {
        Cmd.Target[GIMBAL_AXIS_PAN]  = deg2radf(angle.x);
        Cmd.Target[GIMBAL_AXIS_TILT] = deg2radf(angle.y);
    } else {
        Cmd.Target[GIMBAL_AXIS_PAN]  = angle.x;
        Cmd.Target[GIMBAL_AXIS_TILT] = angle.y;
    }

    Cmd.Mode = ORION_MODE_RATE;
    Cmd.ImpulseTime = 1.0f;
    Cmd.Stabilized = FALSE;

    encodeOrionCmdPacket(&pPkt, &Cmd);
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
    gcs().send_text(MAV_SEVERITY_DEBUG, "%sRx msg id: %u", _trilliumGcsHeader, packet.ID);
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
        break;

    case ORION_PKT_CAMERAS:
        break;

        // TODO: Implement eithe rstoring or debug printing these
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
    case ORION_PKT_STARE_START:
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
        gcs().send_text(MAV_SEVERITY_DEBUG, "%sunhandled msg id: %u", _trilliumGcsHeader, packet.ID);
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
#if AP_MOUNT_TRILLIUM_DEBUG_TX_ALL_MSGS
    gcs().send_text(MAV_SEVERITY_DEBUG, "%stx cmd ID: %u", _trilliumGcsHeader, pPkt->ID);
#endif

    const uint32_t len = pPkt->Length + ORION_PKT_OVERHEAD;

}
#endif // MOUNT_TRILLIUM_ENABLE

