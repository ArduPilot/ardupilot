#include "AP_Mount_Trillium.h"

#if AP_MOUNT_TRILLIUM_ENABLED

#include <GCS_MAVLink/GCS.h>
#include <AP_SerialManager/AP_SerialManager.h>

extern const AP_HAL::HAL& hal;

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
    if (_booting.done || _booting.retries >= 3) {
        return;
    }

    const uint32_t now_ms = AP_HAL::millis();
    if (now_ms - _booting.timestamp_ms < _booting.duration_ms) {
        return;
    }

    if (_booting.rx_expected_cmd_id || _booting.rx_expected_ack_id) {
        // time has expired and this was supposed to be cleared by now. so thats a boot failure - no response.
        // backup retries, reset booting sequence, restore retries count. If we reach max retries then we stop retrying and stay in done=false state forever
        const uint8_t retries = _booting.retries;
        memset(&_booting, 0, sizeof(_booting));
        _booting.retries = retries;
    }

    _booting.timestamp_ms = now_ms;

    uint8_t cmd_id = 0;
    bool expect_ack = AP_MOUNT_TRILLIUM_REQUIRE_ACKS;
    OrionPkt_t Pkt;

    switch (_booting.step++) {
    case 0:
        _booting.duration_ms = 200;
        // Build a version request packet (note that it doesn't matter what you send...)
        MakeOrionPacket(&Pkt, ORION_PKT_CROWN_VERSION, 0);
        OrionCommSend(&Pkt);
        break;

//
//        // I'm not sure this acks or not, so lets just not expect an ack for this.
//        // If it's false then we'll move on. If its true and we don't get an ack then this step will always fail
//        expect_ack = false;
//
//        cmd_id = AP_MOUNT_TRILLIUM_ID_ENABLE_MESSAGE_ACK;
//        send_command(cmd_id, AP_MOUNT_TRILLIUM_REQUIRE_ACKS, AP_MOUNT_TRILLIUM_REQUIRE_ACKS);   // 1,1 means gimbal will ACK all packets we send it
//        break;
//
//    case 1:
//        _booting.duration_ms = 5000;
//        cmd_id = AP_MOUNT_TRILLIUM_ID_INITILISE;
//        send_command(cmd_id, 1, 1); // (1,1) means auto-initialize
//        break;
//
//    case 2:
//        _booting.duration_ms = 5000;
//        cmd_id = AP_MOUNT_TRILLIUM_ID_ENABLE_STREAM_MODE;
//        send_command(cmd_id, 2, AP_MOUNT_TRILLIUM_CURRENT_POS_STREAM_RATE_HZ);
//        break;

    case 3:
    case 4:
    case 5:
    case 6:
    case 7:
    case 8:
    case 9:
    case 10:
        // TODO: add more hw init commands to complete the hw boot-up process without a factory-reseted device ever needing to connect to the company-provided software
    default:
        _booting.done = true;
        break;
    }

    if (expect_ack) {
        _booting.rx_expected_ack_id = cmd_id;
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

    // update based on mount mode
    switch(get_mode()) {
            // move mount to a "retracted" position.  we do not implement a separate servo based retract mechanism
        case MAV_MOUNT_MODE_RETRACT:
            _angle_ef_target_rad = _state._retract_angles.get() * DEG_TO_RAD;
            if (_stow_status != AP_MOUNT_TRILLIUM_STOW_STATE_EXIT_or_NOT_STOWED) {
                _stow_status = AP_MOUNT_TRILLIUM_STOW_STATE_EXIT_or_NOT_STOWED;
                send_command(AP_MOUNT_TRILLIUM_ID_STOW_MODE, _stow_status, 0);
            }
            break;

        // move mount to a neutral position, typically pointing forward
        case MAV_MOUNT_MODE_NEUTRAL:
            _angle_ef_target_rad = _state._neutral_angles.get() * DEG_TO_RAD;
            if (_stow_status != AP_MOUNT_TRILLIUM_STOW_STATE_ENTER_or_DO_STOW) {
                _stow_status = AP_MOUNT_TRILLIUM_STOW_STATE_ENTER_or_DO_STOW;
                send_command(AP_MOUNT_TRILLIUM_ID_STOW_MODE, _stow_status, 0);
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
            if (calc_angle_to_roi_target(_angle_ef_target_rad, true, true)) {
                resend_now = true;
            }
            break;

        case MAV_MOUNT_MODE_SYSID_TARGET:
            if (calc_angle_to_sysid_target(_angle_ef_target_rad, true, true)) {
                resend_now = true;
            }
            break;

        default:
            // we do not know this mode so do nothing
            break;
    }

    // resend target angles at least once per second
    resend_now = resend_now || ((AP_HAL::millis() - _last_send) > AP_MOUNT_TRILLIUM_SERIAL_MINIMUM_INTERVAL_MS);

    if (resend_now) {
        send_target_angles(_angle_ef_target_rad, false);
    }

    if (_stab_pan != _state._stab_pan || _stab_tilt != _state._stab_tilt) {
        _stab_pan = _state._stab_pan;
        _stab_tilt = _state._stab_tilt;
        send_command(AP_MOUNT_TRILLIUM_ID_ENABLE_GYRO_STABILISATION, _stab_pan, _stab_tilt);
    }

}
void AP_Mount_Trillium::send_target_angles(Vector3f angle, bool target_in_degrees)
{
    // convert to degrees if necessary
    Vector3f target_deg = angle;
    if (!target_in_degrees) {
        target_deg *= RAD_TO_DEG;
    }
    send_target_angles(target_deg.x, target_deg.y, target_deg.z);
}

// send_target_angles
void AP_Mount_Trillium::send_target_angles(float pitch_deg, float roll_deg, float yaw_deg)
{
    // datasheet section 3.2.1
    // encode float degrees to encoded uint16_t derived by U16_VALUE = angle * (32768 / 360)
    const float degree_to_encoded_U16 = 91.02222;

    const uint16_t pan = pitch_deg * degree_to_encoded_U16;
    const uint16_t tilt = yaw_deg * degree_to_encoded_U16;

    send_command(AP_MOUNT_TRILLIUM_ID_SET_PAN_TILT_POSITION,
            pan >> 8,
            pan,
            tilt >> 8,
            tilt);
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

    while (num_available-- > 0) {        // Process bytes received
        const uint8_t rxByte = _port->read();

        if (LookForOrionPacketInByte(&_PktIn, rxByte)) {
            handle_packet(_PktIn);
        }
    }
}

const char *AP_Mount_Trillium::get_model_name(const uint8_t gimbal_model_flags)
{
    switch (gimbal_model_flags) {
    case 0x13: return "HD40";
    default:   return "?????";
    }
}


void AP_Mount_Trillium::handle_packet(OrionPkt_t &packet)
{
    const uint8_t len = packet.Length;
    uint16_t index = 0;

    switch (packet.ID) {
    case ORION_PKT_DEBUG_STRING:
        const char* trilliumName = "Trillium: ";

        DebugString_t msg;
        decodeDebugStringPacketStructure(&packet, &msg);

        while (index <= len) {
            gcs().send_text(MAV_SEVERITY_DEBUG, "%s%s", trilliumName, (char*)&msg.description);
            index += (MAVLINK_MSG_STATUSTEXT_FIELD_TEXT_LEN - sizeof(trilliumName));
        }
        break;
    }

    if (_booting.rx_expected_cmd_id != 0 && packet.ID == _booting.rx_expected_cmd_id) {
        // expected packet received! Forget it because we should have handled it in the above switch
        _booting.rx_expected_cmd_id = 0;

        // clear the duration so we immediately continue booting after handling the expected packet
        _booting.duration_ms = 0;
    }
}

void AP_Mount_Trillium::handle_ack()
{
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

/*
 send a command to the Trillium Serial API
*/
void AP_Mount_Trillium::send_command(const uint8_t cmd, const uint8_t* data, const uint8_t size)
{
    if (_port == nullptr || (_port->txspace() < (size + 5U))) {
        return;
    }

    _port->write(data, size);

    // store time of send
    _last_send = AP_HAL::millis();
}

void AP_Mount_Trillium::handle_passthrough(const mavlink_channel_t chan, const mavlink_passthrough_t &packet)
{
    const uint8_t size = packet.payload[2];
    const uint8_t cmd = packet.payload[3];
    const uint8_t* data = &packet.payload[4];

    send_command(cmd, data, size);
}

size_t AP_Mount_Trillium::OrionCommSend(const OrionPkt_t *pPkt)
{
    return _port->write((uint8_t *)pPkt, pPkt->Length + ORION_PKT_OVERHEAD);

}
#endif // MOUNT_TRILLIUM_ENABLE

