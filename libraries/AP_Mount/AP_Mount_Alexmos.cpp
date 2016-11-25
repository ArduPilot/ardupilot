#include "AP_Mount_Alexmos.h"

extern const AP_HAL::HAL& hal;

void AP_Mount_Alexmos::init(const AP_SerialManager& serial_manager)
{
    // check for alexmos protcol
    if ((_port = serial_manager.find_serial(AP_SerialManager::SerialProtocol_AlexMos, 0))) {
        _initialised = true;
        get_boardinfo();
        read_params(0); //we request parameters for profile 0 and therfore get global and profile parameters
    }
}

// update mount position - should be called periodically
void AP_Mount_Alexmos::update()
{
    if (!_initialised) {
        return;
    }

    read_incoming(); // read the incoming messages from the gimbal

    // update based on mount mode
    switch(get_mode()) {
        // move mount to a "retracted" position.  we do not implement a separate servo based retract mechanism
        case MAV_MOUNT_MODE_RETRACT:
            control_axis(_state._retract_angles.get(), true);
            break;

        // move mount to a neutral position, typically pointing forward
        case MAV_MOUNT_MODE_NEUTRAL:
            control_axis(_state._neutral_angles.get(), true);
            break;

        // point to the angles given by a mavlink message
        case MAV_MOUNT_MODE_MAVLINK_TARGETING:
            control_axis(_angle_ef_target_rad, false);
            break;

        // RC radio manual angle control, but with stabilization from the AHRS
        case MAV_MOUNT_MODE_RC_TARGETING:
            // update targets using pilot's rc inputs
            update_targets_from_rc();
            control_axis(_angle_ef_target_rad, false);
            break;

        // point mount to a GPS point given by the mission planner
        case MAV_MOUNT_MODE_GPS_POINT:
            if(_frontend._ahrs.get_gps().status() >= AP_GPS::GPS_OK_FIX_2D) {
                calc_angle_to_location(_state._roi_target, _angle_ef_target_rad, true, true, false);
                control_axis(_angle_ef_target_rad, false);
            }
            break;

        default:
            // we do not know this mode so do nothing
            break;
    }
}

// has_pan_control - returns true if this mount can control it's pan (required for multicopters)
bool AP_Mount_Alexmos::has_pan_control() const
{
    return _gimbal_3axis;
}

// set_mode - sets mount's mode
void AP_Mount_Alexmos::set_mode(enum MAV_MOUNT_MODE mode)
{
    // record the mode change and return success
    _state._mode = mode;
}

// status_msg - called to allow mounts to send their status to GCS using the MOUNT_STATUS message
void AP_Mount_Alexmos::status_msg(mavlink_channel_t chan)
{
    if (!_initialised) {
        return;
    }

    get_angles();
    mavlink_msg_mount_status_send(chan, 0, 0, _current_angle.y*100, _current_angle.x*100, _current_angle.z*100);
}

/*
 * get_angles
 */
void AP_Mount_Alexmos::get_angles()
{
    uint8_t data[1] = {(uint8_t)1};
    send_command(CMD_GET_ANGLES_EXT, data, 1);
}

/*
 * set_data_stream_interval
 */
void AP_Mount_Alexmos::set_data_stream_interval()
{
    alexmos_parameters outgoing_buffer;

    outgoing_buffer.dstream_interval.cmd_id = CMD_REALTIME_DATA_CUSTOM;
    outgoing_buffer.dstream_interval.interval_ms = 1250; // cycles x 0.8msec = 1sec

    outgoing_buffer.dstream_interval.config = (uint32_t)0x40; 
    // bit3<<24: translate from camera to frame  : bit6: IMU attitude as RotMat

    outgoing_buffer.dstream_interval.res0 = 0;
    outgoing_buffer.dstream_interval.res1 = 0;
    send_command(CMD_DATA_STREAM_INTERVAL,
                    (uint8_t *)&outgoing_buffer.dstream_interval,
                    sizeof(alexmos_data_stream_interval));
}

/*
 *  set_ahrs_helper
 */
void AP_Mount_Alexmos::set_ahrs_helper(uint8_t mode, const Vector3f& zenith, const Vector3f& north)
{

    alexmos_parameters outgoing_buffer;
    outgoing_buffer.ahrs_helper.mode = mode;
    for(uint8_t i = 0; i<3; i++)
    {
        outgoing_buffer.ahrs_helper.zenith[i] = zenith[i];
        outgoing_buffer.ahrs_helper.north[i] = north[i];
    }
    send_command(CMD_AHRS_HELPER, (uint8_t *)&outgoing_buffer.ahrs_helper, sizeof(alexmos_ahrs_helper));
}

/*
 * To be called at regular intervals as soon as current imu and encoder angles get updated
 */
float AP_Mount_Alexmos::compensate_mount_imu(uint8_t mntCal_mode)
{
    // AHRS DCM row a corresponds to north direction in vehicle body frame
    // AHRS DCM row c corresponds to nadir direction in vehicle body frame
    // NOTE: Alexmos uses (East, North, Up) frame while APM uses NED
    const Matrix3f& dcm = _frontend._ahrs.get_rotation_body_to_ned();
    Vector3f north = Vector3f(dcm.a.y, dcm.a.x, -dcm.a.z);   //  North in (right, front, up)
    Vector3f zenith = Vector3f(-dcm.c.y, -dcm.c.x, dcm.c.z); // Zenith in (right, front, up)

    float sin_phi = sinf(_current_stat_rot_angle.x*DEG_TO_RAD);
    float cos_phi = cosf(_current_stat_rot_angle.x*DEG_TO_RAD);
    float sin_tht = sinf(_current_stat_rot_angle.y*DEG_TO_RAD);
    float cos_tht = cosf(_current_stat_rot_angle.y*DEG_TO_RAD);
    float sin_ksi = sinf(_current_stat_rot_angle.z*DEG_TO_RAD);
    float cos_ksi = cosf(_current_stat_rot_angle.z*DEG_TO_RAD);

    Matrix3f f2m;

    f2m.a.x =  cos_phi*cos_ksi;
    f2m.b.x = -sin_phi*sin_tht*cos_ksi + cos_tht*sin_ksi;
    f2m.c.x =  sin_phi*cos_tht*cos_ksi + sin_tht*sin_ksi;

    f2m.a.y = -cos_phi*sin_ksi;
    f2m.b.y =  sin_phi*sin_tht*sin_ksi + cos_tht*cos_ksi;
    f2m.c.y = -sin_phi*cos_tht*sin_ksi + sin_tht*cos_ksi;

    f2m.a.z = -sin_phi;
    f2m.b.z = -cos_phi*sin_tht;
    f2m.c.z =  cos_phi*cos_tht;

    Vector3f north_m = f2m*north;           //  North direction in IMU coords
    Vector3f zenith_m = f2m*zenith;         // Zenith direction in IMU coords

    // below section translates (East, North, Up) to (East, North, Down) [in IMU frame]
    // i.e. north & zenith from (right, front, up) to (right, front, down)
    north_m.z = -north_m.z;
    zenith_m.x = -zenith_m.x;
    zenith_m.y = -zenith_m.y;

    /* NOTE: simply removing minus signs from the 3 eqtn. above, as well as
     * from north & zenith vector definitions from the start of this section WILL NOT WORK
     */

    switch (mntCal_mode) {
        case AP_MOUNT_ALEXMOS_COMPENSATE_NORTH:
            // H1_RAW: compensate NORTH once
            set_ahrs_helper(AP_MOUNT_ALEXMOS_AHRS_HLR_SET_H1_RAW, zenith_m, north_m);
            break;

        case AP_MOUNT_ALEXMOS_COMPENSATE_ZENITH:
            // Z1_RAW: compensate ZENITH once
            set_ahrs_helper(AP_MOUNT_ALEXMOS_AHRS_HLR_SET_Z1_RAW, zenith_m, north_m);
            break;

        case AP_MOUNT_ALEXMOS_COMPENSATE_BOTH:
            // RAW: compensate NORTH & ZENITH once
            set_ahrs_helper(AP_MOUNT_ALEXMOS_AHRS_HLR_SET_RAW, zenith_m, north_m);
            break;

        case AP_MOUNT_ALEXMOS_COMPENSATE_NO_OP:
            // fall through
        default:
            break;
    }

    return wrap_180(_current_angle.z - _current_stat_rot_angle.z - _frontend._ahrs.yaw*RAD_TO_DEG);
}

/*
 *
 */
void AP_Mount_Alexmos::update_target_2x720(Vector3f& current_target, const Vector3f& new_target, bool is_earth_fixed, bool invert_pitch)
{
    current_target.x = new_target.x;
    current_target.y = invert_pitch ? -new_target.y : new_target.y;

    if (!is_earth_fixed) {
        // use last yaw encoder data to get to the closest relative position
        current_target.z = _current_stat_rot_angle.z;
    }

    float yaw_error = wrap_180(new_target.z - current_target.z);
    current_target.z = wrap_2x720(current_target.z + yaw_error);
}

/*
 * set_motor will activate motors if true, and disable them if false.
 */
void AP_Mount_Alexmos::set_motor(bool on)
{
    if (on) {
        uint8_t data[1] = {(uint8_t)1};
        send_command(CMD_MOTORS_ON, data, 1);
    } else {
        uint8_t data[1] = {(uint8_t)1};
        send_command(CMD_MOTORS_OFF, data, 1);
    }
}

/*
 * get board version and firmware version
 */
void AP_Mount_Alexmos::get_boardinfo()
{
    if (_board_version != 0) {
        return;
    }
    uint8_t data[1] = {(uint8_t)1};
    send_command(CMD_BOARD_INFO, data, 1);
}

/*
  control_axis : send new angles to the gimbal at a fixed speed of 30 deg/s2
*/
void AP_Mount_Alexmos::control_axis(const Vector3f& angle, bool target_in_degrees, float boost)
{
    // convert to degrees if necessary
    Vector3f target_deg = angle;
    if (!target_in_degrees) {
        target_deg *= RAD_TO_DEG;
    }
    alexmos_parameters outgoing_buffer;
    outgoing_buffer.angle_speed.mode = AP_MOUNT_ALEXMOS_MODE_ANGLE;
    outgoing_buffer.angle_speed.speed_roll = boost * DEGREE_PER_SEC_TO_VALUE(AP_MOUNT_ALEXMOS_SPEED);
    outgoing_buffer.angle_speed.angle_roll = DEGREE_TO_VALUE(target_deg.x);
    outgoing_buffer.angle_speed.speed_pitch = boost * DEGREE_PER_SEC_TO_VALUE(AP_MOUNT_ALEXMOS_SPEED);
    outgoing_buffer.angle_speed.angle_pitch = DEGREE_TO_VALUE(target_deg.y);
    outgoing_buffer.angle_speed.speed_yaw = boost * DEGREE_PER_SEC_TO_VALUE(AP_MOUNT_ALEXMOS_SPEED);
    outgoing_buffer.angle_speed.angle_yaw = DEGREE_TO_VALUE(target_deg.z);
    send_command(CMD_CONTROL, (uint8_t *)&outgoing_buffer.angle_speed, sizeof(alexmos_angles_speed));
}

/*
  read current profile profile_id and global parameters from the gimbal settings
*/
void AP_Mount_Alexmos::read_params(uint8_t profile_id)
{
    uint8_t data[1] = {(uint8_t) profile_id}; 
    send_command(CMD_READ_PARAMS, data, 1);
}

/*
  write new parameters to the gimbal settings
*/
void AP_Mount_Alexmos::write_params()
{
    if (!_param_read_once) {
        return;
    }
    send_command(CMD_WRITE_PARAMS, (uint8_t *)&_current_parameters.params, sizeof(alexmos_params));
}

/*
 send a command to the Alemox Serial API
*/
void AP_Mount_Alexmos::send_command(uint8_t cmd, uint8_t* data, uint8_t size)
{
    if (_port->txspace() < (size + 5U)) {
        return;
    }
    uint8_t checksum = 0;
    _port->write( '>' );
    _port->write( cmd );  // write command id
    _port->write( size );  // write body size
    _port->write( cmd+size ); // write header checkum

    for (uint8_t i = 0;  i != size ; i++) {
        checksum += data[i];
        _port->write( data[i] );
    }
    _port->write(checksum);
}

/*
 * Parse the body of the message received from the Alexmos gimbal
 */
void AP_Mount_Alexmos::parse_body()
{
    switch (_command_id ) {
        case CMD_BOARD_INFO:
            _board_version = _buffer.version._board_version/ 10;
            _current_firmware_version = _buffer.version._firmware_version / 1000.0f ;
            _firmware_beta_version = _buffer.version._firmware_version % 10 ;
            _gimbal_3axis = (_buffer.version._board_features & 0x1);
            _gimbal_bat_monitoring = (_buffer.version._board_features & 0x2);
            break;

        case CMD_GET_ANGLES:
            _current_angle.x = VALUE_TO_DEGREE(_buffer.angles.angle_roll);
            _current_angle.y = VALUE_TO_DEGREE(_buffer.angles.angle_pitch);
            _current_angle.z = VALUE_TO_DEGREE(_buffer.angles.angle_yaw);
            break;

        case CMD_GET_ANGLES_EXT:
            // Alexmoss Pitch/Tilt is positive when pointing downward
            _current_angle.x = VALUE_TO_DEGREE(_buffer.angles_ext.angle_roll);
            _current_angle.y = VALUE_TO_DEGREE(_buffer.angles_ext.angle_pitch);
            _current_angle.z = VALUE_TO_DEGREE(_buffer.angles_ext.angle_yaw);
            update_target_2x720(_angle_ef_target_2x720, _current_angle, true, true);

            _current_stat_rot_angle.x = VALUE_TO_DEGREE(_buffer.angles_ext.stator_rotor_roll);
            _current_stat_rot_angle.y = VALUE_TO_DEGREE(_buffer.angles_ext.stator_rotor_pitch); 
            _current_stat_rot_angle.z = VALUE_TO_DEGREE(_buffer.angles_ext.stator_rotor_yaw);
            // stat_rot_angle does not overflow, so we need to make it consistent with AM IMU yaw angle range (-720, 720)
            _current_stat_rot_angle.z = wrap_2x720(_current_stat_rot_angle.z);

            break;

        case CMD_READ_PARAMS:
            _param_read_once = true;
            _current_parameters.params = _buffer.params;
            break;

        case CMD_WRITE_PARAMS:
            break;

        case CMD_AHRS_HELPER:
            _current_zenith.x = _buffer.ahrs_helper.zenith[0];
            _current_zenith.y = _buffer.ahrs_helper.zenith[1];
            _current_zenith.z = _buffer.ahrs_helper.zenith[2];

            _current_north.x = _buffer.ahrs_helper.north[0];
            _current_north.y = _buffer.ahrs_helper.north[1];
            _current_north.z = _buffer.ahrs_helper.north[2];
            break;

        case CMD_REALTIME_DATA_CUSTOM:
            _rt_data_timestamp = _buffer.rt_data_custom.time_ms;

            _current_zenith.x = _buffer.rt_data_custom.zenith[0];
            _current_zenith.y = _buffer.rt_data_custom.zenith[1];
            _current_zenith.z = _buffer.rt_data_custom.zenith[2];

            _current_north.x = _buffer.rt_data_custom.north[0];
            _current_north.y = _buffer.rt_data_custom.north[1];
            _current_north.z = _buffer.rt_data_custom.north[2];
            break;

        case CMD_CONFIRM:
            _last_command_confirmed = true;
            break;

        case CMD_ERROR:
            break;

        default :
            _last_command_confirmed = true;
            break;
    }
}

/*
 * detect and read the header of the incoming message from the gimbal
 */
void AP_Mount_Alexmos::read_incoming()
{
    uint8_t data;
    int16_t numc;

    numc = _port->available();

    if (numc < 0 ){
        return;
    }

    for (int16_t i = 0; i < numc; i++) {        // Process bytes received
        data = _port->read();
        switch (_step) {
            case 0:
                if ( '>' == data) {
                    _step = 1;
                    _checksum = 0; //reset checksum accumulator
                    _last_command_confirmed = false;
                }
                break;

            case 1: // command ID
                _checksum = data;
                _command_id = data;
                _step++;
                break;

            case 2: // Size of the body of the message
                _checksum += data;
                _payload_length = data;
                _step++;
                break;

            case 3:	// checksum of the header
                if (_checksum != data ) {
                    _step = 0;
                    _checksum = 0;
                    // checksum error
                    break;
                }
                _step++;
                _checksum = 0;
                _payload_counter = 0;                               // prepare to receive payload
                break;

            case 4: // parsing body
                _checksum += data;
                if (_payload_counter < sizeof(_buffer)) {
                    _buffer[_payload_counter] = data;
                }
                if (++_payload_counter == _payload_length)
                    _step++;
                break;

            case 5:// body checksum
                _step = 0;
                if (_checksum  != data) {
                    break;
                }
                parse_body();
        }
    }
}
