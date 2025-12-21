#include "AP_Mount_config.h"

#if HAL_MOUNT_ALEXMOS_ENABLED

#include "AP_Mount_Alexmos.h"

#include <AP_SerialManager/AP_SerialManager.h>

//definition of the commands id for the Alexmos Serial Protocol
#define CMD_READ_PARAMS 'R'
#define CMD_WRITE_PARAMS 'W'
#define CMD_REALTIME_DATA 'D'
#define CMD_BOARD_INFO 'V'
#define CMD_CALIB_ACC 'A'
#define CMD_CALIB_GYRO 'g'
#define CMD_CALIB_EXT_GAIN 'G'
#define CMD_USE_DEFAULTS 'F'
#define CMD_CALIB_POLES 'P'
#define CMD_RESET 'r'
#define CMD_HELPER_DATA 'H'
#define CMD_CALIB_OFFSET 'O'
#define CMD_CALIB_BAT 'B'
#define CMD_MOTORS_ON 'M'
#define CMD_MOTORS_OFF 'm'
#define CMD_CONTROL 'C'
#define CMD_TRIGGER_PIN 'T'
#define CMD_EXECUTE_MENU 'E'
#define CMD_GET_ANGLES 'I'
#define CMD_CONFIRM 'C'
// Board v3.x only
#define CMD_BOARD_INFO_3 20
#define CMD_READ_PARAMS_3 21
#define CMD_WRITE_PARAMS_3 22
#define CMD_REALTIME_DATA_3 23
#define CMD_SELECT_IMU_3 24
#define CMD_READ_PROFILE_NAMES 28
#define CMD_WRITE_PROFILE_NAMES 29
#define CMD_QUEUE_PARAMS_INFO_3 30
#define CMD_SET_PARAMS_3 31
#define CMD_SAVE_PARAMS_3 32
#define CMD_READ_PARAMS_EXT 33
#define CMD_WRITE_PARAMS_EXT 34
#define CMD_AUTO_PID 35
#define CMD_SERVO_OUT 36
#define CMD_ERROR 255

#define AP_MOUNT_ALEXMOS_MODE_NO_CONTROL 0
#define AP_MOUNT_ALEXMOS_MODE_SPEED 1
#define AP_MOUNT_ALEXMOS_MODE_ANGLE 2
#define AP_MOUNT_ALEXMOS_MODE_SPEED_ANGLE 3
#define AP_MOUNT_ALEXMOS_MODE_RC 4

#define AP_MOUNT_ALEXMOS_SPEED 30 // deg/s

#define INT14_DEGREES (360.0f / float(0x3FFF)) // 1 full rotation in degrees over 14 bit range
#define VALUE_TO_DEGREE(d) (float(d)*INT14_DEGREES)
#define DEGREE_TO_VALUE(d) ((int16_t)((float)(d)*(1.0f/INT14_DEGREES)))
#define DEGREE_PER_SEC_TO_VALUE(d) ((int16_t)((float)(d)*(1.0f/0.1220740379f)))

void AP_Mount_Alexmos::init()
{
    const AP_SerialManager& serial_manager = AP::serialmanager();

    // check for alexmos protcol
    if ((_port = serial_manager.find_serial(AP_SerialManager::SerialProtocol_AlexMos, 0))) {
        _initialised = true;
        get_boardinfo();
        read_params(0); //we request parameters for profile 0 and therfore get global and profile parameters
    }
    AP_Mount_Backend::init();
}

// update mount position - should be called periodically
void AP_Mount_Alexmos::update()
{
    AP_Mount_Backend::update();

    if (!_initialised) {
        return;
    }

    read_incoming(); // read the incoming messages from the gimbal

    // update based on mount mode
    update_mnt_target();

    // send target angles or rates depending on the target type
    send_target_to_gimbal();
}

// has_pan_control - returns true if this mount can control its pan (required for multicopters)
bool AP_Mount_Alexmos::has_pan_control() const
{
    return _gimbal_3axis && yaw_range_valid();
}

// get attitude as a quaternion.  returns true on success
bool AP_Mount_Alexmos::get_attitude_quaternion(Quaternion& att_quat)
{
    if (!_initialised) {
        return false;
    }

    // request attitude from gimbal
    get_angles();

    // construct quaternion
    att_quat.from_euler(radians(_current_angle.x), radians(_current_angle.y), radians(_current_angle.z));
    return true;
}

/*
 * get_angles
 */
void AP_Mount_Alexmos::get_angles()
{
    uint8_t data[1] = {(uint8_t)1};
    send_command(CMD_GET_ANGLES, data, 1);
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
  control_axis : send new angle target to the gimbal at a fixed speed of 30 deg/s
*/
void AP_Mount_Alexmos::send_target_angles(const MountAngleTarget& angle_target_rad)
{
    alexmos_parameters outgoing_buffer;
    outgoing_buffer.angle_speed.mode = AP_MOUNT_ALEXMOS_MODE_ANGLE;
    outgoing_buffer.angle_speed.speed_roll = DEGREE_PER_SEC_TO_VALUE(AP_MOUNT_ALEXMOS_SPEED);
    outgoing_buffer.angle_speed.angle_roll = DEGREE_TO_VALUE(degrees(angle_target_rad.roll));
    outgoing_buffer.angle_speed.speed_pitch = DEGREE_PER_SEC_TO_VALUE(AP_MOUNT_ALEXMOS_SPEED);
    outgoing_buffer.angle_speed.angle_pitch = DEGREE_TO_VALUE(degrees(angle_target_rad.pitch));
    outgoing_buffer.angle_speed.speed_yaw = DEGREE_PER_SEC_TO_VALUE(AP_MOUNT_ALEXMOS_SPEED);
    outgoing_buffer.angle_speed.angle_yaw = DEGREE_TO_VALUE(angle_target_rad.get_bf_yaw());
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
            _current_firmware_version = _buffer.version._firmware_version * 0.001f ;
            _firmware_beta_version = _buffer.version._firmware_version % 10 ;
            _gimbal_3axis = (_buffer.version._board_features & 0x1);
            _gimbal_bat_monitoring = (_buffer.version._board_features & 0x2);
            break;

        case CMD_GET_ANGLES:
            _current_angle.x = VALUE_TO_DEGREE(_buffer.angles.angle_roll);
            _current_angle.y = VALUE_TO_DEGREE(_buffer.angles.angle_pitch);
            _current_angle.z = VALUE_TO_DEGREE(_buffer.angles.angle_yaw);
            break;

        case CMD_READ_PARAMS:
            _param_read_once = true;
            _current_parameters.params = _buffer.params;
            break;

        case CMD_WRITE_PARAMS:
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

    if (numc < 0 ) {
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
#endif // HAL_MOUNT_ALEXMOS_ENABLED
