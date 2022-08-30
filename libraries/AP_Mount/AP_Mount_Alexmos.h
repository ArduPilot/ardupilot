/*
  Alexmos Serial controlled mount backend class
*/
#pragma once

#include "AP_Mount_Backend.h"

#ifndef HAL_MOUNT_ALEXMOS_ENABLED
#define HAL_MOUNT_ALEXMOS_ENABLED HAL_MOUNT_ENABLED
#endif

#if HAL_MOUNT_ALEXMOS_ENABLED
#include <AP_HAL/AP_HAL.h>
#include <AP_Param/AP_Param.h>
#include <AP_Math/AP_Math.h>

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

#define VALUE_TO_DEGREE(d) ((float)((d * 720) >> 15))
#define DEGREE_TO_VALUE(d) ((int16_t)((float)(d)*(1.0f/0.02197265625f)))
#define DEGREE_PER_SEC_TO_VALUE(d) ((int16_t)((float)(d)*(1.0f/0.1220740379f)))

class AP_Mount_Alexmos : public AP_Mount_Backend
{
public:
    //constructor
    AP_Mount_Alexmos(AP_Mount &frontend, AP_Mount_Params &params, uint8_t instance):
        AP_Mount_Backend(frontend, params, instance)
    {}

    // init - performs any required initialisation for this instance
    void init() override;

    // update mount position - should be called periodically
    void update() override;

    // has_pan_control - returns true if this mount can control its pan (required for multicopters)
    bool has_pan_control() const override;

protected:

    // get attitude as a quaternion.  returns true on success
    bool get_attitude_quaternion(Quaternion& att_quat) override;

private:

    // get_angles -
    void get_angles();

    // set_motor will activate motors if true, and disable them if false
    void set_motor(bool on);

    // get_boardinfo - get board version and firmware version
    void get_boardinfo();

    // send new angles to the gimbal at a fixed speed of 30 deg/s
    void control_axis(const MountTarget& angle_target_rad);

    // read_params - read current profile profile_id and global parameters from the gimbal settings
    void read_params(uint8_t profile_id);

    // write_params - write new parameters to the gimbal settings
    void write_params();

    bool get_realtimedata(Vector3f& angle);

    // Alexmos Serial Protocol reading part implementation
    // send_command - send a command to the Alemox Serial API
    void send_command(uint8_t cmd, uint8_t* data, uint8_t size);

    // Parse the body of the message received from the Alexmos gimbal
    void parse_body();

    // read_incoming - detect and read the header of the incoming message from the gimbal
    void read_incoming();

    MountTarget _angle_rad;         // latest angle target

    // structure for the Serial Protocol

    // CMD_BOARD_INFO
    struct PACKED alexmos_version {
        uint8_t _board_version;
        uint16_t _firmware_version;
        uint8_t debug_mode;
        uint16_t _board_features;
    };

    // CMD_GET_ANGLES
    struct PACKED alexmos_angles {
        int16_t angle_roll;
        int16_t rc_angle_roll;
        int16_t rc_speed_roll;
        int16_t angle_pitch;
        int16_t rc_angle_pitch;
        int16_t rc_speed_pitch;
        int16_t angle_yaw;
        int16_t rc_angle_yaw;
        int16_t rc_speed_yaw;
    };

    // CMD_CONTROL
    struct PACKED alexmos_angles_speed {
        int8_t mode;
        int16_t speed_roll;
        int16_t angle_roll;
        int16_t speed_pitch;
        int16_t angle_pitch;
        int16_t speed_yaw;
        int16_t angle_yaw;
    };

    // CMD_READ_PARAMS
    struct PACKED alexmos_params {
        uint8_t profile_id;
        uint8_t roll_P;
        uint8_t roll_I;
        uint8_t roll_D;
        uint8_t roll_power;
        uint8_t roll_invert;
        uint8_t roll_poles;
        uint8_t pitch_P;
        uint8_t pitch_I;
        uint8_t pitch_D;
        uint8_t pitch_power;
        uint8_t pitch_invert;
        uint8_t pitch_poles;
        uint8_t yaw_P;
        uint8_t yaw_I;
        uint8_t yaw_D;
        uint8_t yaw_power;
        uint8_t yaw_invert;
        uint8_t yaw_poles;
        uint8_t acc_limiter;
        int8_t ext_fc_gain_roll;
        int8_t ext_fc_gain_pitch;
        int16_t roll_rc_min_angle;
        int16_t roll_rc_max_angle;
        uint8_t roll_rc_mode;
        uint8_t roll_rc_lpf;
        uint8_t roll_rc_speed;
        uint8_t roll_rc_follow;
        int16_t pitch_rc_min_angle;
        int16_t pitch_rc_max_angle;
        uint8_t pitch_rc_mode;
        uint8_t pitch_rc_lpf;
        uint8_t pitch_rc_speed;
        uint8_t pitch_rc_follow;
        int16_t yaw_rc_min_angle;
        int16_t yaw_rc_max_angle;
        uint8_t yaw_rc_mode;
        uint8_t yaw_rc_lpf;
        uint8_t yaw_rc_speed;
        uint8_t yaw_rc_follow;
        uint8_t gyro_trust;
        uint8_t use_model;
        uint8_t pwm_freq;
        uint8_t serial_speed;
        int8_t rc_trim_roll;
        int8_t rc_trim_pitch;
        int8_t rc_trim_yaw;
        uint8_t rc_deadband;
        uint8_t rc_expo_rate;
        uint8_t rc_virt_mode;
        uint8_t rc_map_roll;
        uint8_t rc_map_pitch;	
        uint8_t rc_map_yaw;
        uint8_t rc_map_cmd;
        uint8_t rc_map_fc_roll;
        uint8_t rc_map_fc_pitch;

        uint8_t rc_mix_fc_roll;
        uint8_t rc_mix_fc_pitch;

        uint8_t follow_mode;
        uint8_t follow_deadband;
        uint8_t follow_expo_rate;
        int8_t follow_offset_roll;
        int8_t follow_offset_pitch;
        int8_t follow_offset_yaw;

        int8_t axis_top;
        int8_t axis_right;

        uint8_t gyro_lpf;

        uint8_t gyro_sens;
        uint8_t i2c_internal_pullups;
        uint8_t sky_gyro_calib;

        uint8_t rc_cmd_low;
        uint8_t rc_cmd_mid;
        uint8_t rc_cmd_high;

        uint8_t menu_cmd_1;
        uint8_t menu_cmd_2;
        uint8_t menu_cmd_3;
        uint8_t menu_cmd_4;
        uint8_t menu_cmd_5;
        uint8_t menu_cmd_long;

        uint8_t output_roll;
        uint8_t output_pitch;
        uint8_t output_yaw;

        int16_t bat_threshold_alarm;
        int16_t bat_threshold_motors;
        int16_t bat_comp_ref;

        uint8_t beeper_modes;

        uint8_t follow_roll_mix_start;
        uint8_t follow_roll_mix_range;

        uint8_t booster_power_roll;
        uint8_t booster_power_pitch;
        uint8_t booster_power_yaw;

        uint8_t follow_speed_roll;
        uint8_t follow_speed_pitch;
        uint8_t follow_speed_yaw;

        uint8_t frame_angle_from_motors;

        uint8_t cur_profile_id;

    };
    union PACKED alexmos_parameters {
        DEFINE_BYTE_ARRAY_METHODS
        alexmos_version version;
        alexmos_angles angles;
        alexmos_params params;
        alexmos_angles_speed angle_speed;
    } _buffer,_current_parameters;

    AP_HAL::UARTDriver *_port;
    bool _initialised : 1;

    // result of the get_boardinfo
    uint8_t _board_version;
    float _current_firmware_version;
    uint8_t _firmware_beta_version;
    bool _gimbal_3axis : 1;
    bool _gimbal_bat_monitoring : 1;

    // keep the last _current_angle values
    Vector3f _current_angle;

    // CMD_READ_PARAMS has been called once
    bool _param_read_once : 1;

    // Serial Protocol Variables
    uint8_t _checksum;
    uint8_t _step;
    uint8_t _command_id;
    uint8_t _payload_length;
    uint8_t _payload_counter;

    // confirmed that last command was ok
    bool _last_command_confirmed : 1;
};
#endif // HAL_MOUNT_ALEXMOS_ENABLED
