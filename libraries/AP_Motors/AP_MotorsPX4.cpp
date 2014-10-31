// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-
/*
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

/*
 *       AP_MotorsPX4.cpp - ArduCopter dummy mixer that delegates the actual mixing to the PX4 mixer infrastructure
 *       Code by Holger Steinhaus
 */

#include <AP_HAL.h>

#include "AP_MotorsPX4.h"

#include <drivers/drv_hrt.h>
#include <drivers/drv_pwm_output.h>

#include <fcntl.h>
#include <stdio.h>
#include <unistd.h>

#define FRAME_CONFIG_COUNT 20
#define FRAME_NAME_MAXLEN  20

#define MIXER_DIR_PREFIX    "/etc/mixers"
#define MIXER_DEV_PWM       "/dev/pwm_output"
#define MIXER_DEV_UAVCAN    "/dev/uavcan/esc"

#define PWM_CHANNELS        8
#define PWM_IDLE            1000

extern const AP_HAL::HAL& hal;

const AP_Param::GroupInfo AP_MotorsPX4::var_info[] PROGMEM = {
    // @Param: LAYOUT
    // @DisplayName: Frame layout
    // @Description: Frame layout. A suitable mixer configuration file is chosen by this parameter, together with orientation configured by FRAME.
    // @User: Advanced
    // @Values: 0:Quad,1:Hexa,2:Octo,3:Hexa-coax,4:Octo-coax,16:custom0,17:custom1,18:custom2,19:custom3
    AP_GROUPINFO("LAYOUT", 0, AP_MotorsPX4, _frame_layout, AP_MOTORS_FRAME_LAYOUT),

    // @Param: OUTDEV
    // @DisplayName: Output device
    // @Description: Output device to use. On Pixhawk, PWM and UAVCAN are currently supported.
    // @User: Advanced
    // @Values: 0:PWM,1:UAVCAN
    // @Increment: 1
    AP_GROUPINFO("OUTDEV", 1, AP_MotorsPX4, _output_device, AP_MOTORS_OUTPUT_DEVICE),

    // @Param: SPIN_ARMED
    // @DisplayName: Motors always spin when armed
    // @Description: Controls whether motors always spin when armed (must be below THR_MIN)
    // @Values: 0:Do Not Spin,70:VerySlow,100:Slow,130:Medium,150:Fast
    // @User: Standard
    AP_GROUPINFO("SPIN_ARMED", 5, AP_MotorsPX4, _spin_when_armed, AP_MOTORS_SPIN_WHEN_ARMED),

    AP_GROUPEND
};


// Init
void AP_MotorsPX4::Init()
{
    // call parent Init function to set-up throttle curve
    AP_Motors::Init();

    // enable fast channels or instant pwm
    set_update_rate(_speed_hz);

    // load mixer config
    // @TODO: handle errors
    load_mixer();
    setup_output();

    _limits_sub = orb_subscribe(ORB_ID(multirotor_motor_limits));
    if (_limits_sub == -1) {
       // @TODO: handle errors
       printf("Unable to subscribe to multirotor_motor_limits");
    }
}


// load mixer configuration
bool AP_MotorsPX4::load_mixer()
{
    char mixer_names[FRAME_CONFIG_COUNT][FRAME_NAME_MAXLEN] = {
        "quad",         // LAYOUT = 0
        "hex",          // LAYOUT = 1
        "octo",         // LAYOUT = 2
        "hexa_cox",     // LAYOUT = 3
        "octo_cox",     // LAYOUT = 4
        "", "", "", "", "", "", "", "", "", "", "",
        "custom0",      // LAYOUT = 16
        "custom1",      // LAYOUT = 17
        "custom2",      // LAYOUT = 18
        "custom3",      // LAYOUT = 19
    };

    // mixer config file name depending on configuration
    if (_frame_layout < 0 || _frame_layout >= FRAME_CONFIG_COUNT) {
        return false;
    }

    char orient;
    switch (_flags.frame_orientation) {
        case AP_MOTORS_PLUS_FRAME:
            orient = '+';
            break;
        case AP_MOTORS_X_FRAME:
            orient = 'x';
            break;
        case AP_MOTORS_V_FRAME:
            orient = 'v';
            break;
        default:
            return false;
    }

    char mixer_name[strlen(MIXER_DIR_PREFIX) + FRAME_NAME_MAXLEN];
    sprintf(mixer_name, "%s/FMU_%s_%c.mix", MIXER_DIR_PREFIX, mixer_names[_frame_layout], orient);

    // mixer device name
    const char* mixer_device;
    switch (_output_device) {
        case AP_MOTORS_PX4_PWM_OUTPUT:
            mixer_device = MIXER_DEV_PWM;
            break;
        case AP_MOTORS_PX4_UAVCAN_OUTPUT:
            mixer_device = MIXER_DEV_UAVCAN;
            break;
        default:
            return false;
    }

    // try to load mixer config file
    int devfd;
    int ret;
    char buf[890];     // TODO: buffer handling and size correct? Size chosen to prevent frame size warning.

    if ((devfd = open(mixer_device, 0)) < 0) {
        goto failed;
    }

    if (ioctl(devfd, MIXERIOCRESET, 0)) {
        goto failed;
    }

    if (load_mixer_file(mixer_name, &buf[0], sizeof(buf)) < 0) {
        goto failed;
    }

    ret = ioctl(devfd, MIXERIOCLOADBUF, (unsigned long)buf);
    if (ret < 0) {
        goto failed;
    }

    printf("Successfully loaded mixer %s into %s\n", mixer_name, mixer_device);
    return true;

failed:
    printf("Failed to load mixer %s into %s\n", mixer_name, mixer_device);
    close(devfd);
    return false;
}


// enable - starts allowing signals to be sent to motors
void AP_MotorsPX4::enable()
{
    // nothing to do here yet
}


// get_motor_mask - returns a bitmask of which outputs are being used for motors
uint16_t AP_MotorsPX4::get_motor_mask()
{
    return 0;
}


void AP_MotorsPX4::publish_armed() {
    if (_flags.armed != _old_armed)  {
        _old_armed = _flags.armed;

        // publish armed state
        _armed.timestamp = hrt_absolute_time();
        _armed.armed = _flags.armed;
        _armed.ready_to_arm = true;
        _armed.lockdown = false;

        if (_armed_pub > 0) {
            /* publish armed state */
            orb_publish(ORB_ID(actuator_armed), _armed_pub, &_armed);
        } else {
            /* advertise and publish */
            _armed_pub = orb_advertise(ORB_ID(actuator_armed), &_armed);
        }
    }
}


void AP_MotorsPX4::publish_controls()
{
    _actuators.timestamp = hrt_absolute_time();

    // publish raw input values for use by uavcan mixer
    if (_actuators_0_pub > 0) {
        /* publish the attitude setpoint */
        orb_publish(ORB_ID(actuator_controls_0), _actuators_0_pub, &_actuators);
    } else {
        /* advertise and publish */
        _actuators_0_pub = orb_advertise(ORB_ID(actuator_controls_0), &_actuators);
    }
}


bool AP_MotorsPX4::update_limits()
{
    bool limits_updated = orb_check(_limits_sub, &limits_updated) == 0 && limits_updated;

    if (limits_updated) {
        if (orb_copy(ORB_ID(multirotor_motor_limits), _limits_sub, &limit) != OK) {
            printf("orb_copy failed\n");
        }
    }
    else {  
//        memset(&limit, 0, sizeof(limit));
    }

    return limits_updated;
}


// output_min - sends minimum values out to the motors
void AP_MotorsPX4::output_min()
{
    // set limits flags
    limit.roll_pitch = true;
    limit.yaw = true;
    limit.throttle_lower = true;
    limit.throttle_upper = false;

    _actuators.control[0] = 0.;
    _actuators.control[1] = 0.;
    _actuators.control[2] = 0.;
    _actuators.control[3] = 0.;
    _actuators.timestamp = hrt_absolute_time();

    publish_armed();
    publish_controls(); // not really useful, but keeps PX4IO happy (prevents FMU_FAIL flag / flashing amber LED)
}

// output_armed - sends commands to the motors
// includes new scaling stability patch
void AP_MotorsPX4::output_armed()
{
    // initialize limits flag
    update_limits();

    // Throttle is 0 to 1000 only
    // @TODO: we should not really be limiting this here because we don't "own" this _rc_throttle object
    // copied from AP_MotorsMatrix - no idea if this is important here
    if (_rc_throttle.servo_out < 0) {
        _rc_throttle.servo_out = 0;
        limit.throttle_lower = true;
    }
    if (_rc_throttle.servo_out > _max_throttle) {
        _rc_throttle.servo_out = _max_throttle;
        limit.throttle_upper = true;
    }

    if (_rc_throttle.servo_out == 0) {
        // MIX_SPIN_ARMED, center all controls, spin motors slowly
        _actuators.control[0] = 0.;
        _actuators.control[1] = 0.;
        _actuators.control[2] = 0.;

        // no ramp - some ESCs have trouble to startup motors if ramped slowly to low values, espcially on higher input voltage)
        _actuators.control[3] = _spin_when_armed / 1000.;

        // Every thing is limited
        limit.roll_pitch = true;
        limit.yaw = true;
        limit.throttle_lower = true;

    }
    else {
        // normal ops
        _actuators.control[0] = _rc_roll.servo_out / 4500.;     // roll,        [-1..1]
        _actuators.control[1] = _rc_pitch.servo_out / 4500.;    // pitch,       [-1..1]
        _actuators.control[2] = _rc_yaw.servo_out / 4500.;      // yaw,         [-1..1]
        _actuators.control[3] = _rc_throttle.servo_out / 1000.; // throttle,    [0..1]
    }
    publish_armed();
    publish_controls();
}

// output_disarmed - sends commands to the motors
void AP_MotorsPX4::output_disarmed()
{
    // Send minimum values to all motors
    output_min();
}

// output_test - spin a motor at the pwm value specified
//  motor_seq is the motor's sequence number from 1 to the number of motors on the frame
//  pwm value is an actual pwm value that will be output, normally in the range of 1000 ~ 2000
void AP_MotorsPX4::output_test(uint8_t motor_seq, int16_t pwm)
{
    float power = (pwm - 1000.f)/1000.f;

    _test_motor.motor_number = motor_seq-1;
    _test_motor.timestamp = hrt_absolute_time();
    _test_motor.value = power;

    if (_test_motor_pub > 0) {
   	    /* publish armed state */
        orb_publish(ORB_ID(test_motor), _test_motor_pub, &_test_motor);
    } else {
        /* advertise and publish */
        _test_motor_pub = orb_advertise(ORB_ID(test_motor), &_test_motor);
    }
}


// throttle_pass_through - passes pilot's throttle input directly to all motors - dangerous but used for initialising ESCs
void AP_MotorsPX4::throttle_pass_through(int16_t pwm)
{
    if (armed()) {
        // send the pilot's input directly to each enabled motor
        for (int16_t i=0; i < AP_MOTORS_MAX_NUM_MOTORS; i++) {
            _actuators.control[0] = 0.;
            _actuators.control[1] = 0.;
            _actuators.control[2] = 0.;
            _actuators.control[3] = (pwm - _rc_throttle.radio_min) / float(_rc_throttle.radio_max - _rc_throttle.radio_min);
            _actuators.timestamp = hrt_absolute_time();
            publish_armed();
            publish_controls(); // not really useful, but keeps PX4IO happy (prevents FMU_FAIL flag / flashing amber LED)            }
        }
    }
}


// set update rate to motors - a value in hertz
void AP_MotorsPX4::set_update_rate( uint16_t speed_hz )
{
    AP_Motors::set_update_rate(speed_hz);
    setup_output();
}


bool AP_MotorsPX4::setup_output() 
{
   if (_output_device == AP_MOTORS_PX4_PWM_OUTPUT) {
        // open PWM device
        int fd = open(MIXER_DEV_PWM, 0);
        if (fd < 0) 
            return false;

        // set update rate
        int mask = (1<<PWM_CHANNELS)-1;
        int ret = ioctl(fd, PWM_SERVO_SET_UPDATE_RATE, _speed_hz);
        if (ret != OK) 
            return false;

        ret = ioctl(fd, PWM_SERVO_SET_SELECT_UPDATE_RATE, mask);
        if (ret != OK)
            return false;

        // set failsafe value to 1000, as some ESCs get confused by values lower than the calibrated range
        struct pwm_output_values fs_values = {.values = {1000}, .channel_count = PWM_CHANNELS};
        ret = ioctl(fd, PWM_SERVO_SET_FAILSAFE_PWM, (long unsigned int)&fs_values);
        if (ret != OK)
            return false;

        // no min-max setup here, stay with PX4 default of 1000..2000us 
    }
    else if (_output_device == AP_MOTORS_PX4_UAVCAN_OUTPUT) {
        // currently nothing to set here (update rate is fixed, calibration not needed)
    }

    return true;
}