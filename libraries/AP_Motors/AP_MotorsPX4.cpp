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

#include <fcntl.h>
#include <stdio.h>
#include <unistd.h>


#define FRAME_CONFIG_COUNT 5
#define FRAME_NAME_MAXLEN  20

#define MIXER_DIR_PREFIX "/etc/mixers"

extern const AP_HAL::HAL& hal;

const AP_Param::GroupInfo AP_MotorsPX4::var_info[] PROGMEM = {
    // @Param: LAYOUT
    // @DisplayName: Frame layout
    // @Description: Frame layout. A suitable mixer configuration file is chosen by this parameter, together with orientation configured by FRAME.
    // @User: Advanced
    // @Values: 0:Quad,1:Hexa,2:Octo,3:Hexa-coax,4:Octo-coax
    AP_GROUPINFO("LAYOUT", 0, AP_MotorsPX4, _frame_layout, AP_MOTORS_FRAME_LAYOUT),

    // @Param: OUTDEV
    // @DisplayName: Output device
    // @Description: Output device to use. On Pixhawk, PWM and UAVCAN are currently supported.
    // @User: Advanced
    // @Values: 0:PWM,1:UAVCAN
    // @Increment: 1
    AP_GROUPINFO("OUTDEV", 1, AP_MotorsPX4, _output_device, AP_MOTORS_OUTPUT_DEVICE),

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
}


// load mixer configuration
bool AP_MotorsPX4::load_mixer()
{
    char mixer_names[FRAME_CONFIG_COUNT][FRAME_NAME_MAXLEN] = {
        "quad",
        "hex",
        "octo",
        "hexa_cox",
        "octo_cox",
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
            mixer_device = "/dev/pwm_output";
            break;
        case AP_MOTORS_PX4_UAVCAN_OUTPUT:
            mixer_device = "/dev/uavcan/esc";
            break;
        default:
            return false;
    }

    // try to load mixer config file
    int devfd;
    int ret;
    char buf[890];     // TODO: buffer handling and size correct? Size chosen to prevent frame size warning.

    if ((devfd = open(mixer_device, 0)) < 0) {
        printf("open\n");
        goto failed;
    }

    if (ioctl(devfd, MIXERIOCRESET, 0)) {
        printf("ioctl\n");
        goto failed;
    }

    if (load_mixer_file(mixer_name, &buf[0], sizeof(buf)) < 0) {
        printf("load\n");
        goto failed;
    }

    ret = ioctl(devfd, MIXERIOCLOADBUF, (unsigned long)buf);
    if (ret < 0) {
        printf("ioctl2\n");
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
    // publish raw input values for use by uavcan mixer
    if (_actuators_0_pub > 0) {
        /* publish the attitude setpoint */
        orb_publish(ORB_ID(actuator_controls_0), _actuators_0_pub, &_actuators);
    } else {
        /* advertise and publish */
        _actuators_0_pub = orb_advertise(ORB_ID(actuator_controls_0), &_actuators);
    }
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
    limit.roll_pitch = false;
    limit.yaw = false;
    limit.throttle_lower = false;
    limit.throttle_upper = false;

    // Throttle is 0 to 1000 only
    // To-Do: we should not really be limiting this here because we don't "own" this _rc_throttle object
    if (_rc_throttle.servo_out < 0) {
        _rc_throttle.servo_out = 0;
        limit.throttle_lower = true;
    }
    if (_rc_throttle.servo_out > _max_throttle) {
        _rc_throttle.servo_out = _max_throttle;
        limit.throttle_upper = true;
    }

    // prepare actuators struct
    _actuators.timestamp = hrt_absolute_time();

    if (_rc_throttle.servo_out == 0) {
        // MOT_SPIN_ARMED
        if (_spin_when_armed_ramped < 0) {
             _spin_when_armed_ramped = 0;
        }
        if (_spin_when_armed_ramped > _min_throttle) {
            _spin_when_armed_ramped = _min_throttle;
        }

        _actuators.control[0] = 0.;
        _actuators.control[1] = 0.;
        _actuators.control[2] = 0.;
        _actuators.control[3] = _spin_when_armed_ramped / 1000.;

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
  // unfortuantely currently not possible as we are not the one who can control a single motor
}
