/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

#include <AP_HAL/AP_HAL.h>

#if CONFIG_HAL_BOARD == HAL_BOARD_PX4
#include "RCOutput.h"

#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <unistd.h>

#include <drivers/drv_pwm_output.h>
#include <uORB/topics/actuator_direct.h>
#include <drivers/drv_hrt.h>

extern const AP_HAL::HAL& hal;

using namespace PX4;

void PX4RCOutput::init()
{
    _perf_rcout = perf_alloc(PC_ELAPSED, "APM_rcout");
    _pwm_fd = open(PWM_OUTPUT0_DEVICE_PATH, O_RDWR);
    if (_pwm_fd == -1) {
        AP_HAL::panic("Unable to open " PWM_OUTPUT0_DEVICE_PATH);
    }
    if (ioctl(_pwm_fd, PWM_SERVO_ARM, 0) != 0) {
        hal.console->printf("RCOutput: Unable to setup IO arming\n");
    }
    if (ioctl(_pwm_fd, PWM_SERVO_SET_ARM_OK, 0) != 0) {
        hal.console->printf("RCOutput: Unable to setup IO arming OK\n");
    }
    _rate_mask = 0;
    _alt_fd = -1;    
    _servo_count = 0;
    _alt_servo_count = 0;

    if (ioctl(_pwm_fd, PWM_SERVO_GET_COUNT, (unsigned long)&_servo_count) != 0) {
        hal.console->printf("RCOutput: Unable to get servo count\n");        
        return;
    }

    for (uint8_t i=0; i<ORB_MULTI_MAX_INSTANCES; i++) {
        _outputs[i].pwm_sub = orb_subscribe_multi(ORB_ID(actuator_outputs), i);
    }

#if !defined(CONFIG_ARCH_BOARD_PX4FMU_V4)
    _alt_fd = open("/dev/px4fmu", O_RDWR);
    if (_alt_fd == -1) {
        hal.console->printf("RCOutput: failed to open /dev/px4fmu");
        return;
    }
#endif

    // ensure not to write zeros to disabled channels
    _enabled_channels = 0;
    for (int i=0; i < PX4_NUM_OUTPUT_CHANNELS; i++) {
        _period[i] = PWM_IGNORE_THIS_CHANNEL;
    }

    // publish actuator vaules on demand
    _actuator_direct_pub = NULL;

    // and armed state
    _actuator_armed_pub = NULL;
}


void PX4RCOutput::_init_alt_channels(void) 
{
    if (_alt_fd == -1) {
        return;
    }
    if (ioctl(_alt_fd, PWM_SERVO_ARM, 0) != 0) {
        hal.console->printf("RCOutput: Unable to setup alt IO arming\n");
        return;
    }
    if (ioctl(_alt_fd, PWM_SERVO_SET_ARM_OK, 0) != 0) {
        hal.console->printf("RCOutput: Unable to setup alt IO arming OK\n");
        return;
    }
    if (ioctl(_alt_fd, PWM_SERVO_GET_COUNT, (unsigned long)&_alt_servo_count) != 0) {
        hal.console->printf("RCOutput: Unable to get servo count\n");        
    }
}

/*
  set output frequency on outputs associated with fd
 */
void PX4RCOutput::set_freq_fd(int fd, uint32_t chmask, uint16_t freq_hz) 
{
    // we can't set this per channel
    if (freq_hz > 50) {
        // we're being asked to set the alt rate
        if (ioctl(fd, PWM_SERVO_SET_UPDATE_RATE, (unsigned long)freq_hz) != 0) {
            hal.console->printf("RCOutput: Unable to set alt rate to %uHz\n", (unsigned)freq_hz);
            return;
        }
        _freq_hz = freq_hz;
    }

    /* work out the new rate mask. The outputs have 3 groups of servos. 

       Group 0: channels 0 1
       Group 1: channels 4 5 6 7
       Group 2: channels 2 3

       Channels within a group must be set to the same rate.

       For the moment we never set the channels above 8 to more than
       50Hz
     */
    if (freq_hz > 50) {
        // we are setting high rates on the given channels
        _rate_mask |= chmask & 0xFF;
        if (_rate_mask & 0x3) {
            _rate_mask |= 0x3;
        }
        if (_rate_mask & 0xc) {
            _rate_mask |= 0xc;
        }
        if (_rate_mask & 0xF0) {
            _rate_mask |= 0xF0;
        }
    } else {
        // we are setting low rates on the given channels
        if (chmask & 0x3) {
            _rate_mask &= ~0x3;
        }
        if (chmask & 0xc) {
            _rate_mask &= ~0xc;
        }
        if (chmask & 0xf0) {
            _rate_mask &= ~0xf0;
        }
    }

    if (ioctl(fd, PWM_SERVO_SET_SELECT_UPDATE_RATE, _rate_mask) != 0) {
        hal.console->printf("RCOutput: Unable to set alt rate mask to 0x%x\n", (unsigned)_rate_mask);
    }
}

/*
  set output frequency
 */
void PX4RCOutput::set_freq(uint32_t chmask, uint16_t freq_hz) 
{
    // greater than 400 doesn't give enough room at higher periods for
    // the down pulse
    if (freq_hz > 400) {
        freq_hz = 400;
    }
    uint32_t primary_mask = chmask & ((1UL<<_servo_count)-1);
    uint32_t alt_mask = chmask >> _servo_count;
    if (primary_mask && _pwm_fd != -1) {
        set_freq_fd(_pwm_fd, primary_mask, freq_hz);
    }
    if (alt_mask && _alt_fd != -1) {
        set_freq_fd(_alt_fd, alt_mask, freq_hz);
    }
}

uint16_t PX4RCOutput::get_freq(uint8_t ch) 
{
    if (_rate_mask & (1U<<ch)) {
        return _freq_hz;
    }
    return 50;
}

void PX4RCOutput::enable_ch(uint8_t ch)
{
    if (ch >= PX4_NUM_OUTPUT_CHANNELS) {
        return;
    }
    if (ch >= 8 && !(_enabled_channels & (1U<<ch))) {
        // this is the first enable of an auxillary channel - setup
        // aux channels now. This delayed setup makes it possible to
        // use BRD_PWM_COUNT to setup the number of PWM channels.
        _init_alt_channels();
    }
    _enabled_channels |= (1U<<ch);
    if (_period[ch] == PWM_IGNORE_THIS_CHANNEL) {
        _period[ch] = 0;
    }
}

void PX4RCOutput::disable_ch(uint8_t ch)
{
    if (ch >= PX4_NUM_OUTPUT_CHANNELS) {
        return;
    }

    _enabled_channels &= ~(1U<<ch);
    _period[ch] = PWM_IGNORE_THIS_CHANNEL;
}

void PX4RCOutput::set_safety_pwm(uint32_t chmask, uint16_t period_us)
{
    struct pwm_output_values pwm_values;
    memset(&pwm_values, 0, sizeof(pwm_values));
    for (uint8_t i=0; i<_servo_count; i++) {
        if ((1UL<<i) & chmask) {
            pwm_values.values[i] = period_us;
        }
        pwm_values.channel_count++;
    }
    int ret = ioctl(_pwm_fd, PWM_SERVO_SET_DISARMED_PWM, (long unsigned int)&pwm_values);
    if (ret != OK) {
        hal.console->printf("Failed to setup disarmed PWM for 0x%08x to %u\n", (unsigned)chmask, period_us);
    }
}

void PX4RCOutput::set_failsafe_pwm(uint32_t chmask, uint16_t period_us)
{
    struct pwm_output_values pwm_values;
    memset(&pwm_values, 0, sizeof(pwm_values));
    for (uint8_t i=0; i<_servo_count; i++) {
        if ((1UL<<i) & chmask) {
            pwm_values.values[i] = period_us;
        }
        pwm_values.channel_count++;
    }
    int ret = ioctl(_pwm_fd, PWM_SERVO_SET_FAILSAFE_PWM, (long unsigned int)&pwm_values);
    if (ret != OK) {
        hal.console->printf("Failed to setup failsafe PWM for 0x%08x to %u\n", (unsigned)chmask, period_us);
    }
}

bool PX4RCOutput::force_safety_on(void)
{
    int ret = ioctl(_pwm_fd, PWM_SERVO_SET_FORCE_SAFETY_ON, 0);
    return (ret == OK);
}

void PX4RCOutput::force_safety_off(void)
{
    int ret = ioctl(_pwm_fd, PWM_SERVO_SET_FORCE_SAFETY_OFF, 0);
    if (ret != OK) {
        hal.console->printf("Failed to force safety off\n");
    }
}

void PX4RCOutput::write(uint8_t ch, uint16_t period_us)
{
    if (ch >= _servo_count + _alt_servo_count) {
        return;
    }
    if (!(_enabled_channels & (1U<<ch))) {
        // not enabled
        return;
    }
    if (ch >= _max_channel) {
        _max_channel = ch + 1;
    }
    if (period_us != _period[ch]) {
        _period[ch] = period_us;
        _need_update = true;
    }
}

uint16_t PX4RCOutput::read(uint8_t ch) 
{
    if (ch >= PX4_NUM_OUTPUT_CHANNELS) {
        return 0;
    }
    // if px4io has given us a value for this channel use that,
    // otherwise use the value we last sent. This makes it easier to
    // observe the behaviour of failsafe in px4io
    for (uint8_t i=0; i<ORB_MULTI_MAX_INSTANCES; i++) {
        if (_outputs[i].pwm_sub >= 0 && 
            ch < _outputs[i].outputs.noutputs &&
            _outputs[i].outputs.output[ch] > 0) {
            return _outputs[i].outputs.output[ch];
        }
    }
    return _period[ch];
}

void PX4RCOutput::read(uint16_t* period_us, uint8_t len)
{
    for (uint8_t i=0; i<len; i++) {
        period_us[i] = read(i);
    }
}

/*
  update actuator armed state
 */
void PX4RCOutput::_arm_actuators(bool arm)
{
    if (_armed.armed == arm) {
        // already armed;
        return;
    }

	_armed.timestamp = hrt_absolute_time();
    _armed.armed = arm;
    _armed.ready_to_arm = arm;
    _armed.lockdown = false;
    _armed.force_failsafe = false;

    if (_actuator_armed_pub == NULL) {
        _actuator_armed_pub = orb_advertise(ORB_ID(actuator_armed), &_armed);
    } else {
        orb_publish(ORB_ID(actuator_armed), _actuator_armed_pub, &_armed);
    }
}

/*
  publish new outputs to the actuator_direct topic
 */
void PX4RCOutput::_publish_actuators(void)
{
	struct actuator_direct_s actuators;

    if (_esc_pwm_min == 0 ||
        _esc_pwm_max == 0) {
        // not initialised yet
        return;
    }

	actuators.nvalues = _max_channel;
    if (actuators.nvalues > actuators.NUM_ACTUATORS_DIRECT) {
        actuators.nvalues = actuators.NUM_ACTUATORS_DIRECT;
    }
    // don't publish more than 8 actuators for now, as the uavcan ESC
    // driver refuses to update any motors if you try to publish more
    // than 8
    if (actuators.nvalues > 8) {
        actuators.nvalues = 8;
    }
    bool armed = hal.util->get_soft_armed();
	actuators.timestamp = hrt_absolute_time();
    for (uint8_t i=0; i<actuators.nvalues; i++) {
        if (!armed) {
            actuators.values[i] = 0;
        } else {
            actuators.values[i] = (_period[i] - _esc_pwm_min) / (float)(_esc_pwm_max - _esc_pwm_min);
        }
        // actuator values are from -1 to 1
        actuators.values[i] = actuators.values[i]*2 - 1;
    }

    if (_actuator_direct_pub == NULL) {
        _actuator_direct_pub = orb_advertise(ORB_ID(actuator_direct), &actuators);
    } else {
        orb_publish(ORB_ID(actuator_direct), _actuator_direct_pub, &actuators);
    }
    if (hal.util->safety_switch_state() != AP_HAL::Util::SAFETY_DISARMED) {
        _arm_actuators(true);
    }
}

void PX4RCOutput::_timer_tick(void)
{
    uint32_t now = AP_HAL::micros();

    if ((_enabled_channels & ((1U<<_servo_count)-1)) == 0) {
        // no channels enabled
        _arm_actuators(false);
        goto update_pwm;
    }

    // always send at least at 20Hz, otherwise the IO board may think
    // we are dead
    if (now - _last_output > 50000) {
        _need_update = true;
    }

    // check for PWM count changing. This can happen then the user changes BRD_PWM_COUNT
    if (now - _last_config_us > 1000000) {
        if (_pwm_fd != -1) {
            ioctl(_pwm_fd, PWM_SERVO_GET_COUNT, (unsigned long)&_servo_count);
        }
        if (_alt_fd != -1) {
            ioctl(_alt_fd, PWM_SERVO_GET_COUNT, (unsigned long)&_alt_servo_count);
        }
        _last_config_us = now;
    }
    
    if (_need_update && _pwm_fd != -1) {
        _need_update = false;
        perf_begin(_perf_rcout);
        if (_max_channel <= _servo_count) {
            ::write(_pwm_fd, _period, _max_channel*sizeof(_period[0]));
        } else {
            // we're using both sets of outputs
            ::write(_pwm_fd, _period, _servo_count*sizeof(_period[0]));
            if (_alt_fd != -1 && _alt_servo_count > 0) {
                uint8_t n = _max_channel - _servo_count;
                if (n > _alt_servo_count) {
                    n = _alt_servo_count;
                }
                ::write(_alt_fd, &_period[_servo_count], n*sizeof(_period[0]));
            }
        }

        // also publish to actuator_direct
        _publish_actuators();

        perf_end(_perf_rcout);
        _last_output = now;
    }

update_pwm:
    for (uint8_t i=0; i<ORB_MULTI_MAX_INSTANCES; i++) {
        bool rc_updated = false;
        if (_outputs[i].pwm_sub >= 0 && 
            orb_check(_outputs[i].pwm_sub, &rc_updated) == 0 && 
            rc_updated) {
            orb_copy(ORB_ID(actuator_outputs), _outputs[i].pwm_sub, &_outputs[i].outputs);
        }
    }

}

#endif // CONFIG_HAL_BOARD
