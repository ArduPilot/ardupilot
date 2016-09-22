// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

#include "Plane.h"

/*
  handle creation of PX4 mixer file, for failover to direct RC control
  on failure of FMU

  This will create APM/MIXER.MIX on the microSD card. The user may
  also create APM/CUSTOM.MIX, and if it exists that will be used
  instead. That allows the user to setup more complex failsafe mixes
  that include flaps, landing gear, ignition cut etc
 */

#if HAVE_PX4_MIXER
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <unistd.h>
#include <stdio.h>
#include <drivers/drv_pwm_output.h>
#include <systemlib/mixer/mixer.h>
#include <modules/px4iofirmware/protocol.h>
#include <GCS_MAVLink/include/mavlink/v2.0/checksum.h>

#define PX4_LIM_RC_MIN 900
#define PX4_LIM_RC_MAX 2100

/*
  formatted print to a buffer with buffer advance. Returns true on
  success, false on fail
 */
bool Plane::print_buffer(char *&buf, uint16_t &buf_size, const char *fmt, ...)
{
    va_list arg_list;
    va_start(arg_list, fmt);
    int n = ::vsnprintf(buf, buf_size, fmt, arg_list);
    va_end(arg_list);
    if (n <= 0 || n >= buf_size) {
        return false;
    }
    buf += n;
    buf_size -= n;
    return true;
}

/*
  create a PX4 mixer buffer given the current fixed wing parameters, returns the size of the buffer used
 */
uint16_t Plane::create_mixer(char *buf, uint16_t buf_size, const char *filename)
{
    char *buf0 = buf;
    uint16_t buf_size0 = buf_size;

    /*
      this is the equivalent of channel_output_mixer()
     */
    const int8_t mixmul[5][2] = { { 0, 0 }, { 1, 1 }, { 1, -1 }, { -1, 1 }, { -1, -1 }};
    // these are the internal clipping limits. Use scale_max1 when
    // clipping to user specified min/max is wanted. Use scale_max2
    // when no clipping is wanted (simulated by setting a very large
    // clipping value)
    const float scale_max1 = 10000;
    const float scale_max2 = 1000000;
    // range for mixers
    const uint16_t mix_max = scale_max1 * g.mixing_gain;
    // scaling factors used by PX4IO between pwm and internal values,
    // as configured in setup_failsafe_mixing() below
    const float pwm_min = PX4_LIM_RC_MIN;
    const float pwm_max = PX4_LIM_RC_MAX;
    const float pwm_scale = 2*scale_max1/(pwm_max - pwm_min);

    for (uint8_t i=0; i<8; i++) {
        int32_t c1, c2, mix=0;
        bool rev = false;
        RC_Channel_aux::Aux_servo_function_t function = RC_Channel_aux::channel_function(i);
        if (i == rcmap.pitch()-1 && g.vtail_output > MIXING_DISABLED && g.vtail_output <= MIXING_DNDN) {
            // first channel of VTAIL mix
            c1 = rcmap.yaw()-1;
            c2 = i;
            rev = false;
            mix = -mix_max*mixmul[g.vtail_output][0];
        } else if (i == rcmap.yaw()-1 && g.vtail_output > MIXING_DISABLED && g.vtail_output <= MIXING_DNDN) {
            // second channel of VTAIL mix
            c1 = rcmap.pitch()-1;
            c2 = i;
            rev = true;
            mix = mix_max*mixmul[g.vtail_output][1];
        } else if (i == rcmap.roll()-1 && g.elevon_output > MIXING_DISABLED && 
                   g.elevon_output <= MIXING_DNDN && g.vtail_output == 0) {
            // first channel of ELEVON mix
            c1 = i;
            c2 = rcmap.pitch()-1;
            rev = true;
            mix = mix_max*mixmul[g.elevon_output][1];
        } else if (i == rcmap.pitch()-1 && g.elevon_output > MIXING_DISABLED && 
                   g.elevon_output <= MIXING_DNDN && g.vtail_output == 0) {
            // second channel of ELEVON mix
            c1 = i;
            c2 = rcmap.roll()-1;
            rev = false;
            mix = mix_max*mixmul[g.elevon_output][0];
        } else if (function == RC_Channel_aux::k_aileron || 
                   function == RC_Channel_aux::k_flaperon1 || 
                   function == RC_Channel_aux::k_flaperon2) {
            // a secondary aileron. We don't mix flap input in yet for flaperons
            c1 = rcmap.roll()-1;
        } else if (function == RC_Channel_aux::k_elevator) {
            // a secondary elevator
            c1 = rcmap.pitch()-1;
        } else if (function == RC_Channel_aux::k_rudder || 
                   function == RC_Channel_aux::k_steering) {
            // a secondary rudder or wheel
            c1 = rcmap.yaw()-1;
        } else if (g.flapin_channel > 0 &&
                   (function == RC_Channel_aux::k_flap ||
                    function == RC_Channel_aux::k_flap_auto)) {
            // a flap output channel, and we have a manual flap input channel
            c1 = g.flapin_channel-1;
        } else if (i < 4 ||
                   function == RC_Channel_aux::k_elevator_with_input ||
                   function == RC_Channel_aux::k_aileron_with_input ||
                   function == RC_Channel_aux::k_manual) {
            // a pass-thru channel
            c1 = i;
        } else {
            // a empty output
            if (!print_buffer(buf, buf_size, "Z:\n")) {
                return 0;
            }
            continue;
        }
        if (mix == 0) {
            // pass through channel, possibly with reversal. We also
            // adjust the gain based on the range of input and output
            // channels and adjust for trims
            const RC_Channel *chan1 = RC_Channel::rc_channel(i);
            const RC_Channel *chan2 = RC_Channel::rc_channel(c1);
            int16_t chan1_trim = (i==rcmap.throttle()-1?1500:chan1->get_radio_trim());
            int16_t chan2_trim = (c1==rcmap.throttle()-1?1500:chan2->get_radio_trim());
            chan1_trim = constrain_int16(chan1_trim, PX4_LIM_RC_MIN+1, PX4_LIM_RC_MAX-1);
            chan2_trim = constrain_int16(chan2_trim, PX4_LIM_RC_MIN+1, PX4_LIM_RC_MAX-1);
            // if the input and output channels are the same then we
            // apply clipping. This allows for direct pass-thru
            int32_t limit = (c1==i?scale_max2:scale_max1);
            int32_t in_scale_low;
            if (chan2_trim <= chan2->get_radio_min()) {
                in_scale_low = scale_max1;
            } else {
                in_scale_low = scale_max1*(chan2_trim - pwm_min)/(float)(chan2_trim - chan2->get_radio_min());
            }
            int32_t in_scale_high;
            if (chan2->get_radio_max() <= chan2_trim) {
                in_scale_high = scale_max1;
            } else {
                in_scale_high = scale_max1*(pwm_max - chan2_trim)/(float)(chan2->get_radio_max() - chan2_trim);
            }
            if (chan1->get_reverse() != chan2->get_reverse()) {
                in_scale_low = -in_scale_low;
                in_scale_high = -in_scale_high;
            }
            if (!print_buffer(buf, buf_size, "M: 1\n") ||
                !print_buffer(buf, buf_size, "O: %d %d %d %d %d\n",
                              (int)(pwm_scale*(chan1_trim - chan1->get_radio_min())),
                              (int)(pwm_scale*(chan1->get_radio_max() - chan1_trim)),
                              (int)(pwm_scale*(chan1_trim - 1500)),
                              (int)-scale_max2, (int)scale_max2) ||
                !print_buffer(buf, buf_size, "S: 0 %u %d %d %d %d %d\n", c1,
                              in_scale_low,
                              in_scale_high,
                              0,
                              -limit, limit)) {
                return 0;
            }
        } else {
            const RC_Channel *chan1 = RC_Channel::rc_channel(c1);
            const RC_Channel *chan2 = RC_Channel::rc_channel(c2);
            int16_t chan1_trim = (c1==rcmap.throttle()-1?1500:chan1->get_radio_trim());
            int16_t chan2_trim = (c2==rcmap.throttle()-1?1500:chan2->get_radio_trim());
            chan1_trim = constrain_int16(chan1_trim, PX4_LIM_RC_MIN+1, PX4_LIM_RC_MAX-1);
            chan2_trim = constrain_int16(chan2_trim, PX4_LIM_RC_MIN+1, PX4_LIM_RC_MAX-1);
            // mix of two input channels to give an output channel. To
            // make the mixer match the behaviour of APM we need to
            // scale and offset the input channels to undo the affects
            // of the PX4IO input processing
            if (!print_buffer(buf, buf_size, "M: 2\n") ||
                !print_buffer(buf, buf_size, "O: %d %d 0 %d %d\n", mix, mix, (int)-scale_max1, (int)scale_max1)) {
                return 0;
            }
            int32_t in_scale_low = pwm_scale*(chan1_trim - pwm_min);
            int32_t in_scale_high = pwm_scale*(pwm_max - chan1_trim);
            int32_t offset = pwm_scale*(chan1_trim - 1500);
            if (!print_buffer(buf, buf_size, "S: 0 %u %d %d %d %d %d\n", 
                              c1, in_scale_low, in_scale_high, offset,
                              (int)-scale_max2, (int)scale_max2)) {
                return 0;
            }
            in_scale_low = pwm_scale*(chan2_trim - pwm_min);
            in_scale_high = pwm_scale*(pwm_max - chan2_trim);
            offset = pwm_scale*(chan2_trim - 1500);
            if (rev) {
                if (!print_buffer(buf, buf_size, "S: 0 %u %d %d %d %d %d\n", 
                                  c2, in_scale_low, in_scale_high, offset,
                                  (int)-scale_max2, (int)scale_max2)) {
                    return 0;
                }
            } else {
                if (!print_buffer(buf, buf_size, "S: 0 %u %d %d %d %d %d\n", 
                                  c2, -in_scale_low, -in_scale_high, -offset,
                                  (int)-scale_max2, (int)scale_max2)) {
                    return 0;
                }
            }
        }
    }    

    /*
      if possible, also write to a file for debugging purposes
     */
    int mix_fd = open(filename, O_WRONLY|O_CREAT|O_TRUNC, 0644);
    if (mix_fd != -1) {
        write(mix_fd, buf0, buf_size0 - buf_size);
        close(mix_fd);
    }
    return buf_size0 - buf_size;
}


/*
  setup mixer on PX4 so that if FMU dies the pilot gets manual control
 */
bool Plane::setup_failsafe_mixing(void)
{
    const char *mixer_filename = "/fs/microsd/APM/MIXER.MIX";
    bool ret = false;
    char *buf = NULL;
    const uint16_t buf_size = 2048;
    uint16_t fileSize, new_crc;
    int px4io_fd = -1;
    enum AP_HAL::Util::safety_state old_state = hal.util->safety_switch_state();
    struct pwm_output_values pwm_values = {.values = {0}, .channel_count = 8};
    unsigned mixer_status = 0;

    buf = (char *)malloc(buf_size);
    if (buf == NULL) {
        goto failed;
    }

    fileSize = create_mixer(buf, buf_size, mixer_filename);
    if (!fileSize) {
        hal.console->printf("Unable to create mixer\n");
        goto failed;
    }

    new_crc = crc_calculate((uint8_t *)buf, fileSize);

    if ((int32_t)new_crc == last_mixer_crc) {
        free(buf);
        return true;
    } else {
        last_mixer_crc = new_crc;
    }

    px4io_fd = open("/dev/px4io", 0);
    if (px4io_fd == -1) {
        // px4io isn't started, no point in setting up a mixer
        goto failed;
    }

    if (old_state == AP_HAL::Util::SAFETY_ARMED) {
        // make sure the throttle has a non-zero failsafe value before we
        // disable safety. This prevents sending zero PWM during switch over
        hal.rcout->set_safety_pwm(1UL<<(rcmap.throttle()-1), throttle_min());
    }

    // we need to force safety on to allow us to load a mixer. We call
    // it twice as there have been reports that this call can fail
    // with a small probability
    hal.rcout->force_safety_on();
    hal.rcout->force_safety_no_wait();

    /* reset any existing mixer in px4io. This shouldn't be needed,
     * but is good practice */
    if (ioctl(px4io_fd, MIXERIOCRESET, 0) != 0) {
        hal.console->printf("Unable to reset mixer\n");
        goto failed;
    }

	/* pass the buffer to the device */
    if (ioctl(px4io_fd, MIXERIOCLOADBUF, (unsigned long)buf) != 0) {
        hal.console->printf("Unable to send mixer to IO\n");
        goto failed;        
    }

    // setup RC config for each channel based on user specified
    // mix/max/trim. We only do the first 8 channels due to 
    // a RC config limitation in px4io.c limiting to PX4IO_RC_MAPPED_CONTROL_CHANNELS
    for (uint8_t i=0; i<8; i++) {
        RC_Channel *ch = RC_Channel::rc_channel(i);
        if (ch == NULL) {
            continue;
        }
        struct pwm_output_rc_config config;
        /*
          we use a min/max of 900/2100 to allow for pass-thru of
          larger values than the RC min/max range. This mimics the APM
          behaviour of pass-thru in manual, which allows for dual-rate
          transmitter setups in manual mode to go beyond the ranges
          used in stabilised modes
         */
        config.channel = i;
        config.rc_min = 900;
        config.rc_max = 2100;
        if (rcmap.throttle()-1 == i) {
            // throttle uses a trim of 1500, so we don't get division
            // by small numbers near RC3_MIN
            config.rc_trim = 1500;
        } else {
            config.rc_trim = constrain_int16(ch->get_radio_trim(), config.rc_min+1, config.rc_max-1);
        }
        config.rc_dz = 0; // zero for the purposes of manual takeover

        // we set reverse as false, as users of ArduPilot will have
        // input reversed on transmitter, so from the point of view of
        // the mixer the input is never reversed. The one exception is
        // the 2nd channel, which is reversed inside the PX4IO code,
        // so needs to be unreversed here to give sane behaviour.
        if (i == 1) {
            config.rc_reverse = true;
        } else {
            config.rc_reverse = false;
        }

        if (i+1 == g.override_channel.get()) {
            /*
              This is an OVERRIDE_CHAN channel. We want IO to trigger
              override with a channel input of over 1750. The px4io
              code is setup for triggering below 80% of the range below
              trim. To  map this to values above 1750 we need to reverse
              the direction and set the rc range for this channel to 1000
              to 1813 (1812.5 = 1500 + 250/0.8)
             */
            config.rc_assignment = PX4IO_P_RC_CONFIG_ASSIGNMENT_MODESWITCH;
            config.rc_reverse = true;
            config.rc_max = 1813; // round 1812.5 up to grant > 1750
            config.rc_min = 1000;
            config.rc_trim = 1500;
        } else {
            config.rc_assignment = i;
        }

        if (ioctl(px4io_fd, PWM_SERVO_SET_RC_CONFIG, (unsigned long)&config) != 0) {
            hal.console->printf("SET_RC_CONFIG failed\n");
            goto failed;
        }
    }

    for (uint8_t i = 0; i < pwm_values.channel_count; i++) {
        if (RC_Channel_aux::channel_function(i) >= RC_Channel_aux::k_motor1 &&
            RC_Channel_aux::channel_function(i) <= RC_Channel_aux::k_motor8) {
            pwm_values.values[i] = quadplane.thr_min_pwm;
        } else {
            pwm_values.values[i] = 900;
        }
    }
    if (ioctl(px4io_fd, PWM_SERVO_SET_MIN_PWM, (long unsigned int)&pwm_values) != 0) {
        hal.console->printf("SET_MIN_PWM failed\n");
        goto failed;
    }

    for (uint8_t i = 0; i < pwm_values.channel_count; i++) {
        if (RC_Channel_aux::channel_function(i) >= RC_Channel_aux::k_motor1 &&
            RC_Channel_aux::channel_function(i) <= RC_Channel_aux::k_motor8) {
            hal.rcout->write(i, quadplane.thr_min_pwm);
            pwm_values.values[i] = quadplane.thr_min_pwm;
        } else {
            pwm_values.values[i] = 2100;
        }
    }
    if (ioctl(px4io_fd, PWM_SERVO_SET_MAX_PWM, (long unsigned int)&pwm_values) != 0) {
        hal.console->printf("SET_MAX_PWM failed\n");
        goto failed;
    }
    if (ioctl(px4io_fd, PWM_SERVO_SET_OVERRIDE_OK, 0) != 0) {
        hal.console->printf("SET_OVERRIDE_OK failed\n");
        goto failed;
    }

    if (ioctl(px4io_fd, PWM_SERVO_SET_OVERRIDE_IMMEDIATE, 1) != 0) {
        hal.console->printf("SET_OVERRIDE_IMMEDIATE failed\n");
        goto failed;
    }

    if (ioctl(px4io_fd, PWM_IO_GET_STATUS, (unsigned long)&mixer_status) != 0 ||
        (mixer_status & PX4IO_P_STATUS_FLAGS_MIXER_OK) != 0) {
        hal.console->printf("Mixer failed: 0x%04x\n", mixer_status);
        goto failed;
    }

    ret = true;

failed:
    if (buf != NULL) {
        free(buf);
    }
    if (px4io_fd != -1) {
        close(px4io_fd);
    }
    // restore safety state if it was previously armed
    if (old_state == AP_HAL::Util::SAFETY_ARMED) {
        hal.rcout->force_safety_off();
        hal.rcout->force_safety_no_wait();
    }
    if (!ret) {
        // clear out the mixer CRC so that we will attempt to send it again
        last_mixer_crc = -1;
    }
    return ret;
}


#endif // CONFIG_HAL_BOARD
