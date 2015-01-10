// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

/*
  handle creation of PX4 mixer file, for failover to direct RC control
  on failure of FMU

  This will create APM/MIXER.MIX on the microSD card. The user may
  also create APM/CUSTOM.MIX, and if it exists that will be used
  instead. That allows the user to setup more complex failsafe mixes
  that include flaps, landing gear, ignition cut etc
 */

#if CONFIG_HAL_BOARD == HAL_BOARD_PX4
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <unistd.h>
#include <stdio.h>
#include <drivers/drv_pwm_output.h>
#include <systemlib/mixer/mixer.h>


/*
  create a mixer file given key fixed wing parameters
 */
static bool create_mixer_file(const char *filename)
{
    int mix_fd = open(filename, O_WRONLY|O_CREAT|O_TRUNC, 0644);
    if (mix_fd == -1) {
        hal.console->printf("Unable to create mixer file\n");
        return false;
    }
    dprintf(mix_fd, "Auto-generated mixer file for ArduPilot\n\n");

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
    const float pwm_min = 900;
    const float pwm_max = 2100;
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
            dprintf(mix_fd, "Z:\n\n");
            continue;
        }
        if (mix == 0) {
            // pass thru channel, possibly with reversal. We also
            // adjust the gain based on the range of input and output
            // channels and adjust for trims
            const RC_Channel *chan1 = RC_Channel::rc_channel(i);
            const RC_Channel *chan2 = RC_Channel::rc_channel(c1);
            int16_t chan1_trim = (i==rcmap.throttle()-1?1500:chan1->radio_trim);
            int16_t chan2_trim = (c1==rcmap.throttle()-1?1500:chan2->radio_trim);
            // if the input and output channels are the same then we
            // apply clipping. This allows for direct pass-thru
            int32_t limit = (c1==i?scale_max2:scale_max1);
            int32_t in_scale_low;
            if (chan2_trim <= chan2->radio_min) {
                in_scale_low = scale_max1;
            } else {
                in_scale_low = scale_max1*(chan2_trim - pwm_min)/(float)(chan2_trim - chan2->radio_min);
            }
            int32_t in_scale_high;
            if (chan2->radio_max <= chan2_trim) {
                in_scale_high = scale_max1;
            } else {
                in_scale_high = scale_max1*(pwm_max - chan2_trim)/(float)(chan2->radio_max - chan2_trim);
            }
            if (chan1->get_reverse() != chan2->get_reverse()) {
                in_scale_low = -in_scale_low;
                in_scale_high = -in_scale_high;
            }
            dprintf(mix_fd, "M: 1\n");
            dprintf(mix_fd, "O: %d %d %d %d %d\n",
                    (int)(pwm_scale*(chan1_trim - chan1->radio_min)),
                    (int)(pwm_scale*(chan1->radio_max - chan1_trim)),
                    (int)(pwm_scale*(chan1_trim - 1500)),
                    (int)-scale_max2, (int)scale_max2);
            dprintf(mix_fd, "S: 0 %u %d %d %d %d %d\n\n", c1,
                    in_scale_low,
                    in_scale_high,
                    0,
                    -limit, limit);
        } else {
            const RC_Channel *chan1 = RC_Channel::rc_channel(c1);
            const RC_Channel *chan2 = RC_Channel::rc_channel(c2);
            int16_t chan1_trim = (c1==rcmap.throttle()-1?1500:chan1->radio_trim);
            int16_t chan2_trim = (c2==rcmap.throttle()-1?1500:chan2->radio_trim);
            // mix of two input channels to give an output channel. To
            // make the mixer match the behaviour of APM we need to
            // scale and offset the input channels to undo the affects
            // of the PX4IO input processing
            dprintf(mix_fd, "M: 2\n");
            dprintf(mix_fd, "O: %d %d 0 %d %d\n", mix, mix, (int)-scale_max1, (int)scale_max1);
            int32_t in_scale_low = pwm_scale*(chan1_trim - pwm_min);
            int32_t in_scale_high = pwm_scale*(pwm_max - chan1_trim);
            int32_t offset = pwm_scale*(chan1_trim - 1500);
            dprintf(mix_fd, "S: 0 %u %d %d %d %d %d\n", 
                    c1, in_scale_low, in_scale_high, offset,
                    (int)-scale_max2, (int)scale_max2);
            in_scale_low = pwm_scale*(chan2_trim - pwm_min);
            in_scale_high = pwm_scale*(pwm_max - chan2_trim);
            offset = pwm_scale*(chan2_trim - 1500);
            if (rev) {
                dprintf(mix_fd, "S: 0 %u %d %d %d %d %d\n\n", 
                        c2, in_scale_low, in_scale_high, offset,
                        (int)-scale_max2, (int)scale_max2);
            } else {
                dprintf(mix_fd, "S: 0 %u %d %d %d %d %d\n\n", 
                        c2, -in_scale_low, -in_scale_high, -offset,
                        (int)-scale_max2, (int)scale_max2);
            }
        }
    }    
    close(mix_fd);
    return true;
}


/*
  setup mixer on PX4 so that if FMU dies the pilot gets manual control
 */
static bool setup_failsafe_mixing(void)
{
    // we create MIXER.MIX regardless of whether we will be using it,
    // as it gives a template for the user to modify to create their
    // own CUSTOM.MIX file
    const char *mixer_filename = "/fs/microsd/APM/MIXER.MIX";
    const char *custom_mixer_filename = "/fs/microsd/APM/CUSTOM.MIX";
    bool ret = false;
    char *buf = NULL;
    const uint16_t buf_size = 2048;

    if (!create_mixer_file(mixer_filename)) {
        return false;
    }

    struct stat st;
    const char *filename;
    if (stat(custom_mixer_filename, &st) == 0) {
        filename = custom_mixer_filename;
    } else {
        filename = mixer_filename;
    }

    enum AP_HAL::Util::safety_state old_state = hal.util->safety_switch_state();
    struct pwm_output_values pwm_values = {.values = {0}, .channel_count = 8};

    int px4io_fd = open("/dev/px4io", 0);
    if (px4io_fd == -1) {
        // px4io isn't started, no point in setting up a mixer
        return false;
    }

    buf = (char *)malloc(buf_size);
    if (buf == NULL) {
        goto failed;
    }
    if (load_mixer_file(filename, &buf[0], buf_size) != 0) {
        hal.console->printf("Unable to load %s\n", filename);
        goto failed;
    }

    if (old_state == AP_HAL::Util::SAFETY_ARMED) {
        // make sure the throttle has a non-zero failsafe value before we
        // disable safety. This prevents sending zero PWM during switch over
        hal.rcout->set_safety_pwm(1UL<<(rcmap.throttle()-1), channel_throttle->radio_min);
    }

    // we need to force safety on to allow us to load a mixer
    hal.rcout->force_safety_on();

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
            config.rc_trim = ch->radio_trim;
        }
        config.rc_dz = 0; // zero for the purposes of manual takeover
        config.rc_assignment = i;
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
        if (ioctl(px4io_fd, PWM_SERVO_SET_RC_CONFIG, (unsigned long)&config) != 0) {
            hal.console->printf("SET_RC_CONFIG failed\n");
        }
    }

    for (uint8_t i = 0; i < pwm_values.channel_count; i++) {
        pwm_values.values[i] = 900;
    }
    ioctl(px4io_fd, PWM_SERVO_SET_MIN_PWM, (long unsigned int)&pwm_values);

    for (uint8_t i = 0; i < pwm_values.channel_count; i++) {
        pwm_values.values[i] = 2100;
    }
    ioctl(px4io_fd, PWM_SERVO_SET_MAX_PWM, (long unsigned int)&pwm_values);
    ioctl(px4io_fd, PWM_SERVO_SET_OVERRIDE_OK, 0);

    // setup for immediate manual control if FMU dies
    ioctl(px4io_fd, PWM_SERVO_SET_OVERRIDE_IMMEDIATE, 1);

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
    }
    return ret;
}


#endif // CONFIG_HAL_BOARD
