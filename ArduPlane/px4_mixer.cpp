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
#include <utility>

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
   create a mixer for a normal angle channel
*/
bool Plane::mix_one_channel(char *&buf, uint16_t &buf_size, uint8_t out_chan, uint8_t in_chan)
{
    const float limit = 10000;
    const SRV_Channel *outch = SRV_Channels::srv_channel(out_chan);

    bool is_throttle = in_chan==rcmap.throttle()-1;
    int16_t outch_trim = is_throttle?1500:outch->get_trim();

    outch_trim = constrain_int16(outch_trim, outch->get_output_min()+1, outch->get_output_max()-1);

    if (!print_buffer(buf, buf_size, "M: 1\n")) {
        return false;
    }

    int32_t out_min = limit*(outch_trim - outch->get_output_min()) / (1500 - PX4_LIM_RC_MIN);
    int32_t out_max = limit*(outch->get_output_max() - outch_trim) / (PX4_LIM_RC_MAX - 1500);
    int32_t out_trim = limit*(outch_trim - 1500) / ((PX4_LIM_RC_MAX - PX4_LIM_RC_MIN) / 2);
    int32_t reverse = outch->get_reversed()?-1:1;

    if (!print_buffer(buf, buf_size, "O: %d %d %d %d %d\n",
                      int(out_min*reverse),
                      int(out_max*reverse),
                      int(out_trim),
                      int(-limit), int(limit))) {
        return false;
    }
    if (!print_buffer(buf, buf_size, "S: 0 %u %d %d %d %d %d\n",
                      in_chan,
                      int(limit), int(limit),
                      0,
                      int(-limit), int(limit))) {
        return false;
    }
    return true;
}

/*
  mix two channels using elevon style mixer
*/
bool Plane::mix_two_channels(char *&buf, uint16_t &buf_size, uint8_t out_chan, uint8_t in_chan1, uint8_t in_chan2, bool left_channel)
{
    const float limit = 10000;
    const SRV_Channel *outch = SRV_Channels::srv_channel(out_chan);
    int16_t outch_trim = outch->get_trim();

    outch_trim = constrain_int16(outch_trim, outch->get_output_min()+1, outch->get_output_max()-1);

    if (!print_buffer(buf, buf_size, "M: 2\n")) {
        return false;
    }

    int32_t out_min = limit*(outch->get_trim() - outch->get_output_min()) / (1500 - PX4_LIM_RC_MIN);
    int32_t out_max = limit*(outch->get_output_max() - outch->get_trim()) / (PX4_LIM_RC_MAX - 1500);
    int32_t out_trim = limit*(outch_trim - 1500) / ((PX4_LIM_RC_MAX - PX4_LIM_RC_MIN) / 2);
    int32_t in_mul2 = left_channel?-1:1;
    float in_gain = g.mixing_gain;
    int32_t reverse = outch->get_reversed()?-1:1;

    if (!print_buffer(buf, buf_size, "O: %d %d %d %d %d\n",
                      int(out_min*reverse),
                      int(out_max*reverse),
                      int(out_trim),
                      int(-limit*2), int(limit*2))) {
        return false;
    }
    if (!print_buffer(buf, buf_size, "S: 0 %u %d %d %d %d %d\n",
                      in_chan1,
                      int(limit*in_gain), int(limit*in_gain),
                      0,
                      int(-limit), int(limit))) {
        return false;
    }
    if (!print_buffer(buf, buf_size, "S: 0 %u %d %d %d %d %d\n",
                      in_chan2,
                      int(limit*in_gain*in_mul2), int(limit*in_gain*in_mul2),
                      0,
                      int(-limit), int(limit))) {
        return false;
    }
    return true;
}

/* 
   create a mixer for k_manual and k_rcin*
*/
bool Plane::mix_passthrough(char *&buf, uint16_t &buf_size, uint8_t out_chan, uint8_t in_chan)
{
    const float limit = 10000;

    if (!print_buffer(buf, buf_size, "M: 1\n")) {
        return false;
    }

    if (!print_buffer(buf, buf_size, "O: %d %d %d %d %d\n",
                      int(limit),
                      int(limit),
                      0,
                      int(-limit), int(limit))) {
        return false;
    }
    if (!print_buffer(buf, buf_size, "S: 0 %u %d %d %d %d %d\n",
                      in_chan,
                      int(limit), int(limit),
                      0,
                      int(-limit), int(limit))) {
        return false;
    }
    return true;
}

/* 
   create a mixer for outputting trim only
*/
bool Plane::mix_trim_channel(char *&buf, uint16_t &buf_size, uint8_t out_chan)
{
    const float limit = 10000;
    const SRV_Channel *outch = SRV_Channels::srv_channel(out_chan);

    int16_t outch_trim = outch->get_trim();
    outch_trim = constrain_int16(outch_trim, outch->get_output_min()+1, outch->get_output_max()-1);

    if (!print_buffer(buf, buf_size, "M: 0\n")) {
        return false;
    }

    int32_t out_trim = limit*(outch_trim - 1500) / ((PX4_LIM_RC_MAX - PX4_LIM_RC_MIN) / 2);

    if (!print_buffer(buf, buf_size, "O: %d %d %d %d %d\n",
                      int(limit),
                      int(limit),
                      int(out_trim),
                      int(-limit), int(limit))) {
        return false;
    }
    return true;
}

/*
  create a PX4 mixer buffer given the current fixed wing parameters, returns the size of the buffer used
 */
uint16_t Plane::create_mixer(char *buf, uint16_t buf_size, const char *filename)
{
    char *buf0 = buf;
    uint16_t buf_size0 = buf_size;
    uint16_t manual_mask = uint16_t(g2.manual_rc_mask.get());

    for (uint8_t i=0; i<8; i++) {
        if ((1U<<i) & manual_mask) {
            // handle MANUAL_RCMASK channels
            mix_passthrough(buf, buf_size, i, i);
            continue;
        }
        SRV_Channel::Aux_servo_function_t function = SRV_Channels::channel_function(i);
        switch (function) {
        case SRV_Channel::k_aileron:
        case SRV_Channel::k_flaperon_left:
        case SRV_Channel::k_flaperon_right:
            mix_one_channel(buf, buf_size, i, rcmap.roll()-1);
            break;
        case SRV_Channel::k_elevator:
            mix_one_channel(buf, buf_size, i, rcmap.pitch()-1);
            break;
        case SRV_Channel::k_throttle:
            mix_one_channel(buf, buf_size, i, rcmap.throttle()-1);
            break;
        case SRV_Channel::k_rudder:
        case SRV_Channel::k_steering:
            mix_one_channel(buf, buf_size, i, rcmap.yaw()-1);
            break;
        case SRV_Channel::k_elevon_left:
        case SRV_Channel::k_dspoilerLeft1:
        case SRV_Channel::k_dspoilerLeft2:
            mix_two_channels(buf, buf_size, i, rcmap.pitch()-1, rcmap.roll()-1, true);
            break;
        case SRV_Channel::k_elevon_right:
        case SRV_Channel::k_dspoilerRight1:
        case SRV_Channel::k_dspoilerRight2:
            mix_two_channels(buf, buf_size, i, rcmap.pitch()-1, rcmap.roll()-1, false);
            break;
        case SRV_Channel::k_vtail_left:
            mix_two_channels(buf, buf_size, i, rcmap.pitch()-1, rcmap.yaw()-1, true);
            break;
        case SRV_Channel::k_vtail_right:
            mix_two_channels(buf, buf_size, i, rcmap.pitch()-1, rcmap.yaw()-1, false);
            break;
        case SRV_Channel::k_manual:
            mix_passthrough(buf, buf_size, i, i);
            break;
        case SRV_Channel::k_rcin1 ... SRV_Channel::k_rcin16:
            mix_passthrough(buf, buf_size, i, uint8_t(function - SRV_Channel::k_rcin1));
            break;
        default:
            mix_trim_channel(buf, buf_size, i);
            break;
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
    char *buf = nullptr;
    const uint16_t buf_size = 2048;
    uint16_t fileSize, new_crc;
    int px4io_fd = -1;
    enum AP_HAL::Util::safety_state old_state = hal.util->safety_switch_state();
    struct pwm_output_values pwm_values = {.values = {0}, .channel_count = 8};
    unsigned mixer_status = 0;
    uint16_t manual_mask = uint16_t(g2.manual_rc_mask.get());

    buf = (char *)calloc(1, buf_size);
    if (buf == nullptr) {
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
        SRV_Channels::set_safety_limit(SRV_Channel::k_throttle, aparm.throttle_min<0?SRV_Channel::SRV_CHANNEL_LIMIT_TRIM:SRV_Channel::SRV_CHANNEL_LIMIT_MIN);
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
        RC_Channel *ch = RC_Channels::rc_channel(i);
        if (ch == nullptr) {
            continue;
        }
        struct pwm_output_rc_config config;
        config.channel = i;
        // use high rc limits to allow for correct pass-thru channels
        // without limits
        config.rc_min = ch->get_radio_min();
        config.rc_max = ch->get_radio_max();
        if (rcmap.throttle()-1 == i) {
            // throttle uses a trim between min and max, so we don't get division
            // by small numbers near RC3_MIN
            config.rc_trim = (config.rc_min + config.rc_max)/2;
        } else {
            config.rc_trim = constrain_int16(ch->get_radio_trim(), config.rc_min+1, config.rc_max-1);
        }
        config.rc_dz = 0; // zero for the purposes of manual takeover

        config.rc_reverse = ch->get_reverse();

        if (i == 1) {
            // undo the reversal of channel2 in px4io
            config.rc_reverse = !config.rc_reverse;
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
        } else if (manual_mask & (1U<<i)) {
            // use fixed limits for manual_mask channels
            config.rc_assignment = i;
            config.rc_reverse = i==1?true:false;
            config.rc_max = PX4_LIM_RC_MAX;
            config.rc_min = PX4_LIM_RC_MIN;
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
        if (SRV_Channel::is_motor(SRV_Channels::channel_function(i))) {
            pwm_values.values[i] = quadplane.thr_min_pwm;
        } else {
            pwm_values.values[i] = PX4_LIM_RC_MIN;
        }
    }
    if (ioctl(px4io_fd, PWM_SERVO_SET_MIN_PWM, (long unsigned int)&pwm_values) != 0) {
        hal.console->printf("SET_MIN_PWM failed\n");
        goto failed;
    }

    for (uint8_t i = 0; i < pwm_values.channel_count; i++) {
        if (SRV_Channel::is_motor(SRV_Channels::channel_function(i))) {
            hal.rcout->write(i, quadplane.thr_min_pwm);
            pwm_values.values[i] = quadplane.thr_min_pwm;
        } else {
            pwm_values.values[i] = PX4_LIM_RC_MAX;
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
    if (buf != nullptr) {
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
