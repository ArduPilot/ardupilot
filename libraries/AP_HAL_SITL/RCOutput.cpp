#include <AP_HAL/AP_HAL.h>
#if CONFIG_HAL_BOARD == HAL_BOARD_SITL

#include "RCOutput.h"
#include <AP_BoardConfig/AP_BoardConfig.h>

#define ENABLE_DEBUG 0

#if ENABLE_DEBUG
# include <stdio.h>
# define Debug(fmt, args ...)  do {::printf("%s:%d: " fmt "\n", __FUNCTION__, __LINE__, ## args); } while (0)
#else
# define Debug(fmt, args ...)
#endif

using namespace HALSITL;

extern const AP_HAL::HAL& hal;

struct RCOutput::pwm_group RCOutput::pwm_group_list[] = { HAL_PWM_GROUPS };
uint8_t RCOutput::pwm_group_number = ARRAY_SIZE(pwm_group_list);
// marker for a disabled channel
#define CHAN_DISABLED 255

void RCOutput::init() {
    uint8_t pwm_count = AP_BoardConfig::get_pwm_count();

    for (uint8_t i = 0; i < pwm_group_number; i++ ) {
        //Start Pwm groups
        pwm_group &group = pwm_group_list[i];
        group.current_mode = MODE_PWM_NORMAL;
        for (uint8_t j = 0; j < 4; j++ ) {
            uint8_t chan = group.chan[j];
            if (chan >= pwm_count) {
                group.chan[j] = CHAN_DISABLED;
            }
            if (group.chan[j] != CHAN_DISABLED) {
                num_fmu_channels = MAX(num_fmu_channels, group.chan[j]+1);
                group.ch_mask |= (1U<<group.chan[j]);
            }
        }
        if (group.ch_mask != 0) {
            // _sitlState->output_ready = true; Todo : here ?
            group.pwm_started = true;
        }
    }
    // setup default output rate of 50Hz
    set_freq(0xFFFF, 50);
}

void RCOutput::set_freq(uint32_t chmask, uint16_t freq_hz)
{
    //check if the request spans accross any of the channel groups
    uint8_t update_mask = 0;
    /*
      we enable the new frequency on all groups that have one
      of the requested channels. This means we may enable high
      speed on some channels that aren't requested, but that
      is needed in order to fly a vehicle such a a hex
      multicopter properly
    */
    for (uint8_t i = 0; i < pwm_group_number; i++ ) {
        // greater than 400 doesn't give enough room at higher periods for
        // the down pulse. This still allows for high rate with oneshot and dshot.
        pwm_group &group = pwm_group_list[i];
        uint16_t group_freq = freq_hz;
        if (group_freq > 400 && group.current_mode != MODE_PWM_BRUSHED) {
            group_freq = 400;
        }
        if ((group.ch_mask & chmask) != 0) {
            group.rc_frequency = group_freq;
            update_mask |= group.ch_mask;
        }
        if (group_freq > 50) {
            fast_channel_mask |= group.ch_mask;
        }
    }
    if (chmask != update_mask) {
        hal.console->printf("RCOutput: Failed to set PWM frequency req %x set %x\n", (unsigned)chmask, (unsigned)update_mask);
    }
}
/*
  set default output rate
 */
void RCOutput::set_default_rate(uint16_t freq_hz)
{
    for (uint8_t i = 0; i < pwm_group_number; i++ ) {
        pwm_group &group = pwm_group_list[i];
        if ((group.ch_mask & fast_channel_mask) || group.ch_mask == 0) {
            // don't change fast channels
            continue;
        }
        group.rc_frequency = freq_hz;
    }
}

uint16_t RCOutput::get_freq(uint8_t ch)
{
    if (ch >= SITL_NUM_CHANNELS) {
        return 0;
    }

    for (uint8_t i = 0; i < pwm_group_number; i++ ) {
        pwm_group &group = pwm_group_list[i];
        for (uint8_t j = 0; j < 4; j++) {
            if (group.chan[j] == ch) {
                return group.rc_frequency;
            }
        }
    }
    // assume 50Hz default
    return 50;
}

void RCOutput::enable_ch(uint8_t ch)
{
    if (ch >= SITL_NUM_CHANNELS) {
        return;
    }

    for (uint8_t i = 0; i < pwm_group_number; i++ ) {
        pwm_group &group = pwm_group_list[i];
        for (uint8_t j = 0; j < 4; j++) {
            if ((group.chan[j] == ch) && !(_enable_mask & 1<<ch)) {
                Debug("enable_ch(%u)\n", ch);
                _enable_mask |= 1<<ch;
            }
        }
    }
}

void RCOutput::disable_ch(uint8_t ch)
{
    if (ch >= SITL_NUM_CHANNELS) {
        return;
    }

    for (uint8_t i = 0; i < pwm_group_number; i++ ) {
        pwm_group &group = pwm_group_list[i];
        for (uint8_t j = 0; j < 4; j++) {
            if (group.chan[j] == ch) {
                Debug("disable_ch(%u)\n", ch);
                _enable_mask &= ~(1<<ch);
            }
        }
    }
}

void RCOutput::write(uint8_t ch, uint16_t period_us)
{
    if (ch >= SITL_NUM_CHANNELS) {
        return;
    }
    _sitlState->output_ready = true;
    // keep unscaled value
    _last_sent[ch] = period_us;
    period[ch] = period_us;

    if (ch < num_fmu_channels) {
        active_fmu_channels = MAX(ch+1, active_fmu_channels);
        if (!_corked) {
            push_local();
        }
    }
}

/*
  push values to local channels from period[] array
 */
void RCOutput::push_local()
{
    if (active_fmu_channels == 0) {
        return;
    }
    uint16_t outmask = (1U<<active_fmu_channels)-1;
    outmask &= _enable_mask;

    for (uint8_t i = 0; i < pwm_group_number; i++ ) {
        pwm_group &group = pwm_group_list[i];
        if (!group.pwm_started) {
            continue;
        }
        for (uint8_t j = 0; j < 4; j++) {
            const uint8_t chan = group.chan[j];
            if (chan == CHAN_DISABLED) {
                continue;
            }
            if (outmask & (1UL<<chan)) {
                uint32_t period_us = period[chan];

                if (group.current_mode == MODE_PWM_BRUSHED) {
                    if (period_us <= _esc_pwm_min) {
                        period_us = 0;
                    } else if (period_us >= _esc_pwm_max) {
                        period_us = 100;
                    } else {
                        period_us = ((int32_t)period_us - _esc_pwm_min) * (int32_t)100 / (int32_t)(_esc_pwm_max - _esc_pwm_min);
                    }
                } else if (group.current_mode == MODE_PWM_ONESHOT125) {
                } else if (group.current_mode < MODE_PWM_DSHOT150) {
                } else if (group.current_mode >= MODE_PWM_DSHOT150 && group.current_mode <= MODE_PWM_DSHOT1200) {
                }
                _sitlState->pwm_output[chan] = period_us;
            }
        }
    }
}

uint16_t RCOutput::read(uint8_t ch)
{
    if (ch < SITL_NUM_CHANNELS) {
        // period[chan]; ?
        return _sitlState->pwm_output[ch];
    }
    return 0;
}

void RCOutput::read(uint16_t* period_us, uint8_t len)
{
    memcpy(period_us, _sitlState->pwm_output, len * sizeof(uint16_t));
}

uint16_t RCOutput::read_last_sent(uint8_t ch)
{
    if (ch >= SITL_NUM_CHANNELS) {
        return 0;
    }
    return _last_sent[ch];
}

void RCOutput::read_last_sent(uint16_t* period_us, uint8_t len)
{
    if (len > SITL_NUM_CHANNELS) {
        len = SITL_NUM_CHANNELS;
    }
    for (uint8_t i=0; i < len; i++) {
        period_us[i] = read_last_sent(i);
    }
}

void RCOutput::cork(void)
{
    _corked = true;
}

void RCOutput::push(void)
{
    _corked = false;
    push_local();
}


/*
  setup output mode for a group, using group.current_mode. Used to restore output
  after serial operations
 */
void RCOutput::set_group_mode(pwm_group &group)
{
    if (group.pwm_started) {
        group.pwm_started = false;
    }

    switch (group.current_mode) {
        case MODE_PWM_BRUSHED:
            // force zero output initially
            for (uint8_t i=0; i<4; i++) {
                if (group.chan[i] == CHAN_DISABLED) {
                    continue;
                }
                const uint8_t chan = group.chan[i];
                write(chan, 0);
                _sitlState->output_ready = false;
            }
            break;

        case MODE_PWM_DSHOT150 ... MODE_PWM_DSHOT1200: {
            break;
        }

        case MODE_PWM_ONESHOT:
        case MODE_PWM_ONESHOT125:
            // for oneshot we set a period of 0, which results in no pulses till we trigger
            group.rc_frequency = 1;
            if (group.pwm_started) {
            }
            break;

        case MODE_PWM_NORMAL:
        case MODE_PWM_NONE:
            // nothing needed
            break;
    }

    if (group.current_mode != MODE_PWM_NONE &&
        !group.pwm_started) {
        group.pwm_started = true;
        for (uint8_t j=0; j<4; j++) {
            if (group.chan[j] != CHAN_DISABLED) {
            }
        }
    }
}

/*
  setup output mode
 */
void RCOutput::set_output_mode(uint16_t mask, enum output_mode mode)
{
    for (uint8_t i = 0; i < pwm_group_number; i++ ) {
        pwm_group &group = pwm_group_list[i];
        if (((group.ch_mask) & mask) == 0) {
            // this group is not affected
            continue;
        }
        if (mode > MODE_PWM_NORMAL) {
            fast_channel_mask |= group.ch_mask;
        }
        if (group.current_mode != mode) {
            group.current_mode = mode;
            set_group_mode(group);
        }
    }
}

#endif
