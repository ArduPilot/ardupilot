#include "AP_RCSwitch.h"

#include "RC_Channel.h"

#include <GCS_MAVLink/GCS.h>

// init_aux_switch_function - initialize aux functions
void AP_RCSwitch::init_aux_function(const aux_func_t ch_option, const aux_switch_pos_t ch_flag)
{
    // init channel options
    switch(ch_option) {
    // the following functions to not need to be initialised:
    case DO_NOTHING:
        break;
    default:
        gcs().send_text(MAV_SEVERITY_WARNING, "Failed to initialise RC function (%u)", ch_option);
        break;
    }
}

// read_aux_switches - checks aux switch positions and invokes configured actions
void AP_RCSwitch::read_aux_all()
{
    if (in_rc_failsafe()) {
        // exit immediately during radio failsafe
        return;
    }

    for (uint8_t i=0; i<NUM_RC_CHANNELS; i++) {
        const RC_Channel *channel = AP::rc().rc_channel(i);
        if (channel == nullptr) {
            continue;
        }
        const aux_func_t option = (aux_func_t)channel->option.get();
        if (option == DO_NOTHING) {
            // may wish to add special cases for other "AUXSW" things
            // here e.g. RCMAP_ROLL etc once they become options
            continue;
        }
        const aux_switch_pos_t new_position = read_3pos_switch(channel);
        const aux_switch_pos_t old_position = old_switch_position(i);
        if (new_position == old_position) {
            continue;
        }
        do_aux_function(option, new_position);
        set_old_switch_position(i, new_position);
    }
}

void AP_RCSwitch::do_aux_function_camera_trigger(const aux_switch_pos_t ch_flag)
{
    AP_Camera *camera = AP::camera();
    if (camera == nullptr) {
        return;
    }
    if (ch_flag == HIGH) {
        camera->take_picture();
    }
}

void AP_RCSwitch::do_aux_function(const aux_func_t ch_option, const aux_switch_pos_t ch_flag)
{
    switch(ch_option) {
    case CAMERA_TRIGGER:
        do_aux_function_camera_trigger(ch_flag);
        break;
    default:
        gcs().send_text(MAV_SEVERITY_INFO, "Invalid channel option (%u)", ch_option);
        break;
    }
}


void AP_RCSwitch::init()
{
    for (uint8_t i=0; i<NUM_RC_CHANNELS; i++) {
        const RC_Channel *channel = AP::rc().rc_channel(i);
        if (channel == nullptr) {
            continue;
        }
        const aux_switch_pos_t position = read_3pos_switch(channel);
        set_old_switch_position(i, position);
        init_aux_function((aux_func_t)channel->option.get(), position);
   }
   reset_control_switch();
}

// read_3pos_switch
AP_RCSwitch::aux_switch_pos_t AP_RCSwitch::read_3pos_switch(const RC_Channel *channel) const
{
    const uint16_t in = channel->get_radio_in();
    if (in < AUX_PWM_TRIGGER_LOW) return LOW;   // switch is in low position
    if (in > AUX_PWM_TRIGGER_HIGH) return HIGH; // switch is in high position
    return MIDDLE;                              // switch is in middle position
}

RC_Channel *AP_RCSwitch::find_channel_for_option(const aux_func_t option) const
{
    for (uint8_t i=0; i<NUM_RC_CHANNELS; i++) {
        RC_Channel *channel = AP::rc().rc_channel(i);
        if (channel == nullptr) {
            // odd?
            continue;
        }
        if ((aux_func_t)channel->option.get() == option) {
            return channel;
        }
    }
    return nullptr;
}

// duplicate_options_exist - returns true if any options are duplicated
bool AP_RCSwitch::duplicate_options_exist() const
{
    uint8_t auxsw_option_counts[256] = {};
    for (uint8_t i=0; i<NUM_RC_CHANNELS; i++) {
        RC_Channel *channel = AP::rc().rc_channel(i);
        if (channel == nullptr) {
            // odd?
            continue;
        }
        uint16_t option = channel->option.get();
        if (option > sizeof(auxsw_option_counts)) {
            AP_HAL::panic("8-bit limit on options");
        }
        auxsw_option_counts[option]++;
    }

    for (uint16_t i=0; i<sizeof(auxsw_option_counts); i++) {
        if (i == 0) { // MAGIC VALUE! This is AUXSW_DO_NOTHING
            continue;
        }
        if (auxsw_option_counts[i] > 1) {
            return true;
        }
    }
   return false;
}
