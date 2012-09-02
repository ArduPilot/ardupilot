// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: t -*-

#include <APM_RC.h>
#include "RC_Channel_aux.h"

const AP_Param::GroupInfo RC_Channel_aux::var_info[] PROGMEM = {
    AP_NESTEDGROUPINFO(RC_Channel, 0),

    // @Param: FUNCTION
    // @DisplayName: Servo out function
    // @Description: Setting this to Disabled(0) will disable this output, any other value will enable the corresponding function
    // @Values: 0:Disabled,1:Manual,2:Flap,3:Flap_auto,4:Aileron,5:flaperon,6:mount_pan,7:mount_tilt,8:mount_roll,9:mount_open,10:camera_trigger,11:release
    // @User: Standard
    AP_GROUPINFO("FUNCTION",       1, RC_Channel_aux, function, 0),

    AP_GROUPEND
};

/// Global pointer array, indexed by a "RC function enum" and points to the RC channel output assigned to that function/operation
RC_Channel_aux* g_rc_function[RC_Channel_aux::k_nr_aux_servo_functions];

/// enable_out_ch - enable the channel through APM_RC
void
RC_Channel_aux::enable_out_ch(unsigned char ch_nr)
{
    // enable_out this channel if it assigned to a function
    if( function != k_none ) {
        _apm_rc->enable_out(ch_nr);
    }
}

/// map a function to a servo channel and output it
void
RC_Channel_aux::output_ch(unsigned char ch_nr)
{
    // take care or two corner cases
    switch(function)
    {
    case k_none:                // disabled
        return;
        break;
    case k_manual:              // manual
        radio_out = radio_in;
        break;
    }
    _apm_rc->OutputCh(ch_nr, radio_out);
}

/// Update the g_rc_function array of pointers to rc_x channels
/// This is to be done before rc_init so that the channels get correctly initialized.
/// It also should be called periodically because the user might change the configuration and
/// expects the changes to take effect instantly
/// Supports up to seven aux servo outputs (typically CH5 ... CH11)
/// All servos must be configured with a single call to this function
/// (do not call this twice with different parameters, the second call will reset the effect of the first call)
void update_aux_servo_function( RC_Channel_aux* rc_a,
                                RC_Channel_aux* rc_b,
                                RC_Channel_aux* rc_c,
                                RC_Channel_aux* rc_d,
                                RC_Channel_aux* rc_e,
                                RC_Channel_aux* rc_f,
                                RC_Channel_aux* rc_g)
{
    RC_Channel_aux::Aux_servo_function_t aux_servo_function[7];
    aux_servo_function[0] = (rc_a == NULL) ? RC_Channel_aux::k_none : (RC_Channel_aux::Aux_servo_function_t)rc_a->function.get();
    aux_servo_function[1] = (rc_b == NULL) ? RC_Channel_aux::k_none : (RC_Channel_aux::Aux_servo_function_t)rc_b->function.get();
    aux_servo_function[2] = (rc_c == NULL) ? RC_Channel_aux::k_none : (RC_Channel_aux::Aux_servo_function_t)rc_c->function.get();
    aux_servo_function[3] = (rc_d == NULL) ? RC_Channel_aux::k_none : (RC_Channel_aux::Aux_servo_function_t)rc_d->function.get();
    aux_servo_function[4] = (rc_e == NULL) ? RC_Channel_aux::k_none : (RC_Channel_aux::Aux_servo_function_t)rc_e->function.get();
    aux_servo_function[5] = (rc_f == NULL) ? RC_Channel_aux::k_none : (RC_Channel_aux::Aux_servo_function_t)rc_f->function.get();
    aux_servo_function[6] = (rc_g == NULL) ? RC_Channel_aux::k_none : (RC_Channel_aux::Aux_servo_function_t)rc_g->function.get();

    for (uint8_t i = 0; i < 7; i++) {
        if (aux_servo_function[i] >= RC_Channel_aux::k_nr_aux_servo_functions) {
            // invalid setting
            aux_servo_function[i] = RC_Channel_aux::k_none;
        }
    }

    // Assume that no auxiliary function is used
    for (uint8_t i = 0; i < RC_Channel_aux::k_nr_aux_servo_functions; i++)
    {
        g_rc_function[i] = NULL;
    }

    // assign the RC channel to each function
    if( rc_a != NULL ) { g_rc_function[aux_servo_function[0]] = rc_a; }
    if( rc_b != NULL ) { g_rc_function[aux_servo_function[1]] = rc_b; }
    if( rc_c != NULL ) { g_rc_function[aux_servo_function[2]] = rc_c; }
    if( rc_d != NULL ) { g_rc_function[aux_servo_function[3]] = rc_d; }
    if( rc_e != NULL ) { g_rc_function[aux_servo_function[4]] = rc_e; }
    if( rc_f != NULL ) { g_rc_function[aux_servo_function[5]] = rc_f; }
    if( rc_g != NULL ) { g_rc_function[aux_servo_function[6]] = rc_g; }

    //set auxiliary ranges
    G_RC_AUX(k_flap)->set_range(0,100);
    G_RC_AUX(k_flap_auto)->set_range(0,100);
    G_RC_AUX(k_aileron)->set_angle(4500);
    G_RC_AUX(k_flaperon)->set_range(0,100);
/*
 *       G_RC_AUX(k_mount_pan)->set_range(
 *                               g_rc_function[RC_Channel_aux::k_mount_pan]->angle_min / 10,
 *                               g_rc_function[RC_Channel_aux::k_mount_pan]->angle_max / 10);
 *       G_RC_AUX(k_mount_tilt)->set_range(
 *                               g_rc_function[RC_Channel_aux::k_mount_tilt]->angle_min / 10,
 *                               g_rc_function[RC_Channel_aux::k_mount_tilt]->angle_max / 10);
 *       G_RC_AUX(k_mount_roll)->set_range(
 *                               g_rc_function[RC_Channel_aux::k_mount_roll]->angle_min / 10,
 *                               g_rc_function[RC_Channel_aux::k_mount_roll]->angle_max / 10);
 *       G_RC_AUX(k_mount_open)->set_range(0,100);
 *       G_RC_AUX(k_cam_trigger)->set_range(
 *                               g_rc_function[RC_Channel_aux::k_cam_trigger]->angle_min / 10,
 *                               g_rc_function[RC_Channel_aux::k_cam_trigger]->angle_max / 10);
 */
    G_RC_AUX(k_egg_drop)->set_range(0,100);
}

/// Should be called after the the servo functions have been initialized
void
enable_aux_servos()
{
    // cycle thru all functions except k_none and k_nr_aux_servo_functions
    for (uint8_t i = 1; i < RC_Channel_aux::k_nr_aux_servo_functions; i++)
    {
        if (g_rc_function[i]) g_rc_function[i]->enable_out();
    }
}

