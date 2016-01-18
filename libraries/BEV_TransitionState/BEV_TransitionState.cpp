/*
 *  Copyright (c) BirdsEyeView Aerobotics, LLC, 2016.
 *
 *  This program is free software: you can redistribute it and/or modify it
 *  under the terms of the GNU General Public License version 3 as published
 *  by the Free Software Foundation.
 *
 *  This program is distributed in the hope that it will be useful, but
 *  WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General
 *  Public License version 3 for more details.
 *
 *  You should have received a copy of the GNU General Public License version
 *  3 along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <stdlib.h>
#include <AP_HAL.h>
#include "BEV_TransitionState.h"

extern const AP_HAL::HAL& hal;

BEV_TransitionState::BEV_TransitionState(RC_Channel &rc_transition_switch, RC_Channel &rc_transition_out,
                                         transition_callback_fn_t to_copter_callback, transition_callback_fn_t to_plane_callback, transition_callback_fn_t to_copternav_callback) :
             _rc_transition_switch(rc_transition_switch),
             _rc_transition_out(rc_transition_out),
             _to_copter_callback(to_copter_callback),
             _to_plane_callback(to_plane_callback),
             _to_copternav_callback(to_copternav_callback)
{
    _t_bev_transition_to_px4fmu_regular = 0;
    _t_bev_transition_to_px4fmu_periodic = 0;
    _advert_bev_transition_to_px4io_regular = 0;
    _advert_bev_transition_to_px4io_periodic = 0;

    //initialize the structures
    _to_px4fmu_regular = {0,0,0,0,0,0};
    _to_px4fmu_periodic = {0};
    _to_px4io_regular = {0,0,0,0};
    _to_px4io_periodic = {0};
}

void BEV_TransitionState::Init()
{
    //prevent multiple calls
    static bool first_call = true;
    if(first_call) {
        first_call = false;
    } else {
        return;
    }

#if BEV_TRANSITION_DEBUGGING == ENABLED
    hal.console->println("BEV_TRANSITION: Debugging Enabled");
#endif //BEV_GEAR_DEBUGGING

    //setting up output channel
    _rc_transition_out.set_range(0,1000);
    _rc_transition_out.enable_out();

    //uORB specifics
    _perf_bev_transition_regular = perf_alloc(PC_ELAPSED, "BEV_TRANSITION_REGULAR");
    _perf_bev_transition_periodic = perf_alloc(PC_ELAPSED, "BEV_TRANSITION_PERIODIC");

    //subscribe to the to_px4fmu_regular structure
    _t_bev_transition_to_px4fmu_regular = orb_subscribe(ORB_ID(bev_transition_to_px4fmu_regular));
    if (_t_bev_transition_to_px4fmu_regular == -1) {
        hal.scheduler->panic("Unable to subscribe to BEV TRANSITION TO PX4FMU REGULAR");
    }

    //subscribe to the to_px4fmu_periodic structure
    _t_bev_transition_to_px4fmu_periodic = orb_subscribe(ORB_ID(bev_transition_to_px4fmu_periodic));
    if (_t_bev_transition_to_px4fmu_periodic == -1) {
        hal.scheduler->panic("Unable to subscribe to BEV TRANSITION TO PX4FMU PERIODIC");
    }

    pthread_mutex_init(&_mutex_bev_transition_regular, NULL);
    pthread_mutex_init(&_mutex_bev_transition_periodic, NULL);

    push_to_px4io_regular();
    push_to_px4io_periodic(CMD_TO_COPTER); //ensure transition starts out as copter
}

// update. should be called no slower than 50hz
void BEV_TransitionState::update(bool radio_failsafe, int16_t target_pitch, uint8_t gear_raised)
{
    //decimate so calls are only at 50hz
    static uint32_t last_update_time = 0;
    if(hal.scheduler->millis() - last_update_time < 20) {
        return;
    }
    last_update_time = hal.scheduler->millis();

    //set all the members of the struct
    _to_px4io_regular.transition_switch = _rc_transition_switch.radio_in;
    _to_px4io_regular.should_update = !radio_failsafe;
    _to_px4io_regular.gear_position = gear_raised;
    _to_px4io_regular.target_pitch = target_pitch;
    push_to_px4io_regular();

    if(receive_from_px4io_periodic()) {
        //do callbacks as needed
        switch(_to_px4fmu_periodic.callback_flag) {
        case CALLBACK_NONE:
            break;
        case CALLBACK_COPTER:
#if BEV_TRANSITIONSTATE_DEBUGGING == ENABLED
            hal.console->println("BEV__TransitionState: Calling copter callback");
#endif
            _to_copter_callback();
            break;
        case CALLBACK_PLANE:
#if BEV_TRANSITIONSTATE_DEBUGGING == ENABLED
            hal.console->println("BEV__TransitionState: Calling plane callback");
#endif
            _to_plane_callback();
            break;
        case CALLBACK_COPTERNAV:
#if BEV_TRANSITIONSTATE_DEBUGGING == ENABLED
            hal.console->println("BEV__TransitionState: Calling copternav callback");
#endif
            _to_copternav_callback();
            break;
        }
    }

    if(receive_from_px4io_regular()) {
        //write output to radio
        _rc_transition_out.servo_out = ((_to_px4fmu_regular.transition_servo_permil-500)*2);
        _rc_transition_out.radio_out = _rc_transition_out.angle_to_pwm()+ _rc_transition_out.radio_trim;
        _rc_transition_out.output();
    }

#if BEV_TRANSITIONSTATE_DEBUGGING == ENABLED
    static uint8_t tmp = 0;
    if(!(tmp+=4) && (get_transition_direction() || is_nav_suppressed())) {
        hal.console->printf_P(PSTR("Pitch Override: %d"), is_pitch_override());
        hal.console->printf_P(PSTR(" Ang: %d"), get_pitch_override());
        hal.console->printf_P(PSTR(" TAng: %2.1f"), get_transition_angle_deg());
        hal.console->printf_P(PSTR(" Cptr Active: %d"), is_copter_active());
        hal.console->printf_P(PSTR(" Cptr Pct: %d"), get_copter_percent());
        hal.console->printf_P(PSTR(" Full Cptr: %d"), is_full_copter());
        hal.console->printf_P(PSTR(" Pln Active: %d"), is_plane_active());
        hal.console->printf_P(PSTR(" Pln Pct: %d"), get_plane_percent());
        hal.console->printf_P(PSTR(" Full Pln: %d"), is_full_plane());
        hal.console->printf_P(PSTR(" Nav Sp: %d"), is_nav_suppressed());
        hal.console->printf_P(PSTR(" TDir: %d"), get_transition_direction());
        hal.console->printf_P(PSTR(" isTran: %d\n"), is_transitioning());
    }
#endif

}

void BEV_TransitionState::push_to_px4io_regular()
{
    if(_advert_bev_transition_to_px4io_regular == 0) {
        _advert_bev_transition_to_px4io_regular = orb_advertise(ORB_ID(bev_transition_to_px4io_regular), &_to_px4io_regular);
    }

    orb_publish(ORB_ID(bev_transition_to_px4io_regular), _advert_bev_transition_to_px4io_regular, &_to_px4io_regular);
}

void BEV_TransitionState::push_to_px4io_periodic(uint8_t cmd)
{
    _to_px4io_periodic.transition_cmd = cmd;

    if(_advert_bev_transition_to_px4io_periodic == 0) {
        _advert_bev_transition_to_px4io_periodic = orb_advertise(ORB_ID(bev_transition_to_px4io_periodic), &_to_px4io_periodic);
    }

    orb_publish(ORB_ID(bev_transition_to_px4io_periodic), _advert_bev_transition_to_px4io_periodic, &_to_px4io_periodic);
}

bool BEV_TransitionState::receive_from_px4io_regular()
{
    perf_begin(_perf_bev_transition_regular);
    bool updated = false;
    if (orb_check(_t_bev_transition_to_px4fmu_regular, &updated) == 0 && updated) {
        pthread_mutex_lock(&_mutex_bev_transition_regular);
        orb_copy(ORB_ID(bev_transition_to_px4fmu_regular), _t_bev_transition_to_px4fmu_regular, &_to_px4fmu_regular);

        pthread_mutex_unlock(&_mutex_bev_transition_regular);
        perf_end(_perf_bev_transition_regular);
        return true;
    }
    perf_end(_perf_bev_transition_regular);
    return false;
}

bool BEV_TransitionState::receive_from_px4io_periodic()
{
    perf_begin(_perf_bev_transition_periodic);
    bool updated = false;
    if (orb_check(_t_bev_transition_to_px4fmu_periodic, &updated) == 0 && updated) {
        pthread_mutex_lock(&_mutex_bev_transition_periodic);
        orb_copy(ORB_ID(bev_transition_to_px4fmu_periodic), _t_bev_transition_to_px4fmu_periodic, &_to_px4fmu_periodic);

        pthread_mutex_unlock(&_mutex_bev_transition_periodic);
        perf_end(_perf_bev_transition_periodic);
        return true;
    }
    perf_end(_perf_bev_transition_periodic);
    return false;
}

void BEV_TransitionState::to_copter()
{
#if BEV_TRANSITIONSTATE_DEBUGGING == ENABLED
	hal.console->println("BEV_TransitionState: to_copter()");
#endif
	push_to_px4io_periodic(CMD_TO_COPTER);
}

void BEV_TransitionState::to_plane()
{
#if BEV_TRANSITIONSTATE_DEBUGGING == ENABLED
	hal.console->println("BEV_TransitionState::to_plane()");
#endif
	push_to_px4io_periodic(CMD_TO_PLANE);
}

//this function is debounced. Calls closer than 1000 ms are ignored
void BEV_TransitionState::toggle()
{
#if BEV_TRANSITIONSTATE_DEBUGGING == ENABLED
    hal.console->println("BEV_TransitionState::toggle()");
#endif
    push_to_px4io_periodic(CMD_TOGGLE);
}
