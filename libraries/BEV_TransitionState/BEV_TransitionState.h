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

#ifndef __BEV_TRANSITIONSTATE_H__
#define __BEV_TRANSITIONSTATE_H__

#include <RC_Channel.h>     // RC Channel

//for uORB compliance
#include <AP_HAL_PX4.h>
#include <systemlib/perf_counter.h>
#include <drivers/drv_orb_dev.h>
#include <pthread.h>
#include <uORB/uORB.h>

#define ENABLED                 1
#define DISABLED                0

//debugging options on a uart
#define BEV_TRANSITIONSTATE_DEBUGGING DISABLED

//so we can use the ORB
ORB_DECLARE(bev_transition_to_px4io_regular);
ORB_DECLARE(bev_transition_to_px4io_periodic);
ORB_DECLARE(bev_transition_to_px4fmu_regular);
ORB_DECLARE(bev_transition_to_px4fmu_periodic);

/// @class      BEV_TransitionState
class BEV_TransitionState {
public:
    //callback function pointers
    typedef void (*transition_callback_fn_t)(void);

    /// Constructor
    BEV_TransitionState(RC_Channel &rc_transition_switch, RC_Channel &rc_transition_out,
                        transition_callback_fn_t to_copter_callback, transition_callback_fn_t to_plane_callback, transition_callback_fn_t to_copternav_callback);
    void Init();

    //should be called at the main loop rate
    //target pitch is needed by transition state so that transition axle can be rotated forwards as nose is commanded down. If the
    //nose lowers too much the wing generates negatgive lift which can overcome the thrust capability of the vehicle. Rather than lowering
    //the nose to gain airspeed, the transition axle will be rotated forwards to generate motion.
    void update(bool radio_failsafe, int16_t target_pitch, uint8_t gear_raised);

    //get-ers for transition related items
    bool is_pitch_override() {return _to_px4fmu_regular.boolean_bitmask & BITMASK_PITCH_OVERRIDE;}
    int16_t get_pitch_override() {return _to_px4fmu_regular.pitch_override_cd;}

    float get_transition_angle_deg() {return _to_px4fmu_regular.transition_servo_permil * 0.08f;} //maps [0 1000] to [0 80]
    int8_t get_transition_direction() {return _to_px4fmu_regular.transition_direction;}
    bool is_copter_active() {return _to_px4fmu_regular.boolean_bitmask & BITMASK_COPTER_ACTIVE;}
    bool is_plane_active() {return _to_px4fmu_regular.boolean_bitmask & BITMASK_PLANE_ACTIVE;}
    bool is_full_copter() {return _to_px4fmu_regular.boolean_bitmask & BITMASK_FULL_COPTER;} //true if 100% copter
    bool is_full_plane() {return _to_px4fmu_regular.boolean_bitmask & BITMASK_FULL_PLANE;} //true if 100% plane
    bool is_nav_suppressed() {return _to_px4fmu_regular.boolean_bitmask & BITMASK_NAV_SUPPRESSED;}
    uint8_t get_copter_percent()  {return _to_px4fmu_regular.copter_percent;}
    uint8_t get_plane_percent() {return _to_px4fmu_regular.plane_percent;}
    bool is_transitioning() {return _to_px4fmu_regular.transition_direction != DIRECTION_NONE;} // true if btw copter and plane

    //allows external command of transition (ground station, mission command could override rc switch
    void to_copter(); //called when desired transition to copter
    void to_plane(); //called when desired transition to plane
    void toggle(); //plane <-> copter

    //param hook
    static const struct AP_Param::GroupInfo var_info[];

    //Information going from PX4FMU to PX4IO
    struct to_px4io_regular_struct {
        /* info needed by PX4IO processor */
        uint16_t transition_switch;
        bool should_update;
        uint16_t gear_position;
        int16_t target_pitch;
    };

    struct to_px4io_periodic_struct {
        uint8_t transition_cmd;
    };

    //Information going from PX4IO to PX4FMU
    struct to_px4fmu_regular_struct {
        uint8_t boolean_bitmask; //see bitmask enum
        int16_t pitch_override_cd; //for when transition is overriding desired attitude
        uint8_t plane_percent;
        uint8_t copter_percent;
        int16_t transition_servo_permil;
        int8_t transition_direction;
    };

    struct to_px4fmu_periodic_struct {
        uint8_t callback_flag;
    };

    //enumeration of commands that can be sent to transition
    enum {
        CMD_NONE = 0,
        CMD_TO_COPTER = 1,
        CMD_TO_PLANE = 2,
        CMD_TOGGLE = 3
    };

    enum {
        CALLBACK_NONE = 0,
        CALLBACK_COPTER = 1,
        CALLBACK_PLANE = 2,
        CALLBACK_COPTERNAV = 3,
    };

    enum {
        BITMASK_PITCH_OVERRIDE   = 0b00000001,
        BITMASK_COPTER_ACTIVE    = 0b00000010,
        BITMASK_PLANE_ACTIVE     = 0b00000100,
        BITMASK_FULL_COPTER      = 0b00001000,
        BITMASK_FULL_PLANE       = 0b00010000,
        BITMASK_NAV_SUPPRESSED   = 0b00100000
    };

    //a handy enum
    enum  {
        DIRECTION_TO_COPTER = -1,
        DIRECTION_NONE = 0,
        DIRECTION_TO_PLANE = 1
    };

protected:
    RC_Channel& _rc_transition_out;
    RC_Channel& _rc_transition_switch;

private:
    //methods
    void push_to_px4io_regular();
    void push_to_px4io_periodic(uint8_t cmd);
    bool receive_from_px4io_regular();
    bool receive_from_px4io_periodic();

    //callbacks
    transition_callback_fn_t _to_copter_callback; //called when copter CLAW is 100% active after spending time as a plane
    transition_callback_fn_t _to_plane_callback; //called when plane CLAW is 100% active after spending time as a copter
    transition_callback_fn_t _to_copternav_callback; //called at end of nav_suppresstion

    //io structures
    to_px4io_regular_struct      _to_px4io_regular;
    to_px4io_periodic_struct     _to_px4io_periodic;
    to_px4fmu_regular_struct     _to_px4fmu_regular;
    to_px4fmu_periodic_struct    _to_px4fmu_periodic;

    //sending to PX4IO
    orb_advert_t  _advert_bev_transition_to_px4io_periodic; // info from PX4FMU to PX4IO
    orb_advert_t  _advert_bev_transition_to_px4io_regular; // info from PX4FMU to PX4IO
    int _t_bev_transition_to_px4fmu_periodic;               // info from PX4IO to PX4FMU
    int _t_bev_transition_to_px4fmu_regular;               // info from PX4IO to PX4FMU
    perf_counter_t _perf_bev_transition_regular;
    perf_counter_t _perf_bev_transition_periodic;
    pthread_mutex_t _mutex_bev_transition_regular;
    pthread_mutex_t _mutex_bev_transition_periodic;

};

#endif  // BEV_TRANSITIONSTATE
