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

#include <AP_HAL.h>
#include "BEV_Gimbal.h"

extern const AP_HAL::HAL& hal;

const AP_Param::GroupInfo BEV_Gimbal::var_info[] PROGMEM = {
        // @Param: ANGMIN_TIL
        // @DisplayName: Minimum tilt angle
        // @Description: Minimum physical tilt (pitch) angular position of mount.
        // @Units: centi-Degrees
        // @Range: -18000 17999
        // @Increment: 1
        // @User: Standard
        AP_GROUPINFO("ANGMIN_TIL", 1, BEV_Gimbal, _tilt_angle_min, -4500),

        // @Param: ANGMAX_TIL
        // @DisplayName: Maximum tilt angle
        // @Description: Maximum physical tilt (pitch) angular position of the mount
        // @Units: centi-Degrees
        // @Range: -18000 17999
        // @Increment: 1
        // @User: Standard
        AP_GROUPINFO("ANGMAX_TIL", 2, BEV_Gimbal, _tilt_angle_max, 4500),

        // @Param: ANGMIN_PAN
        // @DisplayName: Minimum pan angle
        // @Description: Minimum physical pan (yaw) angular position of mount.
        // @Units: centi-Degrees
        // @Range: -18000 17999
        // @Increment: 1
        // @User: Standard
        AP_GROUPINFO("ANGMIN_PAN",  3, BEV_Gimbal, _pan_angle_min,  -4500),

        // @Param: ANGMAX_PAN
        // @DisplayName: Maximum pan angle
        // @Description: Maximum physical pan (yaw) angular position of the mount
        // @Units: centi-Degrees
        // @Range: -18000 17999
        // @Increment: 1
        // @User: Standard
        AP_GROUPINFO("ANGMAX_PAN",  4, BEV_Gimbal, _pan_angle_max,  4500),

        // @Param: PIT_P
        // @DisplayName: pitch p gain
        // @Description: Turns desired angle into servo output
        // @Units: N/D
        // @Range: 0 100
        // @Increment: 1
        // @User: Standard
        AP_GROUPINFO("PIT_P", 5, BEV_Gimbal, _pit_p, 0.5),

        // @Param: PAN_P
        // @DisplayName: pan p gain
        // @Description: Turns desired angle into servo output
        // @Units: N/D
        // @Range: 0 100
        // @Increment: 1
        // @User: Standard
        AP_GROUPINFO("PAN_P", 6, BEV_Gimbal, _pan_p, 0.5),

    AP_GROUPEND
};

BEV_Gimbal::BEV_Gimbal(const AP_AHRS &ahrs, const AP_InertialNav &inav, RC_Channel &rc_pitch_out, RC_Channel &rc_yaw_out) :
    _ahrs(ahrs),
    _inav(inav),
    _rc_pitch_out(rc_pitch_out),
    _rc_yaw_out(rc_yaw_out)
{
    //initialize the structures
    _to_px4fmu_regular = {0,0};
    _to_px4io_regular = {0,0,0,0,0,0,0,0};
    _to_px4io_periodic = {0,0,0};
    _to_px4io_infrequent = {0.0f,0.0f,0,0,0,0};

    //broker initializations
    _t_bev_gimbal_to_px4fmu_regular = 0;
    _advert_bev_gimbal_to_px4io_regular = 0;
    _advert_bev_gimbal_to_px4io_periodic = 0;
    _advert_bev_gimbal_to_px4io_infrequent = 0;

    //param initializations
    AP_Param::setup_object_defaults(this, var_info);
}

void BEV_Gimbal::init()
{
    //prevent multiple calls
    static bool first_call = true;
    if(first_call) {
        first_call = false;
    } else {
        return;
    }
#if BEV_GIMBAL_DEBUGGING == ENABLED
    hal.console->println("BEV_GIMBAL: Debugging Enabled");
#endif //BEV_GIMBAL_DEBUGGING

    _rc_pitch_out.enable_out();
    _rc_yaw_out.enable_out();

    //uORB specifics
    _perf_bev_gimbal_regular = perf_alloc(PC_ELAPSED, "BEV_GIMBAL_r");

    //subscribe to the to_px4fmu structure
    _t_bev_gimbal_to_px4fmu_regular = orb_subscribe(ORB_ID(bev_gimbal_to_px4fmu_regular));
    if (_t_bev_gimbal_to_px4fmu_regular == -1) {
        hal.scheduler->panic("Unable to subscribe to BEV GIMBAL TO PX4FMU REGULAR");
    }

    pthread_mutex_init(&_mutex_bev_gimbal_regular, NULL);


    //center the gimbal
    _output_to_channel(_rc_pitch_out, 0);
    _output_to_channel(_rc_yaw_out, 0);
}

void BEV_Gimbal::update(bool radio_failsafe, int16_t target_pitch, int16_t target_roll)
{
    //decimate so calls are only at 50hz
    static uint32_t last_update_time = 0;
    static uint32_t last_infrequent_time = 0;
    if(hal.scheduler->millis() - last_update_time < 20) {
        return;
    }
    last_update_time = hal.scheduler->millis();

    //send present state information
    _push_to_px4io_regular(radio_failsafe, target_pitch, target_roll);

    if(_receive_from_px4io_regular()) {
        //write to rcout
        _output_to_channel(_rc_pitch_out, _to_px4fmu_regular.tilt_out);
        _output_to_channel(_rc_yaw_out, _to_px4fmu_regular.pan_out);
    }

#if BEV_GIMBAL_DEBUGGING == ENABLED
    static uint8_t temp = 0;
    if(!(temp+=4)) {
        hal.console->printf_P(PSTR("Tilt out: %d\tPan out: %d, orbID:%d \n"),_rc_pitch_out.radio_out, _rc_yaw_out.radio_out, _t_bev_gimbal_to_px4fmu_regular);
    }
#endif //BEV_GIMBAL_DEBUGGING ENABLED

    //send params every 5 sec
    if(hal.scheduler->millis() - last_infrequent_time > 5000) {
        _push_to_px4io_infrequent();
        last_infrequent_time = hal.scheduler->millis();
    }
}

void BEV_Gimbal::_push_to_px4io_regular(bool radio_failsafe, int16_t target_pitch, int16_t target_roll)
{
    _to_px4io_regular.present_lat = _inav.get_latitude();
    _to_px4io_regular.present_lon = _inav.get_longitude();
    _to_px4io_regular.present_alt = _inav.get_altitude();
    _to_px4io_regular.gps_status = _ahrs.get_gps().status();
    _to_px4io_regular.yaw_sensor = _ahrs.yaw_sensor;
    _to_px4io_regular.target_roll = target_pitch;
    _to_px4io_regular.target_pitch = target_roll;
    _to_px4io_regular.radio_failsafe = radio_failsafe;

    //send it across
    if(_advert_bev_gimbal_to_px4io_regular == 0) {
        _advert_bev_gimbal_to_px4io_regular = orb_advertise(ORB_ID(bev_gimbal_to_px4io_regular), &_to_px4io_regular);
    }

    orb_publish(ORB_ID(bev_gimbal_to_px4io_regular), _advert_bev_gimbal_to_px4io_regular, &_to_px4io_regular);
}

void BEV_Gimbal::_push_to_px4io_periodic(int32_t lat, int32_t lon, int32_t alt)
{
    _to_px4io_periodic.des_lat = lat;
    _to_px4io_periodic.des_lon = lon;
    _to_px4io_periodic.des_alt = alt;

    if(_advert_bev_gimbal_to_px4io_periodic == 0) {
        _advert_bev_gimbal_to_px4io_periodic = orb_advertise(ORB_ID(bev_gimbal_to_px4io_periodic), &_to_px4io_periodic);
    }

    orb_publish(ORB_ID(bev_gimbal_to_px4io_periodic), _advert_bev_gimbal_to_px4io_periodic, &_to_px4io_periodic);
}

void BEV_Gimbal::_push_to_px4io_infrequent()
{
    _to_px4io_infrequent.pan_p = _pit_p;
    _to_px4io_infrequent.pit_p =  _pan_p;
    _to_px4io_infrequent.tilt_angle_min = _tilt_angle_min;
    _to_px4io_infrequent.tilt_angle_max = _tilt_angle_max;
    _to_px4io_infrequent.pan_angle_min = _pan_angle_min;
    _to_px4io_infrequent.pan_angle_max = _pan_angle_max;

    if(_advert_bev_gimbal_to_px4io_infrequent == 0) {
        _advert_bev_gimbal_to_px4io_infrequent = orb_advertise(ORB_ID(bev_gimbal_to_px4io_infrequent), &_to_px4io_infrequent);
    }

    orb_publish(ORB_ID(bev_gimbal_to_px4io_infrequent), _advert_bev_gimbal_to_px4io_infrequent, &_to_px4io_infrequent);
}

bool BEV_Gimbal::_receive_from_px4io_regular()
{
    perf_begin(_perf_bev_gimbal_regular);
    bool updated = false;
    if (orb_check(_t_bev_gimbal_to_px4fmu_regular, &updated) == 0 && updated) {
        pthread_mutex_lock(&_mutex_bev_gimbal_regular);
        orb_copy(ORB_ID(bev_gimbal_to_px4fmu_regular), _t_bev_gimbal_to_px4fmu_regular, &_to_px4fmu_regular);

        pthread_mutex_unlock(&_mutex_bev_gimbal_regular);
        perf_end(_perf_bev_gimbal_regular);
        return true;
    }
    perf_end(_perf_bev_gimbal_regular);
    return false;
}

void BEV_Gimbal::center()
{
#if BEV_GIMBAL_DEBUGGING == ENABLED
    hal.console->println("BEV_GIMBAL::Center()");
#endif //BEV_GIMBAL_DEBUGGING
    //relay command to px4io
    _push_to_px4io_periodic(0, 0, 0);
}

void BEV_Gimbal::point_here(int32_t lat, int32_t lng, int32_t alt)
{
#if BEV_GIMBAL_DEBUGGING == ENABLED
    hal.console->printf_P(PSTR("BEV_GIMBAL::point here (%3.2f, %3.2f, %3.2f)\n"), lat*1e-7, lng*1e-7, alt*1e-2);
#endif //BEV_GIMBAL_DEBUGGING

    //relay command to px4io
    _push_to_px4io_periodic(lat, lng, alt);
}

void BEV_Gimbal::_output_to_channel(RC_Channel &rc_out, int16_t angle)
{
    //deal with reversing the servo
    if(rc_out.get_reverse()) {
        angle = -angle;
    }

    //scale the desired output to meet the min and max set by the parameters
    if(angle>0) {
        rc_out.radio_out = constrain_int16( (float(angle)/4500)*(rc_out.radio_max-rc_out.radio_trim)+rc_out.radio_trim, rc_out.radio_min, rc_out.radio_max);
    } else {
        rc_out.radio_out = constrain_int16( (float(angle)/4500)*(rc_out.radio_trim-rc_out.radio_min)+rc_out.radio_trim, rc_out.radio_min, rc_out.radio_max);
    }

    //actually write to the output pin
    rc_out.output();
}
