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

#ifndef __BEV_Gimbal_H__
#define __BEV_Gimbal_H__

#include <AP_GPS.h>
#include <AP_AHRS.h>
#include <RC_Channel.h>
#include <AP_InertialNav.h>

//for uORB compliance
#include <AP_HAL_PX4.h>
#include <systemlib/perf_counter.h>
#include <drivers/drv_orb_dev.h>
#include <pthread.h>
#include <uORB/uORB.h>

//so we can use the ORB
ORB_DECLARE(bev_gimbal_to_px4io_regular);
ORB_DECLARE(bev_gimbal_to_px4io_periodic);
ORB_DECLARE(bev_gimbal_to_px4io_infrequent);
ORB_DECLARE(bev_gimbal_to_px4fmu_regular);

#define ENABLED                 1
#define DISABLED                0

#define BEV_GIMBAL_DEBUGGING DISABLED

class BEV_Gimbal
{
public:
    //Constructor
    BEV_Gimbal(const AP_AHRS &ahrs, const AP_InertialNav &inav, RC_Channel &rc_pitch_out, RC_Channel & rc_yaw_out);

    //Initializer
    void                    init();

    // should be called periodically
    void                    update(bool radio_failsafe, int16_t target_pitch, int16_t target_roll);

    // centers the camera
    void                    center();

    // tell the camera where to point
    void                    point_here(int32_t lat, int32_t lng, int32_t alt);

    //param hook
    static const struct AP_Param::GroupInfo var_info[];


    //Information going from PX4FMU to PX4IO
    struct to_px4io_regular_struct {
        int32_t present_lat;
        int32_t present_lon;
        int32_t present_alt;
        int8_t gps_status;
        int32_t yaw_sensor;
        int32_t target_roll;
        int32_t target_pitch;
        bool radio_failsafe;
    };

    struct to_px4io_periodic_struct {
        int32_t des_lat;
        int32_t des_lon;
        int32_t des_alt;
    };

    struct to_px4io_infrequent_struct {
        float pan_p;
        float pit_p;
        int32_t tilt_angle_min;
        int32_t tilt_angle_max;
        int32_t pan_angle_min;
        int32_t pan_angle_max;
    };

    //Information going from PX4IO to PX4FMU
    struct to_px4fmu_regular_struct {
        int32_t tilt_out;
        int32_t pan_out;
    };

private:
    static void                     _output_to_channel(RC_Channel &rc_out, int16_t angle);

    void _push_to_px4io_regular(bool radio_failsafe, int16_t target_pitch, int16_t target_roll);
    void _push_to_px4io_periodic(int32_t lat, int32_t lon, int32_t alt); //zero all fields to center
    void _push_to_px4io_infrequent();

    bool _receive_from_px4io_regular();

    //io structures
    to_px4io_regular_struct     _to_px4io_regular;
    to_px4io_periodic_struct    _to_px4io_periodic;
    to_px4io_infrequent_struct  _to_px4io_infrequent;
    to_px4fmu_regular_struct    _to_px4fmu_regular;

    //members
    const AP_AHRS&                  _ahrs;
    const AP_InertialNav&           _inav;
    RC_Channel&                     _rc_pitch_out;
    RC_Channel&                     _rc_yaw_out;

    // EEPROM parameters
    //controller gains
    AP_Float                        _pit_p; //Pitch p gain
    AP_Float                        _pan_p; //Pan p gain
    //physical limits
    AP_Int32                        _tilt_angle_min; ///< min angle limit of actuated surface in 0.01 degree units
    AP_Int32                        _tilt_angle_max; ///< max angle limit of actuated surface in 0.01 degree units
    AP_Int32                        _pan_angle_min;  ///< min angle limit of actuated surface in 0.01 degree units
    AP_Int32                        _pan_angle_max;  ///< max angle limit of actuated surface in 0.01 degree units

    //sending to PX4IO
    orb_advert_t  _advert_bev_gimbal_to_px4io_regular;  //info from PX4FMU to PX4IO
    orb_advert_t  _advert_bev_gimbal_to_px4io_periodic;
    orb_advert_t  _advert_bev_gimbal_to_px4io_infrequent;
    int _t_bev_gimbal_to_px4fmu_regular;                //info from PX4IO to PX4FMU
    perf_counter_t _perf_bev_gimbal_regular;
    pthread_mutex_t _mutex_bev_gimbal_regular;

};
#endif //__BEV_GIMBAL_H
