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

#ifndef __BEV_SPDHGT_H__
#define __BEV_SPDHGT_H__

#include <AP_Param.h>
#include <AP_AHRS.h>
#include <AP_InertialSensor.h>
#include <AP_InertialNav.h>

//for uORB compliance
#include <AP_HAL_PX4.h>
#include <systemlib/perf_counter.h>
#include <drivers/drv_orb_dev.h>
#include <pthread.h>
#include <uORB/uORB.h>

/***********parameter defaults****************/
//min and max climb rante
#define BEV_SPDHGT_CLMB_MAX (300)
#define BEV_SPDHGT_CLMB_MIN (-300)
//throttle setting
#define BEV_SPDHGT_THR_MAX (75)
#define BEV_SPDHGT_THR_TRIM (50)
#define BEV_SPDHGT_THR_MIN (25)
//commandable acceleration limits
#define BEV_SPDHGT_ACCL_MAX (200)
//commandable pitch limits
#define BEV_SPDHGT_PTCH_MAX (2000)
#define BEV_SPDHGT_PTCH_MIN (-1500)
//alt error to desired vvel gain
#define BEV_SPDHGT_ALT_P (1)
//vvel error to pitch PID
#define BEV_SPDHGT_SPD_P (2.25)
#define BEV_SPDHGT_SPD_I (0.015)
#define BEV_SPDHGT_SPD_IMAX (1500)
#define BEV_SPDHGT_SPD_D (0.1)
//throttle feedforwards
#define BEV_SPDHGT_VVEL2PTC_FF (3)
#define BEV_SPDHGT_PTC2THR_FF (0.015)

/**********useful constants**********/
#define BEV_SPDHGT_NZ_FILT_CONST (0.0f)

//so we can use the ORB
ORB_DECLARE(bev_spdhgt_to_px4io_regular);
ORB_DECLARE(bev_spdhgt_to_px4io_periodic);
ORB_DECLARE(bev_spdhgt_to_px4fmu_regular);

/// @class      BEV_SPDHGT
class BEV_SpdHgt {
public:
    //constructor
    BEV_SpdHgt(const AP_AHRS& ahrs, const AP_InertialNav& inav, const AP_InertialSensor& ins);
    void init(); //setups up uORB coms

    //updater. SHould be called at 50hz
    void update(int32_t desired_alt);

    //called when initializing the flight mode needed spdhgt to ensure continuity
    void initialize();

    //accessors
    int32_t get_current_alt_setpoint() {return _to_px4fmu_regular.alt_setpoint;}
    int16_t get_desired_pitch() {return _to_px4fmu_regular.desired_pitch;}
    int16_t get_desired_throttle() {return _to_px4fmu_regular.desired_throttle;}
    int32_t get_desired_vertical_velocity_cms() {return _to_px4fmu_regular.desired_vvel;}
    int32_t get_max_alt_error_cm() {return _to_px4fmu_regular.max_alt_error;}

    //param hook
    static const struct AP_Param::GroupInfo var_info[];

    //Information going from PX4FMU to PX4IO
    struct to_px4io_regular_struct {
        /* info needed by PX4IO processor */
        float cos_roll;
        float sin_pitch;
        float altitude;
        float vvel;
        float accel_z;
        int32_t desired_alt;
    };

    struct to_px4io_periodic_struct{
        bool init_flag; //when controller needs to be re-initialized
    };

    //Information going from PX4IO to PX4FMU
    struct to_px4fmu_regular_struct {
        int32_t alt_setpoint;
        int16_t desired_pitch;
        int16_t desired_throttle;
        int32_t max_alt_error;
        int16_t desired_vvel;
    };

protected:
private:
    //methods
    void push_to_px4io_regular(int32_t desired_alt);
    void push_to_px4io_periodic(bool init_flag);
    bool receive_from_px4io_regular();

    //io structures
    to_px4io_regular_struct     _to_px4io_regular;
    to_px4io_periodic_struct    _to_px4io_periodic;
    to_px4fmu_regular_struct    _to_px4fmu_regular;

    //params
    AP_Int16 _climb_max;
    AP_Int16 _climb_min;
    AP_Int8 _throttle_max;
    AP_Int8 _throttle_trim;
    AP_Int8 _throttle_min;
    AP_Int16 _accel_max;
    AP_Int16 _pitch_min;
    AP_Int16 _pitch_max;
    AP_Float _alt_p;
    AP_Float _spd_p;
    AP_Float _spd_i;
    AP_Float _spd_imax;
    AP_Float _spd_d;
    AP_Int8 _vvel2ptc_ff;
    AP_Float _ptc2thr_ff;

    //sending to PX4IO
    orb_advert_t  _advert_bev_spdhgt_to_px4io_regular;  //info from PX4FMU to PX4IO
    orb_advert_t  _advert_bev_spdhgt_to_px4io_periodic;  //info from PX4FMU to PX4IO
    int _t_bev_spdhgt_to_px4fmu_regular;                //info from PX4IO to PX4FMU
    perf_counter_t _perf_bev_spdhgt_regular;
    pthread_mutex_t _mutex_bev_spdhgt_regular;

    //for easy access
    const AP_AHRS& _ahrs;
    const AP_InertialNav& _inav;
    const AP_InertialSensor   &_ins;
};

#endif  // BEV_SPDHGT
