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
#include "BEV_SpdHgt.h"

extern const AP_HAL::HAL& hal;

//BEV the params are unused but need to be left in. Otherwise the param number would have to be reved, creating headaches
//for customers when they update the software. At some point I'll need to clean up the param list.
const AP_Param::GroupInfo BEV_SpdHgt::var_info[] PROGMEM = {

    AP_GROUPINFO("CLMB_MAX", 0,      BEV_SpdHgt,  _climb_max, BEV_SPDHGT_CLMB_MAX),
    AP_GROUPINFO("CLMB_MIN", 1,      BEV_SpdHgt,  _climb_min, BEV_SPDHGT_CLMB_MIN),
    AP_GROUPINFO("THR_MAX", 2,      BEV_SpdHgt,  _throttle_max, BEV_SPDHGT_THR_MAX),
    AP_GROUPINFO("THR_TRIM", 3,      BEV_SpdHgt,  _throttle_trim, BEV_SPDHGT_THR_TRIM),
    AP_GROUPINFO("THR_MIN", 4,      BEV_SpdHgt,  _throttle_min, BEV_SPDHGT_THR_MIN),
    AP_GROUPINFO("ACCL_MAX", 5,      BEV_SpdHgt,  _accel_max, BEV_SPDHGT_ACCL_MAX),
    AP_GROUPINFO("PTCH_MIN", 6,      BEV_SpdHgt,  _pitch_min, BEV_SPDHGT_PTCH_MIN),
    AP_GROUPINFO("PTCH_MAX", 7,      BEV_SpdHgt,  _pitch_max, BEV_SPDHGT_PTCH_MAX),
    AP_GROUPINFO("ALT_P", 8,      BEV_SpdHgt,  _alt_p, BEV_SPDHGT_ALT_P),
    AP_GROUPINFO("SPD_P", 9,      BEV_SpdHgt,  _spd_p, BEV_SPDHGT_SPD_P),
    AP_GROUPINFO("SPD_I", 10,      BEV_SpdHgt,  _spd_i, BEV_SPDHGT_SPD_I),
    AP_GROUPINFO("SPD_IMAX", 11,      BEV_SpdHgt,  _spd_imax, BEV_SPDHGT_SPD_IMAX),
    AP_GROUPINFO("SPD_D", 12,      BEV_SpdHgt,  _spd_d, BEV_SPDHGT_SPD_D),
    AP_GROUPINFO("VVEL2PTC", 13,      BEV_SpdHgt,  _vvel2ptc_ff, BEV_SPDHGT_VVEL2PTC_FF),
    AP_GROUPINFO("PTC2THR", 14,      BEV_SpdHgt,  _ptc2thr_ff, BEV_SPDHGT_PTC2THR_FF),

    AP_GROUPEND
};

//
// public methods
//

BEV_SpdHgt::BEV_SpdHgt(const AP_AHRS& ahrs, const AP_InertialNav& inav, const AP_InertialSensor & ins) :
        _ahrs(ahrs),
        _inav(inav),
        _ins(ins)
{
	AP_Param::setup_object_defaults(this, var_info);


    _t_bev_spdhgt_to_px4fmu_regular = 0;
    _advert_bev_spdhgt_to_px4io_regular = 0;
    _advert_bev_spdhgt_to_px4io_periodic = 0;

    //initialize the structures
    _to_px4io_regular = {0.f,0.f,0.f,0.f,0.f};
    _to_px4io_periodic = {0};
    _to_px4fmu_regular = {0,0,0,0};
}

void BEV_SpdHgt::init()
{
    //prevent multiple calls
    static bool first_call = true;
    if(first_call) {
        first_call = false;
    } else {
        return;
    }

    //uORB specifics
    _perf_bev_spdhgt_regular = perf_alloc(PC_ELAPSED, "BEV_SPDHGT_REGULAR");

    //subscribe to the to_px4fmu structure
    _t_bev_spdhgt_to_px4fmu_regular = orb_subscribe(ORB_ID(bev_spdhgt_to_px4fmu_regular));
    if (_t_bev_spdhgt_to_px4fmu_regular == -1) {
        hal.scheduler->panic("Unable to subscribe to BEV SPDHGT TO PX4FMU REGULAR");
    }

    pthread_mutex_init(&_mutex_bev_spdhgt_regular, NULL);
}

void BEV_SpdHgt::update(int32_t desired_alt)
{
    //decimate so calls are only at 50hz
    static uint32_t last_update_time = 0;
    if(hal.scheduler->millis() - last_update_time < 20) {
        return;
    }
    last_update_time = hal.scheduler->millis();

    push_to_px4io_regular(desired_alt);

    if(receive_from_px4io_regular()) {
        //nothing to do
    }
}

void BEV_SpdHgt::initialize()
{
    push_to_px4io_periodic(true);
}

void BEV_SpdHgt::push_to_px4io_regular(int32_t desired_alt)
{
    _to_px4io_regular.cos_roll = _ahrs.cos_roll();
    _to_px4io_regular.sin_pitch = _ahrs.sin_pitch();
    _to_px4io_regular.altitude = _inav.get_altitude();
    _to_px4io_regular.vvel = _inav.get_velocity_z();
    Vector3f accel = _ins.get_accel();
    _to_px4io_regular.accel_z = accel.z;
    _to_px4io_regular.desired_alt = desired_alt;

    if(_advert_bev_spdhgt_to_px4io_regular == 0) {
        _advert_bev_spdhgt_to_px4io_regular = orb_advertise(ORB_ID(bev_spdhgt_to_px4io_regular), &_to_px4io_regular);
    }

    orb_publish(ORB_ID(bev_spdhgt_to_px4io_regular), _advert_bev_spdhgt_to_px4io_regular, &_to_px4io_regular);
}

void BEV_SpdHgt::push_to_px4io_periodic(bool init_flag)
{
    _to_px4io_periodic.init_flag = init_flag;

    if(_advert_bev_spdhgt_to_px4io_periodic == 0) {
        _advert_bev_spdhgt_to_px4io_periodic = orb_advertise(ORB_ID(bev_spdhgt_to_px4io_periodic), &_to_px4io_periodic);
    }

    orb_publish(ORB_ID(bev_spdhgt_to_px4io_periodic), _advert_bev_spdhgt_to_px4io_periodic, &_to_px4io_periodic);
}

bool BEV_SpdHgt::receive_from_px4io_regular()
{
    perf_begin(_perf_bev_spdhgt_regular);
    bool updated = false;
    if (orb_check(_t_bev_spdhgt_to_px4fmu_regular, &updated) == 0 && updated) {
        pthread_mutex_lock(&_mutex_bev_spdhgt_regular);
        orb_copy(ORB_ID(bev_spdhgt_to_px4fmu_regular), _t_bev_spdhgt_to_px4fmu_regular, &_to_px4fmu_regular);

        pthread_mutex_unlock(&_mutex_bev_spdhgt_regular);
        perf_end(_perf_bev_spdhgt_regular);
        return true;
    }
    perf_end(_perf_bev_spdhgt_regular);
    return false;
}
