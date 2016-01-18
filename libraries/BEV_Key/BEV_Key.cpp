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

#include "BEV_Key.h"

extern const AP_HAL::HAL& hal;

//normally I want the parameters owned by the class. However, this would require changing the param format number, which I don't want
//to do because I want the upgrade to be transparent to users.
BEV_Key::BEV_Key(AP_Int32& key_pid, AP_Int32& key_value) :
        _key_pid(key_pid),
        _key_value(key_value),
        _old_key_value(key_value.get())
{
    _t_bev_key_to_px4fmu_periodic = 0;
    _advert_bev_key_to_px4io_periodic = 0;

    //initialize the structures
    _to_px4io_periodic = {0};
    _to_px4fmu_periodic = {0,0};
}

void BEV_Key::init(void)
{
    //prevent multiple calls
    static bool first_call = true;
    if(first_call) {
        first_call = false;
    } else {
        return;
    }

    _perf_bev_key_periodic = perf_alloc(PC_ELAPSED, "BEV_KEY_PERIODIC");
    //subscrube to the to_px4fmu structure
    _t_bev_key_to_px4fmu_periodic = orb_subscribe(ORB_ID(bev_key_to_px4fmu_periodic));
    if (_t_bev_key_to_px4fmu_periodic == -1) {
        hal.scheduler->panic("Unable to subscribe to BEV KEY TO PX4FMU PERIODIC");
    }

    pthread_mutex_init(&_mutex_bev_key_periodic, NULL);

    //send the key_value so we receive a prompt response and can update the key_value
    push_to_px4io_periodic();
}

void BEV_Key::update(void)
{
    //decimate so calls are only at 50hz
    static uint32_t last_update_time = 0;
    if(hal.scheduler->millis() - last_update_time < 20) {
        return;
    }
    last_update_time = hal.scheduler->millis();

    if(receive_from_px4io_periodic()) {
        //update the key_pid. Write to eeProm so user can see
        _key_pid.set_and_save_ifchanged(get_key_pid());
    }

    //see if the key_value has been changed. If so, push to px4io
    if(_key_value.get() != _old_key_value) {
        _old_key_value = _key_value.get();
        push_to_px4io_periodic();
    }
}

void BEV_Key::push_to_px4io_periodic()
{
    _to_px4io_periodic.key_value = _key_value.get();

    if(_advert_bev_key_to_px4io_periodic == 0) {
        _advert_bev_key_to_px4io_periodic = orb_advertise(ORB_ID(bev_key_to_px4io_periodic), &_to_px4io_periodic);
    }

    orb_publish(ORB_ID(bev_key_to_px4io_periodic), _advert_bev_key_to_px4io_periodic, &_to_px4io_periodic);
}

//true if update
bool BEV_Key::receive_from_px4io_periodic()
{
    perf_begin(_perf_bev_key_periodic);
    bool updated = false;
    if (orb_check(_t_bev_key_to_px4fmu_periodic, &updated) == 0 && updated) {
        pthread_mutex_lock(&_mutex_bev_key_periodic);
        orb_copy(ORB_ID(bev_key_to_px4fmu_periodic), _t_bev_key_to_px4fmu_periodic, &_to_px4fmu_periodic);

        pthread_mutex_unlock(&_mutex_bev_key_periodic);
        perf_end(_perf_bev_key_periodic);
        return true;
    }
    perf_end(_perf_bev_key_periodic);
    return false;
}
