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

#ifndef __BEV_KEY_H__
#define __BEV_KEY_H__

#include <AP_HAL_PX4.h>
#include <systemlib/perf_counter.h>
#include <drivers/drv_orb_dev.h>
#include <pthread.h>
#include <uORB/uORB.h>
#include <AP_Param.h>

//so we can use the ORB
ORB_DECLARE(bev_key_to_px4io_periodic);
ORB_DECLARE(bev_key_to_px4fmu_periodic);

class BEV_Key {
public:
    BEV_Key(AP_Int32&, AP_Int32&);
    void init(void);
    void update(void);
    uint8_t get_key_level() {return _to_px4fmu_periodic.key_level;}
    uint32_t get_key_pid() {return _to_px4fmu_periodic.key_PID;}

    //enumeration of different possible key levels
    enum {
        KEY_NONE = 0,
        KEY_SPORT = 1,
        KEY_PRO = 2,
        KEY_MAPPING = 3
    };

    //Information going from PX4FMU to PX4IO
    struct bev_key_to_px4io_periodic_struct {
        /* key value entered by user */
        int32_t key_value;
    };

    //Information going from PX4IO to PX4FMU
    struct bev_key_to_px4fmu_periodic_struct {
        /* key PID and key level (none, sport, pro, or mapping) */
        int32_t key_PID;
        uint8_t key_level;
    };

private:
    //methods
    void push_to_px4io_periodic();
    bool receive_from_px4io_periodic();
    //io structures
    bev_key_to_px4io_periodic_struct     _to_px4io_periodic;
    bev_key_to_px4fmu_periodic_struct    _to_px4fmu_periodic;

    //parameter references (params not owned by class to prevent needing to update param list
    AP_Int32& _key_pid;
    AP_Int32& _key_value;
    //so we can tell when it's changed
    int32_t _old_key_value;

    //sending to PX4IO
    orb_advert_t  _advert_bev_key_to_px4io_periodic; ///< info from PX4FMU to PX4IO
    int _t_bev_key_to_px4fmu_periodic;          ///< info from PX4IO to PX4FMU
    perf_counter_t _perf_bev_key_periodic;
    pthread_mutex_t _mutex_bev_key_periodic;
};

#endif // __BEV_KEY_H__
