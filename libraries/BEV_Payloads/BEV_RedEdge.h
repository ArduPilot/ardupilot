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


#ifndef __BEV_REDEDGE_H__
#define __BEV_REDEDGE_H__

#include "BEV_Device.h"
#include <AP_GPS.h>
#include <AP_AHRS.h>
#include <AP_InertialNav.h>

#define ENABLED                 1
#define DISABLED                0

//debugging options on console
#define BEV_REDEDGE_DEBUGGING ENABLED
#define BEV_REDEDGE_HFDEBUGGING DISABLED

class BEV_RedEdge : public BEV_Device
{
public:
	BEV_RedEdge(BEV_PayloadCommunicator &p, int32_t payloadId);
	void set_ahrs_gps_inav(AP_AHRS const* ahrs, AP_GPS * gps, AP_InertialNav const* inav) {_ahrs = ahrs; _gps = gps; _inav = inav;}
    void update(); //should be called at 50 hz
    void process_message(const bev_payload_struct& s);
    void trigger_camera();
private:
    AP_AHRS const*          _ahrs;
    AP_GPS *          _gps;
    AP_InertialNav const*   _inav;

    void _push_position_message();
    void _push_attitude_message();

    enum {
        MSG_ID_NONE = 0,
        MSG_ID_TRIGGER_CAMERA = 1,
        MSG_ID_POSITION = 2,
        MSG_ID_ATTITUDE = 3,
    };
};

#endif //__BEV_REDEDGE_H__

