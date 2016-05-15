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

#include "BEV_RedEdge.h"

extern const AP_HAL::HAL& hal;

BEV_RedEdge::BEV_RedEdge(BEV_PayloadCommunicator& p, int32_t payloadId) :
    BEV_Device(p, payloadId),
    _ahrs(nullptr),
    _gps(nullptr),
    _inav(nullptr)
{
    ;
}

void BEV_RedEdge::update()
{
    //decimate so calls are only at 5hz
    static uint32_t last_update_time = 0;
    //BEV temp
    if(hal.scheduler->millis() - last_update_time < 200*10) {
        return;
    }
    last_update_time = hal.scheduler->millis();

    //push updated attitude to px4io
    _push_position_message();
    _push_attitude_message();

#if BEV_REDEDGE_HFDEBUGGING == ENABLED
    hal.console->println("BEV_Gimbal::update() - pushing position and attitude messages");
#endif //BEV_GI_DEBUGGING ENABLED
}

void BEV_RedEdge::process_message(const bev_payload_struct& s)
{
    ;
}

void BEV_RedEdge::_push_position_message()
{
    if(!_attached) {
        return;
    }

    if((_gps == nullptr) || (_inav == nullptr)) {
        return;
    }

    bev_payload_struct s;
    s = BEV_PAYLOAD_STRUCT_DEFAULT;
    s.payload_id = _payloadId;
    s.message_id = MSG_ID_POSITION;
    s.d1 = _inav->get_latitude();
    s.d2 = _inav->get_longitude();
    s.d3 = _gps->location().alt;
    s.d4 = (int32_t)_inav->get_altitude();

    _payloadCommunicator.push_unimportant_message(s);
}

void BEV_RedEdge::_push_attitude_message()
{
    if(!_attached) {
        return;
    }

    if((_ahrs == nullptr) || (_gps == nullptr)) {
        return;
    }

    uint64_t time = _gps->time_epoch_usec();
    bev_payload_struct s;
    s = BEV_PAYLOAD_STRUCT_DEFAULT;
    s.payload_id = _payloadId;
    s.message_id = MSG_ID_ATTITUDE;
    s.d1 = (time>>32) | (0xFFFFFFFF);
    s.d2 = (time | 0xFFFFFFFF);
    //s.d3 = (uint16_t(wrap_360_cd(_ahrs->roll_sensor)) << 16) | (uint16_t(wrap_360_cd(_ahrs->pitch_sensor)) | 0x0000FFFF);
    s.d3 = 1<<16 | 2;
    s.d4 = (uint16_t(wrap_360_cd(_ahrs->yaw_sensor)) | 0x0000FFFF);
    //s.d4 = 4 | 0x0000FFFF;

    _payloadCommunicator.push_unimportant_message(s);
}

void BEV_RedEdge::trigger_camera()
{
#if BEV_CAMERATRIGGER_DEBUGGING == ENABLED
    hal.console->println("BEV_RedEdge:: Triggering Camera\n");
#endif

    bev_payload_struct s;
    s = BEV_PAYLOAD_STRUCT_DEFAULT;
    s.payload_id = _payloadId;
    s.message_id = MSG_ID_TRIGGER_CAMERA;

    _payloadCommunicator.push_important_message(s);
}
