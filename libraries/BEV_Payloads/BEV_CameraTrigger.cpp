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

#include "BEV_CameraTrigger.h"

extern const AP_HAL::HAL& hal;

BEV_CameraTrigger::BEV_CameraTrigger(BEV_PayloadCommunicator& p, int32_t payloadId) :
    BEV_Device(p, payloadId)
{
    ;
}

void BEV_CameraTrigger::update()
{
    ;
}

void BEV_CameraTrigger::process_message(const bev_payload_struct& s)
{
    ;
}

void BEV_CameraTrigger::trigger_camera()
{
#if BEV_CAMERATRIGGER_DEBUGGING == ENABLED
    hal.console->println("BEV_CameraTrigger:: Triggering Camera\n");
#endif
    bev_payload_struct s;
    s = BEV_PAYLOAD_STRUCT_DEFAULT;
    s.payload_id = _payloadId;s.message_id = MSG_ID_TRIGGER_CAMERA;

    _payloadCommunicator.push_important_message(s);
}
