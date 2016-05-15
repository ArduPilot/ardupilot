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


#ifndef __BEV_CAMERATRIGGER_H__
#define __BEV_CAMERATRIGGER_H__

#include "BEV_Device.h"

#define ENABLED                 1
#define DISABLED                0

//debugging options on console
#define BEV_CAMERATRIGGER_DEBUGGING ENABLED

class BEV_CameraTrigger : public BEV_Device
{
public:
	BEV_CameraTrigger(BEV_PayloadCommunicator &p, int32_t payloadId);
    void update(); //should be called at 50 hz
    void process_message(const bev_payload_struct& s);
    void trigger_camera();
private:

    enum {
        MSG_ID_NONE = 0,
        MSG_ID_TRIGGER_CAMERA = 1
    };
};

#endif //__BEV_CAMERATRIGGER_H__

