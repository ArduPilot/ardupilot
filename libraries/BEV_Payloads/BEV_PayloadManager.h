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

#ifndef __BEV_PAYLOADMANAGER_H__
#define __BEV_PAYLOADMANAGER_H__

//for uORB compliance
#include <AP_HAL_PX4.h>

//for our use
#include <stdint.h>
#include "BEV_PayloadCommunicator.h"
#include "BEV_CameraTrigger.h"
#include "BEV_Gimbal.h"

#define ENABLED                 1
#define DISABLED                0

//debugging options on console
#define BEV_PAYLOADMANAGER_DEBUGGING DISABLED

class BEV_PayloadManager
{
	public:
		BEV_PayloadManager();
		void init();
		void update();
		void find_payloads();
		void report_payloads(int32_t bits);
		void process_message(bev_payload_struct s);
		void process_my_message(bev_payload_struct s);

		void send_bwtest_message();

		//public members
        BEV_CameraTrigger cameraTrigger;
        BEV_Gimbal gimbal;
	private:
		BEV_PayloadCommunicator _payloadCommunicator;

	    enum {
	        MSG_ID_NONE = 0,
	        MSG_ID_PAYLOADS_ATTACHED = 1
	    };
	    enum {
	        PAYLOAD_ID_NONE = 0,
	        PAYLOAD_ID_MANAGER = 1,
	        PAYLOAD_ID_CAMERATRIGGER = 2,
	        PAYLOAD_ID_GIMBAL = 3
	    };
	    enum {
	        DEVICE_BITMASK_NONE = 0,
	        DEVICE_BITMASK_CAMERATRIGGGER = 1<<0,
	        DEVICE_BITMASK_GIMBAL = 1<<1
	    };
};
#endif // __BEV_PAYLOADMANAGER_H__
