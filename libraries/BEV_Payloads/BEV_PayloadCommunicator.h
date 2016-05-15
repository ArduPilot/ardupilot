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

#ifndef __BEV_PAYLOADCOMMUNICATOR_H__
#define __BEV_PAYLOADCOMMUNICATOR_H__

//for uORB compliance
#include <AP_HAL_PX4.h>
#include <systemlib/perf_counter.h>
#include <drivers/drv_orb_dev.h>
#include <pthread.h>
#include <uORB/uORB.h>

//for our use
#include <stdint.h>
#include "BEV_SQueue.h"

#define ENABLED                 1
#define DISABLED                0

//debugging options on console
#define BEV_PAYLOADCOMMUNICATOR_DEBUGGING DISABLED
#define BEV_PAYLOADCOMMUNICATOR_HFDEBUGGING DISABLED //high frequency debugging.

ORB_DECLARE(bev_payload_to_px4io);
ORB_DECLARE(bev_payload_to_px4fmu);

class BEV_PayloadCommunicator
{
	public:
		BEV_PayloadCommunicator();
		void init(); //initialize uORB comms
		void push_important_message(struct bev_payload_struct s);
		void push_unimportant_message(struct  bev_payload_struct s);
		void update(); //call at 50hz or so.
		
		bool has_new_message_from_px4io() {return _has_new_message;}
		bev_payload_struct get_new_message_from_px4io();
		
	private:
		void _push_important_to_px4io();
		void _push_unimportant_to_px4io();
		void _push_to_px4io(struct bev_payload_struct s);
		bool _receive_from_px4io();
		
		bool _has_new_message;
		
		struct bev_payload_struct _to_px4io;
		struct bev_payload_struct _to_px4fmu;
		
		BEV_SQueue _important_queue;
		BEV_SQueue _unimportant_queue;
		
		//for sending to px4io
		orb_advert_t _advert_bev_payloadcommunicator;
		
		//for receiving from px4fmu
		int _t_bev_payloadcommunicator;
		perf_counter_t _perf_bev_payloadcommunicator;
		pthread_mutex_t _mutex_bev_payloadcommunicator;
};
#endif // __BEV_PAYLOADCOMMUNICATOR_H__
