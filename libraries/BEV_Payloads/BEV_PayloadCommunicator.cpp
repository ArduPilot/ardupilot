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
 
 #include "BEV_PayloadCommunicator.h"
 
extern const AP_HAL::HAL& hal;
 
 BEV_PayloadCommunicator::BEV_PayloadCommunicator() :
	_important_queue(20),
	_unimportant_queue(20)
{
	_has_new_message = false;	
	
	//initalize uORB com objects
	_t_bev_payloadcommunicator = 0;
	_advert_bev_payloadcommunicator = 0;
	
	//initialize the structures
	_to_px4io  = BEV_PAYLOAD_STRUCT_DEFAULT;	
	_to_px4fmu = BEV_PAYLOAD_STRUCT_DEFAULT;
}

void BEV_PayloadCommunicator::init()
{
	//prevent multiple calls
	static bool first_call = true;
	if(first_call) {
		first_call = false;
	} else {
		return;
	}
	
#if BEV_PAYLOADCOMMUNICATOR_DEBUGGING == ENABLED
	hal.console->println("BEV_PAYLOADCOMMUNICATOR: Debugging Enabled");
#endif
	
	_perf_bev_payloadcommunicator = perf_alloc(PC_ELAPSED, "BEV_PAYLOADCOMMUNICATOR");
	
	//subscribe to the to_px4fmu structure
	_t_bev_payloadcommunicator = orb_subscribe(ORB_ID(bev_payload_to_px4fmu));
	if(_t_bev_payloadcommunicator == -1) {
		hal.scheduler->panic("Unable to subscrive to BEV PAYLOADCOMMUNICATOR TO PX4FMU");
	}
	
	pthread_mutex_init(&_mutex_bev_payloadcommunicator, NULL);
}

//read message as fast as possible. Send new ones at 20hz
void BEV_PayloadCommunicator::update()
{
	//don't read new message if present hasn't been processed yet
	if(!_has_new_message && _receive_from_px4io()) {
#if BEV_PAYLOADCOMMUNICATOR_DEBUGGING == ENABLED
	hal.console->println("BEV_PAYLOADCOMMUNICATOR::Update - new message available");
#endif
		_has_new_message = true;
	}
	
	//decimate so sending message is only 25hz.
	//This will prevent messages from being sent faster than the px4io can receive them
	static uint32_t last_update_time = 0;
	if(hal.scheduler->millis() - last_update_time < 40) {
		return;
	}
	last_update_time = hal.scheduler->millis();
	
	if(!_important_queue.isEmpty()) {
		_push_important_to_px4io();
#if BEV_PAYLOADCOMMUNICATOR_HFDEBUGGING == ENABLED
		hal.console->printf_P(PSTR("BEV_PAYLOAD_COMMUNICATOR::Update - sending important message\n"));
#endif
	} else if(!_unimportant_queue.isEmpty()) {
#if BEV_PAYLOADCOMMUNICATOR_HFDEBUGGING == ENABLED
		hal.console->printf_P(PSTR("BEV_PAYLOAD_COMMUNICATOR::Update - sending unimportant message\n"));
#endif
		_push_unimportant_to_px4io();
	}
	
}

//return the most recent message
bev_payload_struct BEV_PayloadCommunicator::get_new_message_from_px4io()
{
	//flag the message as read.
	_has_new_message = false;
	return _to_px4fmu;
}

//see if there's a new message
bool BEV_PayloadCommunicator::_receive_from_px4io()
{
    perf_begin(_perf_bev_payloadcommunicator);
    bool updated = false;
    if (orb_check(_t_bev_payloadcommunicator, &updated) == 0 && updated) {
        pthread_mutex_lock(&_mutex_bev_payloadcommunicator);
        orb_copy(ORB_ID(bev_payload_to_px4fmu), _t_bev_payloadcommunicator, &_to_px4fmu);

        pthread_mutex_unlock(&_mutex_bev_payloadcommunicator);
        perf_end(_perf_bev_payloadcommunicator);
        return true;
    }
    perf_end(_perf_bev_payloadcommunicator);
    return false;
}

//add a message to the important queue
void BEV_PayloadCommunicator::push_important_message(struct bev_payload_struct s)
{
	if(_important_queue.isFull()) {
#if BEV_PAYLOADCOMMUNICATOR_DEBUGGING == ENABLED
		hal.console->println("BEV_PAYLOAD_COMMUNICATOR::PUSH_IMPORTANT_MESSAGE - Queue is full!!");
#endif
	} else {
		_important_queue.push(s);
	}
}

//add a message to the unimportant queue
void BEV_PayloadCommunicator::push_unimportant_message(struct  bev_payload_struct s)
{
	if(_unimportant_queue.isFull()) {
#if BEV_PAYLOADCOMMUNICATOR_DEBUGGING == ENABLED
		hal.console->println("BEV_PAYLOAD_COMMUNICATOR::PUSH_UNIMPORTANT_MESSAGE - Queue is full!!");
#endif
	} else {
		_unimportant_queue.push(s);
	}
}

void BEV_PayloadCommunicator::_push_important_to_px4io()
{
	if(!_important_queue.isEmpty()) {
		_push_to_px4io(_important_queue.pop());
	}
}
void BEV_PayloadCommunicator::_push_unimportant_to_px4io()
{
	if(!_unimportant_queue.isEmpty()) {
		_push_to_px4io(_unimportant_queue.pop());
	}
}

void BEV_PayloadCommunicator::_push_to_px4io(struct bev_payload_struct s)
{
	_to_px4io = s;
	
	if(_advert_bev_payloadcommunicator == 0) {
		_advert_bev_payloadcommunicator = orb_advertise(ORB_ID(bev_payload_to_px4io), &_to_px4io);
	}
	
	orb_publish(ORB_ID(bev_payload_to_px4io), _advert_bev_payloadcommunicator, &_to_px4io);
}
