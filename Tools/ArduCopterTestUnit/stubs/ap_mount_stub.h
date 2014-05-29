/*
 * ap_mount_stub.h
 *
 *  Created on: 29 mai 2014
 *      Author: valentin
 */

#ifndef AP_MOUNT_STUB_H_
#define AP_MOUNT_STUB_H_

class AP_Mount
{
public:
	void configure_msg(mavlink_message_t* msg) {}
	void control_msg(mavlink_message_t* msg) {}
    void status_msg(mavlink_message_t* msg) {}

};

#endif /* AP_MOUNT_STUB_H_ */
