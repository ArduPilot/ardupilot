/*
 * ap_camera_stub.h
 *
 *  Created on: 29 mai 2014
 *      Author: valentin
 */

#ifndef AP_CAMERA_STUB_H_
#define AP_CAMERA_STUB_H_

class AP_Camera {
public:
	void configure_msg(mavlink_message_t* msg) {}
	void control_msg(mavlink_message_t* msg) {}
};

#endif /* AP_CAMERA_STUB_H_ */
