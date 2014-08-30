/*
 * ac_wpnav_stub.h
 *
 *  Created on: 29 mai 2014
 *      Author: valentin
 */

#ifndef AC_WPNAV_STUB_H_
#define AC_WPNAV_STUB_H_

class AC_WPNav {
public:
	float speed_in_cms;
	void set_desired_alt(float desired_alt) {}
	void set_speed_xy(float speed_cms) {speed_in_cms = speed_cms;}
};//AC_WPNav

#endif /* AC_WPNAV_STUB_H_ */
