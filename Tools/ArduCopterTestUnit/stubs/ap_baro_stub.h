/*
 * ap_baro_stub.h
 *
 *  Created on: 28 mai 2014
 *      Author: valentin
 */

#ifndef AP_BARO_STUB_H_
#define AP_BARO_STUB_H_

class AP_Baro
{
public:
	float           get_pressure() {return 1.0;}
	float           get_ground_pressure(void) {return 1.0;}
	float           get_temperature() {return 21;}
};//AP_Baro

#endif /* AP_BARO_STUB_H_ */
