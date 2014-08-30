/*
 * ap_hal_stub.h
 *
 *  Created on: 28 mai 2014
 *      Author: valentin
 */

#ifndef AP_HAL_STUB_H_
#define AP_HAL_STUB_H_

class AP_HAL
{
public:
    AP_HAL() {
    	this->analogin = new AnalogIn;
    	this->i2c = new I2CDriver;
    	this->scheduler = new HAL_Scheduler;
    	this->rcin = new RCInput;
    	this->rcout = new RCOutput;
    	this->gpio = new GPIO;
    	this->util = new Util;
    }
	AnalogIn* analogin;
	I2CDriver* i2c;
	HAL_Scheduler* scheduler;
	RCInput* rcin;
	RCOutput* rcout;
	GPIO* gpio;
    Util*       util;
};//AP_HAL

#endif /* AP_HAL_STUB_H_ */
