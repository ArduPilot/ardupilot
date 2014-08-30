/*
 * ap_servorelayevents_stub.h
 *
 *  Created on: 29 mai 2014
 *      Author: valentin
 */

#ifndef AP_SERVORELAYEVENTS_STUB_H_
#define AP_SERVORELAYEVENTS_STUB_H_

class AP_ServoRelayEvents {
public:
	bool do_set_servo(uint8_t channel, uint16_t pwm) {return true;}
	bool do_set_relay(uint8_t relay_num, uint8_t state) {return true;}
	bool do_repeat_servo(uint8_t channel, uint16_t servo_value, int16_t repeat, uint16_t delay_time_ms) {return true;}
	bool do_repeat_relay(uint8_t relay_num, int16_t count, uint32_t period_ms) {return true;}
};//AP_ServoRelayEvents

#endif /* AP_SERVORELAYEVENTS_STUB_H_ */
