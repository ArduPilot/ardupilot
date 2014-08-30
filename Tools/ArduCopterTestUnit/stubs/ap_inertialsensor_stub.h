/*
 * ap_inertialsensor_stub.h
 *
 *  Created on: 28 mai 2014
 *      Author: valentin
 */

#ifndef AP_INERTIALSENSOR_STUB_H_
#define AP_INERTIALSENSOR_STUB_H_

class AP_InertialSensor_UserInteract {
};//AP_InertialSensor_UserInteract

class AP_InertialSensor_UserInteract_MAVLink: public AP_InertialSensor_UserInteract {
	mavlink_channel_t _chan;
public:
	AP_InertialSensor_UserInteract_MAVLink(mavlink_channel_t chan):_chan(chan) {}
};//AP_InertialSensor_UserInteract_MAVLink

class AP_InertialSensor
{
	Vector3f _gyro;
	Vector3f _accel;
	Vector3f _gyro_offsets;
	Vector3f _accel_offsets;
public:
	const Vector3f &get_gyro(void) const {return _gyro;}
	const Vector3f &get_gyro(uint8_t instance) const {return _gyro;}
	const Vector3f &get_accel(void) const {return _accel;}
	const Vector3f &get_accel(uint8_t instance) const {return _accel;}
	bool healthy () {return true;}
	uint8_t get_gyro_count(void) const { return 1; }
	uint8_t get_accel_count(void) const { return 1; }
    const Vector3f &get_gyro_offsets(void) const { return _gyro_offsets; }
    const Vector3f &get_accel_offsets(void) const { return _accel_offsets; }
    void init_accel() {}
    bool calibrate_accel(AP_InertialSensor_UserInteract *interact, float& trim_roll, float& trim_pitch) {return true;}
};//AP_InertialSensor


#endif /* AP_INERTIALSENSOR_STUB_H_ */
