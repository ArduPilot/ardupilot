/*
 * AP_Navigator.cpp
 *
 *  Created on: Apr 30, 2011
 *      Author: jgoppert
 */

#include "AP_Navigator.h"
#include "../FastSerial/FastSerial.h"
#include "AP_HardwareAbstractionLayer.h"
#include "../AP_DCM/AP_DCM.h"
#include "../AP_Math/AP_Math.h"
#include "../AP_Compass/AP_Compass.h"
#include "AP_MavlinkCommand.h"
#include "AP_Var_keys.h"
#include "../AP_RangeFinder/AP_RangeFinder.h"
#include "../AP_IMU/AP_IMU.h"
#include "../APM_BMP085/APM_BMP085.h"

namespace apo {

AP_Navigator::AP_Navigator(AP_HardwareAbstractionLayer * hal) :
	_hal(hal), _timeStamp(0), _roll(0), _rollRate(0), _pitch(0),
			_pitchRate(0), _yaw(0), _yawRate(0), _airSpeed(0),
			_groundSpeed(0), _vD(0), _lat_degInt(0),
			_lon_degInt(0), _alt_intM(0) {
}
void AP_Navigator::calibrate() {
}
float AP_Navigator::getPD() const {
	return AP_MavlinkCommand::home.getPD(getAlt_intM());
}

float AP_Navigator::getPE() const {
	return AP_MavlinkCommand::home.getPE(getLat_degInt(), getLon_degInt());
}

float AP_Navigator::getPN() const {
	return AP_MavlinkCommand::home.getPN(getLat_degInt(), getLon_degInt());
}

void AP_Navigator::setPD(float _pD) {
	setAlt(AP_MavlinkCommand::home.getAlt(_pD));
}

void AP_Navigator::setPE(float _pE) {
	setLat(AP_MavlinkCommand::home.getLat(_pE));
}

void AP_Navigator::setPN(float _pN) {
	setLon(AP_MavlinkCommand::home.getLon(_pN));
}

DcmNavigator::DcmNavigator(AP_HardwareAbstractionLayer * hal) :
	AP_Navigator(hal), _dcm(), _imuOffsetAddress(0) {

	// if orientation equal to front, store as front
	/**
	 * rangeFinder<direction> is assigned values based on orientation which
	 * is specified in ArduPilotOne.pde.
	 */
	for (uint8_t i = 0; i < _hal-> rangeFinders.getSize(); i++) {
		if (_hal->rangeFinders[i] == NULL)
			continue;
		if (_hal->rangeFinders[i]->orientation_x == 0
				&& _hal->rangeFinders[i]->orientation_y == 0
				&& _hal->rangeFinders[i]->orientation_z == 1)
			_rangeFinderDown = _hal->rangeFinders[i];
	}

	if (_hal->getMode() == MODE_LIVE) {

		if (_hal->adc) {
			_hal->imu = new AP_IMU_Oilpan(_hal->adc, k_sensorCalib);
		}

		if (_hal->imu) {
			_dcm = new AP_DCM(_hal->imu, _hal->gps, _hal->compass);

			// tune down dcm
			_dcm->kp_roll_pitch(0.030000);
			_dcm->ki_roll_pitch(0.00001278),	// 50 hz I term

			// tune down compass in dcm
			_dcm->kp_yaw(0.08);
			_dcm->ki_yaw(0);
		}

		if (_hal->compass) {
			_dcm->set_compass(_hal->compass);
		}
	}
}
void DcmNavigator::calibrate() {

	AP_Navigator::calibrate();

	// TODO: handle cold/warm restart
	if (_hal->imu) {
		_hal->imu->init(IMU::COLD_START,delay);
	}
}
void DcmNavigator::updateFast(float dt) {

	if (_hal->getMode() != MODE_LIVE)
		return;

	setTimeStamp(micros()); // if running in live mode, record new time stamp


	//_hal->debug->println_P(PSTR("nav loop"));

	/**
	 * The altitued is read off the barometer by implementing the following formula:
	 * altitude (in m) = 44330*(1-(p/po)^(1/5.255)),
	 * where, po is pressure in Pa at sea level (101325 Pa).
	 * See http://www.sparkfun.com/tutorials/253 or type this formula
	 * in a search engine for more information.
	 * altInt contains the altitude in meters.
	 */
	if (_hal->baro) {

		if (_rangeFinderDown != NULL && _rangeFinderDown->distance <= 695)
			setAlt(_rangeFinderDown->distance);

		else {
			float tmp = (_hal->baro->Press / 101325.0);
			tmp = pow(tmp, 0.190295);
			//setAlt(44330 * (1.0 - tmp)); //sets the altitude in meters XXX wrong, baro reads 0 press
			setAlt(0.0);
		}
	}

	// dcm class for attitude
	if (_dcm) {
		_dcm->update_DCM_fast();
		setRoll(_dcm->roll);
		setPitch(_dcm->pitch);
		setYaw(_dcm->yaw);
		setRollRate(_dcm->get_gyro().x);
		setPitchRate(_dcm->get_gyro().y);
		setYawRate(_dcm->get_gyro().z);

		/*
		 * accel/gyro debug
		 */
		/*
		 Vector3f accel = _hal->imu->get_accel();
		 Vector3f gyro = _hal->imu->get_gyro();
		 Serial.printf_P(PSTR("accel: %f %f %f gyro: %f %f %f\n"),
		 accel.x,accel.y,accel.z,gyro.x,gyro.y,gyro.z);
		 */
	}
}
void DcmNavigator::updateSlow(float dt) {
	if (_hal->getMode() != MODE_LIVE)
		return;

	setTimeStamp(micros()); // if running in live mode, record new time stamp

	if (_hal->gps) {
		_hal->gps->update();
		updateGpsLight();
		if (_hal->gps->fix && _hal->gps->new_data) {
			setLat_degInt(_hal->gps->latitude);
			setLon_degInt(_hal->gps->longitude);
			setAlt_intM(_hal->gps->altitude * 10); // gps in cm, intM in mm
			setGroundSpeed(_hal->gps->ground_speed / 100.0); // gps is in cm/s
		}
	}

	if (_hal->compass) {
		_hal->compass->read();
		_hal->compass->calculate(_dcm->get_dcm_matrix());
		_hal->compass->null_offsets(_dcm->get_dcm_matrix());
		//_hal->debug->printf_P(PSTR("heading: %f"), _hal->compass->heading);
	}
}
void DcmNavigator::updateGpsLight(void) {
	// GPS LED on if we have a fix or Blink GPS LED if we are receiving data
	// ---------------------------------------------------------------------
	static bool GPS_light = false;
	switch (_hal->gps->status()) {
	case (2):
		//digitalWrite(C_LED_PIN, HIGH); //Turn LED C on when gps has valid fix.
		break;

	case (1):
		if (_hal->gps->valid_read == true) {
			GPS_light = !GPS_light; // Toggle light on and off to indicate gps messages being received, but no GPS fix lock
			if (GPS_light) {
				digitalWrite(_hal->cLedPin, LOW);
			} else {
				digitalWrite(_hal->cLedPin, HIGH);
			}
			_hal->gps->valid_read = false;
		}
		break;

	default:
		digitalWrite(_hal->cLedPin, LOW);
		break;
	}
}

} // namespace apo
