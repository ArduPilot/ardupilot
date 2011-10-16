/*
 * AP_Navigator.h
 * Copyright (C) James Goppert 2010 <james.goppert@gmail.com>
 *
 * This file is free software: you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the
 * Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This file is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#ifndef AP_Navigator_H
#define AP_Navigator_H

#include "constants.h"
#include <inttypes.h>

class RangeFinder;
class IMU;
class AP_DCM;

namespace apo {

class AP_HardwareAbstractionLayer;

/// Navigator class
class AP_Navigator {
public:
	AP_Navigator(AP_HardwareAbstractionLayer * hal);
	virtual void calibrate();
	virtual void updateFast(float dt) = 0;
	virtual void updateSlow(float dt) = 0;
	float getPD() const;
	float getPE() const;
	float getPN() const;
	void setPD(float _pD);
	void setPE(float _pE);
	void setPN(float _pN);

	float getAirSpeed() const {
		return _airSpeed;
	}

	int32_t getAlt_intM() const {
		return _alt_intM;
	}

	float getAlt() const {
		return _alt_intM / scale_m;
	}

	void setAlt(float _alt) {
		this->_alt_intM = _alt * scale_m;
	}

	float getLat() const {
		//Serial.print("getLatfirst");
		//Serial.println(_lat_degInt * degInt2Rad);
		return _lat_degInt * degInt2Rad;
	}

	void setLat(float _lat) {
		//Serial.print("setLatfirst");
		//Serial.println(_lat * rad2DegInt);
		setLat_degInt(_lat*rad2DegInt);
	}

	float getLon() const {
		return _lon_degInt * degInt2Rad;
	}

	void setLon(float _lon) {
		this->_lon_degInt = _lon * rad2DegInt;
	}

	float getVD() const {
		return _vD;
	}

	float getVE() const {
		return sin(getYaw()) * getGroundSpeed();
	}

	float getGroundSpeed() const {
		return _groundSpeed;
	}

	int32_t getLat_degInt() const {
		//Serial.print("getLat_degInt");
		//Serial.println(_lat_degInt);
		return _lat_degInt;

	}

	int32_t getLon_degInt() const {
		return _lon_degInt;
	}

	float getVN() const {
		return cos(getYaw()) * getGroundSpeed();
	}

	float getPitch() const {
		return _pitch;
	}

	float getPitchRate() const {
		return _pitchRate;
	}

	float getRoll() const {
		return _roll;
	}

	float getRollRate() const {
		return _rollRate;
	}

	float getYaw() const {
		return _yaw;
	}

	float getYawRate() const {
		return _yawRate;
	}

	void setAirSpeed(float airSpeed) {
		_airSpeed = airSpeed;
	}

	void setAlt_intM(int32_t alt_intM) {
		_alt_intM = alt_intM;
	}

	void setVD(float vD) {
		_vD = vD;
	}

	void setGroundSpeed(float groundSpeed) {
		_groundSpeed = groundSpeed;
	}

	void setLat_degInt(int32_t lat_degInt) {
		_lat_degInt = lat_degInt;
		//Serial.print("setLat_degInt");
		//Serial.println(_lat_degInt);
	}

	void setLon_degInt(int32_t lon_degInt) {
		_lon_degInt = lon_degInt;
	}

	void setPitch(float pitch) {
		_pitch = pitch;
	}

	void setPitchRate(float pitchRate) {
		_pitchRate = pitchRate;
	}

	void setRoll(float roll) {
		_roll = roll;
	}

	void setRollRate(float rollRate) {
		_rollRate = rollRate;
	}

	void setYaw(float yaw) {
		_yaw = yaw;
	}

	void setYawRate(float yawRate) {
		_yawRate = yawRate;
	}
	void setTimeStamp(int32_t timeStamp) {
		_timeStamp = timeStamp;
	}
	int32_t getTimeStamp() const {
		return _timeStamp;
	}

protected:
	AP_HardwareAbstractionLayer * _hal;
private:
	int32_t _timeStamp; // micros clock
	float _roll; // rad
	float _rollRate; //rad/s
	float _pitch; // rad
	float _pitchRate; // rad/s
	float _yaw; // rad
	float _yawRate; // rad/s
	float _airSpeed; // m/s
	float _groundSpeed; // m/s
	float _vD; // m/s
	int32_t _lat_degInt; // deg / 1e7
	int32_t _lon_degInt; // deg / 1e7
	int32_t _alt_intM; // meters / 1e3
};

class DcmNavigator: public AP_Navigator {
private:
	/**
	 * Sensors
	 */

	RangeFinder * _rangeFinderDown;
	AP_DCM * _dcm;
	IMU * _imu;
	uint16_t _imuOffsetAddress;

public:
	DcmNavigator(AP_HardwareAbstractionLayer * hal);
	virtual void calibrate();
	virtual void updateFast(float dt);
	virtual void updateSlow(float dt);
	void updateGpsLight(void);
};

} // namespace apo

#endif // AP_Navigator_H
// vim:ts=4:sw=4:expandtab
