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

namespace apo {

class AP_Board;

/// Navigator class
class AP_Navigator {
public:
    AP_Navigator(AP_Board * board);

    // note, override these with derived navigator functionality
    virtual void calibrate() {};
    virtual void updateFast(float dt) {};
    virtual void updateSlow(float dt) {};


    // accessors
    float getPD() const;
    float getPE() const;
    float getPN() const;
    void setPD(float _pD);
    void setPE(float _pE);
    void setPN(float _pN);

    float getAirSpeed() const {
        // neglects vertical wind
        float vWN = getVN() + getWindSpeed()*cos(getWindDirection());
        float vWE = getVE() + getWindSpeed()*sin(getWindDirection());
        return sqrt(vWN*vWN+vWE+vWE+getVD()*getVD());
    }

    float getGroundSpeed() const {
        return sqrt(getVN()*getVN()+getVE()*getVE());
    } 

    float getWindSpeed() const {
        return _windSpeed;
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

    float getVN() const {
        return _vN;
    }

    float getVE() const {
        return _vE;
    }

    float getVD() const {
        return _vD;
    }

    int32_t getLat_degInt() const {
        //Serial.print("getLat_degInt");
        //Serial.println(_lat_degInt);
        return _lat_degInt;

    }

    int32_t getLon_degInt() const {
        return _lon_degInt;
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

    float getWindDirection() const {
        return _windDirection;
    }

    float getCourseOverGround() const {
        return atan2(getVE(),getVN());
    }

    float getRelativeCourseOverGround() const {
        float y = getCourseOverGround() - getYaw();
        if (y > 180 * deg2Rad)
            y -= 360 * deg2Rad;
        if (y < -180 * deg2Rad)
            y += 360 * deg2Rad;
        return y;
    }


    float getSpeedOverGround() const {
        return sqrt(getVN()*getVN()+getVE()*getVE());
    }

    float getXAccel() const {
        return _xAccel;
    }

    float getYAccel() const {
        return _yAccel;
    }

    float getZAccel() const {
        return _zAccel;
    }

    void setAirSpeed(float airSpeed) {
        // assumes wind constant and rescale navigation speed
        float vScale = (1 + airSpeed/getAirSpeed());
        float vNorm = sqrt(getVN()*getVN()+getVE()*getVE()+getVD()*getVD());
        _vN *= vScale/vNorm;
        _vE *= vScale/vNorm;
        _vD *= vScale/vNorm;
    }

    void setAlt_intM(int32_t alt_intM) {
        _alt_intM = alt_intM;
    }

    void setVN(float vN) {
        _vN = vN;
    }

    void setVE(float vE) {
        _vE = vE;
    }

    void setVD(float vD) {
        _vD = vD;
    }

    void setXAccel(float xAccel) {
        _xAccel = xAccel;
    }

    void setYAccel(float yAccel) {
        _yAccel = yAccel;
    }

    void setZAccel(float zAccel) {
        _zAccel = zAccel;
    }

    void setGroundSpeed(float groundSpeed) {
        float cog = getCourseOverGround();
        _vN = cos(cog)*groundSpeed;
        _vE = sin(cog)*groundSpeed;
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

    void setWindDirection(float windDirection) {
        _windDirection = windDirection;
    }

    void setWindSpeed(float windSpeed) {
        _windSpeed = windSpeed;
    }

protected:
    AP_Board * _board;
private:
    int32_t _timeStamp;     /// time stamp for navigation data, micros clock
    float _roll;            /// roll angle, radians
    float _rollRate;        /// roll rate, radians/s
    float _pitch;           /// pitch angle, radians
    float _pitchRate;       /// pitch rate, radians/s
    float _yaw;             /// yaw angle, radians
    float _yawRate;         /// yaw rate, radians/s
    // vertical 
    float _windSpeed;       /// wind speed, m/s
    float _windDirection;   /// wind directioin, radians
    float _vN;              /// 
    float _vE;
    float _vD; // m/s
    float _xAccel;
    float _yAccel;
    float _zAccel;
    int32_t _lat_degInt; // deg / 1e7
    int32_t _lon_degInt; // deg / 1e7
    int32_t _alt_intM; // meters / 1e3
};

} // namespace apo

#endif // AP_Navigator_H
// vim:ts=4:sw=4:expandtab
