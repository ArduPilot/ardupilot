/*
 * AP_Navigator.cpp
 *
 *  Created on: Apr 30, 2011
 *      Author: jgoppert
 */

#include "AP_Navigator.h"
#include "AP_MavlinkCommand.h"

namespace apo {

AP_Navigator::AP_Navigator(AP_Board * board) :
    _board(board), _timeStamp(0), _roll(0), _rollRate(0), _pitch(0),
    _pitchRate(0), _yaw(0), _yawRate(0), 
    _windSpeed(0), _windDirection(0),
    _vN(0), _vE(0), _vD(0), _lat_degInt(0),
    _lon_degInt(0), _alt_intM(0) {
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

} // namespace apo
// vim:ts=4:sw=4:expandtab
