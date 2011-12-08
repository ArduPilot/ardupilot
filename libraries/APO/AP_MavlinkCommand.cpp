/*
 * AP_MavlinkCommand.cpp
 *
 *  Created on: Apr 30, 2011
 *      Author: jgoppert
 */

#include "../FastSerial/FastSerial.h"
#include "AP_MavlinkCommand.h"

namespace apo {

AP_MavlinkCommand::AP_MavlinkCommand(const AP_MavlinkCommand & v) :
    _data(v._data), _seq(v._seq) {
    //if (FastSerial::getInitialized(0)) Serial.println("copy ctor");
}

AP_MavlinkCommand::AP_MavlinkCommand(uint16_t index, bool doLoad) :
    _data(k_commands + index), _seq(index) {

    if (FastSerial::getInitialized(0)) {
        Serial.println("index ctor");
        Serial.println("++");
        Serial.print("index: ");
        Serial.println(index);
        Serial.print("key: ");
        Serial.println(k_commands + index);
        Serial.println("++");
    }

    // default values for structure
    _data.get().command = MAV_CMD_NAV_WAYPOINT;
    _data.get().autocontinue = true;
    _data.get().frame = MAV_FRAME_GLOBAL;
    _data.get().param1 = 0;
    _data.get().param2 = 10; // radius of 10 meters
    _data.get().param3 = 0;
    _data.get().param4 = 0;
    _data.get().x = 0;
    _data.get().y = 0;
    _data.get().z = 1000;

    // This is a failsafe measure to stop trying to load a command if it can't load
    if (doLoad && !load()) {
        Serial.println("load failed, reverting to home waypoint");
        _data = AP_MavlinkCommand::home._data;
        _seq = AP_MavlinkCommand::home._seq;
    }
}

AP_MavlinkCommand::AP_MavlinkCommand(const mavlink_waypoint_t & cmd) :
    _data(k_commands + cmd.seq), _seq(cmd.seq) {
    setCommand(MAV_CMD(cmd.command));
    setAutocontinue(cmd.autocontinue);
    setFrame(MAV_FRAME(cmd.frame));
    setParam1(cmd.param1);
    setParam2(cmd.param2);
    setParam3(cmd.param3);
    setParam4(cmd.param4);
    setX(cmd.x);
    setY(cmd.y);
    setZ(cmd.z);
    save();

    // reload home if sent, home must be a global waypoint
    if ( (cmd.seq == 0) && (cmd.frame == MAV_FRAME_GLOBAL)) home.load();

    Serial.println("============================================================");
    Serial.println("storing new command from mavlink_waypoint_t");
    Serial.print("key: ");
    Serial.println(_data.key(),DEC);
    Serial.print("number: ");
    Serial.println(cmd.seq,DEC);
    Serial.print("command: ");
    Serial.println(getCommand());
    Serial.print("autocontinue: ");
    Serial.println(getAutocontinue(),DEC);
    Serial.print("frame: ");
    Serial.println(getFrame(),DEC);
    Serial.print("1000*param1: ");
    Serial.println(int(1000*getParam1()),DEC);
    Serial.print("1000*param2: ");
    Serial.println(int(1000*getParam2()),DEC);
    Serial.print("1000*param3: ");
    Serial.println(int(1000*getParam3()),DEC);
    Serial.print("1000*param4: ");
    Serial.println(int(1000*getParam4()),DEC);
    Serial.print("1000*x0: ");
    Serial.println(int(1000*cmd.x),DEC);
    Serial.print("1000*y0: ");
    Serial.println(int(1000*cmd.y),DEC);
    Serial.print("1000*z0: ");
    Serial.println(int(1000*cmd.z),DEC);
    Serial.print("1000*x: ");
    Serial.println(int(1000*getX()),DEC);
    Serial.print("1000*y: ");
    Serial.println(int(1000*getY()),DEC);
    Serial.print("1000*z: ");
    Serial.println(int(1000*getZ()),DEC);

    load();

    Serial.print("1000*x1: ");
    Serial.println(int(1000*getX()),DEC);
    Serial.print("1000*y1: ");
    Serial.println(int(1000*getY()),DEC);
    Serial.print("1000*z1: ");
    Serial.println(int(1000*getZ()),DEC);
    Serial.println("============================================================");
    Serial.flush();
}

mavlink_waypoint_t AP_MavlinkCommand::convert(uint8_t current) const {
    mavlink_waypoint_t mavCmd;
    mavCmd.seq = getSeq();
    mavCmd.command = getCommand();
    mavCmd.frame = getFrame();
    mavCmd.param1 = getParam1();
    mavCmd.param2 = getParam2();
    mavCmd.param3 = getParam3();
    mavCmd.param4 = getParam4();
    mavCmd.x = getX();
    mavCmd.y = getY();
    mavCmd.z = getZ();
    mavCmd.autocontinue = getAutocontinue();
    mavCmd.current = (getSeq() == current);
    mavCmd.target_component = mavlink_system.compid;
    mavCmd.target_system = mavlink_system.sysid;
    return mavCmd;
}

float AP_MavlinkCommand::bearingTo(const AP_MavlinkCommand & next) const {
    float deltaLon = next.getLon() - getLon();
    /*
     Serial.print("Lon: "); Serial.println(getLon());
     Serial.print("nextLon: "); Serial.println(next.getLon());
     Serial.print("deltaLonDeg * 1e7: "); Serial.println(deltaLon*rad2DegInt);
     */
    float bearing = atan2(
                        sin(deltaLon) * cos(next.getLat()),
                        cos(getLat()) * sin(next.getLat()) - sin(getLat()) * cos(
                            next.getLat()) * cos(deltaLon));
    return bearing;
}

float AP_MavlinkCommand::bearingTo(int32_t latDegInt, int32_t lonDegInt) const {
    // have to be careful to maintain the precision of the gps coordinate
    float deltaLon = (lonDegInt - getLon_degInt()) * degInt2Rad;
    float nextLat = latDegInt * degInt2Rad;
    float bearing = atan2(
                        sin(deltaLon) * cos(nextLat),
                        cos(getLat()) * sin(nextLat) - sin(getLat()) * cos(nextLat)
                        * cos(deltaLon));
    if (bearing < 0)
        bearing += 2 * M_PI;
    return bearing;
}

float AP_MavlinkCommand::distanceTo(const AP_MavlinkCommand & next) const {
    float sinDeltaLat2 = sin((getLat() - next.getLat()) / 2);
    float sinDeltaLon2 = sin((getLon() - next.getLon()) / 2);
    float a = sinDeltaLat2 * sinDeltaLat2 + cos(getLat()) * cos(
                  next.getLat()) * sinDeltaLon2 * sinDeltaLon2;
    float c = 2 * atan2(sqrt(a), sqrt(1 - a));
    return rEarth * c;
}

float AP_MavlinkCommand::distanceTo(int32_t lat_degInt, int32_t lon_degInt) const {
    float sinDeltaLat2 = sin(
                             (lat_degInt - getLat_degInt()) * degInt2Rad / 2);
    float sinDeltaLon2 = sin(
                             (lon_degInt - getLon_degInt()) * degInt2Rad / 2);
    float a = sinDeltaLat2 * sinDeltaLat2 + cos(getLat()) * cos(
                  lat_degInt * degInt2Rad) * sinDeltaLon2 * sinDeltaLon2;
    float c = 2 * atan2(sqrt(a), sqrt(1 - a));
    /*
     Serial.print("wp lat_degInt: "); Serial.println(getLat_degInt());
     Serial.print("wp lon_degInt: "); Serial.println(getLon_degInt());
     Serial.print("lat_degInt: "); Serial.println(lat_degInt);
     Serial.print("lon_degInt: "); Serial.println(lon_degInt);
     Serial.print("sinDeltaLat2: "); Serial.println(sinDeltaLat2);
     Serial.print("sinDeltaLon2: "); Serial.println(sinDeltaLon2);
     */
    return rEarth * c;
}

//calculates cross track of a current location
float AP_MavlinkCommand::crossTrack(const AP_MavlinkCommand & previous,
                                    int32_t lat_degInt, int32_t lon_degInt) const {
    float d = previous.distanceTo(lat_degInt, lon_degInt);
    float bCurrent = previous.bearingTo(lat_degInt, lon_degInt);
    float bNext = previous.bearingTo(*this);
    return asin(sin(d / rEarth) * sin(bCurrent - bNext)) * rEarth;
}

// calculates along  track distance of a current location
float AP_MavlinkCommand::alongTrack(const AP_MavlinkCommand & previous,
                                    int32_t lat_degInt, int32_t lon_degInt) const {
    float t1N = previous.getPN(lat_degInt, lon_degInt);
    float t1E = previous.getPE(lat_degInt, lon_degInt);
    float t2N = previous.getPN(getLat_degInt(), getLon_degInt());
    float t2E = previous.getPE(getLat_degInt(), getLon_degInt());
    float segmentLength = previous.distanceTo(*this);
    if (segmentLength == 0) return 0;
    return (t1N*t2N + t1E*t2E)/segmentLength;
}


AP_MavlinkCommand AP_MavlinkCommand::home = AP_MavlinkCommand(0,false);

} // namespace apo
// vim:ts=4:sw=4:expandtab
