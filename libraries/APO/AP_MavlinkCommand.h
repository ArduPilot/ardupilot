/*
 * AP_MavlinkCommand.h
 *
 *  Created on: Apr 4, 2011
 *      Author: jgoppert
 */

#ifndef AP_MAVLINKCOMMAND_H_
#define AP_MAVLINKCOMMAND_H_

#include "../GCS_MAVLink/GCS_MAVLink.h"
#include "../AP_Common/AP_Common.h"
#include "AP_Var_keys.h"
#include "constants.h"

namespace apo {

class AP_MavlinkCommand {
private:
    struct CommandStorage {
        MAV_CMD command;
        bool autocontinue;
        MAV_FRAME frame;
        float param1;
        float param2;
        float param3;
        float param4;
        float x;
        float y;
        float z;
    };
    AP_VarS<CommandStorage> _data;
    uint16_t _seq;
public:
    static AP_MavlinkCommand home;

    /**
     * Copy Constructor
     */
    AP_MavlinkCommand(const AP_MavlinkCommand & v);

    /**
     * Basic Constructor
     * @param index Start at zero.
     */
    AP_MavlinkCommand(uint16_t index, bool doLoad = true);

    /**
     * Constructor for copying/ saving from a mavlink waypoint.
     * @param cmd The mavlink_waopint_t structure for the command.
     */
    AP_MavlinkCommand(const mavlink_waypoint_t & cmd);

    bool save() {
        return _data.save();
    }
    bool load() {
        return _data.load();
    }
    uint8_t getSeq() const {
        return _seq;
    }
    bool getAutocontinue() const {
        return _data.get().autocontinue;
    }
    void setAutocontinue( bool val) {
        _data.get().autocontinue = val;
    }
    void setSeq(uint8_t val) {
        _seq = val;
    }
    MAV_CMD getCommand() const {
        return _data.get().command;
    }
    void setCommand(MAV_CMD val) {
        _data.get().command = val;
    }
    MAV_FRAME getFrame() const {
        return _data.get().frame;
    }
    void setFrame(MAV_FRAME val) {
        _data.get().frame = val;
    }
    float getParam1() const {
        return _data.get().param1;
    }
    void setParam1(float val) {
        _data.get().param1 = val;
    }
    float getParam2() const {
        return _data.get().param2;
    }
    void setParam2(float val) {
        _data.get().param2 = val;
    }
    float getParam3() const {
        return _data.get().param3;
    }
    void setParam3(float val) {
        _data.get().param3 = val;
    }
    float getParam4() const {
        return _data.get().param4;
    }
    void setParam4(float val) {
        _data.get().param4 = val;
    }
    float getX() const {
        return _data.get().x;
    }
    void setX(float val) {
        _data.get().x = val;
    }
    float getY() const {
        return _data.get().y;
    }
    void setY(float val) {
        _data.get().y = val;
    }
    float getZ() const {
        return _data.get().z;
    }
    void setZ(float val) {
        _data.get().z = val;
    }

    float getYawCommand() const {
        return deg2Rad*getParam4();
    }

    float getLatDeg() const {
        switch (getFrame()) {
        case MAV_FRAME_GLOBAL:
        case MAV_FRAME_GLOBAL_RELATIVE_ALT:
            return getX();
            break;
        case MAV_FRAME_LOCAL:
        case MAV_FRAME_LOCAL_ENU:
        case MAV_FRAME_MISSION:
        default:
            return 0;
            break;
        }
    }
    void setLatDeg(float val) {
        switch (getFrame()) {
        case MAV_FRAME_GLOBAL:
        case MAV_FRAME_GLOBAL_RELATIVE_ALT:
            setX(val);
            break;
        case MAV_FRAME_LOCAL:
        case MAV_FRAME_LOCAL_ENU:
        case MAV_FRAME_MISSION:
        default:
            break;
        }
    }
    float getLonDeg() const {
        switch (getFrame()) {
        case MAV_FRAME_GLOBAL:
        case MAV_FRAME_GLOBAL_RELATIVE_ALT:
            return getY();
            break;
        case MAV_FRAME_LOCAL:
        case MAV_FRAME_LOCAL_ENU:
        case MAV_FRAME_MISSION:
        default:
            return 0;
            break;
        }
    }
    void setLonDeg(float val) {
        switch (getFrame()) {
        case MAV_FRAME_GLOBAL:
        case MAV_FRAME_GLOBAL_RELATIVE_ALT:
            setY(val);
            break;
        case MAV_FRAME_LOCAL:
        case MAV_FRAME_LOCAL_ENU:
        case MAV_FRAME_MISSION:
        default:
            break;
        }
    }
    void setLon(float val) {
        setLonDeg(val * rad2Deg);
    }
    void setLon_degInt(int32_t val) {
        setLonDeg(val / 1.0e7);
    }
    void setLat_degInt(int32_t val) {
        setLatDeg(val / 1.0e7);
    }
    int32_t getLon_degInt() const {
        return getLonDeg() * 1e7;
    }
    int32_t getLat_degInt() const {
        return getLatDeg() * 1e7;
    }
    float getLon() const {
        return getLonDeg() * deg2Rad;
    }
    float getLat() const {
        return getLatDeg() * deg2Rad;
    }
    void setLat(float val) {
        setLatDeg(val * rad2Deg);
    }
    float getAlt() const {
        switch (getFrame()) {
        case MAV_FRAME_GLOBAL:
            return getZ();
            break;
        case MAV_FRAME_GLOBAL_RELATIVE_ALT:
        case MAV_FRAME_LOCAL:
            return -getZ() + home.getAlt();
            break;
        case MAV_FRAME_LOCAL_ENU:
            return getZ() + home.getAlt();
            break;
        case MAV_FRAME_MISSION:
        default:
            return 0;
            break;
        }
    }
    /**
     * set the altitude in meters
     */
    void setAlt(float val) {
        switch (getFrame()) {
        case MAV_FRAME_GLOBAL:
        case MAV_FRAME_GLOBAL_RELATIVE_ALT:
            setZ(val);
            break;
        case MAV_FRAME_LOCAL:
            setZ(home.getLonDeg() - val);
            break;
        case MAV_FRAME_LOCAL_ENU:
            setZ(val - home.getLonDeg());
            break;
        case MAV_FRAME_MISSION:
        default:
            break;
        }
    }
    /**
     * Get the relative altitude to home
     * @return relative altitude in meters
     */
    float getRelAlt() const {
        switch (getFrame()) {
        case MAV_FRAME_GLOBAL:
            return getZ() - home.getAlt();
            break;
        case MAV_FRAME_GLOBAL_RELATIVE_ALT:
        case MAV_FRAME_LOCAL:
            return -getZ();
            break;
        case MAV_FRAME_LOCAL_ENU:
            return getZ();
            break;
        case MAV_FRAME_MISSION:
        default:
            return 0;
            break;
        }
    }
    /**
     * set the relative altitude in meters from home (up)
     */
    void setRelAlt(float val) {
        switch (getFrame()) {
        case MAV_FRAME_GLOBAL:
            setZ(val + home.getAlt());
            break;
        case MAV_FRAME_GLOBAL_RELATIVE_ALT:
        case MAV_FRAME_LOCAL:
            setZ(-val);
            break;
        case MAV_FRAME_LOCAL_ENU:
            setZ(val);
            break;
        case MAV_FRAME_MISSION:
            break;
        }
    }

    float getRadius() const {
        return getParam2();
    }

    void setRadius(float val) {
        setParam2(val);
    }

    /**
     * conversion for outbound packets to ground station
     * @return output the mavlink_waypoint_t packet
     */
    mavlink_waypoint_t convert(uint8_t current) const;

    /**
     * Calculate the bearing from this command to the next command
     * @param next The command to calculate the bearing to.
     * @return the bearing
     */
    float bearingTo(const AP_MavlinkCommand & next) const;

    /**
     * Bearing form this command to a gps coordinate in integer units
     * @param latDegInt latitude in degrees E-7
     * @param lonDegInt longitude in degrees E-7
     * @return
     */
    float bearingTo(int32_t latDegInt, int32_t lonDegInt) const;

    /**
     * Distance to another command
     * @param next The command to measure to.
     * @return The distance in meters.
     */
    float distanceTo(const AP_MavlinkCommand & next) const;

    /**
     * Distance to a gps coordinate in integer units
     * @param latDegInt latitude in degrees E-7
     * @param lonDegInt longitude in degrees E-7
     * @return The distance in meters.
     */
    float distanceTo(int32_t lat_degInt, int32_t lon_degInt) const;

    float getPN(int32_t lat_degInt, int32_t lon_degInt) const {
        // local tangent approximation at this waypoint
        float deltaLat = (lat_degInt - getLat_degInt()) * degInt2Rad;
        return deltaLat * rEarth;
    }

    float getPE(int32_t lat_degInt, int32_t lon_degInt) const {
        // local tangent approximation at this waypoint
        float deltaLon = (lon_degInt - getLon_degInt()) * degInt2Rad;
        return cos(getLat()) * deltaLon * rEarth;
    }

    float getPD(int32_t alt_intM) const {
        return -(alt_intM / scale_m - getAlt());
    }

    float getLat(float pN) const {

        return pN / rEarth + getLat();
    }

    float getLon(float pE) const {

        return pE / rEarth / cos(getLat()) + getLon();
    }

    /**
     * Gets altitude in meters
     * @param pD alt in meters
     * @return
     */
    float getAlt(float pD) const {

        return getAlt() - pD;
    }

    //calculates cross track of a current location
    float crossTrack(const AP_MavlinkCommand & previous, int32_t lat_degInt, int32_t lon_degInt) const;

    // calculates along  track distance of a current location
    float alongTrack(const AP_MavlinkCommand & previous, int32_t lat_degInt, int32_t lon_degInt) const;
};

} // namespace apo


#endif /* AP_MAVLINKCOMMAND_H_ */
// vim:ts=4:sw=4:expandtab
