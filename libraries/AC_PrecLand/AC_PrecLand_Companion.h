#pragma once

#include <AP_Math/AP_Math.h>
#include "AC_PrecLand_Backend.h"

/*
 * AC_PrecLand_Companion - implements precision landing using target vectors provided
 *                         by a companion computer (i.e. Odroid) communicating via MAVLink
 *                         The companion computer must provide "Line-Of-Sight" measurements
 *                         in the form of LANDING_TARGET mavlink messages.
 */

class AC_PrecLand_Companion : public AC_PrecLand_Backend
{
public:
    // Constructor
    using AC_PrecLand_Backend::AC_PrecLand_Backend;

    // perform any required initialisation of backend
    void init() override;

    // retrieve updates from sensor
    void update() override;

    // provides a unit vector towards the target in body frame
    //  returns same as have_los_meas()
    bool get_los_body(Vector3f& ret) override;

    // returns system time in milliseconds of last los measurement
    uint32_t los_meas_time_ms() override;

    // return true if there is a valid los measurement available
    bool have_los_meas() override;

    // provides a local FRD position
    // FRD local frame (x: Front, y: Right, z: Down) with origin that travels with the vehicle Z - down to earth.
    // returns same as have_los_meas_local_frd() 
    bool get_los_local_frd_target(Vector3f& local_frd) override;

    // returns system time in milliseconds of last los measurement
    uint32_t los_meas_local_frd_time_ms() override;

    // return true if there is a valid los measurement available
    bool have_los_meas_local_frd() override;

    // provides a local NED position
    // NED local tangent frame (x: North, y: East, z: Down) with origin fixed relative to earth.
    // returns same as have_los_meas_local_ned() 
    bool get_los_local_ned_target(Vector3f& local_ned) override;

    // returns system time in milliseconds of last los measurement
    uint32_t los_meas_local_ned_time_ms() override;

    // return true if there is a valid los measurement available
    bool have_los_meas_local_ned() override;

    // provides a local offset NED position
    // NED local tangent frame (x: North, y: East, z: Down) with origin that travels with the vehicle.
    // returns same as have_los_meas_local_offset_ned() 
    bool get_los_local_offset_ned_target(Vector3f& local_offset_ned) override;

    // returns system time in milliseconds of last los measurement
    uint32_t los_meas_local_offset_ned_time_ms() override;

    // return true if there is a valid los measurement available
    bool have_los_meas_local_offset_ned() override;


    // returns distance to target in meters (0 means distance is not known)
    float distance_to_target() override;

    // parses a mavlink message from the companion computer
    void handle_msg(const mavlink_landing_target_t &packet, uint32_t timestamp_ms) override;

private:
    float               _distance_to_target;    // distance from the camera to target in meters

    Vector3f            _los_meas_body;         // unit vector in body frame pointing towards target
    bool                _have_los_meas;         // true if there is a valid measurement from the camera
    uint32_t            _los_meas_time_ms;      // system time in milliseconds when los was measured

    Vector3f            _los_meas_local_frd;            // FRD local  frame (x: Front, y: Right, z: Down) with origin that travels with the vehicle Z - down to earth.
    bool                _have_los_meas_local_frd;       // true if there is a valid FRD data
    uint32_t            _los_meas_local_frd_time_ms;    // system time in milliseconds when los was measured
   
    Vector3f            _los_meas_local_ned;            // NED local tangent frame (x: North, y: East, z: Down) with origin fixed relative to earth.
    bool                _have_los_meas_local_ned;       // true if there is a valid NED data
    uint32_t            _los_meas_local_ned_time_ms;    // system time in milliseconds when los was measured

    Vector3f            _los_meas_local_offset_ned;         // NED local tangent frame (x: North, y: East, z: Down) with origin that travels with the vehicle.
    bool                _have_los_meas_local_offset_ned;    // true if there is a valid NED data
    uint32_t            _los_meas_local_offset_ned_time_ms; // system time in milliseconds when los was measured



    bool                _unsupported_frame_msg_sent;  // Flag to send message in case of unsupported frame received.

};
