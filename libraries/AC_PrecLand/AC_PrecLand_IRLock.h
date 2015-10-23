/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-
#ifndef __AC_PRECLAND_IRLOCK_H__
#define __AC_PRECLAND_IRLOCK_H__

#include <AP_Common/AP_Common.h>
#include <AP_Math/AP_Math.h>
#include <AC_PrecLand/AC_PrecLand_Backend.h>
#include <AP_IRLock/AP_IRLock.h>

// this only builds for PX4 so far
#if CONFIG_HAL_BOARD == HAL_BOARD_PX4 || CONFIG_HAL_BOARD == HAL_BOARD_VRBRAIN

/*
 * AC_PrecLand_IRLock - implements precision landing using target vectors provided
 *                         by a companion computer (i.e. Odroid) communicating via MAVLink
 */

class AC_PrecLand_IRLock : public AC_PrecLand_Backend
{
public:

    // Constructor
    AC_PrecLand_IRLock(const AC_PrecLand& frontend, AC_PrecLand::precland_state& state);

    // init - perform any required initialisation of backend controller
    void init();

    // update - give chance to driver to get updates from sensor
    //  returns true if new data available
    bool update();
    // IRLock is hard-mounted to the frame of the vehicle, so it will always be in body-frame
    MAV_FRAME get_frame_of_reference() { return MAV_FRAME_BODY_NED; }

    // get_angle_to_target - returns body frame angles (in radians) to target
    //  returns true if angles are available, false if not (i.e. no target)
    //  x_angle_rad : body-frame roll direction, positive = target is to right (looking down)
    //  y_angle_rad : body-frame pitch direction, postiive = target is forward (looking down)
    bool get_angle_to_target(float &x_angle_rad, float &y_angle_rad);

    // handle_msg - parses a mavlink message from the companion computer
    void handle_msg(mavlink_message_t* msg) { /* do nothing */ }

private:
    AP_IRLock_PX4 irlock;

};
#endif
#endif	// __AC_PRECLAND_IRLOCK_H__
