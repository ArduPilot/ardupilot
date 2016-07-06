#include "AvoidanceHandler.h"

bool AvoidanceHandler::enter(AP_Avoidance::Obstacle &threat, AvoidanceHandler *_old_handler) {
    if (_old_handler != nullptr) {
        // check here whether we should change modes or not
        _old_handler->exit();
    }
    _threat = &threat;
    return true;
}

void AvoidanceHandler::internal_error()
{
    // internal_errors++;
#if CONFIG_HAL_BOARD == HAL_BOARD_SITL
    abort();
#endif
}



// returns XYZ
Vector3f AvoidanceHandler::perpendicular_xyz(const Location &p1, const Vector3f &v1, const Location &p2)
{
    Vector2f delta_p_2d = location_diff(p1, p2);
    Vector3f delta_p_xyz = Vector3f(delta_p_2d[1],delta_p_2d[0],(p2.alt-p1.alt)/100.0f); //check this line
    Vector3f v1_xyz = Vector3f(v1[1], v1[0], -v1[2]);
    Vector3f ret = Vector3f::perpendicular(delta_p_xyz, v1_xyz);
    return ret;
}

// return XY
Vector2f AvoidanceHandler::perpendicular_xy(const Location &p1, const Vector3f &v1, const Location &p2)
{
    Vector2f delta_p = location_diff(p1, p2);
    Vector2f delta_p_n = Vector2f(delta_p[1],delta_p[0]);
    Vector2f v1n(v1[1],v1[0]);
    Vector2f ret_xy = Vector2f::perpendicular(delta_p_n, v1n);
    return ret_xy;
}

// wp_speeds in cm/s
bool AvoidanceHandler::new_destination_perpendicular(Vector3f &newdest_neu, const AP_AHRS &_ahrs, const float wp_speed_xy, const float wp_speed_z, const uint8_t _minimum_avoid_height)
{
    if (_threat == nullptr) {
        // why where we called?!
        return false;
    }

    Location my_abs_pos;
    if (! _ahrs.get_position(my_abs_pos)) {
        // we should not get to here!  If we don't know our position
        // we can't know if there are any threats, for starters!
        return false;
    }

    Vector3f my_pos_ned;
    if (! _ahrs.get_relative_position_NED(my_pos_ned)) {
        // we should not get to here!  If we don't know our position
        // we know if there are any threats, for starters!
        return false;
    }
    
    // if their velocity is moving around close to zero then flying
    // perpendicular to that velocity may mean we do weird things.
    // Instead, we will fly directly away from them:
    if (_threat->_velocity.length() < _low_velocity_threshold) {
        const Vector2f delta_pos_xy =  location_diff(_threat->_location, my_abs_pos);
        const float delta_pos_z = my_abs_pos.alt - _threat->_location.alt;
        Vector3f delta_pos_xyz = Vector3f(delta_pos_xy[0],delta_pos_xy[1],delta_pos_z);
        delta_pos_xyz.normalize();
        newdest_neu[0] = my_pos_ned[0]*100 + delta_pos_xyz[0] * wp_speed_xy * 10; // 10 second
        newdest_neu[1] = my_pos_ned[1]*100 + delta_pos_xyz[1] * wp_speed_xy * 10; // 10 second
        newdest_neu[2] = -my_pos_ned[2]*100 + delta_pos_xyz[2] * wp_speed_z * 10; // 10 seconds
        if(newdest_neu[2] < _minimum_avoid_height*100) {
            newdest_neu[0] = my_pos_ned[0]*100 + delta_pos_xy[0] * wp_speed_xy * 10; // 10 second
            newdest_neu[1] = my_pos_ned[1]*100 + delta_pos_xy[1] * wp_speed_xy * 10; // 10 second
            newdest_neu[2] = -my_pos_ned[2]*100;
        }
        return true;
    }

    {
        Vector3f perp_xyz = perpendicular_xyz(_threat->_location, _threat->_velocity, my_abs_pos);
        perp_xyz.normalize();
        newdest_neu[0] = my_pos_ned[0]*100 + perp_xyz[1] * wp_speed_xy * 10; // ten seconds
        newdest_neu[1] = my_pos_ned[1]*100 + perp_xyz[0] * wp_speed_xy * 10; // ten seconds
        newdest_neu[2] =  -my_pos_ned[2]*100 + perp_xyz[2] * wp_speed_z * 10; // ten seconds
    }

    if (newdest_neu[2] < _minimum_avoid_height*100) {
        // too close to the ground to do 3D avoidance
        // GCS_MAVLINK::send_statustext_all(MAV_SEVERITY_WARNING, "AVOID: PERPENDICULAR: 2D");
        Vector2f perp_xy = perpendicular_xy(_threat->_location, _threat->_velocity, my_abs_pos);
        perp_xy.normalize();
        newdest_neu[0] = my_pos_ned[0]*100 + perp_xy[1] * wp_speed_xy * 10; // ten seconds
        newdest_neu[1] = my_pos_ned[1]*100 + perp_xy[0] * wp_speed_xy * 10; // ten seconds
        newdest_neu[2] = -my_pos_ned[2]*100;
    }

    return true;
}
