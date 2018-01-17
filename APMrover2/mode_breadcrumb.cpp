#include "mode.h"
#include "Rover.h"
#include "AP_HAL/utility/RingBuffer.h"

#define BREADCRUMBS_BUFFER_LEN 500U
#define ZERO_DISTANCE 0.0f

bool ModeBreadcrumb::_enter()
{
    if (_crumbs_ring_buffer == nullptr) {
        // create ring buffer
        _crumbs_ring_buffer = new ObjectBuffer<Location>(BREADCRUMBS_BUFFER_LEN);
        if (_crumbs_ring_buffer == nullptr) {
            goto FAIL;
        }
    }

    if (_crumbs_sem == nullptr) {
        // create semaphore
        _crumbs_sem = hal.util->new_semaphore();
        if (_crumbs_sem == nullptr) {
            goto FAIL;
        }
    }

    lonely_mode = nullptr;

    // setting _last_crumb_added to our current location makes it so
    // crumbs won't be added until target moves far enough away from us
    _last_crumb_added = rover.current_loc;
    _last_crumb_added.alt = 0.0f;
    
    //reuse guided mode
    if (!ModeGuided::_enter()) {
        goto FAIL;
    }

    return true;

    FAIL:
        if (_crumbs_ring_buffer != nullptr) {
            delete _crumbs_ring_buffer;
            _crumbs_ring_buffer = nullptr;
        }

        return false;
}

void ModeBreadcrumb::_exit()
{
    if (_crumbs_ring_buffer != nullptr) {
        delete _crumbs_ring_buffer;
        _crumbs_ring_buffer = nullptr;
    }
}

void ModeBreadcrumb::run_lonely_mode()
{
    if (lonely_mode == nullptr) {
        rover.mode_hold.enter();
        lonely_mode = &rover.mode_hold;

        gcs().send_text(MAV_SEVERITY_INFO, "Breadcrumb: Lonely; %s", lonely_mode->name4());
    }

    lonely_mode->update();
}

bool ModeBreadcrumb::get_next_crumb()
{
    if (!_crumbs_ring_buffer || !_crumbs_ring_buffer->pop(_current_crumb)) {
        return false;
    }

    // re-use guided mode to set waypoint
    ModeGuided::set_desired_location(_current_crumb);

    _has_crumb = true;

    return true;
}

void ModeBreadcrumb::update()
{
    //attempt to get a crumb if we don't have one
    if (!_has_crumb && !get_next_crumb()) {
        // check for timeout (if we have no crumbs and timed out go into lonely)
        const uint32_t now = AP_HAL::millis();
        if (now - target_last_update_ms > target_update_timeout_ms) {
            return run_lonely_mode();
        }
    }

    // get_distance_to_destination returns exactly 0.0f when wp is reached
    if (_has_crumb && ModeGuided::get_distance_to_destination() == ZERO_DISTANCE) {
        _has_crumb = false; // we got there
    } else if (_has_crumb) {

        lonely_mode = nullptr;

        // synchronize fetching of current position of target
        // NOTE: This might all be on the main thread...
        if (!_crumbs_sem->take_nonblocking()) {
            return;
        }

        // since rover doesn't use altitude, we need to equate vehicle altitudes
        // else the distance calculation won't be correct
        const Location rover_loc = rover.current_loc;
        target_loc.alt = rover_loc.alt;

        Vector3f to_vehicle = location_3d_diff_NED(rover_loc, target_loc);
        const float distance_to_vehicle = to_vehicle.length();

        _crumbs_sem->give();

        to_vehicle.normalize();

        //maintain at greater than distance_to_stop from target
        if (fabsf(distance_to_vehicle) >= distance_to_stop) {
            //we need to speed up
            to_vehicle *= closure_speed * 100; // m/s to cm/s
            to_vehicle += target_vel;

            const float desired_speed = to_vehicle.length();

            Mode::set_desired_speed(desired_speed);
        } else {
            Mode::set_desired_speed(0.0f);
        }
    }

    ModeGuided::update();
}

void ModeBreadcrumb::mavlink_packet_received(const mavlink_message_t &msg)
{
    if (rover.control_mode != &rover.mode_breadcrumb) {
        return;
    }
    if (msg.sysid != g.sysid_bcmb_target) {
        return;
    }
    if (msg.msgid != MAVLINK_MSG_ID_GLOBAL_POSITION_INT) {
        return;
    }

    mavlink_global_position_int_t packet;
    mavlink_msg_global_position_int_decode(&msg, &packet);

    target_last_update_ms = AP_HAL::millis();

    if (!_crumbs_sem->take_nonblocking()) {
        return;
    }

    //target location and velocity used to dynamically change rover speed
    target_loc.lat = packet.lat;
    target_loc.lng = packet.lon;

    target_vel.x = packet.vx/100.0f; // cm/s to m/s
    target_vel.y = packet.vy/100.0f; // cm/s to m/s
    target_vel.z = 0.0f;

    _crumbs_sem->give();

    Location new_crumb;
    new_crumb.lat = packet.lat;
    new_crumb.lng = packet.lon;
    new_crumb.alt = 0.0f;

    Vector3f to_vehicle = location_3d_diff_NED(_last_crumb_added, new_crumb);

    const float distance_to_last_crum = to_vehicle.length();

    // prune crumbs that are too close to each other
    const bool add_crumb = distance_to_last_crum > distance_to_prune;

    // try to add a crumb
    if (!add_crumb || !_crumbs_ring_buffer->push(new_crumb)) {
        return;
    }

    _last_crumb_added = new_crumb;
}
