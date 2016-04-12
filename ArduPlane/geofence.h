#pragma once

#include <AP_AHRS/AP_AHRS.h>  // for AP_AHRS_NAVEKF_AVAILABLE
#include <AP_Terrain/AP_Terrain.h>

#include <GCS_MAVLink/GCS.h>
#include <GCS_MAVLink/GCS_MAVLink.h>    // MAVLink GCS definitions

#include "config.h"
#include "defines.h"
#include "Parameters.h"

class AP_PolyFence
{

public:

    AP_PolyFence(Parameters &_g) :
        g(_g)
        { }

    uint8_t max_fencepoints();
    Vector2l get_fence_point_with_index(unsigned i);
    void set_fence_point_with_index(Vector2l &point, unsigned i);

    /* Old names to avoid code churn */
    void geofence_load();
    bool geofence_present();
    void geofence_update_pwm_enabled_state();
    bool geofence_set_enabled(bool enable, GeofenceEnableReason r);
    bool geofence_enabled();
    bool geofence_set_floor_enabled(bool floor_enable);
    void geofence_check(bool altitude_check_only);
    bool geofence_breached();
    void geofence_send_status(mavlink_channel_t chan);

    // new names
    void load();
    bool present();
    void update_pwm_enabled_state();
    bool set_enabled(bool enable, GeofenceEnableReason r);
    bool enabled();
    bool set_floor_enabled(bool floor_enable);
    virtual void check(bool altitude_check_only) = 0;
    bool breached();
    void send_status(mavlink_channel_t chan);

protected:

    /*
     *  The state of geo-fencing. This structure is dynamically allocated
     *  the first time it is used. This means we only pay for the pointer
     *  and not the structure on systems where geo-fencing is not being
     *  used.
     *
     *  We store a copy of the boundary in memory as we need to access it
     *  very quickly at runtime
     */
    struct GeofenceState {
        uint8_t num_points;
        bool boundary_uptodate;
        bool fence_triggered;
        bool is_pwm_enabled;          //true if above FENCE_ENABLE_PWM threshold
        bool previous_is_pwm_enabled; //true if above FENCE_ENALBE_PWM threshold
                                      // last time we checked
        bool is_enabled;
        GeofenceEnableReason enable_reason;
        bool floor_enabled;          //typically used for landing
        uint16_t breach_count;
        uint8_t breach_type;
        uint32_t breach_time;
        uint8_t old_switch_position;
        int32_t guided_lat;
        int32_t guided_lng;
        /* point 0 is the return point */
        Vector2l *boundary;
    } *geofence_state;

    /* Start temporary functions to avoid code churn: */
    virtual void gcs_send_text(MAV_SEVERITY severity, const char *str) = 0;
    virtual void gcs_send_message(enum ap_message id) = 0;
    /* end temporary functions to avoid code churn: */

private:

};

class AP_PolyFence_Plane : public AP_PolyFence
{

public:

    AP_PolyFence_Plane(class Plane &plane, Parameters &_g) :
        AP_PolyFence(_g),
        _plane(plane)
        { }

    void check(bool altitude_check_only);
    bool stickmixing();

    bool geofence_stickmixing();

private:

    /* Old names to avoid code churn */
    bool geofence_check_minalt();
    bool geofence_check_maxalt();

    // new names
    bool check_minalt();
    bool check_maxalt();

    /* Start temporary functions to avoid code churn: */
    void gcs_send_text(MAV_SEVERITY severity, const char *str);
    void gcs_send_message(enum ap_message id);
    /* end temporary functions to avoid code churn: */

    class Plane &_plane;
    class Parameters &g;

};
