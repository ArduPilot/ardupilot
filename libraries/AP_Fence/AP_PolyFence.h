#pragma once

#include <stdint.h>

#include <AP_Math/AP_Math.h>
#include <GCS_MAVLink/GCS.h>

// pwm value on FENCE_CHANNEL to use to enable fenced mode
#ifndef FENCE_ENABLE_PWM
 # define FENCE_ENABLE_PWM 1750
#endif

// a digital pin to set high when the geo-fence triggers. Defaults
// to -1, which means don't activate a pin
#ifndef FENCE_TRIGGERED_PIN
 # define FENCE_TRIGGERED_PIN -1
#endif

/*
 * The cause for the most recent fence enable
 */
typedef enum GeofenceEnableReason {
    NOT_ENABLED = 0,     //The fence is not enabled
    PWM_TOGGLED,         //Fence enabled/disabled by PWM signal
    AUTO_TOGGLED,        //Fence auto enabled/disabled at takeoff.
    GCS_TOGGLED          //Fence enabled/disabled by the GCS via Mavlink
} GeofenceEnableReason;

class AP_PolyFence
{

public:

    AP_PolyFence()
        {
            AP_Param::setup_object_defaults(this, var_info);
        }

    uint8_t max_fencepoints();
    Vector2l get_fence_point_with_index(unsigned i);
    void set_fence_point_with_index(Vector2l &point, unsigned i);

    /* Old names to avoid code churn */
    void geofence_load();
    bool geofence_present() const;
    void geofence_update_pwm_enabled_state();
    bool geofence_set_enabled(bool enable, GeofenceEnableReason r);
    bool geofence_enabled();
    bool geofence_set_floor_enabled(bool floor_enable);
    void geofence_check(bool altitude_check_only);
    bool geofence_breached();
    void geofence_send_status(mavlink_channel_t chan);

    // new names
    void load();
    bool present() const;
    void update_pwm_enabled_state();
    bool set_enabled(bool enable, GeofenceEnableReason r);
    bool enabled();
    bool set_floor_enabled(bool floor_enable);
    virtual void check(bool altitude_check_only) = 0;
    bool breached();
    void send_status(mavlink_channel_t chan);

    bool should_revert_flight_mode() const;
    virtual bool guided_destinations_match() const = 0;
    virtual uint8_t oldSwitchPosition() const = 0;
    virtual bool vehicle_in_mode_guided() const = 0;

    static const struct AP_Param::GroupInfo        var_info[];
    struct fence_params {
        AP_Int8 fence_action;
        AP_Int8 fence_total;
        AP_Int8 fence_channel;
        AP_Int16 fence_minalt;    // meters
        AP_Int16 fence_maxalt;    // meters
        AP_Int16 fence_retalt;    // meters
        AP_Int8 fence_autoenable;
        AP_Int8 fence_ret_rally;
    } g;

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

