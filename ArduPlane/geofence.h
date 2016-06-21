#pragma once

#include <AP_Fence/AP_PolyFence.h>

class AP_PolyFence_Plane : public AP_PolyFence
{

public:

    AP_PolyFence_Plane(class Plane &plane) :
        AP_PolyFence(),
        _plane(plane)
        {
            AP_Param::setup_object_defaults(this, var_info_plane);
        }

    bool stickmixing();

    bool geofence_stickmixing();

private:

    /* Start temporary functions to avoid code churn: */
    void gcs_send_text(MAV_SEVERITY severity, const char *str);
    void gcs_send_message(enum ap_message id);
    /* end temporary functions to avoid code churn: */

    class Plane &_plane;

    static const struct AP_Param::GroupInfo        var_info_plane[];

    uint8_t oldSwitchPosition() const override;
    bool guided_destinations_match() const override;
    bool vehicle_in_mode_guided() const override;
    bool vehicle_in_mode_rtl() const override;
    bool breaches_inhibited() const override;
    void revert_mode() override;
    int32_t vehicle_relative_alt_cm() const override;
    bool vehicle_position(Location &loc) const override;
    void do_breach_action_guided() override;

};
