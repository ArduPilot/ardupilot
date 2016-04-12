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

    static const struct AP_Param::GroupInfo        var_info_plane[];
};
