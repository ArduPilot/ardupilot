#pragma once

#include "SIM_config.h"

#if AP_SIM_MAVLINKGIMBALV2_ENABLED

#include <AP_Common/Location.h>
#include <AP_Math/AP_Math.h>
#include <GCS_MAVLink/GCS_MAVLink.h>

namespace SITL
{

// Deliberately has no access to Aircraft or SITL truth.  The only inputs are
// telemetry received by the gimbal and its three simulated joint encoders.
class MAVLinkGimbalv2_Estimator
{
public:
    void handle_position(const mavlink_global_position_int_t &packet, uint32_t now_ms);
    void handle_attitude(const mavlink_autopilot_state_for_gimbal_device_t &packet, uint32_t now_ms);
    bool position_valid(uint32_t now_ms) const;
    bool attitude_valid(uint32_t now_ms) const;
    bool get_location(uint32_t now_ms, Location &location) const;
    bool get_attitude(uint32_t now_ms, const Vector3f &encoders,
                      Matrix3f &vehicle_dcm, Matrix3f &gimbal_dcm) const;
    const Vector3f &vehicle_rates() const
    {
        return _vehicle_rates;
    }

private:
    static constexpr uint32_t TIMEOUT_MS = 1000;
    bool _have_position = false;
    bool _have_attitude = false;
    uint32_t _position_ms = 0;
    uint32_t _attitude_ms = 0;
    uint32_t _attitude_sample_us = 0;
    uint16_t _estimator_status = 0;
    Location _location;
    Vector3f _velocity;
    Quaternion _attitude;
    Vector3f _vehicle_rates;
};

} // namespace SITL

#endif // AP_SIM_MAVLINKGIMBALV2_ENABLED
