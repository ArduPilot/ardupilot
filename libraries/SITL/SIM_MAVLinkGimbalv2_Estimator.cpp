#include "SIM_MAVLinkGimbalv2_Estimator.h"

#if AP_SIM_MAVLINKGIMBALV2_ENABLED

namespace SITL
{

void MAVLinkGimbalv2_Estimator::handle_position(const mavlink_global_position_int_t &packet, uint32_t now_ms)
{
    _have_position = check_latlng(packet.lat, packet.lon);
    if (!_have_position) {
        return;
    }
    _location.lat = packet.lat;
    _location.lng = packet.lon;
    _location.set_alt_cm(packet.alt / 10, Location::AltFrame::ABSOLUTE);
    _velocity = Vector3f(packet.vx, packet.vy, packet.vz) * 0.01f;
    _position_ms = now_ms;
}

void MAVLinkGimbalv2_Estimator::handle_attitude(const mavlink_autopilot_state_for_gimbal_device_t &packet, uint32_t now_ms)
{
    Quaternion attitude(packet.q[0], packet.q[1], packet.q[2], packet.q[3]);
    const float length = attitude.length();
    if (!isfinite(length) || !is_positive(length) ||
        (packet.estimator_status & ESTIMATOR_ATTITUDE) == 0) {
        _have_attitude = false;
        return;
    }
    attitude.normalize();
    // The sender's sample clock avoids interpreting link jitter as rotation.
    // Use the low 32 bits to also support senders with a wrapping micros clock.
    const uint32_t sample_us = uint32_t(packet.time_boot_us - packet.q_estimated_delay_us);
    const uint32_t dt_us = sample_us - _attitude_sample_us;
    _vehicle_rates.zero();
    if (attitude_valid(now_ms) && dt_us > 0 && dt_us <= TIMEOUT_MS * 1000) {
        (_attitude.inverse() * attitude).to_axis_angle(_vehicle_rates);
        _vehicle_rates *= 1.0e6f / dt_us;
    }
    _attitude = attitude;
    _attitude_sample_us = sample_us;
    _attitude_ms = now_ms;
    _estimator_status = packet.estimator_status;
    _have_attitude = true;
}

bool MAVLinkGimbalv2_Estimator::attitude_valid(uint32_t now_ms) const
{
    return _have_attitude && now_ms - _attitude_ms <= TIMEOUT_MS;
}

bool MAVLinkGimbalv2_Estimator::position_valid(uint32_t now_ms) const
{
    const uint16_t required = ESTIMATOR_POS_HORIZ_ABS | ESTIMATOR_POS_VERT_ABS |
                              ESTIMATOR_VELOCITY_HORIZ | ESTIMATOR_VELOCITY_VERT;
    return _have_position && now_ms - _position_ms <= TIMEOUT_MS &&
           attitude_valid(now_ms) && (_estimator_status & required) == required;
}

bool MAVLinkGimbalv2_Estimator::get_location(uint32_t now_ms, Location &location) const
{
    if (!position_valid(now_ms)) {
        return false;
    }
    const float dt = (now_ms - _position_ms) * 0.001f;
    location = _location;
    location.offset(_velocity.x * dt, _velocity.y * dt);
    // GLOBAL_POSITION_INT altitude is MSL-up, while its velocity is NED.
    location.alt -= _velocity.z * dt * 100;
    return true;
}

bool MAVLinkGimbalv2_Estimator::get_attitude(uint32_t now_ms, const Vector3f &encoders,
        Matrix3f &vehicle_dcm, Matrix3f &gimbal_dcm) const
{
    if (!attitude_valid(now_ms)) {
        return false;
    }
    Quaternion attitude = _attitude;
    attitude.rotate(_vehicle_rates * ((now_ms - _attitude_ms) * 0.001f));
    attitude.rotation_matrix(vehicle_dcm);
    Matrix3f encoder_dcm;
    encoder_dcm.from_euler312(encoders.x, encoders.y, encoders.z);
    gimbal_dcm = vehicle_dcm * encoder_dcm;
    return true;
}

} // namespace SITL

#endif // AP_SIM_MAVLINKGIMBALV2_ENABLED
