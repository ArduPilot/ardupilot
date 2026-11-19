#include "Copter.h"

#if MODE_GUIDED_ENABLED && AP_EXTERNAL_BINDING_ENABLED
// constructor registers custom number and names
ModeGuidedCustom::ModeGuidedCustom(const Number _number, const char* _full_name, const char* _short_name):
    number(_number),
    full_name(_full_name),
    short_name(_short_name)
{
}

// running custom mode, nullptr if the current mode is not a custom mode
ModeGuidedCustom *Copter::active_custom_mode()
{
    for (auto *custom : mode_guided_custom) {
        if (custom != nullptr && flightmode == custom) {
            return custom;
        }
    }
    return nullptr;
}

// guided target for the custom mode owning state, only while it is the
// current mode, so a mode change cannot redirect the target
bool Copter::custom_mode_set_target_posvelaccel_NED(const custom_mode_state *state, const Vector3f& target_pos_ned_m, const Vector3f& target_vel_ned_ms, const Vector3f& target_accel_ned_mss, bool use_yaw, float yaw_deg, bool use_yaw_rate, float yaw_rate_degs, bool yaw_relative)
{
    ModeGuidedCustom *custom = active_custom_mode();
    if (custom == nullptr || &custom->state != state || !custom->accepts_targets(false)) {
        return false;
    }
    return custom->set_pos_vel_accel_NED_m(target_pos_ned_m.topostype(), target_vel_ned_ms, target_accel_ned_mss, use_yaw, radians(yaw_deg), use_yaw_rate, radians(yaw_rate_degs), yaw_relative);
}

// a live owner asked for EKF failsafe disarm, applied in every flight mode
bool Copter::custom_mode_ekf_failsafe_disarm() const
{
    for (const auto *custom : mode_guided_custom) {
        if (custom != nullptr && custom->ekf_failsafe_disarm()) {
            return true;
        }
    }
    return false;
}

bool ModeGuidedCustom::init(bool ignore_checks)
{
    // Stript can block entry
    if (!state.allow_entry) {
        return false;
    }

    // a lost owner blocks re-entry
    if (owner_expired()) {
        return false;
    }

    // Guided entry checks must also pass
    if (!ModeGuided::init(ignore_checks)) {
        return false;
    }

    // requests made before entry are never applied
    controller = Request::GUIDED;
    state.applied_generation = state.generation;
    state.init_ok = false;
    state.complete = false;
    state.active = true;
    return true;
}

void ModeGuidedCustom::exit()
{
    // the incoming mode is already initialised and may own the delegate
    // controllers, so only invalidate our own state
    state.active = false;
    state.applied_generation = state.generation;
    state.init_ok = false;
    state.complete = false;
    state.takeoff_event = false;
    state.arm_request = false;
    controller = Request::GUIDED;
}

// called on registration by a new owner
void ModeGuidedCustom::registered()
{
    // a lost owner latch only clears while disarmed
    if (state.owner_lost && motors->armed()) {
        return;
    }
    state.owner_lost = false;
    state.owner_loss_event = false;
    state.options = 0;
    state.arming_ready = false;
    state.supervision_ms = 0;
    state.takeoff_event = false;
    state.arm_request = false;
}

bool ModeGuidedCustom::owner_expired() const
{
    return state.owner_lost ||
           (state.supervision_ms != 0 &&
            (state.owner_loss_event || AP_HAL::millis() - state.keepalive_ms > state.supervision_ms));
}

// latch owner loss once: drop requests and land, or disarm if landed
void ModeGuidedCustom::supervise()
{
    if (state.owner_lost || !owner_expired()) {
        return;
    }
    state.owner_lost = true;
    state.takeoff_event = false;
    state.arm_request = false;
    state.applied_generation = state.generation;
    state.complete = false;
    if (!motors->armed()) {
        gcs().send_text(MAV_SEVERITY_CRITICAL, "%s: owner lost", name());
        return;
    }
    if (copter.ap.land_complete) {
        gcs().send_text(MAV_SEVERITY_CRITICAL, "%s: owner lost, disarming", name());
        copter.arming.disarm(AP_Arming::Method::LANDED);
        return;
    }
    gcs().send_text(MAV_SEVERITY_CRITICAL, "%s: owner lost, landing", name());
    state.init_ok = start_controller(Request::LAND);
}

void ModeGuidedCustom::run()
{
    supervise();

    // arm on the main thread: arming may delay while the log opens
    if (state.arm_request) {
        state.arm_request = false;
        if (!motors->armed() && state.owner != nullptr && !state.owner_lost) {
            AP::arming().arm(AP_Arming::Method::SCRIPTING);
        }
    }

    if (state.generation != state.applied_generation) {
        state.applied_generation = state.generation;
        state.complete = false;
        state.init_ok = start_controller(state.request);
    }

    switch (controller) {
    case Request::TAKEOFF:
        auto_takeoff.run();
        break;
    case Request::LAND:
        copter.mode_land.run();
        break;
#if MODE_RTL_ENABLED
    case Request::RTL:
        copter.mode_rtl.run(false);
        break;
#endif
#if MODE_LOITER_ENABLED
    case Request::LOITER:
        copter.mode_loiter.run();
        break;
#endif
    default:
        ModeGuided::run();
        break;
    }

    // delegates may have changed mode, e.g. RTL landing escape
    if (copter.flightmode == this && state.init_ok && !state.complete && controller_complete()) {
        state.complete = true;
    }
}

// initialise the delegate for a request, keeping the current one on failure
bool ModeGuidedCustom::start_controller(Request request)
{
    bool ok = false;
    switch (request) {
    case Request::GUIDED:
        ok = ModeGuided::init(true);
        break;
    case Request::TAKEOFF:
        ok = takeoff_start();
        break;
    case Request::LAND:
        ok = copter.mode_land.init(true);
        break;
#if MODE_RTL_ENABLED
    case Request::RTL:
        ok = copter.mode_rtl.init(true);
        break;
#endif
#if MODE_LOITER_ENABLED
    case Request::LOITER:
        ok = copter.mode_loiter.init(true);
        break;
#endif
    default:
        break;
    }
    if (ok) {
        controller = request;
    }
    return ok;
}

// climb takeoff_alt_m above the current altitude holding the armed heading
bool ModeGuidedCustom::takeoff_start()
{
    float alt_m;
    if (!motors->armed() || !is_positive(state.takeoff_alt_m) ||
        !copter.current_loc.get_alt_m(Location::AltFrame::ABOVE_ORIGIN, alt_m)) {
        return false;
    }
    pos_control->D_init_controller();
    auto_takeoff.start_m(alt_m + state.takeoff_alt_m, false);
    attitude_control->reset_yaw_target_and_rate();
    attitude_control->reset_rate_controller_I_terms();
    auto_yaw.set_yaw_angle_and_rate_rad(copter.initial_armed_bearing_rad, 0);
    auto_yaw.set_fixed_yaw_rad(copter.initial_armed_bearing_rad, 0, 0, false);
    copter.set_auto_armed(true);
    copter.set_land_complete(false);
    return true;
}

bool ModeGuidedCustom::controller_complete() const
{
    const auto spool = motors->get_spool_state();
    switch (controller) {
    case Request::TAKEOFF:
        return auto_takeoff.complete;
    case Request::LAND:
        return copter.ap.land_complete &&
               (spool == AP_Motors::SpoolState::GROUND_IDLE || spool == AP_Motors::SpoolState::SHUT_DOWN);
#if MODE_RTL_ENABLED
    case Request::RTL:
        return copter.mode_rtl.is_complete() && spool == AP_Motors::SpoolState::GROUND_IDLE;
#endif
    default:
        return false;
    }
}

bool ModeGuidedCustom::accepts_targets(bool external) const
{
    if (state.owner_lost || (external && option_set(Option::NO_EXTERNAL_GUIDED))) {
        return false;
    }
    // no owner requests, or an applied GUIDED request
    return state.generation == 0 ||
           (controller == Request::GUIDED && state.init_ok && state.applied_generation == state.generation);
}

bool ModeGuidedCustom::gcs_takeoff(MAV_RESULT &result)
{
    if (!option_set(Option::NO_EXTERNAL_GUIDED)) {
        return false;
    }
    // the owner performs the takeoff
    const bool ok = state.owner != nullptr && !state.owner_lost && motors->armed() && copter.ap.land_complete;
    state.takeoff_event |= ok;
    result = ok ? MAV_RESULT_ACCEPTED : MAV_RESULT_FAILED;
    return true;
}

bool ModeGuidedCustom::allows_arming(AP_Arming::Method method) const
{
    if (state.owner_lost) {
        return false;
    }
    if (option_set(Option::ARMING_GATED)) {
        return method == AP_Arming::Method::MAVLINK || state.arming_ready;
    }
    return ModeGuided::allows_arming(method);
}

bool ModeGuidedCustom::move_vehicle_on_ekf_reset() const
{
    return controller == Request::GUIDED && ModeGuided::move_vehicle_on_ekf_reset();
}

bool ModeGuidedCustom::is_landing() const
{
    switch (controller) {
    case Request::LAND:
        return true;
#if MODE_RTL_ENABLED
    case Request::RTL:
        return copter.mode_rtl.is_landing();
#endif
    default:
        return false;
    }
}

bool ModeGuidedCustom::is_taking_off() const
{
    switch (controller) {
    case Request::GUIDED:
        return ModeGuided::is_taking_off();
    case Request::TAKEOFF:
        return !auto_takeoff.complete;
    default:
        return false;
    }
}

bool ModeGuidedCustom::get_nav_target(NavTarget &target) const
{
    switch (controller) {
    case Request::GUIDED:
        return ModeGuided::get_nav_target(target);
#if MODE_RTL_ENABLED
    case Request::RTL:
        return copter.mode_rtl.get_nav_target(target);
#endif
    default:
        return false;
    }
}

float ModeGuidedCustom::wp_distance_m() const
{
    if (controller == Request::GUIDED) {
        return ModeGuided::wp_distance_m();
    }
    Location loc;
    return get_wp(loc) ? copter.current_loc.get_distance(loc) : 0;
}

float ModeGuidedCustom::wp_bearing_deg() const
{
    if (controller == Request::GUIDED) {
        return ModeGuided::wp_bearing_deg();
    }
    Location loc;
    return get_wp(loc) ? degrees(copter.current_loc.get_bearing(loc)) : 0;
}

float ModeGuidedCustom::crosstrack_error_m() const
{
    return controller == Request::GUIDED ? ModeGuided::crosstrack_error_m() : 0;
}

bool ModeGuidedCustom::do_user_takeoff_start_m(float takeoff_alt_m)
{
    return accepts_targets(false) && ModeGuided::do_user_takeoff_start_m(takeoff_alt_m);
}

bool ModeGuidedCustom::pause()
{
    return accepts_targets(true) && ModeGuided::pause();
}

bool ModeGuidedCustom::resume()
{
    return accepts_targets(true) && ModeGuided::resume();
}

#endif
