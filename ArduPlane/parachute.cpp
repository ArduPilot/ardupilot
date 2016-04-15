// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

#include "Plane.h"

#define PARACHUTE_CHECK_TRIGGER_MS         1000  // 1 second of loss of control triggers the parachute


/*
   call parachute library update
*/
void Plane::parachute_check()
{
    parachute.update();

    // check if there is an emergency situation
    parachute_emergency_check();
}

/*
  parachute_release - trigger the release of the parachute
*/
void Plane::parachute_release()
{
    if (parachute.released()) {
        return;
    }

    // send message to gcs and dataflash
    gcs_send_text(MAV_SEVERITY_CRITICAL,"Parachute: Released");

    // release parachute
    parachute.release();
}

/*
  parachute_manual_release - trigger the release of the parachute,
  after performing some checks for pilot error checks if the vehicle
  is landed
*/
bool Plane::parachute_manual_release()
{
    // exit immediately if parachute is not enabled
    if (!parachute.enabled() || parachute.released()) {
        return false;
    }

    // do not release if vehicle is not flying
    if (!is_flying()) {
        // warn user of reason for failure
        gcs_send_text(MAV_SEVERITY_WARNING,"Parachute: Not flying");
        return false;
    }

    if (relative_altitude() < parachute.alt_min()) {
        gcs_send_text_fmt(MAV_SEVERITY_WARNING, "Parachute: Too low");
        return false;
    }

    // if we get this far release parachute
    parachute_release();

    return true;
}

/*
  parachute_emergency_check - trigger the release of the parachute
  automatically if a critical situation is detected
*/
void Plane::parachute_emergency_check()
{
    uint32_t now = millis();

    // exit immediately if parachute or automatic release is not enabled, or already released
    if (!parachute.auto_enabled() || parachute.released()) {
        parachute.control_loss_ms(0);
        return;
    }

    // only release in AUTO mode, do not release if taking off or landing
    if (control_mode != AUTO ||
        auto_state.takeoff_complete == false ||
        auto_state.land_complete == true ||
        mission.get_current_nav_cmd().id == MAV_CMD_NAV_LAND) {

        parachute.control_loss_ms(0);
        return;
    }

    // do not release if critical values for both altitude and sink rate are not reached
    if ((parachute.auto_sink() < 0 || auto_state.sink_rate < parachute.auto_sink()) &&
        relative_altitude() > parachute.auto_alt()) {

        parachute.control_loss_ms(0);
        return;
    }

    // at this point we consider control lost
    if (parachute.control_loss_ms() == 0) {
        gcs_send_text_fmt(MAV_SEVERITY_WARNING, "Crash: Starting to lose control, %d m error", altitude_error_cm * 0.01f);
        parachute.control_loss_ms(now);
    } else if (now - parachute.control_loss_ms() > PARACHUTE_CHECK_TRIGGER_MS) {
        // we have lost control for too long
        gcs_send_text_fmt(MAV_SEVERITY_WARNING, "Crash: Critical altitude error %d m", altitude_error_cm * 0.01f);

        // check if altitude is suitable for parachute deployment
        if (relative_altitude() > parachute.alt_min() && (parachute.alt_max() < 0 || relative_altitude() < parachute.alt_max())) {
            parachute_release();
        }
        parachute.control_loss_ms(0);
    }
}
