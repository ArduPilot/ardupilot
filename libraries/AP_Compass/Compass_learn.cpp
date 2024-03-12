#include <AP_AHRS/AP_AHRS.h>

#include <AP_Compass/AP_Compass.h>

#include "Compass_learn.h"
#include <GCS_MAVLink/GCS.h>
#include <AP_Vehicle/AP_Vehicle.h>
#include <AP_NavEKF/EKFGSF_yaw.h>

#if COMPASS_LEARN_ENABLED

#include <AP_Logger/AP_Logger.h>

extern const AP_HAL::HAL &hal;

// constructor
CompassLearn::CompassLearn(Compass &_compass) :
    compass(_compass)
{
    GCS_SEND_TEXT(MAV_SEVERITY_INFO, "CompassLearn: Initialised");
}

// accuracy threshold applied for GSF yaw estimate
#define YAW_ACCURACY_THRESHOLD_DEG 5.0

/*
  update when new compass sample available
 */
void CompassLearn::update(void)
{
    const AP_Vehicle *vehicle = AP::vehicle();
    if (compass.get_learn_type() != Compass::LEARN_INFLIGHT ||
        !hal.util->get_soft_armed() ||
        vehicle->get_time_flying_ms() < 3000) {
        // only learn when flying and with enough time to be clear of
        // the ground
        return;
    }

    const auto &ahrs = AP::ahrs();
    const auto *gsf = ahrs.get_yaw_estimator();
    if (gsf == nullptr) {
        // no GSF available
        return;
    }
    if (degrees(fabsf(ahrs.get_pitch())) > 50) {
        // we don't want to be too close to nose up, or yaw gets
        // problematic. Tailsitters need to wait till they are in
        // forward flight
        return;
    }

    AP_Notify::flags.compass_cal_running = true;

    ftype yaw_rad, yaw_variance;
    uint8_t n_clips;
    if (!gsf->getYawData(yaw_rad, yaw_variance, &n_clips) ||
        !is_positive(yaw_variance) ||
        n_clips > 1 ||
        yaw_variance >= sq(radians(YAW_ACCURACY_THRESHOLD_DEG))) {
        // not converged
        return;
    }

    const bool result = compass.mag_cal_fixed_yaw(degrees(yaw_rad), (1U<<HAL_COMPASS_MAX_SENSORS)-1, 0, 0, true);
    if (result) {
        AP_Notify::flags.compass_cal_running = false;
        compass.set_learn_type(Compass::LEARN_NONE, true);
        GCS_SEND_TEXT(MAV_SEVERITY_INFO, "CompassLearn: Finished");
    }
}

#endif // COMPASS_LEARN_ENABLED
