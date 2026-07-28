#include <AP_AHRS/AP_AHRS.h>

#include <AP_Compass/AP_Compass.h>

#include "Compass_learn.h"
#include <GCS_MAVLink/GCS.h>
#include <AP_Vehicle/AP_Vehicle.h>
#include <AP_NavEKF/EKFGSF_yaw.h>

#include <AP_Logger/AP_Logger.h>

#if COMPASS_LEARN_ENABLED

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
    if (compass.get_learn_type() != Compass::LearnType::INFLIGHT ||
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
    if (fabsf(ahrs.get_pitch_deg()) > 50) {
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
        compass.set_learn_type(Compass::LearnType::NONE, true);
        GCS_SEND_TEXT(MAV_SEVERITY_INFO, "CompassLearn: Finished");
    }
}

#endif  // COMPASS_LEARN_ENABLED

#if AP_COMPASS_LEARN_COPY_FROM_EKF_ENABLED
/*
  save any compass offsets the EKF has learned.  Called on disarm.
 */
void Compass::save_ekf_learned_offsets()
{
    if (get_learn_type() != LearnType::COPY_FROM_EKF) {
        return;
    }

    auto &ahrs = AP::ahrs();
    if (!ahrs.healthy()) {
        // Note that this is a deliberate tightening rather than part of
        // moving the vehicles' code here: Copter, Sub and Blimp each
        // asked getMagOffsets() directly, with no health precondition.
        // getMagOffsets() judges whether an individual estimate is
        // usable, but it cannot tell that the estimator as a whole is
        // unhealthy or that the active backend is no longer the
        // configured one - and offsets are set_and_save()d, so a bad
        // set persists across a reboot and has to be recalibrated out
        // by hand.  Declining to copy costs a learning opportunity; the
        // vehicle keeps the offsets it already had.
        return;
    }
    if (!ahrs.use_compass()) {
        return;
    }

    // note that this loop is not redundant even though a single EKF
    // core will only ever return offsets for its own selected
    // compass; with EK3_AFFINITY compass affinity enabled each core
    // takes a different compass (AP_NavEKF3_Measurements.cpp
    // update_mag_selection) and the frontend asks every core for each
    // instance in turn (AP_NavEKF3.cpp getMagOffsets).
    uint8_t saved_count = 0;
    for (uint8_t i=0; i<COMPASS_MAX_INSTANCES; i++) {
        Vector3f magOffsets;
        if (ahrs.getMagOffsets(i, magOffsets)) {
            set_and_save_offsets(i, magOffsets);
            saved_count++;
        }
    }

#if HAL_LOGGING_ENABLED
    if (saved_count != 0) {
        // the EKF frequently has nothing to offer here, so log the fact
        // that we did save something; otherwise a failure to learn is
        // indistinguishable from the feature not being enabled
        AP::logger().Write_Event(LogEvent::EKF_MAG_OFFSETS_SAVED);
    }
#endif  // HAL_LOGGING_ENABLED
}
#endif  // AP_COMPASS_LEARN_COPY_FROM_EKF_ENABLED
