#include <AP_Math/AP_Math.h>
#include <AP_AHRS/AP_AHRS.h>

#include <AP_Compass/AP_Compass.h>
#include <AP_Declination/AP_Declination.h>
#include <AP_Logger/AP_Logger.h>

#include "Compass_learn.h"
#include <GCS_MAVLink/GCS.h>

#include <stdio.h>
#include <AP_Vehicle/AP_Vehicle.h>

#if COMPASS_LEARN_ENABLED

extern const AP_HAL::HAL &hal;

// constructor
CompassLearn::CompassLearn(Compass &_compass) :
    compass(_compass)
{
    gcs().send_text(MAV_SEVERITY_INFO, "CompassLearn: Initialised");
    for (uint8_t i=0; i<compass.get_count(); i++) {
        if (compass._state[i].use_for_yaw) {
            // reset scale factors, we can't learn scale factors in
            // flight
            compass.set_and_save_scale_factor(i, 0.0);
        }
    }
}

/*
  update when new compass sample available
 */
void CompassLearn::update(void)
{
    const AP_Vehicle *vehicle = AP::vehicle();
    if (converged || compass.get_learn_type() != Compass::LEARN_INFLIGHT ||
        !hal.util->get_soft_armed() || vehicle->get_time_flying_ms() < 3000) {
        // only learn when flying and with enough time to be clear of
        // the ground
        return;
    }

    const AP_AHRS &ahrs = AP::ahrs();
    if (!have_earth_field) {
        Location loc;
        if (!ahrs.get_position(loc)) {
            // need to wait till we have a global position
            return;
        }

        // remember primary mag
        primary_mag = compass.get_primary();

        // setup the expected earth field in mGauss at this location
        mag_ef = AP_Declination::get_earth_field_ga(loc) * 1000;
        have_earth_field = true;

        // form eliptical correction matrix and invert it. This is
        // needed to remove the effects of the eliptical correction
        // when calculating new offsets
        const Vector3f &diagonals = compass.get_diagonals(primary_mag);
        const Vector3f &offdiagonals = compass.get_offdiagonals(primary_mag);
        mat = Matrix3f(
            diagonals.x, offdiagonals.x, offdiagonals.y,
            offdiagonals.x,    diagonals.y, offdiagonals.z,
            offdiagonals.y, offdiagonals.z,    diagonals.z
            );
        if (!mat.invert()) {
            // if we can't invert, use the identity matrix
            mat.identity();
        }

        // set initial error to field intensity
        float intensity = mag_ef.length();
        for (uint16_t i=0; i<num_sectors; i++) {
            errors[i] = intensity;
        }
        
        gcs().send_text(MAV_SEVERITY_INFO, "CompassLearn: have earth field");
        hal.scheduler->register_io_process(FUNCTOR_BIND_MEMBER(&CompassLearn::io_timer, void));
    }

    AP_Notify::flags.compass_cal_running = true;

    if (sample_available) {
        // last sample still being processed by IO thread
        return;
    }

    Vector3f field = compass.get_field(primary_mag);
    Vector3f field_change = field - last_field;
    if (field_change.length() < min_field_change) {
        return;
    }

    {
        WITH_SEMAPHORE(sem);
        // give a sample to the backend to process
        new_sample.field = field;
        new_sample.offsets = compass.get_offsets(primary_mag);
        new_sample.attitude = Vector3f(ahrs.roll, ahrs.pitch, ahrs.yaw);
        sample_available = true;
        last_field = field;
        num_samples++;
    }

    if (sample_available) {
        AP::logger().Write("COFS", "TimeUS,OfsX,OfsY,OfsZ,Var,Yaw,WVar,N", "QffffffI",
                                               AP_HAL::micros64(),
                                               (double)best_offsets.x,
                                               (double)best_offsets.y,
                                               (double)best_offsets.z,
                                               (double)best_error,
                                               (double)best_yaw_deg,
                                               (double)worst_error,
                                               num_samples);
    }

    if (!converged) {
        WITH_SEMAPHORE(sem);

        // set offsets to current best guess
        compass.set_offsets(primary_mag, best_offsets);

        // set non-primary offsets to match primary
        Vector3f field_primary = compass.get_field(primary_mag);
        for (uint8_t i=0; i<compass.get_count(); i++) {
            if (i == primary_mag || !compass._state[i].use_for_yaw) {
                continue;
            }
            Vector3f field2 = compass.get_field(i);
            Vector3f new_offsets = compass.get_offsets(i) + (field_primary - field2);
            compass.set_offsets(i, new_offsets);
        }

        // stop updating the offsets once converged
        if (num_samples > 30 && best_error < 50 && worst_error > 65) {
            // set the offsets and enable compass for EKF use. Let the
            // EKF learn the remaining compass offset error
            for (uint8_t i=0; i<compass.get_count(); i++) {
                if (compass._state[i].use_for_yaw) {
                    compass.save_offsets(i);
                    compass.set_and_save_scale_factor(i, 0.0);
                    compass.set_use_for_yaw(i, true);
                }
            }
            compass.set_learn_type(Compass::LEARN_NONE, true);
            // setup so use can trigger it again
            converged = false;
            sample_available = false;
            num_samples = 0;
            have_earth_field = false;
            memset(predicted_offsets, 0, sizeof(predicted_offsets));
            worst_error = 0;
            best_error = 0;
            best_yaw_deg = 0;
            best_offsets.zero();
            gcs().send_text(MAV_SEVERITY_INFO, "CompassLearn: finished");
            AP_Notify::flags.compass_cal_running = false;
            AP_Notify::events.compass_cal_saved = true;
        }
    }
}

/*
  we run the math intensive calculations in the IO thread
 */
void CompassLearn::io_timer(void)
{
    if (!sample_available) {
        return;
    }

    struct sample s;

    {
        WITH_SEMAPHORE(sem);
        s = new_sample;
        sample_available = false;
    }

    process_sample(s);
}

/*
  process a new compass sample
 */
void CompassLearn::process_sample(const struct sample &s)
{
    uint16_t besti = 0;
    float bestv = 0, worstv=0;

    /*
      we run through the 72 possible yaw error values, and for each
      one we calculate a value for the compass offsets if that yaw
      error is correct. 
     */
    for (uint16_t i=0; i<num_sectors; i++) {
        float yaw_err_deg = i*(360/num_sectors);

        // form rotation matrix for the euler attitude
        Matrix3f dcm;
        dcm.from_euler(s.attitude.x, s.attitude.y, wrap_2PI(s.attitude.z + radians(yaw_err_deg)));

        // calculate the field we would expect to get if this yaw error is correct
        Vector3f expected_field = dcm.transposed() * mag_ef;

        // calculate a value for the compass offsets for this yaw error
        Vector3f v1 = mat * s.field;
        Vector3f v2 = mat * expected_field;
        Vector3f offsets = (v2 - v1) + s.offsets;
        float delta = (offsets - predicted_offsets[i]).length();

        if (num_samples == 1) {
            predicted_offsets[i] = offsets;
        } else {
            // lowpass the predicted offsets and the error
            const float learn_rate = 0.92f;
            predicted_offsets[i] = predicted_offsets[i] * learn_rate + offsets * (1-learn_rate);
            errors[i] = errors[i] * learn_rate + delta * (1-learn_rate);
        }

        // keep track of the current best prediction
        if (i == 0 || errors[i] < bestv) {
            besti = i;
            bestv = errors[i];
        }
        // also keep the worst error. This is used as part of the convergence test
        if (i == 0 || errors[i] > worstv) {
            worstv = errors[i];
        }
    }

    WITH_SEMAPHORE(sem);

    // pass the current estimate to the front-end
    best_offsets = predicted_offsets[besti];
    best_error = bestv;
    worst_error = worstv;
    best_yaw_deg = wrap_360(degrees(s.attitude.z) + besti * (360/num_sectors));
}

#endif // COMPASS_LEARN_ENABLED

