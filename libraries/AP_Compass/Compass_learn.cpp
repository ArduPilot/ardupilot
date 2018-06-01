#include <AP_Math/AP_Math.h>
#include <AP_AHRS/AP_AHRS.h>

#include <AP_Compass/AP_Compass.h>
#include <AP_Declination/AP_Declination.h>
#include <DataFlash/DataFlash.h>

#include "Compass_learn.h"

#include <stdio.h>

extern const AP_HAL::HAL &hal;

// constructor
CompassLearn::CompassLearn(AP_AHRS &_ahrs, Compass &_compass) :
    ahrs(_ahrs),
    compass(_compass)
{
}

/*
  update when new compass sample available
 */
void CompassLearn::update(void)
{
    if (converged || compass.get_learn_type() != Compass::LEARN_INFLIGHT ||
        !hal.util->get_soft_armed() || ahrs.get_time_flying_ms() < 3000) {
        // only learn when flying and with enough time to be clear of
        // the ground
        return;
    }

    if (!have_earth_field) {
        Location loc;
        if (!ahrs.get_position(loc)) {
            // need to wait till we have a global position
            return;
        }

        // setup the expected earth field at this location
        float declination_deg=0, inclination_deg=0, intensity_gauss=0;
        AP_Declination::get_mag_field_ef(loc.lat*1.0e-7, loc.lng*1.0e-7, intensity_gauss, declination_deg, inclination_deg);

        // create earth field
        mag_ef = Vector3f(intensity_gauss*1000, 0.0, 0.0);
        Matrix3f R;

        R.from_euler(0.0f, -ToRad(inclination_deg), ToRad(declination_deg));
        mag_ef = R * mag_ef;

        sem = hal.util->new_semaphore();

        have_earth_field = true;

        // form eliptical correction matrix and invert it. This is
        // needed to remove the effects of the eliptical correction
        // when calculating new offsets
        const Vector3f &diagonals = compass.get_diagonals(0);
        const Vector3f &offdiagonals = compass.get_offdiagonals(0);
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
        for (uint16_t i=0; i<num_sectors; i++) {
            errors[i] = intensity_gauss*1000;
        }
        
        hal.scheduler->register_io_process(FUNCTOR_BIND_MEMBER(&CompassLearn::io_timer, void));
    }

    if (sample_available) {
        // last sample still being processed by IO thread
        return;
    }

    Vector3f field = compass.get_field(0);
    Vector3f field_change = field - last_field;
    if (field_change.length() < min_field_change) {
        return;
    }
    
    if (sem->take_nonblocking()) {
        // give a sample to the backend to process
        new_sample.field = field;
        new_sample.offsets = compass.get_offsets(0);
        new_sample.attitude = Vector3f(ahrs.roll, ahrs.pitch, ahrs.yaw);
        sample_available = true;
        last_field = field;
        num_samples++;
        sem->give();
    }

    if (sample_available) {
        DataFlash_Class::instance()->Log_Write("COFS", "TimeUS,OfsX,OfsY,OfsZ,Var,Yaw,WVar,N", "QffffffI",
                                               AP_HAL::micros64(),
                                               (double)best_offsets.x,
                                               (double)best_offsets.y,
                                               (double)best_offsets.z,
                                               (double)best_error,
                                               (double)best_yaw_deg,
                                               (double)worst_error,
                                               num_samples);
    }

    if (!converged && sem->take_nonblocking()) {
        // stop updating the offsets once converged
        compass.set_offsets(0, best_offsets);
        if (num_samples > 30 && best_error < 50 && worst_error > 65) {
            // set the offsets and enable compass for EKF use. Let the
            // EKF learn the remaining compass offset error
            compass.save_offsets(0);
            compass.set_use_for_yaw(0, true);
            compass.set_learn_type(Compass::LEARN_EKF, true);
            converged = true;
        }
        sem->give();
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
    if (!sem->take_nonblocking()) {
        return;
    }
    s = new_sample;
    sample_available = false;
    sem->give();

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
            const float learn_rate = 0.92;
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

    if (sem->take_nonblocking()) {
        // pass the current estimate to the front-end
        best_offsets = predicted_offsets[besti];
        best_error = bestv;
        worst_error = worstv;
        best_yaw_deg = wrap_360(degrees(s.attitude.z) + besti * (360/num_sectors));
        sem->give();
    }
}
