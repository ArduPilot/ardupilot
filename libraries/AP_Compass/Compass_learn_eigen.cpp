#ifdef EIGEN_MATH

/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-
#include "Compass.h"
#include <math.h>
#include <AP_Eigen/AP_Eigen.h>

// don't allow any axis of the offset to go above 2000
#define COMPASS_OFS_LIMIT 2000

/*
 *  this offset learning algorithm is inspired by this paper from Bill Premerlani
 *  and rewritten as an example for Eigen by Daniel Frenzel
 *
 *  http://gentlenav.googlecode.com/files/MagnetometerOffsetNullingRevisited.pdf
 *
 *  The base algorithm works well, but is quite sensitive to
 *  noise. After long discussions with Bill, the following changes were
 *  made:
 *
 *   1) we keep a history buffer that effectively divides the mag
 *      vectors into a set of N streams. The algorithm is run on the
 *      streams separately
 *
 *   2) within each stream we only calculate a change when the mag
 *      vector has changed by a significant amount.
 *
 *  This gives us the property that we learn quickly if there is no
 *  noise, but still learn correctly (and slowly) in the face of lots of
 *  noise.
 */
void Compass::learn_offsets() {
    if (_learn == 0) {
        // auto-calibration is disabled
        return;
    }

    // this gain is set so we converge on the offsets in about 5
    // minutes with a 10Hz compass
    const float gain = 0.01f;
    const float max_change = 10.0f;
    const float min_diff = 50.0f;

    if (!_null_init_done) {
        // first time through
        _null_init_done = true;
        for (uint8_t k=0; k<COMPASS_MAX_INSTANCES; k++) {
            const AP_Eigen::Vector3f &field = _state[k].field;
            const AP_Eigen::Vector3f &ofs = _state[k].offset.get();
            for (uint8_t i=0; i<_mag_history_size; i++) {
              // fill the history buffer with the current mag vector, with the offset removed
              _state[k].mag_history[i] = AP_Eigen::Vector3f::get_nearest_integral<int>((AP_Eigen::Vector3f)(field - ofs));
            }
            _state[k].mag_history_index = 0;
        }
        return;
    }

    for (uint8_t k=0; k<COMPASS_MAX_INSTANCES; k++) {
        AP_Eigen::Vector3f ofs   = _state[k].offset.get();
        AP_Eigen::Vector3f field = _state[k].field;

        if (ofs.is_nan()) {
            // offsets are bad possibly due to a past bug - zero them
            _state[k].offset.set(Vector3f());
        }

        // get a past element
        AP_Eigen::Vector3f v3f_hist = _state[k].mag_history[_state[k].mag_history_index];
        // the history buffer doesn't have the offsets
        v3f_hist += ofs;

        // calculate the delta for this sample
        AP_Eigen::Vector3f v3f_diff = field - v3f_hist;
        float v3f_diff_len = v3f_diff.length();
        if (v3f_diff_len < min_diff) {
            // the mag vector hasn't changed enough - we don't get enough information from this vector to use it.
            // Note that we don't put the current vector into the mag history here. 
            // We want to wait for a larger rotation to build up before calculating an offset change, as accuracy
            // of the offset change is highly dependent on the size of the rotation.
            _state[k].mag_history_index = (_state[k].mag_history_index + 1) % _mag_history_size;
            continue;
        }

        // put the vector in the history
        _state[k].mag_history[_state[k].mag_history_index] = AP_Eigen::Vector3f::get_nearest_integral<int>((AP_Eigen::Vector3f)(field - ofs));
        _state[k].mag_history_index                        = (_state[k].mag_history_index + 1) % _mag_history_size;
        // equation 6 of Bills paper
        v3f_diff *= (gain * (field.length() - v3f_hist.length()) / v3f_diff_len);

        // limit the change from any one reading. This is to prevent
        // single crazy readings from throwing off the offsets for a long time
        v3f_diff_len = v3f_diff.length();
        if (v3f_diff_len > max_change) {
            v3f_diff *= max_change / v3f_diff_len;
        }

        AP_Eigen::Vector3f new_offsets = _state[k].offset.get() - v3f_diff;

        // don't apply bad offsets
        if (new_offsets.is_nan() || new_offsets.is_inf() ) {
            continue;
        }
        
        // set the new offsets
        new_offsets.constrain(-COMPASS_OFS_LIMIT, COMPASS_OFS_LIMIT);
        _state[k].offset.set(new_offsets);
    }
}

#endif