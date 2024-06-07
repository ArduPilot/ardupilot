#include "AP_GPS.h"

#if AP_GPS_BLENDED_ENABLED

// defines used to specify the mask position for use of different accuracy metrics in the blending algorithm
#define BLEND_MASK_USE_HPOS_ACC     1
#define BLEND_MASK_USE_VPOS_ACC     2
#define BLEND_MASK_USE_SPD_ACC      4

// pre-arm check of GPS blending.  True means healthy or that blending is not being used
bool AP_GPS::blend_health_check() const
{
    return (_blend_health_counter < 50);
}

/*
 calculate the weightings used to blend GPSs location and velocity data
*/
bool AP_GPS::calc_blend_weights(void)
{
    // zero the blend weights
    memset(&_blend_weights, 0, sizeof(_blend_weights));


    static_assert(GPS_MAX_RECEIVERS == 2, "GPS blending only currently works with 2 receivers");
    // Note that the early quit below relies upon exactly 2 instances
    // The time delta calculations below also rely upon every instance being currently detected and being parsed

    // exit immediately if not enough receivers to do blending
    if (state[0].status <= NO_FIX || state[1].status <= NO_FIX) {
        return false;
    }

    // Use the oldest non-zero time, but if time difference is excessive, use newest to prevent a disconnected receiver from blocking updates
    uint32_t max_ms = 0; // newest non-zero system time of arrival of a GPS message
    uint32_t min_ms = -1; // oldest non-zero system time of arrival of a GPS message
    uint32_t max_rate_ms = 0; // largest update interval of a GPS receiver
    for (uint8_t i=0; i<GPS_MAX_RECEIVERS; i++) {
        // Find largest and smallest times
        if (state[i].last_gps_time_ms > max_ms) {
            max_ms = state[i].last_gps_time_ms;
        }
        if ((state[i].last_gps_time_ms < min_ms) && (state[i].last_gps_time_ms > 0)) {
            min_ms = state[i].last_gps_time_ms;
        }
        max_rate_ms = MAX(get_rate_ms(i), max_rate_ms);
        if (isinf(state[i].speed_accuracy) ||
            isinf(state[i].horizontal_accuracy) ||
            isinf(state[i].vertical_accuracy)) {
            return false;
        }
    }
    if ((max_ms - min_ms) < (2 * max_rate_ms)) {
        // data is not too delayed so use the oldest time_stamp to give a chance for data from that receiver to be updated
        state[GPS_BLENDED_INSTANCE].last_gps_time_ms = min_ms;
    } else {
        // receiver data has timed out so fail out of blending
        return false;
    }

    // calculate the sum squared speed accuracy across all GPS sensors
    float speed_accuracy_sum_sq = 0.0f;
    if (_blend_mask & BLEND_MASK_USE_SPD_ACC) {
        for (uint8_t i=0; i<GPS_MAX_RECEIVERS; i++) {
            if (state[i].status >= GPS_OK_FIX_3D) {
                if (state[i].have_speed_accuracy && state[i].speed_accuracy > 0.0f) {
                    speed_accuracy_sum_sq += sq(state[i].speed_accuracy);
                } else {
                    // not all receivers support this metric so set it to zero and don't use it
                    speed_accuracy_sum_sq = 0.0f;
                    break;
                }
            }
        }
    }

    // calculate the sum squared horizontal position accuracy across all GPS sensors
    float horizontal_accuracy_sum_sq = 0.0f;
    if (_blend_mask & BLEND_MASK_USE_HPOS_ACC) {
        for (uint8_t i=0; i<GPS_MAX_RECEIVERS; i++) {
            if (state[i].status >= GPS_OK_FIX_2D) {
                if (state[i].have_horizontal_accuracy && state[i].horizontal_accuracy > 0.0f) {
                    horizontal_accuracy_sum_sq += sq(state[i].horizontal_accuracy);
                } else {
                    // not all receivers support this metric so set it to zero and don't use it
                    horizontal_accuracy_sum_sq = 0.0f;
                    break;
                }
            }
        }
    }

    // calculate the sum squared vertical position accuracy across all GPS sensors
    float vertical_accuracy_sum_sq = 0.0f;
    if (_blend_mask & BLEND_MASK_USE_VPOS_ACC) {
        for (uint8_t i=0; i<GPS_MAX_RECEIVERS; i++) {
            if (state[i].status >= GPS_OK_FIX_3D) {
                if (state[i].have_vertical_accuracy && state[i].vertical_accuracy > 0.0f) {
                    vertical_accuracy_sum_sq += sq(state[i].vertical_accuracy);
                } else {
                    // not all receivers support this metric so set it to zero and don't use it
                    vertical_accuracy_sum_sq = 0.0f;
                    break;
                }
            }
        }
    }
    // Check if we can do blending using reported accuracy
    bool can_do_blending = (horizontal_accuracy_sum_sq > 0.0f || vertical_accuracy_sum_sq > 0.0f || speed_accuracy_sum_sq > 0.0f);

    // if we can't do blending using reported accuracy, return false and hard switch logic will be used instead
    if (!can_do_blending) {
        return false;
    }

    float sum_of_all_weights = 0.0f;

    // calculate a weighting using the reported horizontal position
    float hpos_blend_weights[GPS_MAX_RECEIVERS] = {};
    if (horizontal_accuracy_sum_sq > 0.0f) {
        // calculate the weights using the inverse of the variances
        float sum_of_hpos_weights = 0.0f;
        for (uint8_t i=0; i<GPS_MAX_RECEIVERS; i++) {
            if (state[i].status >= GPS_OK_FIX_2D && state[i].horizontal_accuracy >= 0.001f) {
                hpos_blend_weights[i] = horizontal_accuracy_sum_sq / sq(state[i].horizontal_accuracy);
                sum_of_hpos_weights += hpos_blend_weights[i];
            }
        }
        // normalise the weights
        if (sum_of_hpos_weights > 0.0f) {
            for (uint8_t i=0; i<GPS_MAX_RECEIVERS; i++) {
                hpos_blend_weights[i] = hpos_blend_weights[i] / sum_of_hpos_weights;
            }
            sum_of_all_weights += 1.0f;
        }
    }

    // calculate a weighting using the reported vertical position accuracy
    float vpos_blend_weights[GPS_MAX_RECEIVERS] = {};
    if (vertical_accuracy_sum_sq > 0.0f) {
        // calculate the weights using the inverse of the variances
        float sum_of_vpos_weights = 0.0f;
        for (uint8_t i=0; i<GPS_MAX_RECEIVERS; i++) {
            if (state[i].status >= GPS_OK_FIX_3D && state[i].vertical_accuracy >= 0.001f) {
                vpos_blend_weights[i] = vertical_accuracy_sum_sq / sq(state[i].vertical_accuracy);
                sum_of_vpos_weights += vpos_blend_weights[i];
            }
        }
        // normalise the weights
        if (sum_of_vpos_weights > 0.0f) {
            for (uint8_t i=0; i<GPS_MAX_RECEIVERS; i++) {
                vpos_blend_weights[i] = vpos_blend_weights[i] / sum_of_vpos_weights;
            }
            sum_of_all_weights += 1.0f;
        };
    }

    // calculate a weighting using the reported speed accuracy
    float spd_blend_weights[GPS_MAX_RECEIVERS] = {};
    if (speed_accuracy_sum_sq > 0.0f) {
        // calculate the weights using the inverse of the variances
        float sum_of_spd_weights = 0.0f;
        for (uint8_t i=0; i<GPS_MAX_RECEIVERS; i++) {
            if (state[i].status >= GPS_OK_FIX_3D && state[i].speed_accuracy >= 0.001f) {
                spd_blend_weights[i] = speed_accuracy_sum_sq / sq(state[i].speed_accuracy);
                sum_of_spd_weights += spd_blend_weights[i];
            }
        }
        // normalise the weights
        if (sum_of_spd_weights > 0.0f) {
            for (uint8_t i=0; i<GPS_MAX_RECEIVERS; i++) {
                spd_blend_weights[i] = spd_blend_weights[i] / sum_of_spd_weights;
            }
            sum_of_all_weights += 1.0f;
        }
    }

    if (!is_positive(sum_of_all_weights)) {
        return false;
    }

    // calculate an overall weight
    for (uint8_t i=0; i<GPS_MAX_RECEIVERS; i++) {
        _blend_weights[i] = (hpos_blend_weights[i] + vpos_blend_weights[i] + spd_blend_weights[i]) / sum_of_all_weights;
    }

    return true;
}

/*
 calculate a blended GPS state
*/
void AP_GPS::calc_blended_state(void)
{
    // initialise the blended states so we can accumulate the results using the weightings for each GPS receiver
    state[GPS_BLENDED_INSTANCE].instance = GPS_BLENDED_INSTANCE;
    state[GPS_BLENDED_INSTANCE].status = NO_FIX;
    state[GPS_BLENDED_INSTANCE].time_week_ms = 0;
    state[GPS_BLENDED_INSTANCE].time_week = 0;
    state[GPS_BLENDED_INSTANCE].ground_speed = 0.0f;
    state[GPS_BLENDED_INSTANCE].ground_course = 0.0f;
    state[GPS_BLENDED_INSTANCE].hdop = GPS_UNKNOWN_DOP;
    state[GPS_BLENDED_INSTANCE].vdop = GPS_UNKNOWN_DOP;
    state[GPS_BLENDED_INSTANCE].num_sats = 0;
    state[GPS_BLENDED_INSTANCE].velocity.zero();
    state[GPS_BLENDED_INSTANCE].speed_accuracy = 1e6f;
    state[GPS_BLENDED_INSTANCE].horizontal_accuracy = 1e6f;
    state[GPS_BLENDED_INSTANCE].vertical_accuracy = 1e6f;
    state[GPS_BLENDED_INSTANCE].have_vertical_velocity = false;
    state[GPS_BLENDED_INSTANCE].have_speed_accuracy = false;
    state[GPS_BLENDED_INSTANCE].have_horizontal_accuracy = false;
    state[GPS_BLENDED_INSTANCE].have_vertical_accuracy = false;
    state[GPS_BLENDED_INSTANCE].location = {};

    _blended_antenna_offset.zero();
    _blended_lag_sec = 0;

#if HAL_LOGGING_ENABLED
    const uint32_t last_blended_message_time_ms = timing[GPS_BLENDED_INSTANCE].last_message_time_ms;
#endif
    timing[GPS_BLENDED_INSTANCE].last_fix_time_ms = 0;
    timing[GPS_BLENDED_INSTANCE].last_message_time_ms = 0;

    if (state[0].have_undulation) {
        state[GPS_BLENDED_INSTANCE].have_undulation = true;
        state[GPS_BLENDED_INSTANCE].undulation = state[0].undulation;
    } else if (state[1].have_undulation) {
        state[GPS_BLENDED_INSTANCE].have_undulation = true;
        state[GPS_BLENDED_INSTANCE].undulation = state[1].undulation;
    } else {
        state[GPS_BLENDED_INSTANCE].have_undulation = false;
    }

    // combine the states into a blended solution
    for (uint8_t i=0; i<GPS_MAX_RECEIVERS; i++) {
        // use the highest status
        if (state[i].status > state[GPS_BLENDED_INSTANCE].status) {
            state[GPS_BLENDED_INSTANCE].status = state[i].status;
        }

        // calculate a blended average velocity
        state[GPS_BLENDED_INSTANCE].velocity += state[i].velocity * _blend_weights[i];

        // report the best valid accuracies and DOP metrics

        if (state[i].have_horizontal_accuracy && state[i].horizontal_accuracy > 0.0f && state[i].horizontal_accuracy < state[GPS_BLENDED_INSTANCE].horizontal_accuracy) {
            state[GPS_BLENDED_INSTANCE].have_horizontal_accuracy = true;
            state[GPS_BLENDED_INSTANCE].horizontal_accuracy = state[i].horizontal_accuracy;
        }

        if (state[i].have_vertical_accuracy && state[i].vertical_accuracy > 0.0f && state[i].vertical_accuracy < state[GPS_BLENDED_INSTANCE].vertical_accuracy) {
            state[GPS_BLENDED_INSTANCE].have_vertical_accuracy = true;
            state[GPS_BLENDED_INSTANCE].vertical_accuracy = state[i].vertical_accuracy;
        }

        if (state[i].have_vertical_velocity) {
            state[GPS_BLENDED_INSTANCE].have_vertical_velocity = true;
        }

        if (state[i].have_speed_accuracy && state[i].speed_accuracy > 0.0f && state[i].speed_accuracy < state[GPS_BLENDED_INSTANCE].speed_accuracy) {
            state[GPS_BLENDED_INSTANCE].have_speed_accuracy = true;
            state[GPS_BLENDED_INSTANCE].speed_accuracy = state[i].speed_accuracy;
        }

        if (state[i].hdop > 0 && state[i].hdop < state[GPS_BLENDED_INSTANCE].hdop) {
            state[GPS_BLENDED_INSTANCE].hdop = state[i].hdop;
        }

        if (state[i].vdop > 0 && state[i].vdop < state[GPS_BLENDED_INSTANCE].vdop) {
            state[GPS_BLENDED_INSTANCE].vdop = state[i].vdop;
        }

        if (state[i].num_sats > 0 && state[i].num_sats > state[GPS_BLENDED_INSTANCE].num_sats) {
            state[GPS_BLENDED_INSTANCE].num_sats = state[i].num_sats;
        }

        // report a blended average GPS antenna position
        Vector3f temp_antenna_offset = params[i].antenna_offset;
        temp_antenna_offset *= _blend_weights[i];
        _blended_antenna_offset += temp_antenna_offset;

        // blend the timing data
        if (timing[i].last_fix_time_ms > timing[GPS_BLENDED_INSTANCE].last_fix_time_ms) {
            timing[GPS_BLENDED_INSTANCE].last_fix_time_ms = timing[i].last_fix_time_ms;
        }
        if (timing[i].last_message_time_ms > timing[GPS_BLENDED_INSTANCE].last_message_time_ms) {
            timing[GPS_BLENDED_INSTANCE].last_message_time_ms = timing[i].last_message_time_ms;
        }
    }

    /*
     * Calculate an instantaneous weighted/blended average location from the available GPS instances and store in the _output_state.
     * This will be statistically the most likely location, but will be not stable enough for direct use by the autopilot.
    */

    // Use the GPS with the highest weighting as the reference position
    float best_weight = 0.0f;
    uint8_t best_index = 0;
    for (uint8_t i=0; i<GPS_MAX_RECEIVERS; i++) {
        if (_blend_weights[i] > best_weight) {
            best_weight = _blend_weights[i];
            best_index = i;
            state[GPS_BLENDED_INSTANCE].location = state[i].location;
        }
    }

    // Calculate the weighted sum of horizontal and vertical position offsets relative to the reference position
    Vector2f blended_NE_offset_m;
    float blended_alt_offset_cm = 0.0f;
    blended_NE_offset_m.zero();
    for (uint8_t i=0; i<GPS_MAX_RECEIVERS; i++) {
        if (_blend_weights[i] > 0.0f && i != best_index) {
            blended_NE_offset_m += state[GPS_BLENDED_INSTANCE].location.get_distance_NE(state[i].location) * _blend_weights[i];
            blended_alt_offset_cm += (float)(state[i].location.alt - state[GPS_BLENDED_INSTANCE].location.alt) * _blend_weights[i];
        }
    }

    // Add the sum of weighted offsets to the reference location to obtain the blended location
    state[GPS_BLENDED_INSTANCE].location.offset(blended_NE_offset_m.x, blended_NE_offset_m.y);
    state[GPS_BLENDED_INSTANCE].location.alt += (int)blended_alt_offset_cm;

    // Calculate ground speed and course from blended velocity vector
    state[GPS_BLENDED_INSTANCE].ground_speed = state[GPS_BLENDED_INSTANCE].velocity.xy().length();
    state[GPS_BLENDED_INSTANCE].ground_course = wrap_360(degrees(atan2f(state[GPS_BLENDED_INSTANCE].velocity.y, state[GPS_BLENDED_INSTANCE].velocity.x)));

    // If the GPS week is the same then use a blended time_week_ms
    // If week is different, then use time stamp from GPS with largest weighting
    // detect inconsistent week data
    uint8_t last_week_instance = 0;
    bool weeks_consistent = true;
    for (uint8_t i=0; i<GPS_MAX_RECEIVERS; i++) {
        if (last_week_instance == 0 && _blend_weights[i] > 0) {
            // this is our first valid sensor week data
            last_week_instance = state[i].time_week;
        } else if (last_week_instance != 0 && _blend_weights[i] > 0 && last_week_instance != state[i].time_week) {
            // there is valid sensor week data that is inconsistent
            weeks_consistent = false;
        }
    }
    // calculate output
    if (!weeks_consistent) {
        // use data from highest weighted sensor
        state[GPS_BLENDED_INSTANCE].time_week = state[best_index].time_week;
        state[GPS_BLENDED_INSTANCE].time_week_ms = state[best_index].time_week_ms;
    } else {
        // use week number from highest weighting GPS (they should all have the same week number)
        state[GPS_BLENDED_INSTANCE].time_week = state[best_index].time_week;
        // calculate a blended value for the number of ms lapsed in the week
        double temp_time_0 = 0.0;
        for (uint8_t i=0; i<GPS_MAX_RECEIVERS; i++) {
            if (_blend_weights[i] > 0.0f) {
                temp_time_0 += (double)state[i].time_week_ms * (double)_blend_weights[i];
            }
        }
        state[GPS_BLENDED_INSTANCE].time_week_ms = (uint32_t)temp_time_0;
    }

    // calculate a blended value for the timing data and lag
    double temp_time_1 = 0.0;
    double temp_time_2 = 0.0;
    for (uint8_t i=0; i<GPS_MAX_RECEIVERS; i++) {
        if (_blend_weights[i] > 0.0f) {
            temp_time_1 += (double)timing[i].last_fix_time_ms * (double) _blend_weights[i];
            temp_time_2 += (double)timing[i].last_message_time_ms * (double)_blend_weights[i];
            float gps_lag_sec = 0;
            get_lag(i, gps_lag_sec);
            _blended_lag_sec += gps_lag_sec * _blend_weights[i];
        }
    }
    timing[GPS_BLENDED_INSTANCE].last_fix_time_ms = (uint32_t)temp_time_1;
    timing[GPS_BLENDED_INSTANCE].last_message_time_ms = (uint32_t)temp_time_2;

#if HAL_LOGGING_ENABLED
    if (timing[GPS_BLENDED_INSTANCE].last_message_time_ms > last_blended_message_time_ms &&
        should_log()) {
        Write_GPS(GPS_BLENDED_INSTANCE);
    }
#endif
}
#endif  // AP_GPS_BLENDED_ENABLED
