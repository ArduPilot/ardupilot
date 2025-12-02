#include "AP_GPS_config.h"

#if AP_GPS_BLENDED_ENABLED

#include "AP_GPS_Blended.h"

// defines used to specify the mask position for use of different accuracy metrics in the blending algorithm
#define BLEND_MASK_USE_HPOS_ACC     1
#define BLEND_MASK_USE_VPOS_ACC     2
#define BLEND_MASK_USE_SPD_ACC      4

#define BLEND_COUNTER_FAILURE_INCREMENT 10

/*
 calculate the weightings used to blend GPSs location and velocity data
*/
bool AP_GPS_Blended::_calc_weights(void)
{
    static_assert(GPS_MAX_RECEIVERS == 2, "GPS blending only currently works with 2 receivers");
    // Note that the early quit below relies upon exactly 2 instances
    // The time delta calculations below also rely upon every instance being currently detected and being parsed

    // exit immediately if not enough receivers to do blending
    if (gps.state[0].status <= AP_GPS::NO_FIX || gps.state[1].status <= AP_GPS::NO_FIX) {
        return false;
    }

    // Use the oldest non-zero time, but if time difference is excessive, use newest to prevent a disconnected receiver from blocking updates
    uint32_t max_ms = 0; // newest non-zero system time of arrival of a GPS message
    uint32_t min_ms = -1; // oldest non-zero system time of arrival of a GPS message
    uint32_t max_rate_ms = 0; // largest update interval of a GPS receiver
    for (uint8_t i=0; i<GPS_MAX_RECEIVERS; i++) {
        // Find largest and smallest times
        if (gps.state[i].last_gps_time_ms > max_ms) {
            max_ms = gps.state[i].last_gps_time_ms;
        }
        if ((gps.state[i].last_gps_time_ms < min_ms) && (gps.state[i].last_gps_time_ms > 0)) {
            min_ms = gps.state[i].last_gps_time_ms;
        }
        max_rate_ms = MAX(gps.get_rate_ms(i), max_rate_ms);
        if (isinf(gps.state[i].speed_accuracy) ||
            isinf(gps.state[i].horizontal_accuracy) ||
            isinf(gps.state[i].vertical_accuracy)) {
            return false;
        }
    }
    if ((max_ms - min_ms) < (2 * max_rate_ms)) {
        // data is not too delayed so use the oldest time_stamp to give a chance for data from that receiver to be updated
        state.last_gps_time_ms = min_ms;
    } else {
        // receiver data has timed out so fail out of blending
        return false;
    }

    // calculate the sum squared speed accuracy across all GPS sensors
    float speed_accuracy_sum_sq = 0.0f;
    if (gps._blend_mask & BLEND_MASK_USE_SPD_ACC) {
        for (uint8_t i=0; i<GPS_MAX_RECEIVERS; i++) {
            if (gps.state[i].status >= AP_GPS::GPS_OK_FIX_3D) {
                if (gps.state[i].have_speed_accuracy && gps.state[i].speed_accuracy > 0.0f) {
                    speed_accuracy_sum_sq += sq(gps.state[i].speed_accuracy);
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
    if (gps._blend_mask & BLEND_MASK_USE_HPOS_ACC) {
        for (uint8_t i=0; i<GPS_MAX_RECEIVERS; i++) {
            if (gps.state[i].status >= AP_GPS::GPS_OK_FIX_2D) {
                if (gps.state[i].have_horizontal_accuracy && gps.state[i].horizontal_accuracy > 0.0f) {
                    horizontal_accuracy_sum_sq += sq(gps.state[i].horizontal_accuracy);
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
    if (gps._blend_mask & BLEND_MASK_USE_VPOS_ACC) {
        for (uint8_t i=0; i<GPS_MAX_RECEIVERS; i++) {
            if (gps.state[i].status >= AP_GPS::GPS_OK_FIX_3D) {
                if (gps.state[i].have_vertical_accuracy && gps.state[i].vertical_accuracy > 0.0f) {
                    vertical_accuracy_sum_sq += sq(gps.state[i].vertical_accuracy);
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
            if (gps.state[i].status >= AP_GPS::GPS_OK_FIX_2D && gps.state[i].horizontal_accuracy >= 0.001f) {
                hpos_blend_weights[i] = horizontal_accuracy_sum_sq / sq(gps.state[i].horizontal_accuracy);
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
            if (gps.state[i].status >= AP_GPS::GPS_OK_FIX_3D && gps.state[i].vertical_accuracy >= 0.001f) {
                vpos_blend_weights[i] = vertical_accuracy_sum_sq / sq(gps.state[i].vertical_accuracy);
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
            if (gps.state[i].status >= AP_GPS::GPS_OK_FIX_3D && gps.state[i].speed_accuracy >= 0.001f) {
                spd_blend_weights[i] = speed_accuracy_sum_sq / sq(gps.state[i].speed_accuracy);
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

bool AP_GPS_Blended::calc_weights()
{
    // adjust blend health counter
    if (!_calc_weights()) {
        _blend_health_counter = MIN(_blend_health_counter+BLEND_COUNTER_FAILURE_INCREMENT, 100);
    } else if (_blend_health_counter > 0) {
        _blend_health_counter--;
    }

    // we are never healthy if we do not have any weights:
    bool non_zero_weight_found = false;
    for (uint8_t i=0; i<GPS_MAX_RECEIVERS; i++) {
        if (_blend_weights[i] > 0) {
            non_zero_weight_found = true;
            break;
        }
    }
    if (!non_zero_weight_found) {
        return false;
    }

    // stop blending if unhealthy
    return _blend_health_counter < 50;
}

/*
 calculate a blended GPS state
*/
void AP_GPS_Blended::calc_state(void)
{
    // initialise the blended states so we can accumulate the results using the weightings for each GPS receiver
    state.instance = GPS_BLENDED_INSTANCE;
    state.status = AP_GPS::NO_FIX;
    state.time_week_ms = 0;
    state.time_week = 0;
    state.ground_speed = 0.0f;
    state.ground_course = 0.0f;
    state.hdop = GPS_UNKNOWN_DOP;
    state.vdop = GPS_UNKNOWN_DOP;
    state.num_sats = 0;
    state.velocity.zero();
    state.speed_accuracy = 1e6f;
    state.horizontal_accuracy = 1e6f;
    state.vertical_accuracy = 1e6f;
    state.have_vertical_velocity = false;
    state.have_speed_accuracy = false;
    state.have_horizontal_accuracy = false;
    state.have_vertical_accuracy = false;
    state.location = {};

    _blended_antenna_offset.zero();
    _blended_lag_sec = 0;

#if HAL_LOGGING_ENABLED
    const uint32_t last_blended_message_time_ms = timing.last_message_time_ms;
#endif
    timing.last_fix_time_ms = 0;
    timing.last_message_time_ms = 0;

    if (gps.state[0].have_undulation) {
        state.have_undulation = true;
        state.undulation = gps.state[0].undulation;
    } else if (gps.state[1].have_undulation) {
        state.have_undulation = true;
        state.undulation = gps.state[1].undulation;
    } else {
        state.have_undulation = false;
    }

    // combine the states into a blended solution
    for (uint8_t i=0; i<GPS_MAX_RECEIVERS; i++) {
        // use the highest status
        if (gps.state[i].status > state.status) {
            state.status = gps.state[i].status;
        }

        // calculate a blended average velocity
        state.velocity += gps.state[i].velocity * _blend_weights[i];

        // report the best valid accuracies and DOP metrics

        if (gps.state[i].have_horizontal_accuracy && gps.state[i].horizontal_accuracy > 0.0f && gps.state[i].horizontal_accuracy < state.horizontal_accuracy) {
            state.have_horizontal_accuracy = true;
            state.horizontal_accuracy = gps.state[i].horizontal_accuracy;
        }

        if (gps.state[i].have_vertical_accuracy && gps.state[i].vertical_accuracy > 0.0f && gps.state[i].vertical_accuracy < state.vertical_accuracy) {
            state.have_vertical_accuracy = true;
            state.vertical_accuracy = gps.state[i].vertical_accuracy;
        }

        if (gps.state[i].have_vertical_velocity) {
            state.have_vertical_velocity = true;
        }

        if (gps.state[i].have_speed_accuracy && gps.state[i].speed_accuracy > 0.0f && gps.state[i].speed_accuracy < state.speed_accuracy) {
            state.have_speed_accuracy = true;
            state.speed_accuracy = gps.state[i].speed_accuracy;
        }

        if (gps.state[i].hdop > 0 && gps.state[i].hdop < state.hdop) {
            state.hdop = gps.state[i].hdop;
        }

        if (gps.state[i].vdop > 0 && gps.state[i].vdop < state.vdop) {
            state.vdop = gps.state[i].vdop;
        }

        if (gps.state[i].num_sats > 0 && gps.state[i].num_sats > state.num_sats) {
            state.num_sats = gps.state[i].num_sats;
        }

        // report a blended average GPS antenna position
        Vector3f temp_antenna_offset = gps.params[i].antenna_offset;
        temp_antenna_offset *= _blend_weights[i];
        _blended_antenna_offset += temp_antenna_offset;

        // blend the timing data
        if (gps.timing[i].last_fix_time_ms > timing.last_fix_time_ms) {
            timing.last_fix_time_ms = gps.timing[i].last_fix_time_ms;
        }
        if (gps.timing[i].last_message_time_ms > timing.last_message_time_ms) {
            timing.last_message_time_ms = gps.timing[i].last_message_time_ms;
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
            state.location = gps.state[i].location;
        }
    }

    // Calculate the weighted sum of horizontal and vertical position offsets relative to the reference position
    Vector2f blended_NE_offset_m;
    float blended_alt_offset_cm = 0.0f;
    blended_NE_offset_m.zero();
    for (uint8_t i=0; i<GPS_MAX_RECEIVERS; i++) {
        if (_blend_weights[i] > 0.0f && i != best_index) {
            blended_NE_offset_m += state.location.get_distance_NE(gps.state[i].location) * _blend_weights[i];
            blended_alt_offset_cm += (float)(gps.state[i].location.alt - state.location.alt) * _blend_weights[i];
        }
    }

    // Add the sum of weighted offsets to the reference location to obtain the blended location
    state.location.offset(blended_NE_offset_m.x, blended_NE_offset_m.y);
    state.location.offset_up_cm(blended_alt_offset_cm);

    // Calculate ground speed and course from blended velocity vector
    state.ground_speed = state.velocity.xy().length();
    state.ground_course = wrap_360(degrees(atan2f(state.velocity.y, state.velocity.x)));

    // If the GPS week is the same then use a blended time_week_ms
    // If week is different, then use time stamp from GPS with largest weighting
    // detect inconsistent week data
    uint8_t last_week_instance = 0;
    bool weeks_consistent = true;
    for (uint8_t i=0; i<GPS_MAX_RECEIVERS; i++) {
        if (last_week_instance == 0 && _blend_weights[i] > 0) {
            // this is our first valid sensor week data
            last_week_instance = gps.state[i].time_week;
        } else if (last_week_instance != 0 && _blend_weights[i] > 0 && last_week_instance != gps.state[i].time_week) {
            // there is valid sensor week data that is inconsistent
            weeks_consistent = false;
        }
    }
    // calculate output
    if (!weeks_consistent) {
        // use data from highest weighted sensor
        state.time_week = gps.state[best_index].time_week;
        state.time_week_ms = gps.state[best_index].time_week_ms;
    } else {
        // use week number from highest weighting GPS (they should all have the same week number)
        state.time_week = gps.state[best_index].time_week;
        // calculate a blended value for the number of ms lapsed in the week
        double temp_time_0 = 0.0;
        for (uint8_t i=0; i<GPS_MAX_RECEIVERS; i++) {
            if (_blend_weights[i] > 0.0f) {
                temp_time_0 += (double)gps.state[i].time_week_ms * (double)_blend_weights[i];
            }
        }
        state.time_week_ms = (uint32_t)temp_time_0;
    }

    // calculate a blended value for the timing data and lag
    double temp_time_1 = 0.0;
    double temp_time_2 = 0.0;
    for (uint8_t i=0; i<GPS_MAX_RECEIVERS; i++) {
        if (_blend_weights[i] > 0.0f) {
            temp_time_1 += (double)gps.timing[i].last_fix_time_ms * (double) _blend_weights[i];
            temp_time_2 += (double)gps.timing[i].last_message_time_ms * (double)_blend_weights[i];
            float gps_lag_sec = 0;
            gps.get_lag(i, gps_lag_sec);
            _blended_lag_sec += gps_lag_sec * _blend_weights[i];
        }
    }
    timing.last_fix_time_ms = (uint32_t)temp_time_1;
    timing.last_message_time_ms = (uint32_t)temp_time_2;

#if HAL_LOGGING_ENABLED
    if (timing.last_message_time_ms > last_blended_message_time_ms &&
        should_log()) {
        gps.Write_GPS(GPS_BLENDED_INSTANCE);
    }
#endif

#if CONFIG_HAL_BOARD == HAL_BOARD_SITL
    // sanity checks
    if (state.status > AP_GPS::NO_FIX && !state.location.initialised()) {
        AP_HAL::panic("status is >NO_FIX but location is zero");
    }
#endif  // CONFIG_HAL_BOARD == HAL_BOARD_SITL
}

bool AP_GPS_Blended::get_lag(float &lag_sec) const
{
        lag_sec = _blended_lag_sec;
        // auto switching uses all GPS receivers, so all must be configured
        uint8_t inst; // we don't actually care what the number is, but must pass it
        return gps.first_unconfigured_gps(inst);
}

#endif  // AP_GPS_BLENDED_ENABLED
