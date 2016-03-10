/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

#include "Copter.h"

#if MOTOR_FAIL_RECOVERY == ENABLED

#define MOT_RECOVERY_HIGH_MOTOR_THRESHOLD 0.95f
#define MOT_RECOVERY_LOW_AVG_THRESHOLD 0.77f
#define MOT_RECOVERY_DETECTION_TIME 0.2f
#define MOT_RECOVERY_ATTEMPT_INTERVAL 1.25f
#define MOT_RECOVERY_MOT_FILT_HZ 5.0f

void Copter::update_motor_fail_detector() {
    uint32_t tnow_ms = millis();

    if (!motors.armed()) {
        // reset fail timer and exit if disarmed
        motor_recovery_state.motor_fail_start_time = tnow_ms;

        //reset motor filters
        memset(motor_recovery_state.motor_out_pct_filtered, 0, sizeof(motor_recovery_state.motor_out_pct_filtered));
        return;
    }

    // compute motor_out_pct
    float motor_out_pct[AP_MOTORS_MAX_NUM_MOTORS];
    for (uint8_t i=0; i<AP_MOTORS_MAX_NUM_MOTORS; i++) {
        if (motors.using_channel(i)) {
            motor_out_pct[i] = (float)(hal.rcout->read_last_sent(i)-motors.get_pwm_out_min()) / (motors.get_pwm_out_max()-motors.get_pwm_out_min());
        } else {
            motor_out_pct[i] = 0.0f;
        }
    }

    if (tnow_ms-motor_recovery_state.motor_recovery_start_time <= MOT_RECOVERY_ATTEMPT_INTERVAL*1.0e3f) {
        // if the last recovery occurred recently, reset timer and exit
        motor_recovery_state.motor_fail_start_time = tnow_ms;

        // reset motor filters
        for (uint8_t i=0; i<AP_MOTORS_MAX_NUM_MOTORS; i++) {
            motor_recovery_state.motor_out_pct_filtered[i] = motor_out_pct[i];
        }
        return;
    }

    // find highest motor and motor average
    static const float motor_filt_alpha = constrain_float(MAIN_LOOP_SECONDS/(MAIN_LOOP_SECONDS+(1.0f/(2.0f*M_PI*MOT_RECOVERY_MOT_FILT_HZ))),0.0f,1.0f);
    uint8_t highest_motor_index = 0;
    float highest_motor = 0.0f;
    float motor_avg = 0.0f;
    uint8_t motor_count = 0;
    for (uint8_t i=0; i<AP_MOTORS_MAX_NUM_MOTORS; i++) {
        motor_recovery_state.motor_out_pct_filtered[i] += (motor_out_pct[i]-motor_recovery_state.motor_out_pct_filtered[i]) * motor_filt_alpha;

        if (motors.using_channel(i)) {
            if (motor_recovery_state.motor_out_pct_filtered[i] > highest_motor) {
                highest_motor = motor_recovery_state.motor_out_pct_filtered[i];
                highest_motor_index = i;
            }

            motor_avg += motor_recovery_state.motor_out_pct_filtered[i];
            motor_count++;
        }
    }
    motor_avg /= motor_count;

    bool motor_fail_criteria_met = (highest_motor_index == motor_recovery_state.last_highest_motor) && (highest_motor >= MOT_RECOVERY_HIGH_MOTOR_THRESHOLD) && (motor_avg <= MOT_RECOVERY_LOW_AVG_THRESHOLD);
    if (!motor_fail_criteria_met) {
        // if we don't meet the failure criteria, reset fail timer
        motor_recovery_state.motor_fail_start_time = tnow_ms;
    } else if(tnow_ms-motor_recovery_state.motor_fail_start_time > MOT_RECOVERY_DETECTION_TIME*1.0e3f) {
        // if criteria for a motor failure are met continuously for MOT_RECOVERY_DETECTION_TIME seconds, attempt a restart on that motor
        motors.do_motor_recovery();
        motor_recovery_state.motor_recovery_start_time = tnow_ms;
    }

    motor_recovery_state.last_highest_motor = highest_motor_index;
}
#endif // MOTOR_FAIL_RECOVERY == ENABLED
