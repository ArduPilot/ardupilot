/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

#define MOT_RECOVERY_HIGH_MOTOR_THRESHOLD 0.95f
#define MOT_RECOVERY_LOW_AVG_THRESHOLD 0.77f
#define MOT_RECOVERY_DETECTION_TIME 0.2f
#define MOT_RECOVERY_POST_DELAY 0.75f
#define MOT_RECOVERY_MOTOR_PCT 0.5f
#define MOT_RECOVERY_RAMP_TIME 0.25f
#define MOT_RECOVERY_MOT_FILT_HZ 5.0f

uint8_t last_highest_motor = 0;
uint32_t motor_fail_start_time = 0;
uint32_t motor_recovery_end_time = 0;
const float motor_filt_alpha = constrain_float(MAIN_LOOP_SECONDS/(MAIN_LOOP_SECONDS+(1.0f/(2.0f*M_PI_F*MOT_RECOVERY_MOT_FILT_HZ))),0.0f,1.0f);
float motor_out_pct_filtered[AP_MOTORS_MAX_NUM_MOTORS];

static void update_motor_fail_detector() {
    uint32_t tnow_ms = millis();

    if (!motors.armed()) {
        // reset fail timer and exit if disarmed
        motor_fail_start_time = tnow_ms;

        //reset motor filters
        memset(motor_out_pct_filtered, 0, sizeof(motor_out_pct_filtered));
        return;
    }

    if (motors.motor_recovery_running()) {
        // if recovering, reset fail timer, reset recovery timer, and exit
        motor_recovery_end_time = tnow_ms;
        motor_fail_start_time = tnow_ms;

        // reset motor filters
        for (uint8_t i=0; i<AP_MOTORS_MAX_NUM_MOTORS; i++) {
            motor_out_pct_filtered[i] = motors.get_motor_out_pct(i);
        }
        return;
    }

    if (tnow_ms-motor_recovery_end_time <= MOT_RECOVERY_POST_DELAY*1.0e3f) {
        // if the last recovery occurred recently, reset timer and exit
        motor_fail_start_time = tnow_ms;

        // reset motor filters
        for (uint8_t i=0; i<AP_MOTORS_MAX_NUM_MOTORS; i++) {
            motor_out_pct_filtered[i] = motors.get_motor_out_pct(i);
        }
        return;
    }

    // find highest motor and motor average
    uint8_t highest_motor_index = 0;
    float highest_motor = 0.0f;
    float motor_avg = 0.0f;
    uint8_t motor_count = 0;
    for (uint8_t i=0; i<AP_MOTORS_MAX_NUM_MOTORS; i++) {
        motor_out_pct_filtered[i] += (motors.get_motor_out_pct(i)-motor_out_pct_filtered[i]) * motor_filt_alpha;

        if (motors.motor_enabled[i]) {
            if (motor_out_pct_filtered[i] > highest_motor) {
                highest_motor = motor_out_pct_filtered[i];
                highest_motor_index = i;
            }

            motor_avg += motor_out_pct_filtered[i];
            motor_count++;
        }
    }
    motor_avg /= motor_count;

    bool motor_fail_criteria_met = (highest_motor_index == last_highest_motor) && (highest_motor >= MOT_RECOVERY_HIGH_MOTOR_THRESHOLD) && (motor_avg <= MOT_RECOVERY_LOW_AVG_THRESHOLD);
    if (!motor_fail_criteria_met) {
        // if we don't meet the failure criteria, reset fail timer
        motor_fail_start_time = tnow_ms;
    } else if(tnow_ms-motor_fail_start_time > MOT_RECOVERY_DETECTION_TIME*1.0e3f) {
        // if criteria for a motor failure are met continuously for MOT_RECOVERY_DETECTION_TIME seconds, attempt a restart on that motor
        motors.do_motor_recovery(0xFF, MOT_RECOVERY_MOTOR_PCT, MOT_RECOVERY_RAMP_TIME);
    }

    last_highest_motor = highest_motor_index;
}
