#pragma once

/*
  class to support Motor_Test
*/

class Motor_Test {
public:
    friend class Copter;
    // motor test definitions
    #define MOTOR_TEST_TIMEOUT_SEC          600     // max timeout is 10 minutes (600 seconds)

    static uint32_t motor_test_start_ms;        // system time the motor test began
    static uint32_t motor_test_timeout_ms;      // test will timeout this many milliseconds after the motor_test_start_ms
    static uint8_t motor_test_seq;              // motor sequence number of motor being tested
    static uint8_t motor_test_count;            // number of motors to test
    static uint8_t motor_test_throttle_type;    // motor throttle type (0=throttle percentage, 1=PWM, 2=pilot throttle channel pass-through)
    static float motor_test_throttle_value;  // throttle to be sent to motor, value depends upon it's type

    uint32_t now;
    int16_t pwm;
    int16_t pwm_min;
    int16_t pwm_max;
};
Motor_Test motor_test;
