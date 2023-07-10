/*
 *  Example of AP_Motors library.
 *  Code by Randy Mackay. DIYDrones.com
 */

/* on Linux run with
    ./waf configure --board linux
    ./waf --targets examples/AP_Motors_test
    ./build/linux/examples/AP_Motors_test
*/

// Libraries
#include <AP_Common/AP_Common.h>
#include <AP_HAL/AP_HAL.h>
#include <AP_BattMonitor/AP_BattMonitor.h>
#include <RC_Channel/RC_Channel.h>     // RC Channel Library
#include <AP_Motors/AP_Motors.h>
#include <RC_Channel/RC_Channel.h>
#include <SRV_Channel/SRV_Channel.h>
#include <AP_ESC_Telem/AP_ESC_Telem.h>
#include <stdio.h>

const AP_HAL::HAL& hal = AP_HAL::get_HAL();

// declare functions
void setup();
void loop();
void motor_order_test();
void stability_test();
void update_motors();
void print_all_motor_matrix();

#define HELI_TEST       0   // set to 1 to test helicopters
#define NUM_OUTPUTS     4   // set to 6 for hexacopter, 8 for octacopter and heli

#if HAL_WITH_ESC_TELEM
AP_ESC_Telem esc_telem;
#endif

#define VERSION "AP_Motors library test ver 1.1"

SRV_Channels srvs;

// uncomment the row below depending upon what frame you are using
//AP_MotorsTri  motors(400);
AP_MotorsMatrix   motors(400);
//AP_MotorsHeli_Single motors(400);
//AP_MotorsSingle motors(400);
//AP_MotorsCoax motors(400);

AP_BattMonitor _battmonitor{0, nullptr, nullptr};

bool thrust_boost = false;

// setup
void setup()
{

    // motor initialisation
    motors.set_dt(1.0/400.0);
    motors.set_update_rate(490);

#if HELI_TEST
    motors.init(AP_Motors::MOTOR_FRAME_HELI, AP_Motors::MOTOR_FRAME_TYPE_PLUS);
#else
#if NUM_OUTPUTS == 8
    motors.init(AP_Motors::MOTOR_FRAME_OCTA, AP_Motors::MOTOR_FRAME_TYPE_X);
#elif NUM_OUTPUTS == 6
    motors.init(AP_Motors::MOTOR_FRAME_HEXA, AP_Motors::MOTOR_FRAME_TYPE_X);
#else
    motors.init(AP_Motors::MOTOR_FRAME_QUAD, AP_Motors::MOTOR_FRAME_TYPE_X);
#endif
#endif // HELI_TEST == 0

#if HELI_TEST == 0
    motors.update_throttle_range();
    motors.set_throttle_avg_max(0.5f);
#endif
    motors.output_min();

    // allow command line args for single run
    uint8_t argc;
    char * const *argv;
    hal.util->commandline_arguments(argc, argv);
    if (argc > 1) {
#if HELI_TEST == 0
        for (uint8_t i = 2; i < argc; i++) {
            const char* arg = argv[i];

            const char *eq = strchr(arg, '=');
            if (eq == NULL) {
                ::printf("Expected argument with \"=\"\n");
                exit(1);
            }
            char cmd[20] {};
            strncpy(cmd, arg, eq-arg);
            const float value = atof(eq+1);
            if (strcmp(cmd,"yaw_headroom") == 0) {
                motors.set_yaw_headroom(value);

            } else if (strcmp(cmd,"throttle_avg_max") == 0) {
                motors.set_throttle_avg_max(value);

            } else if (strcmp(cmd,"thrust_boost") == 0) {
                thrust_boost = value > 0.0;

            } else {
                ::printf("Expected \"yaw_headroom\" or \"throttle_avg_max\"\n");
                exit(1);
            }

        }
#endif
        if (strcmp(argv[1],"t") == 0) {
            motor_order_test();

        } else if (strcmp(argv[1],"s") == 0) {
            stability_test();

#if HELI_TEST == 0
        } else if (strcmp(argv[1],"p") == 0) {
            print_all_motor_matrix();
#endif
        } else {
            ::printf("Expected first argument: 't', 's' or 'p'\n");

        }
        hal.scheduler->delay(1000);
        exit(0);
    }

    hal.scheduler->delay(1000);
}

// loop
void loop()
{
    int16_t value;

    // display help
    hal.console->printf("Press 't' to run motor orders test, 's' to run stability patch test.  Be careful the motors will spin!\n");

    // wait for user to enter something
    while( !hal.console->available() ) {
        hal.scheduler->delay(20);
    }

    // get character from user
    value = hal.console->read();

    // test motors
    if (value == 't' || value == 'T') {
        motor_order_test();
        hal.console->printf("finished test.\n");
    }
    if (value == 's' || value == 'S') {
        stability_test();
        hal.console->printf("finished test.\n");
    }
}

// print motor layout for all frame types in json format
void print_all_motor_matrix()
{
    hal.console->printf("{\n");
    hal.console->printf("\t\"Version\": \"%s\",\n", VERSION);
    hal.console->printf("\t\"layouts\": [\n");

    bool first_layout = true;
    char frame_and_type_string[30];

    for (uint8_t frame_class=0; frame_class <= AP_Motors::MOTOR_FRAME_DECA; frame_class++) {
        for (uint8_t frame_type=0; frame_type < AP_Motors::MOTOR_FRAME_TYPE_Y4; frame_type++) {
            if (frame_type == AP_Motors::MOTOR_FRAME_TYPE_VTAIL ||
                frame_type == AP_Motors::MOTOR_FRAME_TYPE_ATAIL) {
                // Skip the none planar motors types
                continue;
            }
            motors.init((AP_Motors::motor_frame_class)frame_class, (AP_Motors::motor_frame_type)frame_type);
            if (motors.initialised_ok()) {
                if (!first_layout) {
                    hal.console->printf(",\n");
                }
                first_layout = false;

                // Grab full frame string and strip "Frame: " and split
                // This is the long way round, motors does have direct getters, but there protected
                motors.get_frame_and_type_string(frame_and_type_string, ARRAY_SIZE(frame_and_type_string));
                char *frame_string = strchr(frame_and_type_string, ':');
                char *type_string = strchr(frame_and_type_string, '/');
                if (type_string != nullptr) {
                    *type_string = 0;
                }

                hal.console->printf("\t\t{\n");
                hal.console->printf("\t\t\t\"Class\": %i,\n", frame_class);
                hal.console->printf("\t\t\t\"ClassName\": \"%s\",\n", frame_string+2);
                hal.console->printf("\t\t\t\"Type\": %i,\n", frame_type);
                hal.console->printf("\t\t\t\"TypeName\": \"%s\",\n", (type_string != nullptr) ? type_string + 1 : "?");
                hal.console->printf("\t\t\t\"motors\": [\n");
                bool first_motor = true;
                for (uint8_t i = 0; i < AP_MOTORS_MAX_NUM_MOTORS; i++) {
                    float roll, pitch, yaw, throttle;
                    uint8_t testing_order;
                    if (motors.get_factors(i, roll, pitch, yaw, throttle, testing_order)) {
                        if (!first_motor) {
                            hal.console->printf(",\n");
                        }
                        first_motor = false;
                        hal.console->printf("\t\t\t\t{\n");
                        hal.console->printf("\t\t\t\t\t\"Number\": %i,\n", i+1);
                        hal.console->printf("\t\t\t\t\t\"TestOrder\": %i,\n", testing_order);
                        hal.console->printf("\t\t\t\t\t\"Rotation\": ");
                        if (is_positive(yaw)) {
                            hal.console->printf("\"CCW\",\n");
                        } else if (is_negative(yaw)) {
                            hal.console->printf("\"CW\",\n");
                        } else {
                            hal.console->printf("\"?\",\n");
                        }
                        hal.console->printf("\t\t\t\t\t\"Roll\": %0.4f,\n", roll);
                        hal.console->printf("\t\t\t\t\t\"Pitch\": %0.4f\n", pitch);
                        hal.console->printf("\t\t\t\t}");
                    }
                }
                hal.console->printf("\n");
                hal.console->printf("\t\t\t]\n");
                hal.console->printf("\t\t}");

            }
        }
    }

    hal.console->printf("\n");
    hal.console->printf("\t]\n");
    hal.console->printf("}\n");
}

// stability_test
void motor_order_test()
{
    hal.console->printf("testing motor order\n");
    motors.armed(true);
    for (int8_t i=1; i <= AP_MOTORS_MAX_NUM_MOTORS; i++) {
        hal.console->printf("Motor %d\n",(int)i);
        motors.output_test_seq(i, 1150);
        hal.scheduler->delay(300);
        motors.output_test_seq(i, 1000);
        hal.scheduler->delay(2000);
    }
    motors.armed(false);

}

// stability_test
void stability_test()
{
    hal.console->printf("%s\n", VERSION);
    char frame_and_type_string[30];
    motors.get_frame_and_type_string(frame_and_type_string, ARRAY_SIZE(frame_and_type_string));
    hal.console->printf("%s\n", frame_and_type_string);
#if HELI_TEST == 0
    hal.console->printf("Throttle average max: %0.4f\n",  motors.get_throttle_avg_max());
    hal.console->printf("Yaw headroom: %i\n", motors.get_yaw_headroom());
    hal.console->printf("Thrust boost: %s\n", thrust_boost?"True":"False");
#endif

    const float throttle_tests[] = {0.0, 0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7, 0.8, 0.9, 1.0};
    const uint8_t throttle_tests_num = ARRAY_SIZE(throttle_tests);
    const float rpy_tests[] = {-1.0, -0.9, -0.8, -0.7, -0.6, -0.5, -0.4, -0.3, -0.2, -0.1, 0.0, 0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7, 0.8, 0.9, 1.0};
    const uint8_t rpy_tests_num = ARRAY_SIZE(rpy_tests);

    // arm motors
    motors.armed(true);
    motors.set_interlock(true);
    SRV_Channels::enable_aux_servos();

    hal.console->printf("Roll,Pitch,Yaw,Thr,");
    for (uint8_t i=0; i<NUM_OUTPUTS; i++) {
        hal.console->printf("Mot%i,",i+1);
    }
#if HELI_TEST == 0
    for (uint8_t i=0; i<NUM_OUTPUTS; i++) {
        hal.console->printf("Mot%i_norm,",i+1);
    }
#endif
    hal.console->printf("LimR,LimP,LimY,LimThD,LimThU\n");

    // run stability test
    for (uint8_t y=0; y<rpy_tests_num; y++) {
        for (uint8_t p=0; p<rpy_tests_num; p++) {
            for (uint8_t r=0; r<rpy_tests_num; r++) {
                for (uint8_t t=0; t<throttle_tests_num; t++) {
                    const float roll_in = rpy_tests[r];
                    const float pitch_in = rpy_tests[p];
                    const float yaw_in = rpy_tests[y];
                    const float throttle_in = throttle_tests[t];
                    motors.set_roll(roll_in);
                    motors.set_pitch(pitch_in);
                    motors.set_yaw(yaw_in);
                    motors.set_throttle(throttle_in);
                    motors.set_desired_spool_state(AP_Motors::DesiredSpoolState::THROTTLE_UNLIMITED);
                    update_motors();
                    SRV_Channels::calc_pwm();
                    // display input and output
                    hal.console->printf("%0.2f,%0.2f,%0.2f,%0.2f,", roll_in, pitch_in, yaw_in, throttle_in);
                    for (uint8_t i=0; i<NUM_OUTPUTS; i++) {
                        hal.console->printf("%d,",(int)hal.rcout->read(i));
                    }
#if HELI_TEST == 0
                    for (uint8_t i=0; i<NUM_OUTPUTS; i++) {
                        hal.console->printf("%0.4f,", motors.get_thrust_rpyt_out(i));
                    }
#endif
                    hal.console->printf("%d,%d,%d,%d,%d\n",
                            (int)motors.limit.roll,
                            (int)motors.limit.pitch,
                            (int)motors.limit.yaw,
                            (int)motors.limit.throttle_lower,
                            (int)motors.limit.throttle_upper);
                }
            }
        }
    }

    // set all inputs to motor library to zero and disarm motors
    motors.set_pitch(0);
    motors.set_roll(0);
    motors.set_yaw(0);
    motors.set_throttle(0);
    motors.armed(false);

}

void update_motors()
{
    // call update motors 1000 times to get any ramp limiting complete
    for (uint16_t i=0; i<1000; i++) {
        motors.set_thrust_boost(thrust_boost);
        motors.output();
    }
}

AP_HAL_MAIN();
