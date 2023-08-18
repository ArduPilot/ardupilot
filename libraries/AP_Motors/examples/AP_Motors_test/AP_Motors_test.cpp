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
#include <AP_HAL/AP_HAL.h>
#include <AP_BattMonitor/AP_BattMonitor.h>
#include <AP_Motors/AP_Motors.h>
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

// Instantiate a few classes that will be needed so that the singletons can be called from the motors lib
#if HAL_WITH_ESC_TELEM
AP_ESC_Telem esc_telem;
#endif

#define VERSION "AP_Motors library test ver 1.1"

SRV_Channels srvs;
AP_BattMonitor _battmonitor{0, nullptr, nullptr};

AP_Motors *motors;
AP_MotorsMatrix *motors_matrix;

bool thrust_boost = false;

uint8_t num_outputs;

// setup
void setup()
{

    // default to quad frame class, frame class can be changed by argument in parser below
    AP_Motors::motor_frame_class frame_class = AP_Motors::MOTOR_FRAME_QUAD;

    // Parse the command line arguments
    uint8_t argc;
    char * const *argv;
    hal.util->commandline_arguments(argc, argv);
    if (argc > 1) {
        for (uint8_t i = 2; i < argc; i++) {
            const char *arg = argv[i];
            const char *eq = strchr(arg, '=');

            if (eq == NULL) {
                ::printf("Expected argument with \"=\"\n");
                exit(1);
            }

            char cmd[20] {};
            strncpy(cmd, arg, eq-arg);
            const float value = atof(eq+1);
            if (strcmp(cmd,"yaw_headroom") == 0) {
                if (motors_matrix != nullptr) {
                    motors_matrix->set_yaw_headroom(value);
                } else {
                    ::printf("frame_class = %d does not accept yaw_headroom commands\n", frame_class);
                }

            } else if (strcmp(cmd,"throttle_avg_max") == 0) {
                if (motors_matrix != nullptr) {
                    motors_matrix->set_throttle_avg_max(value);
                } else {
                    ::printf("frame_class = %d does not accept throttle_avg_max commands\n", frame_class);
                }

            } else if (strcmp(cmd,"thrust_boost") == 0) {
                thrust_boost = value > 0.0;

            } else if (strcmp(cmd,"swash") == 0) {
                if (frame_class == AP_Motors::MOTOR_FRAME_HELI) {
                    // This is a extremely complicated and unsafe way to get at the swash type parameter
                    // without adding helpers to both AP_MotorsHeli_Single and AP_MotorsHeli_Swash
                    // This works in the same way as AP_Param
                    char *swash = (char*)motors + AP_MotorsHeli_Single::var_info[6].offset;
                    AP_Int8 *swash_type = (AP_Int8*)swash + AP_MotorsHeli_Swash::var_info[0].offset;
                    swash_type->set(value);

                } else if (frame_class == AP_Motors::MOTOR_FRAME_HELI_DUAL) {
                    // More param hacking, its only a example
                    char *swash1 = (char*)motors + AP_MotorsHeli_Dual::var_info[7].offset;
                    char *swash2 = (char*)motors + AP_MotorsHeli_Dual::var_info[8].offset;
                    AP_Int8 *swash_type1 = (AP_Int8*)swash1 + AP_MotorsHeli_Swash::var_info[0].offset;
                    AP_Int8 *swash_type2 = (AP_Int8*)swash2 + AP_MotorsHeli_Swash::var_info[0].offset;
                    swash_type1->set(value);
                    swash_type2->set(value);

                } else {
                    ::printf("frame_class %i does not support swash\n", frame_class);
                    exit(1);
                }
                // Re-init motors to switch to the new swash type
                // Have to do this twice to make sure the swash type sticks
                motors->set_initialised_ok(false);
                motors->init(frame_class, AP_Motors::MOTOR_FRAME_TYPE_X);
                motors->set_initialised_ok(false);
                motors->init(frame_class, AP_Motors::MOTOR_FRAME_TYPE_X);

                // Check that init worked
                if (!motors->initialised_ok()) {
                    ::printf("ERROR: swash=%0.0f re-initialisation failed\n", value);
                    exit(1);
                }


            } else if (strcmp(cmd,"frame_class") == 0) {
                // We must have the frame_class argument 2nd as resulting class is used to determine if
                // we have access to certain functions in the multicopter motors child class
                if (i != 2) {
                    ::printf("frame_class must be second argument\n");
                    exit(1);
                }

                // Setup the correct motors object for the frame class to test
                frame_class = (AP_Motors::motor_frame_class)value;

                switch (frame_class) {

                    case AP_Motors::MOTOR_FRAME_QUAD:
                    case AP_Motors::MOTOR_FRAME_HEXA:
                    case AP_Motors::MOTOR_FRAME_OCTA:
                        motors_matrix = new AP_MotorsMatrix(400);
                        motors = motors_matrix;
                        break;

                    case AP_Motors::MOTOR_FRAME_HELI:
                        motors = new AP_MotorsHeli_Single(400);
                        // Mot 1-3 swashplate, mot 4 tail rotor pitch, mot 5 for 4th servo in H4-90 swash
                        num_outputs = 5;
                        break;

                    case AP_Motors::MOTOR_FRAME_HELI_DUAL:
                        motors = new AP_MotorsHeli_Dual(400);
                        // Mot 1-3 swashplate 1, mot 4-6 swashplate 2, mot 7 and 8 for 4th servos on H4-90 swash plates front and back, respectively
                        num_outputs = 8;
                        break;

                    case AP_Motors::MOTOR_FRAME_HELI_QUAD:
                        motors = new AP_MotorsHeli_Quad(400);
                        num_outputs = 4; // Only 4 collective servos
                        break;

                    default:
                        ::printf("ERROR: frame_class=%d not implemented\n", frame_class);
                        exit(1);
                }

                // Init motors
                motors->init(frame_class, AP_Motors::MOTOR_FRAME_TYPE_X);

                // Get the numper of outputs for a regular copter dynamically.  We can't currently do this for heli because the motor mask doesn't tell us the write thing
                if (motors_matrix != nullptr) {
                    num_outputs = __builtin_popcount(motors->get_motor_mask());
                }

                // Check that init worked
                if (!motors->initialised_ok()) {
                    ::printf("ERROR: frame_class=%d initialisation failed\n", frame_class);
                    exit(1);
                }

            } else {
                ::printf("Expected \"frame_class\", \"yaw_headroom\" or \"throttle_avg_max\"\n");
                exit(1);
            }
        }

        // if we haven't been given a frame class by argument we just assume a quad by default
        // so that the single first argument s or t still works
        if (motors == nullptr) {
            motors_matrix = new AP_MotorsMatrix(400);
            motors = motors_matrix;
            motors->init(AP_Motors::MOTOR_FRAME_QUAD, AP_Motors::MOTOR_FRAME_TYPE_X);
            num_outputs = 4;
        }

        // motor initialisation
        motors->set_dt(1.0/400.0);
        motors->set_update_rate(490);
        motors->output_min();

        // Start a test type based on the input argument
        if (strcmp(argv[1],"t") == 0) {
            motor_order_test();

        } else if (strcmp(argv[1],"s") == 0) {
            stability_test();

        } else if (strcmp(argv[1],"p") == 0) {
            if (motors_matrix == nullptr) {
                ::printf("Print only supports motors matrix\n");
            }
            print_all_motor_matrix();

        } else {
            ::printf("Expected first argument: 't', 's' or 'p'\n");

        }

        hal.scheduler->delay(1000);
        exit(0);

    } else {
        // We haven't been given a frame class so we just assume a quad frame as default
        motors_matrix = new AP_MotorsMatrix(400);
        motors = motors_matrix;
        motors->init(AP_Motors::MOTOR_FRAME_QUAD, AP_Motors::MOTOR_FRAME_TYPE_X);
        num_outputs = 4;
    }

    // motor initialisation
    motors->set_dt(1.0/400.0);
    motors->set_update_rate(490);

    char frame_and_type_string[30];
    motors->get_frame_and_type_string(frame_and_type_string, ARRAY_SIZE(frame_and_type_string));
    hal.console->printf("%s\n", frame_and_type_string);

    if (motors_matrix != nullptr) {
        motors_matrix->update_throttle_range();
        motors_matrix->set_throttle_avg_max(0.5f);
    }

    motors->output_min();

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
            motors_matrix->init((AP_Motors::motor_frame_class)frame_class, (AP_Motors::motor_frame_type)frame_type);
            if (motors_matrix->initialised_ok()) {
                if (!first_layout) {
                    hal.console->printf(",\n");
                }
                first_layout = false;

                // Grab full frame string and strip "Frame: " and split
                // This is the long way round, motors does have direct getters, but there protected
                motors_matrix->get_frame_and_type_string(frame_and_type_string, ARRAY_SIZE(frame_and_type_string));
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
                    if (motors_matrix->get_factors(i, roll, pitch, yaw, throttle, testing_order)) {
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
    motors->armed(true);
    for (int8_t i=1; i <= num_outputs; i++) {
        hal.console->printf("Motor %d\n",(int)i);
        motors->output_test_seq(i, 1150);
        hal.scheduler->delay(300);
        motors->output_test_seq(i, 1000);
        hal.scheduler->delay(2000);
    }
    motors->armed(false);

}

// stability_test
void stability_test()
{
    hal.console->printf("%s\n", VERSION);
    char frame_and_type_string[30];
    motors->get_frame_and_type_string(frame_and_type_string, ARRAY_SIZE(frame_and_type_string));
    hal.console->printf("%s\n", frame_and_type_string);

    if (motors_matrix != nullptr) {
        hal.console->printf("Throttle average max: %0.4f\n",  motors_matrix->get_throttle_avg_max());
        hal.console->printf("Yaw headroom: %i\n", motors_matrix->get_yaw_headroom());
        hal.console->printf("Thrust boost: %s\n", thrust_boost?"True":"False");
    }

    const float throttle_tests[] = {0.0, 0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7, 0.8, 0.9, 1.0};
    const uint8_t throttle_tests_num = ARRAY_SIZE(throttle_tests);
    const float rpy_tests[] = {-1.0, -0.9, -0.8, -0.7, -0.6, -0.5, -0.4, -0.3, -0.2, -0.1, 0.0, 0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7, 0.8, 0.9, 1.0};
    const uint8_t rpy_tests_num = ARRAY_SIZE(rpy_tests);

    // arm motors
    motors->armed(true);
    motors->set_interlock(true);
    SRV_Channels::enable_aux_servos();

    hal.console->printf("Roll,Pitch,Yaw,Thr,");
    for (uint8_t i=0; i<num_outputs; i++) {
        hal.console->printf("Mot%i,",i+1);
    }

    if (motors_matrix != nullptr) {
        for (uint8_t i=0; i<num_outputs; i++) {
            hal.console->printf("Mot%i_norm,",i+1);
        }
    }

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
                    motors->set_roll(roll_in);
                    motors->set_pitch(pitch_in);
                    motors->set_yaw(yaw_in);
                    motors->set_throttle(throttle_in);
                    motors->set_desired_spool_state(AP_Motors::DesiredSpoolState::THROTTLE_UNLIMITED);
                    update_motors();
                    SRV_Channels::calc_pwm();
                    SRV_Channels::output_ch_all();
                    // display input and output
                    hal.console->printf("%0.2f,%0.2f,%0.2f,%0.2f,", roll_in, pitch_in, yaw_in, throttle_in);
                    for (uint8_t i=0; i<num_outputs; i++) {
                        hal.console->printf("%d,",(int)hal.rcout->read(i));
                    }

                    if (motors_matrix != nullptr) {
                        for (uint8_t i=0; i<num_outputs; i++) {
                            hal.console->printf("%0.4f,", motors_matrix->get_thrust_rpyt_out(i));
                        }
                    }

                    hal.console->printf("%d,%d,%d,%d,%d\n",
                            (int)motors->limit.roll,
                            (int)motors->limit.pitch,
                            (int)motors->limit.yaw,
                            (int)motors->limit.throttle_lower,
                            (int)motors->limit.throttle_upper);
                }
            }
        }
    }

    // set all inputs to motor library to zero and disarm motors
    motors->set_pitch(0);
    motors->set_roll(0);
    motors->set_yaw(0);
    motors->set_throttle(0);
    motors->armed(false);

}

void update_motors()
{
    // call update motors 1000 times to get any ramp limiting complete
    for (uint16_t i=0; i<1000; i++) {
        motors->set_thrust_boost(thrust_boost);
        motors->output();
    }
}

AP_HAL_MAIN();
