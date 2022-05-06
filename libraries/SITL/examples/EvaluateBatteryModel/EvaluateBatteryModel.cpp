#include <AP_HAL/AP_HAL.h>

#include <SITL/SIM_Battery.h>
#include <SITL/SIM_Frame.h>

#include <stdio.h>

/* run with:
    ./waf configure --board linux
    ./waf build --targets examples/EvaluateBatteryModel
    ./build/linux/examples/EvaluateBatteryModel Tools/autotest/models/Callisto.json 50
*/

void setup();
void loop();

const AP_HAL::HAL& hal = AP_HAL::get_HAL();

// helper to give access to protected functions
class Frame_helper : public SITL::Frame {
public:
    using SITL::Frame::Frame;

    void public_load_frame_params(const char *model_json) {
        load_frame_params(model_json);
    }

    float get_capacity() { return model.battCapacityAh; }
    float get_max_volt() { return model.maxVoltage; }
    float get_resistance() { return model.refBatRes; }

};

Frame_helper frame {nullptr, 0, nullptr};
SITL::Battery battery;

void setup(void)
{
    uint8_t argc;
    char * const *argv;

    hal.util->commandline_arguments(argc, argv);

    if (argc <= 2) {
        ::printf("pass .json model definition path and current draw\n");
        exit(0);
    }

    // parse JSON vehicle definition
    frame.public_load_frame_params(argv[1]);

    // parse current draw
    const float current = strtof(argv[2], NULL);

    const float amp_hour_capacity = frame.get_capacity();
    const float resistance = frame.get_resistance();
    const float max_voltage = frame.get_max_volt();
    const float min_voltage = max_voltage * 0.7;

    ::printf("Simulating %0.2fv, %0.2f ah battery with resistance of %f\n", max_voltage, amp_hour_capacity, resistance);
    ::printf("Voltage from %0.2f to %0.2f with constant current draw of %0.2f\n", max_voltage, min_voltage, current);

    // setup battery model
    battery.setup(amp_hour_capacity, resistance, max_voltage);
    battery.init_voltage(max_voltage);

    uint64_t time = 0;
    hal.scheduler->stop_clock(time);
    const double time_step = 0.05;

    ::printf("time, voltage\n");
    while (battery.get_voltage() >= min_voltage) {
        battery.set_current(current);

        ::printf("%0.2f, %0.2f\n", time * 1.0e-6, battery.get_voltage());

        time += time_step*1e6;
        hal.scheduler->stop_clock(time);
    }
}

void loop(void)
{
    exit(0);
}

AP_HAL_MAIN();
