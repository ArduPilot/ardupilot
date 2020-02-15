#include "AP_Vehicle.h"

#include <AP_Common/AP_FWVersion.h>

#define SCHED_TASK(func, rate_hz, max_time_micros) SCHED_TASK_CLASS(AP_Vehicle, &vehicle, func, rate_hz, max_time_micros)

/*
  2nd group of parameters
 */
const AP_Param::GroupInfo AP_Vehicle::var_info[] = {
#if HAL_RUNCAM_ENABLED
    // @Group: CAM_RC_
    // @Path: ../AP_Camera/AP_RunCam.cpp
    AP_SUBGROUPINFO(runcam, "CAM_RC_", 1, AP_Vehicle, AP_RunCam),
#endif

    AP_GROUPEND
};

// reference to the vehicle. using AP::vehicle() here does not work on clang
#if APM_BUILD_TYPE(APM_BUILD_Replay) || APM_BUILD_TYPE(APM_BUILD_UNKNOWN)
AP_Vehicle& vehicle = *AP_Vehicle::get_singleton();
#else
extern AP_Vehicle& vehicle;
#endif

/*
  setup is called when the sketch starts
 */
void AP_Vehicle::setup()
{
    // load the default values of variables listed in var_info[]
    AP_Param::setup_sketch_defaults();

    // initialise serial port
    serial_manager.init_console();

    hal.console->printf("\n\nInit %s"
                        "\n\nFree RAM: %u\n",
                        AP::fwversion().fw_string,
                        (unsigned)hal.util->available_memory());

    load_parameters();

    // initialise the main loop scheduler
    const AP_Scheduler::Task *tasks;
    uint8_t task_count;
    uint32_t log_bit;
    get_scheduler_tasks(tasks, task_count, log_bit);
    AP::scheduler().init(tasks, task_count, log_bit);

    // time per loop - this gets updated in the main loop() based on
    // actual loop rate
    G_Dt = scheduler.get_loop_period_s();

    // init_ardupilot is where the vehicle does most of its initialisation.
    init_ardupilot();
}

void AP_Vehicle::loop()
{
    scheduler.loop();
    G_Dt = scheduler.get_loop_period_s();
}

/*
  common scheduler table for fast CPUs - all common vehicle tasks
  should be listed here, along with how often they should be called (in hz)
  and the maximum time they are expected to take (in microseconds)
 */
const AP_Scheduler::Task AP_Vehicle::scheduler_tasks[] = {
#if HAL_RUNCAM_ENABLED
    SCHED_TASK_CLASS(AP_RunCam,    &vehicle.runcam,              update,          50,  50),
#endif
};

void AP_Vehicle::get_common_scheduler_tasks(const AP_Scheduler::Task*& tasks, uint8_t& num_tasks)
{
    tasks = scheduler_tasks;
    num_tasks = ARRAY_SIZE(scheduler_tasks);
}

// initialize the vehicle
void AP_Vehicle::init_vehicle()
{
    if (init_done) {
        return;
    }
    init_done = true;
#if HAL_RUNCAM_ENABLED
    runcam.init();
#endif
#if HAL_HOTT_TELEM_ENABLED
    hott_telem.init();
#endif
}


/*
 *  a delay() callback that processes MAVLink packets. We set this as the
 *  callback in long running library initialisation routines to allow
 *  MAVLink to process packets while waiting for the initialisation to
 *  complete
 */
void AP_Vehicle::scheduler_delay_callback()
{
    static uint32_t last_1hz, last_50hz, last_5s;

    AP_Logger &logger = AP::logger();

    // don't allow potentially expensive logging calls:
    logger.EnableWrites(false);

    const uint32_t tnow = AP_HAL::millis();
    if (tnow - last_1hz > 1000) {
        last_1hz = tnow;
        gcs().send_message(MSG_HEARTBEAT);
        gcs().send_message(MSG_SYS_STATUS);
    }
    if (tnow - last_50hz > 20) {
        last_50hz = tnow;
        gcs().update_receive();
        gcs().update_send();
        _singleton->notify.update();
    }
    if (tnow - last_5s > 5000) {
        last_5s = tnow;
        gcs().send_text(MAV_SEVERITY_INFO, "Initialising ArduPilot");
    }

    logger.EnableWrites(true);
}

void AP_Vehicle::register_scheduler_delay_callback()
{
    // Register scheduler_delay_cb, which will run anytime you have
    // more than 5ms remaining in your call to hal.scheduler->delay
    hal.scheduler->register_delay_callback(scheduler_delay_callback, 5);
}

AP_Vehicle *AP_Vehicle::_singleton = nullptr;

AP_Vehicle *AP_Vehicle::get_singleton()
{
    return _singleton;
}

namespace AP {

AP_Vehicle *vehicle()
{
    return AP_Vehicle::get_singleton();
}

};
