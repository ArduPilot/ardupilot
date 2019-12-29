#include "AP_Vehicle.h"

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

