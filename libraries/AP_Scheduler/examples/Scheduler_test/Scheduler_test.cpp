//
// Simple test for the AP_Scheduler interface
//

#include <AP_HAL/AP_HAL.h>
#include <AP_InertialSensor/AP_InertialSensor.h>
#include <AP_ExternalAHRS/AP_ExternalAHRS.h>
#include <AP_Scheduler/AP_Scheduler.h>
#include <AP_BoardConfig/AP_BoardConfig.h>
#include <AP_Logger/AP_Logger.h>
#include <GCS_MAVLink/GCS_Dummy.h>

const struct AP_Param::GroupInfo        GCS_MAVLINK_Parameters::var_info[] = {
    AP_GROUPEND
};
GCS_Dummy _gcs;

const AP_HAL::HAL& hal = AP_HAL::get_HAL();

AP_Int32 log_bitmask;
AP_Logger AP_Logger{log_bitmask};

class SchedTest {
public:
    void setup();
    void loop();

private:

    AP_InertialSensor ins;
#if HAL_EXTERNAL_AHRS_ENABLED
    AP_ExternalAHRS eAHRS;
#endif // HAL_EXTERNAL_AHRS_ENABLED
    AP_Scheduler scheduler;

    uint32_t ins_counter;
    static const AP_Scheduler::Task scheduler_tasks[];

    void ins_update(void);
    void one_hz_print(void);
    void five_second_call(void);
};

static AP_BoardConfig board_config;
static SchedTest schedtest;

#define SCHED_TASK(func, _interval_ticks, _max_time_micros, _priority) SCHED_TASK_CLASS(SchedTest, &schedtest, func, _interval_ticks, _max_time_micros, _priority)

/*
  scheduler table - all regular tasks should be listed here.

  All entries in this table must be ordered by priority.

  This table is interleaved with the table in AP_Vehicle to determine
  the order in which tasks are run.  Convenience methods SCHED_TASK
  and SCHED_TASK_CLASS are provided to build entries in this structure:

SCHED_TASK arguments:
 - name of static function to call
 - rate (in Hertz) at which the function should be called
 - expected time (in MicroSeconds) that the function should take to run
 - priority (0 through 255, lower number meaning higher priority)

SCHED_TASK_CLASS arguments:
 - class name of method to be called
 - instance on which to call the method
 - method to call on that instance
 - rate (in Hertz) at which the method should be called
 - expected time (in MicroSeconds) that the method should take to run
 - priority (0 through 255, lower number meaning higher priority)

 */
const AP_Scheduler::Task SchedTest::scheduler_tasks[] = {
    SCHED_TASK(ins_update,             50,   1000, 3),
    SCHED_TASK(one_hz_print,            1,   1000, 6),
    SCHED_TASK(five_second_call,      0.2,   1800, 9),
};


void SchedTest::setup(void)
{

    board_config.init();

    ins.init(scheduler.get_loop_rate_hz());

    // initialise the scheduler
    scheduler.init(&scheduler_tasks[0], ARRAY_SIZE(scheduler_tasks), (uint32_t)-1);
}

void SchedTest::loop(void)
{
    // run all tasks
    scheduler.loop();
}

/*
  update inertial sensor, reading data 
 */
void SchedTest::ins_update(void)
{
    ins_counter++;
    ins.update();
}

/*
  print something once a second
 */
void SchedTest::one_hz_print(void)
{
    hal.console->printf("one_hz: t=%lu\n", (unsigned long)AP_HAL::millis());
}

/*
  print something every 5 seconds
 */
void SchedTest::five_second_call(void)
{
    hal.console->printf("five_seconds: t=%lu ins_counter=%u\n", (unsigned long)AP_HAL::millis(), (unsigned)ins_counter);
}

/*
  compatibility with old pde style build
 */
void setup(void);
void loop(void);

void setup(void)
{
    schedtest.setup();
}
void loop(void)
{
    schedtest.loop();
}
AP_HAL_MAIN();
