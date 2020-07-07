#include "AP_Vehicle.h"

#include <AP_BLHeli/AP_BLHeli.h>
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

#if HAL_GYROFFT_ENABLED
    // @Group: FFT_
    // @Path: ../AP_GyroFFT/AP_GyroFFT.cpp
    AP_SUBGROUPINFO(gyro_fft, "FFT_",  2, AP_Vehicle, AP_GyroFFT),
#endif

#if HAL_VISUALODOM_ENABLED
    // @Group: VISO
    // @Path: ../AP_VisualOdom/AP_VisualOdom.cpp
    AP_SUBGROUPINFO(visual_odom, "VISO",  3, AP_Vehicle, AP_VisualOdom),
#endif

    // @Group: VTX_
    // @Path: ../AP_RCTelemetry/AP_VideoTX.cpp
    AP_SUBGROUPINFO(vtx, "VTX_",  4, AP_Vehicle, AP_VideoTX),

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

    // this is here for Plane; its failsafe_check method requires the
    // RC channels to be set as early as possible for maximum
    // survivability.
    set_control_channels();

    // initialise serial manager as early as sensible to get
    // diagnostic output during boot process.  We have to initialise
    // the GCS singleton first as it sets the global mavlink system ID
    // which may get used very early on.
    gcs().init();

    // initialise serial ports
    serial_manager.init();
    gcs().setup_console();

    // Register scheduler_delay_cb, which will run anytime you have
    // more than 5ms remaining in your call to hal.scheduler->delay
    hal.scheduler->register_delay_callback(scheduler_delay_callback, 5);

    // init_ardupilot is where the vehicle does most of its initialisation.
    init_ardupilot();

    // gyro FFT needs to be initialized really late
#if HAL_GYROFFT_ENABLED
    gyro_fft.init(AP::scheduler().get_loop_period_us());
#endif
#if HAL_RUNCAM_ENABLED
    runcam.init();
#endif
#if HAL_HOTT_TELEM_ENABLED
    hott_telem.init();
#endif
#if HAL_VISUALODOM_ENABLED
    // init library used for visual position estimation
    visual_odom.init();
#endif
    vtx.init();

#if AP_PARAM_KEY_DUMP
    AP_Param::show_all(hal.console, true);
#endif
}

void AP_Vehicle::loop()
{
    scheduler.loop();
    G_Dt = scheduler.get_loop_period_s();
}

/*
 fast loop callback for all vehicles. This will get called at the end of any vehicle-specific fast loop.
 */
void AP_Vehicle::fast_loop()
{
#if HAL_GYROFFT_ENABLED
    gyro_fft.sample_gyros();
#endif
}

/*
  common scheduler table for fast CPUs - all common vehicle tasks
  should be listed here, along with how often they should be called (in hz)
  and the maximum time they are expected to take (in microseconds)
 */
const AP_Scheduler::Task AP_Vehicle::scheduler_tasks[] = {
#if HAL_RUNCAM_ENABLED
    SCHED_TASK_CLASS(AP_RunCam,    &vehicle.runcam,         update,                   50, 50),
#endif
#if HAL_GYROFFT_ENABLED
    SCHED_TASK_CLASS(AP_GyroFFT,   &vehicle.gyro_fft,       update,                  400, 50),
    SCHED_TASK_CLASS(AP_GyroFFT,   &vehicle.gyro_fft,       update_parameters,         1, 50),
#endif
    SCHED_TASK(update_dynamic_notch,                   200,    200),
    SCHED_TASK_CLASS(AP_VideoTX,   &vehicle.vtx,            update,                    2, 100),
    SCHED_TASK(send_watchdog_reset_statustext,         0.1,     20),
};

void AP_Vehicle::get_common_scheduler_tasks(const AP_Scheduler::Task*& tasks, uint8_t& num_tasks)
{
    tasks = scheduler_tasks;
    num_tasks = ARRAY_SIZE(scheduler_tasks);
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

// if there's been a watchdog reset, notify the world via a statustext:
void AP_Vehicle::send_watchdog_reset_statustext()
{
    if (!hal.util->was_watchdog_reset()) {
        return;
    }
    const AP_HAL::Util::PersistentData &pd = hal.util->last_persistent_data;
    gcs().send_text(MAV_SEVERITY_CRITICAL,
                    "WDG: T%d SL%u FL%u FT%u FA%x FTP%u FLR%x FICSR%u MM%u MC%u IE%u IEC%u TN:%.4s",
                    pd.scheduler_task,
                    pd.semaphore_line,
                    pd.fault_line,
                    pd.fault_type,
                    (unsigned)pd.fault_addr,
                    pd.fault_thd_prio,
                    (unsigned)pd.fault_lr,
                    (unsigned)pd.fault_icsr,
                    pd.last_mavlink_msgid,
                    pd.last_mavlink_cmd,
                    (unsigned)pd.internal_errors,
                    (unsigned)pd.internal_error_count,
                    pd.thread_name4
        );
}

// @LoggerMessage: FTN
// @Description: Filter Tuning Messages
// @Field: TimeUS: microseconds since system startup
// @Field: NDn: number of active dynamic harmonic notches
// @Field: DnF1: dynamic harmonic notch centre frequency for motor 1
// @Field: DnF2: dynamic harmonic notch centre frequency for motor 2
// @Field: DnF3: dynamic harmonic notch centre frequency for motor 3
// @Field: DnF4: dynamic harmonic notch centre frequency for motor 4
void AP_Vehicle::write_notch_log_messages() const
{
    const float* notches = ins.get_gyro_dynamic_notch_center_frequencies_hz();
    AP::logger().Write(
        "FTN", "TimeUS,NDn,DnF1,DnF2,DnF3,DnF4", "s-zzzz", "F-----", "QBffff", AP_HAL::micros64(), ins.get_num_gyro_dynamic_notch_center_frequencies(),
            notches[0], notches[1], notches[2], notches[3]);
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
