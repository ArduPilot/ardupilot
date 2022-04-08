#include "AP_Vehicle.h"

#include <AP_BLHeli/AP_BLHeli.h>
#include <AP_Common/AP_FWVersion.h>
#include <AP_Arming/AP_Arming.h>
#include <AP_Frsky_Telem/AP_Frsky_Parameters.h>
#include <AP_Logger/AP_Logger.h>
#include <AP_Mission/AP_Mission.h>
#include <AP_OSD/AP_OSD.h>
#include <SRV_Channel/SRV_Channel.h>
#if CONFIG_HAL_BOARD == HAL_BOARD_CHIBIOS
#include <AP_HAL_ChibiOS/sdcard.h>
#endif

#define SCHED_TASK(func, rate_hz, max_time_micros, prio) SCHED_TASK_CLASS(AP_Vehicle, &vehicle, func, rate_hz, max_time_micros, prio)

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
    // @Path: ../AP_VideoTX/AP_VideoTX.cpp
    AP_SUBGROUPINFO(vtx, "VTX_",  4, AP_Vehicle, AP_VideoTX),

#if HAL_MSP_ENABLED
    // @Group: MSP
    // @Path: ../AP_MSP/AP_MSP.cpp
    AP_SUBGROUPINFO(msp, "MSP",  5, AP_Vehicle, AP_MSP),
#endif

#if HAL_WITH_FRSKY_TELEM_BIDIRECTIONAL
    // @Group: FRSKY_
    // @Path: ../AP_Frsky_Telem/AP_Frsky_Parameters.cpp
    AP_SUBGROUPINFO(frsky_parameters, "FRSKY_", 6, AP_Vehicle, AP_Frsky_Parameters),
#endif

#if HAL_GENERATOR_ENABLED
    // @Group: GEN_
    // @Path: ../AP_Generator/AP_Generator.cpp
    AP_SUBGROUPINFO(generator, "GEN_", 7, AP_Vehicle, AP_Generator),
#endif

#if HAL_EXTERNAL_AHRS_ENABLED
    // @Group: EAHRS
    // @Path: ../AP_ExternalAHRS/AP_ExternalAHRS.cpp
    AP_SUBGROUPINFO(externalAHRS, "EAHRS", 8, AP_Vehicle, AP_ExternalAHRS),
#endif

#if HAL_EFI_ENABLED
    // @Group: EFI
    // @Path: ../AP_EFI/AP_EFI.cpp
    AP_SUBGROUPINFO(efi, "EFI", 9, AP_Vehicle, AP_EFI),
#endif

#if AP_AIRSPEED_ENABLED
    // @Group: ARSPD
    // @Path: ../AP_Airspeed/AP_Airspeed.cpp
    AP_SUBGROUPINFO(airspeed, "ARSPD", 10, AP_Vehicle, AP_Airspeed),
#endif

    // @Group: CUST_ROT
    // @Path: ../AP_CustomRotations/AP_CustomRotations.cpp
    AP_SUBGROUPINFO(custom_rotations, "CUST_ROT", 11, AP_Vehicle, AP_CustomRotations),

    AP_GROUPEND
};

// reference to the vehicle. using AP::vehicle() here does not work on clang
#if APM_BUILD_TYPE(APM_BUILD_UNKNOWN) || APM_BUILD_TYPE(APM_BUILD_AP_Periph)
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

#if CONFIG_HAL_BOARD == HAL_BOARD_CHIBIOS
    if (AP_BoardConfig::get_sdcard_slowdown() != 0) {
        // user wants the SDcard slower, we need to remount
        sdcard_stop();
        sdcard_retry();
    }
#endif

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

#if HAL_MSP_ENABLED
    // call MSP init before init_ardupilot to allow for MSP sensors
    msp.init();
#endif

#if HAL_EXTERNAL_AHRS_ENABLED
    // call externalAHRS init before init_ardupilot to allow for external sensors
    externalAHRS.init();
#endif

    // init_ardupilot is where the vehicle does most of its initialisation.
    init_ardupilot();

#if AP_AIRSPEED_ENABLED
    airspeed.init();
    if (airspeed.enabled()) {
        airspeed.calibrate(true);
    } 
#if APM_BUILD_TYPE(APM_BUILD_ArduPlane)
    else {
        GCS_SEND_TEXT(MAV_SEVERITY_WARNING,"No airspeed sensor present or enabled");
    }
#endif
#endif  // AP_AIRSPEED_ENABLED

#if !APM_BUILD_TYPE(APM_BUILD_Replay)
    SRV_Channels::init();
#endif

    // gyro FFT needs to be initialized really late
#if HAL_GYROFFT_ENABLED
    gyro_fft.init(AP::scheduler().get_loop_rate_hz());
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

#if HAL_SMARTAUDIO_ENABLED
    smartaudio.init();
#endif

#if AP_PARAM_KEY_DUMP
    AP_Param::show_all(hal.console, true);
#endif

    send_watchdog_reset_statustext();

#if HAL_GENERATOR_ENABLED
    generator.init();
#endif

// init EFI monitoring
#if HAL_EFI_ENABLED
    efi.init();
#endif

    custom_rotations.init();

    gcs().send_text(MAV_SEVERITY_INFO, "ArduPilot Ready");
}

void AP_Vehicle::loop()
{
    scheduler.loop();
    G_Dt = scheduler.get_loop_period_s();

    if (!done_safety_init) {
        /*
          disable safety if requested. This is delayed till after the
          first loop has run to ensure that all servos have received
          an update for their initial values. Otherwise we may end up
          briefly driving a servo to a position out of the configured
          range which could damage hardware
        */
        done_safety_init = true;
        BoardConfig.init_safety();

        // send RC output mode info if available
        char banner_msg[50];
        if (hal.rcout->get_output_mode_banner(banner_msg, sizeof(banner_msg))) {
            GCS_SEND_TEXT(MAV_SEVERITY_INFO, "%s", banner_msg);
        }
    }
    const uint32_t new_internal_errors = AP::internalerror().errors();
    if(_last_internal_errors != new_internal_errors) {
        AP::logger().Write_Error(LogErrorSubsystem::INTERNAL_ERROR, LogErrorCode::INTERNAL_ERRORS_DETECTED);
        gcs().send_text(MAV_SEVERITY_CRITICAL, "Internal Errors %x", (unsigned)new_internal_errors);
        _last_internal_errors = new_internal_errors;
    }
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
  scheduler table - all regular tasks apart from the fast_loop()
  should be listed here.

  All entries in this table must be ordered by priority.

  This table is interleaved with the table presnet in each of the
  vehicles to determine the order in which tasks are run.  Convenience
  methods SCHED_TASK and SCHED_TASK_CLASS are provided to build
  entries in this structure:

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
const AP_Scheduler::Task AP_Vehicle::scheduler_tasks[] = {
#if AP_AIRSPEED_ENABLED
    SCHED_TASK_CLASS(AP_Airspeed,  &vehicle.airspeed,       update,                   10, 100, 41),    // NOTE: the priority number here should be right before Plane's calc_airspeed_errors
#endif
#if HAL_RUNCAM_ENABLED
    SCHED_TASK_CLASS(AP_RunCam,    &vehicle.runcam,         update,                   50, 50, 200),
#endif
#if HAL_GYROFFT_ENABLED
    SCHED_TASK_CLASS(AP_GyroFFT,   &vehicle.gyro_fft,       update,                  400, 50, 205),
    SCHED_TASK_CLASS(AP_GyroFFT,   &vehicle.gyro_fft,       update_parameters,         1, 50, 210),
#endif
    SCHED_TASK(update_dynamic_notch_at_specified_rate,      LOOP_RATE,                    200, 215),
    SCHED_TASK_CLASS(AP_VideoTX,   &vehicle.vtx,            update,                    2, 100, 220),
    SCHED_TASK(send_watchdog_reset_statustext,         0.1,     20, 225),
#if HAL_WITH_ESC_TELEM
    SCHED_TASK_CLASS(AP_ESC_Telem, &vehicle.esc_telem,      update,                   10,  50, 230),
#endif
#if HAL_GENERATOR_ENABLED
    SCHED_TASK_CLASS(AP_Generator, &vehicle.generator,      update,                   10,  50, 235),
#endif
#if OSD_ENABLED
    SCHED_TASK(publish_osd_info, 1, 10, 240),
#endif
#if HAL_INS_ACCELCAL_ENABLED
    SCHED_TASK(accel_cal_update,      10,    100, 245),
#endif
#if HAL_EFI_ENABLED
    SCHED_TASK_CLASS(AP_EFI,       &vehicle.efi,            update,                   10, 200, 250),
#endif
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
#if APM_BUILD_TYPE(APM_BUILD_Replay)
    // compass.init() delays, so we end up here.
    return;
#endif

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
        if (AP_BoardConfig::in_config_error()) {
            gcs().send_text(MAV_SEVERITY_CRITICAL, "Config Error: fix problem then reboot");
        } else {
            gcs().send_text(MAV_SEVERITY_INFO, "Initialising ArduPilot");
        }
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

bool AP_Vehicle::is_crashed() const
{
    if (AP::arming().is_armed()) {
        return false;
    }
    return AP::arming().last_disarm_method() == AP_Arming::Method::CRASH;
}

// run notch update at either loop rate or 200Hz
void AP_Vehicle::update_dynamic_notch_at_specified_rate()
{
    if (ins.has_harmonic_option(HarmonicNotchFilterParams::Options::LoopRateUpdate)) {
        update_dynamic_notch();
        return;
    }

    // decimated update at 200Hz
    const uint32_t now = AP_HAL::millis();

    if (now - _last_notch_update_ms > 5) {
        _last_notch_update_ms = now;
        update_dynamic_notch();
    }
}

void AP_Vehicle::notify_no_such_mode(uint8_t mode_number)
{
    GCS_SEND_TEXT(MAV_SEVERITY_WARNING,"No such mode %u", mode_number);
    AP::logger().Write_Error(LogErrorSubsystem::FLIGHT_MODE, LogErrorCode(mode_number));
}

// reboot the vehicle in an orderly manner, doing various cleanups and
// flashing LEDs as appropriate
void AP_Vehicle::reboot(bool hold_in_bootloader)
{
    if (should_zero_rc_outputs_on_reboot()) {
        SRV_Channels::zero_rc_outputs();
    }

    // Notify might want to blink some LEDs:
    AP_Notify::flags.firmware_update = 1;
    notify.update();

    // force safety on
    hal.rcout->force_safety_on();

    // flush pending parameter writes
    AP_Param::flush();

    // do not process incoming mavlink messages while we delay:
    hal.scheduler->register_delay_callback(nullptr, 5);

#if CONFIG_HAL_BOARD == HAL_BOARD_SITL
    // need to ensure the ack goes out:
    hal.serial(0)->flush();
#endif

    // delay to give the ACK a chance to get out, the LEDs to flash,
    // the IO board safety to be forced on, the parameters to flush, ...
    hal.scheduler->delay(200);

    hal.scheduler->reboot(hold_in_bootloader);
}

#if OSD_ENABLED
void AP_Vehicle::publish_osd_info()
{
    AP_Mission *mission = AP::mission();
    if (mission == nullptr) {
        return;
    }
    AP_OSD *osd = AP::osd();
    if (osd == nullptr) {
        return;
    }
    AP_OSD::NavInfo nav_info;
    if(!get_wp_distance_m(nav_info.wp_distance)) {
        return;
    }
    float wp_bearing_deg;
    if (!get_wp_bearing_deg(wp_bearing_deg)) {
        return;
    }
    nav_info.wp_bearing = (int32_t)wp_bearing_deg * 100; // OSD expects cd
    if (!get_wp_crosstrack_error_m(nav_info.wp_xtrack_error)) {
        return;
    }
    nav_info.wp_number = mission->get_current_nav_index();
    osd->set_nav_info(nav_info);
}

void AP_Vehicle::get_osd_roll_pitch_rad(float &roll, float &pitch) const
{
    roll = ahrs.roll;
    pitch = ahrs.pitch;
}

#endif

#if HAL_INS_ACCELCAL_ENABLED

#ifndef HAL_CAL_ALWAYS_REBOOT
// allow for forced reboot after accelcal
#define HAL_CAL_ALWAYS_REBOOT 0
#endif

/*
  update accel cal
 */
void AP_Vehicle::accel_cal_update()
{
    if (hal.util->get_soft_armed()) {
        return;
    }
    ins.acal_update();
    // check if new trim values, and set them
    Vector3f trim_rad;
    if (ins.get_new_trim(trim_rad)) {
        ahrs.set_trim(trim_rad);
    }

#if HAL_CAL_ALWAYS_REBOOT
    if (ins.accel_cal_requires_reboot() &&
        !hal.util->get_soft_armed()) {
        hal.scheduler->delay(1000);
        hal.scheduler->reboot(false);
    }
#endif
}
#endif // HAL_INS_ACCELCAL_ENABLED

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
