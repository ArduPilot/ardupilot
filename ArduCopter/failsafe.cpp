#include "Copter.h"

//
//  failsafe support
//  Andrew Tridgell, December 2011
//
//  our failsafe strategy is to detect main loop lockup and disarm the motors
//

static bool failsafe_enabled;
static uint16_t failsafe_last_ticks;
static uint32_t failsafe_last_timestamp;
static bool in_failsafe;
// set when failsafe_check() disarms the motors underneath the vehicle
static volatile bool failsafe_disarmed_motors;

//
// failsafe_enable - enable failsafe
//
void Copter::failsafe_enable()
{
    failsafe_enabled = true;
    failsafe_last_timestamp = micros();
}

//
// failsafe_disable - used when we know we are going to delay the mainloop significantly
//
void Copter::failsafe_disable()
{
    failsafe_enabled = false;
}

//
//  failsafe_check - this function is called from the core timer interrupt at 1kHz.
//
void Copter::failsafe_check()
{
    uint32_t tnow = AP_HAL::micros();

    const uint16_t ticks = scheduler.ticks();
    if (ticks != failsafe_last_ticks) {
        // the main loop is running, all is OK
        failsafe_last_ticks = ticks;
        failsafe_last_timestamp = tnow;
        if (in_failsafe) {
            in_failsafe = false;
            LOGGER_WRITE_ERROR(LogErrorSubsystem::CPU, LogErrorCode::FAILSAFE_RESOLVED);
        }
        return;
    }

    if (!in_failsafe && failsafe_enabled && tnow - failsafe_last_timestamp > 2000000) {
        // motors are running but we have gone 2 second since the
        // main loop ran. That means we're in trouble and should
        // disarm the motors->
        in_failsafe = true;
        // reduce motors to minimum (we do not immediately disarm because we want to log the failure)
        if (motors->armed()) {
            motors->output_min();
        }

        LOGGER_WRITE_ERROR(LogErrorSubsystem::CPU, LogErrorCode::FAILSAFE_OCCURRED);
    }

    if (failsafe_enabled && in_failsafe && tnow - failsafe_last_timestamp > 1000000) {
        // disarm motors every second
        failsafe_last_timestamp = tnow;
        if(motors->armed()) {
            motors->armed(false);
            motors->output();
            failsafe_disarmed_motors = true;
        }
    }
}

//
// failsafe_check_disarm - finish a disarm started by failsafe_check().
// That runs from the timer while the main loop may be stuck, so it can
// only stop the motors.  Once the main loop is running again, disarm
// the vehicle so the arming library, soft-armed state and logs agree
//
void Copter::failsafe_check_disarm()
{
    if (!failsafe_disarmed_motors || in_failsafe) {
        return;
    }
    failsafe_disarmed_motors = false;
    // the main loop stopped while we were armed; don't let the
    // vehicle be armed again without a reboot or a forced arm.  The
    // HAL may already have reported it, with more detail
    if ((AP::internalerror().errors() & uint32_t(AP_InternalError::error_t::main_loop_stuck)) == 0) {
        INTERNAL_ERROR(AP_InternalError::error_t::main_loop_stuck);
    }
#if MODE_TURTLE_ENABLED
    // Turtle arms and disarms itself, reversing the motors and turning
    // failsafes off; let it undo that, and stop it re-arming as soon as
    // the arming library and soft-armed state say we are disarmed
    if (flightmode == &mode_turtle) {
        mode_turtle.shutdown_motors();
    }
#endif
    arming.disarm(AP_Arming::Method::CPUFAILSAFE, false);
}


#if AP_COPTER_ADVANCED_FAILSAFE_ENABLED
/*
  check for AFS failsafe check
*/
void Copter::afs_fs_check(void)
{
    // perform AFS failsafe checks
    g2.afs.check(last_radio_update_ms);
}
#endif
