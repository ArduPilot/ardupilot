/**
 * @file failsafe.pde
 *
 * @author Andrew Tridgell
 * @date December 2011
 * @brief Mainloop lockup failsafe checks - disarm motors in event of crash/lockup.  Other failsafes in events.pde
 */

// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

static bool failsafe_enabled = true;
static uint16_t failsafe_last_mainLoop_count;
static uint32_t failsafe_last_timestamp;
static bool in_failsafe;

//
// failsafe_enable - enable failsafe
//

/**
 * failsafe_enable
 *
 * @return void
 *
 * @brief Main loop lockup failsafe: enable
 */
void failsafe_enable()
{
    failsafe_enabled = true;
    failsafe_last_timestamp = micros();
}

/**
 * failsafe_disable
 *
 * @return void
 *
 * @brief Main loop lockup failsafe disable - used when we know we are going to delay the mainloop significantly
 */
void failsafe_disable()
{
    failsafe_enabled = false;
}

/**
 * failsafe_check
 *
 * @param uint32_t tnow
 * @return void
 *
 * @brief Called at 1Khz - checks for main loop lockups
 */
void failsafe_check(uint32_t tnow)
{
    if (mainLoop_count != failsafe_last_mainLoop_count) {
        // the main loop is running, all is OK
        failsafe_last_mainLoop_count = mainLoop_count;
        failsafe_last_timestamp = tnow;
        in_failsafe = false;
        return;
    }

    if (failsafe_enabled && tnow - failsafe_last_timestamp > 2000000) {
        // motors are running but we have gone 2 second since the
        // main loop ran. That means we're in trouble and should
        // disarm the motors.
        in_failsafe = true;
    }

    if (failsafe_enabled && in_failsafe && tnow - failsafe_last_timestamp > 1000000) {
        // disarm motors every second
        failsafe_last_timestamp = tnow;
        if(motors.armed()) {
            motors.armed(false);
            motors.output();
        }
    }
}
