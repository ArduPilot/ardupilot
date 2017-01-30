/*
  failsafe support
  Andrew Tridgell, December 2011
 */

#include "Rover.h"

/*
  our failsafe strategy is to detect main loop lockup and switch to
  passing inputs straight from the RC inputs to RC outputs.
 */

/*
  this failsafe_check function is called from the core timer interrupt
  at 1kHz.
 */
void Rover::failsafe_check()
{
    static uint16_t last_mainLoop_count;
    static uint32_t last_timestamp;
    static bool in_failsafe;
    uint32_t tnow = AP_HAL::micros();

    if (mainLoop_count != last_mainLoop_count) {
        // the main loop is running, all is OK
        last_mainLoop_count = mainLoop_count;
        last_timestamp = tnow;
        in_failsafe = false;
        return;
    }

    if (tnow - last_timestamp > 200000) {
        // we have gone at least 0.2 seconds since the main loop
        // ran. That means we're in trouble, or perhaps are in
        // an initialisation routine or log erase. Start passing RC
        // inputs through to outputs
        in_failsafe = true;
    }

    if (in_failsafe && tnow - last_timestamp > 20000 &&
        channel_throttle->read() >= (uint16_t)g.fs_throttle_value) {
        // pass RC inputs to outputs every 20ms
        last_timestamp = tnow;
        hal.rcin->clear_overrides();
        uint8_t start_ch = 0;
        for (uint8_t ch=start_ch; ch < 4; ch++) {
            hal.rcout->write(ch, hal.rcin->read(ch));
        }
        SRV_Channels::copy_radio_in_out(SRV_Channel::k_manual, true);
    }
}

#if ADVANCED_FAILSAFE == ENABLED
/*
   check for AFS failsafe check
 */
void Rover::afs_fs_check(void)
{
    // perform AFS failsafe checks
    g2.afs.check(rover.last_heartbeat_ms, false, failsafe.last_valid_rc_ms);  // Rover don't have fence
}
#endif
