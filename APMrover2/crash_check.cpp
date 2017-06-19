#include "Rover.h"

// Code to detect a crash or block
#define CRASH_CHECK_TRIGGER_SEC     2       // 2 seconds blocked indicates a crash
#define CRASH_CHECK_THROTTLE_MIN    5.0f    // vehicle must have a throttle greater that 5% to be considered crashed
#define CRASH_CHECK_VEL_MIN         0.08f   // vehicle must have a velocity under 0.08 m/s or rad/s to be considered crashed

// crash_check - disarms motors if a crash or block has been detected
// crashes are detected by the vehicle being static (no speed) for more than CRASH_CHECK_TRIGGER_SEC and motor are running
// called at 10Hz
void Rover::crash_check()
{
  static uint16_t crash_counter;  // number of iterations vehicle may have been crashed

  // return immediately if disarmed, or crash checking disabled or in HOLD mode
  if (!arming.is_armed() || g.fs_crash_check == FS_CRASH_DISABLE || (control_mode != GUIDED && control_mode != AUTO)) {
    crash_counter = 0;
    return;
  }

  // TODO : Check if min vel can be calculated
  // min_vel = ( CRASH_CHECK_THROTTLE_MIN * g.speed_cruise) / g.throttle_cruise;

  if ((ahrs.groundspeed() >= CRASH_CHECK_VEL_MIN) ||        // Check velocity
      (fabsf(ahrs.get_gyro().z) >= CRASH_CHECK_VEL_MIN) ||  // Check turn speed
      ((100 * fabsf(SRV_Channels::get_output_norm(SRV_Channel::k_throttle))) < CRASH_CHECK_THROTTLE_MIN)) {
    crash_counter = 0;
    return;
  }

  // we may be crashing
  crash_counter++;

  // check if crashing for 2 seconds
  if (crash_counter >= (CRASH_CHECK_TRIGGER_SEC * 10)) {
    // log an error in the dataflash
    Log_Write_Error(ERROR_SUBSYSTEM_CRASH_CHECK, ERROR_CODE_CRASH_CHECK_CRASH);
    // send message to gcs
    gcs_send_text(MAV_SEVERITY_EMERGENCY, "Crash: Going to HOLD");
    // change mode to hold and disarm
    set_mode(HOLD);
    if (g.fs_crash_check == FS_CRASH_HOLD_AND_DISARM) {
      disarm_motors();
    }
  }
}
