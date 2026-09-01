# Parameter Rate Switch

This script implements a easy parameter switching system to help with
in-flight tuning comparison. It allows you tune a particular parameters set
and then switch to an alternative tune to compare before and after

# Parameters

The script adds 2 parameters to control it's behaviour. The parameters
are:

## RTSW_ENABLE

this must be set to 1 to enable the script

## RTSW_RC_FUNC

The RCz_OPTION scripting function binding to be used for this script.
Default RCz_OPTION binding is 300 (scripting1).

The script also adds backups of all of the switched parameters using the prefix "X_"
This allows the tune to be persistent.

# Operation

Install the script in the APM/SCRIPTS folder on your microSD (you can
use mavFTP for that). Then reboot and re-fetch parameters. You will
find you now have PREV_ENABLE and PREV_RC_FUNC parameters.

Set RTSW_ENABLE to 1 and set RTSW_RC_FUNC to an available RC
option. You would typically use 300 if not used by another
script. Then set RCn_OPTION for your chosen R/C channel to switch 
parameters to the PREV_RC_FUNC value (eg. 300).

Now reboot to start the script. To test it try changing one of the
covered parameter values while on the ground, then trigger the
switch with your R/C switch. Then fetch your parameters and you
will see it has been switched. You will see a message "Switched N
parameters" in the messages tab when this happens.

# Covered Parameters

  ATC_ACCEL_P_MAX
  ATC_ACCEL_R_MAX
  ATC_ACCEL_Y_MAX
  ATC_ANG_PIT_P
  ATC_ANG_RLL_P
  ATC_ANG_YAW_P
  ATC_RAT_PIT_P
  ATC_RAT_PIT_I
  ATC_RAT_PIT_D
  ATC_RAT_PIT_D_FF
  ATC_RAT_PIT_FLTD
  ATC_RAT_RLL_P
  ATC_RAT_RLL_I
  ATC_RAT_RLL_D
  ATC_RAT_RLL_D_FF
  ATC_RAT_RLL_FLTD
  ATC_RAT_YAW_P
  ATC_RAT_YAW_I
  ATC_RAT_YAW_D
  ATC_RAT_YAW_D_FF
  ATC_RAT_YAW_FLTD
  ATC_THR_G_BOOST
  ACRO_RP_RATE_TC
  ACRO_Y_RATE_TC
  FSTRATE_ENABLE
  FSTRATE_DIV
  MOT_SPIN_MIN
  MOT_SPIN_MAX
  MOT_THST_EXPO
  SERVO_DSHOT_RATE

  