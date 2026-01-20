# Parameter Revert

This script implements a easy parameter reversion system to help with
manual in-flight tuning. It allows you to do a wide range of manual
tuning while flying and if you get in trouble (eg. an oscillation)
then you can use a switch to instantly revert all the parameter
changes to the values from startup.

# Parameters

The script adds 2 parameters to control it's behaviour. The parameters
are:

## PREV_ENABLE

this must be set to 1 to enable the script

## PREV_RC_FUNC

The RCz_OPTIONS scripting function binding to be used for this script.
Default RCz_OPTIONS binding is 300 (scripting1).

# Operation

Install the script in the APM/SCRIPTS folder on your microSD (you can
use mavFTP for that). Then reboot and re-fetch parameters. You will
find you now have PREV_ENABLE and PREV_RC_FUNC parameters.

Set PREV_ENABLE to 1 and set PREV_RC_FUNC to an available RC
option. You would typically use 300 if not used by another
script. Then set RCn_OPTION for your chosen R/C channel to revert
parameter to the PREV_RC_FUNC value (eg. 300).

Now reboot to start the script. To test it try changing one of the
covered parameter values while on the ground, then trigger the
reversion with your R/C switch. Then fetch your parameters and you
will see it has been reverted. You will see a message "Reverted N
parameters" in the messages tab when this happens.

# Covered Parameters

The script covers the following parameters on quadplanes:

 - Q_A_RAT_RLL_*
 - Q_A_RAT_PIT_*
 - Q_A_RAT_YAW_*
 - Q_A_ANG_RLL_P
 - Q_A_ANG_PIT_P
 - Q_A_ANG_YAW_P
 - Q_A_RATE_*_MAX
 - Q_P_D_ACC_*
 - Q_P_D_VEL_*
 - Q_P_D_POS_*
 - Q_P_NE_VEL_*
 - Q_P_NE_POS_*

The script covers the following parameters on copters:

 - ATC_RAT_RLL_*
 - ATC_RAT_PIT_*
 - ATC_RAT_YAW_*
 - ATC_ANG_RLL_P
 - ATC_ANG_PIT_P
 - ATC_ANG_YAW_P
 - ATC_RATE_*_MAX
 - PSC_D_ACC_*
 - PSC_D_VEL_*
 - PSC_D_POS_*
 - PSC_NE_VEL_*
 - PSC_NE_POS_*

For fixed wing the following parameters are covered:

 - RLL_RATE_*
 - PTCH_RATE_*
 - RLL2SRV_TCONST
 - PTCH2SRV_TCONST
 - all TECS parameters
