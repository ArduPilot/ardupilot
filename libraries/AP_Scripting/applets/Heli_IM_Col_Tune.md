# Heli_IM_Col_Tune Lua Script

This is a useful tool when setting up a traditional heli.  It is used to tune the `IM_STAB_COL_<X>` input curve.  Typically, it is awkward to tune the `IM_STAB_COL_2` and `IM_STAB_COL_3` parameters to get the heli hovering around mid-stick with the desired finesse to maintain the hover, unless you have someone working on the GCS for you.  This tool provides a more intuitive approach using two pots on your RC transmitter.

## How it works

One pot controls the output value that correlates to the 50% input (i.e. mid-stick position).  The other pot will control the sensitivity of the collective from the 40% to the 60% input values (i.e. the gradient of the curve between 40% and 60%).  It then converts the 50% value and the gradient to appropriate values for `IM_STAB_COL_2` and `IM_STAB_COL_3`.  Once you are happy with the tune, flick the save switch on the transmitter and script will save the parameter values and exit the script.

The script updates the parameters every 0.5 seconds based on the position of the pots, allowing for easy tuning until the desired 'feel' is achieved.

It is worth noting that this script preserves the constraints set by ArduPilot:

0 <= `IM_STAB_COL_1` < `IM_STAB_COL_2` < `IM_STAB_COL_3` < `IM_STAB_COL_4` <= 100

As a result, if `IM_STAB_COL_1` or `IM_STAB_COL_4` are set to values that would violate the above constraint, after either `IM_STAB_COL_2` or `IM_STAB_COL_3` are changed by the script, then this script will modify the values of COL_1 and COL_4 to ensure this rule is met.

## Limits

The limits imposed on the values controlled by the pots are as follows:

- The mid-stick output value can be changed +/- 30%, centered on the initial mid-stick output value when the script is initialised.

- The maximum gradient is +100%

- The minimum gradient is +5%.

## Setup and Use

 - If you have not done so already, follow the instructions on ArduPilot's wiki page to enable scripting.

 - To get appropriate feedback from the script it is advised to set `SCR_DEBUG_LVL` to 3.

 - Load Heli_IM_Col_Tune.lua to the 'scripts' folder on your flight controller.

 - Set `RC<X>_OPTION` to 300 for the RC input channel that corresponds to the pot that you would like to control the mid-stick output value.

 - Set `RC<X>_OPTION` to 301 for the RC input channel that corresponds to the pot that you would like to use to set the collective's sensitivity, around the mid-stick value.

 - Set `RC<X>_OPTION` to 302 for the RC input channel that corresponds to the switch that you would like to use to save the parameters and exit the script.

 - Note that the script will only register the assigned RC functions when first booting up the flight controller.  Any changes to the RC allocations will require a reboot.

 - Note that `RC<X>_TRIM` values are ignored.

 - It is advisable to start with your `IM_STAB_COL_<X>` curve set to defaults to start (0/40/60/100 respectively).  Similarly, it is advisable to start with the 50% input pot set to the half-way point and your gradient pot set to maximum.  This way, you will start with your parameter values at their default values.

 - Ensure your save switch is set to low.

 - Reboot your flight controller.

 - Fly in stabilize and tune to get your desired collective 'feel' by adjusting the two pots.  If you plan to use higher automated modes than stabilize, it is sensible to get your heli hovering at mid-stick to ensure a smoother mode change into and out of 'auto-collective' modes (e.g. AltHold and Loiter).  Once happy with your tune set the save switch to high.  The script will save the parameters and exit the script.  If you do not save the params they will not persist after a reboot.

 - Once you have tuned your `IM_STAB_COL_2` and `IM_STAB_COL_3` values using the script, remove the script from your flight controller and reset the `RC<X>_OPTION` parameters.

 - If you wish to make any adjustments to `IM_STAB_COL_1` and `IM_STAB_COL_4` you can now do so using the normal parameters.

## Output Messages

### "LUA: IM_COL tune complete"

The script has finished and will not run again without a reboot.

### "LUA: RC channels not set"

One or more RC channels are not set to the scripting channels (300,301, or 302).  Set them and reboot flight controller.

### "LUA: IM_COL setter stopped"

There has been a fault and the script has exited.  This is caused by the script being unable to read the `IM_STB_VOL_<X>` parameter values.  Another message will precede this one, stating which parameter could not be retrieved.

### "LUA: params saved"

Parameters successfully saved.

### "LUA: param set failed"

Failed to set parameter.  Script will persist to try and set parameters without exiting.  If this message continues to show, consider landing and investigating the underlying issue.