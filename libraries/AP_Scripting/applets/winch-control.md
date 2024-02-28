# Winch Control

Allows the winch to be deployed or retracted at a fixed speed using an auxiliary switch

# Parameters

WINCH_RATE_UP : rate (in m/s) when retracting line
WINCH_RATE_DN : rate (in m/s) when deploying line
WINCH_RC_FUNC : RCn_OPTION number to use to control winch rate. Default is 300 (Scripting1)

# How To Use

1. set RCx_OPTION to 300 to enable controlling the winch rate from an auxiliary switch
2. set WINCH_RATE_UP to the fixed retract speed (in m/s)
3. set WINCH_RATE_DN to the fixed release speed (in m/s)
4. raise the RC auxiliary switch to retract the winch's line
5. lower the RC auxiliary switch to deploy the winch's line
6. center the RC auxiliary switch to stop the winch

Alternatively Mission Planner's Aux Function screen can be used in place of an actual RC switch
