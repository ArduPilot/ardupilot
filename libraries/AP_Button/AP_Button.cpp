/*
   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */


#include "AP_Button.h"
#include <GCS_MAVLink/GCS_MAVLink.h>
#include <GCS_MAVLink/GCS.h>

// very crude debounce method
#define DEBOUNCE_MS 50

extern const AP_HAL::HAL& hal;

AP_Button *AP_Button::_singleton;

const AP_Param::GroupInfo AP_Button::var_info[] = {

    // @Param: ENABLE
    // @DisplayName: Enable button reporting
    // @Description: This enables the button checking module. When this is disabled the parameters for setting button inputs are not visible
    // @Values: 0:Disabled, 1:Enabled
    // @User: Advanced
    AP_GROUPINFO_FLAGS("ENABLE", 0, AP_Button, enable, 0, AP_PARAM_FLAG_ENABLE),

    // @Param: PIN1
    // @DisplayName: First button Pin
    // @Description: Digital pin number for first button input. 
    // @User: Standard
    // @Values: -1:Disabled,50:AUXOUT1,51:AUXOUT2,52:AUXOUT3,53:AUXOUT4,54:AUXOUT5,55:AUXOUT6
    AP_GROUPINFO("PIN1",  1, AP_Button, pin[0], -1),

    // @Param: PIN2
    // @DisplayName: Second button Pin
    // @Description: Digital pin number for second button input. 
    // @User: Standard
    // @Values: -1:Disabled,50:AUXOUT1,51:AUXOUT2,52:AUXOUT3,53:AUXOUT4,54:AUXOUT5,55:AUXOUT6
    AP_GROUPINFO("PIN2",  2, AP_Button, pin[1], -1),

    // @Param: PIN3
    // @DisplayName: Third button Pin
    // @Description: Digital pin number for third button input. 
    // @User: Standard
    // @Values: -1:Disabled,50:AUXOUT1,51:AUXOUT2,52:AUXOUT3,53:AUXOUT4,54:AUXOUT5,55:AUXOUT6
    AP_GROUPINFO("PIN3",  3, AP_Button, pin[2], -1),

    // @Param: PIN4
    // @DisplayName: Fourth button Pin
    // @Description: Digital pin number for fourth button input. 
    // @User: Standard
    // @Values: -1:Disabled,50:AUXOUT1,51:AUXOUT2,52:AUXOUT3,53:AUXOUT4,54:AUXOUT5,55:AUXOUT6
    AP_GROUPINFO("PIN4",  4, AP_Button, pin[3], -1),

    // @Param: REPORT_SEND
    // @DisplayName: Report send time
    // @Description: The duration in seconds that a BUTTON_CHANGE report is repeatedly sent to the GCS regarding a button changing state. Note that the BUTTON_CHANGE message is MAVLink2 only.
    // @User: Standard
    // @Range: 0 3600
    AP_GROUPINFO("REPORT_SEND", 5, AP_Button, report_send_time, 10),

    // @Param: OPTIONS1
    // @DisplayName: Button Pin 1 Options
    // @Description: Options for Pin 1. PWM input detects PWM above or below 1800/1200us instead of logic level. Invert changes HIGH state to be logic low voltage on pin, or below 1200us, if PWM input.
    // @User: Standard
    // @Bitmask: 0:PWM Input,1:InvertInput
    AP_GROUPINFO("OPTIONS1",  6, AP_Button, options[0], 0),

    // @Param: OPTIONS2
    // @DisplayName: Button Pin 2 Options
    // @Description: Options for Pin 2. PWM input detects PWM above or below 1800/1200us instead of logic level. Invert changes HIGH state to be logic low voltage on pin, or below 1200us, if PWM input.
    // @User: Standard
    // @Bitmask: 0:PWM Input,1:InvertInput
    AP_GROUPINFO("OPTIONS2",  7, AP_Button, options[1], 0),

    // @Param: OPTIONS3
    // @DisplayName: Button Pin 3 Options
    // @Description: Options for Pin 3. PWM input detects PWM above or below 1800/1200us instead of logic level. Invert changes HIGH state to be logic low voltage on pin, or below 1200us, if PWM input.
    // @Bitmask: 0:PWM Input,1:InvertInput
    AP_GROUPINFO("OPTIONS3",  8, AP_Button, options[2], 0),

    // @Param: OPTIONS4
    // @DisplayName: Button Pin 4 Options
    // @Description: Options for Pin 4. PWM input detects PWM above or below 1800/1200us instead of logic level. Invert changes HIGH state to be logic low voltage on pin, or below 1200us, if PWM input.
    // @User: Standard
    // @Bitmask: 0:PWM Input,1:InvertInput
    AP_GROUPINFO("OPTIONS4",  9, AP_Button, options[3], 0),

    // @Param: FUNC1
    // @DisplayName: Button Pin 1 RC Channel function
    // @Description: Auxiliary RC Options function executed on pin change
    // @Values{Copter}: 0:Do Nothing, 2:Flip, 3:Simple Mode, 4:RTL, 5:Save Trim, 7:Save WP, 9:Camera Trigger, 10:RangeFinder, 11:Fence, 13:Super Simple Mode, 14:Acro Trainer, 15:Sprayer, 16:Auto, 17:AutoTune, 18:Land, 19:Gripper, 21:Parachute Enable, 22:Parachute Release, 23:Parachute 3pos, 24:Auto Mission Reset, 25:AttCon Feed Forward, 26:AttCon Accel Limits, 27:Retract Mount, 28:Relay On/Off, 29:Landing Gear, 30:Lost Copter Sound, 31:Motor Emergency Stop, 32:Motor Interlock, 33:Brake, 34:Relay2 On/Off, 35:Relay3 On/Off, 36:Relay4 On/Off, 37:Throw, 38:ADSB Avoidance En, 39:PrecLoiter, 40:Proximity Avoidance, 41:ArmDisarm, 42:SmartRTL, 43:InvertedFlight, 46:RC Override Enable, 47:User Function 1, 48:User Function 2, 49:User Function 3, 52:Acro, 55:Guided, 56:Loiter, 57:Follow, 58:Clear Waypoints, 60:ZigZag, 61:ZigZag SaveWP, 62:Compass Learn, 65:GPS Disable, 66:Relay5, 67:Relay6, 68:Stabilize, 69:PosHold, 70:AltHold, 71:FlowHold, 72:Circle, 73:Drift, 75:SurfaceTrackingUpDown, 76:Standby Mode, 78:RunCam Control, 79:RunCam OSD Control, 80:Viso Align, 81:Disarm, 83:ZigZag Auto, 84:Air Mode, 85:Generator, 90:EKF Pos Source, 100:KillIMU1, 101:KillIMU2, 102:Camera Mode Toggle, 105:GPS Disable Yaw, 300:Scripting1, 301:Scripting2, 302:Scripting3, 303:Scripting4, 304:Scripting5, 305:Scripting6, 306:Scripting7, 307:Scripting8
    // @Values{Rover}: 0:Do Nothing, 4:RTL, 5:Save Trim, 7:Save WP, 9:Camera Trigger, 11:Fence, 16:Auto, 19:Gripper, 24:Auto Mission Reset, 27:Retract Mount, 28:Relay On/Off, 30:Lost Rover Sound, 31:Motor Emergency Stop, 34:Relay2 On/Off, 35:Relay3 On/Off, 36:Relay4 On/Off, 40:Proximity Avoidance, 41:ArmDisarm, 42:SmartRTL, 46:RC Override Enable, 50:LearnCruise, 51:Manual, 52:Acro, 53:Steering, 54:Hold, 55:Guided, 56:Loiter, 57:Follow, 58:Clear Waypoints, 59:Simple Mode, 62:Compass Learn, 63:Sailboat Tack, 65:GPS Disable, 66:Relay5, 67:Relay6, 74:Sailboat motoring 3pos, 78:RunCam Control, 79:RunCam OSD Control, 80:Viso Align, 81:Disarm, 90:EKF Pos Source, 100:KillIMU1, 101:KillIMU2, 102:Camera Mode Toggle, 105:GPS Disable Yaw, 201:Roll, 202:Pitch, 203:Walking Height, 207:MainSail, 208:Flap, 300:Scripting1, 301:Scripting2, 302:Scripting3, 303:Scripting4, 304:Scripting5, 305:Scripting6, 306:Scripting7, 307:Scripting8
    // @Values{Plane}: 0:Do Nothing, 4:ModeRTL, 9:Camera Trigger, 16:ModeAuto, 24:Auto Mission Reset, 27:Retract Mount, 28:Relay On/Off, 29:Landing Gear, 30:Lost Plane Sound, 31:Motor Emergency Stop, 34:Relay2 On/Off, 35:Relay3 On/Off, 36:Relay4 On/Off, 38:ADSB Avoidance En, 41:ArmDisarm, 43:InvertedFlight, 46:RC Override Enable, 51:ModeManual, 55:ModeGuided, 56:ModeLoiter, 58:Clear Waypoints, 62:Compass Learn, 64:Reverse Throttle, 65:GPS Disable, 66:Relay5, 67:Relay6, 72:ModeCircle, 77:ModeTakeoff, 78:RunCam Control, 79:RunCam OSD Control, 81:Disarm, 82:QAssist 3pos, 84:Air Mode, 85:Generator, 86: Non Auto Terrain Follow Disable, 87:Crow Select, 88:Soaring Enable, 89:Landing Flare, 90:EKF Pos Source, 91:Airspeed Ratio Calibration, 92:FBWA, 100:KillIMU1, 101:KillIMU2, 102:Camera Mode Toggle, 105:GPS Disable Yaw, 208:Flap, 209: Forward Throttle, 300:Scripting1, 301:Scripting2, 302:Scripting3, 303:Scripting4, 304:Scripting5, 305:Scripting6, 306:Scripting7, 307:Scripting8
    // @User: Standard
    AP_GROUPINFO("FUNC1",  10, AP_Button, pin_func[0], (uint16_t)RC_Channel::AUX_FUNC::DO_NOTHING),

    // @Param: FUNC2
    // @DisplayName: Button Pin 2 RC Channel function
    // @Description: Auxiliary RC Options function executed on pin change
    // @Values{Copter}: 0:Do Nothing, 2:Flip, 3:Simple Mode, 4:RTL, 5:Save Trim, 7:Save WP, 9:Camera Trigger, 10:RangeFinder, 11:Fence, 13:Super Simple Mode, 14:Acro Trainer, 15:Sprayer, 16:Auto, 17:AutoTune, 18:Land, 19:Gripper, 21:Parachute Enable, 22:Parachute Release, 23:Parachute 3pos, 24:Auto Mission Reset, 25:AttCon Feed Forward, 26:AttCon Accel Limits, 27:Retract Mount, 28:Relay On/Off, 29:Landing Gear, 30:Lost Copter Sound, 31:Motor Emergency Stop, 32:Motor Interlock, 33:Brake, 34:Relay2 On/Off, 35:Relay3 On/Off, 36:Relay4 On/Off, 37:Throw, 38:ADSB Avoidance En, 39:PrecLoiter, 40:Proximity Avoidance, 41:ArmDisarm, 42:SmartRTL, 43:InvertedFlight, 46:RC Override Enable, 47:User Function 1, 48:User Function 2, 49:User Function 3, 52:Acro, 55:Guided, 56:Loiter, 57:Follow, 58:Clear Waypoints, 60:ZigZag, 61:ZigZag SaveWP, 62:Compass Learn, 65:GPS Disable, 66:Relay5, 67:Relay6, 68:Stabilize, 69:PosHold, 70:AltHold, 71:FlowHold, 72:Circle, 73:Drift, 75:SurfaceTrackingUpDown, 76:Standby Mode, 78:RunCam Control, 79:RunCam OSD Control, 80:Viso Align, 81:Disarm, 83:ZigZag Auto, 84:Air Mode, 85:Generator, 90:EKF Pos Source, 100:KillIMU1, 101:KillIMU2, 102:Camera Mode Toggle, 105:GPS Disable Yaw, 300:Scripting1, 301:Scripting2, 302:Scripting3, 303:Scripting4, 304:Scripting5, 305:Scripting6, 306:Scripting7, 307:Scripting8
    // @Values{Rover}: 0:Do Nothing, 4:RTL, 5:Save Trim, 7:Save WP, 9:Camera Trigger, 11:Fence, 16:Auto, 19:Gripper, 24:Auto Mission Reset, 27:Retract Mount, 28:Relay On/Off, 30:Lost Rover Sound, 31:Motor Emergency Stop, 34:Relay2 On/Off, 35:Relay3 On/Off, 36:Relay4 On/Off, 40:Proximity Avoidance, 41:ArmDisarm, 42:SmartRTL, 46:RC Override Enable, 50:LearnCruise, 51:Manual, 52:Acro, 53:Steering, 54:Hold, 55:Guided, 56:Loiter, 57:Follow, 58:Clear Waypoints, 59:Simple Mode, 62:Compass Learn, 63:Sailboat Tack, 65:GPS Disable, 66:Relay5, 67:Relay6, 74:Sailboat motoring 3pos, 78:RunCam Control, 79:RunCam OSD Control, 80:Viso Align, 81:Disarm, 90:EKF Pos Source, 100:KillIMU1, 101:KillIMU2, 102:Camera Mode Toggle, 105:GPS Disable Yaw, 201:Roll, 202:Pitch, 203:Walking Height, 207:MainSail, 208:Flap, 300:Scripting1, 301:Scripting2, 302:Scripting3, 303:Scripting4, 304:Scripting5, 305:Scripting6, 306:Scripting7, 307:Scripting8
    // @Values{Plane}: 0:Do Nothing, 4:ModeRTL, 9:Camera Trigger, 16:ModeAuto, 24:Auto Mission Reset, 27:Retract Mount, 28:Relay On/Off, 29:Landing Gear, 30:Lost Plane Sound, 31:Motor Emergency Stop, 34:Relay2 On/Off, 35:Relay3 On/Off, 36:Relay4 On/Off, 38:ADSB Avoidance En, 41:ArmDisarm, 43:InvertedFlight, 46:RC Override Enable, 51:ModeManual, 55:ModeGuided, 56:ModeLoiter, 58:Clear Waypoints, 62:Compass Learn, 64:Reverse Throttle, 65:GPS Disable, 66:Relay5, 67:Relay6, 72:ModeCircle, 77:ModeTakeoff, 78:RunCam Control, 79:RunCam OSD Control, 81:Disarm, 82:QAssist 3pos, 84:Air Mode, 85:Generator, 86: Non Auto Terrain Follow Disable, 87:Crow Select, 88:Soaring Enable, 89:Landing Flare, 90:EKF Pos Source, 91:Airspeed Ratio Calibration, 92:FBWA, 100:KillIMU1, 101:KillIMU2, 102:Camera Mode Toggle, 105:GPS Disable Yaw, 208:Flap, 209: Forward Throttle, 300:Scripting1, 301:Scripting2, 302:Scripting3, 303:Scripting4, 304:Scripting5, 305:Scripting6, 306:Scripting7, 307:Scripting8
    // @User: Standard
    AP_GROUPINFO("FUNC2",  11, AP_Button, pin_func[1], (uint16_t)RC_Channel::AUX_FUNC::DO_NOTHING),

    // @Param: FUNC3
    // @DisplayName: Button Pin 3 RC Channel function
    // @Description: Auxiliary RC Options function executed on pin change
    // @Values{Copter}: 0:Do Nothing, 2:Flip, 3:Simple Mode, 4:RTL, 5:Save Trim, 7:Save WP, 9:Camera Trigger, 10:RangeFinder, 11:Fence, 13:Super Simple Mode, 14:Acro Trainer, 15:Sprayer, 16:Auto, 17:AutoTune, 18:Land, 19:Gripper, 21:Parachute Enable, 22:Parachute Release, 23:Parachute 3pos, 24:Auto Mission Reset, 25:AttCon Feed Forward, 26:AttCon Accel Limits, 27:Retract Mount, 28:Relay On/Off, 29:Landing Gear, 30:Lost Copter Sound, 31:Motor Emergency Stop, 32:Motor Interlock, 33:Brake, 34:Relay2 On/Off, 35:Relay3 On/Off, 36:Relay4 On/Off, 37:Throw, 38:ADSB Avoidance En, 39:PrecLoiter, 40:Proximity Avoidance, 41:ArmDisarm, 42:SmartRTL, 43:InvertedFlight, 46:RC Override Enable, 47:User Function 1, 48:User Function 2, 49:User Function 3, 52:Acro, 55:Guided, 56:Loiter, 57:Follow, 58:Clear Waypoints, 60:ZigZag, 61:ZigZag SaveWP, 62:Compass Learn, 65:GPS Disable, 66:Relay5, 67:Relay6, 68:Stabilize, 69:PosHold, 70:AltHold, 71:FlowHold, 72:Circle, 73:Drift, 75:SurfaceTrackingUpDown, 76:Standby Mode, 78:RunCam Control, 79:RunCam OSD Control, 80:Viso Align, 81:Disarm, 83:ZigZag Auto, 84:Air Mode, 85:Generator, 90:EKF Pos Source, 100:KillIMU1, 101:KillIMU2, 102:Camera Mode Toggle, 105:GPS Disable Yaw, 300:Scripting1, 301:Scripting2, 302:Scripting3, 303:Scripting4, 304:Scripting5, 305:Scripting6, 306:Scripting7, 307:Scripting8
    // @Values{Rover}: 0:Do Nothing, 4:RTL, 5:Save Trim, 7:Save WP, 9:Camera Trigger, 11:Fence, 16:Auto, 19:Gripper, 24:Auto Mission Reset, 27:Retract Mount, 28:Relay On/Off, 30:Lost Rover Sound, 31:Motor Emergency Stop, 34:Relay2 On/Off, 35:Relay3 On/Off, 36:Relay4 On/Off, 40:Proximity Avoidance, 41:ArmDisarm, 42:SmartRTL, 46:RC Override Enable, 50:LearnCruise, 51:Manual, 52:Acro, 53:Steering, 54:Hold, 55:Guided, 56:Loiter, 57:Follow, 58:Clear Waypoints, 59:Simple Mode, 62:Compass Learn, 63:Sailboat Tack, 65:GPS Disable, 66:Relay5, 67:Relay6, 74:Sailboat motoring 3pos, 78:RunCam Control, 79:RunCam OSD Control, 80:Viso Align, 81:Disarm, 90:EKF Pos Source, 100:KillIMU1, 101:KillIMU2, 102:Camera Mode Toggle, 105:GPS Disable Yaw, 201:Roll, 202:Pitch, 203:Walking Height, 207:MainSail, 208:Flap, 300:Scripting1, 301:Scripting2, 302:Scripting3, 303:Scripting4, 304:Scripting5, 305:Scripting6, 306:Scripting7, 307:Scripting8
    // @Values{Plane}: 0:Do Nothing, 4:ModeRTL, 9:Camera Trigger, 16:ModeAuto, 24:Auto Mission Reset, 27:Retract Mount, 28:Relay On/Off, 29:Landing Gear, 30:Lost Plane Sound, 31:Motor Emergency Stop, 34:Relay2 On/Off, 35:Relay3 On/Off, 36:Relay4 On/Off, 38:ADSB Avoidance En, 41:ArmDisarm, 43:InvertedFlight, 46:RC Override Enable, 51:ModeManual, 55:ModeGuided, 56:ModeLoiter, 58:Clear Waypoints, 62:Compass Learn, 64:Reverse Throttle, 65:GPS Disable, 66:Relay5, 67:Relay6, 72:ModeCircle, 77:ModeTakeoff, 78:RunCam Control, 79:RunCam OSD Control, 81:Disarm, 82:QAssist 3pos, 84:Air Mode, 85:Generator, 86: Non Auto Terrain Follow Disable, 87:Crow Select, 88:Soaring Enable, 89:Landing Flare, 90:EKF Pos Source, 91:Airspeed Ratio Calibration, 92:FBWA, 100:KillIMU1, 101:KillIMU2, 102:Camera Mode Toggle, 105:GPS Disable Yaw, 208:Flap, 209: Forward Throttle, 300:Scripting1, 301:Scripting2, 302:Scripting3, 303:Scripting4, 304:Scripting5, 305:Scripting6, 306:Scripting7, 307:Scripting8
    // @User: Standard
    AP_GROUPINFO("FUNC3",  12, AP_Button, pin_func[2], (uint16_t)RC_Channel::AUX_FUNC::DO_NOTHING),

    // @Param: FUNC4
    // @DisplayName: Button Pin 4 RC Channel function
    // @Description: Auxiliary RC Options function executed on pin change
    // @Values{Copter}: 0:Do Nothing, 2:Flip, 3:Simple Mode, 4:RTL, 5:Save Trim, 7:Save WP, 9:Camera Trigger, 10:RangeFinder, 11:Fence, 13:Super Simple Mode, 14:Acro Trainer, 15:Sprayer, 16:Auto, 17:AutoTune, 18:Land, 19:Gripper, 21:Parachute Enable, 22:Parachute Release, 23:Parachute 3pos, 24:Auto Mission Reset, 25:AttCon Feed Forward, 26:AttCon Accel Limits, 27:Retract Mount, 28:Relay On/Off, 29:Landing Gear, 30:Lost Copter Sound, 31:Motor Emergency Stop, 32:Motor Interlock, 33:Brake, 34:Relay2 On/Off, 35:Relay3 On/Off, 36:Relay4 On/Off, 37:Throw, 38:ADSB Avoidance En, 39:PrecLoiter, 40:Proximity Avoidance, 41:ArmDisarm, 42:SmartRTL, 43:InvertedFlight, 46:RC Override Enable, 47:User Function 1, 48:User Function 2, 49:User Function 3, 52:Acro, 55:Guided, 56:Loiter, 57:Follow, 58:Clear Waypoints, 60:ZigZag, 61:ZigZag SaveWP, 62:Compass Learn, 65:GPS Disable, 66:Relay5, 67:Relay6, 68:Stabilize, 69:PosHold, 70:AltHold, 71:FlowHold, 72:Circle, 73:Drift, 75:SurfaceTrackingUpDown, 76:Standby Mode, 78:RunCam Control, 79:RunCam OSD Control, 80:Viso Align, 81:Disarm, 83:ZigZag Auto, 84:Air Mode, 85:Generator, 90:EKF Pos Source, 100:KillIMU1, 101:KillIMU2, 102:Camera Mode Toggle, 105:GPS Disable Yaw, 300:Scripting1, 301:Scripting2, 302:Scripting3, 303:Scripting4, 304:Scripting5, 305:Scripting6, 306:Scripting7, 307:Scripting8
    // @Values{Rover}: 0:Do Nothing, 4:RTL, 5:Save Trim, 7:Save WP, 9:Camera Trigger, 11:Fence, 16:Auto, 19:Gripper, 24:Auto Mission Reset, 27:Retract Mount, 28:Relay On/Off, 30:Lost Rover Sound, 31:Motor Emergency Stop, 34:Relay2 On/Off, 35:Relay3 On/Off, 36:Relay4 On/Off, 40:Proximity Avoidance, 41:ArmDisarm, 42:SmartRTL, 46:RC Override Enable, 50:LearnCruise, 51:Manual, 52:Acro, 53:Steering, 54:Hold, 55:Guided, 56:Loiter, 57:Follow, 58:Clear Waypoints, 59:Simple Mode, 62:Compass Learn, 63:Sailboat Tack, 65:GPS Disable, 66:Relay5, 67:Relay6, 74:Sailboat motoring 3pos, 78:RunCam Control, 79:RunCam OSD Control, 80:Viso Align, 81:Disarm, 90:EKF Pos Source, 100:KillIMU1, 101:KillIMU2, 102:Camera Mode Toggle, 105:GPS Disable Yaw, 201:Roll, 202:Pitch, 203:Walking Height, 207:MainSail, 208:Flap, 300:Scripting1, 301:Scripting2, 302:Scripting3, 303:Scripting4, 304:Scripting5, 305:Scripting6, 306:Scripting7, 307:Scripting8
    // @Values{Plane}: 0:Do Nothing, 4:ModeRTL, 9:Camera Trigger, 16:ModeAuto, 24:Auto Mission Reset, 27:Retract Mount, 28:Relay On/Off, 29:Landing Gear, 30:Lost Plane Sound, 31:Motor Emergency Stop, 34:Relay2 On/Off, 35:Relay3 On/Off, 36:Relay4 On/Off, 38:ADSB Avoidance En, 41:ArmDisarm, 43:InvertedFlight, 46:RC Override Enable, 51:ModeManual, 55:ModeGuided, 56:ModeLoiter, 58:Clear Waypoints, 62:Compass Learn, 64:Reverse Throttle, 65:GPS Disable, 66:Relay5, 67:Relay6, 72:ModeCircle, 77:ModeTakeoff, 78:RunCam Control, 79:RunCam OSD Control, 81:Disarm, 82:QAssist 3pos, 84:Air Mode, 85:Generator, 86: Non Auto Terrain Follow Disable, 87:Crow Select, 88:Soaring Enable, 89:Landing Flare, 90:EKF Pos Source, 91:Airspeed Ratio Calibration, 92:FBWA, 100:KillIMU1, 101:KillIMU2, 102:Camera Mode Toggle, 105:GPS Disable Yaw, 208:Flap, 209: Forward Throttle, 300:Scripting1, 301:Scripting2, 302:Scripting3, 303:Scripting4, 304:Scripting5, 305:Scripting6, 306:Scripting7, 307:Scripting8
    // @User: Standard
    AP_GROUPINFO("FUNC4",  13, AP_Button, pin_func[3], (uint16_t)RC_Channel::AUX_FUNC::DO_NOTHING),

    AP_GROUPEND    
};


// constructor
AP_Button::AP_Button(void)
{
    AP_Param::setup_object_defaults(this, var_info);

    if (_singleton != nullptr) {
        AP_HAL::panic("AP_Button must be singleton");
    }
    _singleton = this;
}

/*
  update and report, called from main loop
 */
void AP_Button::update(void)
{
    if (!enable) {
        return;
    }

    // call setup pins at update rate (5Hz) to allow for runtime parameter change of pins
    setup_pins();

    if (!initialised) {
        initialised = true;

        // get initial mask
        last_mask = get_mask();
        debounce_mask = last_mask;

        // register 1kHz timer callback
        hal.scheduler->register_timer_process(FUNCTOR_BIND_MEMBER(&AP_Button::timer_update, void));        
    }

    // act on any changes in state
    {
        WITH_SEMAPHORE(last_debounced_change_ms_sem);
        if (last_debounced_change_ms > last_debounce_ms) {
            last_debounce_ms = last_debounced_change_ms;
        }
    }

    // update the PWM state:
    uint8_t new_pwm_state = pwm_state;
    for (uint8_t i=0; i<AP_BUTTON_NUM_PINS; i++) {
        const uint8_t mask = (1U << i);
        if (!is_pwm_input(i)) {
            // not a PWM input
            new_pwm_state &= ~mask;
            continue;
        }
        const uint16_t pwm_us = pwm_pin_source[i].get_pwm_us();
        // these values are the same as used in RC_Channel:
        if (pwm_state & mask) {
            // currently asserted; check to see if we should de-assert
            if (pwm_us < RC_Channel::AUX_SWITCH_PWM_TRIGGER_LOW) {
                new_pwm_state &= ~mask;
            }
        } else {
            // currently not asserted; check to see if we should assert
            if (pwm_us > RC_Channel::AUX_SWITCH_PWM_TRIGGER_HIGH) {
                new_pwm_state |= mask;
            }
        }
    }
    if (new_pwm_state != pwm_state) {
        pwm_state = new_pwm_state;
        last_debounce_ms = AP_HAL::millis64();
    }

    if (last_debounce_ms != 0 &&
        (AP_HAL::millis() - last_report_ms) > AP_BUTTON_REPORT_PERIOD_MS &&
        (AP_HAL::millis64() - last_debounce_ms) < report_send_time*1000ULL) {
        // send a change report
        last_report_ms = AP_HAL::millis();

        // send a report to GCS
        send_report();
    }

    if (!aux_functions_initialised) {
        run_aux_functions(true);
        aux_functions_initialised = true;
    }

    if (last_debounce_ms != 0 &&
        last_debounce_ms != last_action_time_ms) {
        last_action_time_ms = last_debounce_ms;
        run_aux_functions(false);
    }
}

void AP_Button::run_aux_functions(bool force)
{
    RC_Channel *rc_channel = rc().channel(1);
    if (rc_channel == nullptr) {
        return;
    }

    for (uint8_t i=0; i<AP_BUTTON_NUM_PINS; i++) {
        const RC_Channel::AUX_FUNC func = RC_Channel::AUX_FUNC(pin_func[i].get());
        if (func == RC_Channel::AUX_FUNC::DO_NOTHING) {
            continue;
        }
        const uint8_t value_mask = (1U<<i);
        bool value;
        if (is_pwm_input(i)) {
            value = (pwm_state & value_mask) != 0;
        } else {
            value = (debounce_mask & value_mask) != 0;
        }
        if (is_input_inverted(i)) {
            value = !value;
        }
        const bool actioned = ((state_actioned_mask & value_mask) != 0);
        if (!force && value == actioned) {
            // no change on this pin
            continue;
        }
        // mark action as done:
        if (value) {
            state_actioned_mask |= value_mask;
        } else {
            state_actioned_mask &= ~value_mask;
        }

        const RC_Channel::AuxSwitchPos pos = value ? RC_Channel::AuxSwitchPos::HIGH : RC_Channel::AuxSwitchPos::LOW;
        // I wonder if we can do better here:
#if !HAL_MINIMIZE_FEATURES
        const char *str = rc_channel->string_for_aux_function(func);
        if (str != nullptr) {
            gcs().send_text(MAV_SEVERITY_INFO, "Button: executing (%s)", str);
        }
#endif
        rc_channel->run_aux_function(func, pos, RC_Channel::AuxFuncTriggerSource::BUTTON);
    }
}

// get state of a button
// used by scripting
bool AP_Button::get_button_state(uint8_t number)
{
    // pins params are 1 indexed not zero
    if (number == 0 || number > AP_BUTTON_NUM_PINS) {
        return false;
    }

    if (is_pwm_input(number-1)) {
        return (pwm_state & (1U<<(number-1)));
    }

    return ( ((1 << (number - 1)) & debounce_mask) != 0);
};

/*
  get current mask
 */
uint8_t AP_Button::get_mask(void)
{
    uint8_t mask = 0;
    for (uint8_t i=0; i<AP_BUTTON_NUM_PINS; i++) {
        if (pin[i] == -1) {
            continue;
        }
        if (is_pwm_input(i)) {
            continue;
        }
        mask |= hal.gpio->read(pin[i]) << i;
    }

    return mask;
}

/*
  called at 1kHz to check for button state change
 */
void AP_Button::timer_update(void)
{
    if (!enable) {
        return;
    }
    uint8_t mask = get_mask();
    uint64_t now = AP_HAL::millis64();
    if (mask != last_mask) {
        last_mask = mask;
        last_change_time_ms = now;
    }
    if (debounce_mask != last_mask &&
        (now - last_change_time_ms) > DEBOUNCE_MS) {
        // crude de-bouncing, debounces all buttons as one, not individually
        debounce_mask = last_mask;
        WITH_SEMAPHORE(last_debounced_change_ms_sem);
        last_debounced_change_ms = now;
    }
}

/*
  send a BUTTON_CHANGE report to the GCS
 */
void AP_Button::send_report(void) const
{
    const uint8_t mask = last_mask | pwm_state;
    const mavlink_button_change_t packet{
            time_boot_ms: AP_HAL::millis(),
            last_change_ms: uint32_t(last_debounce_ms),
            state: mask,
    };
    gcs().send_to_active_channels(MAVLINK_MSG_ID_BUTTON_CHANGE,
                                  (const char *)&packet);
}

/*
  setup the pins as input with pullup. We need pullup to give reliable
  input with a pulldown button
 */
void AP_Button::setup_pins(void)
{
    for (uint8_t i=0; i<AP_BUTTON_NUM_PINS; i++) {
        if (is_pwm_input(i)) {
            pwm_pin_source[i].set_pin(pin[i], "Button");
            continue;
        }
        if (pin[i] == -1) {
            continue;
        }

        hal.gpio->pinMode(pin[i], HAL_GPIO_INPUT);
        // setup pullup
        hal.gpio->write(pin[i], 1);
    }
}

namespace AP {

AP_Button &button()
{
    return *AP_Button::get_singleton();
}

}
