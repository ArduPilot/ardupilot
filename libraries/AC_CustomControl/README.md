# Custom Controller

Custom controller library allows you to implement and easily run your own controller inside ArduPilot in a systematic way. Original-primary-mains means existing controller and custom-secondary means new controller. This library aimed to not interfere with other parts of the main controller or vehicle level code. The controller output is sent to the control allocation library, known as the mixer, the same way the main controller does.

## Features 

- In-flight switching between main and custom controller with RC switch, option 109.
- Bitmask to choose which axis to use the custom controller output
- Filter, integrator reset mechanism when switching between controller
  - Bumpless transfer when switching from custom to the main controller
- Ground and in-flight spool state separation to avoid build-up during arming and take-off with the custom controller
- Frontend-backend separation that allows adding a new controller with very little overhead
- Flag to compile out custom controller related code on hardware, --enable-custom-controller
- Proper parameter table implementation that allows adding new custom controller parameter table without corruption
- Single parameter to switch between different custom controllers, reboot required
- Multiple checks to avoid accidentally running mis-un/configured custom controller with RC switch
- Custom controller parameters start with `CC_` in the GCS.

The frontend library has the following parameters

- `CC_TYPE`: choose which custom controller backend to use, reboot required.
  - Setting it to 0 will turn this feature off, GCS will not display parameters related to the custom controller 
- `CC_AXIS_MASK`: choose which axis to use custom controller output
  - This is a bitmask type parameter. Set 7 to use all output


## Interaction With Main Controller

The custom controller update function is called after the main rate controller is run and before the motor output library is called. This placement allows overriding motor library mixer input, namely `_roll_in`, `_pitch_in`, `_yaw_in` values, without causing functional change inside the motor library. This is the same way the main controller sends its output to the control allocation library, known as a mixer. This reduces latency to the minimum level.

The custom controller library uses the same target attitude as the main controller. Most of the code inside AC_AttitudeControl library is related to input shapings such as how to interpret pilot commands depending on the flight mode or high-level position controller output. For example, in STABILIZE mode pilot RC commands are scaled based on maximum lean angle and yaw rate parameters, these values are passed to `input_euler_angle_roll_pitch_euler_rate_yaw` function. The pilot command is fed into a first-order input shaping algorithm to smooth out any jitter due to RC link and to generate an acceleration-limited attitude target. Later, attitude controller `attitude_controller_run_quat` is called and attitude error is calculated based on acceleration limited `_attitude_target` value, and target rate is generated. Even if acceleration limiting is turned off by setting `ATC_RATE_FF_ENAB` to 0, the target attitude is still presented with `_attitude_target` variable.

Every input shaping function inside `AC_AttitudeControl` calls `attitude_controller_run_quat` at the end to run the attitude controller, except when flying in ACRO mode with the acro option set to rate only. Even in this case `_attitude_target` is updated properly and pilot command can be accessed via `rate_bf_targets()` function.

By default, the input shaping algorithm is turned on. This produces kinematically consistent attitude target and rate feed-forward values. Take a look at the custom PID backend to see how rate feed-forward is added to attitude controller output. It is advised to use the rate feed-forward value if `ATC_RATE_FF_ENAB` is enabled, otherwise the pilot might feel significant lag between RC command and vehicle response.

After running the custom controller, mixer input is sent to the motor library via `set_roll`, `set_pitch`, `set_yaw` functions.

### Bumpless Transfer
When switching from the custom controller to the main controller it is important to reset the main controller target, error, d-term filters and set each axis integrator properly. Otherwise, a sudden jump in the controller error signal or motor output will be observed, which could result in a jerky motion. To allow a smooth transition between controllers, the main controller reset function is called when switching out of the custom controller which reset all three axis of the main controller.

The attitude target and rate target are also made equal to the current attitude and gyro rate to make the error signal grow from zero. In order to avoid impulse input to the controller, the target resetting is not performed when the feedforward is disabled by setting `ATC_RATE_FF_ENAB` parameter to 0. 

The reset is performed inside `reset_main_atti_controller` function. An example of reset per axis is given here.

```C++
_atti_control->get_rate_roll_pid().reset_filter();
_atti_control->get_rate_roll_pid().set_integrator(_atti_control->rate_bf_targets().x - gyro_latest.x, _motors->get_roll());
```

## Backend Type
Currently, there 4 custom controller backend available. These are 
- Empty backend `CC_TYPE` = 1
- PID backend `CC_TYPE` = 2

### Empty Controller - `CC_TYPE` = 1
The empty controller does not do any calculations. It is created to make it easier to copy and implement your new controller. The main controller is not reset when switching from an empty controller.

### PID Controller - `CC_TYPE` = 2
PID controller backend has the same controller architecture as the main controller. It doesn't have any safeguarding mechanism such as acceleration limiting or rate limiting. The default gains are scaled 0.9 times to differentiate the custom controller response from the main one. Since this controller does not have acceleration limiting, specifically a square root controller, it would be safer to give a gentle command while flying with it. Although it has the same architecture as the main one, a proper reset functionality is not implemented intentionally to make it easier to detect the effect of improper resetting.

## How To Use It 
The custom controller is enabled by default in SITL. You can test it using PID backend.

1. Compile and run the default SITL model. In the GCS, choose the custom controller type, assign axis mask and set which RC switch to activate the custom controller. Reboot autopilot. For example in mavproxy,
```
param set CC_TYPE 2
param set CC_AXIS_MASK 7
param set RC6_OPTION 109
reboot
```

2. Run the following command to display backend parameters. These would be under `CC2_` for PID backend.
```
param set CC2*
```

3. Arm and take-off. While at the hover flight, switch RC6 to high. In mavproxy, you can do this with
```
rc 6 2000
```

4. You should be prompted with `Custom controller is ON` message on GCS to indicate that the custom controller is running.

5. Set RC6 to low to switch back to the main controller. You should be prompted with `Custom controller is OFF` message on GCS. 


### Real Flight Testing
It is recommended that you always arm, take-off, land, and disarm while the main controller is running. You should switch to the custom controller while the vehicle is hovering steadily. This will reduce the effect of improper filter resetting. You should arm and take off with the custom controller only if proper ground idling is implemented.

To test it on hardware compile with "--enable-custom-controller" flag.
```C++
./waf configure --board CubeOrange copter --enable-custom-controller
```

### Post Flight Logs
Switching in and out of custom controller logged under the "CC" in log tree. You can see the time and duration the custom controller is active under "CC.Act" 

## How To Add New Custom Controller
You can add your own custom controller backend with the following steps. Let's assume we are adding the 5th custom controller.

1. Generate a copy of `AC_CustomControl_Empty.cpp` and `AC_CustomControl_Empty.h` within `AC_CustomControl` folder. The folder tree would look like this,
```
AC_CustomControl.cpp
AC_CustomControl.h
AC_CustomControl_Backend.h
AC_CustomControl_Empty.cpp
AC_CustomControl_Empty.h
AC_CustomControl_Empty - Copy.cpp
AC_CustomControl_Empty - Copy.h
.
.
.
```

PID and README files are omitted to keep it simple. 

2. Change `Empty - Copy` suffix with your own choice, let's called it `XYZ`, which would look like 
```
AC_CustomControl.cpp
AC_CustomControl.h
AC_CustomControl_Backend.h
AC_CustomControl_Empty.cpp
AC_CustomControl_Empty.h
AC_CustomControl_XYZ.cpp
AC_CustomControl_XYZ.h
.
.
.
```

3. Change every class name, function definition etc. from `AC_CustomControl_Empty` to `AC_CustomControl_XYZ` inside `AC_CustomControl_XYZ.cpp` and `AC_CustomControl_XYZ.h` files. This would look like this for the header file
```C++
#pragma once

#include "AC_CustomControl_Backend.h"

class AC_CustomControl_XYZ : public AC_CustomControl_Backend {
public:
    AC_CustomControl_XYZ(AC_CustomControl &frontend, AP_AHRS_View*& ahrs, AC_AttitudeControl_Multi*& atti_control, AP_MotorsMulticopter*& motors, float dt);


    Vector3f update(void) override;
    void reset(void) override;

    // user settable parameters
    static const struct AP_Param::GroupInfo var_info[];

protected:
    // declare parameters here
    AP_Float param1;
    AP_Float param2;
    AP_Float param3;
};
```

4. Increase the maximum number of custom control variables by one and update the custom control type enum

from 
```C++
#define CUSTOMCONTROL_MAX_TYPES 2
```
to
```C++
#define CUSTOMCONTROL_MAX_TYPES 3
```

```C++
enum class CustomControlType : uint8_t {
	CONT_NONE            = 0,
	CONT_EMPTY           = 1,             
	CONT_PID             = 2,
	CONT_XYZ             = 3,
};            // controller that should be used     
```

5. Add a new backend header in `AC_CustomControl.cpp` file. Place it under other backend includes.
```C++
#include "AC_CustomControl_Backend.h"
#include "AC_CustomControl_Empty.h"
#include "AC_CustomControl_PID.h"
#include "AC_CustomControl_XYZ.h"
``` 

6. Add new backend parameter in `AC_CustomControl.cpp` file. Increment `_backend`, `_backend_var_info` array index by one and also increment backend parameter prefix and parameter table index by one. Place it under the other backend's parameters.
```C++
.
.
.
    // parameters for empty controller
    AP_SUBGROUPVARPTR(_backend, "1_", 6, AC_CustomControl, _backend_var_info[0]),

    // parameters for PID controller
    AP_SUBGROUPVARPTR(_backend, "2_", 7, AC_CustomControl, _backend_var_info[1]),

    // parameters for XYZ controller
    AP_SUBGROUPVARPTR(_backend, "3_", 8, AC_CustomControl, _backend_var_info[2]),

    AP_GROUPEND
};
```

7. Allow creating new backend class in `AC_CustomControl.cpp` file inside `init` function.
```C++
.
.
.
case CustomControlType::CONT_PID:
    _backend = new AC_CustomControl_PID(*this, _ahrs, _atti_control, _motors, _dt);
    _backend_var_info[get_type()] = AC_CustomControl_PID::var_info;
    break;
case CustomControlType::CONT_XYZ:
    _backend = new AC_CustomControl_XYZ(*this, _ahrs, _atti_control, _motors, _dt);
    _backend_var_info[get_type()] = AC_CustomControl_XYZ::var_info;
    break;
default:
            return;
}
```
Add the following lines in the `AC_CustomControl_config.h` file.

```
#ifndef AP_CUSTOMCONTROL_XYZ_ENABLED
#define AP_CUSTOMCONTROL_XYZ_ENABLED AP_CUSTOMCONTROL_BACKEND_DEFAULT_ENABLED
#endif
```

8. This is the bare minimum to compile and run your custom controller. You can add controller related code to `AC_CustomControl_XYZ` file without changing  anything else. 

9. You can add new parameters by following the directions in this [Adding a parameter to a library](https://ardupilot.org/dev/docs/code-overview-adding-a-new-parameter.html#adding-a-parameter-to-a-library) wiki page.

10. Initialize the class object in the backend's constructor. For example in PID backend 
```C++
    // put controller related variable here

    // angle P controller  objects
    AC_P                _p_angle_roll2;
    AC_P                _p_angle_pitch2;
    AC_P                _p_angle_yaw2;

	// rate PID controller  objects
    AC_PID _pid_atti_rate_roll;
    AC_PID _pid_atti_rate_pitch;
    AC_PID _pid_atti_rate_yaw;
```
above P or PID classes are initialized in the backend's constructors,
```C++
AC_CustomControl_PID::AC_CustomControl_PID(AC_CustomControl &frontend, AP_AHRS_View*& ahrs, AC_AttitudeControl_Multi*& atti_control, AP_MotorsMulticopter*& motors, float dt) :
    AC_CustomControl_Backend(frontend, ahrs, atti_control, motors, dt),
    _p_angle_roll2(AC_ATTITUDE_CONTROL_ANGLE_P * 0.90f),
    _p_angle_pitch2(AC_ATTITUDE_CONTROL_ANGLE_P * 0.90f),
    _p_angle_yaw2(AC_ATTITUDE_CONTROL_ANGLE_P * 0.90f),
    _pid_atti_rate_roll(AC_ATC_MULTI_RATE_RP_P * 0.90f, AC_ATC_MULTI_RATE_RP_I * 0.90f, AC_ATC_MULTI_RATE_RP_D * 0.90f, 0.0f, AC_ATC_MULTI_RATE_RP_IMAX * 0.90f, AC_ATC_MULTI_RATE_RPY_FILT_HZ * 0.90f, 0.0f, AC_ATC_MULTI_RATE_RPY_FILT_HZ * 0.90f, dt),
    _pid_atti_rate_pitch(AC_ATC_MULTI_RATE_RP_P * 0.90f, AC_ATC_MULTI_RATE_RP_I * 0.90f, AC_ATC_MULTI_RATE_RP_D * 0.90f, 0.0f, AC_ATC_MULTI_RATE_RP_IMAX * 0.90f, AC_ATC_MULTI_RATE_RPY_FILT_HZ * 0.90f, 0.0f, AC_ATC_MULTI_RATE_RPY_FILT_HZ * 0.90f, dt),
    _pid_atti_rate_yaw(AC_ATC_MULTI_RATE_YAW_P * 0.90f, AC_ATC_MULTI_RATE_YAW_I * 0.90f, AC_ATC_MULTI_RATE_YAW_D * 0.90f, 0.0f, AC_ATC_MULTI_RATE_YAW_IMAX * 0.90f, AC_ATC_MULTI_RATE_RPY_FILT_HZ * 0.90f, AC_ATC_MULTI_RATE_YAW_FILT_HZ * 0.90f, 0.0f, dt)
{
    AP_Param::setup_object_defaults(this, var_info);
}
```

11. Add your controller inside `update` function of `AC_CustomControl_XYZ.cpp` file. This function returns a 3-dimensional vector consisting of roll pitch yaw mixer input, respectively.

12. Add reset functionality inside `reset` function of `AC_CustomControl_XYZ.cpp` file. It is user's responsibility to add proper controller resetting functionality. This highly depends on the controller and it should not be copy-pasted from another backend without testing it in SITL.

13. You can access target attitude and target rate values using `atti_control->get_attitude_target_quat()` and `atti_control->rate_bf_targets()` functions, respectively. You can also access latest gyro measurement using `_ahrs->get_gyro_latest()` function. Take a look at other backends
