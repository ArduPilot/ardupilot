# Custom Controller

`AP_CustomControl` offers a framework to support custom controllers in ArduPlane.
Original-primary-main refers to the standard ArduPilot controller and custom-secondary refers to new controllers.

## Features

- In-flight switching between main and custom controller with RC switch, option 109.
- Custom controller can drive any servo output by function or by number.
- Bitmask to choose which custom controller features to enable.
- Filter, integrator reset mechanism when switching between controllers.
- Ground and in-flight state separation to avoid build-up during arming and take-off with the custom controller.
- Frontend-backend separation that allows adding a new controller with very little overhead.
- Proper parameter table implementation that allows adding new custom controller parameter table without corruption.
- Single parameter to switch between different custom controllers, reboot required.
- Multiple checks to avoid accidentally running mis-un/configured custom controller with RC switch.
- Custom controller parameters start with `CP_` in the GCS.
- Flag to add custom controller features in the ArduPilot binary, --enable-PLANE_CUSTOM_CONTROL.

The frontend library has the following parameters:

- `CP_TYPE`: choose which custom controller backend to use, reboot required.
  - Setting it to 0 will turn this feature off, GCS will not display parameters related to the custom controller.
- `CP_MASK`: choose which features of the custom controller should run.
  - This is a bitmask type parameter. Its behaviour depends on each individual custom controller implementation. It is typically used to gate the custom controller outputs on each individual axis (roll, pitch, yaw) but also any other control output it might provide.

## Interaction With Main Controller

The custom controller update function is called after the stabilization task has run the main controller and before the servo output task.
This placement allows overriding the inputs to the mixers, for example the values written to the `k_aileron`, `k_elevator`, `k_rudder` channels, without requiring functional change inside the servo output codebase.

After running the custom controller, the updated controls will be read by the servo-out task and get sent to the servos and motors.

### Bumpless Transfer

When switching from the custom controller to the main controller it is important to reset the main controller integrators.
They will most likely have wound up, because they are being overriden and cannot affect the aircraft. This happens automatically by the library.

The main controller targets are not reset to the current states, since the Plane controllers are stateless.
So despite the integrator reset, some step change in the actuators should be expected.

## Backend Type

Currently, there 2 custom controller backend types. These are:

- Empty backend `CP_TYPE` = 1. This is an example and cannot be activated.
- PID backend `CP_TYPE` = 2

### Empty Controller - `CP_TYPE` = 1

This is a template controller. It is not functional and cannot be engaged.
The empty controller does not do any calculations, it merely forwards the aileron, elevator, throttle and rudder commands calculated by the main controller.
It is created to make it easier to copy and implement your new controller.

### PID Controller - `CP_TYPE` = 2

The PID controller backend has roughly the same controller architecture as the main controller.
The default gains are the same to those of the main controller, but you can modify them and observe the difference.

Additionally, it demonstrates the capabilities of the library by reading from various RC input channels writing to various output servo channels.
Read the `AP_CustomControl_PID.cpp` file to get a full understanding of the behaviour of this controller.

**WARNING**: Ensure your autopilot is configured with the required input channels and that the additional servo outputs that the PID backend manipulates will not put your flight in danger! If necessary, disable the custom control outputs that can put your flight in danger by editing the `AP_CustomControl_PID.cpp::update()` method and re-compile the firmware.

## How To Use It

The custom controller is enabled by default in SITL. You can test it using PID backend.

1. Compile and run the default SITL model. In the GCS, choose the custom controller type and set which RC switch to activate the custom controller. Reboot autopilot. For example in mavproxy,

```text
param set CP_TYPE 2
param set RC6_OPTION 109
reboot
```

2. Run the following command to display backend parameters. These would be under `CP2_` for PID backend.

```text
param set CP2*
```

3. Arm and take-off. While in flight, in any stabilized mode, switch RC6 to high. In mavproxy, you can do this with

```text
rc 6 2000
```

**Do not** switch to the custom controller while in MANUAL, STABILIZE or ACRO modes, since the custom controller tries to serve the attitude targets, which do not get generated in this flight mode.
The PID backend will refuse engaging while in those modes.

4. You should be prompted with `Custom controller is ON` message on GCS to indicate that the custom controller is running.

5. Set RC6 to low to switch back to the main controller. You should be prompted with `Custom controller is OFF` message on GCS.

### Adding your own controllers

1. In the same folder as the other custom controllers, create new `AP_CustomControl_*.h` and `AP_CustomControl_*.cpp` which will host your controller. You can copy the existing controllers and use them as a starting point.

2. Modify the `.cpp` file to your liking. Declare your custom parameters in `var_info[]`, your control logic in `update()` and your controller reset logic in `reset()`. You are also encouraged to add your engage conditions in the `can_run()` method, to ensure that your controller cannot get activated under the wrong conditions.

3. Register your controller with the library:
    - Add an new entry in `AP_CustomControl.h` in `enum class CustomControlType` and increment the define `CUSTOMCONTROL_MAX_TYPES`.
    - In `AP_CustomControl.cpp` include your new header file, register your new controller parameters in `var_info[]` and initialize the controller in `init()`.

4. Update `AP_CustomControl_config.h` with your new defines.

If your controller is provided externally or autogenerated, it is a good idea to place it as-is in the `AP_CustomControl` folder and include it in your new custom controller files, as explained above.

### Utility API

Even though code written in `AP_CustomControl` can potentially control any aspect of ArduPlane, an API is being offered for safer interaction with the rest of the system.

Access attitude targets with:

```c++
float get_roll_target_deg();
float get_nav_pitch_target_deg();
float get_pitch_target_deg();
```

The `_frontend` object also provides the following methods to output servo values:

```c++
// Write a scaled value to all channels with a function.
void set_output_scaled(SRV_Channel::Function function, float value);
// Write a pwm value to all channels with a function. Not min/max constrained. servos.cpp may overwrite it.
void set_output_pwm(SRV_Channel::Function function, uint16_t value);
// Write pwm values on a channel. Not min/max constrained. servos.cpp may overwrite it.
void set_output_pwm_chan(uint8_t chan, uint16_t value);
// Override pwm values on a channel for one loop. servos.cpp will not overwrite it.
void set_output_pwm_chan_override(uint8_t chan, uint16_t value);
```

To access RC input channels, use the ArduPilot-native `rc()` singleton.
Make sure you check against `nullptr`!

### Real Flight Testing

It is recommended that you always arm, take-off, land, and disarm while the main controller is running. You should switch to the custom controller while the vehicle is in level flight. This will reduce the effect of improper state resetting. You should arm and take off with the custom controller only if proper ground handling is implemented.

To test it on hardware compile with "--enable-PLANE_CUSTOM_CONTROL" flag.

```C++
./waf configure --board CubeOrange copter --enable-PLANE_CUSTOM_CONTROL
```

You can also enable the feature in a custom hwdef board definition by setting

```text
define AP_PLANE_CUSTOMCONTROL_ENABED 1
```

### Post Flight Logs

Switching in and out of custom controller logged under the `CP` in log tree. You can see the time and duration the custom controller is active under `CP.Act`.
