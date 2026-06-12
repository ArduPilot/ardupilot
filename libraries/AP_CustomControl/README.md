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
- Custom controller parameters start with `CC_` in the GCS.
- Flag to add custom controller features in the ArduPilot binary, --enable-custom-controller.

The frontend library has the following parameters:

- `CC_TYPE`: choose which custom controller backend to use, reboot required.
  - Setting it to 0 will turn this feature off, GCS will not display parameters related to the custom controller.
- `CC_MASK`: choose which features of the custom controller should run.
  - This is a bitmask type parameter. Its behaviour depends on each individual custom controller implementation.

## Interaction With Main Controller

The custom controller update function is called after the stabilization task has run the main controller and before the servo output task. This placement allows overriding the inputs to the mixers, for example the values written to the `k_aileron`, `k_elevator`, `k_rudder` channels, without requiring functional change inside the servo output codebase.

After running the custom controller, the updated controls will be read by the servo out task and sent to the servos and motors.

### Bumpless Transfer

When switching from the custom controller to the main controller it is important to reset the main controller integrators. They will most likely have wound up, because they are being overriden and cannot affect the aircraft. This happens automatically by the library.

The main controller targets are not reset to the current states, since the Plane controllers are stateless. So despite the integrator reset, some step change in the actuators should be expected.

## Backend Type

Currently, there 2 custom controller backend available. These are:

- Empty backend `CC_TYPE` = 1
- PID backend `CC_TYPE` = 2

### Empty Controller - `CC_TYPE` = 1

The empty controller does not do any calculations, it merely forwards the aileron, elevator, throttle and rudder commands calculated by the main controller. It is created to make it easier to copy and implement your new controller.

### PID Controller - `CC_TYPE` = 2

PID controller backend has roughly the same controller architecture as the main controller. The default gains are the same to those of the main controller, but you can modify them and observe the difference.

Additionally, it reads from the flaps input channel and writes to various output servo channels, to demonstrate the capabilities of the library. Read the `AP_CustomControl_PID.cpp` file to get a full understanding of the behaviour of this controller.

## How To Use It

The custom controller is enabled by default in SITL. You can test it using PID backend.

1. Compile and run the default SITL model. In the GCS, choose the custom controller type and set which RC switch to activate the custom controller. Reboot autopilot. For example in mavproxy,

```text
param set CC_TYPE 2
param set RC6_OPTION 109
reboot
```

2. Run the following command to display backend parameters. These would be under `CC2_` for PID backend.

```text
param set CC2*
```

3. Arm and take-off. While in flight, in any stabilized mode, switch RC6 to high. In mavproxy, you can do this with

```text
rc 6 2000
```

Do not switch to the custom controller while in MANUAL mode, since the custom controller tries to serve the attitude targets, which do not get generated in this flight mode.

4. You should be prompted with `Custom controller is ON` message on GCS to indicate that the custom controller is running.

5. Set RC6 to low to switch back to the main controller. You should be prompted with `Custom controller is OFF` message on GCS.

### Adding your own controllers

1. In the same folder as the other custom controllers, create new `AP_CustomControl_*.h` and `AP_CustomControl_*.cpp` which will host your controller. You can copy the existing controllers and use them as a starting point.

2. Modify the `.cpp` file to your liking. Declare your custom parameters in `var_info[]`, your control logic in `update()` and your controller reset logic in `reset()`.

3. Register your controller with the library:
    - Add an new entry in `AP_CustomControl.h` in `enum class CustomControlType` and increment the define `CUSTOMCONTROL_MAX_TYPES`.
    - In `AP_CustomControl.cpp` include your new header file, register your new controller parameters in `var_info[]` and initialize the controller in `init()`.

4. Update `AP_CustomControl_config.h` with your new defines.

If your controller is provided externally or autogenerated, it is a good idea to place it as-is in the `AP_CustomControl` folder and include it in your new custom controller files, as explained above.

### Utility API

Even though code written in `AP_CustomControl` can potentially control any aspect of ArduPlane, an API is being offered for safer interaction with the rest of the system.

Access input channel objects with the following pointers and methods.
Always check if the pointers are null, in the case where an input channel has not been defined.

```c++
RC_Channel *channel_roll;
RC_Channel *channel_pitch;
RC_Channel *channel_throttle;
RC_Channel *channel_rudder;
RC_Channel *channel_flap;
RC_Channel *channel_airbrake;
```

You can obtain arbitrary input channels with:

```c++
RC_Channel *channel(uint8_t chan) { return rc().channel(chan-1); } // Get an input RC channel, 1-indexed.
RC_Channel *find_channel_for_option(RC_Channel::AUX_FUNC option) { return rc().find_channel_for_option(option); }
```

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

### Real Flight Testing

It is recommended that you always arm, take-off, land, and disarm while the main controller is running. You should switch to the custom controller while the vehicle is in level flight. This will reduce the effect of improper state resetting. You should arm and take off with the custom controller only if proper ground handling is implemented.

To test it on hardware compile with "--enable-custom-controller" flag.

```C++
./waf configure --board CubeOrangePlus plane --enable-custom-controller
```

### Post Flight Logs

Switching in and out of custom controller logged under the "CC" in log tree. You can see the time and duration the custom controller is active under "CC.Act"
