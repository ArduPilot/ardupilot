# **Flip On Switch (Advanced)**

This applet allows a drone to perform complex flip maneuvers initiated by an RC switch. It utilizes the `vehicle_control.lua` module for the underlying flight control logic. The script supports two distinct operational modes: a **Simple Mode** for continuous flips and an **Advanced Mode** for defining maneuvers through switch gestures.

## **How to Use**

1.  Ensure `vehicle_control.lua` is present in the `APM/scripts/modules` directory on your autopilot's SD card.
2.  Copy the `flip_on_a_switch.lua` script to the `APM/scripts/` directory.
3.  Enable Lua scripting by setting `SCR_ENABLE` to 1 and reboot the autopilot.
4.  Configure the script's parameters to match your vehicle and desired maneuver.
5.  Set up an RC switch according to the desired operational mode below.

---

## **Parameters**

* **FLIP_ENABLE**: Set to 1 to enable the script. Default is 1.
* **FLIP_AXIS**: The axis for the flip. 1 for Roll, 2 for Pitch. Default is 1 (Roll).
* **FLIP_RATE**: The target rotation rate for the flip in degrees per second. Default is 720.
* **FLIP_THROTTLE**: The throttle level to use during the flip's ballistic phase (0-1). A value of -1 will cut the throttle entirely. Default is 0.0.
* **FLIP_HOVER**: The vehicle's true hover throttle (0-1). This is critical for the script's physics calculations. Default is 0.125.
* **FLIP_CLIMB_G**: The desired G-force for the initial climb (e.g., 1.0 for 1g). Higher values result in a more aggressive climb. Default is 1.0.
* **FLIP_CHAN**: Selects the control mode. 0 for Simple Mode, or the raw RC channel number (1-16) for Advanced Mode. Default is 0.
* **FLIP_NUM**: (Simple Mode Only) The number of flips to perform in a single continuous sequence when the switch is held high. Default is 1.
* **FLIP_FLICK_TO**: (Advanced Mode Only) The time in seconds to differentiate a "flick" from a "hold". Any switch activation shorter than this is a flick. Default is 0.5.
* **FLIP_COMMIT_TO**: (Advanced Mode Only) The timeout in seconds after the last switch input before the maneuver is committed and starts. Default is 0.75.

---

## **RC Switch Setup**

The setup depends on the mode selected with the `FLIP_CHAN` parameter.

### **Simple Mode Setup (`FLIP_CHAN = 0`)**

This mode uses an auxiliary function switch. Choose an unused `RCx_OPTION` parameter, where `x` is the channel number of your switch.

* Set `RCx_OPTION` to **300** (Scripting1).

### **Advanced Mode Setup (`FLIP_CHAN > 0`)**

This mode reads the raw PWM value from an RC channel directly.

* Set `FLIP_CHAN` to the RC channel number you wish to use (e.g., set to 9 to use RC9).
* Ensure the corresponding `RC9_OPTION` is set to 0 (Do Nothing).

---

## **Operation**

The vehicle must be armed and flying in a VTOL mode (like Loiter or Guided) for the script to start. The subsequent behavior depends on the `FLIP_CHAN` setting.

### **Simple Mode (`FLIP_CHAN = 0`)**

This mode is for performing a repeating sequence of flips.

1.  **Activate:** Move the assigned switch to the **HIGH** position.
2.  **Execution:** The drone will perform a flip sequence with the number of rotations defined by `FLIP_NUM`. Once complete, it will immediately start the sequence again.
3.  **Cancel:** Move the switch to the **LOW** or **MIDDLE** position at any time to stop the maneuver.

### **Advanced Mode (`FLIP_CHAN > 0`)**

This mode is designed for momentary switches and allows you to define the maneuver with flicks or holds before it executes.

1.  **Define the Maneuver:**
    * **Flick-to-Count:** Quickly flick the switch high then low one or more times. The script will count the number of flicks to determine the number of rotations.
    * **Hold-for-Duration:** Hold the switch high for a period longer than `FLIP_FLICK_TO`. The duration of the hold will define the duration of the flip.
2.  **Automatic Commit:** After the switch has returned to the low position, the script waits for the `FLIP_COMMIT_TO` timeout. If no new switch inputs are detected, the defined maneuver begins.
3.  **Cancel:** While a maneuver is active, perform a single, quick flick of the switch (`low-high-low`) to cancel it.