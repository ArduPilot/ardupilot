# ORCAMotor.lua - ArduPilot Lua Driver Documentation

The `ORCAMotor.lua` library is an ArduPilot Lua script driver designed to interface with Iris Dynamics ORCA linear motors via Modbus RTU over a serial interface. It handles command queuing, state monitoring, register configuration, auto-zeroing routines, and error logging/resolution.

---

## Prelude

* **Document Version:** 1
* **Date Modified:** 2026-11-19
* **Author(s):** Joey Boyer (Iris Dynamics)
* **Website:** [Iris Dynamics](https://irisdynamics.com)
* **Contact:** [Form Submission](https://irisdynamics.com/contact)

---

## Table of Contents

1. [Required Hardware](#required-hardware)
2. [Overview & Architecture](#overview--architecture)
3. [Constants & Configuration](#constants--configuration)
4. [ArduPilot Parameters](#ardupilot-parameters)
5. [Public Class Methods](#public-class-methods)
6. [Public Properties & Constants](#public-properties--constants)
7. [Internal Architecture & Helper Functions](#internal-architecture--helper-functions)
8. [Error Handling & Resolution](#error-handling--resolution)
9. [Limitations](#limitations)
10. [Example Usage](#usage)

---

## Required Hardware

We recommend getting familiar with your motor and testing it with our native IrisControls software prior to testing with Lua. Visit our website to access the ORCA motor reference manual.

* ORCA motor
* Power supply or battery (_Note_: Power specifications can be found in our [Orca Series](https://irisdynamics.com/products/orca-series) lineup under _View All Specifications_)
* RS422 to UART converter (Tested with a generic one)
* RJ45 female breakout connector

### Pin Configuration

| Pin Number | RJ45 Pin Signal | Plug Into | AP Pin Signal | Plug Into |
| :--- | :--- | :--- | :--- | :--- |
| 1 | RX2+ | T+ | VCC | VCC |
| 2 | RX2- | T- | TX | RX |
| 3 | TX2+ | R+ | RX | TX |
| 4 | — | — | CTS | — |
| 5 | — | — | RTS | — |
| 6 | TX2- | R- | GND | GND |
| 7 | +5V | +5V | — | — |
| 8 | GND | GND | — | — |

_Note_: When the motor is connected to an appropriate power source and turned on, it will supply 5V to pin 7. However, if the source is disabled, 5V will need to be supplied to pin 7 if motor configuration is desired. This could be useful if the motor is connected to an external battery or switched power supply that you may wish to have off prior to actuating the motor. Additionally, GND is not a requirement between the RS422-UART converter and the RJ45 breakout connector, as Modbus RS422 is fully differential. Double check the schematic and pin connections on your RS422 to UART converter, as TX and RX connections are easy to mix up.

## Overview & Architecture

The driver is built as a module that can be imported to a user script, exposing an `ORCAMotor` object whose public functions will control and interact with the motor. It uses an asynchronous transaction state machine: Commands triggered by user scripts or internal parameter loads are appended to a queue (`self.queue`) and every call to `ORCAMotor.update()` advances the steps of the transaction. To ensure that the motor's internal state is accurately represented in the driver, the driver uses a custom _Extended Motor Command_ Modbus frame to poll the motor constantly. The motor will exert a communications timeout error if the modbus frame interval exceeds 500ms (by default). We recommend setting an update rate between 50-100ms but this will vary based on the number of scripts running.

When `ORCAMotor:update()` is called:

1. **Response Checking:** Evaluates whether an in-flight Modbus request has received a response from the motor. Processes the response based on the type of request.
2. **Timeout Handling:** Times out requests exceeding `MODBUS_TIMEOUT_MS` (1000 ms).
3. **Queue Processing:** Dequeues and transmits the next pending command. Queue uses a FIFO flow, unless the command was forced to the front.
4. **Idle Polling:** If no commands are queued, it constructs an idle transaction based on `self.target_mode` and the associated `self.target_position` or `self.target_force` (in position or force mode)

### Modbus RTU

The Modbus RTU specification is highly standardized and provides a robust mechanism for simple communication. It relies on Modbus packets consisting of:

* Slave ID (1 byte)
* Function Code (1 byte)
* Data (length varies)
* CRC—Cyclic Redundancy Check (2 bytes)

The ORCA motor adheres to this specification and can utilize an extended set of function codes; it will only respond to well formed Modbus packets.

---

## Constants & Configuration

### Modbus Protocol Settings

* **Baud Rate:** `19200`
* **Parity:** `2` (Even)
* **Stop Bits:** `1`
* **Flow Control:** `0` (None)
* **Slave ID:** `1`
* **Timeout:** `1000 ms`

### Supported Function Codes (`MODBUS_FN_CODE`)

* `ORCA_EXT_MTR` (`0x66`): Custom Iris Dynamics command for streaming mode target and data.
* `READ_SINGLE_REG` (`0x03`): Read Modbus register.
* `WRITE_SINGLE_REG` (`0x06`): Write single 16-bit register.
* `WRITE_MULTI_REG` (`0x10`): Write multiple 16-bit registers.
* `ORCA_MTR_RD` (`0x68`): Custom Iris Dynamics for reading a motor stream.

---

## ArduPilot Parameters

When an instance is created, a parameter table with the prefix `ORCA<instance>_` is registered with ArduPilot. As always, save your parameters so that the motor configuration is maintained across reboots. Also note that not all parameters are validated prior to being sent to the motor as different motor models will have different limits. If you are unsure, leave advanced user settings to their defaults, or contact Iris Dynamics for support.

| Parameter | Index | Default | Description |
| :--- | :--- | :--- | :--- |
| `ENABLE` | 1 | `0` | Enable driver. Reboot recommended after change (`0`: Disabled, `1`: Enabled). |
| `FORCE_SAT` | 2 | `300000` | Limits maximum force ($mN$) in either direction ($\text{0-300,000}$). |
| `P_PID` | 3 | `1000` | Proportional gain ($N/mm$, scaled $64\times$). |
| `I_PID` | 4 | `2000` | Integral gain ($N/mm/s$, scaled $64\times$). |
| `D_PID` | 5 | `100` | Derivative gain ($N \cdot s/mm$, scaled $2\times$). |
| `SPD_LIM` | 6 | `1000` | Max velocity ($mm/s$) in Position Mode ($0 = \infty$). |
| `ACC_LIM` | 7 | `1000` | Max acceleration ($mm/s^2$) in Position Mode. ($0 = \infty$) |
| `DEC_LIM` | 8 | `1000` | Max deceleration ($mm/s^2$) in Position Mode. ($0 = \infty$) |
| `SOFT_MS` | 9 | `3000` | Softstart duration ($ms$) for position force ramping, when entering Position mode. |
| `INV_POS` | 10 | `0` | Invert positive shaft motion direction (`0`: Disabled, `1`: Enabled). |
| `AZ_MODE` | 11 | `0` | Auto-zero mode (`0`: Zero negative positions, `1`: Manual, `2`: Enabled, `3`: On boot). |
| `AZ_FORCE` | 12 | `30` | Auto-zero max force ($N$). |
| `AZ_SPD` | 13 | `50` | Auto-zero shaft speed ($mm/s$). |
| `APPLY` | 14 | `0` | Write `1` to trigger writing current parameter table to motor Modbus registers. |
| `ZERO_POS` | 15 | `0` | Write `1` to trigger zeroing current position. |
| `MODE_SW` | 16 | `0` | Switch the target mode (`0`: No change, `1`: Sleep, `2`: Force, `3`: Position, `5`: Kinematic, `55`: Auto Zero) |

---

## Public Class Methods

### `ORCAMotor.new(port_num, instance_num)`

Constructor. Initializes the instance, registers parameters, and configures the specified UART port.

* **Parameters:**
  * `port_num` (_number_): Serial port index to bind via `serial:find_serial()`.
  * `instance_num` (_number_): Unique instance index (affects parameter naming, e.g., `ORCA1_`).
* **Returns:** `ORCAMotor` object instance.
* **Note:** The constructor initialises the default mode to SLEEP, and the position/force targets to 0.

---

### `ORCAMotor:update()`

Main state machine execution function. **Must be called periodically in the main script loop** (recommended rate: every 50–100 ms).

* **Behavior:**
  * Checks for parameter trigger flags (`APPLY`, `ZERO_POS`).
  * Processes inbound serial data.
  * Flushes expired commands.
  * Transmits queued or periodic idle frames.

---

### `ORCAMotor:set_target_mode(target_mode)`

Queues a operational mode change command.

* **Parameters:**
  * `target_mode` (_number_): Target mode constant (`ORCAMotor.MODE.SLEEP`, `FORCE`, `POSITION`, `KINEMATIC`, or `AUTO_ZERO`).
* **Returns:** Command status handle.

**Note:** If setting to `AUTO_ZERO`, ensure the `AZ_MODE` param has been set to 2 or 3

---

### `ORCAMotor:set_target_position_um(target_pos)`

Queues a position control command and updates the operational mode to Position Mode. If negative positions are desired, ensure the `AZ_MODE` parameter is set to 1, and the shaft is zeroed near the center.

* **Parameters:**
  * `target_pos` (_number_): Target shaft position in micrometers ($\mu m$).
* **Returns:** Command status handle.
* **Note:** This function only needs to be called when a **change** in position is desired. The idle command will maintain the position so long as the mode does not change. The `set_target_force_mN()` function behaves similarly. Additionally, this function will not sanitize or verify the position you provide as our motors are extensively customizable and support negative positions. It will exert a lot of current trying to reach a target that the shaft length cannot achieve. Prevent this from occuring by always zeroing at startup, and ensuring your supplied targets are in reach.

---

### `ORCAMotor:set_target_force_mN(target_force)`

Queues a force control command and updates the operational mode to Force Mode.

* **Parameters:**
  * `target_force` (_number_): Target force in millinewtons ($mN$).
* **Returns:** Command status handle.

---

### `ORCAMotor:load_params()`

Queues Modbus register writes for all active ArduPilot parameter values to sync driver settings to motor control registers. This function must be called at some point in the script whenever the motor reboots (not required if 5V pin is maintained). Calling this function in code, or using `param set ORCAx_APPLY 1` do the same thing.

---

### `ORCAMotor:zero_current_position()`

Queues a command to set the current physical shaft position as the zero position reference ($0\ \mu m$).

* **Returns:** Command status handle.

---

### `ORCAMotor:clear_motor_errors()`

Queues a command to reset latched motor errors by writing to control register 0.

* **Returns:** Command status handle.

---

### `ORCAMotor:set_auto_zero_timeout(ms)`

Sets the maximum allowed duration for an auto-zero routine before timing out.

* **Parameters:**
  * `ms` (_number_): Timeout period in milliseconds.

---

### `ORCAMotor:get_auto_zero_running()`

Queries whether an auto-zero procedure is currently active.

* **Returns:** `boolean` (`true` if running).

---

### `ORCAMotor:set_kin_motion(id, position, time, delay, shape, auto_next, next_id)`

Configure the kinematic motion for a given ID.

* **Parameters**
  * `id` (_number_): Motion ID from 0-31 (inclusive).
  * `position` (_number_): Target shaft position in micrometers ($\mu m$).
  * `time` (_number_): Target time to perform the motion in milliseconds. Set to 0 for maximum speed.
  * `delay` (_number_): Hold duration for the position target.
  * `shape` (`ORCAMotor.KIN_SHAPE`): Configures the acceleration profile of the motion (`MIN_POWER` or `MAX_SMOOTHING`).
  * `auto_next` (`boolean`): Whether to automatically move to the next motion ID.
  * `next_id`: Motion ID of the next motion, 0-31 (inclusive).
* **Returns:** Command status handle.
* **Note:** Motions can be configured to be infinitely repeating by setting the last motion's next ID to the starting ID, and `auto_next` to `true` for all motions.

---

### `ORCAMotor:set_kin_home(id)`

Set the Home ID of the kinematic motions, which will execute immediately when entering Kinematic mode

* **Parameters**
  * `id` (_number_): Motion ID from 0-31 (inclusive).
* **Returns:** Command status handle.

---

### `ORCAMotor:trigger_kinematic_motion(id)`

Enters the kinematic mode and triggers a specific motion

* **Parameters**
  * `id` (_number_): Motion ID from 0-31 (inclusive).
* **Returns:** Command status handle.
* **Note:** Recommend already being in kinematic mode before calling this function, otherwise the Home ID will execute for a brief moment before jumping to specified ID.

---

### `ORCAMotor:get_current_kinematic()`

Get a copy of the current kinematic status

* **Returns:** Table containing current ID and status, see below.

---

## Public Properties & Constants

### Operational Modes (`ORCAMotor.MODE`)

* **SLEEP:** The motor will exact a slight braking force while minimizing current.
* **FORCE:** Set the motor to output a constant force.
* **POSITION:** Set the motor to target a specific position.
* **KINEMATIC:** Motor performs a pre-programmed routine of positions.
* **AUTO_ZERO:** Trigger the motor auto zeroing routine.

```lua
ORCAMotor.MODE = {
    SLEEP     = 1,
    FORCE     = 2,
    POSITION  = 3,
    KINEMATIC = 5,
    AUTO_ZERO = 55
}
```

### Command Status (`ORCAMotor.STATUS`)

The command status in the handle of a transaction will be updated as the transaction progresses. It is not necessary to utilize this handle but is useful for developing extended error handling. All public motor command functions except `load_params()` return a handle `{ id = <ms>, status = <STATUS> }`

```lua
-- ===snip===
ORCAMotor.STATUS = {
    QUEUED     = 0,
    PROCESSING = 1,
    OK         = 2,
    ERROR      = 3
}

-- Grab the status like so
local my_handle = motor:set_target_position_um(15000)

if my_handle.status == ORCAMotor.STATUS.ERROR then
  -- Take some corrective measure
end
```

### Kinematic Status

Calling `ORCAMotor.get_current_kinematic()` will return a table with the current kinematic status and the ID

```lua
-- ===snip===
ORCAMotor.KIN_STATUS = { 
  DONE = 0, -- Motion is done or not running
  WAITING = 1, -- Waiting to enter kinematic mode
  RUNNING = 2  -- Motion is running
}

if motor:get_current_mode() == ORCAMotor.MODE.KINEMATIC then
  local kin = motor:get_current_kinematic()
  if kin.status == ORCAMotor.KIN_STATUS.DONE then
    -- Do something after finish
  else
    gcs:send_text(7, string.format("Current motion: %d", kin.id))
  end 
end
```

### State Telemetry (`self.state`)

Updated automatically following every successful `ORCA_EXT_MTR` command response. During a kinematic routine this state is only partially updated, see the `read_motor_schema` table for details.

* `force_mN` (_int32_): Force generated by motor ($mN$).
* `position_um` (_int32_): Current shaft position ($\mu m$).
* `speed_mm_s` (_int32_): Shaft linear speed ($mm/s$).
* `accel_mm_s_2` (_int32_): Shaft acceleration ($mm/s^2$).
* `board_temp_C` (_int16_): Board temperature ($^\circ C$).
* `coil_temp_C` (_int16_): Coil temperature ($^\circ C$).
* `voltage_mV` (_uint16_): Bus voltage ($mV$).
* `power_W` (_uint16_): Electrical power consumed ($W$).
* `errors` (_uint16_): Active error bitmask.
* `motor_status` (_uint16_): Status flag registry bitmask.

---

## Internal Architecture & Helper Functions

The `ORCAMotor.lua` driver is structured around a non-blocking queue system that handles Modbus RTU communications over serial within ArduPilot's scripting environment.

### Communication & State Machine

* **Queue-Driven Messaging:** Communication is managed through an `in_flight` state machine and a `queue` array. Commands queued by public API methods wait until active transactions are acknowledged or time out ($1000\text{ ms}$). The queue can be jumped if `true` is passed as the argument for `front` of `_queue_write_register()`
* **Idle Telemetry/Control:** When no user commands are pending in the queue, `_build_idle_cmd()` constructs periodic control/telemetry requests (using the custom `ORCA_EXT_MTR` command or single-register reads) to continuously update state data.
* **Frame Validation:** Inbound Modbus frames are verified using `verify_modbus_rx_header()`, which calculates standard Modbus CRCs using `generate_crc()` alongside lookup tables (`crc_hi_lookup`, `crc_lo_lookup`).

### Low-Level Serialization & Helpers

* **Binary Parsing (`parse`):** Bitwise binary parsing functions (`parse.u8`, `parse.i8`, `parse.u16`, `parse.i16`, `parse.u32`, `parse.i32`) convert raw response byte arrays into signed and unsigned 16-bit and 32-bit integers (using two's complement where appropriate).
* **Schema Mapping (`ext_motor_schema` and `read_motor_schema`):** Incoming bytes from the 42-byte Extended Motor Command response or 24-byte Motor Read Stream Command are mapped against a schema table, populating properties inside `motor.state` (such as force, position, temperature, and status bitmasks).
* **Payload Builders:** Internal functions `build_ext_tx_payload()` and `build_write_tx_payload()` serialize targets, modes, and register addresses into byte arrays before adding headers and CRCs.

Custom configuration functions can be made by simply using `_queue_write_register()` and the ORCA Memory Map.

---

## Error Handling & Resolution

Errors reported in `self.state.errors` are parsed against `ORCA_ERR` masks and handled according to assigned resolution protocols:

| Error Flag | Mask | Severity | Resolution Protocol | Action Taken |
| :--- | :--- | :--- | :--- | :--- |
| `CONFIG` | `0x0001` | ERROR | `RESET` | Restores factory tuning, saves to flash, resets motor. |
| `FORCE_CLIP` | `0x0020` | NOTICE | `AUTO` | Logged to GCS; resolved automatically by driver. |
| `TEMP_EXCEEDED` | `0x0040` | WARNING | `SLEEP` | Commands motor state to `SLEEP`. |
| `FORCE_EXCEEDED` | `0x0080` | NOTICE | `AUTO` | Logged to GCS; resolved automatically by driver. |
| `POWER_EXCEEDED` | `0x0100` | ERROR | `SLEEP` | Commands motor state to `SLEEP`. |
| `SHAFT_FAILED` | `0x0200` | ERROR | `EXTERNAL_AND_SLEEP` | Prompts user to re-insert shaft; enters `SLEEP`. |
| `VOLTAGE_INVALID` | `0x0400` | ERROR | `EXTERNAL` | Prompts user to check external power supply. |
| `COMM_TIMEOUT` | `0x0800` | WARNING | `SLEEP` | Commands motor state to `SLEEP`. |
| `AUTO_ZERO_FAIL` | `0x2000` | WARNING | `SLEEP` | Commands motor state to `SLEEP`. |

If the motor error persists, it will log to the GCS every 10 seconds until the corrective measure is taken. Any errors internal to the Lua driver (if a frame fails a CRC check, for example) will only be logged once when it occurs.

---

## Limitations

* **SITL & WSL serial frame dropout:** When testing the ORCA motor with SITL and WSL, there is an occasional random frame that fails to be received by the motor (1 in ~5000). If not in SLEEP mode, this causes a motor timeout error, preventing actuation and requiring a manual swap to SLEEP mode to clear. This issue _does not_ occur when testing with an actual flight controller.
* **Feature Parity versus orcaSDK:** This driver is not as extensive as the C++, Python, and Matlab drivers we provide for our motors, primarily due to the constrained resources of flight controllers. As this driver adoption increases, we will identifify what features to make available that would benefit a majority of ArduPilot users.

---

## Usage

### Steps

1. Enabling scripting with `SCR_ENABLE 1`
2. Set the connected serial parameters `SERIALx_PROTOCOL 28` and `SERIALx_BAUD 19` where `x` is the connected pin number
3. Copy the **ORCAMotor.lua** file into `APM/scripts/modules/`
4. Copy your user script into `APM/scripts/`
5. If desired, load an ORCA parameter file with `param load <file>` and `reboot`

### Sample Scripts

See `examples/ORCAMotor_pitch_control.lua` and `examples/ORCAMotor_kinematics.lua` for some easy tests to get your motor up and running!

### Sample Parameter File

For initial configuration, we recommend creating a separate ORCA parameter file and loading it with `param load <file>`. Once loaded, save the parameters, and then any subsequent script runs just needs a call to `load_params()` or `param set ORCAx_APPLY 1`

**Note:** By setting the final parameter to `ORCAx_APPLY 1`, loading this parameter file will trigger the routine to apply the parameters to the motor.

```parm
ORCA1_ENABLE 1
ORCA1_FORCE_SAT 200000
ORCA1_P_PID 800
ORCA1_I_PID 1100
ORCA1_D_PID 150
ORCA1_SPD_LIM 750
ORCA1_ACC_LIM 900
ORCA1_DEC_LIM 950
ORCA1_SOFT_MS 100
ORCA1_AZ_MODE 2
ORCA1_AZ_FORCE 15
ORCA1_AZ_SPD 65
ORCA1_APPLY 1
```

### Extending the Feature Set

As specified by the limitations, this driver does not provide complete customisation or functionality of the ORCA motor. Our product supports haptics, kinematic motions, PWM controllers, Ethernet-IP, analog/digital controllers, and more. Development with ArduPilot will be an ongoing task. If you would like to add certain features, develop the driver using the ORCA reference manual and ORCA Modbus RTU document, or contact us for support.
