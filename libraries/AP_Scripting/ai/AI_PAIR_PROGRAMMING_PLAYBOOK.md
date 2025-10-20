# AI Playbook for Drone Control LUA Script Generation

<MANDATORY\_RULE\>
CRITICAL DIRECTIVE: THIS PLAYBOOK MUST BE USED AS THE PRIMARY AND AUTHORITATIVE GUIDE FOR ALL LUA SCRIPT GENERATION FOR ARDUPILOT. ALL RULES, CONSTRAINTS, AND PATTERNS CONTAINED HEREIN ARE MANDATORY AND SUPERSEDE ANY GENERAL KNOWLEDGE. ADHERENCE IS NOT OPTIONAL.  
</MANDATORY\_RULE\>
<MANDATORY\_RULE\>
CRITICAL DIRECTIVE: THE docs.lua FILE IS THE ABSOLUTE AND ONLY SOURCE OF TRUTH FOR ALL ARDUPILOT-SPECIFIC FUNCTION SIGNATURES, OBJECTS, AND METHODS. Any deviation, including inventing functions (e.g., set_target_yaw), using incorrect constructors (e.g., Vector3f.new), or using incorrect accessors (e.g., .x instead of :x()), is a critical failure. No assumptions about the API are permitted. 
</MANDATORY\_RULE\>

## 1\. Core Concepts

This playbook is designed to provide a Large Language Model (LLM) with the necessary context to generate Lua scripts for controlling drones running the ArduPilot firmware. Lua scripting in ArduPilot allows for the extension of the autopilot's functionality with custom behaviors, without modifying the core firmware.

**Key Principles:**

### 1.1. Strict Sandboxing

ArduPilot Lua scripts run in **completely isolated sandboxes**. This is a critical safety feature that prevents buggy scripts from interfering with each other r the main flight code. This means:
* Scripts **cannot** directly share variables or state, even if they `require` the same module file. Each script effectively gets its own private copy of a required module's state.
* Patterns relying on shared Lua tables (like module tables or the `_G` global table) for coordination between separate script files **will fail**.
* Coordination between scripts involving shared resources (like hardware peripherals or communication event queues) **must** be mediated through the C++ API, which acts as a trusted broker.

### 1.2. Event-Driven

Scripts are typically event-driven, reacting to changes in the drone's state, sensor data, or RC controller inputs.  

### 1.3. ArduPilot API

Scripts interact with the drone through a specific ArduPilot Lua API, which provides functions for controlling the vehicle, reading sensors, and more.

## 2\. Environment Setup

For a Lua script to be used, the drone's flight controller must be configured to support scripting.

* **Enable Scripting:** In the ArduPilot parameters, SCR\_ENABLE must be set to 1\.  
* **Upload Scripts:** Scripts are uploaded to the APM/scripts directory on the flight controller's SD card.  
* **Reboot:** The flight controller must be rebooted after enabling scripting or uploading new scripts.

## 3\. ArduPilot LUA API Reference

This section details the most common functions and libraries available in the ArduPilot Lua API.

For a definitive list of all available function signatures, the `docs.lua` file is the absolute source of truth.

For a task-oriented guide on how to use these functions to accomplish common drone behaviors (e.g., controlling movement, reading sensors, managing missions), please refer to the **AI_VEHICLE_CONTROL_PLAYBOOK.md**. The `AI_VEHICLE_CONTROL_PLAYBOOK.md` organizes the API by function rather than by library.


### 3.1. Vehicle Control

* **vehicle:get\_location()**: Returns a Location object with the current position of the drone.  
* **vehicle:set\_target\_location(Location)**: Commands the drone to fly to a specific Location.  
* **vehicle:set\_target\_velocity\_NED(Vector3f)**: Sets the drone's target velocity in a North-East-Down (NED) frame.  
* **vehicle:arm()**: Arms the drone.  
* **vehicle:disarm()**: Disarms the drone.  
* **vehicle:get\_mode()**: Returns the current flight mode of the drone.  
* **vehicle:set\_mode(mode)**: Sets the drone's flight mode (e.g., GUIDED, LOITER, RTL).  
* **vehicle:start\_takeoff(altitude)**: Initiates an auto-takeoff to the specified altitude.

**Example:**

\-- Fly to a GPS coordinate  
local target\_location \= Location()  
target\_location:lat(47.397742)  
target\_location:lng(8.545594)  
target\_location:alt(10) \-- 10 meters altitude  
vehicle:set\_target\_location(target\_location)

### 3.2. GPS

* **gps:num\_sensors()**: Returns the number of connected GPS sensors.  
* **gps:status(instance)**: Returns the fix status of a specific GPS sensor.  
* **gps:location(instance)**: Returns a Location object with the position from a specific GPS sensor.  
* **gps:primary\_sensor()**: Returns the index of the primary GPS sensor.

### 3.3. Sensors

* **rangefinder:num\_sensors()**: Returns the number of connected rangefinders.  
* **rangefinder:distance(instance)**: Returns the distance measured by a specific rangefinder.  
* **battery:num\_instances()**: Returns the number of connected batteries.  
* **battery:voltage(instance)**: Returns the voltage of a specific battery.  
* **battery:remaining\_capacity(instance)**: Returns the remaining capacity of a specific battery.

### 3.4. RC Channels

* **rc:get\_channel(channel\_num)**: Returns the current PWM value of a specific RC channel.  
* **rc:get\_aux\_cached(aux\_channel)**: Returns the cached value of an auxiliary channel.

## 4\. Available Scripts

The ArduPilot repository contains a wide variety of pre-written Lua scripts that can be used as-is or adapted for specific needs. These scripts are categorized into applets, drivers, examples, and tests.

### 4.1. Applets

Applets are complete, ready-to-use scripts that require no user editing. They often provide high-level functionality and can be enabled through parameters. Each applet is accompanied by a markdown file (.md) that details its operation.

* BattEstimate  
* BatteryTag  
* Gimbal\_Camera\_Mode  
* Heli\_IM\_COL\_Tune  
* Heli\_idle\_control  
* Hexsoon LEDs  
* MissionSelector  
* ONVIF\_Camera\_Control  
* Param\_Controller  
* QuadPlane\_Low\_Alt\_FW\_mode\_prevention  
* RockBlock  
* Script\_Controller  
* SmartAudio  
* UniversalAutoLand  
* VTOL-quicktune  
* advance-wp  
* ahrs-set-origin  
* ahrs-source-extnav-optflow  
* camera-change-setting  
* copter-deadreckon-home  
* copter-slung-payload  
* copter\_terrain\_brake  
* crsf-calibrate  
* follow-target-send  
* forward\_flight\_motor\_shutdown  
* leds\_on\_a\_switch  
* motor\_failure\_test  
* mount-poi  
* net\_webserver  
* param-set  
* pelco\_d\_antennatracker  
* plane\_package\_place  
* plane\_precland  
* plane\_ship\_landing  
* repl  
* revert\_param  
* rover-quicktune  
* runcam\_on\_arm  
* video-stream-information  
* winch-control  
* x-quad-cg-allocation  
* **Aerobatics:**  
  * plane\_aerobatics  
  * sport\_aerobatics  
* **WebExamples:**  
  * test.lua  
  * test.shtml

### 4.2. Drivers

Drivers provide support for specific hardware or protocols.

* BattMon\_ANX  
* EFI\_DLA  
* EFI\_HFE  
* EFI\_Halo6000  
* EFI\_NMEA2k  
* EFI\_SkyPower  
* Generator\_SVFFI  
* Hobbywing\_DataLink  
* INF\_Inject  
* LTE\_modem  
* UltraMotion  
* mount-djirs2-driver  
* mount-viewpro-driver  
* torqeedo-torqlink  
* **TOFSense-M:**  
  * TOFSENSE-M\_CAN  
  * TOFSENSE-M\_Serial

### 4.3. Examples

Examples provide demonstrations of specific functionalities and can be used as a starting point for custom scripts.

* 6DoF\_roll\_pitch  
* AHRS\_switch  
* BQ40Z\_bms\_shutdown  
* CAN\_MiniCheetah\_drive  
* CAN\_logger  
* CAN\_read  
* CAN\_write  
* DroneCAN\_test  
* EFI\_tester  
* ESC\_slew\_rate  
* FenceBreach  
* FlexDebug  
* Flip\_Mode  
* LED\_matrix\_image  
* LED\_matrix\_text  
* LED\_poslight  
* LED\_roll  
* MAVLinkHL  
* MAVLink\_Commands  
* Mission\_test  
* MotorMatrix\_dotriaconta\_octaquad\_x  
* MotorMatrix\_fault\_tolerant\_hex  
* MotorMatrix\_hexadeca\_octa  
* MotorMatrix\_hexadeca\_octa\_cw\_x  
* MotorMatrix\_setup  
* Motor\_mixer\_dynamic\_setup  
* Motors\_6DoF  
* NMEA-decode  
* OOP\_example  
* RCIN\_test  
* RC\_override  
* RM3100\_self\_test  
* SN-GCJA5-particle-sensor  
* Safety\_States  
* Serial\_Dump  
* UART\_log  
* active\_source\_set  
* ahrs-print-angle-and-rates  
* ahrs-print-home-and-origin  
* ahrs-print-variances  
* ahrs-set-home-to-vehicle-location  
* ahrs-source-gps-optflow  
* ahrs-source-gps-wheelencoders  
* ahrs-source  
* analog\_input\_and\_GPIO  
* arming-check-batt-temp  
* arming-check-wp1-takeoff  
* aux\_cached  
* battery\_internal\_resistance\_check  
* benewakeH30\_can\_rangefinder  
* button\_test  
* camera-test  
* cell\_balance\_check  
* command\_int  
* copter-circle-speed  
* copter-fast-descent  
* copter-fly-vertical-circle  
* copter-nav-script-time  
* copter-posoffset  
* copter-wall-climber  
* copter\_alt\_offset  
* copter\_deploy  
* copter\_pingpong  
* copy\_userdata  
* crosstrack\_restore  
* crsf-menu  
* easter-egg  
* efi\_speed\_check  
* esc\_rpm\_scale  
* fault\_handling  
* frsky\_battery  
* frsky\_rpm  
* frsky\_wp  
* fw\_vtol\_failsafe  
* gen\_control  
* get\_notify\_RGB  
* glide\_into\_wind  
* gps\_synth  
* hello\_world  
* hello\_world\_display  
* i2c\_scan  
* jump\_tags\_calibrate\_agl  
* jump\_tags\_into\_wind\_landing  
* land\_hagl  
* lidar\_control  
* logging  
* mag\_heading  
* message\_interval  
* mission-edit-demo  
* mission-load  
* mission-save  
* mission\_spiral  
* motor\_lost  
* mount-test  
* net\_test  
* notch\_switch  
* opendog\_demo  
* orbit\_follow  
* param\_add  
* param\_get\_set\_test  
* plane-callout-alt  
* plane-doublets  
* plane-wind-failsafe  
* plane-wind-fs  
* plane\_guided\_follow  
* plane\_guided\_terrain  
* protected\_call  
* proximity\_test  
* qnh\_alt  
* quadruped  
* rangefinder\_quality\_test  
* rangefinder\_test  
* readstring\_test  
* relay\_control  
* rgb\_led\_test  
* rgb\_notify\_patterns  
* rover-MinFixType  
* rover-SaveTurns  
* rover-TerrainDetector  
* rover-motor-driver  
* rover-set-steering-and-throttle  
* rover-set-turn-rate  
* serial\_test  
* servo\_scan  
* servo\_set\_get  
* servo\_slew  
* set-angle  
* set-target-location  
* set-target-velocity  
* set\_CAMERA\_INFORMATION  
* set\_target\_posvel\_circle  
* ship\_vel\_match  
* sim\_arming\_pos  
* simple\_loop  
* sitl\_standby\_sim  
* smbus-check-cycles  
* sub\_test\_synthetic\_seafloor  
* temperature\_sensor  
* terrain\_warning  
* test\_get\_target\_location  
* test\_load  
* test\_script\_button  
* test\_update\_target\_location  
* trisonica-mini  
* wp\_test  
* wrap32\_test

## 5\. Code Generation Constraints

The following constraints apply to all Lua code generation:

### 5.1. Lua Version and Libraries

* **Lua Version:** All generated code must be compatible with Lua 5.3.  
* **API Source of Truth:** The docs.lua file is the **definitive source of truth** for all ArduPilot-specific function signatures. In cases of discrepancy between examples and this documentation, the docs.lua file takes precedence.  
* **Do not assume functions exist** If a required mathematical helper function (like dot() or cross()) is missing from an API object, the required logic must be implemented manually in Lua. For vector manipulation prefer Vector3f to Vector2f since it has a much richer API.
* **Allowed Functions:** Functions are limited to:  
  * Standard Lua 5.3 language features.  
  * Functions documented in the provided docs.lua file.  
  * The standard io library for file operations.  
  * The require() function, for loading modules from the script's local APM/scripts directory.
* **API Interaction Subtleties:** The order of operations when calling C++ API functions can be critical, especially when functions have side effects related to internal state (e.g., needing to pop a shared event *before* sending a response related to that event to ensure the necessary context is prepared in the C++ backend).

### 5.2. Script Structure and Execution

* **No Threads:** Scripts must not create or manage their own threads. The ArduPilot environment handles script execution in a single-threaded manner.  
* **Non-Blocking:** Scripts must not contain any blocking calls or long-running loops. Each execution of the script's main function should complete within a few milliseconds.  
* **Update Function Pattern:** The required structure depends on the script's purpose:  
  * **Applets (Continuous Tasks):** Must use a main update() function that performs its tasks and then reschedules itself to run again at a regular interval.  
  * **One-Shot Scripts:** Can execute logic sequentially and terminate without an update() loop.  
  * **Test Scripts:** Typically run a series of assert() checks and may optionally enter a simple loop to report a "tests passed" message.
* **Shared Resource Handling (e.g., Event Queues):** When multiple independent scripts need to access a shared, limited C++ resource like an event queue, special care must be taken to avoid race conditions. A robust pattern involves:
  1.  Using a non-destructive "peek" function (if provided by the C++ API) to check the resource state (e.g., see the next event).
  2.  Checking if the resource state/event belongs to the current script.
  3.  If it belongs, processing it and then using a destructive "pop" function (if provided) to consume the resource/event.
  4.  If it does *not* belong, yielding execution quickly to allow other scripts a chance to process it. (See the CRSF Menu Playbook for a detailed example).
* **Error Handling:** A protected\_wrapper using pcall() is mandatory for all applets. It is also required for any example or test script where a runtime error is possible (e.g., interacting with hardware, complex calculations). For very simple, infallible example scripts (like hello\_world.lua), the wrapper can be omitted for clarity.  
  **Example protected\_wrapper:**
```lua
  function protected_wrapper()
    local success, err = pcall(update)
    if not success then
       gcs:send_text(MAV\_SEVERITY.ERROR, "Internal Error: " .. err)
       -- Reschedule with a longer delay after an error
       return protected_wrapper, 1000
    end
    -- Reschedule with the normal update interval
    return protected_wrapper, 100
  end
```

### 5.3. Initial Condition Checks

* **Use assert():** Scripts should use the assert() function at the beginning to validate essential preconditions. This ensures the script fails early and clearly if the environment is not correctly configured.  
  **Example assert() check:**  
```lua
  -- Check that a required parameter is set
  local my_param = assert(param:get('MYAPL_ENABLE'), 'MYAPL_ENABLE not set')

  -- Check that the vehicle is the correct type
  assert(vehicle:get_frame_class() == 1, 'Script requires a Quad frame')
```

### 5.4. Default Applet Behavior

  1. **Activation:** By default, all applets **must** be activatable via an RC switch.
  2. **RC Function Constant:** The Auxiliary Function number for the switch **must** be defined as a local constant in the script (e.g., `local SCRIPTING_AUX_FUNC = 300`). It **must not** be a user-configurable script parameter (e.g., `FIG8_LAND_SWITCH`). This simplifies user setup.
  3. Use rc:get\_aux\_cached(SCRIPTING\_AUX\_FUNC) to read the switch position (0=low, 1=middle, 2=high).  
  4. The documentation (.md file) must instruct the user to set their desired RCx\_OPTION to this number (e.g., "Set RC9\_OPTION to 300"). This removes a parameter from the script and simplifies user setup.
  5. **User Prompt Precedence:** If the user's prompt explicitly requests an activation method that differs from this default (e.g., "take off automatically on arm"), the user's request shall take precedence. However, the generated `.md` documentation **must** include a section explicitly noting this deviation from the standard applet pattern and explaining the custom activation logic.

**Example 3-Position Switch Logic (for `.lua` file):**

```lua
-- Define a constant for the auxiliary function
local SCRIPTING_AUX_FUNC = 300 -- Corresponds to "Scripting1"

-- In the script's main logic
function update()
    -- Directly use the hardcoded aux function number
    local switch\_pos = rc:get_aux_cached(SCRIPTING_AUX_FUNC)
    if switch_pos == 0 then
        -- Handle LOW position
    elseif switch_pos == 1 then
        -- Handle MIDDLE position
    elseif switch_pos == 2 then
        -- Handle HIGH position
    end
    return update, 200 -- reschedule
end
```

### **5.4.1 Applet Configuration Mandate**

**CRITICAL DIRECTIVE: ALL APPLET SCRIPTS MUST EXPOSE USER-CONFIGURABLE VALUES AS SCRIPT PARAMETERS.** Any value that a user might reasonably want to tune (e.g., distances, speeds, rates, durations, thresholds, modes of operation) **must not** be defined as a hard-coded local variable. Instead, it **must** be created using the `param:add_table()` and `param:add_param()` functions as detailed in section 5.7. This is a non-negotiable requirement for a script to be considered a complete "Applet".

**Violation Example (Incorrect):**

```lua
-- A local variable is used for a key configuration value.
local RADIUS = 20
```
**Compliance Example (Correct):**

```lua
-- The radius is exposed as a proper, user-configurable parameter.
local PARAM_TABLE_KEY = 105
local PARAM_TABLE_PREFIX = "FIG8_"
assert(param:add_table(PARAM_TABLE_KEY, PARAM_TABLE_PREFIX, 1), '...')

local FIG8_RADIUS = assert(param:add_param(PARAM_TABLE_KEY, 1, 'RADIUS', 20), '...')
```

This ensures that all generated applets are immediately usable and configurable by the end-user through standard Ground Control Station software, which is a primary goal of this system.

### 5.5. Deliverable Format and Autotest Generation

* **Assume Applet by Default:** Unless the user specifies otherwise, or the request is clearly for a simple example or test, the primary output should be a complete ArduPilot Applet.  
* **Applet Format:** An applet must include two files:  
  1. The Lua script file (.lua).  
  2. A corresponding Markdown documentation file (.md) that explains the applet's purpose, parameters, and usage.  
* **Example/Test Format:** If generating an example or test, follow the simpler structure observed in the repository (e.g., may omit headers, pcall wrappers, and documentation).  
* **Autotest Generation:** For every new applet, offer to generate a corresponding SITL autotest. The autotest is a Python script that runs in the ArduPilot simulation environment (SITL) to verify the script's behavior automatically.  
  **Mandatory Autotest Structure:**  
  1. **Test Location and Structure**: Autotests **must not be new files**. They must be added as new, self-contained methods to the appropriate existing vehicle-specific test suite. For example, a Copter-related script test must be a new method within a class in Tools/autotest/arducopter.py. If the script is not vehicle-specific, the test should be written for Copter by default. The test method must not introduce new global state but may use the existing test framework's helper functions and self object.  
  2. **install\_applet\_script\_context**: Use the self.install\_applet\_script\_context("my\_script.lua") context manager. This command handles copying the script into the correct SITL directory and ensures it's cleaned up after the test.  
  3. **Two-Stage Parameter Setup**:  
     * **Stage 1**: Set SCR\_ENABLE to 1\. If required, also enable any necessary hardware simulations (like RNGFND1\_TYPE). Then, **must** call self.reboot\_sitl(). This first reboot allows ArduPilot to recognize that scripting is enabled and to generate the parameters defined by the script.  
     * **Stage 2**: Now that the script's parameters exist, set them to the values required for the test (e.g., MYAPL\_ENABLE to 1, RC9\_OPTION to 300). Then, **must** call self.reboot\_sitl() a second time. This reboot ensures the script starts up with the correct parameter values.  
  4. **Test Logic**:  
     * Arm the vehicle (self.arm\_vehicle()) and change to the desired flight mode (self.change\_mode("LOITER")).  
     * Use self.context\_collect('STATUSTEXT') before performing an action that should generate a GCS message.  
     * Trigger the script's conditions. This may involve changing altitude (self.change\_alt(10)), flying to a location (self.fly\_to\_pos(...)), or setting an RC channel input (self.set\_rc(9, 2000)).  
     * Verify the outcome. The primary method for this is self.wait\_statustext("Expected message text", check\_context=True). You can also check for mode changes (self.wait\_mode("BRAKE")) or other vehicle state changes.  
  5. **Cleanup**: Disarm the vehicle (self.disarm\_vehicle()).

**Annotated Autotest Example (to be inserted into arducopter.py):**\# NOTE: This is an example of a method to be added to an existing class  
\# like 'class AutoTestCopter(vehicle\_test\_suite.TestSuite):'  
def do\_lua\_mynewapplet\_test(self):  
    '''Tests the my\_new\_applet.lua script'''  
    self.start\_subtest("Test MyNewApplet functionality")

    \# 2\. Install Script  
    with self.install\_applet\_script\_context("my\_new\_applet.lua"):

        \# 3\. Two-Stage Parameter Setup  
        self.set\_parameters({  
            "SCR\_ENABLE": 1,  
            "RNGFND1\_TYPE": 10, \# Enable SITL rangefinder for testing  
        })  
        self.reboot\_sitl()

        self.set\_parameters({  
            "MYAPL\_ENABLE": 1,  
            "RC9\_OPTION": 300, \# Corresponds to Scripting1 aux function  
            "MYAPL\_ALT": 15,  
        })  
        self.reboot\_sitl()

        \# 4\. Test Logic  
        self.wait\_ready\_to\_arm()  
        self.arm\_vehicle()  
        self.change\_mode("LOITER")  
        self.user\_takeoff(alt\_min=20)

        self.context\_collect('STATUSTEXT')  
        self.set\_rc(9, 2000\) \# Set RC9 high to trigger script  
        self.wait\_statustext("MyNewApplet: State changed to HIGH", check\_context=True, timeout=10)

        \# 5\. Cleanup  
        self.set\_rc(9, 1000\) \# Return RC switch to low  
        self.disarm\_vehicle()

### 5.6. Code Quality

* **Header Comments (Applets Only):** Every applet script must start with a comment block that briefly describes its purpose and functionality. The style should be concise and consistent with other applets in the ArduPilot repository. This is not required for examples or tests.  
* **State Change Feedback:** Applets must provide brief, clear feedback via gcs:send\_text(severity, text) when significant state changes occur (e.g., activation, mode change, action completed). These messages are the primary mechanism for verification in autotests.  
* **Use Enums for Constants:** Avoid using hardcoded integers ("magic numbers") for values like modes, states, or options. Instead, define a local table at the start of the script to act as an enumeration.  
  **Example Enum for gcs:send\_text:**
```lua
  -- Enum for MAV_SEVERITY levels. Using this is mandatory
  -- for gcs:send_text() instead of hardcoded numbers.
  local MAV_SEVERITY = {
      EMERGENCY = 0,
      ALERT = 1,
      CRITICAL = 2,
      ERROR = 3,
      WARNING = 4,
      NOTICE = 5,
      INFO = 6,
      DEBUG = 7
  }

  -- Usage:
  gcs:send_text(MAV_SEVERITY.INFO, "Script initialized")
```

* **luacheck Compliance:** All generated Lua code must be free of errors and warnings when analyzed with the luacheck tool, using the standard ArduPilot configuration.
* No Trailing Whitespace: All generated lines of code **must not** contain any trailing whitespace characters (spaces or tabs at the end of a line).

### 5.7. Parameter Creation

* **Strict Syntax:** Parameter creation is a strict two-step process that must be followed exactly as documented in docs.lua.  
  1. **Declare Table:** Use param:add\_table(table\_key, prefix, num\_params) to declare the parameter group. The prefix must **NOT** have a trailing underscore.  
  2. **Add Parameters:** Iterate through a local table of parameter definitions and add each one using param:add\_param(table\_key, param\_num, name, default\_value).  
* **Naming Convention:**  
  * The prefix should be a short, uppercase string (e.g., MYAPL).  
  * The name in the parameter definition should be the suffix (e.g., ENABLE).  
  * The final parameter name seen by the user is PREFIX\_NAME (e.g., MYAPL\_ENABLE).  
  * The total length of this full name **must not exceed 16 characters**.  
  * The full name must be used when getting/setting the parameter in Lua and in autotests.  
* **Unique Table Key:** The table\_key must be an integer between 1 and 200 and must be unique across all existing scripts in the ArduPilot repository. Do not reuse any of the following keys: 7, 8, 9, 10, 11, 12, 14, 15, 16, 31, 36, 37, 39, 40, 41, 42, 43, 44, 45, 48, 49, 70, 71, 72, 73, 75, 76, 78, 79, 80, 81, 82, 84, 85, 86, 87, 88, 89, 90, 91, 92, 93, 102, 104, 106, 109, 110, 111, 117, 136, 138, 139, 193\.  
  **Correct Parameter Creation Example:**
```lua
  -- This local table is for script organization only.  
  local parameter_definitions = {
      { name = "ENABLE", default = 0 },
      { name = "VALUE", default = 12.3 }
  }
  local PARAM_TABLE_KEY = 101
  local PARAM_TABLE_PREFIX = "MYAPL" -- Note: NO trailing underscore

  -- Step 1: Declare the table with its key, prefix, and the number of parameters.  
  assert(param:add_table(PARAM_TABLE_KEY, PARAM_TABLE_PREFIX, \#parameter_definitions), "Could not add param table")

  -- Step 2: Add each parameter individually using the correct signature.  
  for i, p_def in ipairs(parameter_definitions) do  
      assert(param:add_param(PARAM_TABLE_KEY, i, p_def.name, p_def.default), "Could not add param "..p_def.name)  
  end

  -- Usage in code: use the full, concatenated name.  
  local my_val \= param:get('MYAPL_VALUE')
```

### 5.7.1. Parameter Documentation

<MANDATORY_RULE>
CRITICAL DIRECTIVE: Every script parameter added via `param:add_param()` **must** be immediately preceded by a documentation block in the standard ArduPilot format. This is required for the parameter to be visible and editable in Ground Control Station software. The format is a multi-line comment block (`--[[ ... --]]`) containing specific `@` tags.
</MANDATORY_RULE>

**Correct Parameter Documentation Example:**
```lua
--[[
  // @Param: MYAPL_ENABLE
  // @DisplayName: My Applet Enable
  // @Description: Enables or disables the main functionality of my applet.
  // @User: Standard
  // @Values: 0:Disabled,1:Enabled
--]]
local MYAPL_ENABLE = bind_add_param('ENABLE', 1, 0)

--[[
  // @Param: MYAPL_SPEED
  // @DisplayName: My Applet Speed
  // @Description: The target speed for the applet's maneuver.
  // @User: Standard
  // @Units: m/s
  // @Range: 1 50
--]]
local MYAPL_SPEED = bind_add_param('SPEED', 2, 10)
```

### 5.8. Surgical Modification

<MANDATORY\_RULE\>
When asked to modify an existing file, you must strictly limit your changes to the scope of the user's explicit request. Do not perform any unrelated "tidy-up", refactoring, or stylistic changes. The goal is to produce the smallest possible diff that correctly implements the user's request, respecting the original author's coding style and structure. This rule does not apply to the addition of descriptive comments or temporary debug messages that aid in the development and review process.
</MANDATORY\_RULE\>
**Prohibited Tangential Changes Include:**

* **Reformatting:** Do not change indentation, line breaks, or spacing in code that is unrelated to the direct change. 
* **Variable Renaming:** Do not rename variables or functions for clarity unless that is part of the specific request. 
* **Comment Removal/Alteration:** Do not remove existing comments, unless they are clearly obsolete or incorrect. Try to preserve them exactly as they are.
* **Unrelated Refactoring:** Do not restructure or "improve" the logic of functions or sections of code that are not the direct subject of the modification request.

**Permitted Additions (Encouraged during development):**

 * **Adding Comments:** It is highly encouraged to add comments that explain the purpose of new or modified code blocks, especially for complex logic like state machines.
 * **Adding Debug Messages:** When debugging, it is good practice to add gcs:send_text(MAV_SEVERITY.DEBUG, ...) messages to provide insight into the script's state. These should be preserved across edits unless the user explicitly asks for them to be removed. By adhering to this, you ensure that the user can easily review the changes and trust that no unintended side effects have been introduced.

* **Commenting Guidelines:**
  * **Explain "Why", Not "What":** Good comments explain the *purpose* or complex *behavior* of a code block. They should provide context that isn't obvious from reading the code itself.
  * **Prohibit Change-Log Comments:** You **must not** add comments that merely describe a change you made (e.g., \-- BUGFIX: Corrected loop index, \-- CHANGED: Renamed variable, \-- ADDED: New function). This information belongs in version control history, not in the code, and quickly becomes outdated.
  * **Preserve Existing Comments:** Do not remove or alter existing comments unless they are clearly obsolete or factually incorrect due to the requested changes.

### 5.9. Data Type Coercion

The millis() and micros() functions return special uint32_t and uint64_t userdata types, not standard Lua numbers. These types must be converted to a number before being used in any arithmetic operation or passed to a function expecting a number. The only acceptable methods are my_uint:tofloat() or my_uint:toint(). Using generic Lua tonumber() is incorrect and will fail.

## 6\. Operational Constraints and Safety

When generating Lua scripts, the following constraints must be strictly adhered to, to ensure the safety and stability of the drone.

* **Flight Mode Awareness:** Scripts must be aware of the vehicle's current flight mode. Actions should only be executed if the current flight mode is appropriate. For example, a script that controls the drone's position should only run in modes like GUIDED or AUTO.  
* **Arming Status:** Scripts must always check if the vehicle is armed before executing any commands that could result in motor activation or movement.  
* **Failsafe Integrity:** Scripts must not interfere with critical failsafe mechanisms, such as RC failsafe, battery failsafe, or geofence breaches.  
* **Resource Limits:** Scripts should be mindful of the flight controller's processing and memory limits. Avoid long-running, blocking operations and excessive memory allocation. Use short, efficient functions and schedule them to run periodically.  
* **Parameter Modification:** Scripts should exercise extreme caution when modifying parameters. Critical flight parameters should not be changed without a clear understanding of the consequences. If a script does modify parameters, it should restore them to their original values when the script is disabled or completes its task.  
* **User Control Priority:** The pilot's RC input should always have the highest priority. Scripts should be designed to relinquish control immediately if the pilot provides any input on the sticks or changes the flight mode.

## 7\. Best Practices for Prompting

When requesting a Lua script, provide clear and concise instructions. Referencing the available scripts in the repository can help to improve the clarity and specificity of your requests.

**Prompting Template:**

"Create a Lua script that \[**action**\] when \[**trigger**\]. The script should \[**specific behavior**\]."

**Examples:**

* "Create a Lua script that **makes the drone fly in a 20-meter square pattern** when **the pilot flips RC channel 7 high**. The script should **return the drone to its original position after completing the pattern**."  
* "Create a Lua script that **logs the drone's altitude and battery voltage to a file every 5 seconds** when **the drone is armed**. The script should **stop logging when the drone is disarmed**."  
* "Create a Lua script that **activates a servo connected to output 10** when **the drone's altitude is above 50 meters**. The script should **deactivate the servo when the altitude is below 50 meters**."

## 8\. Example Use Cases

Here are some common drone behaviors with their corresponding text prompts and expected Lua script outputs.

### 8.1. Simple Box Mission

* **Prompt:** "Create a Lua script that makes the drone fly in a 15-meter box pattern when the pilot flips RC channel 8 to high."  
* **Expected Lua Script:**
```lua
  -- This script demonstrates a simple state machine to fly a box pattern.
  
  local BOX_SIZE = 15 -- meters
  local state = {
      active = false,
      stage = 0,
      start_location = nil
  }
  
  -- Define waypoint offsets from the start location (North, East)
  local waypoints = {
      {BOX_SIZE, 0},
      {BOX_SIZE, BOX_SIZE},
      {0, BOX_SIZE},
      {0, 0}
  }
  
  function update()
      -- Use rc:get_pwm() which is the correct function
      if rc:get_pwm(8) > 1800 then
          if not state.active then
              -- Start the pattern
              state.active = true
              state.stage = 1
              state.start_location = ahrs:get_location()
              if state.start_location then
                  local target = state.start_location:copy()
                  target:offset(waypoints[state.stage][1], waypoints[state.stage][2])
                  vehicle:set_target_location(target)
              end
          end
      else
          -- Reset the pattern if the switch is low
          state.active = false
          state.stage = 0
          state.start_location = nil
      end
  
      if state.active and state.start_location then
          local current_pos = ahrs:get_location()
          if current_pos then
              local target = state.start_location:copy()
              target:offset(waypoints[state.stage][1], waypoints[state.stage][2])
              if current_pos:get_distance(target) < 2 then -- 2 meter arrival radius
                  state.stage = state.stage + 1
                  if state.stage > #waypoints then
                      -- Pattern complete, reset
                      state.active = false
                      state.stage = 0
                  else
                      local next_target = state.start_location:copy()
                      next_target:offset(waypoints[state.stage][1], waypoints[state.stage][2])
                      vehicle:set_target_location(next_target)
                  end
              end
          end
      end
      return update, 200
  end
  
  return update()
```

### 8.2. Battery Failsafe

* **Prompt:** "Create a Lua script that triggers a Return-to-Launch (RTL) when the battery voltage drops below 14.8 volts."  
* **Expected Lua Script:**  
```lua
  local LOW_VOLTAGE = 14.8
  local RTL_MODE = 6 -- Mode number for RTL in Copter
  
  function update()
      -- Use instance 1 (1-indexed) and check if it's healthy
      if battery:healthy(1) then
          if battery:voltage(1) < LOW_VOLTAGE then
              -- Use the correct integer mode number for set_mode()
              if vehicle:get_mode() ~= RTL_MODE then
                  vehicle:set_mode(RTL_MODE)
                  gcs:send_text(6, "Battery low, initiating RTL")
              end
          end
      end
      return update, 1000
  end
  
  return update()
```

### 8.3. Landing Gear Control

* **Prompt:** "Create a Lua script that deploys the landing gear (servo on output 9\) when the drone's altitude is below 10 meters and retracts it above 10 meters."  
* **Expected Lua Script:**
```lua
  local LANDING_GEAR_ALTITUDE_M = 10 -- meters
  local SERVO_OUTPUT_CHAN = 8 -- 0-indexed channel for SERVO9
  local DEPLOYED_PWM = 1900
  local RETRACTED_PWM = 1100
  
  local gear_deployed = false
  
  function update()
      local loc = ahrs:get_location()
      if loc and loc:relative_alt() then
          -- get_location():alt() is in CM and is AMSL.
          -- We must get relative altitude and convert it to meters.
          local current_alt_m = loc:alt() / 100.0
  
          if current_alt_m < LANDING_GEAR_ALTITUDE_M then
              if not gear_deployed then
                  -- Use SRV_Channels object and set_output_pwm_chan (0-indexed)
                  SRV_Channels:set_output_pwm_chan(SERVO_OUTPUT_CHAN, DEPLOYED_PWM)
                  gear_deployed = true
              end
          else
              if gear_deployed then
                  SRV_Channels:set_output_pwm_chan(SERVO_OUTPUT_CHAN, RETRACTED_PWM)
                  gear_deployed = false
              end
          end
      end
      return update, 500
  end
  
  return update()
```

## 9\. Working with Aider and a Local ArduPilot Codebase

This section provides a guide for using a tool like aider to directly modify a local clone of the ArduPilot git repository. This workflow is ideal for making changes to existing scripts or autotests.

### 9.1. File Locations

To work with existing scripts, you need to know where they are located within the ArduPilot source tree. When creating new scripts, they should be placed in the appropriate subdirectory based on their function.

* **Lua Scripts**: The source for all Lua scripts is located in the libraries/AP\_Scripting/ directory. They are organized into subdirectories based on their type:  
  * applets/  
  * drivers/  
  * examples/  
* **Autotests**: The Python-based SITL autotests for Lua scripts are located in Tools/autotest/. The tests are vehicle-specific and should be added to the appropriate file (e.g., arducopter.py).

### 9.2. Aider Workflow

The workflow is a two-step conversation between the primary LLM and the user operating aider. The primary LLM generates the code and instructions, and the user applies them via aider.

**Example Workflow:**

Let's say you want to modify the copter\_terrain\_brake.lua applet and add a corresponding autotest.

1. **LLM Requests Files**: The primary LLM first determines wqhich files need to be modified and asks the user to add them to the aider chat.  
   **LLM Output (Step 1):**"I have the requested changes. Please add the following files to the aider chat so I can provide the edits: libraries/AP\_Scripting/applets/copter\_terrain\_brake.lua and Tools/autotest/arducopter.py."  
2. **User Adds Files**: The user adds the requested files to their local aider instance.  
   /add libraries/AP\_Scripting/applets/copter\_terrain\_brake.lua Tools/autotest/arducopter.py

3. **LLM Provides Edits**: Once the files are in the chat context, the primary LLM provides the edits in the required diff format.  
   **LLM Output (Step 2):**"Thank you. Please apply the following changes."  
   (The diff blocks containing the code changes would follow here.)  
4. **User Approves Edits**: The user then approves the changes within aider to apply them to the local files.

By following this process, the primary LLM can handle the creative/logic-based tasks, while aider acts as a tool to execute the changes.

### 9.3. Aider Output Format

<MANDATORY\_RULE\>
When generating changes for existing files, the output must be a set of edits in a diff-style format, using Unix-style line endings (\\n). Do not provide the entire file content unless the file is new. The edits should be clear and easy to apply.  
</MANDATORY\_RULE\>

### 9.4. Commit Message Conventions

<MANDATORY\_RULE\>
When committing changes to the ArduPilot repository, all commits must follow the standard ArduPilot conventions.  
</MANDATORY\_RULE\>

* **Atomic Commits**: Each commit should represent a single, logical change. For example, a change to a Lua applet and the addition of its corresponding autotest should be in two separate commits. Do not bundle unrelated changes into a single commit.  
* **Commit Message Prefix**: The subject line of every commit message **must** be prefixed with the name of the top-level module being changed, followed by a colon. The module is typically the subdirectory within the libraries/ or Tools/ directory where the file is located.  
  * **Example for a Lua script change:**  
    AP\_Scripting: Add new terrain brake applet

  * **Example for an autotest change:**  
    Tools: Add autotest for the terrain brake applet  


## 10. Final Deliverable Checklist

Before concluding a script generation task, the following checklist **must** be completed. All three components are required for a complete Applet deliverable.

1.  **[ ] Lua Script File (`.lua`):**
    * Does the script include a descriptive header comment?
    * Does it include all mandatory precondition `assert()` checks (Rule 5.3)?
    * Does it provide GCS feedback for significant state changes?
    * Is it free of `luacheck` errors and warnings?

2.  **[ ] Markdown Documentation File (`.md`):**
    * Has a complete `.md` file been generated?
    * Does it clearly explain the script's purpose?
    * Does it document all script-specific parameters (`*_ENABLE`, `*_RADIUS`, etc.)?
    * Does it provide clear setup instructions, including the required `RCx_OPTION` for the activation switch?

3.  **[ ] SITL Autotest Offer:**
    * Have you explicitly offered to generate a SITL autotest to verify the script's functionality?
    * Are you prepared to add the test as a new method to the appropriate vehicle test suite (e.g., `arducopter.py`) as per the playbook rules?
