-- Port of Sparkfun Qwiic Scale library for NAU7802
-- https://github.com/sparkfun/SparkFun_NAU7802_Scale_Arduino_Library

-- Set pointer to load cell address
local i2c_thrust = i2c:get_device(1,0x2A)
local i2c_torque = i2c:get_device(0,0x2A)
i2c_thrust:set_retries(10)
i2c_torque:set_retries(10)

-- Time (ms) between samples
local _samp_dt_ms = 0

-- Motor function to override
_motor_channel = SRV_Channels:find_channel(33)
local MOTOR_TIMEOUT = 3000 --(s)

-- Variables needed for setup and init of NAU7802
local _nau7802_setup_started = false
local _nau7802_i2c_dev
local _nau7802_sensor_index
local FAST_SAMPLE = 3
local SLOW_SAMPLE = 0

-- Script vars used in calibration
local THRUST = 1 -- Index for calibration values relating to thrust sensor
local TORQUE = 2 -- Index for calibration values relating to torque sensor
local _dev_initialised = {false, false}
local _dev_name = {"Thrust", "Torque"}
local _calibration_factor = {1,1}
local _zero_offset = {0,0}
local _calibration_ref = {2074,5.47} -- (grams, kg.cm) The known mass/torque value that the sensor will be calibrated with

-- Script vars used in average calculation
local _ave_total = 0 -- Average value obtained
local _ave_n_samp_aquired = 0 -- Number of samples recorded in this average calculation
local _ave_last_samp_ms = 0 -- Time that the last sample was retrieved
local _ave_callback = 0 -- Number of times the average dunction is called back
local _ave_n_samples = 10 -- The number of samples that are requested in the average

-- Average thrust
local _ave_thrust = 0
local _flag_have_thrust = false

-- Buttons
local ARMING_BUTTON = 1      -- Button number to start arm and sweep motor
local CAL_BUTTON = 2         -- Button number for initiating calibration
local DEAD_MAN = 3           -- Button number for dead mans switch
local AUX1_THROTTLE_MODE = 4 -- Button number for Aux 1
-- AUX2 = 5                  -- Button number for Aux 2, Additional btn instance needs adding to AP before this can be used

-- Save to file details
local _file_name_fmt = "thrust_test_%i.csv"
local _file_index_found = false
local _file_index = 0
local format_string = "%s, %.4f, %04i, %.0f, %.4f, %.4f, %.5f, %.5f, %.2f, %.2f, %05i, %03i\n" -- Time (ms), Throttle (), RC Out (us), Motor Commutations (1/min), Voltage (V), Current (A), Thrust (g), Torque (g.cm), ESC Volt (V), ESC Current (A), ESC RPM (rpm), ESC Temperature (deg C)

-- Update throttle
local _flag_hold_throttle = false
local _ramp_rate = 0.03 -- (%/s) How quickly throttle is advanced
local _hold_time = 3 --(s) How long the thottle is held at each discreate step
local _n_throttle_steps = 10 -- Number of discrete steps that the throttle is held at
local _last_thr_update = 0 -- (ms) The last time the throttle was updated
local _current_thr = 0 --(%)
local _hold_thr_last_time = 0
local _next_thr_step = 0 --(%)
local _max_throttle = 1 --(%)
local _thr_inc_dec = 1 --(-) Used to switch the motor from increment to decrement
-- Transient step response modes
local _hover_point_achieved = false -- Has the controller achieved the hover throttle
local _throttle_step_index = 1 -- Index in the table of throttle steps to work through

-- Sys state - The current state of the system
local REQ_CAL_THRUST_ZERO_OFFSET = 1    -- Requires calibration, needs zero offset
local REQ_CAL_THRUST_FACTOR = 2         -- In calibration, needs calibration factor
local REQ_CAL_TORQUE_ZERO_OFFSET = 3    -- Requires calibration, needs zero offset
local REQ_CAL_TORQUE_FACTOR = 4         -- In calibration, needs calibration factor
local DISARMED = 5                      -- Ready to go, not running
local ARMED = 6                         -- Armed and motor running
local CURRENT_PROTECTION = 10           -- Error State

local _sys_state = REQ_CAL_THRUST_ZERO_OFFSET
local _last_sys_state = -1
local _state_str = "Calibration"

-- Used to clear the screen when the system state changes
local _last_state_display = -1
local _last_display_update_ms = 0
local _display_refresh_ms = 500

-- Throttle mode
local THROTTLE_MODE_RAMP = 0 -- Throttle gradual ramp up and down
local THROTTLE_MODE_STEP = 1 -- Step inputs around hover throttle
local _throttle_mode = THROTTLE_MODE_RAMP
local _last_aux1_state

-- LEDs
local _num_leds = 10
local _led_chan = 0
local _last_led_update = 0

-- Lua param allocation
local ZERO_OFFSET_PARAM = {"SCR_USER1","SCR_USER3"}
local CAL_FACT_PARAM = {"SCR_USER2", "SCR_USER4"}
local CURRENT_LIMIT = "SCR_USER5"
local MAX_THR_PARAM = "SCR_USER6"

-- Measurements
local _thrust = 0.0
local _torque = 0.0
local _voltage = 0.0
local _current = 0.0

-- Forward declare functions
local calculate_zero_offset
local calculate_calibration_factor
local init_nau7802

------------------------------------------------------------------------
-- Mask & set a given bit within a register
local function setBit(i2c_dev, bitNumber, registerAddress)
  local value = i2c_dev:read_registers(registerAddress)

  if value == nil then
    return false
  end

  value = value | (1 << bitNumber) -- Set this bit
  return i2c_dev:write_register(registerAddress, value)
end
------------------------------------------------------------------------


------------------------------------------------------------------------
-- Mask & clear a given bit within a register
local function clearBit(i2c_dev, bitNumber, registerAddress)
  local value = i2c_dev:read_registers(registerAddress)

  if value then
    value = value & (255 - (1 << bitNumber)) -- Set this bit
    return i2c_dev:write_register(registerAddress, value)
  end

  return false
end
------------------------------------------------------------------------


------------------------------------------------------------------------
-- Return a given bit within a register
local function getBit(i2c_dev, bitNumber, registerAddress)
  local value = i2c_dev:read_registers(registerAddress)

  if value == nil then
    return 0
  end

  -- Clear all but this bit
  value = value & (1 << bitNumber)
  return value > 0
end
------------------------------------------------------------------------


------------------------------------------------------------------------
-- Returns true if Cycle Ready bit is set (conversion is complete)
local function available(i2c_dev)
  return getBit(i2c_dev, 5, 0)
end
------------------------------------------------------------------------


------------------------------------------------------------------------
-- Check calibration status.
local function calAFEStatus(i2c_dev)
  if getBit(i2c_dev, 2, 2) then
    -- Calibration in progress
    return 1
  end

  if getBit(i2c_dev, 3, 2) then
    -- Calibration failure
    return 2
  end

  -- Calibration passed
  return 0
end
------------------------------------------------------------------------


------------------------------------------------------------------------
-- Function to return the samples per second output by the qwiic scale
local function get_sps(rate)
  -- Error value
  local ret_val = -1
  if rate == 0 then ret_val = 10 end
  if rate == 1 then ret_val = 20 end
  if rate == 2 then ret_val = 40 end
  if rate == 3 then ret_val = 80 end
  if rate == 7 then ret_val = 320 end

  return ret_val
end
------------------------------------------------------------------------


------------------------------------------------------------------------
-- Set the readings per second
-- 10, 20, 40, and 80 samples per second is available
local function setSampleRate(i2c_dev, rate)
  if rate > 7 then
    rate = 7
  end

  local value = i2c_dev:read_registers(2)

  if value == nil then
    return false
  end

  value = value & 143 -- Clear CRS bits
  value = value | (rate << 4)  -- Mask in new CRS bits

  -- set sample delta time in lua
  _samp_dt_ms = math.ceil(1000/get_sps(rate))

  return i2c_dev:write_register(2, value)
end
------------------------------------------------------------------------


------------------------------------------------------------------------
-- Power up digital and analog sections of scale
local function powerUp(i2c_dev)
  setBit(i2c_dev, 1, 0)
  setBit(i2c_dev, 2, 0)

  -- Wait for Power Up bit to be set - takes approximately 200us
  local counter = 0
  while (true) do
    if getBit(i2c_dev, 3, 0) then
      break -- Good to go
    end
    if counter > 100 then
      return false -- Error
    end
    counter = counter + 1
  end
  return true
end
------------------------------------------------------------------------


------------------------------------------------------------------------
-- Resets all registers to Power Off Defaults
local function reset(i2c_dev)
  -- Set RR
  setBit(i2c_dev, 0, 0)
  -- Clear RR to leave reset state
  return clearBit(i2c_dev, 0, 0)
end
------------------------------------------------------------------------


------------------------------------------------------------------------
-- Set the onboard Low-Drop-Out (LDO) voltage regulator to a given value
-- 2.4, 2.7, 3.0, 3.3, 3.6, 3.9, 4.2, 4.5V are available
local function setLDO(i2c_dev, ldoValue)
  -- Error check
  if ldoValue > 7 then
    ldoValue = 7
  end

  -- Set the value of the LDO
  local value = i2c_dev:read_registers(1)

  if value == nil then
    return false
  end

  -- Clear LDO bits
  value = value & 199
  -- Mask in new LDO bits
  value = value | (ldoValue << 3)
  i2c_dev:write_register(1, value)

  -- Enable the internal LDO
  return setBit(i2c_dev, 7, 0)
end
------------------------------------------------------------------------


------------------------------------------------------------------------
-- Set the gain
-- x1, 2, 4, 8, 16, 32, 64, 128 are avaialable
local function setGain(i2c_dev, gainValue)
  if gainValue > 7 then
    gainValue = 7 -- Error check
  end

  local value = i2c_dev:read_registers(1)

  if value == nil then
    return false
  end

  value = value & 248 -- Clear gain bits
  value = value | gainValue -- Mask in new bits

  return i2c_dev:write_register(1, value)
end
------------------------------------------------------------------------


------------------------------------------------------------------------
-- Returns 24-bit reading
-- Assumes CR Cycle Ready bit (ADC conversion complete) has been checked to be 1
local function getReading(i2c_dev)

  local MSB = i2c_dev:read_registers(18) --ADC_OUT[23:16]
  local MID = i2c_dev:read_registers(19) --ADC_OUT[15:8]
  local LSB = i2c_dev:read_registers(20) --ADC_OUT[7:0]

  if MSB and MID and LSB then

    local value = (MSB << 16) | (MID << 8) | LSB
    local sign = (value & (1 << 23)) > 0

    if sign then
        value = 16777215 ~ value -- 24 bit XOR to flip the bits for two's compliment
        value = (value * -1) - 1
    end

    return value
  else
    return false
  end

end
------------------------------------------------------------------------


------------------------------------------------------------------------
local function reset_ave_var()
    -- we don't reset _ave_total here because we need to retrieve it after the function calls are complete
    _ave_n_samp_aquired = 0
    _ave_last_samp_ms = 0
    _ave_callback = 0
end
------------------------------------------------------------------------


------------------------------------------------------------------------
-- Function is called recursivley based on sample rate set in script.  Will do a max of 50 callbacks
-- before force stoping, computing the average obtained at that time.  Two averages cannot be calculated
-- simultanuously.  Returns true on average calculation complete.
function calc_average(i2c_dev)

  -- if this is the first call back then reset the average value
  if _ave_callback < 1 then
    _ave_total = 0
  end

  local now = millis()
  _ave_callback = _ave_callback + 1

  if ((now - _ave_last_samp_ms) > _samp_dt_ms  or  _ave_last_samp_ms == 0) and available(i2c_dev) then
    -- Add new reading to average
    _ave_total = _ave_total + getReading(i2c_dev)
    _ave_n_samp_aquired = _ave_n_samp_aquired + 1

    -- Record time of last measurement
    _ave_last_samp_ms = now
  end

  if (_ave_n_samp_aquired == _ave_n_samples) or (_ave_callback > 50) then
    -- Compute average
    if _ave_n_samp_aquired > 0  then
        _ave_total = _ave_total/_ave_n_samp_aquired
    else
        -- Return error value
        _ave_total = -1
    end

    -- Reset average script variables ready for next inital call to get_average
    reset_ave_var()

    -- Calculation complete
    return true
  end

  -- If we have got this far we haven't finished computing the average
  return false

end
------------------------------------------------------------------------


------------------------------------------------------------------------
-- function to set device for functions that are called back
function set_device(i2c_dev, index)
  _nau7802_i2c_dev = i2c_dev
  _nau7802_index = index -- thrust or torque index
end
------------------------------------------------------------------------


------------------------------------------------------------------------
-- Call when load cell is setup, at running temperature, with no load applied
-- set_device(i2c_dev, index) must be called before this function
calculate_zero_offset = function()

  if calc_average(_nau7802_i2c_dev) then
    _zero_offset[_nau7802_index] = _ave_total
    gcs:send_text(4, _dev_name[_nau7802_index] .. " 0 offset: " .. tostring(_zero_offset[_nau7802_index]))

    -- Save offset to EEPROM with param
    if not(param:set_and_save(ZERO_OFFSET_PARAM[_nau7802_index], _zero_offset[_nau7802_index])) then
        gcs:send_text(4, _dev_name[_nau7802_index] .. " 0 offset param set fail")
    end

    -- Advance state to next step in calibration process
    _sys_state = _sys_state + 1
    return protected_update, _samp_dt_ms
  end

  -- carry on doing average calculation
  return calculate_zero_offset, _samp_dt_ms

end
------------------------------------------------------------------------


------------------------------------------------------------------------
-- Call after zeroing. Provide the float weight sitting on scale. Units do not matter.
-- set_device(i2c_dev, index) must be called before this function
calculate_calibration_factor = function()

  if calc_average(_nau7802_i2c_dev) then
    -- we have got average, calc claibration gradient
    _calibration_factor[_nau7802_index] = (_ave_total - _zero_offset[_nau7802_index]) / _calibration_ref[_nau7802_index]
    gcs:send_text(4, _dev_name[_nau7802_index] .. " calibration factor: " .. tostring(_calibration_factor[_nau7802_index]))

    -- Protect against divide by zero
    if _calibration_factor[_nau7802_index] <= 0.000001 and _calibration_factor[_nau7802_index] > -0.000001 then
      if _calibration_factor[_nau7802_index] < 0 then
        _calibration_factor[_nau7802_index] = -0.000001
      else
        _calibration_factor[_nau7802_index] = 0.000001
      end
    end

    -- Save calibration factor to EEPROM with param
    if not(param:set_and_save(CAL_FACT_PARAM[_nau7802_index], _calibration_factor[_nau7802_index])) then
      gcs:send_text(1, _dev_name[_nau7802_index] .. " cal factor param set fail")
    end

    -- Advance system state and exit calibration
    _sys_state = _sys_state + 1
    return protected_update, _samp_dt_ms

  end

  -- Carry on doing average calculation
  return calculate_calibration_factor, _samp_dt_ms

end
------------------------------------------------------------------------


------------------------------------------------------------------------
-- Returns the instentanious load on a given load cell
local function get_load(i2c_dev, index)

  if not available(i2c_dev) then
    return 0
  end

  local on_scale = getReading(i2c_dev)

  -- Calc and return load
  return (on_scale - _zero_offset[index]) / _calibration_factor[index]

end
------------------------------------------------------------------------


------------------------------------------------------------------------
local function begin(i2c_dev)

  local result = true

  result = result and reset(i2c_dev)                      -- Reset all registers
  result = result and powerUp(i2c_dev)                    -- Power on analog and digital sections of the scale
  result = result and setLDO(i2c_dev, 4)                  -- Set LDO to 3.3V
  result = result and setGain(i2c_dev, 7)                 -- Set gain to 128
  result = result and setSampleRate(i2c_dev, SLOW_SAMPLE) -- Set samples per second to 40.  Sample rate must be the same for both devices
  result = result and i2c_dev:write_register(21, 48)      -- Turn off CLK_CHP. From 9.1 power on sequencing.
  result = result and setBit(i2c_dev, 7, 28)              -- Enable 330pF decoupling cap on chan 2. From 9.14 application circuit note.
  result = result and setBit(i2c_dev, 2, 2)               -- Begin asynchronous calibration of the analog front end.
                                                          -- Poll for completion with calAFEStatus() or wait with waitForCalibrateAFE()
  return result

end
------------------------------------------------------------------------

----------------------------------------------------------------------------------------------------------
--                                  Thrust test stand code                                              --
----------------------------------------------------------------------------------------------------------

------------------------------------------------------------------------
-- Constrain a value between limits
function constrain(v, vmin, vmax)
  v = math.max(v,vmin)
  v = math.min(v,vmax)
  return v
end
------------------------------------------------------------------------


------------------------------------------------------------------------
-- set_device(i2c_dev, index) must be called before this function
init_nau7802 = function()

  if not _nau7802_setup_started then
    if begin(_nau7802_i2c_dev) then
      gcs:send_text(0,"Setup done: " .. _dev_name[_nau7802_index])
      _nau7802_setup_started = true
    else
      gcs:send_text(0,"Setup failed: " .. _dev_name[_nau7802_index])
      return
    end

  else
    local cal_status = calAFEStatus(_nau7802_i2c_dev)
    if cal_status == 0 then
      gcs:send_text(0,"Inialization done: " .. _dev_name[_nau7802_index])
      _dev_initialised[_nau7802_index] = true

      -- Reset _nau7802 calibration variables
      _nau7802_setup_started = false

      -- Device initalised, return to main code
      return init, 100
    end

    if cal_status == 1 then
      -- Cal in progress, wait
      gcs:send_text(4, "DB: cal in progress")
      return init_nau7802, 100
    end

    -- If we got this far then calibration failed
    gcs:send_text(0,"Initialisation failed: " .. _dev_name[_nau7802_index])
    gcs:send_text(0,"NAU7802 calibration status: " .. tostring(cal_status))
    return
  end

  return init_nau7802, 100
end
------------------------------------------------------------------------


------------------------------------------------------------------------
function init()

  -- Init load cells
  if not(_dev_initialised[THRUST]) then
      set_device(i2c_thrust, THRUST)
      return init_nau7802, 100
  end

  if not(_dev_initialised[TORQUE]) then
      set_device(i2c_torque, TORQUE)
      return init_nau7802, 100
  end

  -- Setup file to record data to
  -- Scan through directory to find a unique file name.  Break up search over
  -- with callbacks every 20 file names
  if not(_file_index_found) then
    for i = 1,20 do
      if not (is_file(string.format(_file_name_fmt, _file_index+i))) then
        -- Set file index to
        _file_index = _file_index + i
        _file_index_found = true
        break
      end

      -- Reschedule init to continue searching for a file name
      if i >= 20 then
        _file_index = _file_index+20
        return init, 100
      end
    end
  end

  -- local file = assert(io.open(file_name, "w"),"Could not make file: " .. file_name)
  local file = assert(io.open(string.format(_file_name_fmt, _file_index), "w"), "Could not make file" .. string.format(_file_name_fmt, _file_index))
  local header = 'Time (ms), Throttle (), RC Out (us), Motor Commutations (1/min), Voltage(V), Current (A), Thrust (g), Torque (kg.cm), ESC Volt (V), ESC Current (A), ESC RPM (rpm), ESC Temperature (deg C)\n'
  file:write(header)
  file:close()

  -- Set first throttle step
  set_next_thr_step()

  _calibration_loaded_flag = load_calibration(THRUST)
  if _calibration_loaded_flag then
    gcs:send_text(4,"Thrust calibration values loaded ")
    -- Advance system state, thrust calibration not needed
    _sys_state = REQ_CAL_TORQUE_ZERO_OFFSET
  end

  _calibration_loaded_flag = _calibration_loaded_flag and load_calibration(TORQUE)
  if _calibration_loaded_flag then
    gcs:send_text(4,"Torque calibration values loaded ")
    -- Advance system state, no calibration needed
    _sys_state = DISARMED
  end

  -- Init neopixles
  _led_chan = SRV_Channels:find_channel(98)
  if not _led_chan then
    gcs:send_text(6, "LEDs: channel not set")
    return
  end

  -- Find_channel returns 0 to 15, convert to 1 to 16
  _led_chan = _led_chan + 1

  -- Added an extra LED to account for logic level shifter
  if not serialLED:set_num_neopixel(_led_chan,  _num_leds+1) then
    gcs:send_text(6, "LEDs: neopixle not set")
  end

  --- --- --- Set prerequisite parameters --- --- ---
  -- Setup motor 1 so that we can apply servo overrides.  That way, if the lua script dies the 
  -- override will time out and the motor will be stopped
  local all_set = param:set_and_save('SERVO1_FUNCTION', 33)
  all_set = all_set and param:set_and_save('MOT_SPIN_MIN', 0)
  all_set = all_set and param:set_and_save('MOT_PWM_MIN', 1000)

  -- Setup arming button function to arm and disarm copter aswell. This makes use of logging 
  -- and standard motor protections whilst armed.
  all_set = all_set and param:set_and_save('BTN_FUNC1', 41)

  if all_set then
    -- Now main loop can be started
    return protected_update, 100
  else
    gcs:send_text(6, "Param set fail in init")
    return
  end

end
------------------------------------------------------------------------


------------------------------------------------------------------------
function update()

  -- Bodgy conversion from userdata to number
  local now = tonumber(tostring(millis()))

  -- Get state of inputs
  local cal_button_state = button:get_button_state(CAL_BUTTON)
  local arm_button_state = button:get_button_state(ARMING_BUTTON)
  local run_button_state = button:get_button_state(DEAD_MAN)
  local aux1_state = button:get_button_state(AUX1_THROTTLE_MODE)


  -- must be called before update_lights()
  update_throttle_max()
  update_state_msg()
  update_lights(now)

  -- Update display cannot be called when fast sampling is being used
  -- for logging transient throttle behaviour. It slows the script run time too much
  if not (_sys_state == ARMED and _throttle_mode == THROTTLE_MODE_STEP) then
      update_display(now)
  end

  --- --- --- state machine --- --- ---
  -- Check if we should reset calibration
  if _sys_state == DISARMED and cal_button_state then
    _sys_state = REQ_CAL_THRUST_ZERO_OFFSET

    -- Reset calibration in memory
    if not(param:set_and_save(ZERO_OFFSET_PARAM[THRUST], 0)) then
      gcs:send_text(4,"Thrust 0 offset param set fail")
    end
    if not(param:set_and_save(CAL_FACT_PARAM[THRUST], 0)) then
      gcs:send_text(4,"Thrust cal factor param set fail")
    end
    if not(param:set_and_save(ZERO_OFFSET_PARAM[TORQUE], 0)) then
      gcs:send_text(4,"Torque 0 offset param set fail")
    end
    if not(param:set_and_save(CAL_FACT_PARAM[TORQUE], 0)) then
      gcs:send_text(4,"Torque cal factor param set fail")
    end

    return protected_update, 500
  end

  -- Check if we should arm the system
  if arm_button_state and _sys_state == DISARMED and arming:is_armed() then
    _sys_state = ARMED
  end

  -- Change to local lua diarm state
  if not arm_button_state and not(_sys_state < DISARMED) then
    -- Arm button has been switched off and we are not in a calibration state
    _sys_state = DISARMED
  end

  -- Disarm copter if we are not in 'armed' in lua.
  -- This handles the copter disarm when we enter an error state e.g. over current protection
  if _sys_state ~= ARMED and _last_sys_state >= ARMED then
    local has_disarmed = false
    while (has_disarmed == false) do
      has_disarmed = arming:disarm()
    end
  end

  -------------------------------------------
  --- --- --- Calibration routine --- --- ---
  -------------------------------------------
  if _sys_state == REQ_CAL_THRUST_ZERO_OFFSET and cal_button_state then
    set_device(i2c_thrust, THRUST)
    return calculate_zero_offset, 500
  end

  if _sys_state == REQ_CAL_THRUST_FACTOR and cal_button_state then
    set_device(i2c_thrust, THRUST)
    return calculate_calibration_factor, 500
  end

  if _sys_state == REQ_CAL_TORQUE_ZERO_OFFSET and cal_button_state then
    set_device(i2c_torque, TORQUE)
    return calculate_zero_offset, 500
  end

  if _sys_state == REQ_CAL_TORQUE_FACTOR and cal_button_state then
    set_device(i2c_torque, TORQUE)
    return calculate_calibration_factor, 500
  end

  -- Check if we need to reset after an over-current protection event
  if _sys_state == CURRENT_PROTECTION and not arm_button_state then
    _sys_state = DISARMED
  end

  -- Update throttle mode.
  -- Only allow throttle mode to be changed in Disarmed State
  if _sys_state == DISARMED and _last_aux1_state ~= aux1_state then
    if aux1_state then
      _throttle_mode = THROTTLE_MODE_STEP
      update_sample_rate(FAST_SAMPLE)
    else
      _throttle_mode = THROTTLE_MODE_RAMP
      update_sample_rate(SLOW_SAMPLE)
    end
  end


  -- If system is armed update the throttle
  if _sys_state == ARMED and run_button_state then
    -- Current protection
    local current_limit_amps = param:get(CURRENT_LIMIT)
    if (current_limit_amps <= 0) or (_current < current_limit_amps) then
      -- Update the output throttle
      if _throttle_mode == THROTTLE_MODE_STEP then
        update_throttle_transient(now)
      else
        update_throttle_ramp(now)
      end
    else
      -- Activate current protection
      zero_throttle()
      _sys_state = CURRENT_PROTECTION
    end

  else
    -- Do not run motor without valid calibration, when disarmed, or when in an error state
    zero_throttle()
  end

  -----------------------------------------
  --- --- --- Take measurements --- --- ---
  -----------------------------------------
  if battery:healthy(0) then
    _voltage = battery:voltage(0)
    _current = battery:current_amps(0)
  end

  _thrust = get_load(i2c_thrust, THRUST)
  _torque = get_load(i2c_torque, TORQUE)

  -- Update rpm
  local rpm = RPM:get_rpm(0)
  if (rpm == nil) then
      rpm = -1
  end

  -- Update ESC telemetry
  local esc_voltage = blheli:get_voltage(0)
  local esc_current = blheli:get_current(0)
  local esc_rpm = blheli:get_rpm(0)
  local esc_temp = blheli:get_temp(0)

  -- Log values
  if _sys_state == ARMED and run_button_state then
      -- Only log whilst armed and running
      local file = assert(io.open(string.format(_file_name_fmt, _file_index), "a"), "Could not open file" .. string.format(_file_name_fmt, _file_index))
      file:write(string.format(format_string, tostring(now), _current_thr, calc_pwm(_current_thr), rpm, _voltage, _current, _thrust, _torque, esc_voltage, esc_current, esc_rpm, esc_temp))
      file:close()
  end

  -- Normal re-schedules of update are handled in protected_update()

end
------------------------------------------------------------------------


------------------------------------------------------------------------
function update_state_msg()

  if _sys_state == _last_sys_state then
      return
  end

  if _sys_state == REQ_CAL_THRUST_ZERO_OFFSET then
    gcs:send_text(4,"In calibration")
    gcs:send_text(4,"Unload thrust load cell and press cal button")
    _state_str = "Calibration"
  end

  if _sys_state == REQ_CAL_THRUST_FACTOR then
    gcs:send_text(4,"In calibration")
    gcs:send_text(4,"Apply calibration mass to thrust load")
    gcs:send_text(4,"cell and press calibration button")
    _state_str = "Calibration"
  end

  if _sys_state == REQ_CAL_TORQUE_ZERO_OFFSET then
    gcs:send_text(4,"In calibration")
    gcs:send_text(4,"Unload torque load cell and press cal button")
    _state_str = "Calibration"
  end

  if _sys_state == REQ_CAL_TORQUE_FACTOR then
    gcs:send_text(4,"In calibration")
    gcs:send_text(4,"Apply calibration mass to torque load")
    gcs:send_text(4,"cell and press calibration button")
    _state_str = "Calibration"
  end

  if _sys_state == DISARMED then
    gcs:send_text(4,"Disarmed: Ready for test")
    _state_str = "Disarmed"
  end

  if _sys_state == ARMED then
    gcs:send_text(4,"Armed")
    _state_str = "Armed"
  end

  if _sys_state == CURRENT_PROTECTION then
    gcs:send_text(4,"Over current protection activated ")
    _state_str = "Curr Protect"
  end

  _last_sys_state = _sys_state
end
------------------------------------------------------------------------


------------------------------------------------------------------------
function load_calibration(index)

  -- Load sensor calibration values
  local temp = param:get(ZERO_OFFSET_PARAM[index])
  if (temp == 0) then
    -- Don't have valid calibration stored
    return false
  end
  -- Set value from storage
  _zero_offset[index] = temp

  temp = param:get(CAL_FACT_PARAM[index])
  if (temp == 0) then
      -- Don't have valid calibration stored
      return false
  end

  -- Set value from storage
  _calibration_factor[index] = temp

  gcs:send_text(4,"DB: cal fact = " .. tostring(_calibration_factor[index]))

  return true
end
------------------------------------------------------------------------


------------------------------------------------------------------------
function update_throttle_ramp(time)

  -- Check whether to switch throttle hold off
  if (time - _hold_thr_last_time) > (_hold_time * 1000) and (_flag_hold_throttle == true) then
    _flag_hold_throttle = false
  end

  -- Check if we should advance throttle
  if not(_flag_hold_throttle) and (_last_thr_update > 0) then
    -- Calculate throttle if it is to be increased
    _current_thr = constrain((_current_thr + _ramp_rate * (time - _last_thr_update) * 0.001 * _thr_inc_dec),0,_max_throttle)
    _last_thr_update = time

    -- See if we are at a throttle hold point
    if ((_current_thr >= _next_thr_step) and (_thr_inc_dec > 0)) or ((_current_thr <= _next_thr_step) and (_thr_inc_dec < 0)) then
      _hold_thr_last_time = time
      _flag_hold_throttle = true

      -- Calculate new throttle step
      set_next_thr_step()
    end
  else
    _last_thr_update = time
  end

  SRV_Channels:set_output_pwm_chan_timeout(_motor_channel, calc_pwm(_current_thr), MOTOR_TIMEOUT)

end
------------------------------------------------------------------------


------------------------------------------------------------------------
function update_throttle_transient(time)
  local throttle_steps = {-0.05, 0, 0.05, -0.05, 0, 0.05, 0, -0.1, 0, 0.1}

  local hover_throttle = param:get('MOT_THST_HOVER')
  local step_hold_time = 1 -- (s)

  -- Check whether to switch throttle hold off
  if (time - _hold_thr_last_time) > (step_hold_time * 1000) and (_flag_hold_throttle == true) then
      _flag_hold_throttle = false
  end

  if not(_flag_hold_throttle) and (_last_thr_update > 0) then
    -- Gradually ramp throttle to hover throttle
    if not _hover_point_achieved then
      _current_thr = constrain((_current_thr + _ramp_rate * (time - _last_thr_update) * 0.001 * _thr_inc_dec),0,_max_throttle)
      _last_thr_update = time
      SRV_Channels:set_output_pwm_chan_timeout(_motor_channel, calc_pwm(_current_thr), MOTOR_TIMEOUT)
    else
      -- Step change throttle
      _current_thr = constrain((hover_throttle + throttle_steps[_throttle_step_index]),0,_max_throttle)

      -- Set throttle hold
      _hold_thr_last_time = time
      _flag_hold_throttle = true

      -- Output to motor
      SRV_Channels:set_output_pwm_chan_timeout(_motor_channel, calc_pwm(_current_thr), MOTOR_TIMEOUT)

      -- Increment index for next throttle step
      _throttle_step_index = _throttle_step_index + 1

      -- Check if we have reached the end of our planned steps
      if _throttle_step_index > #throttle_steps then
        -- Ramp the throttle back down
        _throttle_step_index = 1
        _thr_inc_dec = -1
        _hover_point_achieved = false
        -- Ensure current throttle is below hover throttle to ensure throttle rampsback down
        _current_thr = hover_throttle*0.95
      end
    end

    -- Check if initial ramp is complete
    if _current_thr >= hover_throttle then
      _hover_point_achieved = true
      _hold_thr_last_time = time
      _flag_hold_throttle = true
    end

  else
    _last_thr_update = time
  end
end
------------------------------------------------------------------------


------------------------------------------------------------------------
function zero_throttle()
  _current_thr = 0
  -- Ensure minimum of 2 throttle steps
  if _n_throttle_steps < 2 then 
    _n_throttle_steps = 2
  end
  -- Reset throttle step
  _next_thr_step = _max_throttle/_n_throttle_steps
  _flag_hold_throttle = false
  _last_thr_update = 0
  _thr_inc_dec = 1
  _hover_point_achieved = false
  _throttle_step_index = 1
  SRV_Channels:set_output_pwm_chan_timeout(_motor_channel, calc_pwm(_current_thr), MOTOR_TIMEOUT)
end
------------------------------------------------------------------------


------------------------------------------------------------------------
function update_throttle_max()
  if _sys_state == ARMED then
    -- Do not allow max throttle to be changed when armed
    return
  end

  _max_throttle = constrain(param:get(MAX_THR_PARAM)*0.01,0,1)
end
------------------------------------------------------------------------


------------------------------------------------------------------------
function calc_pwm(throttle_pct)
  -- Calc pwm from throttle as a % (0-1)
  local pwm = math.floor((throttle_pct * 1000) + 1000)
  return pwm
end
------------------------------------------------------------------------


------------------------------------------------------------------------
function set_next_thr_step()
  -- Ensure minimum of 2 throttle steps
  if _n_throttle_steps < 2 then 
    _n_throttle_steps = 2
  end

  -- check if we need to start stepping down through the throttle
  if _current_thr >= _max_throttle then
    _thr_inc_dec = -1
  end

  -- Update next throttle step
  _next_thr_step = _next_thr_step + (_max_throttle*_thr_inc_dec)/_n_throttle_steps
  _next_thr_step = constrain(_next_thr_step,0,_max_throttle)
end
------------------------------------------------------------------------


------------------------------------------------------------------------
function update_sample_rate(rate)
  -- Set sample rate appropriate to mode
  if not setSampleRate(i2c_thrust, rate) or not setSampleRate(i2c_torque, rate) then
    gcs:send_text(4,"Amp rate set fail")
  end
end
------------------------------------------------------------------------


------------------------------------------------------------------------
function update_lights(now_ms)

  if (now_ms - _last_led_update <= 250) then
    -- limit the led update rate
    return
  end

  -- Define colours for addressable led display
  local colour = {}
  colour['act'] = {0,0.3,0} -- colour assigned to throttle actual value
  colour['error'] = {1,0,0} -- Colour assignment when in over current protection

  if _throttle_mode == THROTTLE_MODE_STEP then
    colour['max'] = {0.15,0,0.2} -- colour assigned to throttle range
    colour['disarm'] = {0.2,0.4,0.1} -- colour assigned to throttle range when disarmed
  else
    colour['max'] = {0,0,0.3} -- colour assigned to throttle range
    colour['disarm'] = {0.2,0.2,0} -- colour assigned to throttle range when disarmed
  end

  -- Array to keep track of state of each led
  local led_state = {}
  for i = 1, _num_leds do
    led_state[i] = {} --create a new row
    led_state[i]['act'] = 0
    led_state[i]['max'] = 0
  end

  -- Calculate the throttle percentage that each led is worth
  if _num_leds <= 0 then
    _num_leds = 1
  end
  local throttle_to_led_pct = 1/_num_leds
  local r, g, b = 0, 0, 0
  local remain = 0

  -- Update state array for full throttle range
  remain = _max_throttle
  for i = 1, _num_leds do
    remain = remain - throttle_to_led_pct

    if (remain <= 0) and (remain >= -throttle_to_led_pct) then
      -- We have found the led that needs to be partially on
      led_state[i]['max'] = (throttle_to_led_pct+remain)/throttle_to_led_pct

    elseif (remain > 0) then
      -- This led is fully on
      led_state[i]['max'] = 1

    else
      -- Set led to off
      led_state[i]['max'] = 0
    end
  end


  -- Update state array for full throttle range
  remain = _current_thr
  for i = 1, _num_leds do
    remain = remain - throttle_to_led_pct

    if (remain <= 0) and (remain >= -throttle_to_led_pct) then
      -- We have found the led that needs to be partially on
      led_state[i]['act'] = (throttle_to_led_pct+remain)/throttle_to_led_pct

    elseif (remain > 0) then
      -- This led is fully on
      led_state[i]['act'] = 1

    else
      -- Set led to off
      led_state[i]['act'] = 0
    end
  end

  -- Reset the neopixles everytime - this is due to a bug in current master
  serialLED:set_RGB(_led_chan, -1, 0, 0, 0)

  if _sys_state <= DISARMED then
    -- Update led setting
    for i = 1, _num_leds do
      r = colour['disarm'][1]*(led_state[i]['max'])
      g = colour['disarm'][2]*(led_state[i]['max'])
      b = colour['disarm'][3]*(led_state[i]['max'])
      set_LED_colour(i, r, g, b)
    end

  elseif _sys_state == ARMED then
    -- update led setting
    for i = 1, _num_leds do
      r = colour['max'][1]*(led_state[i]['max']-led_state[i]['act']) + colour['act'][1]*led_state[i]['act']
      g = colour['max'][2]*(led_state[i]['max']-led_state[i]['act']) + colour['act'][2]*led_state[i]['act']
      b = colour['max'][3]*(led_state[i]['max']-led_state[i]['act']) + colour['act'][3]*led_state[i]['act']
      set_LED_colour(i, r, g, b)
    end

  else --CURRENT_PROTECTION
    for i = 1, _num_leds do
      r = colour['error'][1]
      g = colour['error'][2]
      b = colour['error'][3]
      set_LED_colour(i, r, g, b)
    end
  end

  serialLED:send(_led_chan)

  -- Update the calibration button led
  local CAL_LED_RED = 95     -- Servo function assigned to red led on calibration ring led
  local CAL_LED_GREEN = 96   -- Servo function assigned to green led on calibration ring led
  local CAL_LED_BLUE = 97    -- Servo function assigned to blue led on calibration ring led

  local OFF_PWM = 0
  local HALF_ON_PWM = 6500
  local ON_PWM = 20000

  if _sys_state == REQ_CAL_THRUST_ZERO_OFFSET then
    SRV_Channels:set_output_pwm(CAL_LED_RED, OFF_PWM)
    SRV_Channels:set_output_pwm(CAL_LED_GREEN, ON_PWM)
    SRV_Channels:set_output_pwm(CAL_LED_BLUE, ON_PWM)
  end

  if _sys_state == REQ_CAL_THRUST_FACTOR then
    SRV_Channels:set_output_pwm(CAL_LED_RED, OFF_PWM)
    SRV_Channels:set_output_pwm(CAL_LED_GREEN, OFF_PWM)
    SRV_Channels:set_output_pwm(CAL_LED_BLUE, ON_PWM)
  end

  if _sys_state == REQ_CAL_TORQUE_ZERO_OFFSET then
    SRV_Channels:set_output_pwm(CAL_LED_RED, ON_PWM)
    SRV_Channels:set_output_pwm(CAL_LED_GREEN, ON_PWM)
    SRV_Channels:set_output_pwm(CAL_LED_BLUE, OFF_PWM)
  end

  if _sys_state == REQ_CAL_TORQUE_FACTOR then
    SRV_Channels:set_output_pwm(CAL_LED_RED, ON_PWM)
    SRV_Channels:set_output_pwm(CAL_LED_GREEN, HALF_ON_PWM)
    SRV_Channels:set_output_pwm(CAL_LED_BLUE, OFF_PWM)
  end

  if _sys_state >= DISARMED then
    SRV_Channels:set_output_pwm(CAL_LED_RED, ON_PWM)
    SRV_Channels:set_output_pwm(CAL_LED_GREEN, OFF_PWM)
    SRV_Channels:set_output_pwm(CAL_LED_BLUE, ON_PWM)
  end

  _last_led_update = now_ms

end
------------------------------------------------------------------------


------------------------------------------------------------------------
-- Set the led colour brightness using ratio from 0 to 1
function set_LED_colour(led, r, g, b)
  local r_byte = math.floor(constrain(r*255,0,255))
  local g_byte = math.floor(constrain(g*255,0,255))
  local b_byte = math.floor(constrain(b*255,0,255))
  serialLED:set_RGB(_led_chan, led, r_byte, g_byte, b_byte)
end
------------------------------------------------------------------------


------------------------------------------------------------------------
-- Output message screens based on current system state
function update_display(now_ms)

  -- Update the display at 2 Hz only. Notify display is updated at max 2 Hz
  if (now_ms - _last_display_update_ms < _display_refresh_ms) then
    return
  end

  -- Always reset the refresh rate back to the fastest needed rate of 2 Hz.
  -- If transients are being recorded and throttle mode is set to step mode
  -- the refresh will be set back to slower rate.
  _display_refresh_ms = 500

  if _sys_state ~= _last_state_display then
    notify:clear_display_text()
  end

  if _sys_state == REQ_CAL_THRUST_ZERO_OFFSET then
    -- Tell user to unload thrust sensor for calibration before pressing the button
    notify:set_display_text(2, "   Unload thrust   ")
    notify:set_display_text(3, " sensor and press  ")
    notify:set_display_text(4, "calibration button.")
  end

  if _sys_state == REQ_CAL_THRUST_FACTOR then
    -- Tell user to apply calibration load
    notify:set_display_text(0, " Apply calibration ")
    notify:set_display_text(1, string.format("  load of %.2f g", _calibration_ref[THRUST]))
    notify:set_display_text(2, " to thrust sensor  ")
    notify:set_display_text(3, "     and press    ")
    notify:set_display_text(4, "calibration button.")
  end

  if _sys_state == REQ_CAL_TORQUE_ZERO_OFFSET then
    -- Tell user to unload torque sensor for calibration before pressing the button
    notify:set_display_text(2, "   Unload torque   ")
    notify:set_display_text(3, " sensor and press  ")
    notify:set_display_text(4, "calibration button.")
  end

  if _sys_state == REQ_CAL_TORQUE_FACTOR then
    -- Tell user to apply calibration load
    notify:set_display_text(0, " Apply calibration ")
    notify:set_display_text(1, string.format(" load of %.2f gcm", _calibration_ref[TORQUE]))
    notify:set_display_text(2, " to torque sensor  ")
    notify:set_display_text(3, "     and press     ")
    notify:set_display_text(4, "calibration button.")
  end

  if _sys_state == DISARMED or _sys_state == ARMED then
    -- Display throttle mode
    if _throttle_mode == THROTTLE_MODE_STEP then
      notify:set_display_text(0, "Step Throttle Mode")
      notify:set_display_text(1, "                   ")
      notify:set_display_text(2, "  Screen will not  ")
      notify:set_display_text(3, "  work when armed  ")
      notify:set_display_text(4, "    due to high    ")
      notify:set_display_text(5, "   logging rate.   ")

    else
      notify:set_display_text(0,"Thr Mode: Ramp")
      -- Display measurements
      notify:set_display_text(1, string.format("Throttle: %.2f %%", _current_thr*100.0))
      notify:set_display_text(2, string.format("  Thrust: %.0f g", _thrust))
      notify:set_display_text(3, string.format("  Torque: %.2f gcm", _torque))
      notify:set_display_text(4, string.format(" Current: %.2f A", _current))
      notify:set_display_text(5, string.format(" Voltage: %.2f V", _voltage))
    end
  end

  if _sys_state == CURRENT_PROTECTION then
    notify:set_display_text(2, "   Over current    ")
    notify:set_display_text(3, " protection active ")
    notify:set_display_text(5, "  Disarm to reset  ")
  end

  _last_state_display = _sys_state
  _last_display_update_ms = now_ms
end
------------------------------------------------------------------------


------------------------------------------------------------------------
function is_file(name)
  -- To test if the file actualy exists attempt to read the first line in the file.
  local file = io.open(name,"r")
  local content = file:read()
  file:close()

  if content~=nil then
    return true
  end
  return false
end
------------------------------------------------------------------------


------------------------------------------------------------------------
function protected_update()
    local success, err = pcall(update)
    if not success then
      gcs:send_text(0, "Internal Error: " .. err)
      return protected_update, 1000
    end

    return protected_update, _samp_dt_ms
end
------------------------------------------------------------------------

-- Wait a while before starting so we have a better chance of seeing any error messages
return init, 2000
