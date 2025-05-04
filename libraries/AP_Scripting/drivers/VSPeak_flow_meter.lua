-- VSPeak_flow_meter.lua: Driver for the VSPeak Modell fuel flow sensor.
--
-- Setup
--      Read the accompanying .md file.
--
-- Usage
--      Read the accompanying .md file.

--[[
Global definitions
--]]
local MAV_SEVERITY = {EMERGENCY=0, ALERT=1, CRITICAL=2, ERROR=3, WARNING=4, NOTICE=5, INFO=6, DEBUG=7}
local SCRIPT_NAME = "VSPeak Modell flow meter driver"
local NUM_SCRIPT_PARAMS = 4
local LOOP_RATE_HZ = 10
local last_warning_time_ms = uint32_t(0)
local WARNING_DEADTIME_MS = 1000
local FRAME_LEN = 7
local BUFFER_LEN = 2*FRAME_LEN - 1
local HEADER_BYTE = 0xFE
local INIT_DELAY_MS = 5000
local uart
local state = {}


-- State machine states.
local FSM_STATE = {
    INACTIVE = 0,
    ACTIVE = 1,
}
local current_state = FSM_STATE.INACTIVE
local next_state = FSM_STATE.INACTIVE

--[[
New parameter declarations
--]]
local PARAM_TABLE_KEY = 142 -- Ensure no other applet is using this key.
local PARAM_TABLE_PREFIX = "VSPF_"

-- Bind a parameter to a variable.
function bind_param(name)
   return Parameter(name)
end

-- Add a parameter and bind it to a variable.
function bind_add_param(name, idx, default_value)
   assert(param:add_param(PARAM_TABLE_KEY, idx, name, default_value), string.format('could not add param %s', name))
   return bind_param(PARAM_TABLE_PREFIX .. name)
end

-- Add param table.
assert(param:add_table(PARAM_TABLE_KEY, PARAM_TABLE_PREFIX, NUM_SCRIPT_PARAMS), SCRIPT_NAME .. ': Could not add param table.')

--[[
  // @Param: ENABLE
  // @DisplayName: Enable this script
  // @Description: When set to 0 this script will not run. When set to 1 this script will run.
  // @Range: 0 1
  // @User: Standard
--]]
local VSPF_ENABLE = bind_add_param('ENABLE', 1, 0)

--[[
  // @Param: BATT_IDX
  // @DisplayName: Index of assigned battery.
  // @Description: Ensure this battery is configured with `BATT*_MONITOR=29`.
  // @User: Standard
--]]
local VSPF_BAT_IDX = bind_add_param('BAT_IDX', 2, 2)

--[[
  // @Param: CFACT
  // @DisplayName: Measurement correction factor
  // @Description: This is multiplicative factor to correct the measured flow. Set to <1 if your sensor measures too high.
  // @User: Standard
--]]
local VSPF_CFACT = bind_add_param('CFACT', 3, 1)

--[[
  // @Param: MODE
  // @DisplayName: Sensor operating mode
  // @Description: 0: The sensor will save the fuel consumption across power resets. 1: The sensor will reset the power consumption.
  // @User: Standard
--]]
local VSPF_MODE = bind_add_param('MODE', 4, 0)

-- Warn the user, throttling the message rate.
function warn_user(msg, severity)
    severity = severity or MAV_SEVERITY.WARNING -- Optional severity argument.
    if millis() - last_warning_time_ms > WARNING_DEADTIME_MS then
            gcs:send_text(severity, SCRIPT_NAME .. ": " .. msg)
        last_warning_time_ms = millis()
    end
end

-- Get uint16 from two bytes
function uint16_value(hbyte, lbyte)
  return ((hbyte & 0xFF) << 8) | (lbyte & 0xFF)
end

function read_uart()
    local n_avail = uart:available():toint()

    -- Discard up to BUFFER_LEN bytes.
    -- These are stale data.
    if (n_avail - BUFFER_LEN) > 0 then
        n_avail = BUFFER_LEN
        for _ = 1, (n_avail-BUFFER_LEN) do
            uart:read()
        end
    end

    -- Read in the rest of the data.
    for _ = 1, n_avail do
        local val = uart:read()
        table.insert(state.buffer, val)
    end

end

function parse_buffer()

    -- Drop old data.
    for _ = 1, (#state.buffer - BUFFER_LEN) do
        table.remove(state.buffer, 1)
    end

    -- Find the header byte and discard up to it.
    for _ = 1, #state.buffer do
        if state.buffer[1] ~= HEADER_BYTE then
            table.remove(state.buffer, 1)
        else
            break
        end
    end
    
    -- Check if the buffer has enough data in it.
    if #state.buffer >= FRAME_LEN then
        -- Parse the data.
        state.flow = uint16_value(state.buffer[3], state.buffer[2])
        state.fuel_ml = uint16_value(state.buffer[5], state.buffer[4])
        state.fuel_pct = state.buffer[6]
        state.checksum = state.buffer[7]
        -- Calculate the checksum.
        local checksum = 0x00
        for i = 1, 6 do
            checksum = (checksum ~ state.buffer[i]) & 0xFF
        end

        -- If parse successful, discard them.
        if checksum == state.checksum then
            state.updated = true
            for _ = 1, FRAME_LEN do
                table.remove(state.buffer, 1)
            end
        else
            warn_user("Checksum failed")
            state.updated = false
            -- Discard the header to force the corrupt data to flush on the next pass.
            table.remove(state.buffer, 1)
        end
        
    end

end

function update_battery()
    if state.updated == false then
        return
    end

    -- Assumption: litres per hour will be used as equivalent to Amperes.
    local bat_state = BattMonitorScript_State()
    bat_state:healthy(true)
    bat_state:cell_count(1)
    bat_state:voltage(1)
    bat_state:current_amps(state.flow/1000*60.0*VSPF_CFACT:get()) -- Convert from ml/min to l/h.
    bat_state:temperature(0)

    if VSPF_MODE:get() == 1 then
        bat_state:consumed_mah(state.fuel_ml*VSPF_CFACT:get())
    end

    battery:handle_scripting(VSPF_BAT_IDX:get()-1, bat_state)
end

--[[
Activation conditions
--]]
-- Check for script activating conditions here.
function can_start()
    return VSPF_ENABLE:get() == 1 and (millis() > INIT_DELAY_MS)
end

--[[
Deactivation conditions
--]]
-- Check for script deactivating conditions here.
function must_stop()
    return VSPF_ENABLE:get() == 0
end

--[[
State machine
--]]
-- Write the state machine transitions.
function fsm_step()
    if current_state == FSM_STATE.INACTIVE then

        if can_start() then
            setup()
            next_state = FSM_STATE.ACTIVE
        end

    elseif current_state == FSM_STATE.ACTIVE then

        -- Feed the buffer.
        read_uart()
        parse_buffer()
        update_battery()

        if must_stop() then
            next_state = FSM_STATE.INACTIVE
        end
	end
    current_state = next_state
end

--[[
Main loop function
--]]
function update()
	-- Stuff is placed here.
    fsm_step()
end

--[[
Setup function
--]]
function setup()
    uart = serial:find_serial(0) -- First scripting serial
    if not uart then
        warn_user("Unable to find scripting serial", MAV_SEVERITY.ERROR)
        return
    end
    uart:begin(19200)

    state.last_read_us = 0x00
    state.buffer = {}
    state.flow = 0x00
    state.fuel_ml = 0x00
    state.fuel_pct = 0x00
    state.checksum = 0x00
    state.updated = false
end

gcs:send_text(MAV_SEVERITY.INFO, SCRIPT_NAME .. string.format(" loaded."))

-- Wrapper around update() to catch errors.
function protected_wrapper()
  local success, err = pcall(update)
  if not success then
    gcs:send_text(MAV_SEVERITY.EMERGENCY, "Internal Error: " .. err)
    return protected_wrapper, 1000
  end
  return protected_wrapper, 1000.0/LOOP_RATE_HZ
end

-- Start running update loop
return protected_wrapper()
