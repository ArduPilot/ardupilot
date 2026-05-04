-- Lua script for control surface + tilt motor check using RC overrides
-- Triggered via RC switch (e.g., RC7_OPTION=300)
-- ⚠️ Changes flight mode to MANUAL for the test, then returns to QLOITER.
-- 🟢 Repeatable on each RC switch HIGH transition

-- USER CONFIGURATION
local RC_CHAN_ROLL           = 1    -- Your aircraft's Roll input channel
local RC_CHAN_PITCH          = 2    -- Your aircraft's Pitch input channel
local RC_CHAN_FLAP           = 9    -- Your aircraft's Flap input channel

local RC_NEUTRAL_US          = 1500 -- Neutral PWM for RC channels
local RC_DEFLECTION          = 500  -- PWM deflection from neutral to simulate full stick
local STEP_DELAY_MS          = 3000 -- Delay between each movement in milliseconds
local MODE_CHANGE_TIMEOUT_MS = 5000 -- Max time to wait for a mode change
local REMINDER_INTERVAL_MS   = 15000 -- 15 seconds for the nag message

-- ArduPilot flight modes
local MODE_MANUAL            = 0 -- Manual mode (QuadPlane)
local MODE_QLOITER           = 19 -- QLoiter mode

-- PARAMETER SETUP
local PARAM_TABLE_KEY        = 73
assert(param:add_table(PARAM_TABLE_KEY, "CTSF_", 4), "Failed to create param table")
assert(param:add_param(PARAM_TABLE_KEY, 1, "TRIG_CH", 10), "Failed to create Param")

local CTSF_TRIG_CH = Parameter()
CTSF_TRIG_CH:init("CTSF_TRIG_CH")

-- Script state
local running                = false -- True if the test sequence is active
local step                   = 0 -- Current step in the sequence
local last_step_time         = 0 -- Timestamp of the last action
local prev_switch_state      = false -- Previous state of the trigger switch
local checks_completed       = false -- Tracks if the sequence has finished successfully
local last_reminder_time     = millis() -- Timestamp for the 30-second reminder timer

-- Request an authorization ID for custom pre-arm checks
local auth_id = arming:get_aux_auth_id()
if not auth_id then
    gcs:send_text(4, "ERROR: Could not get arming aux auth ID")
end

-- Get RC channel objects for Pitch, Roll, and Flaps
local rc_pitch_chan = rc:get_channel(RC_CHAN_PITCH)
local rc_roll_chan = rc:get_channel(RC_CHAN_ROLL)
local rc_flap_chan = rc:get_channel(RC_CHAN_FLAP)

if not rc_pitch_chan or not rc_roll_chan or not rc_flap_chan then
    gcs:send_text(4, "ERROR: RC Roll/Pitch/flap channels not found.")
    return -- Stop the script from running
end

-- Helper to change flight mode
local function change_mode(mode_id, mode_name)
    if vehicle:set_mode(mode_id) then
        gcs:send_text(6, "Attempting to switch to " .. mode_name .. " mode...")
        return true
    else
        gcs:send_text(4, "ERROR: Command to switch to " .. mode_name .. " failed.")
        return false
    end
end

-- Helper to release RC overrides back to transmitter control
local function release_overrides()
    rc_pitch_chan:set_override(0)
    rc_roll_chan:set_override(0)
    rc_flap_chan:set_override(0)
end

-- Main update loop, runs continuously
function update()
    local now = millis()

    -- 1. Pre-Arm Check Injection
    if auth_id then
        if not checks_completed then
            arming:set_aux_auth_failed(auth_id, "Control surface checks required")
        else
            arming:set_aux_auth_passed(auth_id)
        end
    end

    -- 2. Nag Reminder Logic (Runs every 30 seconds if checks aren't done)
    if not checks_completed and not running and (now - last_reminder_time >= REMINDER_INTERVAL_MS) then
        gcs:send_text(4, "Please run control surface checks before flight")
        last_reminder_time = now
    end

    -- Prevent running the check while armed
    if arming:is_armed() then
        return update, 100
    end

    -- 3. Check for the trigger switch activation (rising edge)
    local trig_chan = CTSF_TRIG_CH:get()
    local rc_pwm = rc:get_pwm(trig_chan)
    local switch_is_on = rc_pwm and rc_pwm > 1800
    local trigger_activated = switch_is_on and not prev_switch_state
    prev_switch_state = switch_is_on

    -- Start the sequence if triggered and not already running
    if trigger_activated and not running then
        running = true
        step = 0
        last_step_time = now
        gcs:send_text(1, "Starting control surface check")
    end

    -- 4. Execute the test sequence if it's running
    if not running then
        return update, 100 -- If not running, check again in 100ms
    end

    -- State Machine
    if step == 0 then
        -- Attempt to change to MANUAL mode
        if change_mode(MODE_MANUAL, "MANUAL") then
            step = 1 -- Move to next step to wait for confirmation
            last_step_time = now
            param:set('Q_TAILSIT_VFGAIN', 1)
            gcs:send_text(6, "Set Q_TAILSIT_VFGAIN to 1 (VTOL mode)")
        else
            running = false -- Abort if command fails
        end
    elseif step == 1 then
        -- Wait for MANUAL mode to be active
        if vehicle:get_mode() == MODE_MANUAL then
            gcs:send_text(1, "Mode Changed to Manual")
            step = 2 -- Mode confirmed, start movements
            last_step_time = now
        elseif (now - last_step_time > MODE_CHANGE_TIMEOUT_MS) then
            gcs:send_text(4, "TIMEOUT: Failed to enter MANUAL mode. Aborting.")
            release_overrides()
            running = false
        end
    elseif step >= 2 then
        -- Execute timed movements
        if (now - last_step_time < STEP_DELAY_MS) then
            return update, 50 -- Not yet time for the next movement
        end
        last_step_time = now  -- Reset timer for the next step

        if step == 2 then
            -- Pitch Up
            gcs:send_text(1, "Pitch Up")
            rc_pitch_chan:set_override(RC_NEUTRAL_US - RC_DEFLECTION)
            step = step + 1
        elseif step == 3 then
            -- Pitch Down
            gcs:send_text(1, "Pitch Down")
            rc_pitch_chan:set_override(RC_NEUTRAL_US + RC_DEFLECTION)
            step = step + 1
        elseif step == 4 then
            -- Pitch Neutral
            gcs:send_text(1, "Pitch Neutral")
            rc_pitch_chan:set_override(RC_NEUTRAL_US)
            step = step + 1
        elseif step == 5 then
            -- Roll Right
            gcs:send_text(1, "Roll Right")
            rc_roll_chan:set_override(RC_NEUTRAL_US + RC_DEFLECTION)
            step = step + 1
        elseif step == 6 then
            -- Roll Left
            gcs:send_text(1, "Roll Left")
            rc_roll_chan:set_override(RC_NEUTRAL_US - RC_DEFLECTION)
            step = step + 1
        elseif step == 7 then
            -- Door Closed
            gcs:send_text(1, "Cargo bay door closed")
            -- Also neutralising roll from the previous step
            rc_roll_chan:set_override(RC_NEUTRAL_US)
            rc_flap_chan:set_override(RC_NEUTRAL_US + RC_DEFLECTION)
            step = step + 1
        elseif step == 8 then
            -- Door Open
            gcs:send_text(1, "Cargo bay door open")
            rc_flap_chan:set_override(RC_NEUTRAL_US - RC_DEFLECTION)
            step = step + 1
        elseif step == 9 then
            -- Sequence complete: reset everything
            gcs:send_text(1, "All checks complete. Releasing controls.")
            change_mode(MODE_QLOITER, "QLOITER")
            param:set('Q_TAILSIT_VFGAIN', 0.2)
            gcs:send_text(6, "Set Q_TAILSIT_VFGAIN to 0.2 (VTOL mode)")
            
            release_overrides() -- Releases overrides completely back to TX
            
            checks_completed = true -- Flags checks as done to clear pre-arm block & 30s reminder
            running = false -- Reset for the next trigger
        end
    end

    return update, 50 -- Schedule the next run
end

-- Confirm script loaded
gcs:send_text(6, "Airbound Lua: Control Surface Check Script Loaded")

-- Initial call to start the scripting
return update()