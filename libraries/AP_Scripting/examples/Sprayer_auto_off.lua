
 --Allows crop sprayers to automatically turn off sprayer on RTL, LAND, Failsafe triggers(RC_FS etc.)

-- Check https://ardupilot.org/copter/docs/sprayer.html to activate speed dependant automatic spraying
-- Purpose of this script is improving spraying automation ability of ardupilot. 
--Normally sprayer won't stop on RTL after mission complete or when RC connection is lost without this script
--cause it is only speed dependant.
-- User doesn't need to set sprayer enable on RCx_OPTION parameters 

--### How to use
-- Activate scripting SCR_ENABLE = 1
-- Assign any RCx_OPTION parameter to Scripting1(300) to on/off sprayer



--
-- Allows crop sprayers to automatically turn off sprayer on RTL, LAND, Failsafe triggers(RC_FS etc.)
--

local FREQUENCY      = 250    -- ms
local VERBOSE_MODE   = 2

local sprayer_function = 15    -- AUX function Sprayer Enable is assigned to number 15

local HIGH    = 2    -- High
local LOW     = 0    -- Low

local COPTER_MODE_AUTO    = 3
local COPTER_MODE_RTL     = 6
local COPTER_MODE_LAND    = 9

local sprayer_state = nil  -- Track sprayer state (Running or Stopped)

function sprayer_update()
    local switch_pos = rc:get_aux_cached(300) -- 300 = Lua Script 1 option
    local mode = vehicle:get_mode()    --get current mode
    local current_nav_index = mission:get_current_nav_index()    --get current command index
    local num_command = mission:num_commands()    --total numbers of commands

    local new_state = nil     --current Track sprayer state (Running or Stopped)

    -- Manual override: Stop sprayer if user sets RC channel to low
    if switch_pos == 0 then -- Switch is in "off" position
        new_state = LOW
    -- Automatic control: Stop in RTL, LAND, or at the last waypoint
    elseif mode == COPTER_MODE_RTL or mode == COPTER_MODE_LAND or 
           (mode == COPTER_MODE_AUTO and current_nav_index == num_command - 1) then
        new_state = LOW
    else
        -- Otherwise, start the sprayer
        new_state = HIGH
    end

    -- Update sprayer only if the state has changed
    if new_state ~= sprayer_state then
        sprayer_state = new_state
        rc:run_aux_function(sprayer_function, sprayer_state)
        if VERBOSE_MODE > 0 then
            if sprayer_state == HIGH then
                gcs:send_text(4, "Sprayer running")
            else
                gcs:send_text(4, "Sprayer stopped")
            end
        end
    end

    return sprayer_update, FREQUENCY
end

return sprayer_update, FREQUENCY
