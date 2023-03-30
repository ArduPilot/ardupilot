-- This script calls out altitude if the user is near (WARN) or above (MAX)
-- Allows for units to be feet even if you do everything else in metric becuase the laws typically specify 400 feet for UAV/RPAS in most countries
-- all of the settings are stored in parameters:
-- CALLOUT_ALT_UNITS 1 = metric, 2 (default) = imperial
-- CALLOUT_ALT_MAX max allowed altitude (its still a message there is no action)
-- CALLOUT_ALT_STEP callout (via GC message) when altitide changes by this amount or more
-- CALLOUT_ALT_CALL_SEC secods between callout of flying altitude
-- CALLOUT_ALT_WARN_SEC seconds between callouts that you are less than ALT_STEP below ALT_MAX
-- CALLOUT_ALT_HIGH_SEC seconds between callouts that you have exceeded ALT_MAX

local REFRESH_RATE      = 1000	    --check every 1 second
local MAV_SEVERITY_ERROR = 3        --/* Indicates an error in secondary/redundant systems. | */
local MAV_SEVERITY_WARNING = 4      --/* Indicates about a possible future error if this is not resolved within a given timeframe. Example would be a low battery warning. | */
local MAV_SEVERITY_INFO = 6         --/* No rmal operational messages. Useful for logging. No action is required for these messages. | */

local PARAM_TABLE_KEY = 88
assert(param:add_table(PARAM_TABLE_KEY, "CALLOUT_", 10), 'could not add param table')
assert(param:add_param(PARAM_TABLE_KEY, 1,  'ALT_UNITS', 2), 'could not add CALLOUT_ALT_UNITS')	-- default to feet, not great, but its what most countries have in their laws
local CALLOUT_ALT_UNITS = Parameter()
CALLOUT_ALT_UNITS:init('CALLOUT_ALT_UNITS')
local alt_units = CALLOUT_ALT_UNITS:get()
assert(param:add_param(PARAM_TABLE_KEY, 2,  'ALT_MAX', 400), 'could not add CALLOUT_ALT_MAX')	-- almost everyone is using 400 feet
local CALLOUT_ALT_MAX = Parameter()
CALLOUT_ALT_MAX:init('CALLOUT_ALT_MAX')
local alt_max = CALLOUT_ALT_MAX:get()
assert(param:add_param(PARAM_TABLE_KEY, 3,  'ALT_STEP', 25), 'could not add CALLOUT_ALT_STEP')	-- seems reasonable to callout every 25 feet, but it can be changed.
local CALLOUT_ALT_STEP = Parameter()
CALLOUT_ALT_STEP:init('CALLOUT_ALT_STEP')
local alt_step = CALLOUT_ALT_STEP:get()
assert(param:add_param(PARAM_TABLE_KEY, 4,  'ALT_CALL', 25), 'could not add CALLOUT_ALT_CALL_SEC')	-- how often to callout altitude if all is ok
local CALLOUT_ALT_CALL_SEC = Parameter()
CALLOUT_ALT_CALL_SEC:init('CALLOUT_ALT_CALL')
local alt_call_sec = CALLOUT_ALT_CALL_SEC:get()
gcs:send_text(MAV_SEVERITY_INFO, string.format("alt_call_sec: %i", alt_call_sec))

assert(param:add_param(PARAM_TABLE_KEY, 5,  'ALT_WARN', 25), 'could not add CALLOUT_ALT_WARN_SEC')	-- how often to nag about almost hitting MAX
local CALLOUT_ALT_WARN_SEC = Parameter()
CALLOUT_ALT_WARN_SEC:init('CALLOUT_ALT_WARN')
local alt_warn_sec = CALLOUT_ALT_WARN_SEC:get()
gcs:send_text(MAV_SEVERITY_INFO, string.format("alt_warn_sec: %i", alt_warn_sec))

assert(param:add_param(PARAM_TABLE_KEY, 6,  'ALT_HIGH', 25), 'could not add CALLOUT_ALT_HIGH_SEC')	-- how often to nag about hitting MAX
local CALLOUT_ALT_HIGH_SEC = Parameter()
CALLOUT_ALT_HIGH_SEC:init('CALLOUT_ALT_HIGH')
local alt_high_sec = CALLOUT_ALT_HIGH_SEC:get()


local alt_last = 0
local alt_warn = alt_max - alt_step
gcs:send_text(MAV_SEVERITY_INFO, string.format("alt_warn %i", alt_warn))

local unit = "meters"
if (alt_units == 2) then
    unit = "feet"
end
altitude_max = string.format("%i %s", math.floor(alt_max+0.5), unit )

local time_last_warn_s = 0
local time_last_max_s = 0
local time_last_update_s = 0
local alt_max_exceeded = false
local alt_warn_exceeded = false

----  CLASS: Arming_Check  ----
local Arming_Check = {}

local function idle_while_notarmed()
    if arming:is_armed() then return CallOut, REFRESH_RATE end
    return idle_while_notarmed, REFRESH_RATE * 10
end

function CallOut() -- this is the loop which periodically runs

    if not arming:is_armed() then return idle_while_notarmed() end

    local current_time_s = millis() / 1000
    -- setting the height/altitude variables like this means all the code below works without change for either metric or Imperial units
    local terrain_height = terrain:height_above_terrain(true)
    local altitude = string.format("%i meters", math.floor(terrain_height+0.5) )
    if (alt_units == 2) then
        terrain_height = terrain_height * 3.28084
    end
    altitude = string.format("%i %s", math.floor(terrain_height+0.5), unit )

    -- gcs:send_text(MAV_SEVERITY_INFO, string.format("Altitude: %s", altitude))
    if terrain_height > alt_max then
        if (time_last_max_s < current_time_s - alt_high_sec) then
            gcs:send_text(MAV_SEVERITY_ERROR, string.format("Altitude is too high %s", altitude ))
            time_last_max_s = current_time_s
            alt_max_exceeded = true
        end
    else
        if terrain_height > alt_warn then
            if (time_last_warn_s < current_time_s - alt_warn_sec) then
                gcs:send_text(MAV_SEVERITY_WARNING, string.format("Warning altitude is %s", altitude ))
                time_last_warn_s = current_time_s
                alt_warn_exceeded = true
            end
	else 
	    -- we are fine, but maybe we were not fine. So if we previously displayed altitude messages, let the pilot know we are now fine
            if( alt_max_exceeded or alt_warn_exceeded) then
                gcs:send_text(MAV_SEVERITY_WARNING, string.format("Altitude %s is Ok", altitude ))
            else
                -- nothing else important happened, so see if our altitude has gone up or down by more than ALT_STEP
		-- in which case we call it out
                if (time_last_update_s < current_time_s - alt_call_sec) then
                    local alt_diff = (terrain_height - alt_last) 
                    if (math.abs(alt_diff) > alt_step) then
                       gcs:send_text(MAV_SEVERITY_WARNING, string.format("Altitude is %s", altitude ))
                       alt_last = math.floor(terrain_height / alt_step,0.5) * alt_step
                    end
                    time_last_update_s = current_time_s
                end
            end
            alt_max_exceeded = false
            alt_warn_exceeded = false
        end

    end

    return CallOut, REFRESH_RATE
end

gcs:send_text(MAV_SEVERITY_WARNING, string.format("Altitude call outs max %s", altitude_max) )

return CallOut() -- run immediately before starting to reschedule
