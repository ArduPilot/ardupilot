--[[

   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program.  If not, see <http://www.gnu.org/licenses/>.

--]]

SCRIPT_VERSION = "4.6.0-003"
SCRIPT_NAME = "Plane Altitude Callouts"
SCRIPT_NAME_SHORT = "Callout"

-- This script calls out altitude if the user is near (WARN) or above (MAX)
-- Allows for units to be feet even if you do everything else in metric because the laws typically specify 400 feet for UAV/RPAS in most countries
-- all of the settings are stored in parameters:
-- CALLOUT_ALT_UNITS 1 = metric, 2 (default) = imperial
-- CALLOUT_ALT_MAX max allowed altitude (its still a message there is no action)
-- CALLOUT_ALT_STEP callout (via GC message) when altitude changes by this amount or more
-- CALLOUT_ALT_CALL seconds between callout of flying altitude
-- CALLOUT_ALT_WARN seconds between callouts that you are less than ALT_STEP below ALT_MAX
-- CALLOUT_ALT_HIGH seconds between callouts that you have exceeded ALT_MAX

REFRESH_RATE      = 1000	    --check every 1 second

MAV_SEVERITY = {EMERGENCY=0, ALERT=1, CRITICAL=2, ERROR=3, WARNING=4, NOTICE=5, INFO=6, DEBUG=7}

local PARAM_TABLE_KEY = 102
local PARAM_TABLE_PREFIX = "ZPC_"

-- add a parameter and bind it to a variable
local function bind_add_param(name, idx, default_value)
   assert(param:add_param(PARAM_TABLE_KEY, idx, name, default_value), string.format('could not add param %s', name))
   return Parameter(PARAM_TABLE_PREFIX .. name)
end

-- setup follow mode specific parameters
assert(param:add_table(PARAM_TABLE_KEY, PARAM_TABLE_PREFIX, 10), 'could not add param table')


-- Add these ZPC_ parameters specifically for this script
--[[
  // @Param: ZPC_ALT_UNITS
  // @DisplayName: Plane Callouts Units - defaults to feet
  // @Description: 1: Metric/meters or 2: Imperial/feet
  // @User: Standard
--]]
ZPC_ALT_UNITS = bind_add_param('ALT_UNITS', 1, 2)

--[[
    // @Param: ZPC_ALT_MAX
    // @DisplayName: Plane Callouts max altitude
    // @Description: Maximum altitude in ALT_UNITS
    // @Range: 0 30
    // @Units: seconds
--]]
ZPC_ALT_MAX = bind_add_param("ALT_MAX", 2, 400)

--[[
    // @Param: ZPC_ALT_STEP
    // @DisplayName: Plane Callouts altitude steps
    // @Description: Altitude steps for callouts in ALT_UNITS
    // @Range: 0 100
    // @Units: ALT_UNIT
--]]
ZPC_ALT_STEP = bind_add_param("ALT_STEP", 3, 25)

--[[
    // @Param: ZPC_ALT_CALL
    // @DisplayName: Plane Callouts frequency
    // @Description: How often to callout altitude when flying normally
    // @Range: 0 30
    // @Units: seconds
--]]
ZPC_ALT_CALL = bind_add_param("ALT_CALL", 4, 25)

--[[
    // @Param: ZPC_ALT_WARN
    // @DisplayName: Plane altitude warning frequency
    // @Description: How often to nag about almost hitting MAX
    // @Range: 0 30
    // @Units: seconds
--]]
ZPC_ALT_WARN = bind_add_param("ALT_WARN", 5, 25)

--[[
    // @Param: ZPC_ALT_HIGH
    // @DisplayName: Plane altitude max altitude callout frequency
    // @Description: How often to nag about exceeding MAX
    // @Range: 0 30
    // @Units: seconds
--]]
ZPC_ALT_HIGH = bind_add_param("ALT_HIGH", 6, 10)

local alt_units = ZPC_ALT_UNITS:get() or 2  -- default to feet because "thats what it is"
local alt_max = ZPC_ALT_MAX:get() or 400    -- most places this is the legal limit, if you have a different limit, change this
local alt_step = ZPC_ALT_STEP:get() or 25
local alt_call_sec = ZPC_ALT_CALL:get() or 25
local alt_warn_sec = ZPC_ALT_WARN:get() or 25
local alt_high_sec = ZPC_ALT_HIGH:get() or 10

local alt_last = 0
local alt_warn = alt_max - alt_step

local unit = "meters"
if (alt_units == 2) then
    unit = "feet"
end
local altitude_max = string.format("%i %s", math.floor(alt_max+0.5), unit )

local time_last_warn_s = millis() / 1000
local time_last_max_s = millis() / 1000
local time_last_update_s = millis() / 1000
local alt_max_exceeded = false
local alt_warn_exceeded = false

local function update() -- this is the loop which periodically runs
    local current_time_s = millis() / 1000
    -- setting the height/altitude variables like this means all the code below works without change for either metric or Imperial units
    local terrain_height = terrain:height_above_terrain(true)

    -- if terrain height is not available use height above home
    if terrain_height == nil then
            -- override terrain height with home height (TODO: parameterize this)
        local pos = ahrs:get_relative_position_NED_home()
        if pos == nil then
            return
        else
            terrain_height = -pos:z()
        end
    end

    --- allow these parameters to be changed at runtime.
    alt_units = ZPC_ALT_UNITS:get() or 2
    alt_max = ZPC_ALT_MAX:get() or 400    -- most places this is the legal limit, if you have a different limit, change this
    alt_step = ZPC_ALT_STEP:get() or 25
    alt_call_sec = ZPC_ALT_CALL:get() or 25
    alt_warn_sec = ZPC_ALT_WARN:get() or 25
    alt_high_sec = ZPC_ALT_HIGH:get() or 10
    if (alt_units == 2) then
        unit = "feet"
    else
        unit = "meters"
    end

    if (alt_units == 2) then
        terrain_height = terrain_height * 3.28084
    end
    local altitude = string.format("%i %s", math.floor(terrain_height+0.5), unit )

    if terrain_height > alt_max then
        if (time_last_max_s < current_time_s - alt_high_sec) then
            gcs:send_text(MAV_SEVERITY.ERROR, string.format("Altitude %s too high max %s", altitude,  altitude_max ))
            time_last_max_s = current_time_s
            alt_max_exceeded = true
        end
    elseif terrain_height > alt_warn then
            if (time_last_warn_s < current_time_s - alt_warn_sec) then
                gcs:send_text(MAV_SEVERITY.WARNING, string.format("Warning altitude is %s", altitude ))
                time_last_warn_s = current_time_s
                alt_warn_exceeded = true
            end
	else
	    -- we are fine now, but maybe we were not fine before. 
        -- So if we previously displayed altitude warn/error messages, let the pilot know we are now fine
        if(alt_max_exceeded or alt_warn_exceeded) then
            gcs:send_text(MAV_SEVERITY.WARNING, string.format("Altitude %s is Ok", altitude ))
        else
            -- nothing else important happened, so see if our altitude has gone up or down by more than ALT_STEP
		    -- in which case we call it out
            if (time_last_update_s < current_time_s - alt_call_sec) then
                local alt_diff = (terrain_height - alt_last) 
                if (math.abs(alt_diff) > alt_step) then
                    gcs:send_text(MAV_SEVERITY.WARNING, string.format("Altitude is %s", altitude ))
                    alt_last = math.floor(terrain_height / alt_step + 0.5) * alt_step
                end
                time_last_update_s = current_time_s
            end
        end
        alt_max_exceeded = false
        alt_warn_exceeded = false
    end
end

local displayed_banner = false
-- wrapper around update(). This calls update() at 1Hz,
-- and if update faults then an error is displayed, but the script is not
-- stopped
local function protected_wrapper()

    if not displayed_banner then
        gcs:send_text(MAV_SEVERITY.INFO, string.format("%s %s script loaded callouts in %s", SCRIPT_NAME, SCRIPT_VERSION, unit) )
        displayed_banner = true
    end

    if not arming:is_armed() then return protected_wrapper, REFRESH_RATE * 10 end

    local success, err = pcall(update)
    if not success then
        gcs:send_text(MAV_SEVERITY.ALERT, SCRIPT_NAME_SHORT .. "Internal Error: " .. err)
        -- when we fault we run the update function again after 1s, slowing it
        -- down a bit so we don't flood the console with errors
        return protected_wrapper, 1000
    end
    return protected_wrapper, REFRESH_RATE
end


  -- start running update loop - wait 20 seconds before starting up
return protected_wrapper, 20000

