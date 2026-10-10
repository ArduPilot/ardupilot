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

 Arm/Emergency Stop Interlock for Plane

 The action switch is a three-position switch on the transmitter. Changes to it are accepted only
 while the pilot holds a second, momentary switch (interlock). Configure the interlock to return to
 low when released and send middle or high while held.

 Only the action and interlock channels are needed. Do not assign RCx_OPTION=165;
 the script invokes that function directly, without an RC channel assignment.
--]]

SCRIPT_NAME = "Arm/E-Stop Interlock"
SCRIPT_NAME_SHORT = "AEST-Lock"
SCRIPT_VERSION = "4.8.0-010"

REFRESH_RATE    = 20   -- Hertz
STARTUP_DELAY   = 25  -- wait this many seconds for the FC to come up before starting the script

MAV_SEVERITY    = {ERROR=3, NOTICE=5, INFO=6}

local RC_OPTION = {ESTOPArm=165}
local AuxSwitchPos = {LOW=0, MIDDLE=1, HIGH=2}

local PARAM_TABLE_KEY = 194
local PARAM_TABLE_PREFIX = "INTLCK_"

-- add a parameter and bind it to a variable
function bind_add_param(name, idx, default_value)
    assert(param:add_param(PARAM_TABLE_KEY, idx, name, default_value), SCRIPT_NAME_SHORT .. string.format(': could not add param %s', name))
    return Parameter(PARAM_TABLE_PREFIX .. name)
end

-- Reserve the interlock parameter table.
assert(param:add_table(PARAM_TABLE_KEY, PARAM_TABLE_PREFIX, 10), SCRIPT_NAME_SHORT .. ': could not add param table: ' .. PARAM_TABLE_PREFIX)

--[[
    // @Param: INTLCK_ACT_FN
    // @DisplayName: Arm/EStop Function
    // @Description: Setting an RC channel's _OPTION to this value will use it as the main Arm/EStop switch (similar to the standard RCx_OPTION = 165)
    // @Range: 300 307
    // @User: Advanced
--]]
INTLCK_ACT_FN = bind_add_param("ACT_FN", 1, 306)

--[[
    // @Param: INTLCK_LCK_FN
    // @DisplayName: Arm/EStop Interlock Function
    // @Description: Setting an RC channel's _OPTION to this value will use it as the interlock switch; middle or high allows action switch changes
    // @Range: 300 307
    // @User: Advanced
--]]
INTLCK_LCK_FN = bind_add_param("LCK_FN", 2, 307)

local last_switch_state = 0
local last_switch_function = INTLCK_ACT_FN:get()
local duplicate_functions = false

local function estop_motors()
    rc:run_aux_function(RC_OPTION.ESTOPArm, AuxSwitchPos.LOW)
    gcs:send_text(MAV_SEVERITY.NOTICE, SCRIPT_NAME_SHORT .. " motors OFF")
end

local function enable_motors()
    rc:run_aux_function(RC_OPTION.ESTOPArm, AuxSwitchPos.MIDDLE)
    gcs:send_text(MAV_SEVERITY.NOTICE, SCRIPT_NAME_SHORT .. " motors ON")
end

local function attempt_arm()
    rc:run_aux_function(RC_OPTION.ESTOPArm, AuxSwitchPos.HIGH)
    gcs:send_text(MAV_SEVERITY.NOTICE, SCRIPT_NAME_SHORT .. " arming ...")
end

local function update()
    local switch_function = INTLCK_ACT_FN:get()
    local switch_state = rc:get_aux_cached(switch_function) or -1
    local lock_function = INTLCK_LCK_FN:get()
    if switch_function == lock_function then
        if not duplicate_functions then
            gcs:send_text(MAV_SEVERITY.ERROR, SCRIPT_NAME_SHORT .. " RC functions must differ")
        end
        duplicate_functions = true
        -- Discard changes while misconfigured so correcting the parameters cannot replay them.
        last_switch_state = switch_state
        return
    end
    if duplicate_functions or switch_function ~= last_switch_function then
        -- A configuration change is not a pilot switch movement.
        duplicate_functions = false
        last_switch_function = switch_function
        last_switch_state = switch_state
        return
    end
    if (switch_state ~= last_switch_state) then
        -- we have a change on the main switch, but is the pilot holding down the interlock?
        local lock_state = rc:get_aux_cached(lock_function) or -1

        -- only execute the arm/estop if interlock is held (should be a momentary switch to work best)
        if lock_state > 0 then
            if switch_state == 0 then -- request emergency motor stop
                estop_motors()
            elseif switch_state == 1 then
                enable_motors()
            elseif switch_state == 2 then -- clear emergency stop and request arming
                attempt_arm()
            end
        else
            gcs:send_text(MAV_SEVERITY.ERROR, SCRIPT_NAME_SHORT .. " no Interlock")
        end
        last_switch_state = switch_state
    end
end

-- wrapper around update(). This calls update() at REFRESH_RATE Hz, i.e. every 1000/REFRESH_RATE milliseconds
-- and if update faults then an error is displayed, but the script is not stopped
function Protected_Wrapper()
    local success, err = pcall(update)
    if not success then
       gcs:send_text(0, SCRIPT_NAME_SHORT .. ": Error: " .. err)
       -- when we fault we run the update function again after 1s, slowing it
       -- down a bit so we don't flood the console with errors
       return Protected_Wrapper, 1000
    end
    return Protected_Wrapper, 1000 / REFRESH_RATE
end

function Delayed_Startup()
    gcs:send_text(MAV_SEVERITY.INFO, string.format("%s %s script loaded", SCRIPT_NAME, SCRIPT_VERSION) )
    return Protected_Wrapper()
end

if arming:is_armed() then
    return Delayed_Startup()
else
    -- stop the motors right away so they don't get activated before the AP and script are up
    estop_motors()
    -- wait a bit for AP to come up then start running update loop
    return Delayed_Startup, 1000 * STARTUP_DELAY
end
