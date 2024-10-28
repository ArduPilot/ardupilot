--[[----------------------------------------------------------------------------

rover-l298n ArduPilot Lua script

This script provides ArduPilot Rover support for L298N dual motor controller boards.
Both Ackermann and skid steering applications are supported.

For usage instructions, see:
https://github.com/ArduPilot/ardupilot/blob/master/libraries/AP_Scripting/applets/rover-l298n.md

CAUTION: This script is capable of engaging and disengaging autonomous control
of a vehicle.  Use this script AT YOUR OWN RISK.

-- Yuri -- Oct 2024

LICENSE - GNU GPLv3 https://www.gnu.org/licenses/gpl-3.0.en.html
------------------------------------------------------------------------------]]

local MAV_SEVERITY = { EMERGENCY = 0, ALERT = 1, CRITICAL = 2, ERROR = 3, WARNING = 4, NOTICE = 5, INFO = 6, DEBUG = 7 }
local RUN_INTERVAL_MS = 20 -- 50Hz to coincide with Rover control loop

-- use the default reverse relay + relay 3
local IN1 = 0 -- RELAY1_FUNCTION,5
local IN2 = 2 -- RELAY3_FUNCTION,1

-- for skid steering, use the default second reverse relay + relay 4
local IN3 = 1 -- RELAY2_FUNCTION,6
local IN4 = 3 -- RELAY4_FUNCTION,1

-- validate a few basic configuration settings
local is_ackermann = SRV_Channels:find_channel(70) ~= nil
local is_skid_steer = SRV_Channels:find_channel(73) ~= nil and SRV_Channels:find_channel(74) ~= nil
assert(param:get('MOT_PWM_TYPE') == 3, 'L298N: MOT_PWM_TYPE must be 3')
assert(is_ackermann or is_skid_steer, 'L298N: Throttle output(s) unassigned')
assert(param:get('RELAY1_FUNCTION') == 5 and relay:enabled(IN2), 'L298N: Relays 1 and 3 must be enabled')
if is_skid_steer then
    assert(param:get('RELAY2_FUNCTION') == 6 and relay:enabled(IN4), 'L298N: Relays 2 and 4 must be enabled')
end

-- use the default reverse relay as IN1 (and IN3)
-- set IN2 (and IN4) to the inverse of the default brushed motor reverse relay
local function set_input_pins(in1_state, in3_state)
    -- turn all relays off when disarmed
    if not arming:is_armed() then
        if relay:get(IN2) ~= 0 then
            relay:off(IN2)
        end

        if not is_skid_steer then return end

        if relay:get(IN4) ~= 0 then
            relay:off(IN4)
        end
        return
    end

    -- ensure IN1 and IN2 are always opposite while armed
    if in1_state == relay:get(IN2) then
        relay:toggle(IN2)
    end

    if not is_skid_steer then return end

    -- ensure IN3 and IN4 are always opposite while armed
    if in3_state == relay:get(IN4) then
        relay:toggle(IN4)
    end
end

function update()
    local in1_current = relay:get(IN1)
    local in3_current = not is_skid_steer and 0 or relay:get(IN3)
    set_input_pins(in1_current, in3_current)
    return update, RUN_INTERVAL_MS
end

gcs:send_text(MAV_SEVERITY.INFO, ('L298N (%s) Driver Initialized'):format(is_ackermann and 'Ackermann' or 'Skid Steer'))

return update()
