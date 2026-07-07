-- sub_test_set_target_angle_and_climbrate.lua
-- Test helper for the ArduSub vehicle:set_target_angle_and_climbrate() scripting
-- binding. The commanded attitude and climb rate are read from a dedicated
-- parameter table so the autotest can drive them at runtime:
--   ANGLE_ROLL_DEG  = roll angle (deg)
--   ANGLE_PITCH_DEG = pitch angle (deg)
--   ANGLE_CLIMB_MPS = climb rate (m/s, positive = ascend)

local UPDATE_PERIOD_MS = 50

-- the table key must be unique to this script on the flight controller (see param_add.lua)
-- the full parameter name (prefix + suffix) is limited to 16 characters
local PARAM_TABLE_KEY = 101
assert(param:add_table(PARAM_TABLE_KEY, "ANGLE_", 3), "could not add param table")
assert(param:add_param(PARAM_TABLE_KEY, 1, "ROLL_DEG", 0), "could not add ANGLE_ROLL_DEG")
assert(param:add_param(PARAM_TABLE_KEY, 2, "PITCH_DEG", 0), "could not add ANGLE_PITCH_DEG")
assert(param:add_param(PARAM_TABLE_KEY, 3, "CLIMB_MPS", 0), "could not add ANGLE_CLIMB_MPS")

local ANGLE_ROLL_DEG = Parameter("ANGLE_ROLL_DEG")
local ANGLE_PITCH_DEG = Parameter("ANGLE_PITCH_DEG")
local ANGLE_CLIMB_MPS = Parameter("ANGLE_CLIMB_MPS")

local function update()
    vehicle:set_target_angle_and_climbrate(ANGLE_ROLL_DEG:get() or 0,
                                           ANGLE_PITCH_DEG:get() or 0,
                                           0,
                                           ANGLE_CLIMB_MPS:get() or 0,
                                           false, 0)
    return update, UPDATE_PERIOD_MS
end

gcs:send_text(6, "set_target_angle_and_climbrate test ready")

return update, UPDATE_PERIOD_MS
