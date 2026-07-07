-- sub_test_set_target_velocity_NED.lua
-- Test helper for the ArduSub vehicle:set_target_velocity_NED() scripting
-- binding. A level angle target is sent first so the guided angle controller is
-- running, then the commanded velocity is read from a dedicated parameter table
-- so the autotest can drive it at runtime:
--   VELNED_FWD_MPS   = forward velocity (m/s)
--   VELNED_RIGHT_MPS = lateral velocity (m/s, positive = right)
--   VELNED_DOWN_MPS  = vertical velocity (m/s, positive = descend)

local UPDATE_PERIOD_MS = 50

-- the table key must be unique to this script on the flight controller (see param_add.lua)
-- the full parameter name (prefix + suffix) is limited to 16 characters
local PARAM_TABLE_KEY = 102
assert(param:add_table(PARAM_TABLE_KEY, "VELNED_", 3), "could not add param table")
assert(param:add_param(PARAM_TABLE_KEY, 1, "FWD_MPS", 0), "could not add VELNED_FWD_MPS")
assert(param:add_param(PARAM_TABLE_KEY, 2, "RIGHT_MPS", 0), "could not add VELNED_RIGHT_MPS")
assert(param:add_param(PARAM_TABLE_KEY, 3, "DOWN_MPS", 0), "could not add VELNED_DOWN_MPS")

local VELNED_FWD_MPS = Parameter("VELNED_FWD_MPS")
local VELNED_RIGHT_MPS = Parameter("VELNED_RIGHT_MPS")
local VELNED_DOWN_MPS = Parameter("VELNED_DOWN_MPS")

local function update()
    -- ensure the guided angle controller is active and level
    vehicle:set_target_angle_and_climbrate(0, 0, 0, 0, false, 0)
    local vel = Vector3f()
    vel:x(VELNED_FWD_MPS:get() or 0)
    vel:y(VELNED_RIGHT_MPS:get() or 0)
    vel:z(VELNED_DOWN_MPS:get() or 0)
    vehicle:set_target_velocity_NED(vel)
    return update, UPDATE_PERIOD_MS
end

gcs:send_text(6, "set_target_velocity_NED test ready")

return update, UPDATE_PERIOD_MS
