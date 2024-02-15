-- This script runs a custom arming check for index == 1 and it must be a takeoff missionn item

local auth_id = arming:get_aux_auth_id()

local MAV_CMD_NAV_TAKEOFF = 22
local MAV_CMD_NAV_VTOL_TAKEOFF = 84

function update() -- this is the loop which periodically runs
  if auth_id then
    local cmd_id = mission:get_current_nav_id()
    local index = mission:get_current_nav_index()

    if not cmd_id or not index then
      arming:set_aux_auth_failed(auth_id, "Could not retrieve mission")
    elseif ((index ~= 0) and (index ~= 1)) then
      -- index of 0 is valid because when you switch to AUTO it will automatically change to 1
      arming:set_aux_auth_failed(auth_id, "Mission index is not ready")
    elseif ((cmd_id ~= MAV_CMD_NAV_TAKEOFF) and (cmd_id ~= MAV_CMD_NAV_VTOL_TAKEOFF)) then
      arming:set_aux_auth_failed(auth_id, "Mission is not ready to takeoff")
    else
      arming:set_aux_auth_passed(auth_id)
    end
  end
  return update, 2000 -- reschedules the loop in 2 seconds
end

return update() -- run immediately before starting to reschedule


