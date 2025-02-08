-- Example showing how a position offset can be added to a Copter's auto mission plan
--
-- CAUTION: This script only works for Copter
-- this script waits for the vehicle to be armed and in auto mode and then
-- adds an offset to the position or velocity target
--
-- How To Use
-- 1. copy this script to the autopilot's "scripts" directory
-- 2. set PSC_OFS_xx parameter to the desired position OR velocity offsets desired
-- 3. fly the vehicle in Auto mode (or almost any mode)
-- 4. use the Scripting1 aux switch to enable and disable the offsets

local MAV_SEVERITY = {EMERGENCY=0, ALERT=1, CRITICAL=2, ERROR=3, WARNING=4, NOTICE=5, INFO=6, DEBUG=7}

-- setup script specific parameters
local PARAM_TABLE_KEY = 71
local PARAM_TABLE_PREFIX = "PSC_OFS_"
assert(param:add_table(PARAM_TABLE_KEY, PARAM_TABLE_PREFIX, 6), 'could not add param table')
 
-- add a parameter and bind it to a variable
function bind_add_param(name, idx, default_value)
  assert(param:add_param(PARAM_TABLE_KEY, idx, name, default_value), string.format('could not add param %s', PARAM_TABLE_PREFIX .. name))
  return Parameter(PARAM_TABLE_PREFIX .. name)
end

--[[
  // @Param: PSC_OFS_POS_N
  // @DisplayName: Position Controller Position Offset North
  // @Description: Position controller offset North
  // @Range: 0 1000
  // @Units: m
  // @User: Standard
--]]
local PSC_OFS_POS_N = bind_add_param("POS_N", 1, 0)

--[[
  // @Param: PSC_OFS_POS_E
  // @DisplayName: Position Controller Position Offset East
  // @Description: Position controller offset North
  // @Range: 0 1000
  // @Units: m
  // @User: Standard
--]]
local PSC_OFS_POS_E = bind_add_param("POS_E", 2, 0)

--[[
  // @Param: PSC_OFS_POS_D
  // @DisplayName: Position Controller Position Offset Down
  // @Description: Position controller offset Down
  // @Range: 0 1000
  // @Units: m
  // @User: Standard
--]]
local PSC_OFS_POS_D = bind_add_param("POS_D", 3, 0)

--[[
  // @Param: PSC_OFS_VEL_N
  // @DisplayName: Position Controller Velocity Offset North
  // @Description: Position controller velocity offset North
  // @Range: 0 10
  // @Units: m/s
  // @User: Standard
--]]
local PSC_OFS_VEL_N = bind_add_param("VEL_N", 4, 0)

--[[
  // @Param: PSC_OFS_VEL_E
  // @DisplayName: Position Controller Velocity Offset East
  // @Description: Position controller velocity offset East
  // @Range: 0 10
  // @Units: m/s
  // @User: Standard
--]]
local PSC_OFS_VEL_E = bind_add_param("VEL_E", 5, 0)

--[[
  // @Param: PSC_OFS_VEL_D
  // @DisplayName: Position Controller Velocity Offset Down
  // @Description: Position controller velocity offset Down
  // @Range: 0 10
  // @Units: m/s
  // @User: Standard
--]]
local PSC_OFS_VEL_D = bind_add_param("VEL_D", 6, 0)

-- local variables
local aux_sw_pos_last = 0

-- welcome message to user
gcs:send_text(MAV_SEVERITY.INFO, "copter-posoffset.lua loaded")

function update()

  -- must be armed, flying and in auto mode
  if (not arming:is_armed()) or (not vehicle:get_likely_flying()) then
    return update, 1000
  end

  -- Scripting1 aux switch enables/disables offsets
  local aux_sw_pos = rc:get_aux_cached(300)
  if aux_sw_pos == nil then
    aux_sw_pos = 0
  end

  -- check for change in aux switch position
  if aux_sw_pos ~= aux_sw_pos_last then
    aux_sw_pos_last = aux_sw_pos
    if aux_sw_pos == 0 then
      -- if switch is in low position clear offsets
      if poscontrol:set_posvelaccel_offset(Vector3f(), Vector3f(), Vector3f()) then
        gcs:send_text(MAV_SEVERITY.ERROR, "copter-posoffset: offsets cleared")
        return update, 1000
      else
        gcs:send_text(MAV_SEVERITY.ERROR, "copter-posoffset: failed to clear offsets")
      end
    else
      gcs:send_text(MAV_SEVERITY.ERROR, "copter-posoffset: offsets activated")
    end
  end

  if aux_sw_pos == 0 then
    return update, 1000
  end

  local pos_offsets_zero = PSC_OFS_POS_N:get() == 0 and PSC_OFS_POS_E:get() == 0 and PSC_OFS_POS_D:get() == 0
  local vel_offsets_zero = PSC_OFS_VEL_N:get() == 0 and PSC_OFS_VEL_E:get() == 0 and PSC_OFS_VEL_D:get() == 0

  -- if position offsets are non-zero or all offsets are zero then send position offsets
  if not pos_offsets_zero or vel_offsets_zero then
    -- set the position offset in meters in NED frame
    local pos_offset_NED = Vector3f()
    pos_offset_NED:x(PSC_OFS_POS_N:get())
    pos_offset_NED:y(PSC_OFS_POS_E:get())
    pos_offset_NED:z(PSC_OFS_POS_D:get())
    if not poscontrol:set_posvelaccel_offset(pos_offset_NED, Vector3f(), Vector3f()) then
      gcs:send_text(MAV_SEVERITY.ERROR, "copter-posoffset: failed to set pos offset")
    end
    test_position = not pos_offsets_zero
  else
    -- first get position offset (cumulative effect of velocity offsets)
    local pos_offset_NED, _, _ = poscontrol:get_posvelaccel_offset()
    if pos_offset_NED == nil then
        print("unable to get position offset")
        pos_offset_NED = Vector3f()
    end
    -- test velocity offsets in m/s in NED frame
    local vel_offset_NED = Vector3f()
    vel_offset_NED:x(PSC_OFS_VEL_N:get())
    vel_offset_NED:y(PSC_OFS_VEL_E:get())
    vel_offset_NED:z(PSC_OFS_VEL_D:get())
    if not poscontrol:set_posvelaccel_offset(pos_offset_NED, vel_offset_NED, Vector3f()) then
      gcs:send_text(MAV_SEVERITY.ERROR, "copter-posoffset: failed to set vel offset")
    end
  end

  -- update at 1hz
  return update, 1000
end

return update()
