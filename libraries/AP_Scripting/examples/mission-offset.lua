-- This script offsets the created mission to vehicle's current location
-- Required setup RCx_OPTION = 300 (Scripting1)
-- When this switch is pulled high, move the entire waypoint so that the first waypoint is at the current position
-- Waypoints with lan=0 and lng=0 are ignored

local current_pos -- current position of the vehicle
local rc_option = 300 -- Scripting1
local sw_mission_offset_pos_prev = -1 -- previous position of the switch

function ahrs_ready()
  current_pos = ahrs:get_position()
  if not current_pos then
     return false
  end

  local home = ahrs:get_home()
  if not home then
     return false
  end
  if home:lat() == 0 and home:lng() == 0 then
     return false
  end

  return true
end

function mission_offset() -- run mission offset
  if not ahrs_ready() then
    gcs:send_text(3, "mission-offset.lua: AHRS is not ready")
    return
  end

  local distance_firstwp = 0  -- angle to the first waypoint
  local bearing_firstwp = 0   -- distance to the first waypoint
  local mission_length = mission:num_commands()

  -- read each item but ignore home
  for i = 1, mission_length - 1 do
    local item = mission:get_item(i)
    -- ignore lat:0 lng:0
    if item:x() ~= 0 and item:y() ~= 0 then
      -- remember the first waypoint
      if distance_firstwp == 0 and bearing_firstwp == 0 then
        local firstwp = Location()
        firstwp:lat(item:x())
        firstwp:lng(item:y())
        bearing_firstwp = math.deg(current_pos:get_bearing(firstwp) - math.pi)
        distance_firstwp = current_pos:get_distance(firstwp)
      end

      -- offset waypoint to vehicle position 
      local wp = Location()
      wp:lat(item:x())
      wp:lng(item:y())
      wp:offset_bearing(bearing_firstwp, distance_firstwp)
      item:x(wp:lat())
      item:y(wp:lng())
      mission:set_item(i,item)
    end
  end
  gcs:send_text(6, "mission-offset.lua: offset completed")
end

function is_triggered() -- return true if the switch is pulled high
  local rc_option_mission_offset = assert(rc:find_channel_for_option(rc_option), string.format("mission-offset.lua: RCx_OPTION=%d not set", rc_option))

  local sw_mission_offset_pos = rc_option_mission_offset:get_aux_switch_pos()
  if sw_mission_offset_pos ~= sw_mission_offset_pos_prev then
    sw_mission_offset_pos_prev = sw_mission_offset_pos
    if sw_mission_offset_pos == 2 then
      return true
    end
  end

  return false
end

function update() -- this is the loop which periodically runs
  if is_triggered() then
    mission_offset()
  end

  return update, 1000 -- reschedules the loop
end

return update()
