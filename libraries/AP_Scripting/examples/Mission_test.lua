-- This script is a test for AP_Mission bindings

local last_mission_index = mission:get_current_nav_index()

function update() -- this is the loop which periodically runs

  -- check for scripting DO commands in the mission
  local time_ms, param1, param2, param3, param4 = mission_receive()
  if time_ms then
    gcs:send_text(0, string.format("Scripting CMD @ %u ms, %i, %0.2f, %0.2f, %0.2f", time_ms:tofloat(), param1, param2, param3, param4))
  end

  local mission_state = mission:state()

  -- make sure the mission is running
  if mission_state == mission.MISSION_COMPLETE then
    gcs:send_text(0, "LUA: Mission Complete")
    return update, 1000 -- reschedules the loop
  elseif mission_state == mission.MISSION_STOPPED then
    gcs:send_text(0, "LUA: Mission stopped")
    return update, 1000 -- reschedules the loop
  end

  local mission_index = mission:get_current_nav_index()

  -- see if we have changed since we last checked
  if mission_index ~= last_mission_index then

    gcs:send_text(0, "LUA: New Mission Item") -- we spotted a change

    -- print the current and previous nav commands
    gcs:send_text(0, string.format("Prev: %d, Current: %d",mission:get_prev_nav_cmd_id(),mission:get_current_nav_id()))

    last_mission_index = mission_index;

    -- num commands includes home so - 1
    local mission_length = mission:num_commands() - 1
    if mission_length > 1 and mission_index == mission_length then
      local jump_to = 1
      if mission_length > 2 then
        -- jump back to a random mission item
        jump_to = math.random(mission_length - 1)  -- no point jump to the end so - 1
      end
      if mission:set_current_cmd(jump_to) then
        gcs:send_text(0, string.format("LUA: jumped to mission item %d",jump_to))
      else
        gcs:send_text(0, "LUA: mission item jump failed")
      end
    end
  end

  return update, 1000 -- reschedules the loop
end

return update() -- run immediately before starting to reschedule
