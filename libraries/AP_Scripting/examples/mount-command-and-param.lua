-- configure and trigger camera on Phase One P3 Payload for MAVLink (MNT_TYPE = Gremsy)
--
-- state 0: wait and go to state 1
-- state 1: set aperture to 6.3 and go to state 2
-- state 2: trigger capture and go to state 0

local state = 0
local update_period_ms = 5000

-- the main update function
function update()

  if state == 0 then
    state = 1
    -- set ISO to 6400
    mount:send_param_set(0, 100, "CAM_APERTURE", 6.3, 9)
    gcs:send_text(6, "state:" .. tostring(state) .. " set aperture to 6.3")
    return update, update_period_ms
  end

  if state == 1 then
    state = 2
    -- trigger capture
    mount:send_command_long(0, 100, 203, 0, 0, 0, 0, 1, 0, 0)
    gcs:send_text(6, "state:" .. tostring(state) .. " trigger capture")
    return update, update_period_ms
  end

  if state == 2 then
    state = 0
    -- wait
    gcs:send_text(6, "state:" .. tostring(state) .. " wait" )
    return update, update_period_ms
  end

  return update, update_period_ms
end

return update(), update_period_ms
