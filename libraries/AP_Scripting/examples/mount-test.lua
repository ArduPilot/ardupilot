-- moves 3-axis gimbal (aka "mount") using earth-frame and body-frame rates and angles
--
-- stage 0: move gimbal to neutral position
-- stage 1: yaw CW at 10deg/s in body-frame
-- stage 2: yaw CCW at 10 deg/s in body-frame
-- stage 3: pitch at 10 deg/s in body-frame
-- stage 4: pitch at -10 deg/s in body-frame
-- stage 5: roll at 10 deg/s in body-frame
-- stage 6: roll at -10 deg/s in body-frame
-- stage 7: point North
-- stage 8: point South and center
-- stage 9: point North and Down
-- stage 10: move angle to neutral position

---@diagnostic disable: cast-local-type

local stage = 0
local stage_time_ms = 5000
local stage_start_time_ms = 0
local last_stage = 10

-- the main update function that performs a simplified version of RTL
function update()

  -- get current system time
  local now_ms = millis()

  -- start
  if stage_start_time_ms == 0 then
    stage_start_time_ms = now_ms
  end

  -- check if time to move to next stage
  local update_user = false
  if (now_ms - stage_start_time_ms > stage_time_ms) and (stage < last_stage) then
    stage = stage + 1
    stage_start_time_ms = now_ms
    update_user = true
  end

  if stage == 0 or stage >= last_stage then
    -- move angle to neutral position
    mount:set_angle_target(0, 0, 0, 0, false)
    if update_user then
      gcs:send_text(6, "stage:" .. tostring(stage) .. " move to neutral position")
    end
  end

  if stage == 1 then
    -- yaw CW at 10deg/s in body-frame
    mount:set_rate_target(0, 0, 0, 10, false)
    if update_user then
      gcs:send_text(6, "stage:" .. tostring(stage) .. " yaw at 10deg/s")
    end
  end

  if stage == 2 then
    -- yaw CCW at 10 deg/s in body-frame
    mount:set_rate_target(0, 0, 0, -10, false)
    if update_user then
      gcs:send_text(6, "stage:" .. tostring(stage) .. " yaw at -10deg/s")
    end
  end

  if stage == 3 then
    -- pitch at 10 deg/s in body-frame
    mount:set_rate_target(0, 0, 10, 0, false)
    if update_user then
      gcs:send_text(6, "stage:" .. tostring(stage) .. " pitch at 10deg/s")
    end
  end

  if stage == 4 then
    -- pitch at -10 deg/s in body-frame
    mount:set_rate_target(0, 0, -10, 0, false)
    if update_user then
      gcs:send_text(6, "stage:" .. tostring(stage) .. " pitch at -10deg/s")
    end
  end

  if stage == 5 then
    -- roll at 10 deg/s in body-frame
    mount:set_rate_target(0, 10, 0, 0, false)
    if update_user then
      gcs:send_text(6, "stage:" .. tostring(stage) .. " roll at 10deg/s")
    end
  end

  if stage == 6 then
    -- roll at -10 deg/s in body-frame
    mount:set_rate_target(0, -10, 0, 0, false)
    if update_user then
      gcs:send_text(6, "stage:" .. tostring(stage) .. " roll at -10deg/s")
    end
  end

  if stage == 7 then
    -- point North
    mount:set_angle_target(0, 0, 0, 0, true)
    if update_user then
      gcs:send_text(6, "stage:" .. tostring(stage) .. " point North")
    end
  end

  if stage == 8 then
    -- point South and center
    mount:set_angle_target(0, 0, 0, 180, true)
    if update_user then
      gcs:send_text(6, "stage:" .. tostring(stage) .. " point South")
    end
  end

  if stage == 9 then
    -- point North and Down
    mount:set_angle_target(0, 0, -90, 0, false)
    if update_user then
      gcs:send_text(6, "stage:" .. tostring(stage) .. " point Down")
    end
  end

  if stage > last_stage then
    -- move angle to neutral position
    mount:set_angle_target(0, 0, 0, 0, false)
    if update_user then
      gcs:send_text(6, "stage:" .. tostring(stage) .. " done!")
    end
  end

  return update, 100
end

return update()
