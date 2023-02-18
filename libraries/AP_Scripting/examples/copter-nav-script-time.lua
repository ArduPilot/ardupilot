-- Copter perform a simple maneuver in response to a NAV_SCRIPT_TIME mission command while the vehicle is in Auto mode
--
-- Create a simple mission like below
-- TAKEOFF, Alt:10m
-- NAV_SCRIPT_TIME, command=0 (not used in this script), timeout=30 (seconds), arg1=5 (square width in meters), arg2=10 (square height in meters)
-- RETURN-TO-LAUNCH
--
-- Arm the vehicle (in Loiter mode), switch to Auto and raise the throttle
-- The vehicle should climb to 10m
-- This lua script will be fly the vehicle in square pattern in clockwise direction at the current altitude
-- "arg1" specifies the width (e.g. East-West) in meters
-- "arg2" specifies the height (e.g. North-South) in meters
-- Once the vehicle completes the square or the timeout expires the mission will continue and the vehicle should RTL home

local running = false
local last_id = -1          -- unique id used to detect if a new NAV_SCRIPT_TIME command has started
local start_loc             -- vehicle's location when command starts (South-West corner of square)
local target_loc            -- vehicle's target location
local stage = 0             -- stage0: fly North arg2 meters
                            -- stage1: fly East arg1 meters
                            -- stage2: fly South arg2 meters
                            -- stage3: fly West arg2 meters
                            -- stage4: done
local prev_stage = -1       -- previous stage, used to initate call to move to next corner

function update()
  id, cmd, arg1, arg2, arg3, arg4 = vehicle:nav_script_time()
  if id then
    -- handle start of new command
    if id ~= last_id then
      start_loc = ahrs:get_location()       -- initialise south-west corner of square
      if start_loc then
        running = true
        last_id = id
        stage = 0                           -- start heading North
        prev_stage = -1
      else
        gcs:send_text(0, "nav-script-time: failed to get location")
        running = false
      end
    end

    -- set waypoint target according to stage
    if (running and stage ~= prev_stage) then
      prev_stage = stage
      local corner_loc = start_loc:copy()   -- initialise target location to starting location
      if (stage == 0) then
        corner_loc:offset(arg2, 0)          -- North West corner
      end
      if (stage == 1) then
        corner_loc:offset(arg2, arg1)       -- North East corner
      end
      if (stage == 2) then
        corner_loc:offset(0, arg1)          -- South East corner
      end
      -- stage 3 is back to start_loc
      if vehicle:set_target_location(corner_loc) then
        target_loc = corner_loc
      else
        gcs:send_text(0, "nav-script-time: failed to set target")
        vehicle:nav_script_time_done(last_id)
        running = false
      end
    end

    -- advance stage if we have reached within 1m of target
    if running then
      local curr_loc = ahrs:get_location()
      if curr_loc then
        local dist_m = curr_loc:get_distance(target_loc)
        if (dist_m < 1.0) then
          stage = stage + 1
          if stage == 4 then
            vehicle:nav_script_time_done(last_id)
            running = false
          end
        end
      else
        gcs:send_text(0, "nav-script-time: failed to get location")
      end
    end
  else
    -- no active command
    running = false
  end

  return update, 100                        -- update at 10hz
end

return update()
