-- This script is a depth failsafe for ArduRover.
-- It monitors the water barometer.
-- If the vehicle goes below the critical depth for a set time, it switches to HOLD mode.

-- depth to trigger failsafe
local CRITICAL_DEPTH = 2.0
local DEBOUNCE_MS = 2000
-- run every 100ms
local UPDATE_RATE_MS = 100

-- Internal variables
local time_below_depth = 0
local failsafe_triggered = false
local ROVER_MODE_HOLD = 4

function update()
  -- If already triggered the failsafe, no need to send messages the Ground Control
  if failsafe_triggered then
    return update, 5000
  end

  -- Read altitude from the barometer
  local current_alt = baro:get_altitude()

  -- If barometer isn't initialized, wait and try again
  if current_alt == nil then
    return update, UPDATE_RATE_MS
  end

  -- Water depth
  local current_depth = -current_alt

  if current_depth >= CRITICAL_DEPTH then
    time_below_depth = time_below_depth + UPDATE_RATE_MS
    
    -- If the rover has been too deep longer than the debounce time
    if time_below_depth >= DEBOUNCE_MS then
        -- Send a Warning text
        gcs:send_text(4, string.format("Depth Failsafe! Depth: %.1fm", current_depth))
        
        -- Switch Rover to HOLD mode
        vehicle:set_mode(ROVER_MODE_HOLD)
        
        -- Lock the failsafe so it doesn't trigger
        failsafe_triggered = true
    end
  else
    -- Safe, Reset the timer
    time_below_depth = 0
  end

  return update, UPDATE_RATE_MS
end

return update()