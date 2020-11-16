-- This script checks SMBus battery cycle count

local warning_cycles = 100
local battery_instance = 0

function update()
  if not arming:is_armed() then -- only run check when disarmed
    local cycle_count = battery:get_cycle_count(battery_instance)
    if cycle_count then
      if cycle_count >= warning_cycles then
        gcs:send_text(0, string.format("Battery needs replacing (%d cycles)", cycle_count))
      end
    else
      gcs:send_text(0, "failed to get battery cycles")
    end
  end

  return update, 15000 -- check again in 15 seconds
end

return update(), 15000 -- first message may be displayed 15 seconds after start-up
