-- This script displays ESC usage time (supported only by ToshibaCAN ESCs)

local usage_hours_max = 100 -- maximum safe usage for these ESCs in hours
local usage_sec_max = usage_hours_max*60*60

function update() -- this is the loop which periodically runs
  if not arming:is_armed() then -- only run check when disarmed
    local got_esc_usage = false
    local esc_over_max = false
    for i = 0, 3 do -- check first four ESCs
      local usage_sec = esc_telem:get_usage_seconds(i)
      if usage_sec > 0 then
        got_esc_usage = true
      end
      if usage_sec > usage_sec_max then
          esc_over_max = true
          local usage_hours = usage_sec / 3600
          local usage_minutes = (usage_sec / 60) % 60
          gcs:send_text(0, string.format("ESC" .. tostring(i) .. ": " .. tostring(usage_hours) .. "hrs " .. tostring(usage_minutes) .. "min (limit is " .. tostring(usage_hours_max) .. "hrs)"))
      end
    end
    if not got_esc_usage then
      gcs:send_text(0, "Could not retrieve ESC usage time")
    elseif not esc_over_max then
      gcs:send_text(0, string.format("ESC usage time OK (under " .. tostring(usage_hours_max) .. "hrs limit)"))
    end
  end
  return update, 5000 -- reschedules the loop in 5 seconds
end

return update() -- run immediately before starting to reschedule
