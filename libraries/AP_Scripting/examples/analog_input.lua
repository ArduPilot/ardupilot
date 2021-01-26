-- This script is an example of reading a analog pin

-- load the analog pin
local analog = hal.analog_pin()

analog:set_pin(13) -- typically 13 is the battery input

function update()
  gcs:send_text(0, string.format("voltage: %0.2f", analog:voltage_average()))

  -- analog:voltage_average() the average voltage since the last call
  -- analog:voltage_latest() the latest voltage reading
  -- analog:voltage_average_ratiometric() the average ratiometric voltage (relative to the board 5v)

  return update, 1000
end

return update() -- run immediately before starting to reschedule
