-- This script is an example of manipulating the message stream rates
--
-- It will periodically run and adjust all the messages to their desired loop rates
-- It can be modified to only run once, however some GCS's will manipulate the rates
-- during the initial connection, so by resetting them periodically we ensure we get
-- the expected telemetry rate

-- intervals is a table which contains a table for each entry we want to adjust
-- each entry is arranged as {the serial link to adjust, the mavlink message ID, and the broadcast interval in Hz}
local intervals = {{0, uint32_t(30), 2.0},  -- First serial, ATTITUDE, 2Hz
                   {0, uint32_t(163), 5.0}} -- First serial, AHRS, 5Hz

local loop_time = 5000 -- number of ms between runs

function update() -- this is the loop which periodically runs
  for i = 1, #intervals do -- we want to iterate over every specified interval
    local channel, message, interval_hz = table.unpack(intervals[i]) -- this extracts the channel, MAVLink ID, and interval
    gcs:set_message_interval(channel, message, math.floor(1000000 / interval_hz)) -- actually sets the interval as appropriate
  end
  return update, loop_time -- reschedules the loop
end

return update() -- run immediately before starting to reschedule
