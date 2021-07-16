-- This script is an example jitter correction

-- load jitter correction function, 5 seconds max lag
local jitter_correction = JitterCorrection(5000)

local lag = 2500
local jitter_variance = 50
local loop_time = 100

local last_time_stamp = 0
local last_corrected = 0
local last_now = 0

-- https://rosettacode.org/wiki/Statistics/Normal_distribution#Lua
-- random normal distribution
function gaussian (variance)
    return  math.sqrt(-2 * variance * math.log(math.random())) *
            math.cos(2 * math.pi * math.random())
end

function update()

  local now = millis()

  -- Offboard timestamp is jitter free (except the jitter due to scripting callback time), but is lagged
  local offboard_time_stamp = now - lag

  -- onboard time depends on how long the offboard time to too arive this is the jitter
  local onboard_time_stamp = now:tofloat() + math.max(math.floor(gaussian(jitter_variance) + 0.5), -loop_time + 10)

  -- apply jitter correction to the onbard timestamp
  local corrected = jitter_correction:correct_offboard_timestamp_msec(offboard_time_stamp, onboard_time_stamp)

  -- corrected DT should be close the to true
  local corrected_dt = corrected - last_corrected
  last_corrected = corrected
  gcs:send_named_float("COR_DT",corrected_dt:tofloat())

  -- the uncorrected DT
  local raw_dt = onboard_time_stamp - last_time_stamp
  last_time_stamp =  onboard_time_stamp
  gcs:send_named_float("RAW_DT",raw_dt)

  -- true DT due to scripting callback jitter
  local true_dt = now - last_now
  last_now =  now
  gcs:send_named_float("TRUE_DT",true_dt:tofloat())

  return update, loop_time
end

return update, lag * 2
