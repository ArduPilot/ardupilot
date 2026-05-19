-- Test call back times from 2^0 to 2^18 (0.001 to 262.144 seconds)
local power = 0
local end_power = 18

-- The error must be less than the larger of the two thresholds
-- This can be (alot) smaller if speedup is not used
local absTheshold = 250
local pctThreshold = 1

local last_run_us

local function update()
  local now_us = micros()

  local expectedDelay = (2^power) / 1000
  local measuredDelay = (now_us - last_run_us):tofloat() / 1000000
  local error_ms = (expectedDelay - measuredDelay) * 1000

  print(string.format("Timing test want: %0.3f s, got: %0.3f s, error: %0.3f ms", expectedDelay, measuredDelay, error_ms))

  local threshold = math.max(absTheshold, expectedDelay * 10 * pctThreshold)
  if math.abs(error_ms) > threshold then
    error(string.format("Large timeing error! (>%0.3fms)", threshold))
  end

  power = power + 1
  if power > end_power then
    print("Timing test passed")
    return
  end

  last_run_us = micros()
  return update, 2^power
end

last_run_us = micros()
return update, 2^power
