-- camera-test.lua.  Tests triggering taking pictures at regular intervals

-- global definitions
local MAV_SEVERITY = {EMERGENCY=0, ALERT=1, CRITICAL=2, ERROR=3, WARNING=4, NOTICE=5, INFO=6, DEBUG=7}
local TAKE_PIC_INTERVAL_MS = 5000   -- take pictures at this interval
local CAMERA_INSTANCE = 0           -- always control the first camera

-- local variables
local last_takepic_time_ms = 0      -- system time that picture was last taken

-- the main update function that performs a simplified version of RTL
function update()

  -- get current system time
  local now_ms = millis()

  -- check if time to take picture
  if (now_ms - last_takepic_time_ms > TAKE_PIC_INTERVAL_MS) then
    last_takepic_time_ms = now_ms
    camera:take_picture(CAMERA_INSTANCE)
  end

  -- update at 10hz
  return update, 100
end

-- display startup message
gcs:send_text(MAV_SEVERITY.INFO, "camera-test.lua started")

return update()
