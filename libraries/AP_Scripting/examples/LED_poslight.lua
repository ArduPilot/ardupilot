--[[
 Script to use LED strips as position lights.
 For this script we will use two strips with up to 8 LEDs each.
--]]
local num_leds = 8
local timer = 0

-- Brightness for green or red light.
local br_color = 255

 -- Brightness for flash light when armed.
local br_flash = 255

--[[
 Use SERVOn_FUNCTION 94 for left LED strip 
 Use SERVOn_FUNCTION 95 for right LED strip
--]]
local chan_left = assert(SRV_Channels:find_channel(94),"LEDs left: channel not set")
local chan_right = assert(SRV_Channels:find_channel(95),"LEDs right: channel not set")

-- find_channel returns 0 to 15, convert to 1 to 16
chan_left = chan_left + 1
chan_right = chan_right + 1

gcs:send_text(6, "LEDs strip left: chan=" .. tostring(chan_left))
gcs:send_text(6, "LEDs strip right: chan=" .. tostring(chan_right))

-- initialisation code
assert(serialLED:set_num_neopixel(chan_left, num_leds),"Failed left LED setup")
assert(serialLED:set_num_neopixel(chan_right, num_leds),"Failed right LED setup")
--assert(serialLED:set_num_profiled(chan_left, num_leds),"Failed left LED setup")
--assert(serialLED:set_num_profiled(chan_right, num_leds),"Failed right LED setup")

function update_LEDs()
  if arming:is_armed() then
    if (timer == 0) then
      serialLED:set_RGB(chan_left, -1, br_flash, br_flash, br_flash)
      serialLED:set_RGB(chan_right, -1, br_flash, br_flash, br_flash)
    elseif (timer == 1) then
      serialLED:set_RGB(chan_left, -1, br_color, 0, 0)
      serialLED:set_RGB(chan_right, -1, 0, br_color, 0)
    elseif (timer == 2) then
      serialLED:set_RGB(chan_left, -1, br_flash, br_flash, br_flash)
      serialLED:set_RGB(chan_right, -1, br_flash, br_flash, br_flash)
    elseif (timer == 3) then
      serialLED:set_RGB(chan_left, -1, br_color, 0, 0)
      serialLED:set_RGB(chan_right, -1, 0, br_color, 0)
    end
    timer = timer + 1
    if (timer > 10) then
      timer = 0
    end
  else 
    serialLED:set_RGB(chan_left, -1, br_color, 0, 0)
    serialLED:set_RGB(chan_right, -1, 0, br_color, 0)
    timer = 0
  end
  serialLED:send(chan_left)
  serialLED:send(chan_right)
  return update_LEDs, 100 -- run at 10Hz
end

return update_LEDs()
