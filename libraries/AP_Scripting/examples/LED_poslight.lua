--[[
 Script to use LED strips as position lights.
 For this script we will use four strips with up to 10 LEDs each.
--]]
local num_leds = 10

-- Brightness for green or red light.
local br_color = 255

--[[
 Use SERVOn_FUNCTION 94 for left LED strip 
 Use SERVOn_FUNCTION 95 for right LED strip
 Use SERVOn_FUNCTION 96 for back left LED strip 
 Use SERVOn_FUNCTION 97 for back right LED strip
--]]
local chan_left = assert(SRV_Channels:find_channel(94),"LEDs left: channel not set")
local chan_right = assert(SRV_Channels:find_channel(95),"LEDs right: channel not set")
local chan_left_back = assert(SRV_Channels:find_channel(96),"LEDs back left: channel not set")
local chan_right_back = assert(SRV_Channels:find_channel(97),"LEDs back right: channel not set")

-- find_channel returns 0 to 15, convert to 1 to 16
chan_left = chan_left + 1
chan_right = chan_right + 1
chan_left_back = chan_left_back + 1
chan_right_back = chan_right_back + 1

gcs:send_text(6, "LEDs strip left: chan=" .. tostring(chan_left))
gcs:send_text(6, "LEDs strip right: chan=" .. tostring(chan_right))
gcs:send_text(6, "LEDs strip back left: chan=" .. tostring(chan_left_back))
gcs:send_text(6, "LEDs strip back right: chan=" .. tostring(chan_right_back))

-- initialisation code
assert(serialLED:set_num_neopixel(chan_left, num_leds),"Failed left LED setup")
assert(serialLED:set_num_neopixel(chan_right, num_leds),"Failed right LED setup")
assert(serialLED:set_num_neopixel(chan_left_back, num_leds),"Failed back left LED setup")
assert(serialLED:set_num_neopixel(chan_right_back, num_leds),"Failed back right LED setup")

function update_LEDs()

  serialLED:set_RGB(chan_left, -1, br_color, 0, 0)
  serialLED:set_RGB(chan_right, -1, 0, br_color, 0)
  serialLED:set_RGB(chan_left_back, -1, br_color, 0, 0)
  serialLED:set_RGB(chan_right_back, -1, 0, br_color, 0)

  serialLED:send(chan_left)
  serialLED:send(chan_right)
  serialLED:send(chan_left_back)
  serialLED:send(chan_right_back)

  return update_LEDs
end

return update_LEDs()
