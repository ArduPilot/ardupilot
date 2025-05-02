--[[
script to send a 'rainbow snake' around a long set of LEDs on a single pin
--]]

local num_leds = 60
local led_base = 0
local snake_len = 5
local brightness = 1

--[[
 use SERVOn_FUNCTION 94 for LED. We can control up to 16 separate strips of LEDs
 by putting them on different channels
--]]
local chan = SRV_Channels:find_channel(94)

if not chan then
    gcs:send_text(6, "LEDs: channel not set")
    return
end

-- find_channel returns 0 to 15, convert to 1 to 16
chan = chan + 1

gcs:send_text(6, "LEDs: chan=" .. tostring(chan) .. " nLEDs=" .. tostring(num_leds))

-- initialisation code
serialLED:set_num_LEDs(chan,  num_leds + led_base)

-- constrain a value between limits
function constrain(v, vmin, vmax)
   if v < vmin then
      v = vmin
   end
   if v > vmax then
      v = vmax
   end
   return v
end

--[[
Table of colors on a rainbow, red first
--]]
local rainbow = {
  { 255, 0, 0 },
  { 255, 127, 0 },
  { 255, 255, 0 },
  { 0,   255, 0 },
  { 0,   0,   255 },
  { 75,  0,   130 },
  { 143, 0,   255 },
}

--[[
Function to set a LED to a color on a classic rainbow spectrum, with v=0 giving red
--]]
function set_Rainbow(chan, led, v)
  local num_rows = #rainbow
  local row = math.floor(constrain(v * (num_rows-1)+1, 1, num_rows-1))
  local v0 = (row-1) / (num_rows-1)
  local v1 = row / (num_rows-1)
  local p = (v - v0) / (v1 - v0)
  r = math.floor((rainbow[row][1] + p * (rainbow[row+1][1] - rainbow[row][1])) * brightness)
  g = math.floor((rainbow[row][2] + p * (rainbow[row+1][2] - rainbow[row][2])) * brightness)
  b = math.floor((rainbow[row][3] + p * (rainbow[row+1][3] - rainbow[row][3])) * brightness)
  serialLED:set_RGB(chan, led, r, g, b)
end


--[[
Display a rainbow snake around the LEDs
--]]
function display_snake(start)
  increment = 1.0 / snake_len
  v = 0
  serialLED:set_RGB(chan, -1, 0, 0, 0)
  for i = 0, snake_len-1 do
      led = (i + start) % num_leds
      set_Rainbow(chan, led + led_base, v)
      v = v + increment
  end
end

next = 0

--[[
We will set the colour of the LEDs based on roll of the aircraft
--]]
function update_LEDs()
  serialLED:set_RGB(chan, -1, 0, 0, 0)
  display_snake(next)
  serialLED:send()
  next = (next + 1) % num_leds

  return update_LEDs, 200 -- run at 5Hz
end

return update_LEDs, 1000
