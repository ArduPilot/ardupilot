--[[
Script to control LED strips based on the roll of the aircraft. This is an example to demonstrate
the LED interface for WS2812 LEDs
--]]


--[[
for this demo we will use a single strip with 30 LEDs
--]]
local num_leds = 30

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

gcs:send_text(6, "LEDs: chan=" .. tostring(chan))

-- initialisation code
--serialLED:set_num_neopixel(chan,  num_leds)
serialLED:set_num_profiled(chan,  num_leds)

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
local function set_Rainbow(channel, led, v)
  local num_rows = #rainbow
  local row = math.floor(constrain(v * (num_rows-1)+1, 1, num_rows-1))
  local v0 = (row-1) / (num_rows-1)
  local v1 = row / (num_rows-1)
  local p = (v - v0) / (v1 - v0)
  r = math.floor(rainbow[row][1] + p * (rainbow[row+1][1] - rainbow[row][1]))
  g = math.floor(rainbow[row][2] + p * (rainbow[row+1][2] - rainbow[row][2]))
  b = math.floor(rainbow[row][3] + p * (rainbow[row+1][3] - rainbow[row][3]))
  serialLED:set_RGB(channel, led, r, g, b)
end

--[[
We will set the colour of the LEDs based on roll of the aircraft
--]]
function update_LEDs()
  local roll = constrain(ahrs:get_roll_rad(), math.rad(-60), math.rad(60))

  for led = 0, num_leds-1 do
    local v  = constrain(0.5 + 0.5 * math.sin(roll * (led - num_leds/2) / (num_leds/2)), 0, 1)
    set_Rainbow(chan, led, v)
  end
  serialLED:send(chan)

  return update_LEDs, 20 -- run at 50Hz
end

return update_LEDs, 1000

