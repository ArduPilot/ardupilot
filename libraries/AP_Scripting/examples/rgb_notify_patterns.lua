-- This script is a test of led override

local count = 0
local num_leds = 16

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
function set_Rainbow(led, v)
  local num_rows = #rainbow
  local row = math.floor(constrain(v * (num_rows-1)+1, 1, num_rows-1))
  local v0 = (row-1) / (num_rows-1)
  local v1 = row / (num_rows-1)
  local p = (v - v0) / (v1 - v0)
  r = math.floor(rainbow[row][1] + p * (rainbow[row+1][1] - rainbow[row][1]))
  g = math.floor(rainbow[row][2] + p * (rainbow[row+1][2] - rainbow[row][2]))
  b = math.floor(rainbow[row][3] + p * (rainbow[row+1][3] - rainbow[row][3]))
  notify:handle_rgb_id(r, g, b, led)
end

function update() -- this is the loop which periodically runs
  count = count + 1
  if count > 16 then
    count = 0
  end
  for led = 0, num_leds-1 do
    local v  = ((count+led)%16)/16
    set_Rainbow(led, v)
  end
  return update, 20 -- reschedules the loop in 15 seconds

end

return update() -- run immediately before starting to reschedule
