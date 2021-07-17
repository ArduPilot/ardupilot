local x = 0
local y = 0
local x_inc = 0.1
local y_inc = 0.2

local x_max = 28
local y_max = 15

local screen_tick = 0
local display_screen = true

function update()

  OSD:clear()

  -- this displays the standard AP screens
  if display_screen then
    OSD:draw_screen()
  end
  -- toggle the standard screen
  screen_tick = screen_tick + 1
  if screen_tick > 100 then
    screen_tick = 0
    display_screen = not display_screen
  end

  -- we can also add our own stuff
  OSD:write(math.floor(x+0.5),math.floor(y+0.5),"ARDUPILOT")

  OSD:flush()

  -- bounce the custom text about
  x = x + x_inc
  if x >= x_max then
    x = x_max
    x_inc = -math.abs(x_inc)
  end
  if x < 0 then
    x = 0
    x_inc = math.abs(x_inc)
  end

  y = y + y_inc
  if y >= y_max then
    y = y_max
    y_inc = -math.abs(y_inc)
  end
  if y < 0 then
    y = 0
    y_inc = math.abs(y_inc)
  end

  return update, 100
end

return update()
