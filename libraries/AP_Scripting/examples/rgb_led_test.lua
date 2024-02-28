-- This script is a test of led override

local count = 0

function update() -- this is the loop which periodically runs

  count = count + 1

  if count == 1 then
    -- solid red
    -- red,green,blue,rate hz, duration ms
    notify:handle_rgb(255,0,0,0)
  elseif count == 2 then
    -- solid green
    notify:handle_rgb(0,255,0,0)
  elseif count == 3 then
    -- solid blue
    notify:handle_rgb(0,0,255,0)
  elseif count == 4 then
    -- 1hz red  + green
    notify:handle_rgb(255,255,0,1)
  elseif count == 5 then
    -- 1hz green + blue
    notify:handle_rgb(0,255,255,1)
  elseif count == 6 then
    -- 1hz red + blue
    notify:handle_rgb(255,0,255,1)
  elseif count == 7 then
    -- fast white
    notify:handle_rgb(255,255,255,10)
    count = 0
  end

  return update, 15000 -- reschedules the loop in 15 seconds

end

return update() -- run immediately before starting to reschedule
