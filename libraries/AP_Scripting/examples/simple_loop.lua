-- This script is an example of saying hello.  A lot.
-- Pick a random number to send back
local number_float = math.random()
local number_int = math.random(1, 100)

function update() -- this is the loop which periodically runs
  gcs:send_text(0, "hello, world") -- send the traditional message

  gcs:send_named_float('Lua Float',number_float) -- send a float value
  gcs:send_named_float('Lua Int',number_int) -- send an int value
  number_float = number_float +  math.random() -- change the float value

  return update, 1000 -- reschedules the loop
end

return update() -- run immediately before starting to reschedule
