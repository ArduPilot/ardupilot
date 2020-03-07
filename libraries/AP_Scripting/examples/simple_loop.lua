-- This script is an example of saying hello.  A lot.
-- Pick a random number to send back
local number = math.random()

function update() -- this is the loop which periodically runs
  gcs:send_text(0, "hello, world") -- send the traditional message

  gcs:send_named_float('Lua Float',number) -- send a value
  number = number +  math.random() -- change the value

  return update, 1000 -- reschedules the loop
end

return update() -- run immediately before starting to reschedule
