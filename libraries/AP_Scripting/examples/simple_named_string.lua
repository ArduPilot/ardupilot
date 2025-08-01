-- This script just sends a named value string in a loop...
function update() -- this is the loop which periodically runs
  gcs:send_named_string('Lua String', "Lua String Value") -- send a value
  -- this one is 70 characters long (longer than the field length of Value):
  gcs:send_named_string('Long Lua String', "0123456789012345678901234567890123456789012345678901234567890123456789") -- send a value

  return update, 1000 -- reschedules the loop
end

return update() -- run immediately before starting to reschedule
