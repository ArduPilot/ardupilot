-- This script is an example of saying hello
function update() -- this is the loop which periodically runs

    gcs:send_text(0, "hello from scripting on esp32") -- send the traditional message

  return update, 5000 -- reschedules the loop 
end

return update() -- run immediately before starting to reschedule