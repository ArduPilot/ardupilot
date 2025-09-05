-- hello.lua
-- Simple "Hello World" script for ArduPilot Plane

function update()
    -- Send a message to the GCS console
    gcs:send_text(6, "Hello, world from Lua on Plane!")
    
    -- Run this function again in 1000 ms (1 second)
    return update, 1000
end

-- Start the update loop
return update, 1000
