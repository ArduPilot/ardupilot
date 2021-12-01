-- This script is an example of reading from the CAN bus

-- Load CAN driver, using the scripting protocol and with a buffer size of 5
local driver = CAN.get_device(5)

function update()

  -- Read a message from the buffer
  frame = driver:read_frame()

  if frame then
    -- note that we have to be careful to keep the ID as a uint32_t userdata to retain precision
    gcs:send_text(0,string.format("CAN msg from " .. tostring(frame:id()) .. ": %i, %i, %i, %i, %i, %i, %i, %i", frame:data(0), frame:data(1), frame:data(2), frame:data(3), frame:data(4), frame:data(5), frame:data(6), frame:data(7)))
  end

  return update, 100

end

return update()
