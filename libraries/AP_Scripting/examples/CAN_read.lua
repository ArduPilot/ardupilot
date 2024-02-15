-- This script is an example of reading from the CAN bus

-- Load CAN driver1. The first will attach to a protocol of 10, the 2nd to a protocol of 12
-- this allows the script to distinguish packets on two CAN interfaces
local driver1 = CAN:get_device(5)
local driver2 = CAN:get_device2(5)

if not driver1 and not driver2 then
   gcs:send_text(0,"No scripting CAN interfaces found")
   return
end

-- Only accept DroneCAN node status msg on second driver
-- node status is message ID 341
-- Message ID is 16 bits left shifted by 8 in the CAN frame ID.
driver2:add_filter(uint32_t(0xFFFF) << 8, uint32_t(341) << 8)

function show_frame(dnum, frame)
    gcs:send_text(0,string.format("CAN[%u] msg from " .. tostring(frame:id()) .. ": %i, %i, %i, %i, %i, %i, %i, %i", dnum, frame:data(0), frame:data(1), frame:data(2), frame:data(3), frame:data(4), frame:data(5), frame:data(6), frame:data(7)))
end

function update()

   -- see if we got any frames
   if driver1 then
      frame = driver1:read_frame()
      if frame then
         show_frame(1, frame)
      end
   end
   if driver2 then
      frame = driver2:read_frame()
      if frame then
         show_frame(2, frame)
      end
   end

  return update, 10

end

return update()
