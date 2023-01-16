-- This script is an example of reading from the CAN bus

-- Load CAN driver1. The first will attach to a protocol of 10, the 2nd to a protocol of 12
-- this allows the script to distinguish packets on two CAN interfaces
local driver1 = CAN:get_device(25)
local driver2 = CAN:get_device2(25)

if not driver1 and not driver2 then
   gcs:send_text(0,"No scripting CAN interfaces found")
   return
end

function show_frame(dnum, frame)
    --if uint32_t(frame:id()) == uint32_t(0x222) then
      local is_ext = 0
      if frame:isExtended() then
        is_ext = 1
      end
      gcs:send_text(0,string.format("C%u fi:" .. tostring(frame:id()) .. " dl:%u x:%d %x %x %x %x %x %x %x %x", dnum, frame:dlc(), is_ext, frame:data(0), frame:data(1), frame:data(2), frame:data(3), frame:data(4), frame:data(5), frame:data(6), frame:data(7)))
    --end
end

function update()

   -- see if we got any frames
   if driver1 then
      local frame
      repeat
        frame = driver1:read_frame()
        if frame then
          show_frame(1, frame)
        end
      until not frame
   else
     gcs:send_text(0,"CAN_read: drive1 is null")
   end

  return update, 10

end

return update()
