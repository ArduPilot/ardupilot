--[[
   This script reads the firmware, calculates a checksum and emits a text message.

   It does this 10 seconds after boot.  And any time millis() wraps.
--]]

local flash_file_path = "@SYS/flash.bin"

function update()
   -- wait until we've been booted 10 seconds before sending:
   local now = millis()
   if now < 10000 then
      -- not time yet
      return update, 100
   end

   local result = fs:crc32(flash_file_path)
   if result == nil then
      gcs:send_text(3, string.format("checksum_firmware error: checksumming failed"))
   else
      gcs:send_text(3, string.format("checksum_firmware: 0x%x", result:toint()))
   end

   return  -- this script is one-shot 
end

return update()
