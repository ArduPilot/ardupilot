--[[
   example demonstrating how to read from a serial port into a lua string
--]]

local baud_rate = 57600

local port = assert(serial:find_serial(0), "Could not find Scripting Serial Port")

port:begin(baud_rate)
port:set_flow_control(0)

--[[
  get a string by concatenating bytes from the serial port,
  suitable for ArduPilot 4.4.x
--]]
local function get_string_44(n)
   local ret = ""
   for _ = 1, n do
      local b = port:read()
      ret = ret .. string.char(b)
   end
   return ret
end

--[[
  get a string directly uisng readstring
  suitable for ArduPilot 4.5.x and later
--]]
local function get_string_45(n)
   return port:readstring(n)
end

function update()
   -- test using 4.5 method
   local n = port:available():toint()
   if n > 0 then
      local str = get_string_45(n)
      gcs:send_text(0, string.format("Received: '%s'", str))
   end

   -- test using 4.4 method (just so we don't have an unused function in lua check)
   n = port:available():toint()
   if n > 0 then
      local str = get_string_44(n)
      gcs:send_text(0, string.format("Received: '%s'", str))
    end
   return update, 100
end

return update, 100
