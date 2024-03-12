--[[
 test the load function for loading new code from strings
--]]

gcs:send_text(0,"Testing load() method")

-- a function written as a string. This could come from a file
-- or any other source (eg. mavlink)
-- Note that the [[ xxx ]] syntax is just a multi-line string
-- luacheck: only 0

local func_str = [[
function TestFunc(x,y)
  return math.sin(x) + math.cos(y)
end
]]

function test_load()
   -- load the code into the global environment
   local f,errloc,err = load(func_str,"TestFunc", "t", _ENV)
   if not f then
      gcs:send_text(0,string.format("Error %s: %s", errloc, err))
      return
   end
   -- run the code within a protected call to catch any errors
   local success, err = pcall(f)
   if not success then
      gcs:send_text(0, string.format("Failed to load TestFunc: %s", err))
      return
   end

   -- we now have the new function
   gcs:send_text(0, string.format("TestFunc(3,4) -> %f", TestFunc(3,4)))
end

test_load()
