-- Test script for finding, starting, stopping and querying scripts

local files
local i = 0

function update()

  -- Print if a file is running or not, out of the original scripts found
  for count = 1, #files do
    if scripting:running(files[count]) then
      gcs:send_text(0, files[count] .. " running")
    else
      gcs:send_text(0, files[count] .. " stopped")
    end
  end

  -- Stop and Start again the simple loop (on sitl)
  -- "/APM/scripts/simple_loop.lua" on real hardware
  if i == 5 then
    scripting:stop("./scripts/simple_loop.lua")
  elseif i == 10 then
    scripting:start("./scripts/simple_loop.lua")
    i = 0
  end
  i = i + 1


  return update, 2000
end


function init()
  -- Print all the scripts found in the given directory
  -- "./scripts" for SITL
  -- '/APM/scripts' and '@ROMFS/scripts' on real hardware
  files = scripting:find_in_dir("./scripts")
  for count = 1, #files do
    gcs:send_text(0, "found: " .. files[count])
  end
  return update, 2000
end

return init(), 5000
