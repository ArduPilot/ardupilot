-- this script is a example of loading a 'libary' sub - script

local lib = load_script("./scripts/simple_libary.lual")

function update() -- this is the loop which periodically runs

  -- print the number stored in the lib
  gcs:send_text(0, "lib ".. tostring(lib.number)) 

  -- run a function in the lib
  lib.update()

  return update, 1000 -- reschedules the loop
end

return update() -- run immediately before starting to reschedule
