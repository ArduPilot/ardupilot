-- main require tests are in scripting_test.lua

-- CAREFUL WHEN EDITING!!!! we do some very careful work to check that require
-- works even when the update function's first upvalue is not the script
-- environment _ENV. this fact can be verified by looking at its upvalues using
-- e.g. https://www.luac.nl/

local loop_time = 500 -- number of ms between runs

-- alias global gcs as a local so update uses it as an upvalue
local gcs = gcs -- luacheck: ignore

local passes = 0 -- run both before and after scheduling

-- this time running require, the main function that implicitly wraps the script
-- is the update function and its only upvalue is by definition _ENV; require
-- is expected to work here
local require_global = require("test/nested")

local function update()
  -- reference gcs first so it's update's first upvalue
  gcs:send_text(6, "testing")
  -- require is a global, so referencing it implicitly adds _ENV as update's
  -- second upvalue, thus exercising the problem during the second and third
  -- passes when this is in fact the update function
  local require_local = require("test/nested") -- should not cause an error

  -- validate we got the same object (object contents validated in main test)
  -- no matter when require is called and what set of upvalues are used
  if require_local == require_global then
    passes = passes + 1
  else
    gcs:send_text(0, "Failed: require returned different objects")
  end
  if passes >= 3 then
    gcs:send_text(3, "Require test 2 passed")
  end

  -- now schedule this function as the update function, not the main function
  return update, loop_time
end

-- run immediately before starting to reschedule. the update function doesn't
-- change until the return, so the first time it's run the bug shouldn't trigger
-- as the main function's upvalues are still the ones checked
return update()
