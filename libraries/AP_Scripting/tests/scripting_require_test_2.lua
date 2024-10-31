-- main require tests are in scripting_test.lua

-- DO NOT EDIT!!!! it's very easy to make this accidentally pass even when the
-- original problem is still present!! we do some very careful work to check
-- that require works even when the function's first upvalue is not the script
-- environment.

local loop_time = 500 -- number of ms between runs

-- need to shadow gcs to make the upvalues right
local gcs = gcs -- luacheck: ignore

local passes = 0 -- run both before and after scheduling

local require_global = require("test/nested")

local function update()
   -- need to send before requiring to make the upvalues right
  gcs:send_text(6, "testing")
  local require_local = require("test/nested") -- should not crash

  -- validate we got the same object (object contents validated in main test)
  if require_local == require_global then
    passes = passes + 1
  else
    gcs:send_text(0, "Failed: require returned different objects")
  end
  if passes >= 3 then
    gcs:send_text(3, "Require test 2 passed")
  end

  return update, loop_time
end

return update() -- run immediately before starting to reschedule
