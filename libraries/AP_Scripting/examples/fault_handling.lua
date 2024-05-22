--[[
   example script to test fault handling with pcall
--]]

local MAV_SEVERITY = {EMERGENCY=0, ALERT=1, CRITICAL=2, ERROR=3, WARNING=4, NOTICE=5, INFO=6, DEBUG=7}

gcs:send_text(MAV_SEVERITY.INFO, "Loading fault test")

local test_count = 0
local fault_count = 0

--[[
   evaluate a lua function and return nil on fault or the functions return value
--]]
local function evaluate(f)
   local ok, s = pcall(f)
   eval_func = nil
   if ok then
      return s
   end
   fault_count = fault_count + 1
   return nil
end

local function nil_deref()
   local loc = nil
   return loc:lat()
end

local function bad_random()
   return math.random(1,0)
end

local function run_test()
   local script_variant = test_count % 2
   if script_variant == 0 then
      evaluate(nil_deref)
   elseif script_variant == 1 then
      evaluate(bad_random)
   end
end

local function update()
   if test_count % 100 == 0 then
      gcs:send_text(MAV_SEVERITY.INFO,string.format("Test %u fault_count %u", test_count, fault_count))
   end
   test_count = test_count + 1
   run_test()
   assert(fault_count == test_count, "fault and test counts should match")
   return update,1
end

gcs:send_text(MAV_SEVERITY.INFO, "Starting fault test in 2 seconds")

-- wait a while for GCS to connect
return update,2000
