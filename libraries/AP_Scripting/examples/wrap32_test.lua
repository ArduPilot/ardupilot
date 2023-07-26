--[[
a script to test handling of 32 bit micros timer wrap with BDShot

- Requires bdshot output on SERVO9 to 12
- Requires a wire from one of SERVO9 to 12 to SERVO14 (AUX6, pin 55)
- needs SERVO13_FUNCTION and SERVO14_FUNCTION set to -1
- needs firmware with change for time to start 30s before 32 bit usec wrap and to allow lua reboot
- BRD_SAFETY_DFLT must be 0
- BDShot must be enabled on outputs 9-12
--]]

local PARAM_TABLE_KEY = 138
local PARAM_TABLE_PREFIX = "WRAP32_"

-- setup quicktune specific parameters
assert(param:add_table(PARAM_TABLE_KEY, PARAM_TABLE_PREFIX, 10), 'could not add param table')

-- bind a parameter to a variable
function bind_param(name)
   local p = Parameter()
   assert(p:init(name), string.format('could not find %s parameter', name))
   return p
end

-- add a parameter and bind it to a variable
function bind_add_param(name, idx, default_value)
   assert(param:add_param(PARAM_TABLE_KEY, idx, name, default_value), string.format('could not add param %s', name))
   return bind_param(PARAM_TABLE_PREFIX .. name)
end

local WRAP32_COUNT = bind_add_param('COUNT', 1, 0)
local WRAP32_FAIL = bind_add_param('FAIL', 2, 0)
local WRAP32_ERR = bind_add_param('ERR', 3, 0)
local WRAP32_PIN = bind_add_param('PIN', 4, 55)
local WRAP32_PASS = bind_add_param('PASS', 5, 0)

local wrap_time_ms = uint32_t(0x418937)

--[[
   return number of seconds until we wrap. This will be negative after the wrap
--]]
function time_to_wrap()
   local tnow = millis()
   if tnow < wrap_time_ms then
      return (wrap_time_ms - tnow):tofloat()*0.001
   else
      return -(tnow - wrap_time_ms):tofloat()*0.001
   end
end

function count_changes(wait_ms)
   local start_ms = millis()
   local last_pin_value = 0
   local change_count = 0
   local pin = math.floor(WRAP32_PIN:get())
   local pin_check = pin-1
   gpio:pinMode(pin,0)
   gpio:pinMode(pin_check,1)
   gpio:write(pin-1, 0)
   while millis() - start_ms < wait_ms do
      gpio:toggle(pin_check)
      local v = gpio:read(pin)
      if v ~= last_pin_value then
         change_count = change_count + 1
      end
      last_pin_value = v
   end
   return change_count
end

local done_pre_wrap = false
local done_post_wrap = false

function check_failure()
   local changes = count_changes(100)
   local to_wrap = time_to_wrap()
   gcs:send_text(0,string.format("changes: %u ttw: %.1f", changes, to_wrap))
   if not done_pre_wrap and to_wrap > 5 and to_wrap < 10 then
      -- pre-wrap, should have dshot
      if changes < 4 then
         WRAP32_ERR:set_and_save(WRAP32_ERR:get()+1)
      end
      done_pre_wrap = true
   end
   if not done_post_wrap and to_wrap < -2 and to_wrap > -4 then
      -- post-wrap, should have dshot
      if changes < 4 then
         WRAP32_FAIL:set_and_save(WRAP32_FAIL:get()+1)
      else
         WRAP32_PASS:set_and_save(WRAP32_PASS:get()+1)
      end
      done_post_wrap = true
   end
end

function update()
   check_failure()
   gcs:send_text(0,string.format("Boots:%.0f fail:%.0f err:%.0f pass:%.0f",
                                 WRAP32_COUNT:get(),
                                 WRAP32_FAIL:get(),
                                 WRAP32_ERR:get(),
                                 WRAP32_PASS:get()))
   if done_post_wrap and time_to_wrap() < -6 then
      gcs:send_text(0,string.format("REBOOTING"))
      vehicle:reboot(false)
   end
   return update, 1000
end

-- incremement on boot
WRAP32_COUNT:set_and_save(WRAP32_COUNT:get()+1)

return update()
