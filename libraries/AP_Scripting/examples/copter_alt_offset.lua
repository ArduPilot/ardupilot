--[[
 add ALT_OFFSET parameter for copter
 This behaves similarly to ALT_OFFSET in plane. It operates only in AUTO mode, and slews the BARO_ALT_OFFSET to allow
 for change of altitude without mission change.
--]]


local PARAM_TABLE_KEY = 79
local PARAM_TABLE_PREFIX = "ALT_"

local MODE_AUTO = 3

local LOOP_RATE = 50

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

-- setup SHIP specific parameters
assert(param:add_table(PARAM_TABLE_KEY, PARAM_TABLE_PREFIX, 2), 'could not add param table')
ALT_OFFSET    = bind_add_param('OFFSET', 1, 0)
ALT_OFFSET_RT = bind_add_param('OFFSET_RT', 2, 1)
BARO_ALT_OFFSET = bind_param("BARO_ALT_OFFSET")

local vehicle_mode = nil

-- constrain a value between limits
function constrain(v, vmin, vmax)
   if v < vmin then
      v = vmin
   end
   if v > vmax then
      v = vmax
   end
   return v
end

-- main update function
function update()
   vehicle_mode = vehicle:get_mode()

   if vehicle_mode ~= MODE_AUTO then
      BARO_ALT_OFFSET:set(0)
      return
   end

   if not arming:is_armed() then
      -- zero offset when disarmed
      BARO_ALT_OFFSET:set(0)
      ALT_OFFSET:set(0)
   end
   
   local target = ALT_OFFSET:get() * -1.0

   local current = BARO_ALT_OFFSET:get()
   local delta = target - current
   local dt = 1.0 / LOOP_RATE
   local delta_max = ALT_OFFSET_RT:get() * dt

   delta = constrain(delta, -delta_max, delta_max)
   BARO_ALT_OFFSET:set(current + delta)
end

-- wrapper around update(). This calls update() at 20Hz,
-- and if update faults then an error is displayed, but the script is not
-- stopped
function protected_wrapper()
  local success, err = pcall(update)
  if not success then
     gcs:send_text(0, "Internal Error: " .. err)
     -- when we fault we run the update function again after 1s, slowing it
     -- down a bit so we don't flood the console with errors
     return protected_wrapper, 1000
  end
  return protected_wrapper, 1000/LOOP_RATE
end

gcs:send_text(0, "ALT_OFFSET handler loaded")

-- start running update loop
return protected_wrapper()

