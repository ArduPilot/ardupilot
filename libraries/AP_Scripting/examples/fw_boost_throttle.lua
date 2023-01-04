--[[
   fixed wing boost throttle support

   This is for aircraft with a 2nd forward thrust motor, such as a
   backup electric motor on an ICE plane
--]]

local MAV_SEVERITY_EMERGENCY = 0
local MAV_SEVERITY_INFO = 6

local PARAM_TABLE_KEY = 84
local PARAM_TABLE_PREFIX = "BOOST_"

local UPDATE_RATE_HZ = 50

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

-- setup parameters
assert(param:add_table(PARAM_TABLE_KEY, PARAM_TABLE_PREFIX, 4), 'could not add param table')

local BOOST_FUNC       = bind_add_param('FUNC',  1, 94)
local BOOST_START      = bind_add_param('START', 2, 70)

local K_THROTTLE = 70

function set_boost(boost)
   local boost_func = BOOST_FUNC:get()
   SRV_Channels:set_range(boost_func, 1000)
   SRV_Channels:set_output_scaled(boost_func, boost)
end

function update()
   if not arming:is_armed() then
      set_boost(0)
      return
   end
   local boost_start = BOOST_START:get()
   local current_throttle = SRV_Channels:get_output_scaled(K_THROTTLE)
   if current_throttle < boost_start then
      set_boost(0)
      return
   end
   local boost = math.floor(1000.0 * (current_throttle - boost_start) / (100.0 - boost_start))
   set_boost(boost)
end

-- wrapper around update(). This calls update() at 10Hz,
-- and if update faults then an error is displayed, but the script is not
-- stopped
function protected_wrapper()
  local success, err = pcall(update)
  if not success then
     gcs:send_text(MAV_SEVERITY_EMERGENCY, "Internal Error: " .. err)
     -- when we fault we run the update function again after 1s, slowing it
     -- down a bit so we don't flood the console with errors
     --return protected_wrapper, 1000
     return
  end
  return protected_wrapper, 1000/UPDATE_RATE_HZ
end

gcs:send_text(MAV_SEVERITY_INFO, "Loaded boost throttle")

-- start running update loop
return protected_wrapper()
