--[[
   script to auto-deploy a vehicle on descent after reaching a specified altitude
   uses raw pressure to not depend on either GPS or on home alt
--]]
local PARAM_TABLE_KEY = 72
local PARAM_TABLE_PREFIX = "DEPL_"
local MAV_SEVERITY = {EMERGENCY=0, ALERT=1, CRITICAL=2, ERROR=3, WARNING=4, NOTICE=5, INFO=6, DEBUG=7}
---@diagnostic disable: param-type-mismatch

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

assert(param:add_table(PARAM_TABLE_KEY, PARAM_TABLE_PREFIX, 10), 'could not add param table')

local end_pos    = bind_add_param('OPEN_POS',    1, 1200)
local offset     = bind_add_param('OPEN_OFFSET', 2, 400)
local deployment_alt = bind_add_param('ALT', 3, 1)
local DEPL_CLIMB_ALT = bind_add_param('CLIMB_ALT', 4, 2)
local base_pressure = nil
local SERVO_FUNCTION = 94
local reached_climb_alt = false
local deployed = false

local MODE_LAND = 9

function update_state()
   local pressure = baro:get_pressure()
   if not pressure then
      return
   end
   if not base_pressure then
      base_pressure = pressure
   end
   local altitude = baro:get_altitude_difference(base_pressure, pressure)
   if not altitude then
      return
   end
   gcs:send_named_float('DALT',altitude)
   logger.write("DEPL",'BP,P,Alt','fff',
                base_pressure, pressure, altitude)
   if not reached_climb_alt then
      local start_pos = end_pos:get() + offset:get()
      SRV_Channels:set_output_pwm(SERVO_FUNCTION, start_pos)
      if altitude > DEPL_CLIMB_ALT:get() then
         reached_climb_alt = true
         gcs:send_text(MAV_SEVERITY.ERROR, "DEPL: Reached climb alt")
      end
      return
   end
   if deployed then
      return
   end
   if altitude > deployment_alt:get() then
      return
   end
   deployed = true
   gcs:send_text(MAV_SEVERITY.INFO, "DEPL: deploying")
   vehicle:set_mode(MODE_LAND)
   arming:arm_force()
   if (vehicle:get_mode() ~= MODE_LAND) or not arming:is_armed() then
      gcs:send_text(MAV_SEVERITY.INFO, "DEPL: Arming failed")
      return
   end
   --SRV_Channels:set_output_pwm(SERVO_FUNCTION, end_pos:get())
   gcs:send_text(MAV_SEVERITY.INFO, "DEPL: Deployed successfully")
end
function update() -- this is the loop which periodically runs
   update_state()
   return update, 20 -- Reschedules the loop at 50Hz
end
gcs:send_text(MAV_SEVERITY.INFO, "DEPL: loaded")
return update() -- Run immediately before starting to reschedule
