--[[
 support package place for quadplanes
--]]

local PARAM_TABLE_KEY = 9
local PARAM_TABLE_PREFIX = "PKG_"

local MODE_AUTO = 10

local NAV_VTOL_PAYLOAD_PLACE = 94

-- add a parameter and bind it to a variable
function bind_add_param(name, idx, default_value)
   assert(param:add_param(PARAM_TABLE_KEY, idx, name, default_value), string.format('could not add param %s', name))
   return Parameter(PARAM_TABLE_PREFIX .. name)
end

-- setup package place specific parameters
assert(param:add_table(PARAM_TABLE_KEY, PARAM_TABLE_PREFIX, 5), 'could not add param table')
local PKG_ENABLE        = bind_add_param('ENABLE', 1, 0)
local PKG_RELEASE_FUNC  = bind_add_param('RELEASE_FUNC', 2, 94)
local PKG_RELEASE_HGT   = bind_add_param('RELEASE_HGT',  3, 10)
local PKG_RELEASE_HOLD  = bind_add_param('RELEASE_HOLD', 4, 1)

local Q_LAND_SPEED = Parameter("Q_LAND_SPEED")
local Q_LAND_FINAL_ALT = Parameter("Q_LAND_FINAL_ALT")

local MAV_SEVERITY_INFO = 6
local MAV_SEVERITY_NOTICE = 5

local RNG_ORIENT_DOWN = 25

-- motors state
local MOTORS_SHUT_DOWN = 0

-- release state
local RELEASE_NONE    = 0
local RELEASE_DESCENT = 1
local RELEASE_HOLD1   = 2
local RELEASE_HOLD2   = 3
local RELEASE_DONE    = 4
local release_state = RELEASE_NONE

local release_start_t = 0

if PKG_ENABLE:get() ~= 1 then
   -- not enabled
   return
end

SRV_Channels:set_range(PKG_RELEASE_FUNC:get(), 1000)
SRV_Channels:set_output_scaled(PKG_RELEASE_FUNC:get(), 0)

-- get time in seconds
function get_time()
   return millis():tofloat() * 0.001
end

-- reset state
function reset()
   release_state = RELEASE_NONE
   SRV_Channels:set_output_scaled(PKG_RELEASE_FUNC:get(), 0)
end

--[[
   main update function, called at 20Hz
--]]
function update()
   if PKG_ENABLE:get() ~= 1 then
      -- not enabled
      return
   end

   -- only do something if in AUTO and in a PACKAGE_PLACE waypoint
   if vehicle:get_mode() ~= MODE_AUTO or mission:get_current_nav_id() ~= NAV_VTOL_PAYLOAD_PLACE then
      reset()
      return
   end

   if not arming:is_armed() then
      -- nothing to do when disarmed
      reset()
      return
   end

   -- check spool state for if we are landed
   local landed = false
   if motors:get_desired_spool_state() == MOTORS_SHUT_DOWN and release_state >= RELEASE_DESCENT then
      landed = true
   end

   -- wait till we are in the descent
   if not quadplane:in_vtol_land_descent() and release_state == RELEASE_NONE then
      reset()
      return
   end

   if release_state == RELEASE_NONE then
      -- we have started the descent
      release_state = RELEASE_DESCENT
   end

   -- see if we have valid rangefinder data
   if not landed and not rangefinder:has_data_orient(RNG_ORIENT_DOWN) then
      return
   end

   -- check the distance, if less than RNG_ORIENT_DOWN then release
   local dist_m
   if not landed then
      dist_m = rangefinder:distance_orient(RNG_ORIENT_DOWN)
   else
      dist_m = 0.0
   end

   -- slow down when within Q_LAND_FINAL_ALT of target
   local remaining_m = dist_m - PKG_RELEASE_HGT:get()
   if remaining_m > 0 and remaining_m < Q_LAND_FINAL_ALT:get() then
      vehicle:set_land_descent_rate(Q_LAND_SPEED:get()*0.01)
   end

   if remaining_m <= 0 then
      if PKG_RELEASE_HGT:get() <= 0 and not landed then
         -- wait for landing
         return
      end

      local now = get_time()
      -- we are at the target altitude
      if release_state == RELEASE_DESCENT then
         -- start timer
         release_start_t = now
         release_state = RELEASE_HOLD1
         vehicle:set_land_descent_rate(0)
      elseif release_state == RELEASE_HOLD1 then
         -- start waiting for the hold time for vehicle to settle
         vehicle:set_land_descent_rate(0)
         if now - release_start_t > PKG_RELEASE_HOLD:get() then
            gcs:send_text(MAV_SEVERITY_INFO, string.format("Package released at %.1fm", dist_m))
            SRV_Channels:set_output_scaled(PKG_RELEASE_FUNC:get(), 1000)
            release_state = RELEASE_HOLD2
         end
      elseif release_state == RELEASE_HOLD2 then
         -- do the release and wait for hold time again to ensure clean release
         vehicle:set_land_descent_rate(0)
         if now - release_start_t > PKG_RELEASE_HOLD:get()*2 then
            release_state = RELEASE_DONE
            -- aborting the landing causes us to climb back up and continue the mission
            if quadplane:abort_landing() then
               gcs:send_text(MAV_SEVERITY_INFO, string.format("Climbing"))
            else
               gcs:send_text(MAV_SEVERITY_NOTICE, string.format("land abort failed"))
            end
         end
      end
   end

end

function loop()
   update()
   -- run at 20Hz
   return loop, 50
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
  return protected_wrapper, 50
end

gcs:send_text(MAV_SEVERITY_INFO, "Loaded package place script")

-- start running update loop
return protected_wrapper()

