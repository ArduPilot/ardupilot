-- support takeoff and landing on moving platforms for copters
-- luacheck: only 0

local PARAM_TABLE_KEY = 7
local PARAM_TABLE_PREFIX = "SHIP_"

local MODE_MANUAL = 0
local MODE_RTL = 6
local MODE_AUTO = 3
local MODE_GUIDED = 4
local MODE_AUTO_RLT = 27
local MODE_FOLLOW = 23
local MODE_LAND = 9

local ALT_FRAME_ABSOLUTE = 0

-- 3 throttle position
local THROTTLE_LOW = 0
local THROTTLE_MID = 1
local THROTTLE_HIGH = 2

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
assert(param:add_table(PARAM_TABLE_KEY, PARAM_TABLE_PREFIX, 4), 'could not add param table')
--[[
  // @Param: SHIP_ENABLE
  // @DisplayName: Ship landing enable
  // @Description: Enable ship landing system
  // @Values: 0:Disabled,1:Enabled
  // @User: Standard
--]]
SHIP_ENABLE     = bind_add_param('ENABLE', 1, 0)

--[[
  // @Param: SHIP_LAND_ACTIVE
  // @DisplayName: Is landing active
  // @Description: Is landding in progress, issuing a land or follow mode won't set it to true, but setting the mode to RTL will activate the script.
  // @Range: 0: Disabled,1:Enabled
  // @Units: bool
  // @User: Standard
--]]
SHIP_LAND_ACTIVE = bind_add_param('LAND_ACTIVE', 2, 0)

--[[
  // @Param: SHIP_AUTO_OFS
  // @DisplayName: Ship automatic offset trigger
  // @Description: Settings this parameter to one triggers an automatic follow offset calculation based on current position of the vehicle and the landing target. NOTE: This parameter will auto-reset to zero once the offset has been calculated.
  // @Values: 0:Disabled,1:Trigger
  // @User: Standard
--]]
SHIP_AUTO_OFS   = bind_add_param('AUTO_OFS', 3, 0)

--[[
  // @Param: SHIP_OFS_Z
  // @DisplayName: The height of FOLL_OFS_Z above target (in m)
  // @Description: FOLL_OFS_Z should be set to approach altitude, but the land height has to be known.
  // @Values: 0..lot
  // @Units: m
  // @User: Standard
]]
SHIP_OFS_Z     = bind_add_param('OFS_Z', 4, 15)

-- other parameters
RCMAP_THROTTLE  = bind_param("RCMAP_THROTTLE")
RTL_ALT         = bind_param("RTL_ALT")
RTL_ALT_FINAL   = bind_param("RTL_ALT_FINAL")
RTL_SPEED       = bind_param("RTL_SPEED")
WPNAV_SPEED     = bind_param("WPNAV_SPEED")
WPNAV_RADIUS    = bind_param("WPNAV_RADIUS")
FOLL_OFS_X      = bind_param("FOLL_OFS_X")
FOLL_OFS_Y      = bind_param("FOLL_OFS_Y")
FOLL_OFS_Z      = bind_param("FOLL_OFS_Z")

-- an auth ID to disallow arming when we don't have the beacon
local auth_id = arming:get_aux_auth_id()
arming:set_aux_auth_failed(auth_id, "Ship: no beacon")

-- current target
local target_pos = Location()
local current_pos = Location()
local target_velocity = Vector3f()
local target_heading = 0.0

-- landing stages
local STAGE_APPROACH = 0 -- Approach target in follow mode
local STAGE_DESCEND = 1 -- descend and land using precision landing
local landing_stage = STAGE_APPROACH

-- other state
local vehicle_mode = MODE_MANUAL
local throttle_pos = THROTTLE_HIGH
local have_target = false

-- check key parameters
local function check_parameters()
  --[[
     parameter values which are auto-set on startup
  --]]
   local key_params = {
      FOLL_ENABLE = 1,
      FOLL_OFS_TYPE = 1,
      FOLL_ALT_TYPE = 0,
      PLND_ENABLED = 1,
      PLND_TYPE = 5,
      PLND_EST_TYPE = 1,
      PLND_OPTIONS = 1 -- Allow PrecLand onto moving target
   }

   for p, v in pairs(key_params) do
      local current = param:get(p)
      assert(current, string.format("Parameter %s not found", p))
      if math.abs(v-current) > 0.001 then
         param:set_and_save(p, v)
         gcs:send_text(0,string.format("Parameter %s set to %.2f was %.2f", p, v, current))
      end
   end
end

-- get throttle position
---@returns upvalue
local function get_throttle_pos()
   local tpos
   if not rc:has_valid_input() then
      tpos = THROTTLE_LOW
   else
      local tchan = rc:get_channel(RCMAP_THROTTLE:get())
      local tval = (tchan:norm_input_ignore_trim()+1.0)*0.5
      if tval >= 0.40 then
         tpos = THROTTLE_HIGH
      elseif tval >= 0.1 then
         tpos = THROTTLE_MID
      else
         tpos = THROTTLE_LOW
      end
   end
   return tpos
end

-- wrap angle [0, 360) range
local function wrap_360(angle)
   local res = math.fmod(angle, 360.0)
    if res < 0 then
        res = res + 360.0
    end
    return res
end

-- wrap angle (-180, 180] range
local function wrap_180(angle)
    local res = wrap_360(angle)
    if res > 180 then
       res = res - 360
    end
    return res
end


-- get horizontal distance to ship
---returns number
local function get_home_distance()
   return target_pos:get_distance_NE(current_pos):length()
end

--[[
   check if we should abort a QRTL landing
--]]
local function check_approach_abort()
   local alt = current_pos:alt() * 0.01
   local target_alt = get_target_alt()
   if alt > target_alt then
      gcs:send_text(0, "Aborting landing")
      landing_stage = STAGE_APPROACH
   end
end

-- update state based on vehicle mode
local function update_mode()
   local mode = vehicle:get_mode()
   if mode == vehicle_mode then
      return
   end
   vehicle_mode = mode
   if not (mode == MODE_LAND or mode == MODE_FOLLOW) then
      landing_stage = STAGE_APPROACH -- anything except LAND and FOLLOw will reset the internal state
      SHIP_LAND_ACTIVE:set(0)
   end
end

-- update target state
local function update_target()
   if not follow:have_target() then
      if have_target then
         gcs:send_text(0,"Lost beacon")
         arming:set_aux_auth_failed(auth_id, "Ship: no beacon")
      end
      have_target = false
      return
   end
   if not have_target then
      gcs:send_text(0,"Have beacon")
      arming:set_aux_auth_passed(auth_id)
   end
   have_target = true

   target_pos, target_velocity = follow:get_target_location_and_velocity_ofs()
   target_pos:change_alt_frame(ALT_FRAME_ABSOLUTE)
   target_pos:alt(target_pos:alt() - SHIP_OFS_Z:get() * 100) -- cm
   --gcs:send_text(5, string.format("altitude: %f", target_pos:alt()))
   target_heading = follow:get_target_heading_deg()
   -- zero vertical velocity to reduce impact of ship movement
   target_velocity:z(0)
end

-- returns the copter max approach velocity
---@return number
local function get_rtl_speed()
   if (RTL_SPEED:get() ~= 0) then
      return RTL_SPEED:get() * 0.01
   else
      return WPNAV_SPEED:get() * 0.01
   end

end

-- Check if switching from follow is needed
---@param throttle_pos integer
---@return boolean
-- true if ship landing is active
local function update_approach(throttle_pos)
   if not (SHIP_LAND_ACTIVE:get() == 0) then
      if vehicle_mode == MODE_FOLLOW and landing_stage == STAGE_APPROACH then
         if throttle_pos <= THROTTLE_MID and target_pos:get_distance_NE(current_pos):length() * 100 < WPNAV_RADIUS:get() then
            gcs:send_text(6, "SHIP_LAND start descend.")
            landing_stage = STAGE_DESCEND
            vehicle:set_mode(MODE_LAND)
         end
      elseif vehicle_mode == MODE_LAND and landing_stage == STAGE_DESCEND then
         if throttle_pos >= THROTTLE_HIGH then
            gcs:send_text(5, "SHIP_LAND abort descend due to manual override.")
            landing_stage = STAGE_APPROACH
            vehicle:set_mode(MODE_FOLLOW)
         end
      end
      return true
   elseif (vehicle_mode == MODE_RTL or vehicle_mode == MODE_AUTO_RLT) and landing_stage == STAGE_APPROACH then
      SHIP_LAND_ACTIVE:set(1)
      gcs:send_text(6, 'SHIP_LAND start approach')
      vehicle:set_mode(MODE_FOLLOW)
      return true
   end 
   return false
end

--[[
  update automatic beacon offsets
--]]

local function update_auto_offset()
   if arming:is_armed() or math.floor(SHIP_AUTO_OFS:get()) ~= 1 then
      return
   end

   -- get target without offsets applied
   target_no_ofs, vel = follow:get_target_location_and_velocity()
   target_no_ofs:change_alt_frame(ALT_FRAME_ABSOLUTE)

   -- setup offsets so target location will be current location
   local new = target_no_ofs:get_distance_NED(current_pos)
   new:rotate_xy(-math.rad(target_heading))

   gcs:send_text(0,string.format("Set follow offset (%.2f,%.2f,%.2f)", new:x(), new:y(), new:z()))
   FOLL_OFS_X:set_and_save(new:x())
   FOLL_OFS_Y:set_and_save(new:y())
   FOLL_OFS_Z:set_and_save(new:z() - SHIP_OFS_Z:get())

   SHIP_AUTO_OFS:set_and_save(0)
end

-- main update function
local function update()
   if SHIP_ENABLE:get() < 1 then
      return
   end

   update_target()
   if not have_target then
      return
   end

   current_pos = ahrs:get_position()
   if not current_pos then
      gcs:send_text(0, "No valid pos") -- TOOD remove me
      return
   end
   current_pos:change_alt_frame(ALT_FRAME_ABSOLUTE)

   -- TODO local throttle_pos = get_throttle_pos()
   update_mode()
   update_auto_offset()

   ahrs:set_home(target_pos)

   local throttle_pos = get_throttle_pos()

   if(update_approach(throttle_pos)) then

      if landing_stage == STAGE_DESCEND then
   
         if vehicle_mode == MODE_FOLLOW or vehicle_mode == MODE_LAND then
      vehicle:set_target_location(target_pos)
         
         end
      end

      -- takeoff
   elseif vehicle:is_taking_off() and (vehicle_mode == MODE_GUIDED or vehicle_mode == MODE_AUTO) then

      vehicle:set_velocity_match(target_velocity:xy());
   end
   
end

local function loop()
   update()
   -- run at 20Hz
   return loop, 50
end

check_parameters()

-- wrapper around update(). This calls update() at 20Hz,
-- and if update faults then an error is displayed, but the script is not
-- stopped
local function protected_wrapper()
  local success, err = pcall(update)
  if not success then
     gcs:send_text(0, "Internal Error: " .. err)
     -- when we fault we run the update function again after 1s, slowing it
     -- down a bit so we don't flood the console with errors
     return protected_wrapper, 1000
  end
  return protected_wrapper, 50
end

-- start running update loop
return protected_wrapper()

