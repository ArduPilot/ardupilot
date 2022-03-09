-- support takeoff and landing on moving platforms for VTOL planes

local PARAM_TABLE_KEY = 7
local PARAM_TABLE_PREFIX = "SHIP_"

local MODE_MANUAL = 0
local MODE_RTL = 11
local MODE_QRTL = 21
local MODE_AUTO = 10
local MODE_QLOITER = 19

local NAV_TAKEOFF = 22
local NAV_VTOL_TAKEOFF = 84

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
assert(param:add_table(PARAM_TABLE_KEY, PARAM_TABLE_PREFIX, 3), 'could not add param table')
SHIP_ENABLE     = bind_add_param('ENABLE', 1, 0)
SHIP_LAND_ANGLE = bind_add_param('LAND_ANGLE', 2, 0)
SHIP_AUTO_OFS   = bind_add_param('AUTO_OFS', 3, 0)

-- other parameters
RCMAP_THROTTLE  = bind_param("RCMAP_THROTTLE")
ALT_HOLD_RTL    = bind_param("ALT_HOLD_RTL")
Q_RTL_ALT       = bind_param("Q_RTL_ALT")
TRIM_ARSPD_CM   = bind_param("TRIM_ARSPD_CM")
TECS_LAND_ARSPD = bind_param("TECS_LAND_ARSPD")
Q_TRANS_DECEL   = bind_param("Q_TRANS_DECEL")
WP_LOITER_RAD   = bind_param("WP_LOITER_RAD")
RTL_RADIUS      = bind_param("RTL_RADIUS")
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
local STAGE_HOLDOFF = 0
local STAGE_DESCEND = 1
local STAGE_APPROACH = 2
local STAGE_IDLE = 2
local landing_stage = STAGE_HOLDOFF

-- other state
local vehicle_mode = MODE_MANUAL
local reached_alt = false
local throttle_pos = THROTTLE_HIGH
local have_target = false

-- square a variable
function sq(v)
   return v*v
end

-- check key parameters
function check_parameters()
  --[[
     parameter values which are auto-set on startup
  --]]
   local key_params = {
      FOLL_ENABLE = 1,
      FOLL_OFS_TYPE = 1,
      FOLL_ALT_TYPE = 0,
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

-- update the pilots throttle position
function update_throttle_pos()
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
   if tpos ~= throttle_pos then
      reached_alt = false
      if landing_stage == STAGE_HOLDOFF and tpos <= THROTTLE_MID then
         landing_stage = STAGE_DESCEND
         gcs:send_text(0, string.format("Descending for approach (hd=%.1fm h=%.1f th=%.1f)",
                                        get_holdoff_distance(), current_pos:alt()*0.01, get_target_alt()))
      end
      if landing_stage == STAGE_DESCEND and tpos == THROTTLE_HIGH then
         gcs:send_text(0,"Climbing for holdoff")
         landing_stage = STAGE_HOLDOFF
      end
   end
   throttle_pos = tpos
end

-- get landing airspeed
function get_land_airspeed()
   if TECS_LAND_ARSPD:get() < 0 then
      return TRIM_ARSPD_CM:get() * 0.01
   end
   return TECS_LAND_ARSPD:get()
end

--[[
  calculate stopping distance assuming we are flying at
  TECS_LAND_ARSPD and are approaching the landing target from
  behind. Take account of the wind estimate to get approach
  groundspeed
--]]
function stopping_distance()
   -- get the target true airspeed for approach
   local tas = get_land_airspeed() * ahrs:get_EAS2TAS()

   -- add in wind in direction of flight
   local wind = ahrs:wind_estimate():xy()

   -- rotate wind to be in approach frame
   wind:rotate(-math.rad(target_heading + SHIP_LAND_ANGLE:get()))

   -- ship velocity rotated to the approach frame
   local ship2d = target_velocity:xy()
   ship2d:rotate(-math.rad(target_heading + SHIP_LAND_ANGLE:get()))

   -- calculate closing speed
   -- use pythagoras theorem to solve for the wind triangle
   local tas_sq = sq(tas)
   local y_sq = sq(wind:y())
   local closing_speed
   if tas_sq >= y_sq then
      closing_speed = math.sqrt(tas_sq - y_sq)
   else
      -- min 1 m/s
      closing_speed = 1.0
   end

   -- include the wind in the direction of the ship
   closing_speed = closing_speed + wind:x()

   -- account for the ship velocity
   closing_speed = closing_speed - ship2d:x()

   -- calculate stopping distance
   return sq(closing_speed) / (2.0 * Q_TRANS_DECEL:get())
end

-- get holdoff distance
function get_holdoff_radius()
   if RTL_RADIUS:get() ~= 0 then
      return RTL_RADIUS:get()
   end
   return WP_LOITER_RAD:get()
end

-- get holdoff distance
function get_holdoff_distance()
   local radius = get_holdoff_radius()
   local holdoff_dist = math.abs(radius*1.5)
   local stop_distance = stopping_distance()

   -- increase holdoff distance by up to 50% to ensure we can stop
   holdoff_dist = math.max(holdoff_dist, math.min(holdoff_dist*2.5, stop_distance*2))
   return holdoff_dist
end

-- get the holdoff position
function get_holdoff_position()
   local radius = get_holdoff_radius()
   local heading_deg = target_heading + SHIP_LAND_ANGLE:get()
   local holdoff_dist = get_holdoff_distance()

   local ofs = Vector2f()
   ofs:x(-holdoff_dist)
   ofs:y(radius)
   ofs:rotate(math.rad(heading_deg))
   local target = target_pos:copy()
   target:offset(ofs:x(), ofs:y())
   return target
end

function wrap_360(angle)
   local res = math.fmod(angle, 360.0)
    if res < 0 then
        res = res + 360.0
    end
    return res
end

function wrap_180(angle)
    local res = wrap_360(angle)
    if res > 180 then
       res = res - 360
    end
    return res
end

--[[
   check if we have reached the tangent to the landing location
--]]
function check_approach_tangent()
   local distance = current_pos:get_distance(target_pos)
   local holdoff_dist = get_holdoff_distance()
   if landing_stage == STAGE_HOLDOFF and throttle_pos <= THROTTLE_MID and distance < 4*holdoff_dist then
      gcs:send_text(0, string.format("Descending for approach (hd=%.1fm)", holdoff_dist))
      landing_stage = STAGE_DESCEND
   end
   if reached_alt and landing_stage == STAGE_DESCEND then
      -- go to approach stage when throttle is low, we are
      -- pointing at the ship and have reached target alt.
      -- Also require we are within 2.5 radius of the ship, and our heading is within 20
      -- degrees of the target heading
      local target_bearing_deg = wrap_180(math.deg(current_pos:get_bearing(target_pos)))
      local ground_bearing_deg = wrap_180(math.deg(ahrs:groundspeed_vector():angle()))
      local margin = 10
      local distance = current_pos:get_distance(target_pos)
      local holdoff_dist = get_holdoff_distance()
      local error1 = math.abs(wrap_180(target_bearing_deg - ground_bearing_deg))
      local error2 = math.abs(wrap_180(ground_bearing_deg - (target_heading + SHIP_LAND_ANGLE:get())))
      logger.write('SLND','TBrg,GBrg,Dist,HDist,Err1,Err2','ffffff',target_bearing_deg, ground_bearing_deg, distance, holdoff_dist, error1, error2)
      if (error1 < margin and
          distance < 2.5*holdoff_dist and
          distance > 0.7*holdoff_dist and
          error2 < 2*margin) then
         -- we are on the tangent, switch to QRTL
         gcs:send_text(0, "Starting approach")
         landing_stage = STAGE_APPROACH
         vehicle:set_mode(MODE_QRTL)
      end
   end
end

--[[
   check if we should abort a QRTL landing
--]]
function check_approach_abort()
   local alt = current_pos:alt() * 0.01
   local target_alt = get_target_alt()
   if alt > target_alt then
      gcs:send_text(0, "Aborting landing")
      landing_stage = STAGE_HOLDOFF
      vehicle:set_mode(MODE_RTL)
   end
end

-- update state based on vehicle mode
function update_mode()
   local mode = vehicle:get_mode()
   if mode == vehicle_mode then
      return
   end
   vehicle_mode = mode
   if mode == MODE_RTL then
      landing_stage = STAGE_HOLDOFF
      reached_alt = false
   elseif mode ~= MODE_QRTL then
      landing_stage = STAGE_IDLE
      reached_alt = false
   end
end

-- update target state
function update_target()
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
   target_heading = follow:get_target_heading_deg()
   -- zero vertical velocity to reduce impact of ship movement
   target_velocity:z(0)
end

-- get the alt target for holdoff, AMSL
function get_target_alt()
   local base_alt = target_pos:alt() * 0.01
   if landing_stage == STAGE_HOLDOFF then
      return base_alt + ALT_HOLD_RTL:get() * 0.01
   end
   return base_alt + Q_RTL_ALT:get()
end

function update_alt()
   local alt = current_pos:alt() * 0.01
   local target_alt = get_target_alt()
   if landing_stage == STAGE_HOLDOFF or landing_stage == STAGE_DESCEND then
      if math.abs(alt - target_alt) < 3 then
         if not reached_alt then
            gcs:send_text(0,"Reached target altitude")
         end
         reached_alt = true
      end
   end
end

--[[
  update automatic beacon offsets
--]]

function update_auto_offset()
   if arming:is_armed() or math.floor(SHIP_AUTO_OFS:get()) ~= 1 then
      return
   end

   -- get target without offsets applied
   target_no_ofs, vel = follow:get_target_location_and_velocity()

   -- setup offsets so target location will be current location
   local new = target_no_ofs:get_distance_NED(current_pos)
   new:rotate_xy(-math.rad(target_heading))

   gcs:send_text(0,string.format("Set follow offset (%.2f,%.2f,%.2f)", new:x(), new:y(), new:z()))
   FOLL_OFS_X:set_and_save(new:x())
   FOLL_OFS_Y:set_and_save(new:y())
   FOLL_OFS_Z:set_and_save(new:z())

   SHIP_AUTO_OFS:set_and_save(0)
end

-- main update function
function update()
   if SHIP_ENABLE:get() < 1 then
      return
   end

   update_target()
   if not have_target then
      return
   end

   current_pos = ahrs:get_position()
   if not current_pos then
      return
   end
   current_pos:change_alt_frame(ALT_FRAME_ABSOLUTE)

   update_throttle_pos()
   update_mode()
   update_alt()
   update_auto_offset()

   ahrs:set_home(target_pos)

   local next_WP = vehicle:get_target_location()
   if not next_WP then
      -- not in a flight mode with a target location
      return
   end

   if vehicle_mode == MODE_RTL then
      local holdoff_pos = get_holdoff_position()
      holdoff_pos:change_alt_frame(ALT_FRAME_ABSOLUTE)
      holdoff_pos:alt(math.floor(get_target_alt()*100))
      vehicle:update_target_location(next_WP, holdoff_pos)

      if throttle_pos == THROTTLE_LOW then
         check_approach_tangent()
      end

   elseif vehicle_mode == MODE_QRTL then
      vehicle:set_velocity_match(target_velocity:xy())
      target_pos:alt(next_WP:alt())
      vehicle:update_target_location(next_WP, target_pos)

      if throttle_pos == THROTTLE_HIGH then
         check_approach_abort()
      end
      
   elseif vehicle_mode == MODE_AUTO then
      local id = mission:get_current_nav_id()
      if id == NAV_VTOL_TAKEOFF or id == NAV_TAKEOFF then
         vehicle:set_velocity_match(target_velocity:xy())
         local tpos = current_pos:copy()
         tpos:alt(next_WP:alt())
         vehicle:update_target_location(next_WP, tpos)
      end

   elseif vehicle_mode == MODE_QLOITER then
      vehicle:set_velocity_match(target_velocity:xy())
   end

end

function loop()
   update()
   -- run at 20Hz
   return loop, 50
end

check_parameters()

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

-- start running update loop
return protected_wrapper()

