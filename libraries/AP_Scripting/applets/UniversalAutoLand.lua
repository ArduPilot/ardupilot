--[[ Upon Arming , creates a four item mission consisting of: NAV_TAKEOFF, DO_LAND_START,Final Approach WP opposite bearing from HOME of heading used during takeoff, at TKOFF_ALT or SCR_USER3 above home, SCR_USER2 or 2X TKOFF_DIST, and a LAND waypoint at HOME and stops until next disarm/boot. SCR_USER1 is used to enable or disable it.
--]]
local MAV_SEVERITY = {EMERGENCY=0, ALERT=1, CRITICAL=2, ERROR=3, WARNING=4, NOTICE=5, INFO=6, DEBUG=7}

local PARAM_TABLE_KEY = 80
local PARAM_TABLE_PREFIX = "AUTOLAND_"
assert(param:add_table(PARAM_TABLE_KEY, PARAM_TABLE_PREFIX, 4), 'could not add param table')

-- add a parameter and bind it to a variable
function bind_add_param(name, idx, default_value)
    assert(param:add_param(PARAM_TABLE_KEY, idx, name, default_value), string.format('could not add param %s', name))
    return Parameter(PARAM_TABLE_PREFIX .. name)
end

--[[
  // @Param: AUTOLAND_ENABLE
  // @DisplayName: AUTOLAND ENABLE
  // @Description: enable AUTOLAND script action
  // @Values: 0:Disabled,1:Enabled
  // @User: Standard
--]]
local AULND_ENABLE = bind_add_param('ENABLE', 1, 1)
local enable = AULND_ENABLE:get()

--[[
  // @Param: AUTOLAND_WP_ALT
  // @DisplayName: Final approach waypoint alt
  // @Description: Altitude of final approach waypoint created by script
  // @Range: 1 100
  // @Units: m
  // @User: Standard
--]]
local AULND_ALT = bind_add_param('WP_ALT', 2, 0)
local final_wp_alt = AULND_ALT:get()
--[[
  // @Param: AUTOLAND_WP_DIST
  // @DisplayName: Final approach waypoint distance
  // @Description: Distance from landng point (HOME) to final approach waypoint created by script in the opposite direction of initial takeoff
  // @Range: 0 1000
  // @Units: m
  // @User: Standard
--]]
local AULND_DIST = bind_add_param('WP_DIST', 3, 0)
local final_wp_dist = AULND_DIST:get()

FRAME_GLOBAL = 3
NAV_WAYPOINT = 16
NAV_TAKEOFF = 22
NAV_LAND = 21
DO_LAND_START = 189
LOITER_TO_ALT = 31

TAKEOFF_PITCH = 15

local function wrap_360(angle)
   local res = math.fmod(angle, 360.0)
    if res < 0 then
        res = res + 360.0
    end
    return res
end

local function wrap_180(angle)
    local res = wrap_360(angle)
    if res > 180 then
       res = res - 360
    end
    return res
end

function get_final_approach_loc(bearing, dist) --degs,m
    local loc = ahrs:get_home()
    loc:offset_bearing(bearing,dist) ---degs and meters
    return loc
end

function create_final_approach_WP(loc,alt) --loc,m
   local item = mavlink_mission_item_int_t()

   item:seq(4)
   item:frame(FRAME_GLOBAL)
   item:command(NAV_WAYPOINT)
   item:param1(0)
   item:param2(0)
   item:param3(0)
   item:param4(0)
   item:x(loc:lat())
   item:y(loc:lng())
   item:z(alt)
   return item
end

function create_takeoff_WP(alt)
   local item = mavlink_mission_item_int_t()
   local loc = ahrs:get_home()

   item:seq(1)
   item:frame(FRAME_GLOBAL)
   item:command(NAV_TAKEOFF)
   item:param1(TAKEOFF_PITCH)
   item:param2(0)
   item:param3(0)
   item:param4(0)
   item:x(loc:lat())
   item:y(loc:lng())
   item:z(alt)
   return item
end

function create_land_WP()
   local item = mavlink_mission_item_int_t()
   local loc = ahrs:get_home()

   item:seq(5)
   item:frame(FRAME_GLOBAL)
   item:command(NAV_LAND)
   item:param1(15)
   item:param2(0)
   item:param3(0)
   item:param4(0)
   item:x(loc:lat())
   item:y(loc:lng())
   item:z(0)
   return item
end

function create_do_land_start_WP()
   local item = mavlink_mission_item_int_t()

   item:seq(2)
   item:frame(FRAME_GLOBAL)
   item:command(DO_LAND_START)
   item:param1(0)
   item:param2(0)
   item:param3(0)
   item:param4(0)
   item:x(0)
   item:y(0)
   item:z(0)
   return item
end

function create_loiter_to_alt_WP(bearing,land_start,radius,alt) -- degs,loc,m,m
   local item = mavlink_mission_item_int_t()
   local loc = land_start:copy()
   -- NOTE: Unlike the true AUTOLAND mode, we are creating the waypoints at takeoff
   -- So we can't know for sure if we'll be left or right of the final approach wp
   -- Defaulting to clockwise loiter
   -- NOTE: AUTOLAND uses the navigation controller-sanitized radius and it's not
   -- exposed via the Lua API apparently, so using the radius as is
   loc:offset_bearing(wrap_360(bearing-90),radius)

   item:seq(3)
   item:frame(FRAME_GLOBAL)
   item:command(LOITER_TO_ALT)
   item:param1(1)
   item:param2(radius)
   item:param3(1)
   item:param4(0)
   item:x(loc:lat())
   item:y(loc:lng())
   item:z(alt)
   return item
end

function update()
  if not arming:is_armed() then --if disarmed, wait until armed
    mission_loaded = false
    return update,1000
  end
  
  if not mission_loaded then --if first time after arm and enabled is valid then create  mission 
    local home = ahrs:get_home()
    local location = ahrs:get_location()
    local speed = gps:ground_speed(0)
    local alt = baro:get_altitude()
    if location and home and speed > (0.5 * param:get("AIRSPEED_MIN")) then
        local yaw = gps:ground_course(0)
        local land_start = get_final_approach_loc(wrap_180(yaw+180),final_wp_dist)
        local loiter_radius = math.min(AULND_DIST:get() * 0.333,math.abs(param:get("WP_LOITER_RAD")))
        mission:set_item(3,create_loiter_to_alt_WP(wrap_180(yaw+180),land_start,loiter_radius,final_wp_alt))
        mission:set_item(4,create_final_approach_WP(land_start,final_wp_alt))
        mission:set_item(5,create_land_WP())
        mission_loaded = true
        gcs:send_text(MAV_SEVERITY.NOTICE, string.format("Captured initial takeoff direction = %.1f at %.1f m and %.1f m/s",yaw, alt, speed))
    end
  end
  return update, 200
end 

gcs:send_text(MAV_SEVERITY.INFO,"Loaded UniversalAutoLand.lua")
if enable == 1 then 
   if final_wp_dist == 0 or final_wp_alt ==0 then
      gcs:send_text(MAV_SEVERITY.CRITICAL, string.format("Must set Final Waypoint alt and dist values!"))
      return
   end
   mission:clear()
   local item = mavlink_mission_item_int_t()
   item:command(NAV_WAYPOINT)
   mission:set_item(0,item)
   mission:set_item(1,create_takeoff_WP(param:get("TKOFF_ALT"))) 
   mission:set_item(2,create_do_land_start_WP()) 
   gcs:send_text(MAV_SEVERITY.CRITICAL, string.format("Set Final Waypoint alt and dist values!"))   
   return update, 1000
else
   gcs:send_text(MAV_SEVERITY.CRITICAL, string.format("Script disabled by AUTOLAND_ENABLE"))
return
end

