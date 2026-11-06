--[[
based on copter_terrain_brake.lua
script to automatically RTL the plane when above a given altitude AGL.
--]]

local PARAM_TABLE_KEY = 92
local PARAM_TABLE_PREFIX = "FEN_AGL_"

local MODE_AUTO = 10
local MODE_RTL = 11

local MAV_SEVERITY = {EMERGENCY=0, ALERT=1, CRITICAL=2, ERROR=3, WARNING=4, NOTICE=5, INFO=6, DEBUG=7}

-- add a parameter and bind it to a variable
function bind_add_param(name, idx, default_value)
   assert(param:add_param(PARAM_TABLE_KEY, idx, name, default_value), string.format('could not add param %s', name))
   return Parameter(PARAM_TABLE_PREFIX .. name)
end

-- Setup script specific parameters
assert(param:add_table(PARAM_TABLE_KEY, PARAM_TABLE_PREFIX, 14), 'could not add param table')

--[[
  // @Param: FEN_AGL_ENABLE
  // @DisplayName: fence AGL enable
  // @Description: fence AGL enable
  // @Values: 0:Disabled,1:Enabled
  // @User: Standard
--]]
local FEN_AGL_ENABLE = bind_add_param('ENABLE', 1, 1)

--[[
  // @Param: FEN_AGL_ALT
  // @DisplayName: Fence RTL altitude
  // @Description: Fence RTL altitude. The altitude above the ground above which RTL mode will be engaged if in AUTO mode.
  // @Range: 1 150
  // @Units: m
  // @User: Standard
--]]
local FEN_AGL_ALT = bind_add_param('ALT', 2, 140)

--[[
  // @Param: FEN_AGL_HDIST
  // @DisplayName: Fence RTL home distance
  // @Description: Fence RTL  home distance. The distance from home past which the rtl failsafe will be enabled. When within this distance of home the script will not activate.
  // @Range: 0 1000
  // @Units: m
  // @User: Standard
--]]
local FEN_AGL_HDIST = bind_add_param('HDIST', 3, 1000)

triggered = false

local function run_checks()
    if FEN_AGL_ENABLE:get() ~= 1 then
      triggered = false
      return
   end
   if not arming:is_armed() then
      triggered = false
      return
   end
   if triggered then
      -- only trigger once
      return
   end
   if vehicle:get_mode() ~= MODE_AUTO then
      return
   end

   if not ahrs:home_is_set() then
      return
   end
   local home = ahrs:get_home()
   local pos = ahrs:get_location()
   if not pos then
      return
   end
   local home_dist = pos:get_distance(home)
   if home_dist <= FEN_AGL_HDIST:get() then
      return
   end

   -- Get height above terrain with extrapolation
   local hagl = terrain:height_above_terrain(true)
   if hagl <= FEN_AGL_ALT:get() then
      -- Height is less than FEN_AGL_ALT, so don't activate
      return
   end

   if vehicle:set_mode(MODE_RTL) then
      triggered = true
      gcs:send_text(MAV_SEVERITY.EMERGENCY, string.format("Terrain %.1fm - RTL", hagl))
   end
end

-- Main update function, called at 1Hz
function update()
   run_checks()
   return update, 100
end

if FEN_AGL_ENABLE:get() == 1 then
   gcs:send_text(MAV_SEVERITY.INFO, string.format("Loaded AGL alt fence"))
end

-- Start running update loop
return update, 1000

