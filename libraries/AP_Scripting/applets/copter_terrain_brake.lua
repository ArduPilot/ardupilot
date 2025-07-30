--[[
script to prevent terrain impact in LOITER mode while flying copters in steep terrain
--]]

local PARAM_TABLE_KEY = 84
local PARAM_TABLE_PREFIX = "TERR_BRK_"

local MODE_LOITER = 5
local MODE_BRAKE = 17

local MAV_SEVERITY = {EMERGENCY=0, ALERT=1, CRITICAL=2, ERROR=3, WARNING=4, NOTICE=5, INFO=6, DEBUG=7}

-- add a parameter and bind it to a variable
function bind_add_param(name, idx, default_value)
   assert(param:add_param(PARAM_TABLE_KEY, idx, name, default_value), string.format('could not add param %s', name))
   return Parameter(PARAM_TABLE_PREFIX .. name)
end

-- setup script specific parameters
assert(param:add_table(PARAM_TABLE_KEY, PARAM_TABLE_PREFIX, 14), 'could not add param table')

--[[
  // @Param: TERR_BRK_ENABLE
  // @DisplayName: terrain brake enable
  // @Description: terrain brake enable
  // @Values: 0:Disabled,1:Enabled
  // @User: Standard
--]]
local TERR_BRK_ENABLE = bind_add_param('ENABLE', 1, 1)

--[[
  // @Param: TERR_BRK_ALT
  // @DisplayName: terrain brake altitude
  // @Description: terrain brake altitude. The altitude above the ground below which BRAKE mode will be engaged if in LOITER mode.
  // @Range: 1 100
  // @Units: m
  // @User: Standard
--]]
local TERR_BRK_ALT = bind_add_param('ALT', 2, 30)

--[[
  // @Param: TERR_BRK_HDIST
  // @DisplayName: terrain brake home distance
  // @Description: terrain brake home distance. The distance from home where the auto BRAKE will be enabled. When within this distance of home the script will not activate
  // @Range: 0 1000
  // @Units: m
  // @User: Standard
--]]
local TERR_BRK_HDIST = bind_add_param('HDIST', 3, 100)

--[[
  // @Param: TERR_BRK_SPD
  // @DisplayName: terrain brake speed threshold
  // @Description: terrain brake speed threshold. Don't trigger BRAKE if both horizontal speed and descent rate are below this threshold. By setting this to a small value this can be used to allow the user to climb up to a safe altitude in LOITER mode. A value of 0.5 is recommended if you want to use LOITER to recover from an emergency terrain BRAKE mode change.
  // @Range: 0 5
  // @Units: m/s
  // @User: Standard
--]]
local TERR_BRK_SPD = bind_add_param('SPD', 4, 0)

local function sq(x)
   return x*x
end

local function run_checks()
   if TERR_BRK_ENABLE:get() ~= 1 then
      return
   end
   if not arming:is_armed() then
      return
   end
   if vehicle:get_mode() ~= MODE_LOITER then
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
   if home_dist <= TERR_BRK_HDIST:get() then
      return
   end

   --[[
      get height above terrain with extrapolation
   --]]
   local hagl = terrain:height_above_terrain(true)
   if hagl >= TERR_BRK_ALT:get() then
      return
   end

   --[[
      allow for climbing in LOITER mode if enabled
   --]]
   if TERR_BRK_SPD:get() > 0 then
      local spd = ahrs:get_velocity_NED()
      if spd ~= nil then
         local hspd = math.sqrt(sq(spd:x())+sq(spd:y()))
         local drate = spd:z()
         if hspd < TERR_BRK_SPD:get() and drate < TERR_BRK_SPD:get() then
            return
         end
      end
   end

   if vehicle:set_mode(MODE_BRAKE) then
      gcs:send_text(MAV_SEVERITY.EMERGENCY, string.format("@Terrain %.1fm - BRAKE", hagl))
   end
end

--[[
   main update function, called at 1Hz
--]]
function update()
   run_checks()
   return update, 100
end

if TERR_BRK_ENABLE:get() == 1 then
   gcs:send_text(MAV_SEVERITY.INFO, string.format("Loaded Loiter/Brake checker"))
end

-- start running update loop
return update, 1000

