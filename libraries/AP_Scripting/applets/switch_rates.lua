--[[
   rate switch utility. This helps to compare different tunes in-flight to compare performance
--]]

---@diagnostic disable: param-type-mismatch


local MAV_SEVERITY = {EMERGENCY=0, ALERT=1, CRITICAL=2, ERROR=3, WARNING=4, NOTICE=5, INFO=6, DEBUG=7}

local PARAM_TABLE_KEY = 33
local PARAM_TABLE_PREFIX = "RTSW_"
local PARAM_BACKUP_TABLE_KEY = 34
local PARAM_BACKUP_TABLE_PREFIX = "X_"

local UPDATE_RATE_HZ = 4

-- bind a parameter to a variable, old syntax to support older firmware
function bind_param(name)
   local p = Parameter()
   if not p:init(name) then
      return nil
   end
   return p
end

-- add a parameter and bind it to a variable
function bind_add_param(name, idx, default_value)
   assert(param:add_param(PARAM_TABLE_KEY, idx, name, default_value), string.format('could not add param %s', name))
   local p = bind_param(PARAM_TABLE_PREFIX .. name)
   assert(p, string.format("could not find parameter %s", name))
   return p
end

-- add a backup parameter with appropriate length and bind it to a variable
function bind_add_backup_param(name, idx, default_value)
   -- shorten pname to fit with X_ prefix
   local short_name = string.sub(name, math.max(1, 1 + string.len(name) - (16 - string.len(PARAM_BACKUP_TABLE_PREFIX))))
   assert(param:add_param(PARAM_BACKUP_TABLE_KEY, idx, short_name, default_value), string.format('could not add param %s', short_name))
   local p = bind_param(PARAM_BACKUP_TABLE_PREFIX .. short_name)
   assert(p, string.format("could not find parameter %s", PARAM_BACKUP_TABLE_PREFIX .. short_name))
   return p
end

-- setup script specific parameters
param:add_table(PARAM_TABLE_KEY, PARAM_TABLE_PREFIX, 5)

--[[
  // @Param: RTSW_ENABLE
  // @DisplayName: parameter reversion enable
  // @Description: Enable parameter reversion system
  // @Values: 0:Disabled,1:Enabled
  // @User: Standard
--]]
local PREV_ENABLE      = bind_add_param('ENABLE',         1, 0)

--[[
  // @Param: RTSW_RC_FUNC
  // @DisplayName: param reversion RC function
  // @Description: RCn_OPTION number to used to trigger parameter reversion
  // @User: Standard
--]]
local PREV_RC_FUNC     = bind_add_param('RC_FUNC',       2, 300)

-- params dictionary indexed by name
local param_table = {
   "ATC_ACCEL_P_MAX",
   "ATC_ACCEL_R_MAX",
   "ATC_ACCEL_Y_MAX",
   "ATC_ANG_PIT_P",
   "ATC_ANG_RLL_P",
   "ATC_ANG_YAW_P",
   "ATC_RAT_PIT_P",
   "ATC_RAT_PIT_I",
   "ATC_RAT_PIT_D",
   "ATC_RAT_PIT_D_FF",
   "ATC_RAT_PIT_FLTD",
   "ATC_RAT_RLL_P",
   "ATC_RAT_RLL_I",
   "ATC_RAT_RLL_D",
   "ATC_RAT_RLL_D_FF",
   "ATC_RAT_RLL_FLTD",
   "ATC_RAT_YAW_P",
   "ATC_RAT_YAW_I",
   "ATC_RAT_YAW_D",
   "ATC_RAT_YAW_D_FF",
   "ATC_RAT_YAW_FLTD",
   "ATC_THR_G_BOOST",
   "ACRO_RP_RATE_TC",
   "ACRO_Y_RATE_TC",
   "FSTRATE_ENABLE",
   "FSTRATE_DIV",
   "MOT_SPIN_MIN",
   "MOT_SPIN_MAX",
   "MOT_THST_EXPO",
   "SERVO_DSHOT_RATE",
}

-- setup script specific parameters
param:add_table(PARAM_BACKUP_TABLE_KEY, PARAM_BACKUP_TABLE_PREFIX, #param_table)

local params = {}
local prev_params = {}
local param_count = 0

if PREV_ENABLE:get() == 0 then
   gcs:send_text(MAV_SEVERITY.NOTICE, string.format("Rate switch: disabled"))
   return
end

local function add_param(pname)
   local p = bind_param(pname)
   if p then
      params[pname] = p
      param_count = param_count + 1
      local px = bind_add_backup_param(pname, param_count, p:get())
      if px then
         prev_params[pname] = px
      end
   end
end

for _, pname in pairs(param_table) do
   add_param(pname)
end

local function switch_parameters()
   local count = 0
   for pname, p1 in pairs(params) do
      local p2 = prev_params[pname]
      local v1 = p1:get()
      local v2 = p2:get()
      if v1 ~= v2 then
         p1:set_and_save(v2)
         p2:set_and_save(v1)
         count = count + 1
      end
   end
   return count
end

gcs:send_text(MAV_SEVERITY.INFO, string.format("Rate switch: stored %u parameters", param_count))

local AuxSwitchPos = {LOW=0, MIDDLE=1, HIGH=2}
local AuxSwitchPosNames = {"LOW", "MIDDLE", "HIGH"}

-- start in LOW start
local prev_pos = AuxSwitchPos.LOW

-- main update function
function update()
   local sw_pos = rc:get_aux_cached(PREV_RC_FUNC:get())

   if sw_pos ~= nil and sw_pos ~= prev_pos then
      count = switch_parameters()
      gcs:send_text(MAV_SEVERITY.INFO, string.format("Rate switch: %u parameters changed to %s", count, AuxSwitchPosNames[sw_pos + 1]))
      prev_pos = sw_pos
   end
end

-- wrapper around update(). This calls update() at 10Hz,
-- and if update faults then an error is displayed, but the script is not
-- stopped
function protected_wrapper()
  local success, err = pcall(update)
  if not success then
     gcs:send_text(MAV_SEVERITY.EMERGENCY, "Rate switch: internal Error: " .. err)
     -- when we fault we run the update function again after 1s, slowing it
     -- down a bit so we don't flood the console with errors
     --return protected_wrapper, 1000
     return
  end
  return protected_wrapper, 1000/UPDATE_RATE_HZ
end

-- start running update loop
return protected_wrapper()
