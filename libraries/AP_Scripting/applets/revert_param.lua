--[[
   parameter reversion utility. This helps with manual tuning
   in-flight by giving a way to instantly revert parameters to the startup parameters
--]]
local MAV_SEVERITY = {EMERGENCY=0, ALERT=1, CRITICAL=2, ERROR=3, WARNING=4, NOTICE=5, INFO=6, DEBUG=7}

local PARAM_TABLE_KEY = 31
local PARAM_TABLE_PREFIX = "PREV_"

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
   assert(p, string.format("count not find parameter %s", name))
   return p
end

-- setup script specific parameters
assert(param:add_table(PARAM_TABLE_KEY, PARAM_TABLE_PREFIX, 5), 'could not add param table')

--[[
  // @Param: PREV_ENABLE
  // @DisplayName: parameter reversion enable
  // @Description: Enable parameter reversion system
  // @Values: 0:Disabled,1:Enabled
  // @User: Standard
--]]
local PREV_ENABLE      = bind_add_param('ENABLE',         1, 0)

--[[
  // @Param: PREV_RC_FUNC
  // @DisplayName: param reversion RC function
  // @Description: RCn_OPTION number to used to trigger parameter reversion
  // @User: Standard
--]]
local PREV_RC_FUNC     = bind_add_param('RC_FUNC',       2, 300)

-- params dictionary indexed by name
local params = {}
local param_saved = {}
local param_count = 0

local ATC_prefixes = { "ATC", "Q_A" }
local PSC_prefixes = { "PSC", "Q_P" }
local PID_prefixes = { "_RAT_RLL_", "_RAT_PIT_", "_RAT_YAW_" }
local PID_suffixes = { "FF", "P", "I", "D", "IMAX", "FLTD", "FLTE", "FLTT", "SMAX" }
local angle_axes = { "RLL", "PIT", "YAW" }
local PSC_types = { "ACCZ", "VELZ", "POSZ", "VELXY", "POSXY" }
if PREV_ENABLE:get() == 0 then
   return
end

local function add_param(pname)
   local p = bind_param(pname)
   if p then
      params[pname] = p
      param_saved[pname] = p:get()
      param_count = param_count + 1
      -- gcs:send_text(MAV_SEVERITY.INFO, string.format("Added %s", pname))
   end
end

-- add rate PIDs
for _, atc in ipairs(ATC_prefixes) do
   for _, prefix in ipairs(PID_prefixes) do
      for _, suffix in ipairs(PID_suffixes) do
         add_param(atc .. prefix .. suffix)
      end
   end
end

-- add angle Ps
for _, atc in ipairs(ATC_prefixes) do
   for _, axis in ipairs(angle_axes) do
      add_param(atc .. "_ANG_" .. axis .. "_P" )
   end
end

-- add fixed wing tuning
for _, suffix in ipairs(PID_suffixes) do
   add_param("RLL_RATE_" .. suffix)
   add_param("PIT_RATE_" .. suffix)
   add_param("YAW_RATE_" .. suffix)
end

-- add PSC tuning
for _, psc in ipairs(PSC_prefixes) do
   for _, ptype in ipairs(PSC_types) do
      for _, suffix in ipairs(PID_suffixes) do
         add_param(psc .. "_" .. ptype .. "_" .. suffix)
      end
   end
end


local function revert_parameters()
   local count = 0
   for pname, p in pairs(params) do
      local v1 = p:get()
      local v2 = param_saved[pname]
      if v1 ~= v2 then
         p:set_and_save(param_saved[pname])
         count = count + 1
      end
   end
   return count
end

gcs:send_text(MAV_SEVERITY.INFO, string.format("Stored %u parameters", param_count))

local done_revert = false

-- main update function
function update()
   local sw_pos = rc:get_aux_cached(PREV_RC_FUNC:get())
   if not sw_pos then
      return
   end
   if sw_pos == 2 and not done_revert then
      done_revert = true
      count = revert_parameters()
      gcs:send_text(MAV_SEVERITY.INFO, string.format("Reverted %u parameters", count))
      return
   end
   if sw_pos == 0 then
      done_revert = false
   end
end

-- wrapper around update(). This calls update() at 10Hz,
-- and if update faults then an error is displayed, but the script is not
-- stopped
function protected_wrapper()
  local success, err = pcall(update)
  if not success then
     gcs:send_text(MAV_SEVERITY.EMERGENCY, "Internal Error: " .. err)
     -- when we fault we run the update function again after 1s, slowing it
     -- down a bit so we don't flood the console with errors
     --return protected_wrapper, 1000
     return
  end
  return protected_wrapper, 1000/UPDATE_RATE_HZ
end

-- start running update loop
return protected_wrapper()
