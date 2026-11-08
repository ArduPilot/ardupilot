--[[
   SmartRTL return point reset from an RC switch

   Moving the configured switch to its high position clears the SmartRTL
   breadcrumb path and makes the vehicle's current position the point that
   SmartRTL will return to (and land at).

   This is mainly useful on vehicles navigating without GPS, for example
   with optical flow and a rangefinder, where the position estimate drifts
   over time.  The oldest part of the path, including the original return
   point, becomes the least accurate, so resetting the return point at a
   known safe location keeps the return journey short and accurate.

   The reset is refused while disarmed (arming resets the path anyway) and
   while in SmartRTL mode (the path is being flown).
--]]

local PARAM_TABLE_KEY = 86
local PARAM_TABLE_PREFIX = "SRTL_RST_"

local MAV_SEVERITY = {EMERGENCY=0, ALERT=1, CRITICAL=2, ERROR=3, WARNING=4, NOTICE=5, INFO=6, DEBUG=7}

-- SmartRTL flight mode number for the vehicles that support it
local SMART_RTL_MODE = { [1] = 12, -- Rover
                         [2] = 21, -- Copter
                         [13] = 21 } -- Heli

-- add a parameter and bind it to a variable
local function bind_add_param(name, idx, default_value)
   assert(param:add_param(PARAM_TABLE_KEY, idx, name, default_value), string.format('could not add param %s', name))
   return Parameter(PARAM_TABLE_PREFIX .. name)
end

-- setup script specific parameters
assert(param:add_table(PARAM_TABLE_KEY, PARAM_TABLE_PREFIX, 2), 'could not add param table')

--[[
  // @Param: SRTL_RST_ENABLE
  // @DisplayName: SmartRTL reset enable
  // @Description: Enable resetting the SmartRTL return point from an RC switch
  // @Values: 0:Disabled,1:Enabled
  // @User: Standard
--]]
local SRTL_RST_ENABLE = bind_add_param('ENABLE', 1, 1)

--[[
  // @Param: SRTL_RST_FN
  // @DisplayName: SmartRTL reset RC function
  // @Description: RCx_OPTION function number of the switch that resets the SmartRTL return point when moved to the high position. Use one of the scripting functions (300 to 307)
  // @Range: 300 307
  // @User: Standard
--]]
local SRTL_RST_FN = bind_add_param('FN', 2, 300)

local smart_rtl_mode = SMART_RTL_MODE[FWVersion:type()]
assert(smart_rtl_mode ~= nil, "SmartRTL is not supported on this vehicle")

local last_switch_pos  -- switch position seen on the previous update, nil until first read
local last_warn_ms = uint32_t(0)

-- send a warning to the GCS at most every 10 seconds
local function warn(msg)
   local now_ms = millis()
   if now_ms - last_warn_ms > 10000 then
      gcs:send_text(MAV_SEVERITY.WARNING, "SmartRTL reset: " .. msg)
      last_warn_ms = now_ms
   end
end

-- reset the SmartRTL return point to the current position
local function reset_return_point()
   if not smart_rtl:enabled() then
      gcs:send_text(MAV_SEVERITY.WARNING, "SmartRTL reset: SmartRTL disabled")
      return
   end
   if not arming:is_armed() then
      gcs:send_text(MAV_SEVERITY.WARNING, "SmartRTL reset: not armed")
      return
   end
   if vehicle:get_mode() == smart_rtl_mode then
      gcs:send_text(MAV_SEVERITY.WARNING, "SmartRTL reset: not while returning")
      return
   end
   -- on success the SmartRTL library reports "SmartRTL: return point reset"
   if not smart_rtl:reset_home() then
      gcs:send_text(MAV_SEVERITY.WARNING, "SmartRTL reset: failed, no position")
   end
end

local function update()
   if SRTL_RST_ENABLE:get() ~= 1 then
      last_switch_pos = nil
      return update, 1000
   end

   local switch = rc:find_channel_for_option(math.floor(SRTL_RST_FN:get()))
   if not switch then
      warn(string.format("no RC switch with option %d", math.floor(SRTL_RST_FN:get())))
      last_switch_pos = nil
      return update, 1000
   end

   -- trigger once when the switch is moved to the high position
   -- a switch that is already high when the script starts does not trigger
   local switch_pos = switch:get_aux_switch_pos()
   if switch_pos == 2 and last_switch_pos ~= nil and last_switch_pos ~= 2 then
      reset_return_point()
   end
   last_switch_pos = switch_pos

   return update, 100
end

gcs:send_text(MAV_SEVERITY.INFO, "SmartRTL reset: loaded")

return update, 1000
