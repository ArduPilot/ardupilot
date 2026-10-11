--[[
   Switch the EKF to an ExternalNav source set when GNSS is lost or degraded
   while armed, then watch the ExternalNav source and change mode if its
   quality drops, a time limit is reached, or it stops.

   See gnss-loss-extnav-fallback.md for setup and testing.
--]]

local PARAM_TABLE_KEY = 128
local PARAM_TABLE_PREFIX = "GNSSFB_"

local MAV_SEVERITY = {EMERGENCY=0, ALERT=1, CRITICAL=2, ERROR=3, WARNING=4, NOTICE=5, INFO=6, DEBUG=7}

local LOOP_MS = 100
local SOURCE_SET_PRIMARY = 0
local GPS_SILENT_MS = 1000     -- no message from the GPS for this long counts as no fix
local EKF_SOURCE_EXTNAV = 6
local FIRMWARE_COPTER = 2
local FIRMWARE_HELI = 13

-- add a parameter and bind it to a variable
local function bind_add_param(name, idx, default_value)
   assert(param:add_param(PARAM_TABLE_KEY, idx, name, default_value), string.format('could not add param %s', name))
   return Parameter(PARAM_TABLE_PREFIX .. name)
end

local vehicle_type = FWVersion:type()
if vehicle_type ~= FIRMWARE_COPTER and vehicle_type ~= FIRMWARE_HELI then
   gcs:send_text(MAV_SEVERITY.ERROR, "GNSSFB: Copter only, script stopped")
   return
end

assert(param:add_table(PARAM_TABLE_KEY, PARAM_TABLE_PREFIX, 11), 'could not add param table')

--[[
  // @Param: GNSSFB_ENABLE
  // @DisplayName: GNSS fallback enable
  // @Description: Enable switching to the ExternalNav source set when GNSS is lost or degraded while armed
  // @Values: 0:Disabled,1:Enabled
  // @User: Standard
--]]
local GNSSFB_ENABLE = bind_add_param('ENABLE', 1, 0)

--[[
  // @Param: GNSSFB_SRC_SET
  // @DisplayName: GNSS fallback source set
  // @Description: EKF source set to switch to when GNSS is lost or degraded. Its EK3_SRCn_POSXY should be ExternalNav
  // @Values: 2:Secondary (EK3_SRC2_),3:Tertiary (EK3_SRC3_)
  // @User: Standard
--]]
local GNSSFB_SRC_SET = bind_add_param('SRC_SET', 2, 2)

--[[
  // @Param: GNSSFB_LOSS_TIME
  // @DisplayName: GNSS fallback loss time
  // @Description: Time the primary GPS must report less than a 3D fix, or send nothing, before switching. A GPS that sends nothing for 1 second counts as having no fix
  // @Range: 0.1 5
  // @Units: s
  // @User: Standard
--]]
local GNSSFB_LOSS_TIME = bind_add_param('LOSS_TIME', 3, 0.3)

--[[
  // @Param: GNSSFB_SAT_MIN
  // @DisplayName: GNSS fallback minimum satellites
  // @Description: The primary GPS is degraded while it reports fewer satellites than this. 0 disables this check
  // @Range: 0 30
  // @User: Standard
--]]
local GNSSFB_SAT_MIN = bind_add_param('SAT_MIN', 4, 6)

--[[
  // @Param: GNSSFB_HACC_MAX
  // @DisplayName: GNSS fallback maximum horizontal accuracy
  // @Description: The primary GPS is degraded while its reported horizontal accuracy is worse than this. 0 disables this check
  // @Range: 0 100
  // @Units: m
  // @User: Standard
--]]
local GNSSFB_HACC_MAX = bind_add_param('HACC_MAX', 5, 5)

--[[
  // @Param: GNSSFB_DEGR_TIME
  // @DisplayName: GNSS fallback degraded time
  // @Description: Time the primary GPS must be degraded (see GNSSFB_SAT_MIN and GNSSFB_HACC_MAX) before switching
  // @Range: 0.1 30
  // @Units: s
  // @User: Standard
--]]
local GNSSFB_DEGR_TIME = bind_add_param('DEGR_TIME', 6, 2)

--[[
  // @Param: GNSSFB_XN_QUAL
  // @DisplayName: GNSS fallback ExternalNav minimum quality
  // @Description: After switching, change to GNSSFB_RET_MODE if the ExternalNav source reports a quality from 1 to below this value. A quality of 0 (unknown) is ignored. 0 disables this check
  // @Range: 0 100
  // @Units: %
  // @User: Standard
--]]
local GNSSFB_XN_QUAL = bind_add_param('XN_QUAL', 7, 0)

--[[
  // @Param: GNSSFB_XN_TOUT
  // @DisplayName: GNSS fallback ExternalNav loss time
  // @Description: After switching, change to GNSSFB_LND_MODE if the ExternalNav source is unhealthy, or its quality is -1 (failed) or below VISO_QUAL_MIN so the EKF ignores it, for this long
  // @Range: 0.1 10
  // @Units: s
  // @User: Standard
--]]
local GNSSFB_XN_TOUT = bind_add_param('XN_TOUT', 8, 0.5)

--[[
  // @Param: GNSSFB_XN_TMAX
  // @DisplayName: GNSS fallback ExternalNav time limit
  // @Description: After switching, change to GNSSFB_RET_MODE once the vehicle has used the ExternalNav source set for this long. 0 disables this check
  // @Range: 0 3600
  // @Units: s
  // @User: Standard
--]]
local GNSSFB_XN_TMAX = bind_add_param('XN_TMAX', 9, 0)

--[[
  // @Param: GNSSFB_RET_MODE
  // @DisplayName: GNSS fallback return mode
  // @Description: Mode number to change to when the ExternalNav quality is low or the time limit is reached. The default is RTL. -1 announces without changing mode
  // @User: Standard
--]]
local GNSSFB_RET_MODE = bind_add_param('RET_MODE', 10, 6)

--[[
  // @Param: GNSSFB_LND_MODE
  // @DisplayName: GNSS fallback land mode
  // @Description: Mode number to change to when the ExternalNav source is lost. The default is LAND. -1 announces without changing mode
  // @User: Standard
--]]
local GNSSFB_LND_MODE = bind_add_param('LND_MODE', 11, 9)

-- state, cleared on disarm
local gps_seen_good = false   -- primary GPS had a 3D fix since arming
local loss_since = nil        -- time the GPS stopped giving a 3D fix
local degraded_since = nil    -- time the GPS became degraded (or lost)
local switched_at = nil       -- time this script switched source set
local switched_set = nil      -- source set index this script switched to
local switched_mode = nil     -- vehicle mode when this script switched
local xn_bad_since = nil      -- time ExternalNav became unusable
local returned = false        -- return mode change done
local landed = false          -- land mode change done
local mode_overridden = false -- mode changed by someone else after the switch
local stood_down = false      -- source set changed by someone else after the switch
local refused_reported = false

local auth_id = arming:get_aux_auth_id()

local function say(severity, text)
   gcs:send_text(severity, "GNSSFB: " .. text)
end

local function reset()
   gps_seen_good = false
   loss_since = nil
   degraded_since = nil
   switched_at = nil
   switched_set = nil
   switched_mode = nil
   xn_bad_since = nil
   returned = false
   landed = false
   mode_overridden = false
   stood_down = false
   refused_reported = false
end

-- seconds since a millis() time; unsigned arithmetic, so safe across the uint32 wrap
local function elapsed_s(t, since)
   return (t - since):tofloat() * 0.001
end

-- returns a description of what is wrong with the setup, or nil
local function setup_error()
   local set_param = GNSSFB_SRC_SET:get()
   if set_param ~= 2 and set_param ~= 3 then
      return "SRC_SET must be 2 or 3"
   end
   if param:get('AHRS_EKF_TYPE') ~= 3 then
      return "needs AHRS_EKF_TYPE 3"
   end
   if param:get(string.format('EK3_SRC%d_POSXY', set_param)) ~= EKF_SOURCE_EXTNAV then
      return string.format("EK3_SRC%d_POSXY not ExternalNav", set_param)
   end
   if visual_odom == nil or (param:get('VISO_TYPE') or 0) == 0 then
      return "VISO_TYPE not set"
   end
   return nil
end

local function update_prearm(enabled)
   if auth_id == nil then
      return
   end
   local err = enabled and setup_error() or nil
   if err ~= nil then
      arming:set_aux_auth_failed(auth_id, "GNSSFB: " .. err)
   else
      arming:set_aux_auth_passed(auth_id)
   end
end

local function switch_source(t, set_index, reason)
   -- the setup is checked before arming, but parameters can change in flight
   local err = setup_error()
   if err ~= nil then
      if not refused_reported then
         say(MAV_SEVERITY.ERROR, string.format("%s, not switching: %s", reason, err))
         refused_reported = true
      end
      return
   end
   refused_reported = false
   ahrs:set_posvelyaw_source_set(set_index)
   switched_at = t
   switched_set = set_index
   switched_mode = vehicle:get_mode()
   say(MAV_SEVERITY.WARNING, string.format("%s, source set %d", reason, set_index + 1))
end

-- returns true if the mode change was done, or if it is disabled
local function change_mode(mode, reason)
   if mode < 0 then
      say(MAV_SEVERITY.WARNING, reason)
      return true
   end
   if vehicle:set_mode(mode) then
      say(MAV_SEVERITY.WARNING, string.format("%s, mode %d", reason, mode))
      return true
   end
   if not refused_reported then
      say(MAV_SEVERITY.ERROR, string.format("%s, mode %d refused", reason, mode))
      refused_reported = true
   end
   return false
end

-- watch the primary GPS and switch source set when it is lost or degraded
local function watch_gps(t, set_index)
   if ahrs:get_posvelyaw_source_set() ~= SOURCE_SET_PRIMARY then
      -- the pilot (or something else) has already chosen a source set
      loss_since = nil
      degraded_since = nil
      return
   end
   local instance = gps:primary_sensor()
   local silent = (millis() - gps:last_message_time_ms(instance)):tofloat() > GPS_SILENT_MS
   local no_fix = silent or gps:status(instance) < gps.GPS_OK_FIX_3D
   if not no_fix then
      gps_seen_good = true
   end
   if not gps_seen_good then
      -- no GPS fix since arming, so there is nothing to lose
      return
   end

   local degraded = no_fix
   if not degraded then
      local sat_min = GNSSFB_SAT_MIN:get()
      local hacc_max = GNSSFB_HACC_MAX:get()
      local hacc = gps:horizontal_accuracy(instance)
      degraded = (sat_min > 0 and gps:num_sats(instance) < sat_min) or
                 (hacc_max > 0 and hacc ~= nil and hacc > hacc_max)
   end
   if no_fix then
      loss_since = loss_since or t
   else
      loss_since = nil
   end
   if degraded then
      degraded_since = degraded_since or t
   else
      degraded_since = nil
   end

   if loss_since ~= nil and elapsed_s(t, loss_since) >= GNSSFB_LOSS_TIME:get() then
      switch_source(t, set_index, "GNSS lost")
   elseif degraded_since ~= nil and elapsed_s(t, degraded_since) >= GNSSFB_DEGR_TIME:get() then
      switch_source(t, set_index, "GNSS degraded")
   end
end

-- watch the ExternalNav source after the switch
local function watch_extnav(t)
   if visual_odom == nil or landed or mode_overridden then
      return
   end
   local mode = vehicle:get_mode()
   if mode ~= switched_mode and not (returned and mode == GNSSFB_RET_MODE:get()) then
      -- the pilot, the GCS or a failsafe changed mode: do not override it
      mode_overridden = true
      say(MAV_SEVERITY.WARNING, "mode changed, no more mode changes")
      return
   end

   local quality = visual_odom:quality()
   local ekf_qual_min = param:get('VISO_QUAL_MIN') or 0
   -- unusable when stale, failed (-1), or below the quality the EKF accepts
   local usable = visual_odom:healthy() and quality >= 0 and quality >= ekf_qual_min
   if usable then
      xn_bad_since = nil
   else
      xn_bad_since = xn_bad_since or t
   end

   if xn_bad_since ~= nil and elapsed_s(t, xn_bad_since) >= GNSSFB_XN_TOUT:get() then
      landed = change_mode(GNSSFB_LND_MODE:get(), "ExtNav lost")
      return
   end

   if returned then
      return
   end
   if usable and quality >= 1 and quality < GNSSFB_XN_QUAL:get() then
      returned = change_mode(GNSSFB_RET_MODE:get(), "ExtNav quality low")
      return
   end
   local tmax = GNSSFB_XN_TMAX:get()
   if tmax > 0 and elapsed_s(t, switched_at) >= tmax then
      returned = change_mode(GNSSFB_RET_MODE:get(), "ExtNav time limit")
   end
end

local function update()
   local enabled = GNSSFB_ENABLE:get() == 1

   if not arming:is_armed() then
      if switched_set ~= nil and ahrs:get_posvelyaw_source_set() == switched_set then
         ahrs:set_posvelyaw_source_set(SOURCE_SET_PRIMARY)
         say(MAV_SEVERITY.INFO, "disarmed, back to source set 1")
      end
      reset()
      update_prearm(enabled)
      return
   end

   if not enabled then
      -- keep switched_set so that disarming still restores the primary source set
      loss_since = nil
      degraded_since = nil
      xn_bad_since = nil
      return
   end

   if stood_down then
      return
   end

   local t = millis()
   if switched_set == nil then
      local set_param = GNSSFB_SRC_SET:get()
      if set_param == 2 or set_param == 3 then
         watch_gps(t, set_param - 1)
      end
      return
   end

   if ahrs:get_posvelyaw_source_set() ~= switched_set then
      -- someone else changed the source set: leave it to them until disarm
      stood_down = true
      switched_set = nil
      say(MAV_SEVERITY.WARNING, "source set changed, standing down")
      return
   end
   watch_extnav(t)
end

local function protected_wrapper()
   local success, err = pcall(update)
   if not success then
      gcs:send_text(MAV_SEVERITY.ERROR, "GNSSFB: Internal Error: " .. err)
      return protected_wrapper, 1000
   end
   return protected_wrapper, LOOP_MS
end

if auth_id == nil then
   say(MAV_SEVERITY.WARNING, "no arming check slot, setup not checked")
end
if GNSSFB_ENABLE:get() == 1 then
   say(MAV_SEVERITY.INFO, "loaded")
end

return protected_wrapper, 1000
