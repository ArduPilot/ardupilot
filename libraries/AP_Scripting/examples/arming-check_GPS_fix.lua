-- This script checks the current GPS RTK fix status. If in Copter Auto mode, it does not allow arming. Any other mode, it will report RTK fix isn't there.
-- If RTK fix goes in flight during auto mission, it will switch to guided mode to climb to some height.

local MODE_AUTO = 3
local MODE_LOITER = 5
local MODE_GUIDED = 4

local guided_climbing_mode = false
local update_rate_ms = 250
local sent_rtk_fix_message = false
local saved_location_height
local last_message_sent_ms = 0

auth_id = arming:get_aux_auth_id()

PARAM_TABLE_KEY = 122
PARAM_TABLE_PREFIX = "RTK_FS_"

-- bind a parameter to a variable given
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

-- Setup RTK FS Parameters
assert(param:add_table(PARAM_TABLE_KEY, PARAM_TABLE_PREFIX, 5), 'could not add RTK_FS param table')

--[[
  // @Param: RTK_FS_ENABLE
  // @DisplayName: Enable GPS RTK Failsafe features
  // @Description: Enable GPS RTK Failsafe features
  // @Values: 0:Disabled,1:Enabled
  // @User: Standard
--]]
local RTK_FS_ENABLE     = bind_add_param('ENABLE',     1, 0)

--[[
  // @Param: RTK_FS_HEIGHT
  // @DisplayName: RTK Failsafe height gain
  // @Description: Vehicle will gain this many meters in height after RTK fixed status is lost in Auto mode
  // @Range: 1 5
  // @User: Standard
--]]
local RTK_FS_HEIGHT     = bind_add_param('HEIGHT',    2, 3)


function handle_emergency_climb()
      local current_loc = ahrs:get_location()
      current_loc:alt(saved_location_height + RTK_FS_HEIGHT:get() * 100)
      vehicle:set_target_location(current_loc)
end

function send_message_with_timeout(msg_string, timeout_ms)
  if (millis() - last_message_sent_ms > timeout_ms) then
    gcs:send_text(0, msg_string)
    last_message_sent_ms = millis()
  end
end

function update() -- this is the loop which periodically runs

  if RTK_FS_ENABLE:get() == 0 then
    arming:set_aux_auth_passed(auth_id)
    return update, update_rate_ms
  end

  local current_gps_status = gps:status(0)
  local current_mode = vehicle:get_mode()
  if not arming:is_armed() then
    if (current_gps_status ~= gps.GPS_OK_FIX_3D_RTK_FIXED) then
      -- do not allow arming if vehicle is in auto mode but not RTK fix
      sent_rtk_fix_message = false
      if current_mode == MODE_AUTO then
        arming:set_aux_auth_failed(auth_id, "Waiting for RTK fix")
      else
        send_message_with_timeout(string.format("Waiting for RTK fix"), 10000)
        -- pass check but warn user
        arming:set_aux_auth_passed(auth_id)
      end
    else
      -- all good with the GPS
      if sent_rtk_fix_message == false then
        gcs:send_text(6, string.format("GPS has RTK Fix"))
        sent_rtk_fix_message = true
      end
      arming:set_aux_auth_passed(auth_id)
    end
    return update, update_rate_ms
  end

  if guided_climbing_mode then
    if (current_mode ~= MODE_GUIDED) then
      guided_climbing_mode = false
      gcs:send_text(0, string.format("Mode changed. Exiting RTK failsafe"))
    else
      handle_emergency_climb()
    end
    return update, update_rate_ms
  end

  -- armed
  if current_gps_status ~= gps.GPS_OK_FIX_3D_RTK_FIXED then
    if current_mode == MODE_LOITER then
      send_message_with_timeout(string.format("Attention, RTK fix lost"), 5000)
    elseif current_mode == MODE_AUTO then
      gcs:send_text(0, string.format("RTK fix lost, mission stopped"))
      guided_climbing_mode = true
      vehicle:set_mode(MODE_GUIDED)
      saved_location_height = ahrs:get_location():alt() -- record height. We don't want to go lower than this height 
    end
  end
  return update, update_rate_ms -- reschedules the loop
end

return update() -- run immediately before starting to reschedule
