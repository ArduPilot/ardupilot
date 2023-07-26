-- mount-test.lua: allows the winch to be deployed or retracted at a fixed speed using an auxiliary switch
--
-- How To Use
--   1. set RCx_OPTION to 300 to enable controlling the winch rate from an auxiliary switch
--   2. optionally set WINCH_RATE_UP to the fixed retract speed (in m/s)
--   3. optionally set WINCH_RATE_DN to the fixed deploy speed (in m/s)
--   4. raise the RC auxiliary switch to retract the winch's line
--   5. lower the RC auxiliary switch to deploy the winch's line
--   6. center the RC auxiliary switch to stop the winch
-- Alternatively a servo *output* can be used in place of the auxiliary switch input by setting WINCH_SRV_SRC_FN to match a servo channel's function.  For example
--   a. set SERVO10_FUNCTION = 28 (Gripper)
--   b. set WINCH_SRV_SRC_FN to 28
--   c. use Mission Planner's Data screen's Servo/Relay tab to set the SERVO10 output to Low, Mid or High values
--   Note: the full list of SERVOx_FUNCTION values that will work are 0:None, 1:Manual, 22:SprayerPump, 23:SprayerSpinner, 28/Gripper, 51:RCIN1 to 66:RCIN16
--

-- global definitions
local UPDATE_INTERVAL_MS = 100

-- add new parameters
local PARAM_TABLE_KEY = 80
assert(param:add_table(PARAM_TABLE_KEY, "WINCH_", 3), "could not add param table")
assert(param:add_param(PARAM_TABLE_KEY, 1, "RATE_UP", 0.5), "could not add WINCH_RATE_UP param")
assert(param:add_param(PARAM_TABLE_KEY, 2, "RATE_DN", 2), "could not add WINCH_RATE_DN param")
assert(param:add_param(PARAM_TABLE_KEY, 3, "SRV_SRC_FN", -1), "could not add WINCH_SRV_SRC_FN param")

local winch_rate_up = Parameter("WINCH_RATE_UP")
local winch_rate_dn = Parameter("WINCH_RATE_DN")
local winch_srv_src_fn = Parameter("WINCH_SRV_SRC_FN")

-- local variables and definitions
local last_rc_switch_pos = -1   -- last known rc switch position.  Used to detect change in RC switch position

-- the main update function
function update()

  local rc_switch_pos = 1   -- default to middle position

  -- check if servo output is used (as an input)
  if winch_srv_src_fn:get() > 0 then
    if not SRV_Channels:find_channel(winch_srv_src_fn:get()) then
      gcs:send_text(3, string.format("Winch: SERVOx_FUNCTION = %d not found", winch_srv_src_fn:get()))    -- MAV_SEVERITY_ERROR
      return update, 10000  -- check again in 10 seconds
    end
    local output_pwm = SRV_Channels:get_output_pwm(winch_srv_src_fn:get())
    if output_pwm <= 0 then
      -- servo output all zero so ignore
      return update, UPDATE_INTERVAL_MS
    end
    if output_pwm <= 1300 then
      rc_switch_pos = 0   -- LOW
    elseif output_pwm >= 1700 then
      rc_switch_pos = 2   -- HIGH
    end
  else
    -- find RC channel used to control winch
    local rc_switch_ch = rc:find_channel_for_option(300) --scripting ch 1
    if (rc_switch_ch == nil) then
      gcs:send_text(3, "Winch: RCx_OPTION = 300 not set")    -- MAV_SEVERITY_ERROR
      return update, 10000  -- check again in 10 seconds
    end

    -- get RC switch position
    rc_switch_pos = rc_switch_ch:get_aux_switch_pos()
  end

  -- initialise RC switch at startup
  if last_rc_switch_pos == -1 then
    last_rc_switch_pos = rc_switch_pos
  end

  -- check if user has moved RC switch
  if rc_switch_pos == last_rc_switch_pos then
    return update, UPDATE_INTERVAL_MS
  end
  last_rc_switch_pos = rc_switch_pos

  -- set winch rate based on switch position
  if rc_switch_pos == 0 then -- LOW, deploy winch line
    local rate_dn = math.abs(winch_rate_dn:get())
    winch:set_desired_rate(rate_dn)
    gcs:send_text(6, string.format("Winch: lowering at %.1f m/s", rate_dn))
  end
  if rc_switch_pos == 1 then -- MIDDLE, stop winch
    winch:set_desired_rate(0)
    gcs:send_text(6, "Winch: stopped")
  end
  if rc_switch_pos == 2 then -- HIGH, retract winch line
    local rate_up = math.abs(winch_rate_up:get())
    winch:set_desired_rate(-rate_up)
    gcs:send_text(6, string.format("Winch: raising at %.1f m/s", rate_up))
  end

  return update, UPDATE_INTERVAL_MS
end

return update()
