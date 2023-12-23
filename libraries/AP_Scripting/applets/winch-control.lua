-- winch-control.lua: allows the winch to be deployed or retracted at a fixed speed using an auxiliary switch
--
-- How To Use
--   1. set RCx_OPTION to 300 to enable controlling the winch rate from an auxiliary switch
--   2. set WINCH_RATE_UP to the fixed retract speed (in m/s)
--   3. set WINCH_RATE_DN to the fixed deploy speed (in m/s)
--   4. raise the RC auxiliary switch to retract the winch's line
--   5. lower the RC auxiliary switch to deploy the winch's line
--   6. center the RC auxiliary switch to stop the winch
-- Alternatively Mission Planner's Aux Function screen can be used in place of an actual RC switch
--

-- global definitions
local MAV_SEVERITY = {EMERGENCY=0, ALERT=1, CRITICAL=2, ERROR=3, WARNING=4, NOTICE=5, INFO=6, DEBUG=7}
local PARAM_TABLE_KEY = 80
local PARAM_TABLE_PREFIX = "WINCH_"

-- bind a parameter to a variable
function bind_param(name)
   local p = Parameter()
   assert(p:init(name), string.format("WinchControl: could not find %s parameter", name))
   return p
end

-- add a parameter and bind it to a variable
function bind_add_param(name, idx, default_value)
   assert(param:add_param(PARAM_TABLE_KEY, idx, name, default_value), string.format("WinchControl: could not add param %s", name))
   return bind_param(PARAM_TABLE_PREFIX .. name)
end

-- setup quicktune specific parameters
assert(param:add_table(PARAM_TABLE_KEY, PARAM_TABLE_PREFIX, 3), "WinchControl: could not add param table")

--[[
  // @Param: WINCH_RATE_UP
  // @DisplayName: WinchControl Rate Up
  // @Description: Maximum rate when retracting line
  // @Range: 0.1 5.0
  // @User: Standard
--]]
local WINCH_RATE_UP = bind_add_param('RATE_UP', 1, 0.5)

--[[
  // @Param: WINCH_RATE_DN
  // @DisplayName: WinchControl Rate Down
  // @Description: Maximum rate when releasing line
  // @Range: 0.1 5.0
  // @User: Standard
--]]
local WINCH_RATE_DN = bind_add_param('RATE_DN', 2, 2.0)

--[[
  // @Param: WINCH_RC_FUNC
  // @DisplayName: Winch Rate Control RC function
  // @Description: RCn_OPTION number to use to control winch rate
  // @Values: 300:Scripting1, 301:Scripting2, 302:Scripting3, 303:Scripting4, 304:Scripting5, 305:Scripting6, 306:Scripting7, 307:Scripting8
  // @User: Standard
--]]
local WINCH_RC_FUNC = bind_add_param('RC_FUNC', 3, 300)

-- local variables and definitions
local UPDATE_INTERVAL_MS = 100
local last_rc_switch_pos = -1   -- last known rc switch position.  Used to detect change in RC switch position

-- initialisation
gcs:send_text(MAV_SEVERITY.INFO, "WinchControl: started")

-- the main update function
function update()

  -- get RC switch position
  local rc_switch_pos = rc:get_aux_cached(WINCH_RC_FUNC:get())
  if not rc_switch_pos then
    -- if rc switch has never been set the return immediately
    return update, UPDATE_INTERVAL_MS
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
    local rate_dn = math.abs(WINCH_RATE_DN:get())
    winch:set_desired_rate(rate_dn)
    gcs:send_text(6, string.format("Winch: lowering at %.1f m/s", rate_dn))
  end
  if rc_switch_pos == 1 then -- MIDDLE, stop winch
    winch:set_desired_rate(0)
    gcs:send_text(6, "Winch: stopped")
  end
  if rc_switch_pos == 2 then -- HIGH, retract winch line
    local rate_up = math.abs(WINCH_RATE_UP:get())
    winch:set_desired_rate(-rate_up)
    gcs:send_text(6, string.format("Winch: raising at %.1f m/s", rate_up))
  end

  return update, UPDATE_INTERVAL_MS
end

return update()
