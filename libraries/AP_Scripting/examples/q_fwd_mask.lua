--[[
   a script to disable a subset of tilting motors in forward flight
--]]

local PARAM_TABLE_KEY = 87
local PARAM_TABLE_PREFIX = "QFWD_"

local MAV_SEVERITY = {
    EMERGENCY = 0,
    ALERT = 1,
    CRITICAL = 2,
    ERROR = 3,
    WARNING = 4,
    NOTICE = 5,
    INFO = 6,
    DEBUG = 7
}

local k_thrust_out = 126
local k_motor1 = 33

local PERIOD_MS = 25

-- bind a parameter to a variable
function bind_param(name)
   return Parameter(name)
end

-- add a parameter and bind it to a variable
function bind_add_param(name, idx, default_value)
   assert(param:add_param(PARAM_TABLE_KEY, idx, name, default_value), string.format('could not add param %s', name))
   return bind_param(PARAM_TABLE_PREFIX .. name)
end

-- setup quicktune specific parameters
assert(param:add_table(PARAM_TABLE_KEY, PARAM_TABLE_PREFIX, 8), 'could not add param table')

--[[
  // @Param: QFWD_MASK
  // @DisplayName: VTOL Tilt forward mask
  // @Description: Mask of VTOL motors to disable in forward flight
  // @Bitmask: 0:Motor 1, 1:Motor 2, 2:Motor 3, 3:Motor 4, 4:Motor 5, 5:Motor 6, 6:Motor 7, 7:Motor 8, 8:Motor 9, 9:Motor 10, 10:Motor 11, 11:Motor 12
  // @User: Standard
--]]
local QFWD_MASK = bind_add_param('MASK', 1, 0)

--[[
  // @Param: QUIK_RC_FUNC
  // @DisplayName: VTOL tilt forward mask enable function
  // @Description: RCn_OPTION number to use to enable motor masking
  // @User: Standard
--]]
local QFWD_AUX_FUNC = bind_add_param('AUX_FUNC', 2, 300)

-- the PWM to set the VTOL motors to when disabled
local Q_M_PWM_MIN = Parameter('Q_M_PWM_MIN')

local is_masked = false

--[[
   return true if we should mask the motors
--]]
local function should_mask_motors()
   local sw_current = rc:get_aux_cached(QFWD_AUX_FUNC:get())
   if sw_current ~= 2 then
      -- function not active
      return false
   end
   if QFWD_MASK:get() == 0 then
      -- nothing to mask
      return false
   end

   -- see if VTOL motors running, don't mask if running
   local thrust = SRV_Channels:get_output_scaled(k_thrust_out)
   if thrust > 0 then
      return false
   end

   if quadplane:in_vtol_mode() or quadplane:in_assisted_flight() then
      return false
   end

   return true
end

local function update_motors()
   local should_mask = should_mask_motors()
   if should_mask ~= is_masked then
      if should_mask then
         gcs:send_text(MAV_SEVERITY.INFO, "QFWD mask enabled")
      else
         gcs:send_text(MAV_SEVERITY.INFO, "QFWD mask disabled")
      end
      is_masked = should_mask
   end
   if not should_mask then
      -- rely on timeout
      return
   end
   local mask = QFWD_MASK:get()
   for i = 0,7 do
      if ((1<<i) & mask) ~= 0 then
         local id = k_motor1 + i
         local chan = SRV_Channels:find_channel(id)
         if chan then
            SRV_Channels:set_output_pwm_chan_timeout(chan, Q_M_PWM_MIN:get(), PERIOD_MS*4)
         end
      end
   end
end

function update()
   update_motors()
   return update, PERIOD_MS
end

gcs:send_text(MAV_SEVERITY.INFO, "Loaded QFWD masking")

return update, PERIOD_MS
