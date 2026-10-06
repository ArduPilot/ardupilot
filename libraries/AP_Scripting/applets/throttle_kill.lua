--[[
   script to allow for engine kill on turbine engines using an auxilliary function
--]]

local MAV_SEVERITY = {EMERGENCY=0, ALERT=1, CRITICAL=2, ERROR=3, WARNING=4, NOTICE=5, INFO=6, DEBUG=7}

local PARAM_TABLE_KEY = 102
local PARAM_TABLE_PREFIX = "THR_"

-- add a parameter and bind it to a variable
function bind_add_param(name, idx, default_value)
   assert(param:add_param(PARAM_TABLE_KEY, idx, name, default_value), string.format('could not add param %s', name))
   return Parameter(PARAM_TABLE_PREFIX .. name)
end

-- setup quicktune specific parameters
assert(param:add_table(PARAM_TABLE_KEY, PARAM_TABLE_PREFIX, 5), 'could not add param table')

local PERIOD_MS = 50

--[[
  // @Param: THR_KILL_FUNC
  // @DisplayName: AUX function to kill engine
  // @Description: AUX function to kill engine. This can be activated either with a RCn_OPTION and a R/C switch or with a ground station auxilliary function
  // @Range: 300 307
  // @User: Standard
--]]
local THR_KILL_FUNC = bind_add_param('KILL_FUNC', 1, 300)

--[[
  // @Param: THR_KILL_PWM
  // @DisplayName: PWM on kill active
  // @Description: PWM on kill active
  // @Range: 800 2200
  // @User: Standard
--]]
local THR_KILL_PWM = bind_add_param('KILL_PWM', 2, 900)

--[[
  // @Param: THR_KILL_CHAN
  // @DisplayName: output channel to change on throttle kill
  // @Description: output channel to change on throttle kill, a value of zero disables the feature
  // @Range: 0 32
  // @User: Standard
--]]
local THR_KILL_CHAN = bind_add_param('KILL_CHAN', 3, 0)

--[[
  // @Param: THR_KILL_VAL
  // @DisplayName: auxilliary value to kill throttle
  // @Description: auxilliary value to kill throttle. Set to 2 to kill the throttle when the auxilliary is high. Set to 0 to kill when auxilliary is low
  // @Range: 0 2
  // @User: Standard
--]]
local THR_KILL_VAL = bind_add_param('KILL_VAL', 4, 2)

--[[
  // @Param: THR_KILL_DEF
  // @DisplayName: throttle kill default value
  // @Description: throttle kill default value. The default auxilliary function position on boot
  // @Range: 0 2
  // @User: Standard
--]]
local THR_KILL_DEF = bind_add_param('KILL_DEF', 5, 0)

local last_active = false

--[[
   update the throttle kill from AUX function
--]]
local function update_kill()
   if THR_KILL_CHAN:get() <= 0 then
      return
   end
   local sw_pos = rc:get_aux_cached(THR_KILL_FUNC:get())
   if not sw_pos then
      sw_pos = THR_KILL_DEF:get()
   end
   if sw_pos == THR_KILL_VAL:get() then
      SRV_Channels:set_output_pwm_chan_timeout(THR_KILL_CHAN:get()-1, math.floor(THR_KILL_PWM:get()), 2*PERIOD_MS)
      if not last_active then
         last_active = true
         gcs:send_text(MAV_SEVERITY.WARNING, "Engine disabled")
      end
   else
      if last_active then
         gcs:send_text(MAV_SEVERITY.WARNING, "Engine enabled")
      end
      last_active = false
   end
end

gcs:send_text(MAV_SEVERITY.INFO, "Engine kill script started")

local function update()
   update_kill()
   return update, PERIOD_MS
end

return update, PERIOD_MS
