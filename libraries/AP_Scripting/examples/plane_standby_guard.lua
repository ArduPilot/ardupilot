--[[
   Stand-By guard for a redundant ("ride along") flight controller.

   Install this ONLY on a flight controller that is meant to boot as the backup.

   It closes two gaps in the RC-switch path to AUX_FUNC STANDBY (76):

   1. Boot state. RC_Channel::init_aux() runs before any RC input exists and,
      when it cannot read a valid switch position, defaults it to LOW - which
      *releases* standby. A backup controller therefore boots believing it is
      the controller in command.

   2. A standby signal that is present but meaningless. RC_Channels::read_aux_all()
      does nothing at all while there is no valid RC input, and a channel floating
      at mid-stick reads as MIDDLE, which the vehicle handler treats the same as
      "released". Either way the unit quietly ends up out of standby with nothing
      reporting it.

   This script mirrors the supervisor's channel onto the flag itself, so that the
   only way to be out of standby is for the channel to be readable AND
   unambiguously low:

       channel clearly high (>1800us)  -> standby asserted
       channel clearly low  (<1200us)  -> standby released
       anything else, no RC input,
       or no channel configured        -> standby asserted, and say so

   Mirroring rather than merely defaulting matters: if the guard asserted standby
   while RC was invalid and RC then returned already sitting low, the aux path
   would see no *change* of switch position and would never run, leaving the
   vehicle stuck in standby. Driving the flag to match the channel avoids that.
--]]

local PARAM_TABLE_KEY = 82
local PARAM_TABLE_PREFIX = "STBY_"

local AUX_FUNC_STANDBY = 76
local AUX_LOW = 0
local AUX_HIGH = 2

local PWM_HIGH = 1800
local PWM_LOW = 1200
local PWM_MIN = 800     -- matches RC_MIN_LIMIT_PWM in RC_Channel.cpp
local PWM_MAX = 2200    -- matches RC_MAX_LIMIT_PWM

local UPDATE_MS = 200
local SEVERITY_INFO = 6
local SEVERITY_WARNING = 4

assert(param:add_table(PARAM_TABLE_KEY, PARAM_TABLE_PREFIX, 1),
       "StandbyGuard: could not add param table")

--[[
  // @Param: STBY_GUARD
  // @DisplayName: Standby guard enable
  // @Description: Mirror the RCx_OPTION=76 channel onto the standby flag, asserting standby at boot and whenever that channel is missing, unreadable or ambiguous. Enable on a backup flight controller only; set to 0 on the controller that boots in command.
  // @Values: 0:Disabled,1:Enabled
--]]
assert(param:add_param(PARAM_TABLE_KEY, 1, "GUARD", 1),
       "StandbyGuard: could not add STBY_GUARD")

local guard_enable = Parameter()
assert(guard_enable:init("STBY_GUARD"), "StandbyGuard: STBY_GUARD missing")

local standby_ch = nil
local last_reason = nil

-- find the channel carrying the standby function, so no second parameter is
-- needed and the two can never be configured inconsistently
local function find_standby_channel()
   for i = 1, 16 do
      local opt = param:get(string.format("RC%u_OPTION", i))
      if opt ~= nil and math.floor(opt) == AUX_FUNC_STANDBY then
         return i
      end
   end
   return nil
end

-- decide what the flag should be, and why. returns want_standby, reason
-- reason is nil when the supervisor's channel is being followed normally
local function decide()
   if standby_ch == nil then
      standby_ch = find_standby_channel()
      if standby_ch == nil then
         return true, "no RCx_OPTION=76 channel configured"
      end
      gcs:send_text(SEVERITY_INFO,
                    string.format("StandbyGuard: watching RC%u", standby_ch))
   end

   -- this is the same gate RC_Channels::read_aux_all() uses; while it is false
   -- the aux function cannot run at all
   if not rc:has_valid_input() then
      return true, "no valid RC input"
   end

   local pwm = rc:get_pwm(standby_ch)
   if pwm == nil or pwm <= PWM_MIN or pwm >= PWM_MAX then
      return true, "standby channel unreadable"
   end
   if pwm >= PWM_HIGH then
      return true, nil
   end
   if pwm <= PWM_LOW then
      return false, nil
   end
   -- keep under the 50-character MAVLink statustext limit
   return true, string.format("standby ch %uus: ambiguous", pwm)
end

function update()
   if guard_enable:get() ~= 1 then
      return update, 1000
   end

   local want_standby, reason = decide()

   if reason ~= last_reason then
      if reason ~= nil then
         gcs:send_text(SEVERITY_WARNING, "StandbyGuard: " .. reason)
      else
         gcs:send_text(SEVERITY_INFO, "StandbyGuard: following RC normally")
      end
      last_reason = reason
   end

   -- only touch the flag when the effective state differs, so we neither spam
   -- the log nor fight the supervisor's own switch handling
   local cached = rc:get_aux_cached(AUX_FUNC_STANDBY)
   local want_pos = want_standby and AUX_HIGH or AUX_LOW
   if cached == nil or cached ~= want_pos then
      rc:run_aux_function(AUX_FUNC_STANDBY, want_pos)
   end

   return update, UPDATE_MS
end

-- Boot state: assert standby before any RC has been seen, so the window between
-- power-up and the first debounced switch reading is spent in standby rather
-- than in command.
if guard_enable:get() == 1 then
   rc:run_aux_function(AUX_FUNC_STANDBY, AUX_HIGH)
   gcs:send_text(SEVERITY_WARNING, "StandbyGuard: asserted standby at boot")
else
   gcs:send_text(SEVERITY_INFO, "StandbyGuard: disabled (STBY_GUARD=0)")
end

return update, UPDATE_MS
