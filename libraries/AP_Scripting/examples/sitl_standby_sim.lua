-- This script is a allow to switch the standby function by the switch of the remote controller on 2 simulated fcus.

-- variable to limit the number of messages sent to GCS
local last_rc_input = 0

-- constants
local switch_high = 2
local switch_low = 0

-- Standby function number in ArduPilot
local standby_function = 76
-- RC pin to use to select the FCU
local rc_input_pin = 8


-- for fast param access it is better to get a param object,
-- this saves the code searching for the param by name every time
local JSON_MASTER = Parameter()
if not JSON_MASTER:init('SIM_JSON_MASTER') then
  gcs:send_text(6, 'get SIM_JSON_MASTER failed')
end

local SYSID_THISMAV = Parameter()
if not SYSID_THISMAV:init('SYSID_THISMAV') then
  gcs:send_text(6, 'get SYSID_THISMAV failed')
end

local sysid = SYSID_THISMAV:get()

gcs:send_text(6, 'LUA: Standby LUA loaded')

-- The loop check the value of the switch of the remote controller and switch the standby function of the simulated fcus.
-- FCU should have SYSID_THISMAV set to 1 or 2.
-- The switch is on the channel 8 (default) of the remote controller.
-- fcu1 and fcu2 have mirrored standby function, so only one is active at a time.
-- As this is for simulation, the SIM_JSON_MASTER param is also update to select which fcu control the motors.

function update() -- this is the loop which periodically runs
  rc_input = rc:get_pwm(rc_input_pin)

  if rc_input == nil then
    gcs:send_text(6, 'LUA: rc_input is nil')
    return update, 1000 -- reschedules the loop
  end

  if rc_input ~= last_rc_input then
      if sysid == 2 then
          if rc_input > 1500 then
              if not JSON_MASTER:set(0) then
                  gcs:send_text(6, string.format('LUA: failed to set SIM_JSON_MASTER'))
              else
                  rc:run_aux_function(standby_function, switch_high)
                  gcs:send_text(6, string.format('LUA: set SIM_JSON_MASTER to 0'))
              end
          else
              if not JSON_MASTER:set(1) then
                  gcs:send_text(6, string.format('LUA: failed to set SIM_JSON_MASTER'))
              else
                  rc:run_aux_function(standby_function, switch_low)
                  gcs:send_text(6, string.format('LUA: set SIM_JSON_MASTER to 1'))
              end
          end
      end
      if sysid == 1 then
          if rc_input > 1500 then
              if not JSON_MASTER:set(0) then
                  gcs:send_text(6, string.format('LUA: failed to set SIM_JSON_MASTER'))
              else
                  rc:run_aux_function(standby_function, switch_low)
                  gcs:send_text(6, string.format('LUA: set SIM_JSON_MASTER to 0'))
              end
          else
              if not JSON_MASTER:set(1) then
                  gcs:send_text(6, string.format('LUA: failed to set SIM_JSON_MASTER'))
              else
                  rc:run_aux_function(standby_function, switch_high)
                  gcs:send_text(6, string.format('LUA: set SIM_JSON_MASTER to 1'))
              end
          end
      end
      last_rc_input = rc_input
  end

  return update, 1000 -- reschedules the loop
end

return update() -- run immediately before starting to reschedule
