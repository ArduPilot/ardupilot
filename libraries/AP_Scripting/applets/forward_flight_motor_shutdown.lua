-- This is a script that stops motors in forward flight, for use on tiltrotors and tailsitters
-- Thanks to PteroDynamics for supporting the development of this script
-- Set the motor numbers to stop and configure a RCx_OPTION to 300 to enable and disable
-- Throttle thresholds also allow automatic enable and disable of stop motors
-- slew up and down times allow to configure how fast the motors are disabled and re-enabled



-- Config

-- Motors numbers to stop
local stop_motors = {2,4}

-- motors should be shutdown if throttle goes lower than this value (%), set to > 100 to disable auto enable/disable
local throttle_off_threshold = 50

-- motors should be re-enabled if throttle higher than this value (%)
local throttle_on_threshold = 75

local slew_down_time = 5 -- seconds
local slew_up_time = 1 -- seconds

-- end of config

local switch = assert(rc:find_channel_for_option(300),"Lua: Could not find switch")

-- read spin min param, we set motors to this PWM to stop them
local pwm_min = assert(param:get("Q_M_PWM_MIN"),"Lua: Could not read Q_M_PWM_MIN")
local pwm_max = assert(param:get("Q_M_PWM_MAX"),"Lua: Could not read Q_M_PWM_MAX")

-- calculate the slew up and down steps to achieve the given slew time
local pwm_range = pwm_max - pwm_min
local slew_down = (-pwm_range / slew_down_time) * (10/1000)
local slew_up = (pwm_range / slew_up_time) * (10/1000)

if throttle_off_threshold < 100 then
  assert((throttle_off_threshold < throttle_on_threshold) and (throttle_on_threshold < 100), "throttle on and off thresholds not configured correctly")
end

for i = 1, #stop_motors do
  -- Check for a valid motor number
  assert(stop_motors[i] >= 1 and stop_motors[i] <= 12, string.format("Lua: motor %i not valid",stop_motors[i]))
end

-- find any motors enabled, populate channels to stop and channels to run
-- run channels provide a average motor PWM to slew down from
local stop_motor_chan = {}
local run_motor_fun = {}
for i = 1, 12 do

  local output_function
  if i <= 8 then
    output_function = i+32
  else
    output_function = i+81-8
  end

  local temp_chan = SRV_Channels:find_channel(output_function)

  local should_stop = false
  for j = 1, #stop_motors do
    if i == stop_motors[j] then
      should_stop = true
      break
    end
  end

  if should_stop then
    assert(temp_chan, string.format("Lua: Could not find motor %i",i))
    table.insert(stop_motor_chan, temp_chan)
  elseif temp_chan then
    table.insert(run_motor_fun, output_function)
  end
end
assert(#stop_motor_chan == #stop_motors, "Lua: could not find all motors to stop")
assert(#run_motor_fun > 0, "Lua: cannot stop all motors")

-- keep track of last time in a VTOL mode, this allows to delay switching after a transition/assist
local last_vtol_active = uint32_t(0)

-- current action
local script_enabled = false
local motors_disabled = false
local slew
local slew_pwm
function update()

  if switch:get_aux_switch_pos() == 2 then
    if not script_enabled then
      gcs:send_text(0, "Lua: Forward flight motor shutdown enabled")
    end
    script_enabled = true
  else
    if script_enabled then
      gcs:send_text(0, "Lua: Forward flight motor shutdown disabled")
    end
    script_enabled = false
  end

  if quadplane:in_vtol_mode() or quadplane:in_assisted_flight() or not arming:is_armed() then
    -- in a VTOL mode, nothing to do
    last_vtol_active = millis()
    motors_disabled = false
    return update, 1000 -- reschedule at 1hz
  end

  -- script not enabled
  if not script_enabled then
    motors_disabled = false
    return update, 1000 -- reschedule at 1hz
  end

  -- in forward flight and enabled with switch, if armed then check that we are at least 10s past transition
  if (millis() - last_vtol_active) < 10000 then
    -- armed and have not been in a VTOL mode for longer than 10s
    motors_disabled = false
    return update, 1000 -- reschedule at 10hz
  end

  -- check throttle level to see if motors should be disabled or enabled
  local throttle = SRV_Channels:get_output_scaled(70)

  if motors_disabled and (throttle > throttle_on_threshold) then
    gcs:send_text(0, "Lua: Throttle high motors enabled")
    slew = slew_up

  elseif (not motors_disabled) and (throttle < throttle_off_threshold) then
    slew_pwm = pwm_max
    motors_disabled = true
    gcs:send_text(0, "Lua: Throttle low motors disabled")
    slew = slew_down
  end

  if not motors_disabled then
    return update, 10
  end

  local average_pwm = 0
  for i = 1, #run_motor_fun do
    average_pwm = average_pwm + SRV_Channels:get_output_pwm(run_motor_fun[i])
  end
  average_pwm = average_pwm / #run_motor_fun

  slew_pwm = slew_pwm + slew
  if (slew > 0) and (slew_pwm > average_pwm) then
    -- slewed back up past the output of other motors, stop overriding
    motors_disabled = false
    slew_pwm = pwm_max
    return update, 1000
  end

  -- never slew lower than min
  slew_pwm = math.max(slew_pwm, pwm_min)

  -- never output higher than other motors
  local output_pwm = math.min(slew_pwm, average_pwm)

  -- make sure a integer
  output_pwm = math.floor(output_pwm + 0.5)

  for i = 1, #stop_motor_chan do
    -- override for 15ms, should be called every 10ms when active
    -- using timeout means if the script dies the timeout will expire and all motors will come back
    -- we cant leave the vehicle in a un-flyable state
    SRV_Channels:set_output_pwm_chan_timeout(stop_motor_chan[i],output_pwm,15)
  end

  return update, 10 -- reschedule at 100hz
end

return update() -- run immediately before starting to reschedule
