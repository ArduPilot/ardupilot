-- replicate functionality of plane MAV_CMD_NAV_ALTITUDE_WAIT

local altitude, descent_rate, wiggle_time
local filtered_sink_rate
local last_wiggle_ms
local idle_wiggle_stage
local active_id

local MAV_CMD_NAV_ALTITUDE_WAIT = 83
local MAV_SEVERITY_INFO = 6
local ABOVE_HOME = 1

local aileron = 4
local elevator = 19
local rudder = 21
local throttle = 70

local function assert_param_get(name)
  return assert(param:get(name), "Failed to get param: " .. name)
end

-- build a table of servo output chanel's to be overridden
local servo_outputs = {}
for i = 1, 16 do
  local servo_prefix = string.format("SERVO%i_",i)
  local function_num = assert_param_get(servo_prefix .. "FUNCTION")
  if function_num == aileron or
      function_num == elevator or
      function_num == rudder or
      function_num == throttle then

    local min = assert_param_get(servo_prefix .. "MIN")
    local max = assert_param_get(servo_prefix .. "MAX")
    local trim = assert_param_get(servo_prefix .. "TRIM")

    local servo_output = {}
    servo_output['chan'] = i-1
    servo_output['function'] = function_num
    servo_output['min'] = min
    servo_output['max'] = max
    servo_output['trim'] = trim

    table.insert(servo_outputs, servo_output)
  end
end

gcs:send_text(MAV_SEVERITY_INFO, "Nav Altitude Wait loaded")

local function update_servos()
  local servo_value
  -- move over full range for 4 seconds
  if idle_wiggle_stage ~= 0 then
      idle_wiggle_stage = idle_wiggle_stage + 5
  end

  if idle_wiggle_stage == 0 then
      servo_value = 0
  elseif idle_wiggle_stage < 50 then
      servo_value = idle_wiggle_stage * (4500 / 50)
  elseif idle_wiggle_stage < 100 then
      servo_value = (100 - idle_wiggle_stage) * (4500 / 50)
  elseif idle_wiggle_stage < 150 then
      servo_value = (100 - idle_wiggle_stage) * (4500 / 50)
  elseif idle_wiggle_stage < 200 then
     servo_value = (idle_wiggle_stage-200) * (4500 / 50)
  else
      idle_wiggle_stage = 0
      servo_value = 0
  end
  servo_value = servo_value / 4500

  -- scan servo outputs and override where required
  for i = 1, #servo_outputs do
    local servo_output = servo_outputs[i]
    local pwm_out = servo_output['trim']
    if servo_output['function'] ~= throttle then
      if servo_value > 0 then
        pwm_out = pwm_out + (servo_output['max'] - servo_output['trim']) * servo_value
      else
        pwm_out = pwm_out + (servo_output['trim'] - servo_output['min']) * servo_value
      end
    end
    SRV_Channels:set_output_pwm_chan_timeout(servo_output['chan'], pwm_out, 150)
  end

end

function run_alt_wait()
  local now = millis()

  if (wiggle_time ~= 0) and (idle_wiggle_stage == 0) and ((now - last_wiggle_ms) > (wiggle_time*1000)) then
    idle_wiggle_stage = 1
    last_wiggle_ms = now
  end

  update_servos()

  loc = ahrs:get_location()
  vel = ahrs:get_velocity_NED()
  if (not loc) or (not vel) then
    -- wait to get location and velocity
    filtered_sink_rate = 0
    return run_alt_wait, 100
  end
  loc:change_alt_frame(ABOVE_HOME)

  if loc:alt() > (altitude*100.0) then
    gcs:send_text(MAV_SEVERITY_INFO, "Reached altitude")
    vehicle:nav_script_time_done(active_id)
    return update, 100
  end

  filtered_sink_rate = filtered_sink_rate * 0.8 + vel:z() * 0.2
  if filtered_sink_rate > descent_rate then
    gcs:send_text(MAV_SEVERITY_INFO, string.format("Reached descent rate %.1f m/s", filtered_sink_rate));
    vehicle:nav_script_time_done(active_id)
    return update, 100
  end

  return run_alt_wait, 100
end

function update()
  -- wait to receive command
  local cmd, id
  id, cmd = vehicle:nav_script()
  if cmd and (cmd:command() == MAV_CMD_NAV_ALTITUDE_WAIT) then
    gcs:send_text(MAV_SEVERITY_INFO, "Nav Altitude Wait started")
    altitude = cmd:param1()
    descent_rate = cmd:param2()
    wiggle_time = cmd:param3()
    gcs:send_text(MAV_SEVERITY_INFO, string.format("alt %0.1fm, decent rate %0.1fm/s, wiggle %is", altitude, descent_rate, wiggle_time))
    filtered_sink_rate = 0
    idle_wiggle_stage = 0
    last_wiggle_ms = 0
    active_id = id
    return run_alt_wait()
  end

  return update, 100
end

return update()
