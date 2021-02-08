-- This script runs a check of the parachute arm disarm

local _disarm_radius = 25 -- bubble around home in Meters
local _high_cycles = 5 -- number of loops needed for rc to read high

local logged = false
local launched = false
local notified_arm = false
local notified_disarm = false
local chan7_continuous = 0

local chute_channel = 94 -- placeholder, pin channel not currently known

function update() -- this is the loop which periodically runs
  -- update the parachute arm and kill switches engaged variables with a protected call (try / except)
  local status_chute, parachute_armed = pcall(chute_armed)
  local status_kill, kills_engaged = pcall(kill_switch_engaged)

  -- send debug messages from the protected call functions to the gcs
  if not status_chute then
    gcs:send_text(6, "PM-LUA: " ..tostring(status_chute).."  "..tostring(parachute_armed))
  end
  if not status_kill then
    gcs:send_text(6, "PM-LUA: "..tostring(status_kill).."  "..tostring(kills_engaged))
  end

  if parachute_armed and kills_engaged and status_chute and status_kill then
    -- if the chute is armed and the kill switches are engaged then launch and the functions to evaluate these have successfully run
    launch()
  elseif not parachute_armed and kills_engaged then
    if not logged and not launched then
      -- send a notification when the kill switch is engaged but parachute is disarmed - only sends once
      gcs:send_text(6, "PM-LUA: Kill switches engaged - parachute disarmed")
      logged = true
    end
  else
    logged = false
  end

  -- send notification to GCS only once when chute is armed
  if parachute_armed and not notified_arm then
    gcs:send_text(6, "PM-LUA: Chute armed")
    notified_arm = true
  elseif not parachute_armed then
    notified_arm = false
  end

  -- send notification to GCS only once when chute is disarmed
  if not parachute_armed and not notified_disarm and not launched then
    gcs:send_text(6, "PM-LUA: Chute disarmed")
    notified_disarm = true
  elseif parachute_armed then
    notified_disarm = false
  end

  return update, 200 -- reschedules the loop at 5hz
end

function kill_switch_engaged()
  -- gets RC channel, if kill switches pulled, returns true, false if not
  -- read in the RC
  pwm7 = rc:get_pwm(7)
  --gcs:send_text(6, "PWM channel 7: "..pwm7)
  if pwm7 and pwm7 > 1800 then
    -- if channel 7 is high then return true - kill switches engaged
    chan7_continuous = chan7_continuous + 1
    if chan7_continuous > _high_cycles then
      if chan7_continuous == 10 then
        chan7_continuous = 6 -- stops counter getting too large
        return true
      end
    else
      -- need continuous high signal for 1s to trigger
      return false
    end
  else
    -- else the kill switches are not engaged
    -- reset continuous counter
    chan7_continuous = 0
    return false
  end
end

function chute_armed()
  -- determines the distance from home
  -- returns true if bubble has been broken (i.e. parachute armed)
  if arming:is_armed() then
    -- if armed - only worry if the aircraft is within home radius when the aircraft is armed 
    -- solves initialisation errors and any chute arms when carrying drone etc

    -- get home and aircraft position
    local home_pos = ahrs:get_home()
    local current_pos = ahrs:get_position()

    if not current_pos or not home_pos then
      gcs:send_text(6, "PM-LUA: Aircraft position estimate or home position not valid")
      -- if there is no valid position estimate then arm the chute
      return true
    end

    -- get distance from EKF origin
    --origin_distance = current_pos:get_vector_from_origin_NEU():length()

    -- distance from home position - appears more accurate than the above function
    vect_distance = home_pos:get_distance_NED(current_pos):length()

    -- send home distance to the gcs from monitoring
    gcs:send_text(6, "PM-LUA: Home_distance: "..vect_distance) --..", origin_distance: "..origin_distance)

    if vect_distance > _disarm_radius then
      -- if the aircraft is outside the radius then return true 
      return true
    else
      return false
    end
  else
    return false
  end
end

function launch()
  -- kills motors and launches pwm
  launched = true -- log that the chute has been launched
  -- use protected call to kill the motors
  local status_kill = pcall(kill_motors)
  local status_det = pcall(detonate_chute)
  --gcs:send_text(6, tostring(status_det))
end

function detonate_chute()
  -- sends high pwm from a pin out to initiate parachute detonation
  --servo.set_output_pwm(chute_channel, 2000)
  gcs:send_text(6, "PM-LUA: Chute launched")
end

function kill_motors()
  -- sends motor kill command
  -- disarm = arming:disarm() --"Returns false only if already disarmed."
  gcs:send_text(6, "PM-LUA: Motors Disarm")
  return arming:is_armed()
end

gcs:send_text(6, "PM-LUA: Initialising parachute LUA script")
servo.set_output_pwm(chute_channel, 1000)  -- initialise the servo pin 
return update() -- run immediately before starting to reschedule