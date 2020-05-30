-- Example of accessing AC_PID objects, not all are available on all vehicles

-- Copter, also used by quadplanes
local roll_pid = vehicle:get_AC_PID(vehicle.RATE_ROLL)
local pitch_pid = vehicle:get_AC_PID(vehicle.RATE_PITCH)
local yaw_pid = vehicle:get_AC_PID(vehicle.RATE_YAW)

-- Rover
local steer_pid = vehicle:get_AC_PID(vehicle.RATE_STEER)
local throttle_pid = vehicle:get_AC_PID(vehicle.THROTTLE_SPEED)
local pitch_throttle = vehicle:get_AC_PID(vehicle.PITCH_THROTTLE)
local sail_heel = vehicle:get_AC_PID(vehicle.SAILBOAT_HEEL)

-- Tracker
local tracker_pitch = vehicle:get_AC_PID(vehicle.TRACKER_PITCH)
local tracker_yaw = vehicle:get_AC_PID(vehicle.TRACKER_YAW)

function print_pid(type, object)
  gcs:send_text(0,string.format("%s: P %0.2f, I %0.2f, D %0.2f",type,object:kP(),object:kI(),object:kD()))
end

function update()
  -- Print the gain values from the PID controllers
  if roll_pid and pitch_pid and yaw_pid then -- copter and quadplane
    print_pid('Roll',roll_pid)
    print_pid('Pitch',pitch_pid)
    print_pid('Yaw',yaw_pid)
  end

  if steer_pid and throttle_pid and pitch_throttle and sail_heel then -- rover
    print_pid('Steer',steer_pid)
    print_pid('Throttle',throttle_pid)
    print_pid('Pitch',pitch_throttle)
    print_pid('heel',sail_heel)
  end

  if tracker_pitch and tracker_yaw then -- tracker
    print_pid('Pitch',tracker_pitch)
    print_pid('Yaw',tracker_yaw)
  end

  return update, 10000 -- reschedules the loop in 10 seconds
end

  -- All methods available
  --[[
  reset_I()
  reset_filter()

  -- get accessors
  kP()
  kI()
  kD()
  ff()
  filt_T_hz()
  filt_E_hz()
  filt_D_hz()
  imax()
  get_filt_alpha(float)
  get_filt_T_alpha()
  get_filt_E_alpha()
  get_filt_D_alpha()

  -- set accessors, only saved permanently once save_gains() is called
  set_kP(float)
  set_kI(float)
  set_kD(float)
  set_ff(float)
  set_imax(float)
  set_filt_T_hz(float)
  set_filt_E_hz(float)
  set_filt_D_hz(float)
  save_gains()
  ]]--

return update() -- run immediately before starting to reschedule
