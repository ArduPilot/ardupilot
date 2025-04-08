-- This script displays the vehicle lean angles and rotation rates at 1hz

function update() -- this is the loop which periodically runs
  roll = math.deg(ahrs:get_roll())
  pitch = math.deg(ahrs:get_pitch())
  yaw = math.deg(ahrs:get_yaw())
  rates = ahrs:get_gyro()
  if rates then
    roll_rate = math.deg(rates:x())
    pitch_rate = math.deg(rates:y())
    yaw_rate = math.deg(rates:z())
  else
    roll_rate = 0
    pitch_rate = 0
    yaw_rate = 0
  end
  gcs:send_text(0, string.format("Ang R:%.1f P:%.1f Y:%.1f Rate R:%.1f P:%.1f Y:%.1f", roll, pitch, yaw, roll_rate, pitch_rate, yaw_rate))
  return update, 1000 -- reschedules the loop
end

return update() -- run immediately before starting to reschedule
