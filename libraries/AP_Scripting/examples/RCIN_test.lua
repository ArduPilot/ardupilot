-- example of getting RC input


function update()
  pwm1 = rc:get_pwm(1)
  pwm2 = rc:get_pwm(2)
  pwm3 = rc:get_pwm(3)
  pwm4 = rc:get_pwm(4)
  gcs:send_text(0, "RCIN 1:" .. tostring(pwm1) .. " 2:" .. tostring(pwm2).. " 3:" .. tostring(pwm3).. " 4:" .. tostring(pwm4))
  return update, 1000 -- reschedules the loop
end

return update()
