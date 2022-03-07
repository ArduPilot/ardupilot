-- example of overriding RC inputs

local RC4 = rc:get_channel(4)

function update()
   -- mirror RC1 onto RC4
   rc1_input = rc:get_pwm(1)
   RC4:set_override(rc1_input)
   return update, 10
end

gcs:send_text(0, "RC_override example")

return update()
