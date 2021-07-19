-- example of getting RC input

local scripting_rc_1 = rc:find_channel_for_option(300)
local scripting_rc_2 = rc:find_channel_for_option(301)
local flip_flop = 0

function update()
  pwm1 = rc:get_pwm(1)
  pwm2 = rc:get_pwm(2)
  pwm3 = rc:get_pwm(3)
  pwm4 = rc:get_pwm(4)
  gcs:send_text(0, "RCIN 1:" .. tostring(pwm1) .. " 2:" .. tostring(pwm2).. " 3:" .. tostring(pwm3).. " 4:" .. tostring(pwm4))

  -- read normalized input from designated scripting RCx_OPTION
  if scripting_rc_1 then
    gcs:send_text(0, "Scripting in 1:" .. tostring(scripting_rc_1:norm_input()))
  end

  -- read switch input from second designated scripting RCx_OPTION
  if scripting_rc_2 then
    local sw_pos = scripting_rc_2:get_aux_switch_pos()
    if sw_pos == 0 then 
      gcs:send_text(0, "Scripting switch is low")
    elseif sw_pos == 1 then
      gcs:send_text(0, "Scripting switch is middle")
    else
      gcs:send_text(0, "Scripting switch is high")
    end
  end

  -- we can also call functions that are available to RC switches
  -- 28 is Relay one
  rc:run_aux_function(28, flip_flop)

  if (flip_flop == 0) then
    flip_flop = 2 -- switch high
  else
    flip_flop = 0 -- switch low
  end


  return update, 1000 -- reschedules the loop
end

return update()
