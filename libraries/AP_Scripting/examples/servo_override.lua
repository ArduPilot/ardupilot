-- Testing set_output_pwm_chan_timeout
--
-- This will set MAIN1 servo to 1700 pwm for 1 second,
-- then assigned function behavior for 1 second, and then 1100 pwm for 1 second

local flipflop = true

function update()
    if flipflop then
        SRV_Channels:set_output_pwm_chan_timeout(0, 1700, 1000)
        gcs:send_text(6, "flip---")
    else
        SRV_Channels:set_output_pwm_chan_timeout(0, 1100, 1000)
        gcs:send_text(6, "---flop")
    end
    flipflop = not flipflop
    return update, 2000
end

gcs:send_text(6, "servo_override.lua is running")
return update, 1000
