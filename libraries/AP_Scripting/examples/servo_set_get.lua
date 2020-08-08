-- Testing set_output_pwm_chan_timeout and get_output_pwm
--
-- This will set MAIN1 servo to 1700 pwm for 1 second,
-- then assigned function behavior for 1 second, and then 1100 pwm for 1 second

local flipflop = true
local K_AILERON = 4
local aileron_channel = SRV_Channels:find_channel(K_AILERON)

function update()
    if flipflop then
        SRV_Channels:set_output_pwm_chan_timeout(aileron_channel, 1700, 1000)
        gcs:send_text(6, "flip---")
    else
        SRV_Channels:set_output_pwm_chan_timeout(aileron_channel, 1100, 1000)
        gcs:send_text(6, "---flop")
    end
    flipflop = not flipflop
    output_pwm = SRV_Channels:get_output_pwm(K_AILERON)
    gcs:send_text(6, "Function "..K_AILERON..", channel "..aileron_channel..", output "..output_pwm)
    return update, 2000
end

gcs:send_text(6, "servo_set_get.lua is running")
return update, 1000
