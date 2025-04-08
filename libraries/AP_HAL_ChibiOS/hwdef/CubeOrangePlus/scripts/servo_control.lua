local SERVOx_FUNCTION = 94 -- SERVOx_FUNCTION selected (94 = Script1)
local servo_channel = assert(SRV_Channels:find_channel(SERVOx_FUNCTION))
local rc_channel = 6       -- Radiomaster TX16s 6th channel (SC switch)

function update()
    rc_input = rc:get_pwm(rc_channel)

    if rc_input > 1800 then
        SRV_Channels:set_output_pwm_chan(servo_channel, param:get("SERVO8_MAX"))
    end

    if rc_input < 1100 then
        SRV_Channels:set_output_pwm_chan(servo_channel, param:get("SERVO8_MIN"))
    end

    return update, 50
end

return update()
