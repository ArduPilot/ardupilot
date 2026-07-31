local Scripting1 = 300
local Script1 = 94


local function update()
    local channel = rc:find_channel_for_option(Scripting1)
    local value = rc:has_valid_input() and channel and channel:get_aux_switch_pos() > 0 and 1100 or 1900
    SRV_Channels:set_output_pwm(Script1, value)

    return update, 1000
end

return update()
