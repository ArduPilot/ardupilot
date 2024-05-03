-- leds_on_a_switch.lua: control led brightness with a radio switch
--

---@diagnostic disable: cast-local-type

-- constants
local AuxSwitchPos = {LOW=0, MIDDLE=1, HIGH=2}
local NTF_LED_BRIGHT = Parameter("NTF_LED_BRIGHT")
local MAV_SEVERITY = {EMERGENCY=0, ALERT=1, CRITICAL=2, ERROR=3, WARNING=4, NOTICE=5, INFO=6, DEBUG=7}

-- state
local prev_pos = -1

function update()
    local sw_pos = rc:get_aux_cached(300)
    if sw_pos ~= prev_pos then
        if sw_pos == AuxSwitchPos.LOW then
            NTF_LED_BRIGHT:set(0)
            gcs:send_text(MAV_SEVERITY.INFO, "LEDs turned OFF")
        elseif sw_pos == AuxSwitchPos.MIDDLE then
            NTF_LED_BRIGHT:set(1)
            gcs:send_text(MAV_SEVERITY.INFO, "LEDs dimmed")
        else
            NTF_LED_BRIGHT:set(3)
            gcs:send_text(MAV_SEVERITY.INFO, "LEDs turned ON")
        end
        prev_pos = sw_pos
    end
    return update, 1000
end

return update, 5000
