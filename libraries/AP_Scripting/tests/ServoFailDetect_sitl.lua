-- QuadPlane.ServoFailDetect fixture: run the applet with its real bindings,
-- but allow the test to withhold callbacks while CAN telemetry keeps arriving.
-- SCR_USER1: 0 normal, 1 pause for five seconds then resume, 2 pause indefinitely.
local file = assert(io.open("scripts/modules/ServoFailDetect.lua", "r"))
local source = file:read("a")
file:close()
local applet_update = assert(load(source, "ServoFailDetect", "t", _ENV))()
local stage_param = Parameter("SCR_USER1")
local gap_start
local resumed = false

local function update()
    local stage = stage_param:get()
    if stage == 2 then
        return update, 20
    end
    if stage == 1 then
        gap_start = gap_start or millis()
        if (millis() - gap_start):tofloat() < 5000 then
            return update, 20
        end
    else
        gap_start = nil
        resumed = false
    end

    local before = SRV_Channels:get_output_pwm_chan(10)
    applet_update = applet_update()
    if stage == 1 and not resumed then
        -- A timeout-zero release still writes its PWM argument. Catch a
        -- shutdown pulse here, before the main loop can overwrite it.
        local after = SRV_Channels:get_output_pwm_chan(10)
        assert(before > 1000 and after == before, "expired override reapplied shutdown PWM")
        gcs:send_text(6, "SFD test: resumed without shutdown pulse")
        resumed = true
    end
    return update, 20
end

return update, 20
