-- warn the user if wind speed exceeds a threshold, failsafe if a second threshold is exceeded

-- note that this script is only intended to be run on ArduPlane

-- tuning parameters
local warn_speed = 10 -- metres/second
local failsafe_speed = 15 -- metres/second
local warning_interval_ms = uint32_t(15000) -- send user message every 15s

-- constants
local MODE_RTL = 11 -- Return to Launch mode for ArduPlane
local PERIOD_MS = 1000

local warning_last_sent_ms = uint32_t() -- time last warning message was sent to the user

local function update()
    local wind = ahrs:get_wind() -- get the wind estimate
    if wind then
        -- make a 2D wind vector
        local wind_xy = Vector2f()
        wind_xy:x(wind:x())
        wind_xy:y(wind:y())
        local speed = wind_xy:length() -- compute the wind speed
        if speed > failsafe_speed then
            gcs:send_text(0, "Wind failsafe at " .. speed .. " metres/second")
            vehicle:set_mode(MODE_RTL)
            return
        end
        if speed > warn_speed then
            if millis() - warning_last_sent_ms > warning_interval_ms then
                    gcs:send_text(4, "Wind warning at " .. speed .. " metres/second")
                warning_last_sent_ms = millis()
            end
        end
    end
    return update, PERIOD_MS
end

return update, PERIOD_MS
