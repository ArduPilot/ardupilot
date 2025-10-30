-- GPS management in Q modes: disables auto switch, sets GPS[1] as primary
-- switches back to GPS[0] if sats drop

-- Q-mode numbers
local QMODES = {
    [17] = true,  -- QSTABILIZE
    [18] = true,  -- QHOVER
    [19] = true,  -- QLOITER
    [20] = true,  -- QLAND
    [21] = true,  -- QRTL
    [22] = true,  -- QAUTOTUNE
    [23] = true,  -- QACRO
}

local run_interval = 1.0 -- once every run_interval seconds
local SAT_THRESHOLD = 10 -- minimum sats required for GPS[1]

local default_primary = 1 -- 0 is FW GPS, 1 is VTOL GPS 
local default_gps_auto_switch = 1 -- USE_BEST

local function update()
    local mode = vehicle:get_mode()
    local current_gps_auto_switch = param:get("GPS_AUTO_SWITCH")

    if not QMODES[mode] then
        -- In modes such as FBWA and Auto
        -- Reset GPS auto switch to default setting
        if default_gps_auto_switch ~= nil and current_gps_auto_switch ~= default_gps_auto_switch then
            gcs:send_text(6, "Not a Q mode, resetting GPS settings to default")
            param:set("GPS_AUTO_SWITCH", default_gps_auto_switch)
            param:set("GPS_PRIMARY", default_primary)
            gcs:send_text(6, string.format("GPS_AUTO_SWITCH: %d", default_gps_auto_switch))
            gcs:send_text(6, string.format("GPS[%d] set as primary", default_primary))
        end

    else
        -- In Q Modes
        -- Set GPS auto switch to only use primary
        if current_gps_auto_switch ~= nil and current_gps_auto_switch ~= 0 then
            gcs:send_text(6, "Q mode, disabling GPS autoswitch")
            param:set("GPS_AUTO_SWITCH", 0)
        end

        -- Set GPS 1 as primary
        local current_primary = param:get("GPS_PRIMARY")
        if current_primary ~= nil and current_primary ~= 1 and gps:num_sats(1) >= SAT_THRESHOLD then
            gcs:send_text(6, "Q mode, GPS[1] set as primary")
            param:set("GPS_PRIMARY", 1)
        elseif current_primary ~= 0 and gps:num_sats(0) >= SAT_THRESHOLD and gps:num_sats(1) < SAT_THRESHOLD then
            gcs:send_text(6, string.format("Q mode, Using GPS[0], GPS[1] sats: %d < %d", gps:num_sats(1), SAT_THRESHOLD))
            param:set("GPS_PRIMARY", 0)
        end
    end

    return update, run_interval
end

gcs:send_text(6, "Airbound Lua: GPS switch in Q modes script loaded")
gcs:send_text(6, string.format("GPS default primary: %d", default_primary))
gcs:send_text(6, string.format("GPS default auto switch setting: %d", default_gps_auto_switch))

return update, run_interval