-- GPS management in Q modes: disables auto switch, sets GPS[1] as primary
-- switches to GPS[0] if sats or vacc drop below a threshold
-- switches back to GPS[1] if sats and vacc rise above a threshold

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

-- Mission commands for landing phase
local NAV_CMDS = {
    [17] = true, -- MAV_CMD_NAV_LOITER_UNLIM
    [19] = true, -- MAV_CMD_NAV_LOITER_TIME
    [31] = true, -- MAV_CMD_NAV_LOITER_TO_ALT
    [85] = true, -- MAV_CMD_NAV_VTOL_LAND
}

local MODES = { auto = 10 } -- other valid modes which need additional checks

local run_interval = 1.0 -- once every run_interval seconds

local VTOL_GPS = 1
local FW_GPS  = 0

-- Asymmetric thresholds give hysteresis.
local SAT_MAX_THRESHOLD  = 10  -- switch AWAY when sats < this
local SAT_MIN_THRESHOLD  = 15  -- switch BACK only when sats >= this
local VACC_MIN_THRESHOLD = 2.0 -- switch AWAY when vacc > this
local VACC_MAX_THRESHOLD = 1.5 -- switch BACK only when vacc <= this

local DEFAULT_PRIMARY = 1 -- 0 is FW GPS, 1 is VTOL GPS
local DEFAULT_GPS_AUTO_SWITCH = 1 -- USE_BEST


-- True when the vehicle is in a VTOL flight phase:
-- i.e. in a Q mode, or in Auto mode executing a VTOL/loiter mission
local function in_vtol_phase(mode)
    if QMODES[mode] then return true end
    if mode == MODES.auto then
        local nav_cmd = mission:get_current_nav_id()
        if nav_cmd and NAV_CMDS[nav_cmd] then return true end
    end
    if quadplane and quadplane:in_vtol_land_descent() then return true end
    return false
end

local function update()
    local mode = vehicle:get_mode()
    local current_gps_auto_switch = param:get("GPS_AUTO_SWITCH")

    if not in_vtol_phase(mode) then
        if DEFAULT_GPS_AUTO_SWITCH ~= nil and current_gps_auto_switch ~= DEFAULT_GPS_AUTO_SWITCH then
            gcs:send_text(6, "Not in VTOL phase, resetting GPS settings to default")
            param:set("GPS_AUTO_SWITCH", DEFAULT_GPS_AUTO_SWITCH)
            param:set("GPS_PRIMARY", DEFAULT_PRIMARY)
            gcs:send_text(6, string.format("GPS_AUTO_SWITCH: %d", DEFAULT_GPS_AUTO_SWITCH))
            gcs:send_text(6, string.format("GPS[%d] set as primary", DEFAULT_PRIMARY))
        end
        return update, run_interval * 1000
    end

    -- Q mode
    if current_gps_auto_switch ~= nil and current_gps_auto_switch ~= 0 then
        gcs:send_text(6, "Q mode, disabling GPS autoswitch")
        param:set("GPS_AUTO_SWITCH", 0)
    end

    local current_primary = param:get("GPS_PRIMARY")
    if current_primary == nil then return update, run_interval * 1000 end

    -- Make sure both GPS instances exist
    if gps:num_sensors() <= VTOL_GPS then return update, run_interval * 1000 end

    local vtol_sats = gps:num_sats(VTOL_GPS)
    local vtol_vacc = gps:vertical_accuracy(VTOL_GPS) or 0
    
    if current_primary == VTOL_GPS then
        -- on preferred GPS
        if vtol_sats < SAT_MAX_THRESHOLD or vtol_vacc > VACC_MIN_THRESHOLD then
            gcs:send_text(6, string.format(
                "Q mode, switching to GPS[%d]: GPS[%d] sats=%d vacc=%.2f",
                FW_GPS, VTOL_GPS, vtol_sats, vtol_vacc))
            param:set("GPS_PRIMARY", FW_GPS)
        end
    else
        -- on fallback GPS
        if vtol_sats >= SAT_MIN_THRESHOLD and vtol_vacc <= VACC_MAX_THRESHOLD then
            gcs:send_text(6, string.format(
                "Q mode, returning to GPS[%d]: sats=%d vacc=%.2f",
                VTOL_GPS, vtol_sats, vtol_vacc))
            param:set("GPS_PRIMARY", VTOL_GPS)
        end
    end

    return update, run_interval * 1000
end

gcs:send_text(6, "Airbound Lua: GPS switch in Q modes script loaded")
gcs:send_text(6, string.format("GPS default primary: %d", DEFAULT_PRIMARY))
gcs:send_text(6, string.format("GPS default auto switch setting: %d", DEFAULT_GPS_AUTO_SWITCH))

return update, run_interval * 1000