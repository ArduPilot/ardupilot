-- Failsafe detector script
-- Displays message to user if there is a change in failsafe state
--
-- these SITL parameters may be use to test the script:
--     RC: SIM_RC_FAIL (0 to fail, 1 to restore)
--     Battery: SIM_BATT_VOLTAGE (9 to fail, 12.6 and restart to restore)
--     EKF: SIM_GPS1_ENABLE (0 to fail, 1 to restore)
--     GCS: MAV_GCS_SYSID (254 to fail, 255 to restore) or if using MAVProxy "set heartbeat 0" to fail, "set heartbeat 1" to restore
--     Terrain: TERRAIN_ENABLE (0 to fail, 1 to restore)

MAV_SEVERITY = {EMERGENCY=0, ALERT=1, CRITICAL=2, ERROR=3, WARNING=4, NOTICE=5, INFO=6, DEBUG=7}
MODE_CHANGE_REASON = {RADIO_FAILSAFE=3, BATTERY_FAILSAFE=4, GCS_FAILSAFE=5, EKF_FAILSAFE=6, TERRAIN_FAILSAFE=11, FAILSAFE=25}

-- variables to detect change in failsafe state
local rc_has_valid_input_last = rc:has_valid_input()
local battery_has_failsafed_last = battery:has_failsafed()
local vehicle_has_ekf_failsafed_last = vehicle:has_ekf_failsafed()
local gcs_lost_last = false
local mode_get_control_mode_reason_last = vehicle:get_control_mode_reason()
local status_last_print_ms = uint32_t(0)

-- display welcome message
gcs:send_text(0, "failsafe-detector.lua loaded")

function update()

    -- RC failsafe check
    if rc:has_valid_input() ~= rc_has_valid_input_last then
        if not rc:has_valid_input() then
            gcs:send_text(MAV_SEVERITY.WARNING, "failsafe-detect: RC lost")
        else
            gcs:send_text(MAV_SEVERITY.INFO, "failsafe-detect: RC restored")
        end
        rc_has_valid_input_last = rc:has_valid_input()
    end

    -- Battery failsafe check
    if battery:has_failsafed() ~= battery_has_failsafed_last then
        if battery:has_failsafed() then
            gcs:send_text(MAV_SEVERITY.WARNING, "failsafe-detect: Battery failsafe")
        else
            gcs:send_text(MAV_SEVERITY.INFO, "failsafe-detect: Battery restored")
        end
        battery_has_failsafed_last = battery:has_failsafed()
    end

    -- EKF failsafe check
    if vehicle:has_ekf_failsafed() ~= vehicle_has_ekf_failsafed_last then
        if vehicle:has_ekf_failsafed() then
            gcs:send_text(MAV_SEVERITY.WARNING, "failsafe-detect: EKF failsafe")
        else
            gcs:send_text(MAV_SEVERITY.INFO, "failsafe-detect: EKF restored")
        end
        vehicle_has_ekf_failsafed_last = vehicle:has_ekf_failsafed()
    end

    -- GCS failsafe check
    local last_seen_ms = gcs:last_seen()
    local now_ms = millis();
    local gcs_lost = (now_ms - last_seen_ms) > 5000
    if gcs_lost ~= gcs_lost_last then
        if gcs_lost then
            gcs:send_text(MAV_SEVERITY.WARNING, "failsafe-detect: GCS lost")
        else
            gcs:send_text(MAV_SEVERITY.INFO, "failsafe-detect: GCS restored")
        end
    end
    gcs_lost_last = gcs_lost

    -- Flight mode change caused by failsafe
    if vehicle:get_control_mode_reason() ~= mode_get_control_mode_reason_last then
        local reason
        if vehicle:get_control_mode_reason() == MODE_CHANGE_REASON.RADIO_FAILSAFE then
            reason = "RC failsafe"
        elseif vehicle:get_control_mode_reason() == MODE_CHANGE_REASON.BATTERY_FAILSAFE then
            reason = "Battery failsafe"
        elseif vehicle:get_control_mode_reason() == MODE_CHANGE_REASON.GCS_FAILSAFE then
            reason = "GCS failsafe"
        elseif vehicle:get_control_mode_reason() == MODE_CHANGE_REASON.EKF_FAILSAFE then
            reason = "EKF failsafe"
        elseif vehicle:get_control_mode_reason() == MODE_CHANGE_REASON.TERRAIN_FAILSAFE then
            reason = "Terrain failsafe"
        elseif vehicle:get_control_mode_reason() == MODE_CHANGE_REASON.FAILSAFE then
            reason = "General failsafe"
        end
        if reason then
            gcs:send_text(MAV_SEVERITY.WARNING, "failsafe-detect: mode changed due to " .. reason)
        end
        mode_get_control_mode_reason_last = vehicle:get_control_mode_reason()
    end

    -- display overall status every 5 seconds
    if now_ms - status_last_print_ms >= 5000 then
        gcs:send_text(MAV_SEVERITY.INFO, "failsafe-detect: "
            .. "RC:" .. (rc_has_valid_input_last and "OK" or "Fail")
            .. " Battery:" .. (battery_has_failsafed_last and "Fail" or "OK")
            .. " EKF:" .. (vehicle_has_ekf_failsafed_last and "Fail" or "OK")
            .. " GCS:" .. (gcs_lost_last and "Fail" or "OK")
        )
        status_last_print_ms = now_ms
    end

    -- run check at 1hz
    return update, 1000
end

return update()
