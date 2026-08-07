-- Test mount scripting backend
local UPDATE_MS = 100
local REPORT_INTERVAL_MS = 2000
local last_report_ms = millis()

function update()
    -- Keep mount healthy
    mount:set_attitude_euler(0, 0, -30, 0)

    -- Test get_location_target every 2 seconds
    local now_ms = millis()
    if now_ms - last_report_ms >= REPORT_INTERVAL_MS then
        last_report_ms = now_ms

        local mode = mount:get_mode(0)
        local target = mount:get_location_target(0)

        if target then
            gcs:send_text(6, string.format("Mode %d: %.6f,%.6f",
                mode, target:lat()/1e7, target:lng()/1e7))
        else
            gcs:send_text(6, string.format("Mode %d: No target", mode))
        end
    end

    return update, UPDATE_MS
end

return update()

