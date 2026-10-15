-- This is failsafe script for ArduPlane.
-- If GUIDED mode lasts more than 60 seconds in an uncontrolled situation, switch to RTL mode
local MODE_GUIDED = 15
local MODE_RTL = 11

-- Heartbeat threshold
local last_seen_threshold = 60000
-- Trigger threshold
local trigger_threashold = 60

local detected = uint32_t(0)

function guided_moniter()
    local vehicle_mode = vehicle:get_mode()
    if not arming:is_armed()
       or not vehicle:get_likely_flying()
       or not (vehicle_mode == MODE_GUIDED)
       or rc:has_valid_input()
       or millis() - gcs:last_seen() < last_seen_threshold then
        detected = uint32_t(0)
        return guided_moniter, 100
    end
    
    if vehicle_mode == MODE_GUIDED and detected == 0 then
        detected = millis()
        gcs:send_text(0, "GUID_MONITER: uncontrolled detected")
    end

    -- Trigger RTL
    if detected > 0 and (millis() - detected) / 1000 > trigger_threashold then
        gcs:send_text(0, "GUID_MONITER: switch to RTL")
        vehicle:set_mode(MODE_RTL)
    end

    return guided_moniter, 100
end

function update()
    gcs:send_text(1, "GUID_MONITER: loaded")

    return guided_moniter, 1000
end

return update, 1000
