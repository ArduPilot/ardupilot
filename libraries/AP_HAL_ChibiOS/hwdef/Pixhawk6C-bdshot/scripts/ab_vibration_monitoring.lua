-- Vibration Monitor Script using Vector3f via get_vibration (or similar)
-- Prints a warning if vibration magnitude in x / y / z > threshold
-- And prints a “script loaded” message on startup

local VIBRATION_THRESHOLD = 30.0  -- threshold in same units as vector's components
local CHECK_INTERVAL_MS = 1000    -- check every 1000 ms

local script_initialized = false

-- update callback
function update()
    if not script_initialized then
        gcs:send_text(6, " Vibration Monitor Script Loaded")
        script_initialized = true
    end

    -- Try obtaining the vibration vector
    local vib = nil
    -- Replace with correct API call; using ahrs:get_vibration() as example
    if ahrs and ahrs.get_vibration then
        vib = ahrs:get_vibration()
    end

    if vib then
        -- vib is a Vector3f userdata
        local x = math.abs(vib:x())
        local y = math.abs(vib:y())
        local z = math.abs(vib:z())

        if x > VIBRATION_THRESHOLD or y > VIBRATION_THRESHOLD or z > VIBRATION_THRESHOLD then
            gcs:send_text(4, string.format(
                " HIGH VIBRATION! X=%.2f  Y=%.2f  Z=%.2f", x, y, z))
        end
    else
        -- optionally indicate the vibration API is not available
        gcs:send_text(4, " get_vibration API not available")
    end

    return update, CHECK_INTERVAL_MS
end

return update()
