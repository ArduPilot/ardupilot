--pullup lockout code

-- Define safe areas as polygons to perform balloon drop. 
local safe_zones = {
    -- Can define multiple polygons over ocean areas. 
    {
        {lat = 39.758170, lng = -74.065188}, -- (39.758170, -74.065188)
        {lat = 40.490719, lng = -73.917797}, -- (40.490719, -73.917797)
        {lat = 40.678260, lng = -72.799620}, -- (40.678260, -72.799620)
        {lat = 39.684256, lng = -72.799620} --(39.684256, -72.799620)
    }
}

function point_in_polygon(lat, lng, polygon)
    local inside = false
    local p1x, p1y = polygon[1].lat, polygon[1].lng
    
    for i = 1, #polygon do
        local p2x, p2y = polygon[i].lat, polygon[i].lng
        if ((p2y > lng) ~= (p1y > lng)) and 
           (lat < (p1x - p2x) * (lng - p2y) / (p1y - p2y) + p2x) then
            inside = not inside
        end
        p1x, p1y = p2x, p2y
    end
    
    return inside
end

function is_over_safe_zone()
    local loc = ahrs:get_position()
    if loc == nil then -- Can't determine position
        return false  
    end
    
    local lat = loc:lat() / 1e7  -- Convert from integer degrees
    local lng = loc:lng() / 1e7
    
    for _, zone in ipairs(safe_zones) do
        if point_in_polygon(lat, lng, zone) then
            return true
        end
    end
    
    return false
end

function altitude_updater()
    -- get current EKF position as a Location userdata (or nil if not reasonable)
    local loc = ahrs:get_position()
    if loc == nil then
        gcs:send_text(6, "Abs Alt: position unavailable")
        return update, 1000
    end

    -- altitude from Location is in centimeters
    local alt_cm = loc:alt() or 0

    -- If this Location is relative to home, convert to absolute by adding home alt
    --[[if loc:relative_alt() then
        if ahrs:home_is_set() then
            local home = ahrs:get_home()
            if home then
                alt_cm = alt_cm + (home:alt() or 0)
            end
        end
    end
    --]]

    alt_m = alt_cm / 100.0
end

function control_lockout()
    --returns true if control lockout is required
    --disabled under 20km
    --if pitch is < -30 degrees and speed is < 100, then it's locked out

    local minimum_for_lockout = 20000

    

    if alt_m ~= nil then
        --gcs:send_text(6, "HAGL: "..tostring(alt_m))
        if alt_m < minimum_for_lockout then
            return false
        end
    end

    if ahrs:healthy() then
        local current_pitch = ahrs:get_pitch_rad()
        local minimum_pitch = math.rad(-30)

        local minimum_speed = 150
        local current_speed = ahrs:airspeed_estimate()

        --gcs:send_text(6, "Pitch: "..tostring(math.deg(current_pitch)).." Speed: "..tostring(current_speed))

        if current_pitch < minimum_pitch or current_speed < minimum_speed then
            return true
        end
    end

    return false
end

function drop_detector()
    --if it detects low g for more than thres_time, detect a drop
    if not ahrs:healthy() then
        return false
    end

    local acceleration = ahrs:get_accel()
    local accel_abs = acceleration:length()

    local low_g_thres_accel = 3.0 --m/s/s i hope
    local low_g_thres_time_ms = 1000

    if accel_abs < low_g_thres_accel then
        local elapsed_time = millis() - drop_detector_last_time_ms
        low_g_duration = low_g_duration + elapsed_time
        
        if low_g_duration > low_g_thres_time_ms then
            return true
        end
    else
        low_g_duration = 0
    end
    drop_detector_last_time_ms = millis()
    return false
end

function nominal_cutaway()
    --cutaway reaches alt

    if not drop_detected then
        SRV_Channels:set_output_pwm(97, 2000)
        relay:on(0)
    else
        SRV_Channels:set_output_pwm(97, 1000)
        relay:off(0)
        cutaway_confirmed = true
        
    end
end

function emergency_cutaway()
    --cutaway when unexpected pop
    if emergency_cutaway_start_ms == 0 then
        emergency_cutaway_start_ms = millis()
    end
    local cutting_time = millis() - emergency_cutaway_start_ms

    if cutting_time < 10000 then
        SRV_Channels:set_output_pwm(97, 2000)
        relay:on(0)

    else
        
        SRV_Channels:set_output_pwm(97, 1000)
        relay:off(0)
        cutaway_confirmed = true
    end 

end

function ready_to_drop()
    --determines if drop conditions are met
    local in_safe_zone = is_over_safe_zone()

    if not in_safe_zone then
        return false
    end

    local is_above_alt = alt_m > 33866.6328

    if is_above_alt then
        return true
    end

    return false
end

function update()

    local mode = vehicle:get_mode()
    local MANUAL = 0
    local RTL = 11

    altitude_updater()

    if millis() - last_time_lo_ms > 1000 then
        last_time_lo_ms = millis()
        gcs:send_text(6, "Mode: "..tostring(mode))

        if drop_detected then
            gcs:send_text(6, "Drop detected")
        else
            gcs:send_text(6, "On balloon")
        end
        if pullup_complete then
            gcs:send_text(6, "Pullup Complete")
        else
            gcs:send_text(6, "Pullup, lockout")
        end
        if cutaway_confirmed then
            gcs:send_text(6, "cut away!")
        else
            gcs:send_text(6, "on flight train")
        end
        if drop_commanded then
            gcs:send_text(6, "commanded drop!")
        else
            gcs:send_text(6, "no command")
        end

        if relay:get(0) then
            gcs:send_text(6, "relay 1 hot")
        else
            gcs:send_text(6, "relay 1 cold")
        end
        gcs:send_text(6, "AltAGL: ".. alt_m)
    end

    if not drop_detected then
        drop_detected = drop_detector()

        if mode == RTL then
            if vehicle:set_mode(MANUAL) then
                gcs:send_text(6, "Switched to MANUAL")
            else
                gcs:send_text(4, "Mode change to MANUAL failed")
            end
        end
    end

    if not cutaway_confirmed and arming:is_armed() then
        if ready_to_drop() or drop_commanded then --case where we're on the balloon and ready to drop
            drop_commanded = true
            nominal_cutaway()
        elseif drop_detected and not drop_commanded and is_over_safe_zone() then
            emergency_cutaway()
        end
    end

    local loc = ahrs:get_position()
    local current_time = millis()

    if arming:is_armed() and drop_detected then
        local target = Location()
        
        -- Track GPS acquisition status
        if loc == nil then
            -- GPS not available
            if gps_lost_start_time == nil then
                gps_lost_start_time = current_time
                gcs:send_text(6, "GPS lost - starting timer")
            end
            
            local gps_lost_duration = current_time - gps_lost_start_time
            
            -- Check if GPS has been lost for 5 minutes (300,000 milliseconds)
            if gps_lost_duration >= 300000 then
                gcs:send_text(4, "Warning: GPS not acquired for 5min. New destination: East :)")
                target:lat(33896418) -- (33.896418, -19.786401)
                target:lng(-19786401)
                target:change_alt_frame(0)
                target:alt(1)
            else
                -- GPS lost but not for 5 minutes yet - don't set target, just wait
                local remaining_seconds = (300000 - gps_lost_duration) / 1000
                gcs:send_text(6, string.format("GPS lost for %.1fs, waiting %.1fs more", gps_lost_duration/1000, remaining_seconds))
                return update, 1000
            end
        else
            -- GPS is available - reset the lost timer and use normal target
            gps_lost_start_time = nil
            target:lat(403373343)
            target:lng(-746123932)
            target:change_alt_frame(0)
            target:alt(1)
        end

        local GUIDED = 15 --guided mode enum

        if not control_lockout() then
            --gcs:send_text(6, "Control Lockout Disabled")
            --vehicle:set_target_location(target)
            pullup_complete = true
            
        else
            --gcs:send_text(6, "Controls locked out")
        end

        if pullup_complete then
            if vehicle:get_mode() ~= GUIDED then
                if vehicle:set_mode(GUIDED) then
                    vehicle:set_target_location(target)
                    gcs:send_text(6, "Switched to GUIDED")
                    
                else
                    gcs:send_text(4, "Mode change to GUIDED failed")
                end
            end
        else
            if vehicle:get_mode() ~= MANUAL then
                if vehicle:set_mode(MANUAL) then
                    gcs:send_text(6, "Switched to MANUAL")
                else
                    gcs:send_text(4, "Mode change to MANUAL failed")
                end
            end
        end 
    end

    return update, 100
end


alt_m = 0.0
drop_detected = false
drop_commanded = false
pullup_complete = false
last_time_lo_ms = millis()
low_g_duration = 0.0
drop_detector_last_time_ms = millis()
gps_lost_start_time = nil  -- Track when GPS was first lost
emergency_cutaway_start_ms = 0
cutaway_confirmed = false

return update, 100