--pullup lockout code
--currently NOT ready for flight, only works with PLAV mixing

function control_lockout()
    --returns true if control lockout is required
    --disabled under 20km
    --if pitch is < -30 degrees and speed is < 100, then it's locked out

    local minimum_for_lockout = 20000

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

    local alt_m = alt_cm / 100.0

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
 
function update()

    local mode = vehicle:get_mode()

    if millis() - last_time_lo_ms > 1000 then
        last_time_lo_ms = millis()
        gcs:send_text(6, "Mode: "..tostring(mode))
        if free_from_balloon then
            gcs:send_text(6, "Free from balloon, no lockout")
        else
            gcs:send_text(6, "On balloon, lockout")
        end
    end

    if arming:is_armed() then
        local target = Location()
        target:lat( 403373343)
        target:lng(-746123932)
        target:change_alt_frame(0)
        target:alt(1)

        local GUIDED = 15 --guided mode enum
        local MANUAL = 0

        if not control_lockout() then
            --gcs:send_text(6, "Control Lockout Disabled")
            --vehicle:set_target_location(target)
            free_from_balloon = true
            
        else
            --gcs:send_text(6, "Controls locked out")
        end

        if free_from_balloon then
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

free_from_balloon = false
last_time_lo_ms = millis()

return update, 100