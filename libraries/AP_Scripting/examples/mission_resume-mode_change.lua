-- Mission editing script for resuming missions at the point of RTH.
-- Adapted from https://github.com/ArduPilot/ardupilot/blob/master/libraries/AP_Scripting/examples/mission-edit-demo.lua

current_pos = nil
home = nil
mode_changed = false
rth_triggered = false
last_location = nil
mission_started = false
motors_armed = false


function update()
    current_pos = ahrs:get_location()

    if current_pos == nil then
        gcs:send_text(6, "LUA - Current position is nil, retrying...")
        return update, 1000
    end

    local mode = vehicle:get_mode()

    -- Check if motors are armed
    if not motors_armed then
        motors_armed = arming:is_armed()
        if motors_armed then
           gcs:send_text(4, "LUA - Motors armed")
        end
    end

    -- Once armed, monitor mission start and mode changes
    if motors_armed then
        if not mission_started and mode == 3 then -- 3 is AUTO mode
            mission_started = true
            gcs:send_text(4, "LUA - Mission started, monitoring for mode changes")
            local mission_state = mission:state()
            gcs:send_text(6, string.format("LUA - MISSION STATE: %d", mission_state))
        end

        if mission_started then
            -- Handle mode changes from AUTO or to RTH
            if (mode ~= 3 and not mode_changed) or (mode == 6 and not rth_triggered) then
                mode_changed = true
                rth_triggered = (mode == 6)
                last_location = current_pos
                gcs:send_text(4, "LUA - Mode change detected, saving current location")
                save_current_location_as_waypoint()
            elseif mode ~= 6 and rth_triggered then
                rth_triggered = false
                gcs:send_text(4, "LUA - Exited RTH mode")
            end
        end
    end

    return update, 1000
end

-- Save current location and altitude as a new waypoint
function save_current_location_as_waypoint()
    if last_location == nil then
        gcs:send_text(6, "LUA - Last location is nil, cannot save waypoint")
        return
    end

    local lat = last_location:lat()
    local lng = last_location:lng()
    local alt = last_location:alt()

    lat = math.floor(tonumber(lat))
    lng = math.floor(tonumber(lng))
    alt = alt
    home = ahrs:get_home()
    home_alt = home:alt()

    local wp_num = mission:get_current_nav_index()
    local new_wp = mission:get_item(wp_num)
    gcs:send_text(6, string.format("LUA - Current waypoint: %d", wp_num))

    -- Shift subsequent waypoints to resume from added waypoint
    for i = mission:num_commands(), wp_num+1, -1 do
        local wp = mission:get_item(i - 1)
        mission:set_item(i, wp)
    end

    new_wp:command(16) -- 16 = normal WAYPOINT
    new_wp:x(lat)
    new_wp:y(lng)
    new_wp:z((alt - home_alt)/100) -- Use relative altitude

    mission:set_item(wp_num, new_wp)
    gcs:send_text(6, string.format("LUA - Setting new waypoint: x=%d, y=%d, z=%.2f", new_wp:x(), new_wp:y(), new_wp:z()))
    gcs:send_text(4, "LUA - Added new waypoint at current location")
end

-- Wait for current and home location
function wait_for_home()
    current_pos = ahrs:get_location()
    if current_pos == nil then
        gcs:send_text(6, "LUA - Waiting for current position...")
        return wait_for_home, 2000
    end

    home = ahrs:get_home()
    if home == nil or home:lat() == 0 then
        gcs:send_text(6, "LUA - Waiting for home location...")
        return wait_for_home, 2000
    end

    gcs:send_text(4, "LUA - Home location acquired")

    return update, 1000
end

function delayed_boot()
    gcs:send_text(6, "LUA - Mission script starting")
    return wait_for_home, 2000
end

return delayed_boot, 5000
