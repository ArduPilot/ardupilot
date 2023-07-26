-- Example script for accessing waypoint info
-- luacheck: only 0

local wp_index
local wp_distance
local wp_bearing
local wp_error
local wp_max_distance = 0
local last_log_ms = millis()


function on_wp_change(index, distance)
    wp_index = index
    wp_distance = distance
    wp_distance_max = distance;
    gcs:send_text(0, string.format("NEW WP: idx=%d, dist=%.01fm", index, distance))
end

function refresh_wp_info()
    local index = mission:get_current_nav_index()
    local distance = vehicle:get_wp_distance_m()
    local bearing = vehicle:get_wp_bearing_deg()
    local error = vehicle:get_wp_crosstrack_error_m()

    if index ~= nil and index ~= wp_index and distance ~= nil then
        on_wp_change(index, distance)
    end 

    if index ~= nil and distance ~= nil and bearing ~= nil and error ~= nil then
        wp_index = index
        wp_bearing = bearing
        wp_distance = distance
        wp_error = error
        wp_distance_max = math.max(wp_distance_max, wp_distance)
    end
end

function log_wp_info(index, distance, bearing, error)
    if index ~= nil and distance ~= nil and bearing ~= nil and error ~= nil then
        local perc = wp_distance_max > 0 and 100-(100*(distance/wp_distance_max)) or 0
        gcs:send_text(0, string.format("WP: %d, %.01fm (%.01f%%), b: %d°, xe:%.01fm", index, distance, perc, math.floor(bearing+0.5), error))
    end
end

function update()
    refresh_wp_info()
    log_wp_info(wp_index, wp_distance, wp_bearing, wp_error)
    return update, 1000 -- 1Hz
end
  
return update(), 1000 -- start with a 1 sec delay