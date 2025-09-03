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

-- Script to wait for safe zone using ArduPilot script structure

function update()
    if not is_over_safe_zone() then
        -- Optional: Print current position for debugging
        local loc = ahrs:get_position()
        if loc then
            local lat = loc:lat() / 1e7
            local lng = loc:lng() / 1e7
            gcs:send_text(6, string.format("Current position: %.6f, %.6f - Not over safe zone", lat, lng))
        else
            gcs:send_text(4, "Warning: Unable to get current position")
        end
        
        -- Continue checking - return to update function in 1000ms
        return update, 1000
    else
        gcs:send_text(0, "Now over safe zone! Ready for balloon drop.")
        -- Could add balloon drop logic here, or return nil to stop script
        return update, 1000  -- Keep running or change to nil to stop
    end
end

-- Initial message and start the loop
--gcs:send_text(0, "Waiting for safe zone...")
--return update, 1000