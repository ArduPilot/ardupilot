-- Check if the script is being run directly by APM, if so exit
if ... == nil then
    return
end

local funcs = {}
funcs.__index = funcs

-- Calculate the resulting value
function funcs:map_error(resulting)
    if resulting == nil then
        return 0
    end

    if resulting == 0 then
        return 0
    elseif math.abs(resulting) < 180 then
        return -resulting
    else
        return (math.abs(resulting)/(resulting + 0.001))*(360 - math.abs(resulting))
    end
end

-- Function to map an angle to the range [0, 360]
function funcs:map_to_360(angle)
    if angle == nil then
        return 0
    end

    local mapped_angle = angle
    if mapped_angle < 0 then
        mapped_angle = mapped_angle + 360
    end
    
    return mapped_angle
end

-- Conversion between degrees and radians
function funcs:to_radians(mdegrees)
    if mdegrees == nil then
        return 0
    end
    return mdegrees*math.pi/180.0
end

function funcs:to_degrees(mradians)
    if mradians == nil then
        return 0
    end
    return mradians*180.0/math.pi
end

return funcs
