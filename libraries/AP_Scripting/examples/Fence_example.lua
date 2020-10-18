-- this script loads in a fence about the home position

function add_fence(center)

    -- build fence points into this table
    local fence_points = {}

    -- draw a star!
    local inner_rad = 50
    local outer_rad = 100
    for i = 0, 9 do
        local angle = math.rad(i*(360/10))
        local temp = Location()
        -- temp = center does not work
        temp:lat(center:lat())
        temp:lng(center:lng())
        if i%2 == 0 then
            temp:offset(math.sin(angle)*outer_rad, math.cos(angle)*outer_rad)
        else 
            temp:offset(math.sin(angle)*inner_rad, math.cos(angle)*inner_rad)
        end

        fence_points[i+1] = AC_PolyFenceItem()
        fence_points[i+1]:type(98) -- POLYGON_INCLUSION
        fence_points[i+1]:vertex_count(10) -- 10 points in total

        local location = Vector2l()
        location:x(temp:lat())
        location:y(temp:lng())
        -- fence_points[i+1]:loc():x(temp:lat()) does not work
        fence_points[i+1]:loc(location)

    end

    fence:load(table.unpack(fence_points))

    gcs:send_text(4,'Added '..#fence_points .. ' point fence')
end

function wait()
    -- wait until we have a valid home location
    local home = ahrs:get_home()
    if home and home:lat() ~= 0 and home:lng() ~= 0 then
        add_fence(home)
        return
    end

    return wait, 1000
end

return wait()
