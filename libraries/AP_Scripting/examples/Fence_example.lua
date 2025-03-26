-- this script loads in a fence about the home position
local rad1 = 50
local rad2 = 100
local home

function add_fence()

    -- build fence points into this table
    local fence_points = {}

    -- draw a star!
    for i = 0, 9 do
        local angle = math.rad(i*(360/10))
        local temp = Location()
        -- temp = center does not work
        temp:lat(home:lat())
        temp:lng(home:lng())
        if i%2 == 0 then
            temp:offset(math.sin(angle)*rad1, math.cos(angle)*rad1)
        else 
            temp:offset(math.sin(angle)*rad2, math.cos(angle)*rad2)
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
    assert(fence:send(),'Failed to send fence')

    gcs:send_text(4,'Added '..#fence_points .. ' point fence')

    if rad1 < rad2 then
        rad1 = rad2 + 50
    else
        rad2 = rad1 + 50
    end
end

function wait()
    -- wait until we have a valid home location
    home = ahrs:get_home()
    if home and home:lat() ~= 0 and home:lng() ~= 0 then
        return add_fence()
    end

    return wait, 1000
end

return wait()
