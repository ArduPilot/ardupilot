-- test update_target_location functions for copter

local current_target = nil
local cur_loc = nil
local new_target= nil

function update()

    if not (vehicle:get_mode() == 4) then
        gcs:send_text(0, "not in Guided")
        return update, 1000
    end

    current_target = vehicle:get_target_location()
    if not current_target then
        return update, 1000
    end

    gcs:send_text(6, string.format("Current target %d %d %d frame %d", current_target:lat(), current_target:lng(), current_target:alt(), current_target:get_alt_frame()))

    cur_loc = ahrs:get_position()
    if not cur_loc then
        gcs:send_text(0, "current position is not good")
        return update, 1000
    end

    gcs:send_text(6, string.format("alt is %f", cur_loc:alt()*0.01))
    if cur_loc:alt()*0.01 < 650 then
        gcs:send_text(0, "too low")
        return update, 1000
    end

    -- just add some offset to current location
    new_target = cur_loc:copy()
    new_target:lat(cur_loc:lat() + 1000)
    new_target:lng(cur_loc:lng() + 1000)
    new_target:alt(cur_loc:alt() + 1000)

    gcs:send_text(6, string.format("New target %d %d %d frame %d", new_target:lat(), new_target:lng(), new_target:alt(), new_target:get_alt_frame()))

    vehicle:update_target_location(current_target, new_target)

    return update, 2000
end

return update()
