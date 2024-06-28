-- test get_target_location functions for copter
-- https://discuss.ardupilot.org/t/vehicle-get-target-location-in-lua-copter/108901

function update()
    local next_WP = vehicle:get_target_location()
    if not next_WP then
        -- not in a flight mode with a target location
        gcs:send_text(0,"not in a flight mode with target loc")
    else
        gcs:send_text(0,"target loc : " .. next_WP:lat() .. " ; " .. next_WP:lng())
    end

    return update, 10000
end

return update()
