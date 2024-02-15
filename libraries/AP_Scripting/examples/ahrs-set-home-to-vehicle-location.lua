-- example script for using "set_home()"
-- sets the home location to the current vehicle location every 5 seconds

function update ()

    if ahrs:home_is_set() then
        ahrs:set_home(ahrs:get_location())
        gcs:send_text(0, "Home position reset")
    end

    return update, 5000

end

return update()