-- example script for using "set_origin()"" and "initialised()"
-- sets the ekf origin if not already set


function update ()

    if not ahrs:initialised() then
        return update, 5000
    end
    
    origin = assert(not ahrs:get_origin(),"Refused to set EKF origin - already set")
    location = Location() location:lat(-353632640) location:lng(1491652352) location:alt(58409)
    
    if ahrs:set_origin(location) then
        gcs:send_text(6, string.format("Origin Set - Lat:%.7f Long:%.7f Alt:%.1f", location:lat()/10000000, location:lng()/10000000, location:alt()/100))
    else
        gcs:send_text(0, "Refused to set EKF origin")
    end

    return
end

return update()