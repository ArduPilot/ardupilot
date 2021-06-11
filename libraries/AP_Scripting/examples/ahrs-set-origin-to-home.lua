-- example script for using "set_origin()"
-- sets the ekf origin to the home location every 5 seconds

function update ()
    
    home = ahrs:get_home()
    origin = ahrs:get_origin()

    ahrs:set_origin(home)
    
    return update, 5000
end

return update()
