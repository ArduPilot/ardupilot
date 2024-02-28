-- example script for using "get_origin()"
-- prints the home and ekf origin lat long and altitude to the console every 5 seconds

function update ()

    home = ahrs:get_home()
    origin = ahrs:get_origin()

    if home then
        gcs:send_text(0, string.format("Home - Lat:%.1f Long:%.1f Alt:%.1f", home:lat(), home:lng(), home:alt()))
    end

    if origin then
        gcs:send_text(0, string.format("Origin - Lat:%.1f Long:%.1f Alt:%.1f", origin:lat(), origin:lng(), origin:alt()))
    end

    return update, 5000

end

return update()