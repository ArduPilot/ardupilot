--Example For disarming the drone when roll or pitch exceeds n degrees below n altitude.
--used to hand recover quads and to prevent prop strikes in case of low altitude collision
--Must have a downward facing rangefinder
--Made by shroxgh

local rotation_downward = 25
local ALT_THRESH = 250 -- Altitude in which in centimeters 
local ANGLE_THRESH = 80 -- Roll/Pitch angle threshold in degrees

function update()
    local sensor_count = rangefinder:num_sensors()
    

    -- Check altitude from downward rangefinder
    if rangefinder:has_data_orient(rotation_downward) then
        local distance_cm = rangefinder:distance_cm_orient(rotation_downward)
        if distance_cm and distance_cm < ALT_THRESH then
            -- Check roll and pitch
            local roll = ahrs:get_roll() * 57.2958 -- Convert radians to degrees
            local pitch = ahrs:get_pitch() * 57.2958 -- Convert radians to degrees

           

            if math.abs(roll) > ANGLE_THRESH or math.abs(pitch) > ANGLE_THRESH then
                -- Disarm the drone
                arming:disarm() -- 
                gcs:send_text(0, "Disarming: Roll or Pitch exceeded limits below altitude threshold!")
            end
        end
    else
        gcs:send_text(1, "No valid data from downward rangefinder!")
    end

    -- Schedule the next update in milliseconds
    return update, 10
end

return update()
