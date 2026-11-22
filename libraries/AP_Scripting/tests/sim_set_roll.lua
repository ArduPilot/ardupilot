-- SITL only: set the roll angle of the simulated vehicle.
-- Setting SCR_USER1 to a roll angle in degrees rotates the simulated vehicle
-- to that roll at its current location, with zero velocity and rotation rate.
-- Used to put a vehicle on the ground upside down.  Requires AHRS_EKF_TYPE=10
-- so the AHRS uses the simulated attitude, and SIM_GND_BEHAV=0 so the
-- simulator does not level a multicopter on the ground.

local last_roll_deg = 0

local function update()
    local roll_deg = param:get('SCR_USER1')
    if roll_deg ~= nil and roll_deg ~= last_roll_deg then
        local loc = ahrs:get_location()
        if loc then
            last_roll_deg = roll_deg
            local quat = Quaternion()
            quat:from_euler(math.rad(roll_deg), 0, ahrs:get_yaw_rad())
            sim:set_pose(0, loc, quat, Vector3f(), Vector3f())
            gcs:send_text(0, string.format("SIM: roll set to %.1f", roll_deg))
        end
    end
    return update, 100
end

return update, 1000
