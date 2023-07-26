-- Commands copter to fly circle trajectory using posvel method in guided mode. 
-- The trajectory start from the current location
-- 
-- CAUTION: This script only works for Copter.
-- This script start when the in GUIDED mode and above 5 meter.
--      1) arm and takeoff to above 5 m 
--      2) switch to GUIDED mode 
--      3) the vehilce will follow a circle in clockwise direction with increasing speed until ramp_up_time_s time has passed.
--      4) switch out of and into the GUIDED mode any time to restart the trajectory from the start.
-- luacheck: only 0

-- Edit these variables
local rad_xy_m = 10.0   -- circle radius in xy plane in m
local target_speed_xy_mps = 5.0     -- maximum target speed in m/s
local ramp_up_time_s = 10.0     -- time to reach target_speed_xy_mps in second
local sampling_time_s = 0.05    -- sampling time of script

-- Fixed variables
local omega_radps = target_speed_xy_mps/rad_xy_m
local copter_guided_mode_num = 4
local theta = 0.0
local time = 0.0
local test_start_location = Vector3f(0.0, 0.0, 0.0)

gcs:send_text(0,"Script started")
gcs:send_text(0,"Trajectory period: " .. tostring(2 * math.rad(180) / omega_radps))

function circle()
    local cur_freq = 0
    -- increase target speed lineary with time until ramp_up_time_s is reached
    if time <= ramp_up_time_s then 
        cur_freq = omega_radps*(time/ramp_up_time_s)^2
    else 
        cur_freq = omega_radps
    end

    -- calculate circle reference position and velocity
    theta = theta + cur_freq*sampling_time_s

    local th_s = math.sin(theta)
    local th_c = math.cos(theta) 

    local pos = Vector3f()
    pos:x(rad_xy_m*th_s)
    pos:y(-rad_xy_m*(th_c-1))
    pos:z(0)

    local vel = Vector3f()
    vel:x(cur_freq*rad_xy_m*th_c)
    vel:y(cur_freq*rad_xy_m*th_s)
    vel:z(0)

    return pos, vel
end

function update()
    if arming:is_armed() and vehicle:get_mode() == copter_guided_mode_num and -test_start_location:z()>=5 then

        -- calculate current position and velocity for circle trajectory
        local target_pos = Vector3f()
        local target_vel = Vector3f()
        target_pos, target_vel = circle()

        -- advance the time
        time = time + sampling_time_s

        -- send posvel request
        if not vehicle:set_target_posvel_NED(target_pos+test_start_location, target_vel) then
            gcs:send_text(0, "Failed to send target posvel at " .. tostring(time) .. " seconds")
        end
    else 
        -- calculate test starting location in NED
        local cur_loc = ahrs:get_location()        
        if cur_loc then
             test_start_location = cur_loc.get_vector_from_origin_NEU(cur_loc)             
             if test_start_location then
                test_start_location:x(test_start_location:x() * 0.01) 
                test_start_location:y(test_start_location:y() * 0.01) 
                test_start_location:z(-test_start_location:z() * 0.01) 
             end             
        end

        -- reset some variable as soon as we are not in guided mode
        time = 0
        theta = 0
    end

    return update, sampling_time_s * 1000
end

return update()