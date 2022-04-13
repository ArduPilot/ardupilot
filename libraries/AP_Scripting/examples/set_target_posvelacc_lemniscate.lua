-- Commands copter to fly lemniscate trajectory using posvelacc method in guided mode. 
-- The trajectory start from the current location
-- 
-- CAUTION: This script only works for Copter.
-- This script start when the in GUIDED mode and above 5 meter.
--      1) arm and takeoff to above 5 m 
--      2) switch to GUIDED mode 
--      3) the vehilce will follow a lemniscate in clockwise direction with increasing speed until ramp_up_time_s time has passed.
--      4) switch out of and into the GUIDED mode any time to restart the trajectory from the start.

-- Edit these variables
local rad_xy_m = 3.14   -- lemniscate radius in xy plane in m
local omega_radps = 0.95  -- 
local ramp_up_time_s = 10.0     -- time to reach target_speed_xy_mps in second
local sampling_time_s = 0.05    -- sampling time of script

-- Fixed variables
local copter_guided_mode_num = 4
local theta = 0.0
local time = 0.0
local test_start_location = Vector3f(0.0, 0.0, 0.0)

gcs:send_text(0,"PosVelAcc script started")
gcs:send_text(0,"Trajectory period: " .. tostring(2 * math.rad(180) / omega_radps))

function lemniscate()
    if time < 0 then
        return Vector3f(0.0, 0.0, 0.0), Vector3f(0.0, 0.0, 0.0), Vector3f(0.0, 0.0, 0.0)
    end
    
    local cur_freq = 0
    -- increase target speed lineary with time until ramp_up_time_s is reached
    if time <= ramp_up_time_s then 
        cur_freq = omega_radps*(time/ramp_up_time_s)^2
    else 
        cur_freq = omega_radps
    end

    -- calculate lemniscate reference position and velocity
    theta = theta + cur_freq*sampling_time_s

    local th_s = math.sin(theta)
    local th_s2 = math.sin(2*theta)
    local th_c = math.cos(theta)
    local th_c2 = math.cos(2*theta)

    local pos = Vector3f()
    pos:x(-rad_xy_m*(th_c-1))
    pos:y(rad_xy_m*th_s2)
    pos:z(0)

    local vel = Vector3f()
    vel:x(cur_freq*rad_xy_m*th_s)
    vel:y(2*cur_freq*rad_xy_m*th_c2)
    vel:z(0)

    local acc = Vector3f()
    acc:x(cur_freq^2*rad_xy_m*th_c)
    acc:y(-4*cur_freq^2*rad_xy_m*th_s2)
    acc:z(0)

    return pos, vel, acc
end

function update()
    if arming:is_armed() and vehicle:get_mode() == copter_guided_mode_num and -test_start_location:z()>=5 then

        -- calculate current position and velocity for lemniscate trajectory
        local target_pos = Vector3f()
        local target_vel = Vector3f()
        local target_acc = Vector3f()
        target_pos, target_vel, target_acc = lemniscate()

        -- advance the time
        time = time + sampling_time_s

        -- send posvelacc request
        if not vehicle:set_target_posvelaccel_NED(target_pos+test_start_location, target_vel, target_acc, false, 0, false, 0, false) then
            gcs:send_text(0, "Failed to send target posvelacc at " .. tostring(time) .. " seconds")
        end
    else
        -- calculate test starting location in NED
        local cur_loc = ahrs:get_position()
        if cur_loc then
             test_start_location = cur_loc.get_vector_from_origin_NEU(cur_loc)
             if test_start_location then
                test_start_location:x(test_start_location:x() * 0.01)
                test_start_location:y(test_start_location:y() * 0.01)
                test_start_location:z(-test_start_location:z() * 0.01)
             end
        end

        -- reset some variable as soon as we are not in guided mode
        time = -5
        theta = 0
    end

    return update, sampling_time_s * 1000
end

return update()