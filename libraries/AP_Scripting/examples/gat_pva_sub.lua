-- Move forward at a constant velocity while following the terrain
-- The pilot can set the heading, and the script will maintain it (similar to stabilize)

local RUN_HZ = 20
local GUIDED_MODE = 4

-- configuration
local ACC_XY = 0.5                  -- xy acceleration in m/s^2
local P_GAIN_Z = 1.0                -- gain for vertical correction
local MAX_VEL_Z = 2.0               -- max vertical velocity correction in m/s

-- control variables
local active = false
local last_time_ms = millis()       -- last time in ms
local yaw_target_rad = 0            -- yaw target in radians
local hagl_target = 0               -- HAGL target in m
local pos_target = Vector3f()       -- position target in m
local vel_target = Vector3f()       -- velocity target in m/s
local acc_target = Vector3f()       -- acceleration target in m/s^2
local pos_offset = Vector3f()       -- position offset in m
local vel_offset = Vector3f()       -- velocity offset in m/s

local function update()
    -- calculate dt
    local tnow = millis()
    local dt = (tnow - last_time_ms):tofloat() / 1000.0
    if (dt > 2.0 / RUN_HZ) then
        dt = 1.0 / RUN_HZ
    end
    last_time_ms = tnow

    local current_mode = vehicle:get_mode()
    if current_mode ~= GUIDED_MODE then
        if active then
            active = false
            gcs:send_text(6, "GAT: not active")
        end
        return update, 1000 / RUN_HZ
    end

    if not active then
        -- check EKF
        local current_pos = ahrs:get_relative_position_NED_origin()
        local current_vel = ahrs:get_velocity_NED()
        if not current_pos or not current_vel then
            gcs:send_text(4, "GAT: waiting for relative position")
            return update, 1000 / RUN_HZ
        end

        -- check rangefinder
        if not rangefinder:has_data_orient(25) or rangefinder:status_orient(25) ~= 4 then -- 4 is RangeFinder_Good
            gcs:send_text(4, "GAT: downward rangefinder not healthy")
            return update, 1000 / RUN_HZ
        end

        -- reset control variables
        yaw_target_rad = ahrs:get_yaw_rad()
        hagl_target = rangefinder:distance_orient(25)

        pos_target:x(current_pos:x())
        pos_target:y(current_pos:y())
        pos_target:z(current_pos:z())

        vel_target:x(current_vel:x())
        vel_target:y(current_vel:y())
        vel_target:z(0)

        acc_target:x(0)
        acc_target:y(0)
        acc_target:z(0)

        pos_offset:x(0)
        pos_offset:y(0)
        pos_offset:z(0)

        vel_offset:x(0)
        vel_offset:y(0)
        vel_offset:z(0)

        active = true
        gcs:send_text(6, string.format("GAT: active, target HAGL %.1fm", hagl_target))
    end

    -- get the pilot yaw intent
    local yaw_chan = rc:get_channel(4)
    local yaw_input = yaw_chan and yaw_chan:norm_input_dz() or 0

    -- get the current yaw rate
    local gyro = ahrs:get_gyro()
    local yaw_rate_rads = gyro and gyro:z() or 0

    -- if the stick is not in deadband, or rotation has not stopped, update yaw_target_rad
    if yaw_input ~= 0 or math.abs(yaw_rate_rads) >= 0.05 then
        yaw_target_rad = ahrs:get_yaw_rad()
    end

    -- get the desired horizontal speed
    local speed_ms = param:get('SCR_USER1')
    if speed_ms == 0 then
        speed_ms = 0.5
    end

    -- calculate the desired horizontal velocity in north and east
    local vel_desired_ne = Vector2f()
    vel_desired_ne:x(math.cos(yaw_target_rad) * speed_ms)
    vel_desired_ne:y(math.sin(yaw_target_rad) * speed_ms)

    -- apply ACC_XY to ramp the speed up / down and limit cornering acceleration
    local vel_diff_ne = Vector2f()
    vel_diff_ne:x(vel_desired_ne:x() - vel_target:x())
    vel_diff_ne:y(vel_desired_ne:y() - vel_target:y())

    local vel_diff_len = vel_diff_ne:length()
    local step_max = ACC_XY * dt
    if vel_diff_len > step_max then
        local scale = step_max / vel_diff_len
        vel_diff_ne:x(vel_diff_ne:x() * scale)
        vel_diff_ne:y(vel_diff_ne:y() * scale)
    end

    -- capture the old velocity for position integration
    local vel_target_old_ne = Vector2f()
    vel_target_old_ne:x(vel_target:x())
    vel_target_old_ne:y(vel_target:y())

    -- update the velocity target
    vel_target:x(vel_target:x() + vel_diff_ne:x())
    vel_target:y(vel_target:y() + vel_diff_ne:y())

    -- update the acceleration target
    acc_target:x(vel_diff_ne:x() / dt)
    acc_target:y(vel_diff_ne:y() / dt)

    -- update the position target
    pos_target:x(pos_target:x() + (vel_target_old_ne:x() + vel_target:x()) * 0.5 * dt)
    pos_target:y(pos_target:y() + (vel_target_old_ne:y() + vel_target:y()) * 0.5 * dt)

    -- get the current HAGL
    local hagl_current
    if rangefinder:has_data_orient(25) and rangefinder:status_orient(25) == 4 then
        hagl_current = rangefinder:distance_orient(25)
    else
        -- fallback if rangefinder lost: maintain current offset
        hagl_current = hagl_target
    end

    -- calculate the vertical error
    local hagl_error = hagl_target - hagl_current

    -- set the velocity offset
    vel_offset:z(-1.0 * (hagl_error * P_GAIN_Z))

    -- limit the vertical velocity correction
    if vel_offset:z() > MAX_VEL_Z then
        vel_offset:z(-MAX_VEL_Z)
    elseif vel_offset:z() < -MAX_VEL_Z then
        vel_offset:z(-MAX_VEL_Z)
    end

    -- integrate the vertical offset
    pos_offset:z(pos_offset:z() + vel_offset:z() * dt)

    -- send the targets to the controllers
    vehicle:set_target_posvelaccel_NED(pos_target, vel_target, acc_target, false, 0, false, 0, false)
    poscontrol:set_posvelaccel_offset(pos_offset, vel_offset, Vector3f())

    -- calculate the horizontal position error
    local current_pos = ahrs:get_relative_position_NED_origin()
    local distance = 0
    if current_pos then
        distance = (pos_target:xy() - current_pos:xy()):length()
    end

    -- log key metrics
    logger:write('GATD', 'HAGL,Offset,TargZ,YInp,YRat,TYaw,PErr,VDL', 'ffffffff', 'mmm-rrmn', '--------', hagl_current, pos_offset:z(), pos_target:z(), yaw_input, yaw_rate_rads, yaw_target_rad, distance, vel_diff_len)

    return update, 1000 / RUN_HZ
end

gcs:send_text(6, "GAT: script loaded")
return update, 1000
