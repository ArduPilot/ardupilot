-- This scirpt uses Rover's turn rate controller to make the vehicle move in circles of fixed radius

-- Edit these variables
local rad_xy_m = 0.5                -- circle radius in xy plane in m
local target_speed_xy_mps = 0.5     -- target speed in m/s
local rc_channel_switch = 7         -- switch this channel to "high" to get the script working
local cw_turn = true                -- change this to false for ccw circle instead of default cw



-- Fixed variables
local omega_radps = target_speed_xy_mps/rad_xy_m
local rover_guided_mode_num = 15
local direction = 1
if not cw_turn then
    direction = -1
end


-- Script Start --

gcs:send_text(0,"Script started")
gcs:send_text(0,"Trajectory period: " .. tostring(2 * math.rad(180) / omega_radps))

local circle_active = false
local last_mode = 0


function update()

    if not circle_active then
        last_mode = vehicle:get_mode()
    end

    if arming:is_armed() and rc:get_pwm(rc_channel_switch) > 1700 and not circle_active then
        -- set guided mode since rc switch is now high
        vehicle:set_mode(rover_guided_mode_num)
        circle_active = true
    elseif arming:is_armed() and rc:get_pwm(rc_channel_switch) < 1200 and circle_active then
        -- set back to last mode since rc switch is low
        vehicle:set_mode(last_mode)
        circle_active = false
    end

    if circle_active then
        --target turn rate in degrees including direction
        local target_turn_rate = math.deg(omega_radps * direction)

        -- send guided message
        if not vehicle:set_desired_turn_rate_and_speed(target_turn_rate, target_speed_xy_mps) then
            gcs:send_text(0, "Failed to send target ")
        end
    end

    return update, 100
end

return update()
