package.path = package.path .. ';./scripts/modules/?.lua'
local pid = require("pid")
local Funcs = require("functions")

local steering_pid = pid:new(0.05, 0.01, 0.0, 0.8, -0.8, 0.8, -0.8)


local yaw_angle_current = 200


-- Try 10 iterations of the PID controller
local rc1_pwm_list = {1600, 1650, 1700, 1750, 1800, 1750, 1700, 1650}
for i = 1, #rc1_pwm_list do
    local desired_yaw = yaw_angle_current
    gcs:send_text(6, "from pid" .. steering_pid.P)

    local TRIM1 = 1500
    local rc1_pwm = rc1_pwm_list[i]
    local addsteering = (rc1_pwm - TRIM1) / 450

    desired_yaw = desired_yaw + 0.1*addsteering
    local vh_yaw = Funcs:map_to_360(Funcs:to_degrees(Funcs:to_radians(yaw_angle_current)))
    local steering_error = Funcs:map_error(vh_yaw - desired_yaw)
    gcs:send_text(6, "Steering error" .. steering_error)

    
    local mysteering = steering_pid:compute(0, -steering_error,0.2)
    gcs:send_text(6, "Steering output" .. mysteering)

end
