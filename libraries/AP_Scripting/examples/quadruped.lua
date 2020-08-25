-- quadruped robot script 

local FRAME_LEN = 80    -- frame length in mm
local FRAME_WIDTH = 150 -- frame width in mm

local COXA_LEN = 30     -- distance (in mm) from coxa (aka hip) servo to femur servo
local FEMUR_LEN = 85    -- distance (in mm) from femur servo to tibia servo
local TIBIA_LEN = 125   -- distance (in mm) from tibia servo to foot

--body position and rotation parameters
local body_rot_max = 10 -- body rotation maximum for any individual axis
local body_rot_x = 0    -- body rotation about the X axis (i.e. roll rotation)
local body_rot_y = 0    -- body rotation about the Y axis (i.e. pitch rotation)
local body_rot_z = 0    -- body rotation about the Z axis (i.e. yaw rotation)
local body_pos_x = 0    -- body position in the X axis (i.e. forward, back).  should be -40mm to +40mm
local body_pos_y = 0    -- body position in the Y axis (i.e. right, left).  should be -40mm to +40mm
local body_pos_z = 0    -- body position in the Z axis (i.e. up, down).  should be -40mm to +40mm

-- starting positions of the legs
local endpoints1 = {math.cos(45/180*math.pi)*(COXA_LEN + FEMUR_LEN), math.sin(45/180*math.pi)*(COXA_LEN + FEMUR_LEN), TIBIA_LEN }
local endpoints2 = {math.cos(45/180*math.pi)*(COXA_LEN + FEMUR_LEN), math.sin(-45/180*math.pi)*(COXA_LEN + FEMUR_LEN), TIBIA_LEN }
local endpoints3 = {-math.cos(45/180*math.pi)*(COXA_LEN + FEMUR_LEN), math.sin(-45/180*math.pi)*(COXA_LEN + FEMUR_LEN), TIBIA_LEN }
local endpoints4 = {-math.cos(45/180*math.pi)*(COXA_LEN + FEMUR_LEN), math.sin(45/180*math.pi)*(COXA_LEN + FEMUR_LEN), TIBIA_LEN }

-- control input enum
local control_input_roll = 1
local control_input_pitch = 2
local control_input_throttle = 3
local control_input_yaw = 4

local xy_speed_max = 40     -- x and y axis speed max (used to convert control input) in mm
local yaw_speed_max = 10    -- yaw speed maximum (used to convert control input)
local speed_dz = 5          -- speed deadzone.  x, y and yaw speed requests are ignored if their absolute value is less than this number
local x_speed = 0           -- forward speed target
local y_speed = 0           -- lateral speed target (right is positive, left is negative)
local yaw_speed = 0         -- yaw rotation speed target

local leg_lift_height = 50  -- leg lift height (in mm) while walking

-- gait definition parameters
local gait_type = 0         -- gait pattern.  0 = alternating gait, 1 = wave gait.
local gait_step = 0         -- gait step in execution
local gait_step_total = 0   -- number of steps in gait
local gait_step_leg_start = {0,0,0,0} -- leg starts moving on this gait step (front-right, front-left, back-left, back-right)
local gait_lifted_steps = 0 -- number of steps that a leg is lifted for
local gait_down_steps = 0   -- number of steps that the leg lifted needs to be put down for
local gait_lift_divisor = 0 -- when a leg is lifted and brought back down the action is divided into 2 or multiple steps, so the travel distance also need to be split in between the steps to make the transition natural
local gait_half_lift_height = 0 -- used to split lift across two steps
local gait_speed_divisor = 0    -- number of steps in the gait the leg is touching the floor, this is used as a factor to split the travel distance between the steps

local gait_pos_x = {0,0,0,0}    -- X-axis position for each leg (front-right, front-left, back-right, back-left)
local gait_pos_y = {0,0,0,0}    -- Y-axis position for each leg (front-right, front-left, back-right, back-left)
local gait_pos_z = {0,0,0,0}    -- Z-axis position for each leg (front-right, front-left, back-right, back-left)
local gait_rot_z = {0,0,0,0}    -- Z-axis rotation for each leg (front-right, front-left, back-right, back-left)
local leg_index = 0
local last_angle = {0,0,0,0,0,0,0,0,0,0,0,0}
local current = {0,0,0,0,0,0,0,0,0,0,0,0}
local start_time = 0
local curr_target = 0
local pwm = {1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500}

function Gaitselect()
    if (gait_type == 0) then
        -- alternating gait
        gait_step_total = 6
        gait_step_leg_start = {1,4,1,4}
        gait_lifted_steps = 2
        gait_down_steps = 1
        gait_lift_divisor = 2
        gait_half_lift_height = 1
        gait_speed_divisor = 4
    elseif (gait_type == 1) then
        -- wave gait with 28 steps
        gait_step_total = 28
        gait_step_leg_start = {8,15,1,22}
        gait_lifted_steps = 3
        gait_down_steps = 2
        gait_lift_divisor = 2
        gait_half_lift_height = 3
        gait_speed_divisor = 24
    end
end

-- Calculate Gait sequence
function calc_gait_sequence()
    local move_requested = (math.abs(x_speed) > speed_dz) or (math.abs(y_speed) > speed_dz) or (math.abs(yaw_speed) > speed_dz)

    if move_requested then
        for leg_index=1, 4 do
            update_leg(leg_index)
        end

        gait_step = gait_step + 1
        if (gait_step>gait_step_total) then
            gait_step = 1
        end
    else
        gait_pos_x = {0,0,0,0}
        gait_pos_y = {0,0,0,0}
        gait_pos_z = {0,0,0,0}
        gait_rot_z = {0,0,0,0}
    end
end

-- in order for the robot to move forward it needs to move its legs in a
-- specific order and this is repeated over and over to attain linear motion. when a
-- specific leg number is passed the update_leg() produces the set of values for the
-- given leg at that step, for each cycle of the gait each leg will move to a set
-- distance which is decided by the x_speed, yaw_speed, y_speed
function update_leg(moving_leg)
    local leg_step = gait_step - gait_step_leg_start[moving_leg]
    local move_requested = (math.abs(x_speed) > speed_dz) or (math.abs(y_speed) > speed_dz) or (math.abs(yaw_speed) > speed_dz)

    if ((move_requested and (gait_lifted_steps and 1) and leg_step==0) or
        (not move_requested and leg_step==0 and ((gait_pos_x[moving_leg]>2) or
        (gait_pos_z[moving_leg]>2) or (gait_rot_z[moving_leg] >2)))) then
        gait_pos_x[moving_leg] = 0
        gait_pos_z[moving_leg] = -leg_lift_height
        gait_pos_y[moving_leg] = 0
        gait_rot_z[moving_leg] = 0

    elseif (((gait_lifted_steps==2 and leg_step==0) or (gait_lifted_steps>=3 and
            (leg_step==-1 or leg_step==(gait_step_total-1)))) and move_requested) then
        gait_pos_x[moving_leg] = -x_speed/gait_lift_divisor
        gait_pos_z[moving_leg] = -3*leg_lift_height/(3+gait_half_lift_height)
        gait_pos_y[moving_leg] = -y_speed/gait_lift_divisor
        gait_rot_z[moving_leg] = -yaw_speed/gait_lift_divisor

    elseif ((gait_lifted_steps>=2) and (leg_step==1 or leg_step==-(gait_step_total-1)) and move_requested) then
        gait_pos_x[moving_leg] = x_speed/gait_lift_divisor
        gait_pos_z[moving_leg] = -3*leg_lift_height/(3+gait_half_lift_height)
        gait_pos_y[moving_leg] = y_speed/gait_lift_divisor
        gait_rot_z[moving_leg] = yaw_speed/gait_lift_divisor

    elseif (((gait_lifted_steps==5 and (leg_step==-2 ))) and move_requested) then
        gait_pos_x[moving_leg] = -x_speed/2
        gait_pos_z[moving_leg] = -leg_lift_height/2
        gait_pos_y[moving_leg] = -y_speed/2
        gait_rot_z[moving_leg] = -yaw_speed/2

    elseif ((gait_lifted_steps==5) and (leg_step==2 or leg_step==-(gait_step_total-2)) and move_requested) then
        gait_pos_x[moving_leg] = x_speed/2
        gait_pos_z[moving_leg] = -leg_lift_height/2
        gait_pos_y[moving_leg] = y_speed/2
        gait_rot_z[moving_leg] = yaw_speed/2

    elseif ((leg_step==gait_down_steps or leg_step==-(gait_step_total-gait_down_steps)) and gait_pos_y[moving_leg]<0) then
        gait_pos_x[moving_leg] = x_speed/2
        gait_pos_z[moving_leg] = y_speed/2
        gait_pos_y[moving_leg] = yaw_speed/2
        gait_rot_z[moving_leg] = 0

    else
        gait_pos_x[moving_leg] = gait_pos_x[moving_leg] - (x_speed/gait_speed_divisor)
        gait_pos_z[moving_leg] = 0
        gait_pos_y[moving_leg] = gait_pos_z[moving_leg] - (y_speed/gait_speed_divisor)
        gait_rot_z[moving_leg] = gait_rot_z[moving_leg] - (yaw_speed/gait_speed_divisor)
    end
end

-- Body Forward Kinematics calculates where each leg should be.
-- inputs are
--   a) body rotations: body_rot_x, body_rot_y, body_rot_z
--   b) body position: body_pos_x, body_pos_y, body_pos_z
--   c) offset of the center of body
function body_forward_kinematics(X, Y, Z, Xdist, Ydist, Zrot)
    local totaldist = {X + Xdist + body_pos_x, Y + Ydist + body_pos_y}
    local distBodyCenterFeet = math.sqrt(totaldist[1]^2 + totaldist[2]^2)
    local AngleBodyCenter = math.atan(totaldist[2], totaldist[1])
    local rolly = math.tan(body_rot_y * math.pi/180) * totaldist[1]
    local pitchy = math.tan(body_rot_x * math.pi/180) * totaldist[2]

    local ansx = math.cos(AngleBodyCenter + ((body_rot_z+Zrot)  * math.pi/180)) * distBodyCenterFeet - totaldist[1] + body_pos_x
    local ansy = math.sin(AngleBodyCenter + ((body_rot_z+Zrot) * math.pi/180)) * distBodyCenterFeet - totaldist[2] + body_pos_y
    local ansz = rolly + pitchy + body_pos_z
    local ans = {ansx, ansy, ansz}
    return ans
end

-- Leg Inverse Kinematics calculates the angles for each servo of each joint using the output of the
-- body_forward_kinematics() function which gives the origin of each leg on the body frame
function leg_inverse_kinematics(X , Y , Z)
    local coxa = math.atan(X,Y)* 180/math.pi
    local trueX = math.sqrt(X^2+ Y^2 ) - COXA_LEN
    local im = math.sqrt(trueX^2 + Z^2)

    local q1 = -math.atan(Z,trueX)
    local d1 = FEMUR_LEN^2 - TIBIA_LEN^2 + im^2
    local d2 = 2*FEMUR_LEN*im
    local q2 = math.acos(d1/d2)
    local femur = (q1+q2) * 180/math.pi

    local d1 = FEMUR_LEN^2 - im^2 + TIBIA_LEN^2
    local d2 = 2*TIBIA_LEN*FEMUR_LEN
    local tibia = (math.acos(d1/d2)-1.57) * 180/math.pi
    local ang = { coxa, -femur ,-tibia}
    return ang
end

-- checks if the servo has moved to its expected postion 
function servo_estimate(start_time,current,last_angle)
    local target = 0
    for j = 1, 12 do
        curr_target = math.abs(current[j] - last_angle[j])
        if curr_target > target then
            target = curr_target
        end
    end
    local target_time = target * (0.24/60) * 1000
    local now = millis()

    if (target_time + start_time) <= now then
        return true
    else
        return false
    end
end

-- main_inverse_kinematics produces the inverse kinematic solution for each
-- leg joint servo by taking into consideration the initial_pos, gait offset and the body inverse kinematic values.
function main_inverse_kinematics()
    local ans1 = body_forward_kinematics(endpoints1[1]+gait_pos_x[1], endpoints1[2]+gait_pos_y[1], endpoints1[3]+gait_pos_z[1], FRAME_LEN/2, FRAME_WIDTH/2,gait_rot_z[1])
    local angles1 = leg_inverse_kinematics(endpoints1[1]+ans1[1]+gait_pos_x[1],endpoints1[2]+ans1[2]+gait_pos_y[1], endpoints1[3]+ans1[3]+gait_pos_z[1])
    angles1 = {-45 + angles1[1],angles1[2],angles1[3]}

    local ans2 = body_forward_kinematics(endpoints2[1]+gait_pos_x[2], endpoints2[2]+gait_pos_y[2], endpoints2[3]+gait_pos_z[2], FRAME_LEN/2, -FRAME_WIDTH/2,gait_rot_z[2])
    local angles2 = leg_inverse_kinematics(endpoints2[1]+ans2[1]+gait_pos_x[2],endpoints2[2]+ans2[2]+gait_pos_y[2], endpoints2[3]+ans2[3]+gait_pos_z[2])
    angles2 = {-135 + angles2[1],angles2[2],angles2[3]}

    local ans3 = body_forward_kinematics(endpoints3[1]+gait_pos_x[3], endpoints3[2]+gait_pos_y[3], endpoints3[3]+gait_pos_z[3], -FRAME_LEN/2, -FRAME_WIDTH/2,gait_rot_z[3])
    local angles3 = leg_inverse_kinematics(endpoints3[1]+ans3[1]+gait_pos_x[3],endpoints3[2]+ans3[2]+gait_pos_y[3], endpoints3[3]+ans3[3]+gait_pos_z[3])
    angles3 = {135 + angles3[1],angles3[2],angles3[3]}

    local ans4 = body_forward_kinematics(endpoints4[1]+gait_pos_x[4], endpoints4[2]+gait_pos_y[4], endpoints4[3]+gait_pos_z[4], -FRAME_LEN/2, FRAME_WIDTH/2,gait_rot_z[4])
    local angles4 = leg_inverse_kinematics(endpoints4[1]+ans4[1]+gait_pos_x[4],endpoints4[2]+ans4[2]+gait_pos_y[4], endpoints4[3]+ans4[3]+gait_pos_z[4])
    angles4 = {45 + angles4[1],angles4[2],angles4[3]}
    Gaitselect()
    current = {angles1[1],angles1[2],angles1[3],angles2[1],angles2[2],angles2[3],angles3[1],angles3[2],angles3[3],angles4[1],angles4[2],angles4[3]}

    if servo_estimate(start_time, current, last_angle) then
        start_time = millis()
        calc_gait_sequence()
        last_angle = current
    end

    return angles1, angles4, angles3, angles2
end

-- servo angles when robot is disarmed and resting body on the ground
local rest_angles = { 45, -90, 40,      -- front right leg (coxa, femur, tibia)
                     -45, -90, 40,      -- front left leg (coxa, femur, tibia)
                      45, -90, 40,      -- back right leg (coxa, femur, tibia)
                     -45, -90, 40}      -- back left leg (coxa, femur, tibia)

local servo_direction = { 1,  1,  1,    -- front right leg (coxa, femur, tibia)
                         -1, -1, -1,    -- front left leg (coxa, femur, tibia)
                         -1, -1,  1,    -- back right leg (coxa, femur, tibia)
                          1,  1, -1}    -- back left leg (coxa, femur, tibia)

function update()
    x_speed = vehicle:get_control_output(control_input_throttle) * xy_speed_max
    yaw_speed = vehicle:get_control_output(control_input_yaw) * yaw_speed_max

    body_rot_x = -vehicle:get_control_output(control_input_roll) * body_rot_max
    body_rot_y = -vehicle:get_control_output(control_input_pitch) * body_rot_max

    if arming:is_armed() then
        FR_angles, BL_angles, BR_angles, FL_angles = main_inverse_kinematics()
        angles = { FR_angles[1],FR_angles[2],FR_angles[3] , FL_angles[1],FL_angles[2],FL_angles[3],BR_angles[1],BR_angles[2],BR_angles[3], BL_angles[1],BL_angles[2],BL_angles[3]}
    else
        angles = rest_angles
    end

    for j = 1, 12 do
        pwm[j] = math.floor(((angles[j] * servo_direction[j] * 1000)/90) + 1500)
    end

    for i = 1, 12 do
        SRV_Channels:set_output_pwm_chan_timeout(i-1, pwm[i], 1000)
    end

    return update,10
end

-- turn off rudder based arming/disarming
param:set_and_save('ARMING_RUDDER', 0)
gcs:send_text(0, "quadruped simulation")
return update()
