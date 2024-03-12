-- Lua "motor driver" for a four legged (aka quadruped) walking robot
-- 
-- This script consumes controller outputs (i.e. roll, pitch, yaw/steering, throttle, lateral) from
-- the vehicle code and then calculates the outputs for 12 servos controlling four legs
--
-- AutoPilot servo connections:
-- Output1: front right coxa (hip) servo
-- Output2: front right femur (thigh) servo
-- Output3: front right tibia (shin) servo
-- Output4: front left coxa (hip) servo
-- Output5: front left femur (thigh) servo
-- Output6: front left tibia (shin) servo
-- Output7: back left coxa (hip) servo
-- Output8: back left femur (thigh) servo
-- Output9: back left tibia (shin) servo
-- Output10: back right coxa (hip) servo
-- Output11: back right femur (thigh) servo
-- Output12: back right tibia (shin) servo
--
-- CAUTION: This script should only be used with ArduPilot Rover's firmware
-- luacheck: only 0


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
local endpoint_LB = {math.cos(math.rad(45))*(COXA_LEN + FEMUR_LEN), math.sin(math.rad(45))*(COXA_LEN + FEMUR_LEN), TIBIA_LEN}
local endpoint_LF = {math.cos(math.rad(45))*(COXA_LEN + FEMUR_LEN), math.sin(math.rad(-45))*(COXA_LEN + FEMUR_LEN), TIBIA_LEN}
local endpoint_RF = {-math.cos(math.rad(45))*(COXA_LEN + FEMUR_LEN), math.sin(math.rad(-45))*(COXA_LEN + FEMUR_LEN), TIBIA_LEN}
local endpoint_RB = {-math.cos(math.rad(45))*(COXA_LEN + FEMUR_LEN), math.sin(math.rad(45))*(COXA_LEN + FEMUR_LEN), TIBIA_LEN}

-- control input enum
local control_input_roll = 1
local control_input_pitch = 2
local control_input_throttle = 3
local control_input_yaw = 4
local control_input_height = 8

local xy_travel_max = 80     -- x and y axis travel max (used to convert control input) in mm
local yaw_travel_max = 10    -- yaw travel maximum (used to convert control input)
local height_max = 40       -- height maximum (used to convert control input)
local travel_dz = 5          -- travel deadzone.  x, y and yaw travel requests are ignored if their absolute value is less than this number
local x_travel = 0           -- target lenght of gait along x
local y_travel = 0           -- target travel of gait along y
local yaw_travel = 0         -- yaw rotation travel target


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
local gait_travel_divisor = 0    -- number of steps in the gait the leg is touching the floor, this is used as a factor to split the travel distance between the steps

local gait_pos_x = {0,0,0,0}    -- X-axis position for each leg (back-right, front-right, back-left, front-left)
local gait_pos_y = {0,0,0,0}    -- Y-axis position for each leg (back-right, front-right, back-left, front-left)
local gait_pos_z = {0,0,0,0}    -- Z-axis position for each leg (back-right, front-right, back-left, front-left)
local gait_rot_z = {0,0,0,0}    -- Z-axis rotation for each leg (back-right, front-right, back-left, front-left)
local last_angle = {0,0,0,0,0,0,0,0,0,0,0,0}
local start_time = 0
local curr_target = 0

function Gaitselect()
    if (gait_type == 0) then
        -- alternating gait
        gait_step_total = 6
        gait_step_leg_start = {1,4,4,1}
        gait_lifted_steps = 2
        gait_down_steps = 1
        gait_lift_divisor = 2
        gait_half_lift_height = 1
        gait_travel_divisor = 4
    elseif (gait_type == 1) then
        -- wave gait with 28 steps
        gait_step_total = 28
        gait_step_leg_start = {8,15,1,22}
        gait_lifted_steps = 3
        gait_down_steps = 2
        gait_lift_divisor = 2
        gait_half_lift_height = 3
        gait_travel_divisor = 24
    end
end

-- Calculate Gait sequence
function calc_gait_sequence()
    local move_requested = (math.abs(x_travel) > travel_dz) or (math.abs(y_travel) > travel_dz) or (math.abs(yaw_travel) > travel_dz)

    if move_requested then
        for leg_index=1, 4 do
            update_leg(leg_index,move_requested)
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
-- distance which is decided by the x_travel, yaw_travel, y_travel
function update_leg(moving_leg,move_requested)
    local leg_step = gait_step - gait_step_leg_start[moving_leg]
    
    if ((move_requested and (gait_lifted_steps > 0) and leg_step==0) or
        (not move_requested and leg_step==0 and ((gait_pos_x[moving_leg]>2) or
        (gait_pos_y[moving_leg]>2) or (gait_rot_z[moving_leg] >2)))) then
        gait_pos_x[moving_leg] = 0
        gait_pos_z[moving_leg] = -leg_lift_height
        gait_pos_y[moving_leg] = 0
        gait_rot_z[moving_leg] = 0

    elseif (((gait_lifted_steps==2 and leg_step==0) or (gait_lifted_steps>=3 and
            (leg_step==-1 or leg_step==(gait_step_total-1)))) and move_requested) then
        gait_pos_x[moving_leg] = -x_travel/gait_lift_divisor
        gait_pos_z[moving_leg] = -3*leg_lift_height/(3+gait_half_lift_height)
        gait_pos_y[moving_leg] = -y_travel/gait_lift_divisor
        gait_rot_z[moving_leg] = -yaw_travel/gait_lift_divisor

    elseif ((gait_lifted_steps>=2) and (leg_step==1 or leg_step==-(gait_step_total-1)) and move_requested) then
        gait_pos_x[moving_leg] = x_travel/gait_lift_divisor
        gait_pos_z[moving_leg] = -3*leg_lift_height/(3+gait_half_lift_height)
        gait_pos_y[moving_leg] = y_travel/gait_lift_divisor
        gait_rot_z[moving_leg] = yaw_travel/gait_lift_divisor

    elseif (((gait_lifted_steps==5 and (leg_step==-2 ))) and move_requested) then
        gait_pos_x[moving_leg] = -x_travel * 0.5
        gait_pos_z[moving_leg] = -leg_lift_height * 0.5
        gait_pos_y[moving_leg] = -y_travel * 0.5
        gait_rot_z[moving_leg] = -yaw_travel * 0.5

    elseif ((gait_lifted_steps==5) and (leg_step==2 or leg_step==-(gait_step_total-2)) and move_requested) then
        gait_pos_x[moving_leg] = x_travel * 0.5
        gait_pos_z[moving_leg] = -leg_lift_height * 0.5
        gait_pos_y[moving_leg] = y_travel * 0.5
        gait_rot_z[moving_leg] = yaw_travel * 0.5

    elseif ((leg_step==gait_down_steps or leg_step==-(gait_step_total-gait_down_steps)) and gait_pos_y[moving_leg]<0) then
        gait_pos_x[moving_leg] = x_travel * 0.5
        gait_pos_z[moving_leg] = 0
        gait_pos_y[moving_leg] = y_travel * 0.5
        gait_rot_z[moving_leg] = yaw_travel * 0.5

    else
        gait_pos_x[moving_leg] = gait_pos_x[moving_leg] - (x_travel/gait_travel_divisor)
        gait_pos_z[moving_leg] = 0
        gait_pos_y[moving_leg] = gait_pos_y[moving_leg] - (y_travel/gait_travel_divisor)
        gait_rot_z[moving_leg] = gait_rot_z[moving_leg] - (yaw_travel/gait_travel_divisor)
    end
end

-- Body Forward Kinematics calculates where each leg should be.
-- inputs are
--   a) body rotations: body_rot_x, body_rot_y, body_rot_z
--   b) body position: body_pos_x, body_pos_y, body_pos_z
--   c) offset of the center of body
function body_forward_kinematics(X, Y, Z, Xdist, Ydist, Zrot)
    local totaldist_x = X + Xdist + body_pos_x
    local totaldist_y = Y + Ydist + body_pos_y
    local distBodyCenterFeet = math.sqrt(totaldist_x^2 + totaldist_y^2)
    local AngleBodyCenter = math.atan(totaldist_y, totaldist_x)
    local rolly = math.tan(math.rad(body_rot_y)) * totaldist_x
    local pitchy = math.tan(math.rad(body_rot_x)) * totaldist_y

    local ansx = math.cos(AngleBodyCenter + math.rad(body_rot_z+Zrot)) * distBodyCenterFeet - totaldist_x + body_pos_x
    local ansy = math.sin(AngleBodyCenter + math.rad(body_rot_z+Zrot)) * distBodyCenterFeet - totaldist_y + body_pos_y
    local ansz = rolly + pitchy + body_pos_z
    return {ansx, ansy, ansz}
end

-- Leg Inverse Kinematics calculates the angles for each servo of each joint using the output of the
-- body_forward_kinematics() function which gives the origin of each leg on the body frame
function leg_inverse_kinematics(x, y, z)
    local coxa = math.deg(math.atan(x, y))
    local trueX = math.sqrt(x^2 + y^2) - COXA_LEN
    local im = math.sqrt(trueX^2 + z^2)

    local q1 = -math.atan(z, trueX)
    local d1 = FEMUR_LEN^2 - TIBIA_LEN^2 + im^2
    local d2 = 2*FEMUR_LEN*im
    local q2 = math.acos(d1/d2)
    local femur = math.deg(q1+q2)

    d1 = FEMUR_LEN^2 - im^2 + TIBIA_LEN^2
    d2 = 2*TIBIA_LEN*FEMUR_LEN
    local tibia = math.deg(math.acos(d1/d2)-math.rad(90))
    return {coxa, -femur, -tibia}
end

-- checks if the servo has moved to its expected position 
function servo_estimate(current_angle)
    local target = 0
    for j = 1, 12 do
        curr_target = math.abs(current_angle[j] - last_angle[j])
        if curr_target > target then
            target = curr_target
        end
    end
    local target_time = target * (0.24/60) * 1000
    return (millis() - start_time) > target_time
end

-- main_inverse_kinematics produces the inverse kinematic solution for each
-- leg joint servo by taking into consideration the initial_pos, gait offset and the body inverse kinematic values.
function main_inverse_kinematics()
    local ans_RB = body_forward_kinematics(endpoint_RB[1]+gait_pos_x[1],
                                           endpoint_RB[2]+gait_pos_y[1], 
                                           endpoint_RB[3]+gait_pos_z[1],
                                           -FRAME_LEN*0.5, FRAME_WIDTH*0.5, 
                                           gait_rot_z[1])
    local angles_RB = leg_inverse_kinematics(endpoint_RB[1]+ans_RB[1]+gait_pos_x[1],
                                             endpoint_RB[2]+ans_RB[2]+gait_pos_y[1], 
                                             endpoint_RB[3]+ans_RB[3]+gait_pos_z[1])
    angles_RB[1] = 45 + angles_RB[1]

    local ans_RF = body_forward_kinematics(endpoint_RF[1]+gait_pos_x[2],
                                           endpoint_RF[2]+gait_pos_y[2],
                                           endpoint_RF[3]+gait_pos_z[2],
                                          -FRAME_LEN*0.5, -FRAME_WIDTH*0.5,
                                           gait_rot_z[2])
    local angles_RF = leg_inverse_kinematics(endpoint_RF[1]-ans_RF[1]+gait_pos_x[2],
                                             endpoint_RF[2]-ans_RF[2]-gait_pos_y[2], 
                                             endpoint_RF[3]+ans_RF[3]+gait_pos_z[2])
    angles_RF[1] = 135 + angles_RF[1]

    local ans_LB = body_forward_kinematics(endpoint_LB[1]+gait_pos_x[3],
                                           endpoint_LB[2]+gait_pos_y[3], 
                                           endpoint_LB[3]+gait_pos_z[3], 
                                           FRAME_LEN*0.5, FRAME_WIDTH*0.5,
                                           gait_rot_z[3])
    local angles_LB = leg_inverse_kinematics(endpoint_LB[1]+ans_LB[1]+gait_pos_x[3],
                                             endpoint_LB[2]+ans_LB[2]+gait_pos_y[3], 
                                             endpoint_LB[3]+ans_LB[3]+gait_pos_z[3])
    angles_LB[1] = -45 + angles_LB[1]

    local ans_LF = body_forward_kinematics(endpoint_LF[1]+gait_pos_x[4],
                                           endpoint_LF[2]+gait_pos_y[4], 
                                           endpoint_LF[3]+gait_pos_z[4], 
                                           FRAME_LEN*0.5, -FRAME_WIDTH*0.5, 
                                           gait_rot_z[4])
    local angles_LF = leg_inverse_kinematics(endpoint_LF[1]-ans_LF[1]+gait_pos_x[4],
                                             endpoint_LF[2]-ans_LF[2]-gait_pos_y[4], 
                                             endpoint_LF[3]+ans_LF[3]+gait_pos_z[4])
    angles_LF[1] = -135 + angles_LF[1]
    Gaitselect()
    local current_angle = {angles_RF[1],angles_RF[2],angles_RF[3],
                           angles_LF[1],angles_LF[2],angles_LF[3],
                           angles_LB[1],angles_LB[2],angles_LB[3],
                           angles_RB[1],angles_RB[2],angles_RB[3]}

    if servo_estimate(current_angle) then
        start_time = millis()
        calc_gait_sequence()
        last_angle = current_angle
    end

    return current_angle
end

-- servo angles when robot is disarmed and resting body on the ground
local rest_angles = { 45, -90, 40,      -- front right leg (coxa, femur, tibia)
                     -45, -90, 40,      -- front left leg (coxa, femur, tibia)
                     -45, -90, 40,      -- back left leg (coxa, femur, tibia)
                      45, -90, 40}      -- back right leg (coxa, femur, tibia)

function update()
    local throttle = vehicle:get_control_output(control_input_throttle) * xy_travel_max
    local gait_direction
    if throttle > 0 then
        gait_direction = -1
        y_travel = throttle
    elseif throttle < 0 then
        gait_direction = 1
        y_travel = -throttle
    elseif throttle == 0 then
        gait_direction = 1
        y_travel = 0
    end

    yaw_travel = -vehicle:get_control_output(control_input_yaw) * yaw_travel_max
    body_rot_x = -vehicle:get_control_output(control_input_roll) * body_rot_max
    body_rot_y = -vehicle:get_control_output(control_input_pitch) * body_rot_max
    body_pos_z =  vehicle:get_control_output(control_input_height) * height_max

    local servo_direction = { gait_direction *  1, -1,  1,    -- front right leg (coxa, femur, tibia)
                              gait_direction *  1,  1, -1,    -- front left leg (coxa, femur, tibia)
                              gait_direction * -1, -1,  1,    -- back left leg (coxa, femur, tibia)
                              gait_direction * -1,  1, -1}    -- back right leg (coxa, femur, tibia)
                              
    local angles
    if arming:is_armed() then
        angles = main_inverse_kinematics()
    else
        angles = rest_angles
    end

    for i = 1, 12 do
        SRV_Channels:set_output_pwm_chan_timeout(i-1, math.floor(((angles[i] * servo_direction[i] * 1000)/90) + 1500), 1000)
    end

    return update,10
end

-- turn off rudder based arming/disarming
param:set_and_save('ARMING_RUDDER', 0)
gcs:send_text(0, "quadruped simulation")
return update()
