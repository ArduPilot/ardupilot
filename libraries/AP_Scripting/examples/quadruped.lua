-- quadruped robot script 

local L = 80  -- length of frame
local W = 150  -- width of frame 

local L_COXA = 30 --distance from coxa servo to femur servo
local L_FEMUR = 85    --distance from femur servo to tibia servo
local L_TIBIA = 125   --distance from tibia servo to foot

--body position and rotation parameters
local bodyRotX = 0
local bodyRotY = 0         
local bodyRotZ = 0    
local bodyPosX = 0     
local bodyPosY = 0            
local bodyPosZ = 0

-- starting positions of the legs
local endpoints1 = {math.cos(45/180*math.pi)*(L_COXA + L_FEMUR), math.sin(45/180*math.pi)*(L_COXA + L_FEMUR),      L_TIBIA }
local endpoints2 = {math.cos(45/180*math.pi)*(L_COXA + L_FEMUR), math.sin(-45/180*math.pi)*(L_COXA + L_FEMUR),      L_TIBIA }
local endpoints3 = {-math.cos(45/180*math.pi)*(L_COXA + L_FEMUR), math.sin(-45/180*math.pi)*(L_COXA + L_FEMUR),      L_TIBIA }
local endpoints4 = {-math.cos(45/180*math.pi)*(L_COXA + L_FEMUR), math.sin(45/180*math.pi)*(L_COXA + L_FEMUR),      L_TIBIA }

--select a gait pattern(default gait = 0)
local GaitType = 0
--lift height while walking 
local LegLiftHeight = 50  
--gait step in exectution 
local GaitStep = 0   
--initial position of the leg
local GaitLegNr = {0,0,0,0}  
local TLDivFactor = 0       
local NrLiftedPos = 0    
local LiftDivFactor = 0    
local HalfLiftHeigth = 0     
local FrontDownPos = 0			
local TravelRequest = false       
--Number of steps in gait     
local StepsInGait = 0 
local GaitStep = 0         
local GaitPosX = {0,0,0,0}         
local GaitPosY = {0,0,0,0}            
local GaitPosZ = {0,0,0,0}             
local GaitRotZ = {0,0,0,0}      
local LegIndex = 0   
local Walking = false
local X_speed = 0
local Yaw_speed = 0
local Y_speed = 0
local DeadZone = 5
local last_angle = {0,0,0,0,0,0,0,0,0,0,0,0}
local current = {0,0,0,0,0,0,0,0,0,0,0,0}
local start_time = 0
local curr_target = 0
local throttle = 3
local yaw = 4
local roll = 1
local pitch = 2
local gait = 5
local mode = 6
local max_step_factor = 40
local max_rotation_factor = 10
local max_yaw_factor = 10
local pwm = { 1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500}
-- turn off rudder based arming/disarming
param:set_and_save('ARMING_RUDDER',0)
function Gaitselect()
    --Alternating gait
    if (GaitType == 0) then
        GaitLegNr = {1,4,1,4}
        NrLiftedPos = 2
        FrontDownPos = 1	
        LiftDivFactor = 2
        HalfLiftHeigth = 1
        TLDivFactor = 4   
        StepsInGait = 6    
    elseif (GaitType == 1) then
    -- wave gait with 28 steps
        GaitLegNr = {8,15,1,22}
        NrLiftedPos = 3
        FrontDownPos = 2	
        LiftDivFactor = 2
        HalfLiftHeigth = 3
        TLDivFactor = 24 
        StepsInGait = 28
    end 
end

-- Calculate Gait sequence
function Sequence_Gen()
    TravelRequest =(math.abs(X_speed) > DeadZone) or (math.abs(Y_speed) > DeadZone) or (math.abs(Yaw_speed) > DeadZone) 
    
    if TravelRequest then
        for LegIndex=1,4,1 
        do 
            Gaitgen(LegIndex)
        end

        GaitStep = GaitStep + 1
        if (GaitStep>StepsInGait) then
            GaitStep = 1
        end 
    else
        GaitPosX = {0,0,0,0}         
        GaitPosY = {0,0,0,0}            
        GaitPosZ = {0,0,0,0}             
        GaitRotZ = {0,0,0,0}  
    end
end

-- In order for the bot to move forward it needs to move its legs in a
-- specific order and this is repeated over and over to attain linear motion . when a
-- specific leg number is passed the Gaitgen() produces the set off values for the
-- given leg at that step , for each cycle of the gait each leg will move to a set
-- distance which is decided by the X_speed,Yaw_speed,Y_speed
function Gaitgen(moving_leg)
    local LegStep = GaitStep - GaitLegNr[moving_leg]

    if ((TravelRequest and (NrLiftedPos and 1) and 
    LegStep==0) or 
    (not TravelRequest and LegStep==0 and ((GaitPosX[moving_leg]>2) or 
    (GaitPosZ[moving_leg]>2) or (GaitRotZ[moving_leg] >2)))) 
    then
        GaitPosX[moving_leg] = 0
        GaitPosZ[moving_leg] = -LegLiftHeight
        GaitPosY[moving_leg] = 0
        GaitRotZ[moving_leg] = 0

    elseif (((NrLiftedPos==2 and LegStep==0) or (NrLiftedPos>=3 and 
    (LegStep==-1 or LegStep==(StepsInGait-1))))
    and TravelRequest)
    then
        GaitPosX[moving_leg] = -X_speed/LiftDivFactor
        GaitPosZ[moving_leg] = -3*LegLiftHeight/(3+HalfLiftHeigth)  
        GaitPosY[moving_leg] = -Y_speed/LiftDivFactor
        GaitRotZ[moving_leg] = -Yaw_speed/LiftDivFactor

    elseif ((NrLiftedPos>=2) and (LegStep==1 or LegStep==-(StepsInGait-1)) and TravelRequest)
    then
        GaitPosX[moving_leg] = X_speed/LiftDivFactor
        GaitPosZ[moving_leg] = -3*LegLiftHeight/(3+HalfLiftHeigth)  
        GaitPosY[moving_leg] = Y_speed/LiftDivFactor
        GaitRotZ[moving_leg] = Yaw_speed/LiftDivFactor

    elseif (((NrLiftedPos==5 and (LegStep==-2 ))) and TravelRequest)
    then
        GaitPosX[moving_leg] = -X_speed/2
        GaitPosZ[moving_leg] = -LegLiftHeight/2 
        GaitPosY[moving_leg] = -Y_speed/2
        GaitRotZ[moving_leg] = -Yaw_speed/2

    elseif ((NrLiftedPos==5) and (LegStep==2 or LegStep==-(StepsInGait-2)) and TravelRequest)
    then
        GaitPosX[moving_leg] = X_speed/2
        GaitPosZ[moving_leg] = -LegLiftHeight/2 
        GaitPosY[moving_leg] = Y_speed/2
        GaitRotZ[moving_leg] = Yaw_speed/2

    elseif ((LegStep==FrontDownPos or LegStep==-(StepsInGait-FrontDownPos)) and GaitPosY[moving_leg]<0)
    then
        GaitPosX[moving_leg] = X_speed/2
        GaitPosZ[moving_leg] = Y_speed/2
        GaitPosY[moving_leg] = Yaw_speed/2   
        GaitRotZ[moving_leg] = 0

    else 
        GaitPosX[moving_leg] = GaitPosX[moving_leg] - (X_speed/TLDivFactor)
        GaitPosZ[moving_leg] = 0
        GaitPosY[moving_leg] = GaitPosZ[moving_leg] - (Y_speed/TLDivFactor)
        GaitRotZ[moving_leg] = GaitRotZ[moving_leg] - (Yaw_speed/TLDivFactor)
    end
end

-- This function determines where each leg should be.
-- To calculate each legs pose this takes pose of the body
-- as input bodyRotX, bodyRotY, bodyRotZ - rotation of and body ,bodyPosX,
-- bodyPosY - offset of the center of body
function Body_FK(X , Y , Z,   Xdist, Ydist,Zrot)
    local totaldist = { X + Xdist + bodyPosX, Y + Ydist + bodyPosY }
    local distBodyCenterFeet = math.sqrt(totaldist[1]^2 + totaldist[2]^2)
    local AngleBodyCenter = math.atan(totaldist[2], totaldist[1])
    local rolly = math.tan(bodyRotY * math.pi/180) * totaldist[1]
    local pitchy = math.tan(bodyRotX * math.pi/180) * totaldist[2]

    local ansx = math.cos(AngleBodyCenter + ((bodyRotZ+Zrot)  * math.pi/180)) * distBodyCenterFeet - totaldist[1] + bodyPosX
    local ansy = math.sin(AngleBodyCenter + ((bodyRotZ+Zrot) * math.pi/180)) * distBodyCenterFeet - totaldist[2] + bodyPosY
    local ansz = rolly+pitchy + bodyPosZ
    local ans = {ansx, ansy ,ansz}
    return ans 
end 

-- Calculates the angles of servos of each joint using the output of the
-- Body_FK() function which gives the origin of each leg on the body frame
function Leg_IK(X , Y , Z)
    local coxa = math.atan(X,Y)* 180/math.pi
    local trueX = math.sqrt(X^2+ Y^2 ) - L_COXA
    local im = math.sqrt(trueX^2 + Z^2)

    local q1 = -math.atan(Z,trueX)
    local d1 = L_FEMUR^2 - L_TIBIA^2 + im^2
    local d2 = 2*L_FEMUR*im
    local q2 = math.acos(d1/d2)
    local femur = (q1+q2) * 180/math.pi

    local d1 = L_FEMUR^2 - im^2 + L_TIBIA^2
    local d2 = 2*L_TIBIA*L_FEMUR
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

-- main_IK produces the IK solution for each
-- leg joint servos by taking into consideration the initial_pos, gait offset and the
-- bodyIK values.
function main_IK()
    local ans1 = Body_FK(endpoints1[1]+GaitPosX[1], endpoints1[2]+GaitPosY[1], endpoints1[3]+GaitPosZ[1], L/2, W/2,GaitRotZ[1])
    local angles1 = Leg_IK(endpoints1[1]+ans1[1]+GaitPosX[1],endpoints1[2]+ans1[2]+GaitPosY[1], endpoints1[3]+ans1[3]+GaitPosZ[1])
    angles1 = {-45 + angles1[1],angles1[2],angles1[3]}

    local ans2 = Body_FK(endpoints2[1]+GaitPosX[2], endpoints2[2]+GaitPosY[2], endpoints2[3]+GaitPosZ[2], L/2, -W/2,GaitRotZ[2])
    local angles2 = Leg_IK(endpoints2[1]+ans2[1]+GaitPosX[2],endpoints2[2]+ans2[2]+GaitPosY[2], endpoints2[3]+ans2[3]+GaitPosZ[2])
    angles2 = {-135 + angles2[1],angles2[2],angles2[3]}

    local ans3 = Body_FK(endpoints3[1]+GaitPosX[3], endpoints3[2]+GaitPosY[3], endpoints3[3]+GaitPosZ[3], -L/2, -W/2,GaitRotZ[3])
    local angles3 = Leg_IK(endpoints3[1]+ans3[1]+GaitPosX[3],endpoints3[2]+ans3[2]+GaitPosY[3], endpoints3[3]+ans3[3]+GaitPosZ[3])
    angles3 = {135 + angles3[1],angles3[2],angles3[3]}

    local ans4 = Body_FK(endpoints4[1]+GaitPosX[4], endpoints4[2]+GaitPosY[4], endpoints4[3]+GaitPosZ[4], -L/2, W/2,GaitRotZ[4])
    local angles4 = Leg_IK(endpoints4[1]+ans4[1]+GaitPosX[4],endpoints4[2]+ans4[2]+GaitPosY[4], endpoints4[3]+ans4[3]+GaitPosZ[4])
    angles4 = {45 + angles4[1],angles4[2],angles4[3]}
    Gaitselect()
    current = {angles1[1],angles1[2],angles1[3],angles2[1],angles2[2],angles2[3],angles3[1],angles3[2],angles3[3],angles4[1],angles4[2],angles4[3]}
  
    if servo_estimate(start_time,current,last_angle) then
        start_time = millis()
        Sequence_Gen()
        last_angle = current
    end

    return angles1,angles4,angles3,angles2
end

local rest_angles = { 45, -90, 40,
                -45, -90, 40,
                45, -90, 40,
                -45, -90, 40}
local servo_direction = { 1,1,1,
                        -1,-1,-1,
                        -1,-1,1,
                        1,1,-1} 

function update()
    X_speed = vehicle:get_control_output(throttle) * max_step_factor
    Yaw_speed = vehicle:get_control_output(yaw) * max_yaw_factor

    bodyRotX = -(vehicle:get_control_output(roll) * 10)
    bodyRotY = -(vehicle:get_control_output(pitch) * 10)

    if arming:is_armed() then
        FR_angles ,  BL_angles, BR_angles, FL_angles = main_IK()

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

gcs:send_text(0, "quadruped simulation")
return update()
