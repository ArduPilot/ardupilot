local SERVO_FUNCTION1 = 94
local SERVO_FUNCTION2 = 95
local SERVO_FUNCTION3 = 96
local SERVO_FUNCTION4 = 97


local PERIOD = 0.5

function normalize(rc_pwm)
    if rc_pwm < 1515 then
        local scaled_pwm = (rc_pwm  - 1102) /412
        scaled_pwm = scaled_pwm - 1
        return scaled_pwm
    else
        local scaled_pwm = (rc_pwm - 1515) /412
        return scaled_pwm
    end
end

function scale(value)
    OldRange = (0.5 - (-0.5))  
    NewRange = (1927 - 1102)  
    NewValue = (((value - (-0.5)) * NewRange) / OldRange) + 1102


    return NewValue
end

function update() -- this is the loop which periodically runs
    pwm1 = rc:get_pwm(1)
    pwm2 = rc:get_pwm(2)
    pwm3 = rc:get_pwm(3)
    pwm4 = rc:get_pwm(4)

    alt = normalize(pwm3)
    pitch = normalize(pwm4)
    yaw = normalize(pwm1)
    thrust = normalize(pwm2)

    mot3 = (pitch + alt) *0.5
    mot4 = (-pitch + alt) *0.5
    -- mot2 = (-alt + yaw) *0.5
    -- mot3 = (alt + yaw) *0.5

    mot1 = (yaw - thrust) *0.5
    mot2 = (-yaw - thrust) *0.5
    -- mot1 = (-thrust + pitch) *0.5
    -- mot4 = (thrust + pitch) *0.5

    
    mot3_pwm = math.floor(scale(mot3))
    mot4_pwm = math.floor(scale(mot4))

    mot1_pwm = math.floor(scale(mot1))
    mot2_pwm = math.floor(scale(mot2))

   
    -- local t = 0.001 * millis():tofloat()
    -- local pi = 3.1415
    -- local output = math.cos(pi * t * PERIOD * 2.0)
    -- local pwm = math.floor(1500 + 500 * output)
    

    SRV_Channels:set_output_pwm(SERVO_FUNCTION1, mot1_pwm)
    SRV_Channels:set_output_pwm(SERVO_FUNCTION2, mot3_pwm)
    SRV_Channels:set_output_pwm(SERVO_FUNCTION3, mot4_pwm)
    SRV_Channels:set_output_pwm(SERVO_FUNCTION4, mot2_pwm)
  
  --return update, 20 -- reschedules the loop at 50Hz
end


--return update() -- run immediately before starting to reschedule

function protected_wrapper()
    local success, err = pcall(update)
    if not success then
       gcs:send_text(0, "Internal Error: " .. err)
       -- when we fault we run the update function again after 1s, slowing it
       -- down a bit so we don't flood the console with errors
       return protected_wrapper, 1000
    end
    return protected_wrapper, 200
  end

  return protected_wrapper()