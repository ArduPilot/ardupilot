--@param control_output integer - CONTROLE MODE 4
--| '1' # Roll
--| '2' # Pitch
--| '3' # Throttle
--| '4' # Yaw
--| '5' # Lateral
--| '6' # MainSail
--| '7' # WingSail
--| '8' # Walking_Height
--@return number|nil
package.path = package.path .. ';./scripts/modules/?.lua'
local PID = require("pid")
local funcs = require("functions")

CONTROL_OUTPUT_THROTTLE = 3
CONTROL_OUTPUT_YAW = 4
TRIM3 = 1500
TRIM1 = 1500
SYSTEM_STARTED = Parameter()
local desired_yaw = -1.0


local steering_pid = PID:new(0.25, 0.001, 0.01, 0.8, -0.8, 0.8, -0.8)  -- Configure os ganhos como necessários


-- Severity for logging in GCS
MAV_SEVERITY = {EMERGENCY=0, ALERT=1, CRITICAL=2, ERROR=3, WARNING=4, NOTICE=5, INFO=6, DEBUG=7}

--[[ 
Control Allocation Function 
It basically calculates the norm of the desired control vector (t,s) 
and then splits it into the right and left side
--]]
local function new_control_allocation(t, s)

  -- Calculating the norm of each component on the desired control vector (t,s)
  local aloc = 400

  local hip = math.sqrt(t*t + s*s) + 0.0001

  local nTa = aloc * t / hip
  local nSa = aloc * s / hip

  local T = math.abs(nTa / (math.abs(nTa) + math.abs(nSa) + 0.0001))
  local S = math.abs(nSa / (math.abs(nTa) + math.abs(nSa) + 0.0001))

  -- Splitting on the right and left side
  local nft = t * T * aloc
  local nfs = s * S * aloc

  local naloc_right = math.floor(nft + nfs)
  local naloc_left = math.floor(nft - nfs)

  -- Getting the trim values for PWM outputs
  local pwm0_trim_value = tonumber(param:get('SERVO1_TRIM')) or 0
  local pwm1_trim_value = tonumber(param:get('SERVO2_TRIM')) or 0
  local pwm2_trim_value = tonumber(param:get('SERVO3_TRIM')) or 0
  local pwm3_trim_value = tonumber(param:get('SERVO4_TRIM')) or 0
  local pwm4_trim_value = tonumber(param:get('SERVO5_TRIM')) or 0
  local pwm5_trim_value = tonumber(param:get('SERVO6_TRIM')) or 0

  -- Setting the PWM outputs based on the control allocation directions
  if naloc_right >= 0 then
    SRV_Channels:set_output_pwm_chan_timeout(1, 2*naloc_right + pwm1_trim_value, 300)
    SRV_Channels:set_output_pwm_chan_timeout(2, 2*naloc_right + pwm2_trim_value, 300)
    SRV_Channels:set_output_pwm_chan_timeout(4, pwm4_trim_value, 300)
  end

  if naloc_right < 0 then
    SRV_Channels:set_output_pwm_chan_timeout(1, pwm1_trim_value, 300)
    SRV_Channels:set_output_pwm_chan_timeout(2, pwm2_trim_value, 300)
    SRV_Channels:set_output_pwm_chan_timeout(4, pwm4_trim_value - 2*naloc_right, 300)
  end

  if naloc_left >= 0 then
    SRV_Channels:set_output_pwm_chan_timeout(0, 2*naloc_left + pwm0_trim_value, 300)
    SRV_Channels:set_output_pwm_chan_timeout(3, 2*naloc_left + pwm3_trim_value, 300)
    SRV_Channels:set_output_pwm_chan_timeout(5, pwm5_trim_value, 300)
  end

  if naloc_left < 0 then
    SRV_Channels:set_output_pwm_chan_timeout(0, pwm0_trim_value, 300)
    SRV_Channels:set_output_pwm_chan_timeout(3, pwm3_trim_value, 300)
    SRV_Channels:set_output_pwm_chan_timeout(5, pwm5_trim_value - 2*naloc_left, 300)
  end
  
end -- new_control_allocation function


--[[ 
Main update function 
It checks if the vehicle is a boat, if it is armed, and then 
it gets the control values from the RC or internal control of the vehicle and passes 
them to the control allocation function
--]]
local function update()


  -- Check if the vehicle is a boat
  local vehicle_type = param:get('SCR_USER5')
  if not (vehicle_type == 1) then
    gcs:send_text(MAV_SEVERITY.INFO, string.format("Not a boat, exiting this lua script."))
    return
  end

  -- Check if the system was already started before running this function
  if not SYSTEM_STARTED:init('BATT_SOC_SYSSTART') then
    gcs:send_text(MAV_SEVERITY.CRITICAL, 'BATT_SOC_SYSSTART is not set yet, not running control allocation ...')
    return update, 1000
  end

  -- Check if armed to begin control allocation safely
  if not arming:is_armed() then
    gcs:send_text(MAV_SEVERITY.INFO, string.format("BOAT - disarmed, waiting for arming."))

    desired_yaw = -1.0

    -- Get the trim values for the RC channels
    TRIM3 = param:get('RC3_TRIM')
    TRIM1 = param:get('RC1_TRIM')

    -- Send the PWM trim (neutral) values to the servo outputs
    local pwm0_trim_value = tonumber(param:get('SERVO1_TRIM')) or 1500
    local pwm1_trim_value = tonumber(param:get('SERVO2_TRIM')) or 1500
    local pwm2_trim_value = tonumber(param:get('SERVO3_TRIM')) or 1500
    local pwm3_trim_value = tonumber(param:get('SERVO4_TRIM')) or 1500
    local pwm4_trim_value = tonumber(param:get('SERVO5_TRIM')) or 1500
    local pwm5_trim_value = tonumber(param:get('SERVO6_TRIM')) or 1500
    SRV_Channels:set_output_pwm_chan_timeout(0, pwm0_trim_value, 3000)
    SRV_Channels:set_output_pwm_chan_timeout(1, pwm1_trim_value, 3000)
    SRV_Channels:set_output_pwm_chan_timeout(2, pwm2_trim_value, 3000)
    SRV_Channels:set_output_pwm_chan_timeout(3, pwm3_trim_value, 3000)
    SRV_Channels:set_output_pwm_chan_timeout(4, pwm4_trim_value, 3000)
    SRV_Channels:set_output_pwm_chan_timeout(5, pwm5_trim_value, 3000)

    return update, 2000
  end

  -- Create the input control variables
  local steering = 0 --vehicle:get_control_output(CONTROL_OUTPUT_YAW)
  local throttle = 0 --vehicle:get_control_output(CONTROL_OUTPUT_THROTTLE)
  local rc3_pwm = 0
  local rc1_pwm = 0

   

  if vehicle:get_mode() == 0 then -- Manual mode
    -- Get the trim values for the RC channels
    TRIM3 = param:get('RC3_TRIM')
    TRIM1 = param:get('RC1_TRIM')
    -- Get the RC PWM values
    rc3_pwm = tonumber(rc:get_pwm(3)) or 1500
    rc1_pwm = tonumber(rc:get_pwm(1)) or 1500
    -- Transform the RC PWM values to the desired control values and send to allocation
    throttle = (TRIM3 - rc3_pwm) / 450
    steering = (rc1_pwm - TRIM1) / 450
    new_control_allocation(throttle, steering)

    return update, 200


  -- ROVER_MODE_STEERING
  elseif vehicle:get_mode()==3 then

    if desired_yaw == -1 then
      desired_yaw = funcs:map_to_360(funcs:to_degrees(ahrs:get_yaw()))
    end

    TRIM3 = param:get('RC3_TRIM')
    rc3_pwm = tonumber(rc:get_pwm(3)) or 1500
    throttle = (TRIM3 - rc3_pwm) / 450

    TRIM1 = param:get('RC1_TRIM')
    rc1_pwm = tonumber(rc:get_pwm(1)) or 1500
    local addsteering = (rc1_pwm - TRIM1) / 450
    
    local k1 = param:get('SCR_USER4')

    desired_yaw = funcs:map_to_360(desired_yaw + k1*addsteering)
    local vh_yaw = funcs:map_to_360(To_degrees(ahrs:get_yaw()))
    local steering_error = funcs:map_error(desired_yaw - vh_yaw)

    steering_pid.P = param:get('SCR_USER3')
    steering_pid.I = param:get('SCR_USER2')
    steering_pid.D = param:get('SCR_USER1')
    local mysteering = steering_pid:compute(0,steering_error)

    new_control_allocation(throttle, mysteering)

    return update, 200

  elseif vehicle:get_mode()==4 then

    if desired_yaw == -1 then
      desired_yaw = funcs:map_to_360(To_degrees(ahrs:get_yaw()))
    end

    local mysteering, vh_yaw, steering_error = 0,0,0

    TRIM3 = param:get('RC3_TRIM')
    rc3_pwm = tonumber(rc:get_pwm(3)) or 1500
    throttle = (TRIM3 - rc3_pwm) / 450

    TRIM1 = param:get('RC1_TRIM')
    rc1_pwm = tonumber(rc:get_pwm(1)) or 1500
    steering = (rc1_pwm - TRIM1) / 450

    if math.abs(steering) > 0.05 then

      desired_yaw = funcs:map_to_360(To_degrees(ahrs:get_yaw()))
      new_control_allocation(throttle, steering)

    else

      vh_yaw = funcs:map_to_360(To_degrees(ahrs:get_yaw()))
      steering_error = funcs:map_error(desired_yaw - vh_yaw)
      mysteering = steering_pid:compute(0,steering_error)
      new_control_allocation(throttle, mysteering)

      
    end

    return update, 200
  

  else -- Autonomous mode
    -- Set the mode to 10 (Auto) if not already
    -- if vehicle:get_mode() < 10 then 
    --   vehicle:set_mode(10)
    -- end

    -- Get the control outputs from the vehicle and pass into control allocation
    steering = tonumber(vehicle:get_control_output(CONTROL_OUTPUT_YAW)) or 0
    throttle = tonumber(vehicle:get_control_output(CONTROL_OUTPUT_THROTTLE)) or 0
    --gcs:send_text(4, string.format("t,s = %f ; %f",  throttle, steering))
    new_control_allocation(throttle,steering)

    return update, 200
  end

end -- update function

return update, 3000 -- run immediately before starting to reschedule
