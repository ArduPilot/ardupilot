

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
local CONTROL_OUTPUT_THROTTLE = 3
local CONTROL_OUTPUT_YAW = 4

local THROTTLE_LEFT_id = 73
local THROTTLE_RIGHT_id = 74
local THROTTLE_LEFT_COMMAND = SRV_Channels:find_channel(73)
local THROTTLE_RIGHT_COMMAND = SRV_Channels:find_channel(74)

local steering = 0
local throttle = 0

local trim3 
local trim1 



local function isempty(s)
  return s == nil or s == ''
end

local function ControleAlocacao()

  --local 
  steering = vehicle:get_control_output(CONTROL_OUTPUT_YAW)
  throttle = vehicle:get_control_output(CONTROL_OUTPUT_THROTTLE)

  if arming:is_armed() and throttle > 0 then

    --gcs:send_text(4, string.format("%f - %f ", steering, throttle))

    local wl = math.floor(400.0*throttle + 100.0*steering)
    local wr = math.floor(400.0*throttle - 100.0*steering)

    local rossteering = math.floor(1000.0 * steering + 1000)
    local rosthrottle = math.floor(1000.0 * throttle)

    SRV_Channels:set_output_pwm_chan_timeout(THROTTLE_LEFT_COMMAND, wl+1500,300) 
    SRV_Channels:set_output_pwm_chan_timeout(THROTTLE_RIGHT_COMMAND,wr+1500,300)
    SRV_Channels:set_output_pwm_chan_timeout(6,rossteering,300) 
    SRV_Channels:set_output_pwm_chan_timeout(7,rosthrottle,300)



  end

end


local function CtrlAlocacaonovo(t, s)

  local aloc = 450

  
  local hip = math.sqrt(t*t + s*s) + 0.0001

  local nTa = aloc * t / hip
  local nSa = aloc * s / hip

  T = math.abs(nTa / (math.abs(nTa) + math.abs(nSa) + 0.0001))
  S = math.abs(nSa / (math.abs(nTa) + math.abs(nSa) + 0.0001))

  local nft = t * T * aloc
  local nfs = s * S * aloc

  local nalocDir = math.floor(nft + nfs)
  local nalocEsq = math.floor(nft - nfs)


  local PWM0_TRIM_VALUE = param:get('SERVO1_TRIM')
  local PWM2_TRIM_VALUE = param:get('SERVO3_TRIM')


  SRV_Channels:set_output_pwm_chan_timeout(2, nalocDir+PWM2_TRIM_VALUE, 300)
  SRV_Channels:set_output_pwm_chan_timeout(0, nalocEsq+PWM0_TRIM_VALUE, 300)


  
end


function update() -- this is the loop which periodically runs

  local tipoveiculo = param:get('SCR_USER5')

  if not (tipoveiculo==2) then
    gcs:send_text(4, string.format("nao e ROVER saindo do lua"))
    return
  end

  if not arming:is_armed() then
    
    --vehicle:set_mode(15)
    gcs:send_text(4, string.format("ROVER - desarmado "))


    local PWM0_TRIM_VALUE = param:get('SERVO1_TRIM')
    local PWM1_TRIM_VALUE = param:get('SERVO2_TRIM')
    local PWM2_TRIM_VALUE = param:get('SERVO3_TRIM')
    local PWM3_TRIM_VALUE = param:get('SERVO4_TRIM')

    -- SRV_Channels:set_output_pwm_chan_timeout(THROTTLE_LEFT_COMMAND, 1500,300) 
    -- SRV_Channels:set_output_pwm_chan_timeout(THROTTLE_RIGHT_COMMAND,1500,300)

 
    SRV_Channels:set_output_pwm_chan_timeout(0,PWM0_TRIM_VALUE,3000) 
    SRV_Channels:set_output_pwm_chan_timeout(1,PWM1_TRIM_VALUE,3000)
    SRV_Channels:set_output_pwm_chan_timeout(2,PWM2_TRIM_VALUE,3000) 
    SRV_Channels:set_output_pwm_chan_timeout(3,PWM3_TRIM_VALUE,3000)


    return update, 2000
  end


    steering = vehicle:get_control_output(CONTROL_OUTPUT_YAW)
    throttle = vehicle:get_control_output(CONTROL_OUTPUT_THROTTLE)

    local rc3_pwm = 0
    local rc1_pwm = 0

    if vehicle:get_mode()== 0 then

      trim3 = param:get('RC3_TRIM')
      trim1 = param:get('RC1_TRIM')

      rc3_pwm = rc:get_pwm(3)
      rc1_pwm = rc:get_pwm(1)

      throttle = (trim3 - rc3_pwm) / 450
      steering = (rc1_pwm - trim1) / 450

      CtrlAlocacaonovo(throttle,steering)
      return update, 200

    else

      if vehicle:get_mode()< 10 then
        vechicle:set_mode(10)
      end

      steering = vehicle:get_control_output(CONTROL_OUTPUT_YAW)
      throttle = vehicle:get_control_output(CONTROL_OUTPUT_THROTTLE)
    
      --gcs:send_text(4, string.format("t,s = %f ; %f",  throttle, steering))
      --CtrlAlocacaonovo(throttle,steering)
      ControleAlocacao()

      return update, 200
    end

end

return update, 3000 -- run immediately before starting to reschedule
