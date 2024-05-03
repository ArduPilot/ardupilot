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
CONTROL_OUTPUT_THROTTLE = 3
CONTROL_OUTPUT_YAW = 4
STEERING = 0
THROTTLE = 0
TRIM3 = 1500
TRIM1 = 1500

local function isempty(s)
  return s == nil or s == ''
end

local function new_control_allocation(t, s)

  local aloc = 400
  
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
  local PWM1_TRIM_VALUE = param:get('SERVO2_TRIM')
  local PWM2_TRIM_VALUE = param:get('SERVO3_TRIM')
  local PWM3_TRIM_VALUE = param:get('SERVO4_TRIM')
  local PWM4_TRIM_VALUE = param:get('SERVO5_TRIM')
  local PWM5_TRIM_VALUE = param:get('SERVO6_TRIM')

  if nalocDir >= 0 then
    SRV_Channels:set_output_pwm_chan_timeout(1, 2*nalocDir+PWM1_TRIM_VALUE, 300)
    SRV_Channels:set_output_pwm_chan_timeout(2, 2*nalocDir+PWM2_TRIM_VALUE, 300)
    SRV_Channels:set_output_pwm_chan_timeout(4, PWM4_TRIM_VALUE, 300)
  end

  if nalocDir < 0 then
    SRV_Channels:set_output_pwm_chan_timeout(1, PWM1_TRIM_VALUE, 300)
    SRV_Channels:set_output_pwm_chan_timeout(2, PWM2_TRIM_VALUE, 300)
    SRV_Channels:set_output_pwm_chan_timeout(4, PWM4_TRIM_VALUE - 2*nalocDir, 300)
  end

  if nalocEsq >= 0 then
    SRV_Channels:set_output_pwm_chan_timeout(0, 2*nalocEsq+PWM0_TRIM_VALUE, 300)
    SRV_Channels:set_output_pwm_chan_timeout(3, 2*nalocEsq+PWM3_TRIM_VALUE, 300)
    SRV_Channels:set_output_pwm_chan_timeout(5, PWM5_TRIM_VALUE, 300)
  end

  if nalocEsq < 0 then
    SRV_Channels:set_output_pwm_chan_timeout(0, PWM0_TRIM_VALUE, 300)
    SRV_Channels:set_output_pwm_chan_timeout(3, PWM3_TRIM_VALUE, 300)
    SRV_Channels:set_output_pwm_chan_timeout(5, PWM5_TRIM_VALUE - 2*nalocEsq, 300)
  end
  
end

function update() -- this is the loop which periodically runs

  local tipoveiculo = param:get('SCR_USER5')

  if not (tipoveiculo==1) then
    gcs:send_text(4, string.format("nao e BARCO saindo do lua"))
    return
  end

  if not arming:is_armed() then
    -- fatorThrottle = param:get('SCR_USER3')
    -- fatoryaw = param:get('SCR_USER4')
    -- STEERINGlimit = param:get('SCR_USER5')
    
    TRIM3 = param:get('RC3_TRIM')
    TRIM1 = param:get('RC1_TRIM')

    --vehicle:set_mode(15)
    gcs:send_text(4, string.format("BOAT - desarmado "))

    local PWM0_TRIM_VALUE = param:get('SERVO1_TRIM')
    local PWM1_TRIM_VALUE = param:get('SERVO2_TRIM')
    local PWM2_TRIM_VALUE = param:get('SERVO3_TRIM')
    local PWM3_TRIM_VALUE = param:get('SERVO4_TRIM')
    local PWM4_TRIM_VALUE = param:get('SERVO5_TRIM')
    local PWM5_TRIM_VALUE = param:get('SERVO6_TRIM')

    SRV_Channels:set_output_pwm_chan_timeout(0,PWM0_TRIM_VALUE,3000) 
    SRV_Channels:set_output_pwm_chan_timeout(1,PWM1_TRIM_VALUE,3000)
    SRV_Channels:set_output_pwm_chan_timeout(2,PWM2_TRIM_VALUE,3000) 
    SRV_Channels:set_output_pwm_chan_timeout(3,PWM3_TRIM_VALUE,3000)
    SRV_Channels:set_output_pwm_chan_timeout(4,PWM4_TRIM_VALUE,3000)
    SRV_Channels:set_output_pwm_chan_timeout(5,PWM5_TRIM_VALUE,3000)

    return update, 2000
  end

  STEERING = vehicle:get_control_output(CONTROL_OUTPUT_YAW)
  THROTTLE = vehicle:get_control_output(CONTROL_OUTPUT_THROTTLE)

  local rc3_pwm = 0
  local rc1_pwm = 0

  if vehicle:get_mode() == 0 then
    TRIM3 = param:get('RC3_TRIM')
    TRIM1 = param:get('RC1_TRIM')

    rc3_pwm = rc:get_pwm(3)
    rc1_pwm = rc:get_pwm(1)

    THROTTLE = (TRIM3 - rc3_pwm) / 450
    STEERING = (rc1_pwm - TRIM1) / 450
    new_control_allocation(THROTTLE,STEERING)

    return update, 200
  else
    if vehicle:get_mode()< 10 then
      vehicle:set_mode(10)
    end

    if temos obstaculo then
      pega controles aqui do algoritmo de desvio
        new_control_allocation(THROTTLE,STEERING)
    end
    STEERING = vehicle:get_control_output(CONTROL_OUTPUT_YAW)
    THROTTLE = vehicle:get_control_output(CONTROL_OUTPUT_THROTTLE)  
    --gcs:send_text(4, string.format("t,s = %f ; %f",  THROTTLE, STEERING))
    new_control_allocation(THROTTLE,STEERING)

    return update, 200    
  end

end

return update, 3000 -- run immediately before starting to reschedule
