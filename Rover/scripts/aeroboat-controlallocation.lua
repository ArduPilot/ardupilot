

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

local steering = 0
local throttle = 0

local trim3 = 1500
local trim1 = 1500



local function isempty(s)
  return s == nil or s == ''
end


local function CtrlAlocacaonovo(t, s)

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


local PWM0_MIN_VALUE = param:get('SERVO1_MIN')
local PWM1_MIN_VALUE = param:get('SERVO2_MIN')
local PWM2_MIN_VALUE = param:get('SERVO3_MIN')
local PWM3_MIN_VALUE = param:get('SERVO4_MIN')
local PWM4_MIN_VALUE = param:get('SERVO5_MIN')
local PWM5_MIN_VALUE = param:get('SERVO6_MIN')


if nalocDir >= 0 then
  SRV_Channels:set_output_pwm_chan_timeout(1, 2*nalocDir+PWM1_MIN_VALUE, 300)
  SRV_Channels:set_output_pwm_chan_timeout(2, 2*nalocDir+PWM2_MIN_VALUE, 300)
  SRV_Channels:set_output_pwm_chan_timeout(4, PWM4_MIN_VALUE, 300)
end

if nalocDir < 0 then
  SRV_Channels:set_output_pwm_chan_timeout(1, PWM1_MIN_VALUE, 300)
  SRV_Channels:set_output_pwm_chan_timeout(2, PWM2_MIN_VALUE, 300)
  SRV_Channels:set_output_pwm_chan_timeout(4, PWM4_MIN_VALUE - 2*nalocDir, 300)
end

if nalocEsq >= 0 then
  SRV_Channels:set_output_pwm_chan_timeout(0, 2*nalocEsq+PWM0_MIN_VALUE, 300)
  SRV_Channels:set_output_pwm_chan_timeout(3, 2*nalocEsq+PWM3_MIN_VALUE, 300)
  SRV_Channels:set_output_pwm_chan_timeout(5, PWM5_MIN_VALUE, 300)
end

if nalocEsq < 0 then
  SRV_Channels:set_output_pwm_chan_timeout(0, PWM0_MIN_VALUE, 300)
  SRV_Channels:set_output_pwm_chan_timeout(3, PWM3_MIN_VALUE, 300)
  SRV_Channels:set_output_pwm_chan_timeout(5, PWM5_MIN_VALUE - 2*nalocEsq, 300)
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
    -- steeringlimit = param:get('SCR_USER5')
    
    trim3 = param:get('RC3_TRIM')
    trim1 = param:get('RC1_TRIM')

    --vehicle:set_mode(15)
    gcs:send_text(4, string.format("ROVER - desarmado "))


    local PWM0_MIN_VALUE = param:get('SERVO1_MIN')
    local PWM1_MIN_VALUE = param:get('SERVO2_MIN')
    local PWM2_MIN_VALUE = param:get('SERVO3_MIN')
    local PWM3_MIN_VALUE = param:get('SERVO4_MIN')
    local PWM4_MIN_VALUE = param:get('SERVO5_MIN')
    local PWM5_MIN_VALUE = param:get('SERVO6_MIN')

 
    SRV_Channels:set_output_pwm_chan_timeout(0,PWM0_MIN_VALUE,3000) 
    SRV_Channels:set_output_pwm_chan_timeout(1,PWM1_MIN_VALUE,3000)
    SRV_Channels:set_output_pwm_chan_timeout(2,PWM2_MIN_VALUE,3000) 
    SRV_Channels:set_output_pwm_chan_timeout(3,PWM3_MIN_VALUE,3000)
    SRV_Channels:set_output_pwm_chan_timeout(4,PWM4_MIN_VALUE,3000)
    SRV_Channels:set_output_pwm_chan_timeout(5,PWM5_MIN_VALUE,3000)

    return update, 2000
  end




function vehicle:get_control_output(control_output) end
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

      if (vehicle:get_mode()<10) then
        vehicle:set_mode(10)
      end

      steering = vehicle:get_control_output(CONTROL_OUTPUT_YAW)
      throttle = vehicle:get_control_output(CONTROL_OUTPUT_THROTTLE)
    
      --gcs:send_text(4, string.format("t,s = %f ; %f",  throttle, steering))
      CtrlAlocacaonovo(throttle,steering)

      return update, 200
    end

end

return update, 3000 -- run immediately before starting to reschedule
