-- move a servo in a sinisoidal fashion
local PERIOD = 0.5
local pi = 3.1415
local THROTTLE_LEFT_id = 73
local THROTTLE_RIGHT_id = 74
local THROTTLE_BLEFT_id = 35
local THROTTLE_BRIGHT_id = 36
local THROTTLE_LEFT_COMMAND = SRV_Channels:find_channel(73)
local THROTTLE_RIGHT_COMMAND = SRV_Channels:find_channel(74)
local THROTTLE_BackLEFT_COMMAND = SRV_Channels:find_channel(THROTTLE_BLEFT_id)
local THROTTLE_BackRIGHT_COMMAND = SRV_Channels:find_channel(THROTTLE_BRIGHT_id)
local CONTROL_OUTPUT_THROTTLE = 3
local CONTROL_OUTPUT_YAW = 4
local motor1 = SRV_Channels:find_channel(94)
local file_name = "aAHRS_DATA.csv"
local file
local steering = 0
local throttle = 0
local THROTTLE_LEFT 
local THROTTLE_RIGHT 
local time = 0
local firsttime = 0
local yaw
local distance = 0
local bearing = 0
local x = 0
local y = 0
local myGPS
local gyroratings
local velocidades
local arquivocriado = 0
local fatorThrottle = 400
local fatoryaw = 100
local steeringlimit = 0.5
local trim3 = 1500
local trim1 = 1500

local homelocation
-- local teste = io.open("/home/grin/ardupilot/Rover/adados.txt",w)
-- teste:write("aqui/n")
-- teste:write("aqui/n")
-- teste:close()

local function isempty(s)
  return s == nil or s == ''
end


local function CtrlAlocacaonovo(t, s)

  local aloc = 400

  -- if math.abs(t) <= 0.035 then
  --   t = 0
  -- end

  -- if math.abs(s) <= 0.035 then
  --   s = 0
  -- end


  local hip = math.sqrt(t*t + s*s) + 0.0001

  local nTa = aloc * t / hip
  local nSa = aloc * s / hip

  T = math.abs(nTa / (math.abs(nTa) + math.abs(nSa) + 0.0001))
  S = math.abs(nSa / (math.abs(nTa) + math.abs(nSa) + 0.0001))

  local nft = t * T * aloc
  local nfs = s * S * aloc

  local nalocDir = math.floor(nft + nfs)
  local nalocEsq = math.floor(nft - nfs)

  -- if math.abs(nalocDir) <= 15 then
  --   nalocDir = 0
  -- end

  -- if math.abs(nalocEsq) <= 15 then
  --   nalocEsq = 0
  -- end

 -- gcs:send_text(6, string.format("t,s = %f ; %f ",  t, s))

  --criar variavel para substituir 1100
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

  if not arming:is_armed() then
    fatorThrottle = param:get('SCR_USER3')
    fatoryaw = param:get('SCR_USER4')
    steeringlimit = param:get('SCR_USER5')
    
    trim3 = param:get('RC3_TRIM')
    trim1 = param:get('RC1_TRIM')

    local m1 = SRV_Channels:get_output_pwm(73)

    --vehicle:set_mode(15)
    gcs:send_text(4, string.format("lua desarmadao t,s = %d",  m1))


    param:set('SERVO1_FUNCTION',74)
    param:set('SERVO4_FUNCTION',74)
    param:set('SERVO5_FUNCTION',35)
    param:set('SERVO2_FUNCTION',73)
    param:set('SERVO3_FUNCTION',73)
    param:set('SERVO6_FUNCTION',36)
    param:set('CRUISE_SPEED',2.0)

    local setmin = 1000
    local settrim = 1000

    local PWM0_MIN_VALUE = param:get('SERVO1_MIN')
    local PWM1_MIN_VALUE = param:get('SERVO2_MIN')
    local PWM2_MIN_VALUE = param:get('SERVO3_MIN')
    local PWM3_MIN_VALUE = param:get('SERVO4_MIN')
    local PWM4_MIN_VALUE = param:get('SERVO5_MIN')
    local PWM5_MIN_VALUE = param:get('SERVO6_MIN')

    param:set('SERVO1_MIN',setmin)
    param:set('SERVO2_MIN',setmin)
    param:set('SERVO3_MIN',setmin)
    param:set('SERVO4_MIN',setmin)
    param:set('SERVO5_MIN',setmin)
    param:set('SERVO6_MIN',setmin)

    param:set('SERVO1_TRIM',settrim)
    param:set('SERVO2_TRIM',settrim)
    param:set('SERVO3_TRIM',settrim)
    param:set('SERVO4_TRIM',settrim)
    param:set('SERVO5_TRIM',settrim)
    param:set('SERVO6_TRIM',settrim)


    local shift = 100

    SRV_Channels:set_output_pwm_chan_timeout(0,PWM0_MIN_VALUE,3000) 
    SRV_Channels:set_output_pwm_chan_timeout(1,PWM1_MIN_VALUE,3000)
    SRV_Channels:set_output_pwm_chan_timeout(2,PWM2_MIN_VALUE,3000) 
    SRV_Channels:set_output_pwm_chan_timeout(3,PWM3_MIN_VALUE,3000)
    SRV_Channels:set_output_pwm_chan_timeout(4,PWM4_MIN_VALUE,3000)
    SRV_Channels:set_output_pwm_chan_timeout(5,PWM5_MIN_VALUE,3000)

    return update, 2000
  end

 -- if param:get('SCR_USER1')==0 and param:get('SCR_USER2') == 0 then
    steering = vehicle:get_control_output(CONTROL_OUTPUT_YAW)
    throttle = vehicle:get_control_output(CONTROL_OUTPUT_THROTTLE)

    local ch3pwm = 0
    local ch1pwm = 0

    if vehicle:get_mode()== 0 then

      trim3 = param:get('RC3_TRIM')
      trim1 = param:get('RC1_TRIM')

      ch3pwm = rc:get_pwm(3)
      ch1pwm = rc:get_pwm(1)

      throttle = (trim3 - ch3pwm) / 450
      steering = (ch1pwm - trim1) / 450

      CtrlAlocacaonovo(throttle,steering)
      return update, 200

    else

      steering = vehicle:get_control_output(CONTROL_OUTPUT_YAW)
      throttle = vehicle:get_control_output(CONTROL_OUTPUT_THROTTLE)


      gcs:send_text(4, string.format("t,s = %f ; %f",  throttle, steering))
      CtrlAlocacaonovo(throttle,steering)

      return update, 200
    end
  

  return update, 200 -- reschedules the loop at 5Hz
end

return update, 3000 -- run immediately before starting to reschedule
