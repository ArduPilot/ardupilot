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

local homelocation
-- local teste = io.open("/home/grin/ardupilot/Rover/adados.txt",w)
-- teste:write("aqui/n")
-- teste:write("aqui/n")
-- teste:close()

local function isempty(s)
  return s == nil or s == ''
end

local function CrirArquivo()

  myGPS = gps:primary_sensor()
  time = gps:time_week_ms(myGPS):toint()

  --gcs:send_text(2, string.format("arquivo = data_%d.csv", time))

  if arquivocriado == 0 and  time>0 then

    file_name = string.format("data_%d.csv", time)

 
    file = io.open(file_name, "w")
    if not file then
      error("Could not make file")
    end
  
    file:write('time,TL,TR,Tb1,Tb2,x,y,bearing,yaw,vel,gyroratings\n')
    file:flush()

    arquivocriado = 1

    
  end
  
end

local function GeradadosMissao()


  gyroratings = ahrs:get_gyro():z()
  velocidades = ahrs:groundspeed_vector()
  local TL = SRV_Channels:get_output_pwm(THROTTLE_LEFT_id) - 1500
  local TR = SRV_Channels:get_output_pwm(THROTTLE_RIGHT_id) - 1500
  local Tb1 = SRV_Channels:get_output_pwm(THROTTLE_BLEFT_id)- 1500
  local Tb2 = SRV_Channels:get_output_pwm(THROTTLE_BRIGHT_id)- 1500
  local current_pos = ahrs:get_position() 
  local home = ahrs:get_home()
  local yaw = ahrs:get_yaw()

  local ieT = not isempty(throttle)
  local ieS = not isempty(steering)
  local ieG = not isempty(gyroratings)
  local ieV = not isempty(velocidades)
  local direcao = 1.0

  if ieT and ieS and ieG and ieV and current_pos and home then


    distance = current_pos:get_distance_NE(home)
    bearing = current_pos:get_bearing(home)
    myGPS = gps:primary_sensor()
    time = gps:time_week_ms(myGPS):toint()
    local vel = velocidades:length()

    if (firsttime == 0) then

      firsttime = gps:time_week_ms(myGPS):toint()

      
    end



    --gcs:send_text(4, string.format("LUA: w = %2.2f ,v = %2.2f, B = %2.2f, y = %2.2f", 
    --gyroratings, vel,bearing, yaw ))


    if distance and not isempty(distance) then
      x = distance:x()
      y = distance:y()
      --gcs:send_text(4, string.format("%f - %f - %f", x, y, bearing))

      if param:get('SCR_USER2')== 1 then
        if not file then
          gcs:send_text(2, "Could not open file")
          error("Could not open file")
        end


        --time,TL,TR,Tb1,Tb2,x,y,bearing,yaw,vel,gyroratings
        file:write(string.format("%d,%d,%d,%d,%d,%f,%f,%f,%f,%f,%f\n",
        time-firsttime,TL ,TR, Tb1, Tb2, x, y ,bearing, yaw, vel, gyroratings ))


        --file:write(string.format("%d,%d,%d,%f,%f,%f,%f,%f,%f\n",
        --time,TL-1500 ,TR-1500, x, y ,bearing, yaw, direcao, vel, gyroratings ))

        file:flush()
      end

    end

  end
end

local function AjustaMotores(m1, m2)

    local surplus = 0

  if m1 < 0 then
    surplus = 0 - m1
    m2 = m2 + surplus
    m1 = 0
  end

  if m1 > 500 then
    surplus = m1 - 500
    m2 = m2 - surplus
    m1 = 500
  end

  return m1, m2
  
end

local function anegD(msteering, mthrottle)

  local ret = 0
  if msteering > 0 then
    ret = math.floor(mthrottle * math.abs(steering))
  end

  return ret

end

local function anegE(msteering, mthrottle)

  local ret = 0
  if msteering < 0 then
    ret = math.floor(mthrottle * math.abs(steering))
  end

  return ret

end


local function CtrlAlocacaonovo(t, s)

  local aloc = 450


  local hip = math.sqrt(t*t + s*s) + 0.0001

  local nTa = aloc * t / hip
  local nSa = aloc * s / hip

  T = math.abs(nTa / (math.abs(nTa) + math.abs(nSa)))
  S = math.abs(nSa / (math.abs(nTa) + math.abs(nSa)))

  local nft = t * T * aloc
  local nfs = s * S * aloc

  local nalocDir = math.floor(nft + nfs)
  local nalocEsq = math.floor(nft - nfs)

  --gcs:send_text(6, string.format("t,s = %d ; %d ",  nalocDir, nalocEsq))

  if nalocDir >= 0 then
    SRV_Channels:set_output_pwm_chan_timeout(1,nalocDir+1500,300)
    SRV_Channels:set_output_pwm_chan_timeout(2,nalocDir+1500,300)
    SRV_Channels:set_output_pwm_chan_timeout(4, 1500 , 300)
  end
  if nalocDir < 0 then
    SRV_Channels:set_output_pwm_chan_timeout(1,1500,300)
    SRV_Channels:set_output_pwm_chan_timeout(2,1500,300)
    SRV_Channels:set_output_pwm_chan_timeout(4, 1500 - nalocDir, 300)
  end

  if nalocEsq >= 0 then
    SRV_Channels:set_output_pwm_chan_timeout(0, nalocEsq+1500,300)
    SRV_Channels:set_output_pwm_chan_timeout(3, nalocEsq+1500,300)
    SRV_Channels:set_output_pwm_chan_timeout(5, 1500  ,300)
  end
  if nalocEsq < 0 then
    SRV_Channels:set_output_pwm_chan_timeout(0, 1500,300)
    SRV_Channels:set_output_pwm_chan_timeout(3, 1500,300)
    SRV_Channels:set_output_pwm_chan_timeout(5, 1500 - nalocEsq ,300)

  end
  
end

local function ControleAlocacao()

  --local 
  --steering = vehicle:get_control_output(CONTROL_OUTPUT_YAW)
  --throttle = vehicle:get_control_output(CONTROL_OUTPUT_THROTTLE)


  if arming:is_armed()  then --and mission:state()==1
    --gcs:send_text(4, string.format("controle = %f ; %f ", steering, throttle))
      local m5l, m6r = 0, 0
    local stbalance = 0

    -- ajusta o a distribuicao que sera exclusiva para os propulsores frontais
    -- o balanco fica por conta dos reversos 
    -- if steering<-steeringlimit then
    --     stbalance = st + fatoryaw*steeringlimit
    --     st = -fatoryaw*steeringlimit
    -- end
    -- if steering>steeringlimit then
    --     stbalance = st - fatoryaw*steeringlimit
    --     st = fatoryaw*steeringlimit
    --   end

    -- steering e steeringlimit estao em PU
    if steering<-steeringlimit then
        stbalance =  steering + steeringlimit
        steering  = -steeringlimit
        
    elseif steering>steeringlimit then
        stbalance =  steering - steeringlimit
        steering = steeringlimit
    end
    

    -- tr e st estao em PWM
    local tr = fatorThrottle * throttle
    local st = fatoryaw*steering/2.0
      
    local rigthset = tr + st
    local leftset = tr - st

    if throttle >= 0 then

      --se houver necessidade faz o balanco de potencia
      if not(stbalance == 0) then
        if steering < 0 then
          m5l = - stbalance * fatoryaw
          leftset = leftset - stbalance* fatoryaw/2 -- sao dois motores para frente e um para tras)
        else
          m6r = stbalance * fatoryaw
          rigthset = rigthset + stbalance* fatoryaw/2
        end
      end

      rigthset, leftset = AjustaMotores(rigthset, leftset)
      leftset, rigthset = AjustaMotores(leftset, rigthset)

      rigthset = math.floor(rigthset)
      leftset = math.floor(leftset)
      m5l = math.floor(m5l)
      m6r = math.floor(m6r)


      SRV_Channels:set_output_pwm_chan_timeout(0, leftset+1500,300)
      SRV_Channels:set_output_pwm_chan_timeout(3, leftset+1500,300)
      SRV_Channels:set_output_pwm_chan_timeout(1,rigthset+1500,300)
      SRV_Channels:set_output_pwm_chan_timeout(2,rigthset+1500,300)
      SRV_Channels:set_output_pwm_chan_timeout(4, m5l+1500,300)
      SRV_Channels:set_output_pwm_chan_timeout(5, m6r+1500,300)

    else

      -- so os motores trazeiros irao funcionar.. sao a metade da potencia dos dianteiros
      rigthset = -rigthset
      leftset  =  -leftset

        --gcs:send_text(6, "throttle is lower than zero")
        --gcs:send_text(6, string.format("controle = %f ; %f ; %f ; %f", steering, throttle,rigthset, leftset))

        rigthset, leftset = AjustaMotores(rigthset, leftset)
        leftset, rigthset = AjustaMotores(leftset, rigthset)

        rigthset = math.floor(rigthset)
        leftset = math.floor(leftset)

        local mr = 0
        local ml = 0

        local mrigthset = anegD(steering,rigthset)
        local mleftset = anegE(steering,leftset)

        SRV_Channels:set_output_pwm_chan_timeout(4, rigthset  + 1500,300)
        SRV_Channels:set_output_pwm_chan_timeout(5, leftset + 1500,300)
        SRV_Channels:set_output_pwm_chan_timeout(0, 1500 + mleftset,300)
        SRV_Channels:set_output_pwm_chan_timeout(3, 1500,300)
        SRV_Channels:set_output_pwm_chan_timeout(1, 1500 ,300)
        SRV_Channels:set_output_pwm_chan_timeout(2, 1500 + mrigthset,300)
        --SRV_Channels:set_output_pwm_chan_timeout(4, leftset+1000,300)
        --SRV_Channels:set_output_pwm_chan_timeout(5, rigthset+1000,300)

    end


  end

  

end





function update() -- this is the loop which periodically runs





  if not arming:is_armed() then

    fatorThrottle = param:get('SCR_USER3')
    fatoryaw = param:get('SCR_USER4')
    steeringlimit = param:get('SCR_USER5')
    --vehicle:set_mode(15)

    gcs:send_text(4, "desarmado lua")

 

    
    param:set('SERVO1_FUNCTION',74)
    param:set('SERVO4_FUNCTION',74)
    param:set('SERVO5_FUNCTION',35)
    param:set('SERVO2_FUNCTION',73)
    param:set('SERVO3_FUNCTION',73)
    param:set('SERVO6_FUNCTION',36)
    param:set('CRUISE_SPEED',5.0)

    param:set('SERVO1_TRIM',1500)
    param:set('SERVO2_TRIM',1500)
    param:set('SERVO3_TRIM',1500)
    param:set('SERVO4_TRIM',1500)
    param:set('SERVO5_TRIM',1500)
    param:set('SERVO6_TRIM',1500)


    -- SRV_Channels:set_output_pwm_chan_timeout(THROTTLE_LEFT_COMMAND, 1000,300) 
    -- SRV_Channels:set_output_pwm_chan_timeout(THROTTLE_RIGHT_COMMAND,1000,300)
    SRV_Channels:set_output_pwm_chan_timeout(0,1500,3000) 
    SRV_Channels:set_output_pwm_chan_timeout(1,1500,3000)
    SRV_Channels:set_output_pwm_chan_timeout(2,1500,3000) 
    SRV_Channels:set_output_pwm_chan_timeout(3,1500,3000)
    SRV_Channels:set_output_pwm_chan_timeout(4,1500,3000)
    SRV_Channels:set_output_pwm_chan_timeout(5,1500,3000)
        


    -- if param:get('SCR_USER1')==1 and param:get('SCR_USER2') == 0 then
    --   CrirArquivo()
    -- end

    return update, 2000
  end


 -- if param:get('SCR_USER1')==0 and param:get('SCR_USER2') == 0 then


    steering = vehicle:get_control_output(CONTROL_OUTPUT_YAW)
    throttle = vehicle:get_control_output(CONTROL_OUTPUT_THROTTLE)

    gcs:send_text(4, string.format("s,t = %f ; %f ", steering, throttle))

    local ch3pwm = 0
    local ch4pwm = 0
 --a

    if vehicle:get_mode()== 0 then

      -- substituir pelo parametro rcmap_yaw e rcmap_throttle
      ch3pwm = rc:get_pwm(3)
      ch4pwm = rc:get_pwm(4)


      throttle = (1515 - ch3pwm) / 450
      steering = (ch4pwm - 1515) / 450

      --gcs:send_text(4, string.format("s,t = %f ; %f ", steering, throttle))

      CtrlAlocacaonovo(throttle,steering)
      return update, 200

    else
      CtrlAlocacaonovo(throttle,steering)
      --ControleAlocacao()
      return update, 200
      

    end
  


  --else

    --GeradadosMissao()

  --end


  return update, 200 -- reschedules the loop at 5Hz

end




return update, 3000 -- run immediately before starting to reschedule
