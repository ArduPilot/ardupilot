

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

local last_mission_index =  -1

local PID = {}

local throttle_output = 0
local steering_output = 0

local steering = 0
local throttle = 0

local trim3 
local trim1 



function PID:new(p_gain, i_gain, d_gain, i_max, i_min, pid_max, pid_min)
  local obj = {
      P = p_gain or 0,
      I = i_gain or 0,
      D = d_gain or 0,
      integrator = 0,
      last_error = 0,
      i_max = i_max or 0,
      i_min = i_min or 0,
      pid_max = pid_max or 1,
      pid_min = pid_min or -1
  }
  setmetatable(obj, self)
  self.__index = self
  return obj
end


function PID:compute(setpoint, pv)
  local error = setpoint - pv
  local deriv = (error - self.last_error) / 0.2
  self.integrator = self.integrator + error * 0.2
  self.integrator = math.max(math.min(self.integrator, self.i_max), self.i_min)
  
  local output = math.min(math.max(self.P * error + self.I * self.integrator + self.D * deriv,self.pid_min),self.pid_max)
  self.last_error = error
  
  return output
end

-- function PID:compute(setpoint, pv)
--   local error = setpoint - pv
--   local deriv = error - self.last_error
--   self.integrator = self.integrator + error
--   self.integrator = math.max(math.min(self.integrator, self.i_max), self.i_min)
  
--   local output = self.P * error + self.I * self.integrator + self.D * deriv
--   self.last_error = error
  
--   return output
-- end

-- -- Função para calcular a direção para o próximo waypoint
-- function get_bearing_to_next_waypoint()
--   local next_wp = mission:get_current_nav_cmd()  -- Pega o waypoint corrente
--   if next_wp then
--       local wp_lat, wp_lon = next_wp:x(), next_wp:y()  -- Coordenadas do waypoint
--       local current_lat, current_lon = gps:location()  -- Localização atual do GPS
--       local current_location = Location(current_lat, current_lon)
--       local wp_location = Location(wp_lat, wp_lon)
--       return current_location:get_bearing_to(wp_location) -- Calcula a direção para o waypoint
--   end
--   return nil
-- end

-- -- Função para calcular a velocidade ideal baseada na distância ao waypoint
-- function get_speed_to_next_waypoint()
--   local next_wp = mission:get_current_nav_cmd()
--   if next_wp then
--       local wp_lat, wp_lon = next_wp:x(), next_wp:y()
--       local current_lat, current_lon = gps:location()
--       local current_location = Location(current_lat, current_lon)
--       local wp_location = Location(wp_lat, wp_lon)
--       local distance = current_location:get_distance(wp_location) -- Calcula a distância para o waypoint
--       return math.min(1.0, distance / 10)  -- Velocidade proporcional à distância (ajuste conforme necessário)
--   end
--   return 0.1  -- Velocidade mínima
-- end


-- function update_control()
--   local setpoint_speed = get_speed_to_next_waypoint()  -- Velocidade ajustada para o waypoint
--   local setpoint_heading = get_bearing_to_next_waypoint()  -- Direção ajustada para o waypoint

--   local current_speed = vehicle:get_speed()  -- Velocidade atual do veículo
--   local current_heading = vehicle:yaw()      -- Direção atual do veículo

--   throttle_output = throttle_pid:compute(setpoint_speed, current_speed)
--   steering_output = steering_pid:compute(setpoint_heading, current_heading)

-- end


local function isempty(s)
  return s == nil or s == ''
end

function mapTo360(angle)
  if angle < 0 then
      return angle + 360
  else
      return angle
  end
end

function mapError(wp_bearing, yaw)

  local resultante = yaw - wp_bearing

  if resultante == 0 then

    return 0

  elseif math.abs(resultante) < 180 then
    
    return -resultante

  else 

    return (math.abs(resultante)/(resultante+0.001))*(360-math.abs(resultante))

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


  SRV_Channels:set_output_pwm_chan_timeout(0, nalocDir+PWM2_TRIM_VALUE, 300)
  SRV_Channels:set_output_pwm_chan_timeout(2, nalocEsq+PWM0_TRIM_VALUE, 300)


  
end

local steering_pid = PID:new(0.05, 0.01, 0.0, 0.8, -0.8, 0.8, -0.8)  -- Configure os ganhos como necessários

local rudder_pid = PID:new(0.05, 0.01, 0.0, 0.8, -0.8, 0.8, -0.8)
local throttle_pid = PID:new(0.05, 0.01, 0.0, 0.8, -0.8, 0.8, -0.8)

local last_wpx, last_wpy = 0,0
local current_wpx, current_wpy = 0,0


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

      if last_mission_index == -1 then
        last_mission_index = mission:get_current_nav_index()
        local mylocation = ahrs:get_position()

        last_wpx = mylocation:lat()/1e7
        last_wpy = mylocation:lng()/1e7

        local missionitem = mission:get_item(last_mission_index)
        current_wpx = missionitem:x()/1e7
        current_wpy = missionitem:y()/1e7

      end

      if vehicle:get_mode()< 10 then
        vechicle:set_mode(10)
      end

      local target = vehicle:get_target_location()
      local wp_bearing = vehicle:get_wp_bearing_deg()
      local vh_yaw = mapTo360(ahrs:get_yaw()*180.0/3.1415)
      local steering_error = mapError(wp_bearing,vh_yaw)

      local id0  = mission:get_current_nav_id()
      local nextwp = mission:get_prev_nav_cmd_id()

      steering = vehicle:get_control_output(CONTROL_OUTPUT_YAW)
      throttle = vehicle:get_control_output(CONTROL_OUTPUT_THROTTLE)

      local mysteering = steering_pid:compute(0,-steering_error)

      local mission_state = mission:state()

      --[[
        Verifica se terminou a missao
      --]]
      if mission_state == 2 then
        steering = 0
        throttle = 0
        vehicle:set_mode(0)
        last_mission_index = -1
      end

      local mission_index = mission:get_current_nav_index()

      if mission_index ~= last_mission_index then

        gcs:send_text(4, "LUA: New Mission Item") -- we spotted a change
    
        -- print the current and previous nav commands
        gcs:send_text(4, string.format("Prev: %d, Current: %d",last_mission_index, mission_index))
    
        last_mission_index = mission_index;

        local missionitem = mission:get_item(mission_index)
        local a5 = missionitem:x()
        local a6 = missionitem:y()
        gcs:send_text(4, string.format("%d %d",a5,a6)) 
   
    
      end
      
    
      --gcs:send_text(4, string.format("t,s, b = %f ; %f ; %f; %f",  throttle, steering,  steering_error, mysteering))
      

      
      CtrlAlocacaonovo(throttle,mysteering)


      return update, 200
    end

end

return update, 3000 -- run immediately before starting to reschedule
