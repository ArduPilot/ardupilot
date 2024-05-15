

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


local function isempty(s)
  return s == nil or s == ''
end

function mapToUnid(value)
  if value < -1 then
      return  -1
  elseif value > 1 then
    return 1
  else
      return value
  end
end

function mapTo360(angle)
  if angle < 0 then
      return angle + 360
  else
      return angle
  end
end

function mapError(resultante)

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


local rudder_pid = PID:new(0.05, 0.01, 0.0, 0.8, -0.8, 0.8, -0.8)
local throttle_pid = PID:new(0.05, 0.01, 0.0, 0.8, -0.8, 0.8, -0.8)

local last_wpx, last_wpy = 0,0
local current_wpx, current_wpy = 0,0


function not_armed()
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
end

function manual_mode()

  local rc3_pwm = 0
  local rc1_pwm = 0

  trim3 = param:get('RC3_TRIM')
  trim1 = param:get('RC1_TRIM')

  rc3_pwm = rc:get_pwm(3)
  rc1_pwm = rc:get_pwm(1)

  throttle = (trim3 - rc3_pwm) / 450
  steering = (rc1_pwm - trim1) / 450

  CtrlAlocacaonovo(throttle,steering)
  
end

function point_relative_to_vector(p0x, p0y, p1x, p1y, rx, ry)
  -- Calcula as componentes do vetor AB
  local vx = p1x - p0x
  local vy = p1y - p0y

  -- Calcula o produto vetorial em 2D
  local cross_product = vx * (ry - p0y) - vy * (rx - p0x)

  -- Interpreta o resultado do produto vetorial
  if cross_product > 0 then
      return 1  -- O ponto está à esquerda do vetor
  elseif cross_product < 0 then
      return -1 -- O ponto está à direita do vetor
  else
      return 0  -- O ponto está na linha do vetor
  end
end


-- Função para converter graus para radianos
function to_radians(mdegrees)
  return mdegrees * math.pi / 180.0
end

function to_degrees(mradians)
  return mradians * 180 / math.pi
end

function vector_magnitude(x, y)
  return math.sqrt(x^2 + y^2)
end

function dot_product(u_x, u_y, v_x, v_y)
  return u_x * v_x + u_y * v_y
end

function calculate_angle(u_x, u_y, v_x, v_y)
  local dot = dot_product(u_x, u_y, v_x, v_y)
  local mag_u = vector_magnitude(u_x, u_y)
  local mag_v = vector_magnitude(v_x, v_y)
  local cos_theta = dot / (mag_u * mag_v)
  local angle = math.acos(cos_theta)  -- Resultado em radianos
  return angle
end

local function  calculate_correction_angle(p0x, p0y, p1x, p1y, rx, ry)
  local dxwp = p1x - p0x
  local dywp = p1y - p0y
  local dx_r_wp = p1x - rx
  local dy_r_wp = p1y - ry

  local angle = calculate_angle(dxwp, dywp, dx_r_wp, dy_r_wp)
end

-- Calcula a orientação de um vetor dado
local function calculate_bearing(lat1, lon1, lat2, lon2)
  local radLat1, radLon1 = to_radians(lat1), to_radians(lon1)
  local radLat2, radLon2 = to_radians(lat2), to_radians(lon2)
  local dLon = radLon2 - radLon1
  local y = math.sin(dLon) * math.cos(radLat2)
  local x = math.cos(radLat1) * math.sin(radLat2) - math.sin(radLat1) * math.cos(radLat2) * math.cos(dLon)
  local bearing = math.atan(y, x)
  return (to_degrees(bearing) + 360) % 360  -- Normaliza o resultado para (0, 360)
end

-- Função Haversine para calcular distância entre dois pontos geográficos
function haversine_distance(lat1, lon1, lat2, lon2)
  local R = 6371000 -- Raio da Terra em metros
  local radLat1, radLon1 = to_radians(lat1), to_radians(lon1)
  local radLat2, radLon2 = to_radians(lat2), to_radians(lon2)
  local deltaLat = radLat2 - radLat1
  local deltaLon = radLon2 - radLon1

  local a = math.sin(deltaLat / 2) ^ 2 + 
            math.cos(radLat1) * math.cos(radLat2) * 
            math.sin(deltaLon / 2) ^ 2

  local c = 2 * math.atan(math.sqrt(a), math.sqrt(1 - a))

  return R * c
end


function Point_to_line_distance(px, py, psi, x0, y0, x1, y1)
  local dx, dy = x1 - x0, y1 - y0
  local length_squared = dx^2 + dy^2
  local t = ((px - x0) * dx + (py - y0) * dy) / length_squared

  -- Se a projeção cai fora do segmento de reta, o mais próximo é o ponto final mais próximo
    local nearest_x, nearest_y
    if t < 0 then
        nearest_x, nearest_y = x0, y0
    elseif t > 1 then
        nearest_x, nearest_y = x1, y1
    else
        nearest_x = x0 + t * dx
        nearest_y = y0 + t * dy
    end

  local distancia = haversine_distance(px, py, nearest_x, nearest_y)
  local bearing_to_wp = calculate_bearing(px, py, x1, y1)
  --local angle_difference = (bearing_to_wp - psi + 360) % 360
  local angle_difference = (bearing_to_wp - psi/2 + 360) % 360

  local orientacao = point_relative_to_vector(x0,y0,x1,y1,px,py)

  return distancia, orientacao*angle_difference


end



function update_mission_setpoints()

  local mission_state = mission:state()
  --[[
    Verifica se terminou a missao
  --]]
  if mission_state == 2 then
    steering = 0
    throttle = 0
    vehicle:set_mode(0)
    last_mission_index = -1

    return steering, throttle
  end

  if last_mission_index == -1 then

    last_mission_index = mission:get_current_nav_index()
    
    local mylocation = ahrs:get_position()

    last_wpx = mylocation:lat()/1e7
    last_wpy = mylocation:lng()/1e7

    
    local missionitem = mission:get_item(last_mission_index)
    current_wpx = missionitem:x()/1e7
    current_wpy = missionitem:y()/1e7

  end

  local mission_index = mission:get_current_nav_index()

  if mission_index ~= last_mission_index then

    gcs:send_text(4, "LUA: New Mission Item") -- we spotted a change

    last_wpx = current_wpx
    last_wpy = current_wpy

    last_mission_index = mission_index;

    local missionitem = mission:get_item(mission_index)
    current_wpx = missionitem:x()/1e7
    current_wpy= missionitem:y()/1e7

  end

  local mylocation = ahrs:get_position()
  local myx = mylocation:lat()/1e7
  local myy = mylocation:lng()/1e7

  local vh_yaw = mapTo360(to_degrees(ahrs:get_yaw()))


  local dist, ang = Point_to_line_distance(myx, myy, vh_yaw, last_wpx, last_wpy, current_wpx, current_wpy)

  return dist, ang
  
end

function to_cartesian(r, theta)
  local x = r * math.cos(theta)
  local y = r * math.sin(theta)
  return x, y
end

function to_polar(x, y)
  local r = math.sqrt(x^2 + y^2)
  local theta = math.atan(y, x)
  return r, theta
end

function add_polars(r1, theta1, r2, theta2)
  local x1, y1 = to_cartesian(r1, theta1)
  local x2, y2 = to_cartesian(r2, theta2)

  local x_total = x1 + x2
  local y_total = y1 + y2

  return to_polar(x_total, y_total)
end


local steering_pid = PID:new(0.05, 0.01, 0.0, 0.8, -0.8, 0.8, -0.8)  -- Configure os ganhos como necessários


function update_simple_setpoints()

  local wp_bearing = vehicle:get_wp_bearing_deg()
  local vh_yaw = mapTo360(ahrs:get_yaw()*180.0/3.1415)
  local steering_error = mapError(vh_yaw - wp_bearing)

  --steering = vehicle:get_control_output(CONTROL_OUTPUT_YAW)
  throttle = tonumber(vehicle:get_control_output(CONTROL_OUTPUT_THROTTLE))

  local mysteering = steering_pid:compute(0,-steering_error)

  return mysteering, throttle

end







local newsteering_pid = PID:new(0.001, 0.03, 0.0, 0.9, -0.9, 0.9, -0.9)  


function update_disturbed_setpoints()

  local distance,newsteering_error = update_mission_setpoints()
  --steering = vehicle:get_control_output(CONTROL_OUTPUT_YAW)
  throttle = tonumber(vehicle:get_control_output(CONTROL_OUTPUT_THROTTLE))

  local mysteering = newsteering_pid:compute(0,newsteering_error)

  return mysteering, throttle

end






function update() -- this is the loop which periodically runs

  local tipoveiculo = param:get('SCR_USER5')

  if not (tipoveiculo==2) then
    gcs:send_text(4, string.format("nao e ROVER saindo do lua"))
    return
  end

  if not arming:is_armed() then
    
    not_armed()

    return update, 2000
  end

    -- steering = vehicle:get_control_output(CONTROL_OUTPUT_YAW) 
    -- throttle = vehicle:get_control_output(CONTROL_OUTPUT_THROTTLE)

  if vehicle:get_mode()== 0 then

    manual_mode()
    return update, 200

  else

    if vehicle:get_mode()< 10 then
      vehicle:set_mode(10)
    end
    

    local mission_state = mission:state()
    local newsteering, newthrottle

    if mission_state == 0 then
      steering, throttle = update_simple_setpoints()
      local testtarget = vehicle:get_wp_distance_m()
      local testbearing = vehicle:get_wp_bearing_deg()
      newsteering, newthrottle =steering, throttle

    else
      steering, throttle = update_simple_setpoints()
      newsteering, newthrottle = update_disturbed_setpoints()
      
      
      --gcs:send_text(4, string.format("distancia ao SR %f - ang %f",ajdthrotte,ajdsteering) )

    end
    

    

    --steering, throttle = update_mission_setpoints()

    CtrlAlocacaonovo(newthrottle, (0.4*steering+ 0.6*newsteering))
      

    --CtrlAlocacaonovo(throttle,steering)


      return update, 200

  end

end

return update, 3000 -- run immediately before starting to reschedule
