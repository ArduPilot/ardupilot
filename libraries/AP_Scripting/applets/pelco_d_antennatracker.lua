--[[
   Pelco-D control implementation for antennatracker.
   Implemented by using knowledge from the excellent python implementation in https://gist.github.com/jn0/cc5c78f4a0f447a6fb2e45a5d9efa13d.
--]]

local SERVO_PAN = 71
local SERVO_TILT = 72
local SERIAL_BAUD = 9600

-- Antennattracker modes
local MODE_STOP = 1
local MODE_SCAN = 2
local MODE_SERVOTEST = 3
local MODE_GUIDED = 4

gcs:send_text(0, "Starting Pelco-D Control")

local port = assert(serial:find_serial(0), "Pelco-D: No Scripting Serial Port")
port:begin(SERIAL_BAUD)
port:set_flow_control(0)

function set_bit(value, n)
  return value | (0x01 << n) 
end

function PelcoD_msg_addchecksum(msg)
  local sum = 0
  for i = 2, #msg-1 do
      sum = sum + msg[i]
  end
  checksum =  sum % 256
  msg[7] = checksum
end

function PelcoD_move(panspeed, tiltspeed)
  local command = 0x00
  local scale = 63 -- max pelcod speed
  if panspeed < 0 then -- left
      command = set_bit(command, 2)
  elseif panspeed > 0 then -- right
      command = set_bit(command, 1)
  end
  if tiltspeed < 0 then -- down
      command = set_bit(command, 4)
  elseif tiltspeed > 0 then -- up
      command = set_bit(command, 3)
  end
  local msg = {0xFF, 0x01, 0x00, command, math.floor(math.abs(panspeed) * scale), math.floor(math.abs(tiltspeed) * scale), 0x00}
  PelcoD_msg_addchecksum(msg)
  return msg
end

-- write msg to the serial port
function send_message(msg)
  for _, v in ipairs(msg) do
    port:write(v)
  end
end
 
function update()
  tilt_norm = SRV_Channels:get_output_scaled(SERVO_TILT)
  pan_norm = SRV_Channels:get_output_scaled(SERVO_PAN)
  if (vehicle:get_mode() == MODE_SCAN or vehicle:get_mode() == MODE_SERVOTEST or vehicle:get_mode() == MODE_GUIDED) then
    -- Limit pan and tilt to -1...+1
    pan_norm=math.max(pan_norm,-1.0)
    pan_norm=math.min(pan_norm,1.0)
    tilt_norm=math.max(tilt_norm,-1.0)
    tilt_norm=math.min(tilt_norm,1.0)
    local msg=PelcoD_move(-pan_norm,tilt_norm)
    send_message(msg)
  elseif (vehicle:get_mode() == MODE_STOP) then
    local msg=PelcoD_move(0,0)
    send_message(msg)
  end 

  return update, 20 -- 50 hz
end

PelcoD_move(0,0)
return update()


--[[
function PelcoD_pan_absolute_position(degrees)
  centidegrees = degrees*100
  local msg = {0xFF, 0x01, 0x00, 0x4b, (centidegrees >> 8) & 255 , centidegrees & 255, 0x00}
  PelcoD_msg_addchecksum(msg)
  return msg
end

function PelcoD_tilt_absolute_position(degrees)
  centidegrees = degrees*100
  local msg = {0xFF, 0x01, 0x00, 0x4d, (centidegrees >> 8) & 255 , centidegrees & 255, 0x00}
  PelcoD_msg_addchecksum(msg)
  return msg
end

function PelcoD_zoom_absolute_position(position)
  local msg = {0xFF, 0x01, 0x00, 0x4f, (position >> 8) & 255 , position & 255, 0x00}
  PelcoD_msg_addchecksum(msg)
  return msg
end

function PelcoD_zero_absolute_position(degrees)
  centidegrees = degrees*100
  local msg = {0xFF, 0x01, 0x00, 0x49, 0x00, 0x00, 0x00}
  PelcoD_msg_addchecksum(msg)
  return msg
end
--]]