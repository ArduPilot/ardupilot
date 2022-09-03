-- Control MiniCheetah motor driver over CAN
-- https://os.mbed.com/users/benkatz/code/HKC_MiniCheetah/docs/tip/CAN__com_8cpp_source.html

-- Load CAN driver with a buffer size of 20
local driver = CAN:get_device(20)

local target_ID = uint32_t(1)

local pos_max = 12.5
local vel_max = 65
local Kp_min = 0
local Kp_max = 500
local Kd_min = 0
local kd_max = 5
local torque_max = 18

local position_des = 0
local position_inc = 0.01

-- convert decimal to int within given range and width
function to_uint(val, min, max, bits)
  local range = max - min
  local int_range = 0
  for i = 0, bits - 1 do
    int_range = int_range | (1 << i)
  end
  return math.floor((((val - min)/range) * int_range) + 0.5)
end

-- convert int to decimal within given range and width
function from_uint(val, min, max, bits)
  local range = max - min
  local int_range = 0
  for i = 0, bits - 1 do
    int_range = int_range | (1 << i)
  end
  return ((val / int_range) * range) + min
end

-- send a motor command
function send(position, velocity, Kp, Kd, torque)

  -- 16 bit position command, between -4*pi and 4*pi
  -- 12 bit velocity command, between -30 and + 30 rad/s
  -- 12 bit kp, between 0 and 500 N-m/rad
  -- 12 bit kd, between 0 and 100 N-m*s/rad
  -- 12 bit feed forward torque, between -18 and 18 N-m

  -- range check
  assert(math.abs(position) <= pos_max, "position out of range")
  assert(math.abs(velocity) <= vel_max, "velocity out of range")
  assert((Kp >= Kp_min) and (Kp <= Kp_max), "Kp out of range")
  assert((Kd >= Kd_min) and (Kd <= kd_max), "Kd out of range")
  assert(math.abs(torque) <= torque_max, "torque out of range")

  -- convert from decimal to integer
  position = to_uint(position, -pos_max,    pos_max,    16)
  velocity = to_uint(velocity, -vel_max,    vel_max,    12)
  Kp       = to_uint(Kp,        Kp_min,     Kp_max,     12)
  Kd       = to_uint(Kd,        Kd_min,     kd_max,     12)
  torque   = to_uint(torque,   -torque_max, torque_max, 12)

  msg = CANFrame()
  msg:id(target_ID)

  -- 0: [position[15-8]]
  msg:data(0, position >> 8)

  -- 1: [position[7-0]] 
  msg:data(1, position & 0xFF)

  -- 2: [velocity[11-4]]
  msg:data(2, velocity >> 4)

  -- 3: [velocity[3-0], kp[11-8]]
  msg:data(3, ((velocity << 4) | (Kp >> 8)) & 0xFF)

  -- 4: [kp[7-0]]
  msg:data(4, Kp & 0xFF)

  -- 5: [kd[11-4]]
  msg:data(5, Kd >> 4)

  -- 6: [kd[3-0], torque[11-8]]
  msg:data(6, ((Kd << 4) | (torque >> 8)) & 0xFF)

  -- 7: [torque[7-0]]
  msg:data(7, torque & 0xFF)

  -- sending 8 bytes of data
  msg:dlc(8)

  -- write the frame with a 10000us timeout
  driver:write_frame(msg, 10000)

end

-- send command to enable motor
function enable()
  msg = CANFrame()

  msg:id(target_ID)

  msg:data(0, 0xFF)
  msg:data(1, 0xFF)
  msg:data(2, 0xFF)
  msg:data(3, 0xFF)
  msg:data(4, 0xFF)
  msg:data(5, 0xFF)
  msg:data(6, 0xFF)
  msg:data(7, 0xFC)

  msg:dlc(8)

  driver:write_frame(msg, 10000)
end

-- send command to disable motor
function disable()
  msg = CANFrame()

  msg:id(target_ID)

  msg:data(0, 0xFF)
  msg:data(1, 0xFF)
  msg:data(2, 0xFF)
  msg:data(3, 0xFF)
  msg:data(4, 0xFF)
  msg:data(5, 0xFF)
  msg:data(6, 0xFF)
  msg:data(7, 0xFD)

  msg:dlc(8)

  driver:write_frame(msg, 10000)
end

-- send command to zero motor
function zero()
  msg = CANFrame()

  msg:id(target_ID)

  msg:data(0, 0xFF)
  msg:data(1, 0xFF)
  msg:data(2, 0xFF)
  msg:data(3, 0xFF)
  msg:data(4, 0xFF)
  msg:data(5, 0xFF)
  msg:data(6, 0xFF)
  msg:data(7, 0xFE)

  msg:dlc(8)

  driver:write_frame(msg, 10000)
end

-- receive data from motor
function receive()

    -- Read a message from the buffer
    frame = driver:read_frame()

    -- noting waiting, return early
    if not frame then
      return
    end

  -- 8 bit ID
  -- 16 bit position, between -4*pi and 4*pi
  -- 12 bit velocity, between -30 and + 30 rad/s
  -- 12 bit current, between -40 and 40;

  -- 0: [ID[7-0]]
  -- 1: [position[15-8]]
  -- 2: [position[7-0]]
  -- 3: [velocity[11-4]]
  -- 4: [velocity[3-0], current[11-8]]
  -- 5: [current[7-0]]

  local ID = frame:data(0)
  local position = (frame:data(1) << 8) | frame:data(2)
  local velocity = (frame:data(3) << 4) | (frame:data(4) >> 4)
  local current = ( (frame:data(4) << 8) | frame:data(5)) & 0xFFF

  -- from integer to decimal
  position = from_uint(position, -pos_max,    pos_max,    16)
  velocity = from_uint(velocity, -vel_max,    vel_max,    12)
  current  = from_uint(current,  -torque_max, torque_max, 12)

  return ID, position, velocity, current

end

function update()

  send(position_des, 0, 100, 1, 0)

  local ID, position, velocity, current = receive()
  if ID then
    gcs:send_named_float('POS',position)
    gcs:send_named_float('VEL',velocity)
    gcs:send_named_float('CUR',current)
  end

  position_des = position_des + position_inc

  if position_des > pos_max then
    position_inc = -math.abs(position_inc)
    position_des = pos_max
  end
  if position_des < -pos_max then
    position_inc = math.abs(position_inc)
    position_des = -pos_max
  end

  return update, 10

end

function init()
  enable()
  return update, 100
end

return init, 1000
