-- This script is an example of writing to CAN bus

-- Load CAN driver, using the scripting protocol and with a buffer size of 5
local driver = CAN:get_device(5)

-- transfer ID of the message were sending
local Transfer_ID = 0

-- RGB colours in 0 - 255 range, and RGB fade speed
local red = 255
local green = 0
local blue  = 0
local fade_speed = 5


function update()

  -- send UAVCAN Light command
  -- uavcan.equipment.indication.LightsCommand
  -- note that as we don't do dynamic node allocation, the target light device must have a static node ID

  msg = CANFrame()

  -- extended frame, priority 30, message ID 1081 and node ID 11
  -- lua cannot handle numbers so large, so we have to use uint32_t userdata
  msg:id( (uint32_t(1) << 31) | (uint32_t(30) << 24) | (uint32_t(1081) << 8) | uint32_t(11) )

  msg:data(0,0) -- set light_id = 0

  -- convert colors to 565 rgb
  local red_5bit   = red >> 3
  local green_6bit = green >> 2
  local blue_5bit  = blue >> 3

  -- first is made up of 5 bits red and 3 bits of green
  msg:data(1, (red_5bit << 3) | (green_6bit >> 3))

  -- remaining 3 bits of green and 5 of blue
  msg:data(2, ((green_6bit << 5) | blue_5bit) & 0xFF)

  -- Tail includes, start of transfer, end of transfer, toggle and transfer ID bits.
  msg:data(3, (1 << 7) | (1 << 6) | Transfer_ID)

  Transfer_ID = Transfer_ID + 1
  -- transfer ID is 5 bits
  if Transfer_ID > 31 then
    Transfer_ID = 0
  end

  -- sending 4 bytes of data
  msg:dlc(4)

  -- write the frame with a 10000us timeout
  driver:write_frame(msg, 10000)

  -- basic RGB fade
  if red > 0 and blue == 0 then
    red = red - fade_speed
    green = green + fade_speed
  end
  if green > 0 and red == 0 then
    green = green - fade_speed
    blue = blue + fade_speed
  end
  if blue > 0 and green == 0 then
    red = red + fade_speed
    blue = blue - fade_speed
  end

  return update, 100

end

return update()
