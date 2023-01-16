-- This script is an example of writing to CAN bus

-- Load CAN driver, using the scripting protocol and with a buffer size of 5
local driver = CAN:get_device(5)

-- transfer ID of the message were sending
local FrameId = 0x223

-- msgs to request attitude
local request_attitude_msg1 = {0xAA, 0x13, 0x00, 0x03, 0x00, 0x00, 0x00, 0x00}
local request_attitude_msg2 = {0x03, 0x00, 0x10, 0x4E, 0x0E, 0x02, 0x01, 0x08}
local request_attitude_msg3 = {0x9F, 0xB2, 0x07}

function send_can_msg(msg_data)
  canframe = CANFrame()
  canframe:id(FrameId)
  canframe:dlc(#msg_data)
  for i = 0, #msg_data-1 do
    canframe:data(i,msg_data[i+1])
  end
  driver:write_frame(canframe, 10000)
  -- debug
  local is_ext = 0
  if canframe:isExtended() then
    is_ext = 1
  end
  gcs:send_text(0,string.format("W fi:" .. tostring(canframe:id()) .. " dlc:%u x:%d %x %x %x %x %x %x %x %x", canframe:dlc(), is_ext, canframe:data(0), canframe:data(1), canframe:data(2), canframe:data(3), canframe:data(4), canframe:data(5), canframe:data(6), canframe:data(7)))
  --gcs:send_text(0,string.format("Wr fi:" .. tostring(canframe:id()) .. " dlc:%u ex:%d", canframe:dlc(), is_ext))
end

--userdata AP_HAL::CANFrame rename CANFrame
--userdata AP_HAL::CANFrame field id uint32_t'skip_check read write
--userdata AP_HAL::CANFrame field data'array int(ARRAY_SIZE(ud->data)) uint8_t'skip_check read write
--userdata AP_HAL::CANFrame field dlc uint8_t read write 0 int(ARRAY_SIZE(ud->data))
--userdata AP_HAL::CANFrame method id_signed int32_t
--userdata AP_HAL::CANFrame method isExtended boolean
--userdata AP_HAL::CANFrame method isRemoteTransmissionRequest boolean
--userdata AP_HAL::CANFrame method isErrorFrame boolean

function update()

  -- send DJI R2 gimbal command to retrieve attitude
  send_can_msg(request_attitude_msg1)
  send_can_msg(request_attitude_msg2)
  send_can_msg(request_attitude_msg3)

  -- update every 3 seconds
  return update, 5000

end

return update()
