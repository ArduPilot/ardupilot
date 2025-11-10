local ser_driver = serial:find_serial(0)
local ser_device = serial:find_simulated_device(28, 0)

if ser_driver == nil or ser_device == nil then
  error("bad config")
end

ser_driver:begin(115200) -- baud rate does not matter

function test_driver_to_device()
  local msg_send = "hello device"
  local num_sent = 0
  for ci = 1,#msg_send do
    num_sent = num_sent + ser_driver:write(msg_send:byte(ci, ci)):toint()
  end
  local msg_recv = ser_device:readstring(#msg_send)
  if msg_send == msg_recv and num_sent == #msg_send then
    gcs:send_text(6, "driver -> device good")
  end
end

function test_device_to_driver()
  local msg_send = "hello driver"
  local num_sent = ser_device:writestring(msg_send)
  local msg_recv = ser_driver:readstring(#msg_send)
  if msg_send == msg_recv and num_sent == #msg_send then
    gcs:send_text(6, "device -> driver good")
  end
end

function update()
  test_driver_to_device()
  test_device_to_driver()
end

return update()
