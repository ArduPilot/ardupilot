-- this script sets the update rate of mavlink messages on serial port X
-- gcs:set_message_interval(uint8_t port_num, uint32_t msg_id, int32_t interval_us)
--   port_num: The serial port (SerialX).
--   msg_id: The MAVLink message ID
--   (see https://mavlink.io/en/messages/common.html or https://mavlink.io/en/messages/ardupilotmega.html)
--   interval_us: The interval between two messages. Set to -1 to disable and 0 to request default rate.
function update ()
  gcs:set_message_interval(0, uint32_t()+30, 500000) -- ATTITUDE 2Hz
  gcs:set_message_interval(0, uint32_t()+163, 200000) -- AHRS 5Hz

end

return update, 5000
