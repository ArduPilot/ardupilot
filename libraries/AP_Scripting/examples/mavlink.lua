mavlink_msgs = require("mavlink_msgs")

msg_map = {}

heartbeat_msgid = mavlink_msgs.get_msgid("HEARTBEAT")

msg_map[heartbeat_msgid] = "HEARTBEAT"
-- initialise mavlink rx with number of messages, and buffer depth
mavlink.init(1, 10)
-- register message id to receive
mavlink.receive_msgid(heartbeat_msgid)
test_named_value = 0.0
function str_to_bytes(str)
    str_len = string.len(str)
    bytes =  {}
    for i = 1, str_len do
        bytes[i] = string.byte(str, i)
    end
    return bytes
end
function update()
    local msg = mavlink.receive()
    if msg then
        parsed_msg = mavlink_msgs.decode(msg, msg_map)
        if parsed_msg.msgid == heartbeat_msgid then
            gcs:send_text(6, string.format("Received heartbeat from %d", parsed_msg.sysid))
        end
    else
        gcs:send_text(6, "No heartbeat received")
    end
    test_named_value = test_named_value + 1.0
    -- send named value float to channel 0
    mavlink.send(0, mavlink_msgs.encode("NAMED_VALUE_FLOAT", {time_boot_ms = millis():toint(), name = str_to_bytes("test"), value = test_named_value}))
    return update, 1000
end

return update()
