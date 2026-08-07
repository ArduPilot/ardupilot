---@diagnostic disable: param-type-mismatch
---@diagnostic disable: missing-parameter

local mavlink_msgs = require("mavlink/mavlink_msgs")

local msg_map = {}

local heartbeat_msgid = mavlink_msgs.get_msgid("HEARTBEAT")

msg_map[heartbeat_msgid] = "HEARTBEAT"
-- initialize MAVLink rx with buffer depth and number of rx message IDs to register
mavlink.init(10, 1)
-- register message id to receive
mavlink.register_rx_msgid(heartbeat_msgid)
local test_named_value = 0.0
function update()
    local msg,chan,timestamp_ms = mavlink.receive_chan()
    if msg then
        gcs:send_text(6, string.format("Received message on channel %d at %s", chan, tostring(timestamp_ms)))
        local parsed_msg = mavlink_msgs.decode(msg, msg_map)
        if parsed_msg.msgid == heartbeat_msgid then
            gcs:send_text(6, string.format("Received heartbeat from %d", parsed_msg.sysid))
        end
    else
        gcs:send_text(6, "No heartbeat received")
    end
    test_named_value = test_named_value + 1.0
    -- send named value float
    mavlink.send_chan(0, mavlink_msgs.encode("NAMED_VALUE_FLOAT", {time_boot_ms = millis():toint(), name = "test", value = test_named_value}))
    return update, 1000
end

return update()
