--[[
    Example of sending unknown MAVLink messages

    Requires modules/MAVLink to have been generated after modification to mavgen that includes
    additional metadata in `mavlink_msg_*.lua` files and definition of `encode_full` function
--]]

local mavlink_msgs = require("MAVLink/mavlink_msgs")

function update()
    local named_value_float = {
        time_boot_ms = millis():toint(),
        name = "test",
        value = 123.45
    }

    mavlink:send_chan_unknown(0, mavlink_msgs.encode_full("NAMED_VALUE_FLOAT", named_value_float))


    return update, 1000
end

return update()
