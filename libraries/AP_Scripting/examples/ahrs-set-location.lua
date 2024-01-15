-- Example of receiving MAVLink commands

local mavlink_msgs = require("MAVLink/mavlink_msgs")

local COMMAND_ACK_ID = mavlink_msgs.get_msgid("COMMAND_ACK")
local COMMAND_LONG_ID = mavlink_msgs.get_msgid("COMMAND_LONG")

local msg_map = {}
msg_map[COMMAND_ACK_ID] = "COMMAND_ACK"
msg_map[COMMAND_LONG_ID] = "COMMAND_LONG"

-- initialize MAVLink rx with number of messages, and buffer depth
mavlink:init(1, 10)

-- register message id to receive
mavlink:register_rx_msgid(COMMAND_LONG_ID)

local MAV_CMD_USER_1 = 31010
local MAV_CMD_DO_YOU_ARE_HERE = MAV_CMD_USER_1

function handle_command_long(cmd)
    if (cmd.command == MAV_CMD_DO_YOU_ARE_HERE) then
        if cmd.param5 == 0 or cmd.param6 == 0 then
            return 2 -- MAV_RESULT_DENIED
        end
        local new_ahrs_location
        if cmd.param7 == 0 then
            new_ahrs_location = ahrs:get_location()
        else
            new_ahrs_location = Location()
            new_ahrs_location:alt(math.floor(cmd.param7*100))
        end
        if new_ahrs_location ~= nil then
            new_ahrs_location:lat(math.floor(cmd.param5*1.0e7))
            new_ahrs_location:lng(math.floor(cmd.param6*1.0e7))
            if ahrs:set_location(new_ahrs_location) then
                return 0 -- MAV_RESULT_ACCEPTED
            end
        end
        return 4 -- MAV_RESULT_FAILED
    end
    return nil
end

function update()
    local msg, chan = mavlink:receive_chan()
    if (msg ~= nil) then
        local parsed_msg = mavlink_msgs.decode(msg, msg_map)
        if (parsed_msg ~= nil) then

            local result
            if parsed_msg.msgid == COMMAND_LONG_ID then
                result = handle_command_long(parsed_msg)
            end

            if (result ~= nil) then
                -- Send ack if the command is one were intrested in
                local ack = {}
                ack.command = parsed_msg.command
                ack.result = result
                ack.progress = 0
                ack.result_param2 = 0
                ack.target_system = parsed_msg.sysid
                ack.target_component = parsed_msg.compid

                mavlink:send_chan(chan, mavlink_msgs.encode("COMMAND_ACK", ack))
            end
        end
    end

    return update, 1000
end

return update()
