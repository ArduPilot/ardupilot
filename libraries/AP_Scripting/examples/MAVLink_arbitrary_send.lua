local mavlink_msgs = require("MAVLink/mavlink_msgs")


local COMMAND_ACK_ID = mavlink_msgs.get_msgid("COMMAND_ACK")
local COMMAND_LONG_ID = mavlink_msgs.get_msgid("COMMAND_LONG")
local ARBITRARY_ID = mavlink_msgs.get_msgid("ARBITRARY")
local HEARTBEAT_ID = mavlink_msgs.get_msgid("HEARTBEAT")

local MAV_CMD_REQUEST_MESSAGE  = 512


local msg_map = {}
msg_map[COMMAND_ACK_ID] = "COMMAND_ACK"
msg_map[COMMAND_LONG_ID] = "COMMAND_LONG"
msg_map[ARBITRARY_ID] = "ARBITRARY"
msg_map[HEARTBEAT_ID] = "HEARTBEAT"

mavlink:init(10, 5)
local last_arbitrary_send=0
local GCS_chan=nil

function send_arbitrary()
	last_arbitrary_send=millis()
	if (GCS_chan==nil) then
		return
	end
	local arbitrary_msg = {}
	arbitrary_msg.target_system=255
	arbitrary_msg.target_component=190
	arbitrary_msg.flags=0x55
	arbitrary_msg.subtype=0xAA
	arbitrary_msg.payload={string.byte("sample run-time defined message send from ardupilot",1,-1)}
	mavlink:send_chan(GCS_chan, mavlink_msgs.encode("ARBITRARY", arbitrary_msg))
	
end

function handle_command_long(cmd,_)
    if (cmd.command == MAV_CMD_REQUEST_MESSAGE and cmd.param1==ARBITRARY_ID) then
        gcs:send_text(6, "Got ARBITRARY request")
	send_arbitrary()
	return 0
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
                result = handle_command_long(parsed_msg,chan)
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
            
            if parsed_msg.msgid==HEARTBEAT_ID and GCS_chan==nil then
				--check sysid/compid of msg and check to see if they match the "standard" gcs
				if(string.byte(msg,3)==0xFD and string.byte(msg,8)==255 and string.byte(msg,9)==190) then	--mavlink type 2 message        	
					gcs:send_text(6, "got GCS channel")
					GCS_chan=chan
				end
				if(string.byte(msg,3)==0xFE and string.byte(msg,6)==255 and string.byte(msg,7)==190) then	--mavlink type 1 message        	
					gcs:send_text(6, "got GCS channel")
					GCS_chan=chan
				end
            end
            	
        end
    end

    if (millis()-last_arbitrary_send)>20000 then
		gcs:send_text(6, "Timer ARBITRARY send")
		send_arbitrary()
    end

    return update, 10
end

mavlink:register_rx_msgid(COMMAND_LONG_ID)
mavlink:register_rx_msgid(HEARTBEAT_ID)
mavlink:hook_message_request(ARBITRARY_ID)
return update()

