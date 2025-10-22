-- TI BQ40Z BMS shutdown script

local mavlink_msgs = require("MAVLink/mavlink_msgs")

local COMMAND_ACK_ID = mavlink_msgs.get_msgid("COMMAND_ACK")
local COMMAND_LONG_ID = mavlink_msgs.get_msgid("COMMAND_LONG")
local msg_map = {}
msg_map[COMMAND_ACK_ID] = "COMMAND_ACK"
msg_map[COMMAND_LONG_ID] = "COMMAND_LONG"

local PARAM_TABLE_KEY = 51
local PARAM_TABLE_PREFIX = "BATT_BQ40Z_"
-- add a parameter and bind it to a variable
local function bind_add_param(name, idx, default_value)
    assert(param:add_param(PARAM_TABLE_KEY, idx, name, default_value), string.format('could not add param %s', name))
    return Parameter(PARAM_TABLE_PREFIX .. name)
end
assert(param:add_table(PARAM_TABLE_KEY, PARAM_TABLE_PREFIX, 2), 'could not add param table')
--[[
  // @Param: BATT_BQ40Z_BUS
  // @DisplayName: Bus number for the BQ40Z
  // @Description: Bus number for the BQ40Z
  // @Range: 0 3
  // @User: Standard
--]]
local BATT_BQ40Z_BUS = bind_add_param('BUS', 1, 0)


local MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN = 246
local MAV_RESULT_ACCEPTED = 0
local MAV_RESULT_FAILED = 4

-- initialize MAVLink rx with buffer depth and number of rx message IDs to register
mavlink:init(10, 1)
-- Register message id to receive
mavlink:register_rx_msgid(COMMAND_LONG_ID)
-- Block MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN so we can handle the ACK
mavlink:block_command(MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN)

-- Init BMS i2c device
local i2c_addr = BATT_BQ40Z_BUS:get()
assert(i2c_addr ~= nil, "BATT_BQ40Z_BUS not retrievable")
local bms = i2c:get_device(math.floor(i2c_addr), 0x0B)

-- Exit emergency shutdown (for BQ40Z60, twice for redundancy)
bms:transfer("\x00\xA7\x23", 0)
bms:transfer("\x00\xA7\x23", 0)

-- Function that is returned to the AP scheduler when we want to shutdown
local function shutdown_loop()
    local ret = bms:transfer("\x00\x10\x00", 0)
    if ret == nil then
        gcs:send_text(0, "BQ40Z shutdown transfer failed")
    end

    return shutdown_loop, 500
end

-- Main loop
local function update()
    local msg, chan = mavlink:receive_chan()
    local parsed_msg = nil

    if (msg ~= nil) then
        parsed_msg = mavlink_msgs.decode(msg, msg_map)
    end

    if parsed_msg ~= nil and (parsed_msg.msgid == COMMAND_LONG_ID) and (parsed_msg.command == MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN) then
        -- Prepare ack
        local ack = {}
        ack.command = parsed_msg.command
        ack.result = MAV_RESULT_ACCEPTED
        ack.progress = 0
        ack.result_param2 = 0
        ack.target_system = parsed_msg.sysid
        ack.target_component = parsed_msg.compid

        -- Don't shutdown if armed
        if arming:is_armed() and parsed_msg.param1 == 2 then
            gcs:send_text(1, "Not sutting down BQ40Z as vehicle is armed")
            ack.result = MAV_RESULT_FAILED
            mavlink:send_chan(chan, mavlink_msgs.encode("COMMAND_ACK", ack))

        -- Shutdown
        elseif parsed_msg.param1 == 2 then
            gcs:send_text(1, "Shutting down BQ40Z!!!")
            mavlink:send_chan(chan, mavlink_msgs.encode("COMMAND_ACK", ack))
            return shutdown_loop, 0

        -- Pass through the command if it isn't requesting shutdown
        else
            local command_int = {
                p1 = parsed_msg.param1,
                p2 = parsed_msg.param2,
                p3 = parsed_msg.param3,
                p4 = parsed_msg.param4,
            }
            local result = gcs:run_command_int(MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN, command_int)
            ack.result = result
            mavlink:send_chan(chan, mavlink_msgs.encode("COMMAND_ACK", ack))
        end
    end

    return update, 1000
end


return update, 1000
