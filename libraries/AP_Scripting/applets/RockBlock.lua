--[[ Lua script to send a recieve very basic MAVLink telemetry over a
Rockblock SBD satellite modem
Requires https://github.com/stephendade/rockblock2mav at the GCS end

Setup:
This script requires 1 serial port:
A "Script" to connect the RockBlock modem

Usage:
Use the MAVLink High Latency Control ("link hl on|off" in MAVProxy) to control
whether to send or not (or use "force_hl_enable")
Use the RCK_DEBUG param to view debugging statustexts at the GCS
Use the RCK_FORCEHL param to control the mode of operation: 0=Disabled, 1=Enabled, 2=Enabled on 5000ms telemetry loss

Caveats:
This will *only* send HIGH_LATENCY2 packets via the SBD modem. No heartbeats,
no command acknowledgements, no statustexts, no parameters, etc
A single HIGH_LATENCY2 packet will be send every RCK_PERIOD seconds
MAVLink 1 will be used, as it's slightly more efficient (50 vs 52 bytes for a HL2 message)
Any incoming packets on the first mailbox check will be ignored (as these may be from a long time in the past)
Only 1 command can be sent at a time from the GCS. Any subsequent commands will overwrite the previous command
The param SCR_VM_I_COUNT may need to be increased in some circumstances

Written by Stephen Dade (stephen_dade@hotmail.com)
]]--

local PARAM_TABLE_KEY = 10
local PARAM_TABLE_PREFIX = "RCK_"

local port = serial:find_serial(0)

if not port then
    gcs:send_text(0, "Rockblock: No Scripting Serial Port")
    return
end

port:begin(19200)
port:set_flow_control(0)

-- number of millsec that GCS telemetry has been lost for
local link_lost_for = 0

-- bind a parameter to a variable
function bind_param(name)
    local p = Parameter()
    assert(p:init(name), string.format('could not find %s parameter', name))
    return p
 end
 
 -- add a parameter and bind it to a variable
 function bind_add_param(name, idx, default_value)
    assert(param:add_param(PARAM_TABLE_KEY, idx, name, default_value), string.format('could not add param %s', name))
    return bind_param(PARAM_TABLE_PREFIX .. name)
 end

 -- setup RCK specific parameters
assert(param:add_table(PARAM_TABLE_KEY, PARAM_TABLE_PREFIX, 5), 'could not add param table')
--[[
  // @Param: RCK_FORCEHL
  // @DisplayName: Force enable High Latency mode
  // @Description: Automatically enables High Latency mode if not already enabled
  // @Values: 0:Disabled,1:Enabled,2:Enabled on telemetry loss
  // @User: Standard
--]]
RCK_FORCEHL     = bind_add_param('FORCEHL', 1, 0)

--[[
  // @Param: RCK_PERIOD
  // @DisplayName: Update rate
  // @Description: When in High Latency mode, send Rockblock updates every N seconds
  // @Range: 0 600
  // @Units: s
  // @User: Standard
--]]
RCK_PERIOD     = bind_add_param('PERIOD', 2, 30)

--[[
  // @Param: RCK_DEBUG
  // @DisplayName: Display Rockblock debugging text
  // @Description: Sends Rockblock debug text to GCS via statustexts
  // @Values: 0:Disabled,1:Enabled
  // @User: Standard
--]]
RCK_DEBUG     = bind_add_param('DEBUG', 3, 0)

--[[
  // @Param: RCK_ENABLE
  // @DisplayName: Enable Message transmission
  // @Description: Enables the Rockblock sending and recieving
  // @Values: 0:Disabled,1:Enabled
  // @User: Standard
--]]
RCK_ENABLE     = bind_add_param('ENABLE', 4, 1)

--[[
  // @Param: RCK_TIMEOUT
  // @DisplayName: GCS timeout to start sendin Rockblock messages
  // @Description: If RCK_FORCEHL=2, this is the number of seconds of GCS timeout until High Latency mode is auto-enabled
  // @Range: 0 600
  // @Units: s
  // @User: Standard
--]]
RCK_TIMEOUT     = bind_add_param('TIMEOUT', 5, 5)

--[[
Returns true if the value is NaN, false otherwise
--]]
local function isNaN (x)
    return (x ~= x)
  end

--[[
Lua Object for decoding and encoding MAVLink (V1 only) messages
--]]
local function MAVLinkProcessor()
    -- public fields
    local self = {
        -- define MAVLink message id's
        COMMAND_LONG = 76,
        COMMAND_INT = 75,
        HIGH_LATENCY2 = 235,
        MISSION_ITEM_INT = 73,
        SET_MODE = 11,
        MISSION_SET_CURRENT = 41
    }

    -- private fields
    local _mavbuffer = ""
    local _mavresult = {}
    local _payload_len = 0
    local _mavdecodestate = 0 -- 0=looking for marker, 1=getting header,2=getting payload,3=getting crc
    PROTOCOL_MARKER_V1 = 0xFE
    HEADER_LEN_V1 = 6
    local _txseqid = 0

    -- AUTOGEN from MAVLink generator
    local _crc_extra = {}
    _crc_extra[75] = 0x9e
    _crc_extra[76] = 0x98
    _crc_extra[235] = 0xb3
    _crc_extra[73] = 0x26
    _crc_extra[11] = 0x59
    _crc_extra[41] = 0x1c
    
    local _messages = {}
    _messages[75] = { -- COMMAND_INT
        {"param1", "<f"}, {"param2", "<f"}, {"param3", "<f"}, {"param4", "<f"},
        {"x", "<i4"}, {"y", "<i4"}, {"z", "<f"}, {"command", "<I2"},
        {"target_system", "<B"}, {"target_component", "<B"}, {"frame", "<B"},
        {"current", "<B"}, {"autocontinue", "<B"}
    }
    _messages[76] = { -- COMMAND_LONG
        {"param1", "<f"}, {"param2", "<f"}, {"param3", "<f"}, {"param4", "<f"},
        {"param5", "<f"}, {"param6", "<f"}, {"param7", "<f"},
        {"command", "<I2"}, {"target_system", "<B"}, {"target_component", "<B"},
        {"confirmation", "<B"}
    }
    _messages[235] = { -- HIGH_LATENCY2
        {"timestamp", "<I4"}, {"latitude", "<i4"}, {"longitude", "<i4"},
        {"custom_mode", "<I2"}, {"altitude", "<i2"}, {"target_altitude", "<i2"},
        {"target_distance", "<I2"}, {"wp_num", "<I2"}, {"failure_flags", "<I2"},
        {"type", "<B"}, {"autopilot", "<B"}, {"heading", "<B"},
        {"target_heading", "<B"}, {"throttle", "<B"}, {"airspeed", "<B"},
        {"airspeed_sp", "<B"}, {"groundspeed", "<B"}, {"windspeed", "<B"},
        {"wind_heading", "<B"}, {"eph", "<B"}, {"epv", "<B"},
        {"temperature_air", "<b"}, {"climb_rate", "<b"}, {"battery", "<b"},
        {"custom0", "<B"}, -- should be <b (int8), but we're hacking this into a uint8 instead
        {"custom1", "<b"}, {"custom2", "<b"}
    }
    _messages[73] = { -- MISSION_ITEM_INT
        {"param1", "<f"}, {"param2", "<f"}, {"param3", "<f"}, {"param4", "<f"},
        {"x", "<i4"}, {"y", "<i4"}, {"z", "<f"}, {"seq", "<I2"},
        {"command", "<I2"}, {"target_system", "<B"}, {"target_component", "<B"},
        {"frame", "<B"}, {"current", "<B"}, {"autocontinue", "<B"}
    }
    _messages[11] = { -- SET_MODE
        { "custom_mode", "<I4" }, { "target_system", "<B" }, { "base_mode", "<B" },
    }
    _messages[41] = { -- MISSION_SET_CURRENT
        { "seq", "<I2" }, { "target_system", "<B" }, { "target_component", "<B" },
    }
    function self.getSeqID() return _txseqid end

    function self.generateCRC(buffer)
        -- generate the x25crc for a given buffer. Make sure to include crc_extra!
        local crc = 0xFFFF
        for i = 1, #buffer do
            local tmp = string.byte(buffer, i, i) ~ (crc & 0xFF)
            tmp = (tmp ~ (tmp << 4)) & 0xFF
            crc = (crc >> 8) ~ (tmp << 8) ~ (tmp << 3) ~ (tmp >> 4)
            crc = crc & 0xFFFF
        end
        return string.pack("<H", crc)
    end

    function self.parseMAVLink(byte)
        -- parse a new byte and see if we've got MAVLink message
        -- returns true if a packet was decoded, false otherwise
        _mavbuffer = _mavbuffer .. string.char(byte)

        -- check if this is a start of packet
        if _mavdecodestate == 0 and byte == PROTOCOL_MARKER_V1 then
            -- we have a packet start, discard the buffer before this byte
            _mavbuffer = string.char(byte)
            _mavdecodestate = 1
            return
        end

        -- if we have a full header, try parsing
        if #_mavbuffer == HEADER_LEN_V1 and _mavdecodestate == 1 then
            local read_marker = 1
            _, read_marker = string.unpack("<B", _mavbuffer, read_marker)
            _payload_len, read_marker = string.unpack("<B", _mavbuffer,
                                                      read_marker) -- payload is always the second byte
            -- fetch seq/sysid/compid
            _mavresult.seq, read_marker =
                string.unpack("<B", _mavbuffer, read_marker)
            _mavresult.sysid, read_marker =
                string.unpack("<B", _mavbuffer, read_marker)
            _mavresult.compid, read_marker =
                string.unpack("<B", _mavbuffer, read_marker)
            -- fetch the message id
            _mavresult.msgid, _ = string.unpack("<B", _mavbuffer, read_marker)

            _mavdecodestate = 2
            return
        end

        -- get payload
        if _mavdecodestate == 2 and #_mavbuffer ==
            (_payload_len + HEADER_LEN_V1) then
            _mavdecodestate = 3
            _mavresult.payload = string.sub(_mavbuffer, HEADER_LEN_V1 + 1)
            return
        end

        -- get crc, then process if CRC ok
        if _mavdecodestate == 3 and #_mavbuffer ==
            (_payload_len + HEADER_LEN_V1 + 2) then
            _mavdecodestate = 0
            _mavresult.crc = string.sub(_mavbuffer, -2, -1)

            local message_map = _messages[_mavresult.msgid]
            if not message_map then
                -- we don't know how to decode this message, bail on it
                _mavbuffer = ""
                return true
            end

            -- check CRC, if message defined
            local crc_extra_msg = _crc_extra[_mavresult.msgid]
            if crc_extra_msg ~= nil then
                local calccrc = self.generateCRC(
                                    string.sub(_mavbuffer, 2, -3) ..
                                        string.char(crc_extra_msg))
                if _mavresult.crc ~= calccrc then
                    gcs:send_text(3,
                                  "Rockblock: Bad Mavlink CRC: " ..
                                      self.bytesToString(_mavbuffer, -2, -1) ..
                                      ", " .. self.bytesToString(calccrc, 1, 2))
                    _mavbuffer = ""
                    return
                end
            end

            -- map all the fields out
            local offset = 1
            for _, v in ipairs(message_map) do
                if v[3] then
                    _mavresult[v[1]] = {}
                    for j = 1, v[3] do
                        _mavresult[v[1]][j], offset = string.unpack(v[2],
                                                                    _mavresult.payload,
                                                                    offset)
                    end
                else
                    _mavresult[v[1]], offset = string.unpack(v[2],
                                                             _mavresult.payload,
                                                             offset)
                end
            end
            -- only process COMMAND_LONG and COMMAND_INT and  MISSION_ITEM_INT messages
            if _mavresult.msgid == self.MISSION_ITEM_INT then
                -- goto somewhere (guided mode target)
                if _mavresult.command == 16 then -- MAV_CMD_NAV_WAYPOINT
                    local loc = Location()
                    loc:lat(_mavresult.x)
                    loc:lng(_mavresult.y)
                    loc:alt(_mavresult.z * 100)
                    if _mavresult.frame == 10 then -- MAV_FRAME_GLOBAL_TERRAIN_ALT
                        loc:terrain_alt(true)
                        loc:relative_alt(true)
                    elseif _mavresult.frame == 3 then -- MAV_FRAME_GLOBAL_RELATIVE_ALT
                        loc:relative_alt(true)
                    end
                    vehicle:set_target_location(loc)
                end
            elseif _mavresult.msgid == self.SET_MODE then
                vehicle:set_mode(_mavresult.custom_mode)
            elseif _mavresult.msgid == self.COMMAND_LONG then
                --- need to do a little conversion here. Taken from convert_COMMAND_LONG_to_COMMAND_INT()
                local command_long_stores_location = 0
                if (_mavresult.command == 179 or       -- MAV_CMD_DO_SET_HOME
                    _mavresult.command ==  201 or      -- MAV_CMD_DO_SET_ROI
                    _mavresult.command ==  195 or      -- MAV_CMD_DO_SET_ROI_LOCATION
                    _mavresult.command ==  192 or      -- MAV_CMD_DO_REPOSITION
                    _mavresult.command ==  43003) then -- MAV_CMD_EXTERNAL_POSITION_ESTIMATE
                    command_long_stores_location = 1
                end
                local int_frame_conv = 0       -- MAV_FRAME_GLOBAL
                if (_mavresult.command ==  195) then       -- MAV_CMD_DO_SET_ROI_LOCATION
                    int_frame_conv = 3         -- MAV_FRAME_GLOBAL_RELATIVE_ALT
                elseif (_mavresult.command ==  179) then   -- MAV_CMD_DO_SET_HOME
                    int_frame_conv = 0        -- MAV_FRAME_GLOBAL
                elseif (_mavresult.command ==  201) then   -- MAV_CMD_DO_SET_ROI
                    int_frame_conv = 3        -- MAV_FRAME_GLOBAL_RELATIVE_ALT
                elseif (_mavresult.command ==  42006) then   -- MAV_CMD_FIXED_MAG_CAL_YAW
                    int_frame_conv = 3        -- MAV_FRAME_GLOBAL_RELATIVE_ALT
                end
                local int_x = _mavresult.param5
                local int_y = _mavresult.param6
                if isNaN(int_x) then
                    int_x = 0
                end
                if isNaN(int_y) then
                    int_y = 0
                end
                if command_long_stores_location == 1 then
                    int_x = 1E7 * int_x
                    int_y = 1E7 * int_y
                end
                gcs:run_command_int(_mavresult.command, { p1 = _mavresult.param1,
                                                          p2 = _mavresult.param2,
                                                          p3 = _mavresult.param3,
                                                          p4 = _mavresult.param4,
                                                          x = int_x,
                                                          y = int_y,
                                                          z = _mavresult.param7,
                                                          int_frame = int_frame_conv,
                                                          current = 0,
                                                          autocontinue = 0 })
            elseif _mavresult.msgid == self.COMMAND_INT then
                gcs:run_command_int(_mavresult.command, { p1 = _mavresult.param1,
                                                          p2 = _mavresult.param2,
                                                          p3 = _mavresult.param3,
                                                          p4 = _mavresult.param4,
                                                          x = _mavresult.x,
                                                          y = _mavresult.y,
                                                          z = _mavresult.z,
                                                          frame = _mavresult.frame })
            elseif _mavresult.msgid == self.MISSION_SET_CURRENT then
                mission:set_current_cmd(_mavresult.seq)
            end
            _mavbuffer = ""
            return true
        end

        -- packet too big ... start again
        if #_mavbuffer > 263 then 
            _mavbuffer = ""
            _mavdecodestate = 0
        end
        return false
    end

    function self.bytesToString(buf, start, stop)
        local ret = ""
        for idx = start, stop do
            ret = ret .. string.format("0x%x ", buf:byte(idx), 1, -1) .. " "
        end
        return ret
    end

    function self.createMAVLink(message, msgid)
        -- generate a mavlink message (V1 only)

        -- create the payload
        local message_map = _messages[msgid]
        if not message_map then
            -- we don't know how to encode this message, bail on it
            gcs:send_text(3, "Rockblock: Unknown MAVLink message " .. msgid)
            return nil
        end

        local packString = "<"
        local packedTable = {}
        local packedIndex = 1
        for i, v in ipairs(message_map) do
            if v[3] then
                packString = (packString ..
                                 string.rep(string.sub(v[2], 2), v[3]))
                for j = 1, v[3] do
                    packedTable[packedIndex] = message[message_map[i][1]][j]
                    packedIndex = packedIndex + 1
                end
            else
                packString = (packString .. string.sub(v[2], 2))
                packedTable[packedIndex] = message[message_map[i][1]]
                packedIndex = packedIndex + 1
            end
        end

        local payload = string.pack(packString, table.unpack(packedTable))

        -- create the header. Assume componentid of 1
        local header = string.pack('<BBBBBB', PROTOCOL_MARKER_V1, #payload,
                                   _txseqid, param:get('MAV_SYSID'), 1,
                                   msgid)

        -- generate the CRC
        local crc_extra_msg = _crc_extra[msgid]
        local crc = self.generateCRC(string.sub(header, 2) .. payload ..
                                         string.char(crc_extra_msg))

        -- iterate sequence id
        _txseqid = (_txseqid + 1) % 255

        return header .. payload .. crc
    end

    -- return the instance
    return self
end


--[[
Lua Object for managing the RockBlock modem
--]]
local function RockblockModem()
    -- public fields
    local self = {
        is_transmitting = false,
        first_sucessful_mailbox_check = false,
        time_last_tx = millis():tofloat() * 0.001,
        last_modem_status_check = 0,
        modem_detected = false,
        in_read_cycle = false,
        rx_len = 0
    }

    -- private fields
    local _AT_query = "AT+CGMM\r"
    local _AT_mailbox_check = "AT+SBDIXA\r"
    local _AT_load_tx_buffer = "AT+SBDWB="
    local _AT_read_rx_buffer = "AT+SBDRB\r"
    local _AT_clear_tx_buffer = "AT+SBDD0\r"
    local _AT_clear_rx_buffer = "AT+SBDD1\r"
    
    local _modem_history = {}
    local _modem_to_send = {}
    local _str_received = ""
    
    -- Get any incoming data
    function self.rxdata(inchar)
        read = string.char(inchar)
        local maybepkt = nil
        if _modem_history[#_modem_history] == 'AT+SBDRB' and self.in_read_cycle == false and self.rx_len > 0 then
            -- read buffer may include /r and /n, so need a special cycle to capture all up to the self.rx_len
            self.in_read_cycle = true
            _str_received = _str_received .. read
        elseif self.in_read_cycle and #_str_received == self.rx_len + 3 then
            -- get last byte in read cycle
            _str_received = _str_received .. read
            self.in_read_cycle = false
            self.rx_len = 0
            table.insert(_modem_history, _str_received)
            _str_received = ""
            if RCK_DEBUG:get() == 1 then
                gcs:send_text(3, "Rockblock: Modem rx msg: " ..
                                  self.nicestring(_modem_history[#_modem_history]))
            end
        elseif (read == '\r' or read == '\n') and not self.in_read_cycle then
            if #_str_received > 0 then
                table.insert(_modem_history, _str_received)
                if RCK_DEBUG:get() == 1 then
                    gcs:send_text(3, "Rockblock: Modem response: " ..
                                      self.nicestring(_modem_history[#_modem_history]))
                end
                maybepkt = self.check_cmd_return()
            end
            _str_received = ""
        else
            _str_received = _str_received .. read
        end
        return maybepkt
    end

    -- Parse any incoming text from the modem
    -- returns any payload data, nil otherwise
    function self.check_cmd_return()
        -- modem detection (response to AT_query)
        if #_modem_history == 3 and _modem_history[1] == 'AT+CGMM' and
            _modem_history[3] == 'OK' then
            gcs:send_text(3, "Rockblock modem detected - " ..
                              self.nicestring(_modem_history[2]))
            _modem_history = nil
            _modem_history = {}
            self.modem_detected = true
            table.insert(_modem_to_send, _AT_clear_rx_buffer)
            table.insert(_modem_to_send, _AT_clear_tx_buffer)

            -- enable high latency mode, if desired
            if RCK_FORCEHL:get() == 1 then gcs:enable_high_latency_connections(true) end
            return nil
        end

        if self.modem_detected then
            -- TX Buffer clear (response to AT_query)
            if #_modem_history >= 3 and _modem_history[#_modem_history - 2] ==
                'AT+SBDD0' and _modem_history[#_modem_history] == 'OK' then
                if RCK_DEBUG:get() == 1 then
                    gcs:send_text(3, "Rockblock cleared modem TX buffer")
                end
                _modem_history = nil
                _modem_history = {}
                return nil
            end

            -- RX buffer clear (response to AT_query)
            if #_modem_history >= 3 and _modem_history[#_modem_history - 2] ==
                'AT+SBDD1' and _modem_history[#_modem_history] == 'OK' then
                if RCK_DEBUG:get() == 1 then
                    gcs:send_text(3, "Rockblock cleared modem RX buffer")
                end
                _modem_history = nil
                _modem_history = {}
                return nil
            end

            -- Tx buffer loaded (response to AT command)
            if #_modem_history >= 4 and
                string.find(_modem_history[#_modem_history - 3], _AT_load_tx_buffer, 1,
                            true) and _modem_history[#_modem_history] == 'OK' then
                if RCK_DEBUG:get() == 1 then
                    gcs:send_text(3, "Rockblock loaded packet into tx buffer, " ..
                                      tostring(_modem_history[#_modem_history - 3]))
                end
                if _modem_history[#_modem_history - 1] ~= '0' then
                    gcs:send_text(3, "Rockblock Error loading packet into buffer")
                end
                _modem_history = nil
                _modem_history = {}
                return nil
            end

            -- Got received data (response to AT command)
            if _modem_history[#_modem_history - 2] == 'AT+SBDRB' and
                _modem_history[#_modem_history] == 'OK' then
                -- Message format is { 2 byte msg length} + {message} + {2 byte checksum}
                local totallen = #(_modem_history[#_modem_history - 1])
                local len = string.unpack(">i2", _modem_history[#_modem_history - 1])
                local msg = string.sub(_modem_history[#_modem_history - 1], 3,
                                       totallen - 2)
                local checksumrx = string.sub(_modem_history[#_modem_history - 1],
                                              totallen - 1, totallen)
                local highByte, lowByte = self.checksum(msg)

                -- check that received message is OK, then send to MAVLink processor
                if len ~= #msg then
                    gcs:send_text(3,
                                  "Rockblock: Bad RX message length " .. tostring(len) ..
                                      " vs actual " .. tostring(#msg))
                elseif checksumrx ~= (tostring(highByte) .. tostring(lowByte)) then
                    gcs:send_text(3,
                                  "Rockblock: Bad RX CRC " .. checksumrx .. " vs " ..
                                      tostring(highByte) .. tostring(lowByte))
                else
                    if RCK_DEBUG:get() == 1 then
                        gcs:send_text(3, "totmsg=" ..
                                          self.nicestring(
                                              _modem_history[#_modem_history - 1]))
                    end
                end

                _modem_history = nil
                _modem_history = {}
                return msg
            end

            -- Mailbox check  (response to AT command) (can be a fail or success)
            if _modem_history[#_modem_history - 2] == 'AT+SBDIXA' and
                _modem_history[#_modem_history] == 'OK' then

                -- Parse response (comma and : delimited, trimming whitespace)
                local statusReponse = {}
                for w in _modem_history[#_modem_history - 1]:gmatch("[^,:]+") do
                    table.insert(statusReponse, w)
                end

                if #statusReponse == 7 then
                    if tonumber(statusReponse[2]) < 5 then
                        gcs:send_text(3, "Rockblock Packet Transmitted successfully")
                        -- check for rx packet
                        if tonumber(statusReponse[4]) == 1 and self.first_sucessful_mailbox_check then
                            self.rx_len = tonumber(statusReponse[6])
                            gcs:send_text(3, "Rockblock Packet Received of len " .. tostring(self.rx_len))
                            -- read messages, if not first mailbox check
                            table.insert(_modem_to_send, _AT_read_rx_buffer)
                        elseif RCK_DEBUG:get() == 1 then
                            gcs:send_text(3, "Rockblock: No message to receive")
                        end
                        self.first_sucessful_mailbox_check = true
                    elseif tonumber(statusReponse[2]) == 32 then
                        gcs:send_text(3, "Rockblock Error: No network service")
                    else
                        gcs:send_text(3, "Rockblock Error: Unable to send")
                    end
                end
                _modem_history = nil
                _modem_history = {}
                self.is_transmitting = false
                return nil
            end
        end
    end

    function self.checksum(bytes)
        -- Checksum calculation for SBDWB
        -- The checksum is the least significant 2-bytes of the summation of the entire SBD message
        local SUM = 0

        for idx = 1, #bytes do SUM = SUM + bytes:byte(idx) end

        local SUM_H = (SUM & 0xFF << 8) >> 8
        local SUM_L = SUM & 0xFF

        if RCK_DEBUG:get() == 1 then
            gcs:send_text(3, "Rockblock: Modem CRC: " .. string.char(SUM_H) .. " " ..
                              string.char(SUM_L))
        end

        return string.char(SUM_H), string.char(SUM_L)
    end

    function self.nicestring(instr)
        -- make any strings printable to GCS (constrain to ASCII range)
        local retstr = ""
        for i = 1, #instr do
            local c = string.byte(instr:sub(i, i))
            if c < 0x20 or c > 0x7E then c = 0x5F end
            retstr = retstr .. string.char(c)
        end

        return tostring(retstr)
    end
    
    function self.checkmodem()
        --- send detect command to modem every 10 sec if not detected
        if (millis():tofloat() * 0.001) - self.last_modem_status_check > 10 then
            table.insert(_modem_to_send, _AT_query)
            self.last_modem_status_check = millis():tofloat() * 0.001
        end
    end
    
    function self.getnextcommandtosend()
        --- get the next string to send to the modem
        if #_modem_to_send == 0 then
            return nil
        end
        
        local ret = _modem_to_send[1]
        if RCK_DEBUG:get() == 1 then
            gcs:send_text(3, "Rockblock: Sent to modem: " .. self.nicestring(_modem_to_send[1]))
        end
        table.remove(_modem_to_send, 1)
        
        return ret
    end
    
    function self.loadtxbuffer(pkt)
        -- Load the transmit buffer of the modem
        -- Send as binary data, plus checksum
        local highByte, lowByte = self.checksum(pkt)
        table.insert(_modem_to_send, _AT_load_tx_buffer .. tostring(#pkt) .. '\r')
        table.insert(_modem_to_send, pkt)
        table.insert(_modem_to_send, highByte .. lowByte .. "\r")

        self.time_last_tx = millis():tofloat() * 0.001
        if RCK_ENABLE:get()  == 1 then
            table.insert(_modem_to_send, _AT_mailbox_check)
            self.is_transmitting = true
        end
    end
    
    -- return the instance
    return self
end

-- Transmitted HIGH_LATENCY2 packet
local hl2 = {}
hl2.timestamp = 0
hl2.latitude = 0
hl2.longitude = 0
hl2.custom_mode = 0
hl2.altitude = 0
hl2.target_altitude = 0
hl2.target_distance = 0
hl2.wp_num = 0
hl2.failure_flags = 0
hl2.type = gcs:frame_type()
hl2.autopilot = 3 -- MAV_AUTOPILOT_ARDUPILOTMEGA
hl2.heading = 0
hl2.target_heading = 0
hl2.throttle = 0
hl2.airspeed = 0
hl2.airspeed_sp = 0
hl2.groundspeed = 0
hl2.windspeed = 0
hl2.wind_heading = 0
hl2.eph = 0
hl2.epv = 0
hl2.temperature_air = 0
hl2.climb_rate = 0
hl2.battery = 0
hl2.custom0 = 1 -- MAV_MODE_FLAG_CUSTOM_MODE_ENABLED
hl2.custom1 = 0
hl2.custom2 = 0

function wrap_360(angle)
    local res = angle % 360
    if res < 0 then res = res + 360 end
    return res
end

-- Define the MAVLink processor
local mavlink = MAVLinkProcessor()

-- Define the RockBlock interface
local rockblock = RockblockModem()

function HLSatcom()
    -- read in any bytes from rockblock and form into received commands
    local n_bytes = port:available()
    while n_bytes > 0 do
        read = port:read()
        pkt = rockblock.rxdata(read)
        n_bytes = n_bytes - 1
        -- we've got a MAVLink message from the GCS, parse
        if pkt ~= nil then
            for idx = 1, #pkt do
                mavlink.parseMAVLink(pkt:byte(idx))
            end
        end
    end

    -- write out commands from send list. one cmd per loop
    local cmd = rockblock.getnextcommandtosend()
    if cmd ~= nil then
        for idx = 1, #cmd do
            port:write(cmd:byte(idx))
        end
    end

    --- check if modem is there
    if not rockblock.modem_detected then
        gcs:send_text(3, "Rockblock: Trying to detect modem")
        rockblock.checkmodem()
    end

    --- check if GCS telemetry has been lost for RCK_TIMEOUT sec (if param enabled)
    if RCK_FORCEHL:get() == 2 then
        -- link lost time = boot time - GCS last seen time
        link_lost_for = (millis()- gcs:last_seen()):toint()
        -- gcs:last_seen() is set to millis() during boot (on plane). 0 on rover/copter
        -- So if it's less than 10000 assume no GCS packet received since boot
        if link_lost_for > (RCK_TIMEOUT:get() * 1000) and not gcs:get_high_latency_status() and gcs:last_seen() > 10000 then
            gcs:enable_high_latency_connections(true)
        elseif link_lost_for < (RCK_TIMEOUT:get() * 1000) and gcs:get_high_latency_status() then
            gcs:enable_high_latency_connections(false)
        end
    end
    
    -- send HL2 packet every RCK_PERIOD sec, if not aleady in a mailbox check
    if rockblock.modem_detected and gcs:get_high_latency_status() and
        (millis():tofloat() * 0.001) - rockblock.time_last_tx > RCK_PERIOD:get() and not rockblock.is_transmitting then

        -- update HL2 packet
        hl2.timestamp = millis():tofloat()
        local position = ahrs:get_location()
        local wind = ahrs:wind_estimate()

        if position then
            hl2.latitude = tonumber(position:lat())
            hl2.longitude = tonumber(position:lng())
            hl2.altitude = math.floor(tonumber(position:alt()) * 0.01)
        end
        if wind then
            wind_xy = Vector2f()
            wind_xy:x(wind:x())
            wind_xy:y(wind:y())
            hl2.windspeed = math.abs(math.floor(wind_xy:length() * 5))
            hl2.wind_heading = math.floor(wrap_360(wind_xy:angle()) / 2)
        end
        hl2.custom_mode = vehicle:get_mode()

        if vehicle:get_wp_distance_m() ~= nil then
            hl2.target_distance = math.floor(vehicle:get_wp_distance_m() / 10)
        end
        if mission:get_current_nav_index() ~= nil then
            hl2.wp_num = mission:get_current_nav_index()
        end
        if vehicle:get_wp_bearing_deg() ~= nil then
            hl2.target_heading = math.floor(wrap_360(
                                                vehicle:get_wp_bearing_deg()) /
                                                2)
        end

        -- failure flags
        hl2.failure_flags = 0
        if not ahrs:healthy() then
            hl2.failure_flags = hl2.failure_flags + 4096 -- HL_FAILURE_FLAG_ESTIMATOR
        end
        if battery:num_instances() > 0 and not battery:healthy(0) then
            hl2.failure_flags = hl2.failure_flags + 128 -- HL_FAILURE_FLAG_BATTERY
        end
        if gps:num_sensors() > 0 and gps:status(0) <= gps.NO_FIX then
            hl2.failure_flags = hl2.failure_flags + 1 -- HL_FAILURE_FLAG_GPS
        end
        if (FWVersion:type() == 2 or FWVersion:type() == 3) and terrain:status() ==
            terrain.TerrainStatusUnhealthy then
            -- only for copter and plane
            hl2.failure_flags = hl2.failure_flags + 64 -- HL_FAILURE_FLAG_TERRAIN
        end
        if not rc:has_valid_input() then
            hl2.failure_flags = hl2.failure_flags + 256 -- HL_FAILURE_FLAG_RC_RECEIVER
        end

        hl2.heading = math.floor(wrap_360(math.deg(ahrs:get_yaw_rad())) / 2)
        hl2.throttle = math.floor(gcs:get_hud_throttle())
        if ahrs:airspeed_EAS() ~= nil then
            hl2.airspeed = math.abs(math.floor(ahrs:airspeed_EAS() * 5))
        end
        -- hl2.airspeed_sp = 0
        hl2.groundspeed = math.abs(math.floor(
                                       ahrs:groundspeed_vector():length() * 5))

        hl2.temperature_air = math.floor(baro:get_external_temperature())
        
        if battery:num_instances() > 0 and battery:capacity_remaining_pct(0) ~= nil then
            hl2.battery = battery:capacity_remaining_pct(0)
        else
            hl2.battery = 0
        end

        -- just sending armed state here for simplicity. Flight mode is in the custom_mode field
        if arming:is_armed() then
            hl2.custom0 = 129 -- MAV_MODE_FLAG_SAFETY_ARMED + MAV_MODE_FLAG_CUSTOM_MODE_ENABLED
        else
            hl2.custom0 = 1 -- MAV_MODE_FLAG_CUSTOM_MODE_ENABLED
        end

        local newpkt = mavlink.createMAVLink(hl2, mavlink.HIGH_LATENCY2)
        if #newpkt > 50 then
            gcs:send_text(3, "Rockblock: Tx packet > 50 bytes: " .. tostring(#newpkt))
        end

        -- send packet
        rockblock.loadtxbuffer(newpkt)
        
    end
end

-- wrapper around HLSatcom(). This calls HLSatcom() and if HLSatcom faults
-- then an error is displayed, but the script is not stopped
function protected_wrapper()
    local success, err = pcall(HLSatcom)
    if not success then
        gcs:send_text(3, "Internal Error: " .. err)
        -- when we fault we run the HLSatcom function again after 1s, slowing it
        -- down a bit so we don't flood the console with errors
        return protected_wrapper, 1000
    end
    return protected_wrapper, math.floor(1000 / 10)
end

-- start running HLSatcom loop
return protected_wrapper()
