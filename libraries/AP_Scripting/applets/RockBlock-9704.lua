--[[
Lua script to send a recieve very basic MAVLink telemetry over a
Rockblock 9704 SBD satellite modem
Requires https://github.com/stephendade/rockblock2mav at the GCS end

Setup:
This script requires 1 serial port, 1 scripting serial port, 1 relay output and 1 GPIO input

ftp put /home/stephen/Documents/UAVCode/ardupilot/libraries/AP_Scripting/applets/RockBlock-9704.lua APM/Scripts/RockBlock-9704.lua

Usage:
Use the MAVLink High Latency Control ("link hl on|off" in MAVProxy) to control
whether to send/receive or not (or use "force_hl_enable")

Written by Stephen Dade (stephen_dade@hotmail.com)


]]--

-- number of millsec that GCS telemetry has been lost for
local link_lost_for = 0

-- Params
local PARAM_TABLE_KEY = 108
local PARAM_TABLE_PREFIX = "RK9_"

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

 -- setup RK9 specific parameters
assert(param:add_table(PARAM_TABLE_KEY, PARAM_TABLE_PREFIX, 9), 'could not add param table')
--[[
  // @Param: RK9_FORCEHL
  // @DisplayName: Force enable High Latency mode
  // @Description: Automatically enables High Latency mode if not already enabled
  // @Values: 0:Disabled,1:Enabled,2:Enabled on telemetry loss
  // @User: Standard
--]]
RK9_FORCEHL     = bind_add_param('FORCEHL', 1, 0)

--[[
  // @Param: RK9_PERIOD
  // @DisplayName: Update rate
  // @Description: When in High Latency mode, send Rockblock updates every N seconds
  // @Range: 0 600
  // @Units: s
  // @User: Standard
--]]
RK9_PERIOD     = bind_add_param('PERIOD', 2, 30)

--[[
  // @Param: RK9_DEBUG
  // @DisplayName: Display Rockblock debugging text
  // @Description: Sends Rockblock debug text to GCS via statustexts
  // @Values: 0:Disabled,1:Enabled
  // @User: Standard
--]]
RK9_DEBUG     = bind_add_param('DEBUG', 3, 0)

--[[
  // @Param: RK9_ENABLE
  // @DisplayName: Enable Message transmission
  // @Description: Enables the Rockblock sending and recieving
  // @Values: 0:Disabled,1:Enabled
  // @User: Standard
--]]
RK9_ENABLE     = bind_add_param('ENABLE', 4, 1)

--[[
  // @Param: RK9_TIMEOUT
  // @DisplayName: GCS timeout to start sendin Rockblock messages
  // @Description: If RK9_FORCEHL=2, this is the number of seconds of GCS timeout until High Latency mode is auto-enabled
  // @Range: 0 600
  // @Units: s
  // @User: Standard
--]]
RK9_TIMEOUT     = bind_add_param('TIMEOUT', 5, 5)

--[[
  // @Param: RK9_SERPORT
  // @DisplayName: Rockblock Serial Port
  // @Description: Serial port number to which the Rockblock is connected.This is the index of the SERIALn_ ports that are set to 28 for "scripting"
  // @Range: 0 8
  // @User: Standard
--]]
RK9_SERPORT     = bind_add_param('SERPORT', 6, 0)

--[[
  // @Param: RK9_SCRPORT
  // @DisplayName: Rockblock Scripting Serial Port
  // @Description: Scripting Serial port number to which the Rockblock is connected for HL2 messages This is the index of the SCR_SDEV ports that are set to 2 for "MavlinkHL"
  // @Range: 0 8
  // @User: Standard
--]]
RK9_SCRPORT     = bind_add_param('SCRPORT', 7, 0)

--[[
  // @Param: RK9_RELAY
  // @DisplayName: Rockblock Power Relay
  // @Description: RELAYn output to control Rockblock power. This connects to I_EN on the Rockblock header.
  // @Range: 1 8
  // @User: Standard
--]]
RK9_RELAY     = bind_add_param('RELAY', 8, 1)

--[[
  // @Param: RK9_BOOTED
  // @DisplayName: Rockblock booted feedback
  // @Description: SERVOn GPIO channel that reads the Rockblock booted state. This connects to I_BTD on the Rockblock header. Requires SERVON_FUNCTION=-1
  // @Range: 50 110
  // @User: Standard
--]]
RK9_BOOTED     = bind_add_param('BOOTED', 9, 52)

local pinboot = RK9_BOOTED:get()
if pinboot then
    gpio:pinMode(math.floor(pinboot), 0) -- set AUX 3 (servo11) to input I_BTD SERVO11_FUNCTION=-1
end

-- Modem States
local ModemState = {
    POWER0 = 0, --not powered
    POWER1 = 1,
    BOOTED1 = 3,
    BOOTED2 = 4,
    BOOTED3 = 5,
    API_CONFIGURED = 6,
    SIM_CONFIGURED = 7,
    OPERATIONAL_CONFIGURED = 8,
    TOPIC_RECIEVED = 9
}

-- The delay between loops
local loop_delay_ms = 200

local base64 = 'ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/'
-- Create lookup table for bit patterns to base64 chars
local base64_bit_to_char = {}
for i = 0, 63 do
    local c = base64:sub(i + 1, i + 1)
    local pattern = ''
    local val = i
    for j = 5, 0, -1 do
        pattern = pattern .. (val >= 2^j and '1' or '0')
        if val >= 2^j then val = val - 2^j end
    end
    base64_bit_to_char[pattern] = c
end

-- Create inverse lookup table for base64 chars to bit patterns
local base64_char_to_bits = {}
for pattern, char in pairs(base64_bit_to_char) do
    base64_char_to_bits[char] = pattern
end

local function base64_encode(input)
    return ((input:gsub('.', function(x)
        local r, b_char = '', x:byte()
        for i = 8, 1, -1 do r = r .. (b_char % 2 ^ i - b_char % 2 ^ (i - 1) > 0 and '1' or '0') end
        return r
    end) .. '0000'):gsub('%d%d%d?%d?%d?%d?', function(x)
        if (#x < 6) then return '' end
        return base64_bit_to_char[x:sub(1,6)]
    end) .. ({ '', '==', '=' })[#input % 3 + 1])
end

local function base64_decode(input)
    -- Strip invalid characters and padding
    input = input:gsub('[^'..base64..'=]', '')
    input = input:gsub('=', '')

    -- Convert base64 chars to bit sequence
    local bits = input:gsub('.', function(x)
        return base64_char_to_bits[x] or ''
    end)

    -- Convert 8-bit sequences to binary string
    local result = ""
    for i = 1, #bits, 8 do
        local byte = bits:sub(i, i+7)
        -- Pad with zeros if needed
        byte = byte .. string.rep('0', 8 - #byte)
        local value = 0
        for j = 1, 8 do
            value = value * 2 + tonumber(byte:sub(j,j))
        end
        result = result .. string.char(value)
    end
    return result
end

--[[
Lua Object for managing the RockBlock modem
--]]
local function RockblockModem9704()
    -- public fields
    local self = {
        state = ModemState.POWER0,
        raw_topic_id = nil,
        message_id = 0,
        request_reference = 0,
        cur_message = nil,
        rxbuffer = "",
        txbuffer = nil,
        txbufferlen = 0,
        bars = 0,
        port = nil,
        scr_port = nil,
        last_status_check = 0,
        status_interval = 10000, -- 10 seconds
        last_mavlink_send = 0,
        scr_string = ""
    }
    
    -- Encode message with CRC
    function self.encode_message(message)
        local crc = crc_xmodem(message)
        local crc_bytes = string.char((crc >> 8) & 0xFF) .. string.char(crc & 0xFF)
        local result = message .. crc_bytes
        local encoded = base64_encode(result)
        return encoded
    end

    -- Decode message and verify CRC
    function self.decode_message(encoded_message)
        local decoded = base64_decode(encoded_message)
        local msg_len = #decoded
        local crc_received = string.sub(decoded, msg_len-1)
        local message_data = string.sub(decoded, 1, msg_len-2)
        local crc_calculated = crc_xmodem(message_data)
        local crc_calculated_bytes = string.char((crc_calculated >> 8) & 0xFF) .. 
                                     string.char(crc_calculated & 0xFF)
        -- print out the message data and fields
        if RK9_DEBUG:get() == 1 then
            gcs:send_text(6, string.format("Rockblock: Received CRC: %02X %02X", crc_received:byte(1), crc_received:byte(2)))
            gcs:send_text(6, string.format("Rockblock: Calculated CRC: %02X %02X", crc_calculated_bytes:byte(1), crc_calculated_bytes:byte(2)))
        end
        return message_data, crc_received, crc_calculated_bytes
    end


    -- Send GET command to the modem
    function self.send_get_command(command)
        self.cur_message = command
        local command_str = string.format("GET %s {}\r", command)
        if RK9_DEBUG:get() == 1 then
            gcs:send_text(6, "Rockblock: TX: " .. command_str:gsub("\r", ""))
        end
        if self.port then
            self.port:writestring(command_str)
        end
    end

    -- Send PUT command to the modem
    function self.send_put_command(command, options)
        local command_str = string.format("PUT %s {%s}\r", command, options)
        if RK9_DEBUG:get() == 1 then
            gcs:send_text(6, "Rockblock: TX: " .. command_str:gsub("\r", ""))
        end
        self.cur_message = command
        if self.port then
            self.port:writestring(command_str)
        end
    end

    -- Process provisioning message
    function self.process_provisioning_message(message, target_name)
        -- Find the entire object containing the target_name
        local object_str = message:match('{[^}]-"topic_name"%s*:%s*"' .. target_name .. '"[^}]-}')
        if not object_str then
            return nil
        end

        -- Extract topic_id from that object
        local topic_id = object_str:match('"topic_id"%s*:%s*(%d+)')
        return topic_id and tonumber(topic_id) or nil
    end

    -- Process JSON response from the modem
    -- This is a simplified JSON parser for ArduPilot Lua
    function self.process_json(json_str, target)
        local result = {}
        
        -- Extract key fields using pattern matching
        result.active_version_major = tonumber(string.match(json_str, "\"major\":([%d]+)") or "0")
        result.constellation_visible = string.match(json_str, "\"constellation_visible\":([%a]+)") == "true"
        result.signal_bars = tonumber(string.match(json_str, "\"signal_bars\":([%d]+)") or "0")
        result.message_response = string.match(json_str, "\"message_response\":\"([%w_]+)\"")
        result.message_id = tonumber(string.match(json_str, "\"message_id\":([%d]+)") or "0")
        result.final_mo_status = string.match(json_str, "\"final_mo_status\":\"([%w_]+)\"")
        result.data = string.match(json_str, "\"data\":\"([%w+/=]+)\"")
        
        -- Find RAW topic id
        if target == "messageProvisioning" then
            local topic_id = self.process_provisioning_message(json_str, "RAW")
            if topic_id then
                result.raw_topic_id = topic_id
            else
                gcs:send_text(2, "Rockblock: Failed to find RAW topic ID. Check modem is activated.")
            end
        end
        
        return result
    end

    -- Process a line received from the modem
    function self.process_line(line)        
        -- Extract response code and target
        local response_code = tonumber(string.match(line, "^(%d+)") or "0")
        local target = string.match(line, "%d+ ([%w_]+)")
        local json_start = line:find("{")
        
        if not json_start then return end
        
        local json_str = line:sub(json_start)
        local json_response = self.process_json(json_str, target)
        
        -- Command acknowledgment
        if RK9_DEBUG:get() == 1 then
            gcs:send_text(6, "Rockblock: Command " .. target .. " acknowledged")
        end
        if target == self.cur_message then
            self.cur_message = nil
        end
        
        -- Handle state transitions
        if response_code == 400 then
            self.state = ModemState.BOOTED2
            return
        end
        
        if target == "apiVersion" and (response_code == 200 or response_code == 299 or response_code == 402) and 
           json_response.active_version_major >= 1 then
            self.state = ModemState.BOOTED3
        elseif target == "apiVersion" then
            self.state = ModemState.BOOTED2
        elseif target == "hwInfo" then
            self.state = ModemState.API_CONFIGURED
            gcs:send_text(5, "Rockblock: Modem detected")
        elseif target == "simConfig" then
            self.state = ModemState.SIM_CONFIGURED
        elseif target == "operationalState" then
            self.state = ModemState.OPERATIONAL_CONFIGURED
            gcs:send_text(5, "Rockblock: Modem configured")
        elseif target == "constellationState" then
            self.bars = json_response.signal_bars or 0
            if json_response.constellation_visible and RK9_DEBUG:get() == 1 then
                gcs:send_text(5, "Rockblock: Signal bars: " .. self.bars .. "/5")
            end
            self.last_status_check = millis()
        elseif target == "messageProvisioning" then
            if json_response.raw_topic_id then
                self.raw_topic_id = json_response.raw_topic_id
                if RK9_DEBUG:get() == 1 then
                    gcs:send_text(5, "Rockblock: RAW topic ID: " .. self.raw_topic_id)
                end
                self.state = ModemState.TOPIC_RECIEVED
            else
                gcs:send_text(2, "Rockblock: Failed to find RAW topic ID. Check modem is activated.")
            end
        elseif target == "messageOriginate" then
            if json_response.message_response == "message_accepted" then
                self.message_id = json_response.message_id
                
                -- Send the segment
                if self.txbuffer then
                    self.send_put_command("messageOriginateSegment",
                                    string.format("\"topic_id\":%d, \"message_id\":%d, \"segment_length\":%d, \"segment_start\":0, \"data\":\"%s\"",
                                                 self.raw_topic_id, self.message_id, self.txbufferlen, self.txbuffer))
                end
            end
        elseif target == "messageOriginateStatus" then
            if json_response.final_mo_status == "mo_ack_received" and json_response.message_id == self.message_id then
                gcs:send_text(5, "Rockblock: Message " .. self.message_id .. " sent successfully")
            elseif json_response.message_id == self.message_id then
                gcs:send_text(2, "Rockblock: Message " .. self.message_id .. " failed to send" .. 
                               " with status: " .. json_response.final_mo_status)
            end
            self.txbuffer = nil
        elseif target == "messageTerminateSegment" and response_code == 299 then
            if json_response.data then
                if RK9_DEBUG:get() == 1 then
                    gcs:send_text(6, "Rockblock: Data received: " .. json_response.data)
                end
                local message_data, crc_received, crc_calculated = self.decode_message(json_response.data)
                
                -- Check CRC
                local crc_ok = true
                for i = 1, #crc_received do
                    if crc_received:byte(i) ~= crc_calculated:byte(i) then
                        crc_ok = false
                        break
                    end
                end
                
                if not crc_ok then
                    gcs:send_text(2, "Rockblock: CRC mismatch in received message")
                else
                    self.scr_port:writestring(message_data)
                end
            end
        end
    end

    -- Send a message through the modem
    function self.send_message(message)
        if RK9_ENABLE:get() == 0 then
            if RK9_DEBUG:get() == 1 then
                gcs:send_text(6, "Rockblock: Rockblock sending disabled by RK9_ENABLE param")
            end
            return false
        end

        if not self.raw_topic_id then
            if RK9_DEBUG:get() == 1 then
                gcs:send_text(6, "Rockblock: No RAW topic ID available")
            end
            return false
        end
        
        if self.bars == 0 then
            if RK9_DEBUG:get() == 1 then
                gcs:send_text(6, "Rockblock: No signal bars available")
            end
            return false
        end
        
        if self.txbuffer then
            if RK9_DEBUG:get() == 1 then
                gcs:send_text(6, "Rockblock: Modem already processing a message")
            end
            return false
        end
        
        -- Send messageOriginate
        self.request_reference = self.request_reference + 1
        self.txbuffer = self.encode_message(message)
        -- note the buffer len is before the base64 encoding takes place
        self.txbufferlen = #message + 2  -- +2 for CRC
        self.send_put_command("messageOriginate", 
                        string.format("\"topic_id\":%d, \"message_length\":%d, \"request_reference\":%d", 
                                     self.raw_topic_id, self.txbufferlen, self.request_reference))

        return true
    end

    -- Read from the serial port and process data
    function self.modem_process()
        if not self.port then return end
        
        -- Read available data from rockblock
        local n_bytes = math.min(self.port:available():toint(), 70)
        local rxstring = self.port:readstring(n_bytes)

        -- Read available data from scripting serial port
        n_bytes = self.scr_port:available():toint()
        self.scr_string = self.scr_string .. self.scr_port:readstring(n_bytes)
        -- if there's more than 150 bytes in the buffer, just trim to the last 150 bytes
        if #self.scr_string > 150 then
            self.scr_string = self.scr_string:sub(-150)
        end

        -- Send pending messages if we're in HL mode and it's been RK9_PERIOD:get() since last send
        if #self.scr_string > 52 then
            local now = (millis():tofloat() * 0.001)
            if now - self.last_mavlink_send > RK9_PERIOD:get() and gcs:get_high_latency_status() then
                if self.state == ModemState.TOPIC_RECIEVED and self.bars > 0 and not self.txbuffer then
                    --iterate through the string to find 0xFD (MAVLink start of packet)
                    while #self.scr_string > 10 do
                        local fd_pos = self.scr_string:find(string.char(0xFD))
                        --ensure it's a HL2 packet by looking at the message ID (0xEB)
                        if fd_pos and #self.scr_string > (fd_pos + 12) and self.scr_string:byte(fd_pos + 7) == 0xEB then
                            local scr_string_aligned = self.scr_string:sub(fd_pos, fd_pos + 51)
                            if RK9_DEBUG:get() == 1 then
                                gcs:send_text(6, "Rockblock: Sending message of len " .. #scr_string_aligned)
                            end
                            self.send_message(scr_string_aligned)
                            self.scr_string = ""
                        elseif fd_pos then
                            -- remove up to and including the FD
                            self.scr_string = self.scr_string:sub(fd_pos + 1)
                        else
                            -- no FD found, clear the buffer
                            self.scr_string = ""
                        end
                    end
                elseif self.state ~= ModemState.TOPIC_RECIEVED then
                    gcs:send_text(2, "Rockblock: Cannot send message, modem not configured")
                elseif self.bars == 0 then
                    gcs:send_text(2, "Rockblock: Cannot send message, no signal bars")
                end
                self.last_mavlink_send = now
            end
        end
        
        -- Append to buffer
        self.rxbuffer = self.rxbuffer .. rxstring
        
        -- Process complete lines
        while true do
            local cr_pos = self.rxbuffer:find("\r")
            if not cr_pos then break end
            
            local line = self.rxbuffer:sub(1, cr_pos - 1)
            self.rxbuffer = self.rxbuffer:sub(cr_pos + 1)
            
            if #line > 0 then
                self.process_line(line)
            end
        end
    end

    -- Initialize the modem
    function self.initialize_modem()
        -- Find and configure the serial port
        local port = RK9_SERPORT:get()
        if port then
            self.port = serial:find_serial(math.floor(port))
            if not self.port then
                gcs:send_text(2, "Rockblock: Failed to find serial port")
                return false
            end
        else
            gcs:send_text(2, "Rockblock: RK9_SERPORT parameter not set")
            return false    
        end

        --Find and configure scripting serial ports for HL2 (43) messages
        port = RK9_SCRPORT:get()
        if port then
            self.scr_port = serial:find_simulated_device(43, math.floor(port))
            if not self.scr_port then
                gcs:send_text(2, 'Rockblock: could not find SCR_SDEV device')
                return false
            end
        else
            gcs:send_text(2, "Rockblock: RK9_SCRPORT parameter not set")
            return false    
        end
        
        -- Configure serial port
        self.port:begin(230400)
        self.port:set_flow_control(0)
        
        -- Start modem initialization sequence
        self.send_get_command("apiVersion")
        
        return true
    end

    function self.clear_buffers()
        -- Clear the RX buffer
        self.rxbuffer = ""
        
        -- Clear the TX buffer
        self.txbuffer = nil
        self.cur_message = nil

        -- drain the serial port rx buffers
        self.port:readstring(self.port:available():toint())
        self.scr_port:readstring(self.scr_port:available():toint())

        if RK9_DEBUG:get() == 1 then
            gcs:send_text(6, "Rockblock: Buffers cleared")
        end
    end

    -- return the instance
    return self
end

-- Define the RockBlock interface
local rockblock = RockblockModem9704()

-- Main update function called periodically by ArduPilot
function HLSatcom()
    -- boot the modem
    if rockblock.state == ModemState.POWER0 then
        -- set I_EN to high to power the modem, if not already powered
        local pin = RK9_RELAY:get()
        if pin then
            relay:on(math.floor(pin) - 1) -- turn on I_EN. Note the indexing change from 1 to 0
        end
        gcs:send_text(5, "Rockblock: Powering modem")
        rockblock.state = ModemState.POWER1
        loop_delay_ms = 1000 -- wait 1 seconds for modem to power up
        return
    end

    if rockblock.state == ModemState.POWER1 then
        local pin = RK9_BOOTED:get()
        if pin then
            if gpio:read(math.floor(pin)) then
                rockblock.state = ModemState.BOOTED1 -- modem is booted
                gcs:send_text(5, "Rockblock: Modem booted")
                return
            end
        end
    end

    -- enable high latency mode, if desired
    if RK9_FORCEHL:get() == 1 and not gcs:get_high_latency_status() then
        gcs:enable_high_latency_connections(true)
    end

    -- Initialize the port on first run
    if not rockblock.port or not rockblock.scr_port then
        if not rockblock.initialize_modem() then
            -- raise an error to pcall to stop the script
            gcs:send_text(2, "Rockblock: Initialization failed, check serial port settings")
            loop_delay_ms = 20000 -- try again in 20 seconds
            return
        end
        rockblock.clear_buffers()
    end

    --- check if GCS telemetry has been lost for RK9_TIMEOUT sec (if param enabled)
    if RK9_FORCEHL:get() == 2 then
        -- link lost time = boot time - GCS last seen time
        link_lost_for = (millis()- gcs:last_seen()):toint()
        -- gcs:last_seen() is set to millis() during boot (on plane). 0 on rover/copter
        -- So if it's less than 10000 assume no GCS packet received since boot
        if link_lost_for > (RK9_TIMEOUT:get() * 1000) and not gcs:get_high_latency_status() and gcs:last_seen() > 10000 then
            gcs:enable_high_latency_connections(true)
        elseif link_lost_for < (RK9_TIMEOUT:get() * 1000) and gcs:get_high_latency_status() then
            gcs:enable_high_latency_connections(false)
        end
    end

    -- Read and process data from the serial port
    rockblock.modem_process()

    -- Handle state machine
    if not rockblock.cur_message then  -- Only proceed if no command is pending
        if rockblock.state == ModemState.BOOTED2 then
            rockblock.send_put_command("apiVersion", "\"active_version\": {\"major\": 1, \"minor\": 6, \"patch\": 1}")
            loop_delay_ms = 200
        elseif rockblock.state == ModemState.BOOTED3 then
            rockblock.send_get_command("hwInfo")
        elseif rockblock.state == ModemState.API_CONFIGURED then
            rockblock.send_put_command("simConfig", "\"interface\": \"internal\"")
        elseif rockblock.state == ModemState.SIM_CONFIGURED then
            rockblock.send_put_command("operationalState", "\"state\": \"active\"")
        elseif rockblock.state == ModemState.OPERATIONAL_CONFIGURED then
            local now = millis()
            if now - rockblock.last_status_check > rockblock.status_interval then
                rockblock.last_status_check = now
                rockblock.send_get_command("messageProvisioning")
                loop_delay_ms = 100 -- once modem is configured, speed up the loop to 100ms
            end
        elseif rockblock.state == ModemState.TOPIC_RECIEVED and gcs:get_high_latency_status() then
            -- Periodic constellation status check
            local now = millis()
            if now - rockblock.last_status_check > rockblock.status_interval and rockblock.cur_message == nil then
                rockblock.last_status_check = now
                rockblock.send_get_command("constellationState")
            end
        end
    end

    -- Trying to find modem
    if rockblock.state == ModemState.BOOTED1 then
        gcs:send_text(5, "Rockblock: Trying to detect modem")
        -- Start modem initialization sequence
        rockblock.send_get_command("apiVersion")
        loop_delay_ms = 5000 -- try again in 5 seconds
    end
end

-- wrapper around HLSatcom(). This calls HLSatcom() and if HLSatcom faults
-- then an error is displayed, but the script is not stopped
function protected_wrapper()
    local success, err = pcall(HLSatcom)
    if not success then
        gcs:send_text(2, "Rockblock: Internal Error: " .. err)
        -- when we fault we run the HLSatcom function again after 1s, slowing it
        -- down a bit so we don't flood the console with errors
        return protected_wrapper, 1000
    end
    return protected_wrapper, math.floor(loop_delay_ms)
end

-- start running HLSatcom loop
return protected_wrapper()