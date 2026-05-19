-- Send throttle commands to a Traxxas Velineon ESC and receive ESC telem (RPM only)
-- Tested on a VXL-4s
-- This is a very basic implementation, its probably possible to get more data

-- I could not get half duplex to work, so the TX and RX pins are connected together, this means we see all our own traffic!
-- This is protected against by keeping a list of messages that are sent by us
local port = assert(serial:find_serial(0), "No Scripting Serial port found")
port:begin(230400)
port:set_unbuffered_writes(true)

-- Apply scale factor to convert ERPM to wheel RPM
-- Gear ratio is 12.37:1 and motor has 4 poles.
esc_telem:set_rpm_scale(0, 1.0 / (12.37 * 4.0))

--- Apply a byte to the crc and return the result
---@param crc integer
---@param byte integer
---@return integer
local function applyCRCByte(crc, byte)
    crc = crc ~ byte
    for _ = 1, 8 do
        if (crc & 0x80) ~= 0 then
            crc = ((crc << 1) ~ 0xE7) & 0xFF
        else
            crc = (crc << 1) & 0xFF
        end
    end
    return crc
end

--- Apply a table of bytes to the crc and return the result
---@param crc integer
---@param data integer[]
---@return integer
local function applyCRC(crc, data)
    for i = 1, #data do
        crc = applyCRCByte(crc, data[i])
    end
    return crc
end

-- Rolling sequence number
local sequence = 0

-- Record of sent messages so we can ignore our own
local sent = {}

--- Send a command and add the checksum value
---@param header integer[]
---@param payload integer[]
local function send(header, payload)

    -- Calculate 8 bit CRC
    local crc = 0
    crc = applyCRC(crc, header)
    crc = applyCRCByte(crc, sequence)
    crc = applyCRC(crc, payload)

    -- Write to uart
    local msg = string.char(table.unpack(header)) .. string.char(sequence) .. string.char(table.unpack(payload)) .. string.char(crc)
    port:writestring(msg)

    -- Add to record of sent messages
    table.insert(sent, { sequence=sequence, length=#msg, crc=crc })

    -- Make sure record is not too long
    for _ = 11, #sent do
        table.remove(sent, 1)
    end

    -- Increment the sequence number
    sequence = sequence + 1
    if sequence > 255 then
        sequence = 0
    end
end

---Check if this message is one we have sent
---@param seq integer
---@param len integer
---@param crc integer
---@return boolean
local function checkOwnMessage(seq, len, crc)

    for i,message in ipairs(sent) do
        if message.sequence == seq and
           message.length == len and
           message.crc == crc then
            -- This is our own message
            -- We can remove it, and anything before from the sent record
            for j = i, 1, -1 do
                table.remove(sent, j)
            end
            return true
        end
    end

    return false
end

--- Send a PWM command to the ESC
local function sendPWM()

    -- Throttle is sent as a PWM in the 1000 to 2000 range
    local pwm = SRV_Channels:get_output_pwm(70)
    if pwm == nil or pwm == 0 then
        -- Channel not assigned, or safety on, send 1500 for stop
        pwm = 1500
    end

    -- 0x06, 0x0D is the header
    -- Not sure what 0x05 denotes, it is sometimes also 0x01
    -- Then pass the PWM value directly
    send({0x06, 0x0D}, {0x05, pwm & 0xFF, (pwm >> 8) & 0xFF})

end

-- Track handshake progress
local handshakeStage = 0

-- Last connection time, allows restart if connection lost
local lastConnection = millis()

---Handle the ESC response to the handshake
---@param _payload string
local function ESCHandshakeA(_payload)
    --[[
    local buf = "Handshake A: "
    for i = 1, #payload do
        buf = buf .. string.format("0x%02X",string.byte(payload, i)) .. " "
    end
    print(buf)
    ]]--

    -- Send second part of handshake
    handshakeStage = 1
    send({0x04, 0x0F}, {0x05})
end

---Handle the ESC response to the handshake
---@param _payload string
local function ESCHandshakeB(_payload)
    --[[
    local buf = "Handshake B: "
    for i = 1, #payload do
        buf = buf .. string.format("0x%02X",string.byte(payload, i)) .. " "
    end
    print(buf)
    --]]

    -- Handshake is done!
    handshakeStage = 2
    lastConnection = millis()
end

-- Handle heartbeat message
---@param _payload string
local function heartbeat(_payload)
    lastConnection = millis()
end

-- Handle rpm report
---@param payload string
local function rpmReport(payload)
    local mode, rpm = string.unpack("Bl", payload, 2)

    --[[
        Mode is:
        0: Off
        1: Forward
        2: Reverse
        3: Brake
    ]]--

    -- Write to log
    logger:write("VXL4", "mode,rpm", "Bf", "-q", "00", mode, rpm)

    -- Update ESC telem backend
    esc_telem:update_rpm(0, rpm, 0)

    lastConnection = millis()
end

local messageLookup = {
    -- msg A sent both ways, "heartbeat"
    { header = { 0x04, 0x13 }, len=5, fun=heartbeat },
    -- RPM report, sent from ESC
    { header = { 0x09, 0x15 }, len=10, fun=rpmReport },
    -- Telem request, sent to ESC
    { header = { 0x09, 0x14 }, len=10, fun=nil },
    -- Throttle control, sent to ESC in sendPWM function
    { header = { 0x06, 0x0D }, len=7, fun=nil },
    -- HandShakes, sent to ESC as part of init
    { header = { 0x09, 0x01 }, len=10, fun=nil },
    { header = { 0x04, 0x0F }, len=5, fun=nil },
    -- HandShakes, from ESC
    { header = { 0x09, 0x02 }, len=10, fun=ESCHandshakeA },
    { header = { 0x15, 0x10 }, len=22, fun=ESCHandshakeB },
}

-- Incoming message buffer
local buffer = ""

-- Parse the buffer
local function parse()
    -- Need at least enough data to fit the smallest message
    if #buffer < 5 then
        return
    end

    -- Parse header
    local headerA = string.byte(buffer, 1)
    local headerB = string.byte(buffer, 2)
    local seq = string.byte(buffer, 3)

    -- We can calculate the crc of the header without knowing the message length
    local headerCrc = 0
    headerCrc = applyCRCByte(headerCrc, headerA)
    headerCrc = applyCRCByte(headerCrc, headerB)
    headerCrc = applyCRCByte(headerCrc, seq)

    for _,message in ipairs(messageLookup) do
        if headerA == message.header[1] and
           headerB == message.header[2] then
            -- Header matches
            if #buffer < message.len then
                -- Don't have enough data yet, try again later
                return
            end
            -- Have enough data to parse whole message
            -- Calculate the remainder of the crc
            local crc = headerCrc
            for i = 4, message.len - 1 do
                crc = applyCRCByte(crc, string.byte(buffer, i))
            end
            if crc == string.byte(buffer, message.len) then
                -- CRC is correct, check if it is a message we sent
                if not checkOwnMessage(seq, message.len, crc) then
                    -- Pass payload to the handler function
                    if message.fun then
                        local payload = string.sub(buffer, 4, message.len - 1)
                        message.fun(payload)
                    end
                end
                -- Remove message from buffer
                buffer = string.sub(buffer, message.len + 1)

                -- Run again on next section of buffer
                return parse()
            end
            -- CRC failed, maybe a incorrect header match, drop one element and try again
            break
        end
    end

    -- Remove one element and run again
    buffer = string.sub(buffer, 2)

    -- Run again on next section of buffer
    return parse()
end

-- Read incoming data (this includes stuff we have sent!)
local function receive()
    while true do
        local data = port:readstring(32)
        if data == nil then
            return
        end

        -- Add to the buffer
        buffer = buffer .. data

        if #data < 32 then
            -- Did not read full length, must have emptied the port buffer
            break
        end
    end

    parse()
end

-- Forward declare init function
local init

local sendItem = 0
local function update()

    if sendItem == 0 then
        -- Send throttle command
        sendPWM()

    elseif sendItem == 1 then
        -- Send a RPM request
        send({0x09, 0x14}, {0x05, 0x00, 0x00, 0x00, 0x00, 0x00})

--[[
    elseif sendItem == 2 then
        -- Send a "heartbeat", this does not seem to be required
        send({0x04, 0x13}, {0x01})
]]--
    end

    -- Cycle through sending each item
    sendItem = sendItem + 1
    if sendItem > 1 then
        sendItem = 0
    end

    receive()

    -- Check for connection loss
    if millis() - lastConnection > 1000 then
        -- Not heard from the ESC for 1 second, go back to trying the handshake
        print("ESC connection lost")
        return init, 200
    end

    -- Run at 100Hz
    return update, 10
end

-- How many times to listen before going back to init
local initRetry

local function listen()
    if initRetry == 0 then
        return init, 200
    end
    initRetry = initRetry - 1

    receive()

    if handshakeStage == 2 then
        print("ESC Handshake complete")
        return update()
    end

    return listen, 1
end

init = function()

    -- Send handshake
    send({0x09, 0x01}, {0x00, 0x03, 0x00, 0x00, 0x00, 0x00})

    -- Listen 10 times before trying init again
    initRetry = 10
    handshakeStage = 0

    -- Run very fast, need to be quick for handshake to work correctly
    return listen, 1
end

return init()
