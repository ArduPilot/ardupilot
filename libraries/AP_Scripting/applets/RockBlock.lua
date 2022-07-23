-- Lua script to send a recieve very basic MAVLink telemetry over a
-- Rockblock SBD satellite modem
-- Requires https://github.com/stephendade/rockblock2mav at the GCS end

-- Setup:
-- This script requires 1 serial port:
-- A "Script" to connect the modem

-- Usage:
-- Use the MAVLink High Latency Control ("link hl on|off" in MAVProxy) to control
-- whether to send or not (or use "force_hl_enable")
-- Use the "debug_text" variable to view debugging statustexts at the GCS
-- Use the "force_hl_enable" variable to force-enable high latency mode, instead of enabling from GCS

-- Caveats:
-- This will *only* send HIGH_LATENCY2 packets via the SBD modem. No heartbeats,
-- no command acknowledgements, no statustexts, no parameters, etc
-- A single HIGH_LATENCY2 packet will be send every 20 sec
-- MAVLink 1 will be used, as it's slightly more efficient (50 vs 52 bytes for a HL2 message)
-- Any incoming packets on the first mailbox check will be ignored (as these may be from a long time in the past)
-- Only 1 command can be sent at a time from the GCS. Any subsequent commands will overwrite the previous command

-- Written by Stephen Dade (stephen_dade@hotmail.com)

local port = serial:find_serial(0)

port:begin(19200)
port:set_flow_control(0)

-- true to use MAVLink1, false to use MAVLink2
MAVLink:begin(true)

local AT_query = "AT+CGMM\r"
local AT_mailbox_check = "AT+SBDIXA\r"
local AT_load_tx_buffer = "AT+SBDWB="
local AT_read_rx_buffer = "AT+SBDRB\r"
local AT_clear_tx_buffer = "AT+SBDD0\r"
local AT_clear_rx_buffer = "AT+SBDD1\r"
local modem_detected = false
local str_recieved = ""
local modem_history = {}
local modem_to_send = {}

local debug_text = false
local force_hl_enable = true
local is_transmitting = false
local first_sucessful_mailbox_check = false

local time_last_tx = millis():tofloat() * 0.001
local last_modem_status_check = 0

-- Parse any incoming text from the modem
function check_cmd_return()
    -- modem detection (response to AT_query)
    if #modem_history == 3 and modem_history[1] == 'AT+CGMM' and
        modem_history[3] == 'OK' then
        gcs:send_text(3, "SATCOM modem detected - " ..
                          nicestring(modem_history[2]))
        modem_history = nil
        modem_history = {}
        modem_detected = true
        table.insert(modem_to_send, AT_clear_rx_buffer)
        table.insert(modem_to_send, AT_clear_tx_buffer)

        -- enable high latency mode, if desired
        if force_hl_enable then MAVLink:set_high_latency_enabled(true) end
    end

    if modem_detected then
        -- TX Buffer clear (response to AT_query)
        if #modem_history >= 3 and modem_history[#modem_history - 2] ==
            'AT+SBDD0' and modem_history[#modem_history] == 'OK' then
            if debug_text then
                gcs:send_text(3, "SATCOM cleared modem TX buffer")
            end
            modem_history = nil
            modem_history = {}
        end

        -- RX buffer clear (response to AT_query)
        if #modem_history >= 3 and modem_history[#modem_history - 2] ==
            'AT+SBDD1' and modem_history[#modem_history] == 'OK' then
            if debug_text then
                gcs:send_text(3, "SATCOM cleared modem RX buffer")
            end
            modem_history = nil
            modem_history = {}
        end

        -- Tx buffer loaded (response to AT command)
        if #modem_history >= 4 and
            string.find(modem_history[#modem_history - 3], AT_load_tx_buffer, 1,
                        true) and modem_history[#modem_history] == 'OK' then
            if debug_text then
                gcs:send_text(3, "SATCOM loaded packet into tx buffer, " ..
                                  tostring(modem_history[#modem_history - 3]))
            end
            if modem_history[#modem_history - 1] ~= '0' then
                gcs:send_text(3, "SATCOM Error loading packet into buffer")
            end
            modem_history = nil
            modem_history = {}
        end

        -- Got received data (response to AT command)
        if modem_history[#modem_history - 2] == 'AT+SBDRB' and
            modem_history[#modem_history] == 'OK' then
            -- Message format is { 2 byte msg length} + {message} + {2 byte checksum}
            local totallen = #(modem_history[#modem_history - 1])
            local len = string.unpack(">i2", modem_history[#modem_history - 1])
            local msg = string.sub(modem_history[#modem_history - 1], 3,
                                   totallen - 2)
            local checksumrx = string.sub(modem_history[#modem_history - 1],
                                          totallen - 1, totallen)
            local highByte, lowByte = checksum(msg)

            -- check that received message is OK, then send to MAVLink processor
            if len ~= #msg then
                gcs:send_text(3,
                              "SATCOM: Bad RX message length " .. tostring(len) ..
                                  " vs actual " .. tostring(#msg))
            elseif checksumrx ~= (tostring(highByte) .. tostring(lowByte)) then
                gcs:send_text(3,
                              "SATCOM: Bad RX CRC " .. checksumrx .. " vs " ..
                                  tostring(highByte) .. tostring(lowByte))
            else
                if debug_text then
                    gcs:send_text(3, "totmsg=" ..
                                      nicestring(
                                          modem_history[#modem_history - 1]))
                end
                for idx = 1, #msg do
                    MAVLink:receive(msg:byte(idx))
                end

            end

            modem_history = nil
            modem_history = {}
        end

        -- Mailbox check  (response to AT command) (can be a fail or success)
        if modem_history[#modem_history - 2] == 'AT+SBDIXA' and
            modem_history[#modem_history] == 'OK' then

            -- Parse response (comma and : delimited, trimming whitespace)
            local statusReponse = {}
            for w in modem_history[#modem_history - 1]:gmatch("[^,:]+") do
                table.insert(statusReponse, w)
            end

            if #statusReponse == 7 then
                if tonumber(statusReponse[2]) < 5 then
                    gcs:send_text(3, "SATCOM Packet Transmitted successfully")
                    -- check for rx packet
                    if tonumber(statusReponse[4]) == 1 and
                        first_sucessful_mailbox_check then
                        gcs:send_text(3, "SATCOM Packet received")
                        -- read messages, if not first mailbox check
                        table.insert(modem_to_send, AT_read_rx_buffer)
                    elseif debug_text then
                        gcs:send_text(3, "SATCOM: No message to receive")
                    end
                    first_sucessful_mailbox_check = true
                elseif tonumber(statusReponse[2]) == 32 then
                    gcs:send_text(3, "SATCOM Error: No network service")
                else
                    gcs:send_text(3, "SATCOM Error: Unable to send")
                end
            end
            modem_history = nil
            modem_history = {}
            is_transmitting = false
        end
    end
end

function checksum(bytes)
    -- Checksum calculation for SBDWB
    -- The checksum is the least significant 2-bytes of the summation of the entire SBD message
    local SUM = 0

    for idx = 1, #bytes do SUM = SUM + bytes:byte(idx) end

    local SUM_H = (SUM & 0xFF << 8) >> 8
    local SUM_L = SUM & 0xFF

    if debug_text then
        gcs:send_text(3, "Modem CRC: " .. string.char(SUM_H) .. " " ..
                          string.char(SUM_L))
    end

    return string.char(SUM_H), string.char(SUM_L)
end

function nicestring(instr)
    -- make any strings printable to GCS (constrain to ASCII range)
    local retstr = ""
    for i = 1, #instr do
        local c = string.byte(instr:sub(i, i))
        if c < 0x20 or c > 0x7E then c = 0x5F end
        retstr = retstr .. string.char(c)
    end

    return tostring(retstr)
end

function HLSatcom()
    -- read in any bytes and form into received commands
    local n_bytes = port:available()
    while n_bytes > 0 do
        read = port:read()
        read = string.char(read)
        if read == '\r' or read == '\n' then
            if #str_recieved > 0 then
                table.insert(modem_history, str_recieved)
                if debug_text then
                    gcs:send_text(3, "Modem response: " ..
                                      nicestring(modem_history[#modem_history]))
                end
                check_cmd_return()
            end
            str_recieved = ""
        else
            str_recieved = str_recieved .. read
        end
        n_bytes = n_bytes - 1
    end

    -- write out commands from send list. one cmd per loop
    if #modem_to_send > 0 then
        for idx = 1, #modem_to_send[1] do
            port:write(modem_to_send[1]:byte(idx))
        end
        if debug_text then
            gcs:send_text(3, "Sent to modem: " .. nicestring(modem_to_send[1]))
        end
        table.remove(modem_to_send, 1)
    end

    --- send detect command to modem every 10 sec if not detected
    if not modem_detected and (millis():tofloat() * 0.001) -
        last_modem_status_check > 10 then
        table.insert(modem_to_send, AT_query)
        last_modem_status_check = millis():tofloat() * 0.001
    end

    -- send HL2 packet every 30 sec, if not aleady in a mailbox check
    if modem_detected and MAVLink:is_high_latency_enabled() and
        (millis():tofloat() * 0.001) - time_last_tx > 30 and not is_transmitting then

        local pkt = MAVLink:create_high_latency_packet()

        local highByte, lowByte = checksum(pkt)

        if #pkt > 50 then
            gcs:send_text(3, "SATCOM Tx packet > 50 bytes: " .. tostring(#pkt))
        end

        -- Send as binary data, plus checksum
        table.insert(modem_to_send, AT_load_tx_buffer .. tostring(#pkt) .. '\r')
        table.insert(modem_to_send, pkt)
        table.insert(modem_to_send, highByte .. lowByte .. "\r")

        time_last_tx = millis():tofloat() * 0.001
        table.insert(modem_to_send, AT_mailbox_check)
        is_transmitting = true

    end

    return HLSatcom, 100
end

return HLSatcom, 100
