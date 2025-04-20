--[[
    Script for direct connection to an NTRIP caster for Ethernet enabled autopilots.
    One port must be configured as a scripting TCP client:
        NET_Px_PROTOCOL	28
        NET_Px_TYPE	3
    The connected LAN must have WAN (Internet) access.
    The NTRIP caster must allow http connections (will not work over https).

    Edit CASTER_IP, CASTER_PORT, MOUNTPOINT, USERNAME, and PASSWORD to match
    your NTRIP caster and credentials.

    RTK2Go is available at 3.143.243.81 port 2101
]]

local CASTER_IP             = "3.143.243.81"
local CASTER_PORT           = 2101
local MOUNTPOINT            = "MOUNTPOINT"
local USERNAME              = "user-at-email.com"
local PASSWORD              = "none"

local MAV_SEVERITY          = { EMERGENCY = 0, ALERT = 1, CRITICAL = 2, ERROR = 3, WARNING = 4, NOTICE = 5, INFO = 6, DEBUG = 7 }

local BOOT_DELAY_MS         = 15000
local ACK_TIMEOUT_MS        = 1000
local CONNECTION_TIMEOUT_MS = 3000
local MAX_RETRIES           = 3
local POLL_RATE_HZ          = 10
local RECV_BLOCK_SIZE       = 1024

local tcp_socket            = nil
local retries               = 0
local rtcm_buffer           = ""
local last_data_time        = uint32_t(0)
local msg_count             = {}
local send_debug_msg_count  = false

-- Base64 encode a string
local function base64_encode(input)
    local b = 'ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/'
    return ((input:gsub('.', function(x)
        local r, b_char = '', x:byte()
        for i = 8, 1, -1 do r = r .. (b_char % 2 ^ i - b_char % 2 ^ (i - 1) > 0 and '1' or '0') end
        return r
    end) .. '0000'):gsub('%d%d%d?%d?%d?%d?', function(x)
        if (#x < 6) then return '' end
        local c = 0
        for i = 1, 6 do c = c + (x:sub(i, i) == '1' and 2 ^ (6 - i) or 0) end
        return b:sub(c + 1, c + 1)
    end) .. ({ '', '==', '=' })[#input % 3 + 1])
end

local function process_rtcm_buffer()
    -- RTCM message format:
    -- Byte 0: Preamble (0xD3)
    -- Byte 1-2: Message length (10 bits) + reserved (6 bits)
    -- Bytes 3-N: Message data
    -- Last 3 bytes: CRC

    while #rtcm_buffer >= 3 do
        local preamble_pos = rtcm_buffer:find(string.char(0xD3))
        if not preamble_pos then
            -- clear buffer on no preamble
            rtcm_buffer = ""
            return
        end

        if preamble_pos > 1 then
            -- slice buffer to preamble
            rtcm_buffer = rtcm_buffer:sub(preamble_pos)
        end

        if #rtcm_buffer < 3 then
            -- not enough data for header
            return
        end

        local length_high = (rtcm_buffer:byte(2) & 0x03)
        local length_low = rtcm_buffer:byte(3)
        local length = (length_high << 8) | length_low
        local total_length = 3 + length + 3 -- header + data + crc

        if #rtcm_buffer < total_length then
            -- not enough data for message
            return
        end

        if send_debug_msg_count then
            local message_type = (rtcm_buffer:byte(4) << 4) | (rtcm_buffer:byte(5) >> 4)
            msg_count[message_type] = (msg_count[message_type] or 0) + 1
            gcs:send_named_float(message_type, msg_count[message_type])
        end

        local complete_message = rtcm_buffer:sub(1, total_length)
        rtcm_buffer = rtcm_buffer:sub(total_length + 1)

        gps:inject_data(complete_message, #complete_message)

        if #rtcm_buffer >= 3 then
            process_rtcm_buffer() -- recurse
        end
    end
end

function send_connection_request()
    retries = retries + 1
    if retries > MAX_RETRIES then
        gcs:send_text(MAV_SEVERITY.WARNING, "NTRIP: Failed - max retries")
        return
    end

    tcp_socket = Socket(0)
    if not tcp_socket then
        gcs:send_text(MAV_SEVERITY.WARNING, "NTRIP: Error creating TCP socket")
        return send_connection_request, CONNECTION_TIMEOUT_MS
    end

    tcp_socket:set_blocking(true)

    gcs:send_text(MAV_SEVERITY.INFO, ("NTRIP: Connecting (attempt %d)"):format(retries))

    local success, err = tcp_socket:connect(CASTER_IP, CASTER_PORT)
    if not success then
        tcp_socket:close()
        gcs:send_text(MAV_SEVERITY.WARNING, ("NTRIP: Connection error (%s)"):format(err or "Unknown error"))
        return send_connection_request, CONNECTION_TIMEOUT_MS
    end

    if not tcp_socket then
        gcs:send_text(MAV_SEVERITY.WARNING, "NTRIP: TCP socket lost (on connection)")
        return send_connection_request, CONNECTION_TIMEOUT_MS
    end

    local auth = base64_encode(USERNAME .. ":" .. PASSWORD)
    local request = "GET /" .. MOUNTPOINT .. " HTTP/1.0\r\n" ..
        "User-Agent: NTRIP APLuaClient/1.0\r\n" ..
        "Authorization: Basic " .. auth .. "\r\n" ..
        "Ntrip-Version: Ntrip/2.0\r\n" ..
        "Connection: close\r\n\r\n"

    local send_success, send_err = tcp_socket:send(request, #request)
    if not send_success then
        gcs:send_text(MAV_SEVERITY.WARNING, ("NTRIP: Error sending request (%s)"):format(send_err or "Unknown error"))
        tcp_socket:close()
        return send_connection_request, CONNECTION_TIMEOUT_MS
    end

    tcp_socket:set_blocking(false)

    return read_server_response, ACK_TIMEOUT_MS
end

function read_server_response()
    if not tcp_socket then
        gcs:send_text(MAV_SEVERITY.WARNING, "NTRIP: TCP socket lost (on response)")
        return send_connection_request, CONNECTION_TIMEOUT_MS
    end

    local response, read_err = tcp_socket:recv(RECV_BLOCK_SIZE)
    if not response then
        gcs:send_text(MAV_SEVERITY.WARNING, ("NTRIP: Error reading response (%s)"):format(read_err))
        tcp_socket:close()
        return send_connection_request, CONNECTION_TIMEOUT_MS
    end

    -- check for HTTP 200 response
    if response:match("^HTTP/1%.%d 200") then
        gcs:send_text(MAV_SEVERITY.NOTICE, ("NTRIP: Connected (%s)"):format(MOUNTPOINT))
        retries = 0
        last_data_time = millis()
        return update, 1000 / POLL_RATE_HZ
    elseif response:match("^HTTP/1%.%d") then
        gcs:send_text(MAV_SEVERITY.WARNING, ("NTRIP: Server error (%s)"):format(response))
        tcp_socket:close()
        return send_connection_request, CONNECTION_TIMEOUT_MS
    else
        -- no response
        gcs:send_text(MAV_SEVERITY.WARNING, "NTRIP: No response to connection request")
        tcp_socket:close()
        return send_connection_request, CONNECTION_TIMEOUT_MS
    end
end

function update()
    if not tcp_socket then
        gcs:send_text(MAV_SEVERITY.WARNING, "NTRIP: TCP socket lost (after connection)")
        return send_connection_request, CONNECTION_TIMEOUT_MS
    end

    local now = millis()
    local rtcm_data = tcp_socket:recv(RECV_BLOCK_SIZE)

    if rtcm_data then
        last_data_time = now
        rtcm_buffer = rtcm_buffer .. rtcm_data
        process_rtcm_buffer()
    end

    if now - last_data_time > CONNECTION_TIMEOUT_MS then
        gcs:send_text(MAV_SEVERITY.WARNING, "NTRIP: Connection timeout")
        tcp_socket:close()
        return send_connection_request, CONNECTION_TIMEOUT_MS
    end

    return update, 1000 / POLL_RATE_HZ
end

gcs:send_text(MAV_SEVERITY.INFO, "NTRIP: Script loaded")

return send_connection_request, BOOT_DELAY_MS
