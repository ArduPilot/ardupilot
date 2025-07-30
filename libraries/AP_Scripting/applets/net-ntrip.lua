--[[
    Script for direct connection to an NTRIP caster for Ethernet enabled autopilots.
    The connected LAN must have WAN (Internet) access.
    The NTRIP caster must allow http connections (will not work over https).

    Edit CASTER_IP, CASTER_PORT, MOUNTPOINT, USERNAME, and PASSWORD to match
    your NTRIP caster and credentials.

    Edit NTRIP_VERSION as needed for your caster:
        - "Ntrip/2.0" for modern casters (RTK2Go supports this)
        - "Ntrip/1.0" for legacy casters (uses ICY 200 OK response)

    Edit GGA_RATE_HZ to set the rate of GGA messages to send to the caster.
    Set to 0 to disable sending own position to NTRIP caster.
    Many NTRIP v1.0 casters require GGA messages for authentication.

    RTK2Go is available at 3.143.243.81 port 2101
]]

local CASTER_IP             = "3.143.243.81"
local CASTER_PORT           = 2101
local MOUNTPOINT            = "MOUNTPOINT"
local USERNAME              = "user-at-email.com"
local PASSWORD              = "none" -- "none" is ok for RTK2G0
local NTRIP_VERSION         = 2      -- 1 or 2
local GGA_RATE_HZ           = 0      -- set 1 or 2 Hz as required, 0 to disable

local MAV_SEVERITY          = { EMERGENCY = 0, ALERT = 1, CRITICAL = 2, ERROR = 3, WARNING = 4, NOTICE = 5, INFO = 6, DEBUG = 7 }

-- map MAVLink GPS status to NMEA quality code
local GPS_QUAL              = { [0] = 0, [1] = 0, [2] = 1, [3] = 1, [4] = 2, [5] = 2, [6] = 2 }

local BOOT_DELAY_MS         = 15000 -- wait 15 seconds for GPS and network to initialize
local ACK_TIMEOUT_MS        = 1000
local CONNECTION_TIMEOUT_MS = 5000
local MAX_RETRIES           = 3
local RESET_MS              = 120000 -- wait 2 minutes after max retries
local POLL_RATE_HZ          = 10
local RECV_BLOCK_SIZE       = 1024

local tcp_socket            = nil
local retries               = 0
local rtcm_buffer           = ""
local last_data_time        = uint32_t(0)
local last_gga_time         = uint32_t(0)
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

        gps:inject_data(complete_message)
    end
end

local function get_gga_timestamp()
    local leap_seconds = 18
    local ms_per_min = 60 * 1000
    local ms_per_hour = 60 * ms_per_min
    local ms_per_day = 24 * ms_per_hour
    local now = gps:time_week_ms(0):toint()
    now = now - leap_seconds * 1000

    local today_ms = now % ms_per_day
    local hour = math.floor(today_ms / ms_per_hour)
    local mins = (today_ms % ms_per_hour) / ms_per_min
    local secs = (mins % 1) * 60

    return ("%02d%02d%02.2f"):format(hour, math.floor(mins), secs)
end

-- use own position to generate GGA message
local function generate_gga()
    local gps_data = gps:location(0)
    if not gps_data then
        return nil
    end

    local lat = gps_data:lat() / 1e7
    local lon = gps_data:lng() / 1e7
    local alt = gps_data:alt() / 100
    local hdop = gps:get_hdop(0) / 100
    local satellites = gps:num_sats(0)

    local lat_deg = math.floor(math.abs(lat))
    local lat_min = (math.abs(lat) - lat_deg) * 60
    local lat_dir = lat >= 0 and "N" or "S"

    local lon_deg = math.floor(math.abs(lon))
    local lon_min = (math.abs(lon) - lon_deg) * 60
    local lon_dir = lon >= 0 and "E" or "W"

    local lat_str = string.format("%02d%07.4f", lat_deg, lat_min)
    local lon_str = string.format("%03d%07.4f", lon_deg, lon_min)

    local qual = GPS_QUAL[gps:status(0)] or 1

    local now_str = get_gga_timestamp()

    local gga = string.format("$GPGGA,%s,%s,%s,%s,%s,%d,%02d,%.1f,%.1f,M,0.0,M,,",
        now_str, lat_str, lat_dir, lon_str, lon_dir, qual, satellites, hdop, alt)

    local checksum = 0
    for i = 2, #gga do
        checksum = checksum ~ string.byte(gga, i)
    end

    return gga .. "*" .. string.format("%02X", checksum) .. "\r\n"
end

function send_connection_request()
    retries = retries + 1
    if retries > MAX_RETRIES then
        local reset_mins = RESET_MS / 60000
        gcs:send_text(MAV_SEVERITY.WARNING, ("NTRIP: Failed - max retries (reset in %.1f mins)"):format(reset_mins))
        retries = 0
        return send_connection_request, RESET_MS
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
        "Authorization: Basic " .. auth .. "\r\n"

    if NTRIP_VERSION == 2 then
        request = request .. "Ntrip-Version: Ntrip/2.0\r\n"
    end -- else simply default to Ntrip/1.0 (no version header)

    request = request .. "Connection: close\r\n\r\n"

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

    if response:match("^ICY 200 OK") then
        gcs:send_text(MAV_SEVERITY.NOTICE, ("NTRIP: Connected (%s / v1.0)"):format(MOUNTPOINT))
        retries = 0
        last_data_time = millis()
        return update, 1000 / POLL_RATE_HZ
    elseif response:match("^HTTP/1%.%d 200") then
        gcs:send_text(MAV_SEVERITY.NOTICE, ("NTRIP: Connected (%s / v2.0)"):format(MOUNTPOINT))
        retries = 0
        last_data_time = millis()
        return update, 1000 / POLL_RATE_HZ
    elseif response:match("^HTTP/1%.%d") or response:match("^ICY") then
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

    if GGA_RATE_HZ > 0 and (now - last_gga_time) >= (1000 / GGA_RATE_HZ) then
        local gga = generate_gga()
        if gga then
            local send_success, send_err = tcp_socket:send(gga, #gga)
            if send_success then
                last_gga_time = now
            else
                gcs:send_text(MAV_SEVERITY.WARNING, ("NTRIP: Error sending GGA (%s)"):format(send_err or "Unknown error"))
            end
        end
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
