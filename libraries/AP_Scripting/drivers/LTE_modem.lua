--[[
    driver for LTE modems with AT command set
    supported chipsets:
      - SIM7600
--]]

local MAV_SEVERITY = {EMERGENCY=0, ALERT=1, CRITICAL=2, ERROR=3, WARNING=4, NOTICE=5, INFO=6, DEBUG=7}

local PARAM_TABLE_KEY = 106
local PARAM_TABLE_PREFIX = "LTE_"

-- local MAVLINK2 = 2
local PPP = 48

-- add a parameter and bind it to a variable
local function bind_add_param(name, idx, default_value)
    assert(param:add_param(PARAM_TABLE_KEY, idx, name, default_value), string.format('could not add param %s', name))
    return Parameter(PARAM_TABLE_PREFIX .. name)
end

-- Setup Parameters
assert(param:add_table(PARAM_TABLE_KEY, PARAM_TABLE_PREFIX, 20), 'LTE_modem: could not add param table')

--[[
    // @Param: LTE_ENABLE
    // @DisplayName: LTE Enable
    // @Description: Enable or disable the LTE modem driver
    // @Values: 0:Disabled,1:Enabled
    // @User: Standard
--]]
local LTE_ENABLE = bind_add_param('ENABLE',  1, 1)

--[[
    // @Param: LTE_SERPORT
    // @DisplayName: Serial Port
    // @Description: Serial port to use for the LTE modem. This is the index of the SERIALn_ ports that are set to 28 for "scripting"
    // @Range: 0 8
    // @User: Standard
--]]
local LTE_SERPORT = bind_add_param('SERPORT',  2, 0)

--[[
    // @Param: LTE_SCRPORT
    // @DisplayName: Scripting Serial Port
    // @Description: Scripting Serial port to use for the LTE modem. This is the index of the SCR_SDEV ports that are set to 2 for "MAVLink2"
    // @Range: 0 8
    // @User: Standard
--]]
local LTE_SCRPORT = bind_add_param('SCRPORT',  3, 0)

--[[
    // @Param: LTE_SERVER_IP0
    // @DisplayName: Server IP 0
    // @Description: First octet of the server IP address to connect to
    // @Range: 0 255
    // @User: Standard
--]]
local LTE_SERVER_IP0  = bind_add_param('SERVER_IP0',  4, 0)

--[[
    // @Param: LTE_SERVER_IP1
    // @DisplayName: Server IP 1
    // @Description: Second octet of the server IP address to connect to
    // @Range: 0 255
    // @User: Standard
--]]
local LTE_SERVER_IP1  = bind_add_param('SERVER_IP1',  5, 0)

--[[
    // @Param: LTE_SERVER_IP2
    // @DisplayName: Server IP 2
    // @Description: Third octet of the server IP address to connect to
    // @Range: 0 255
    // @User: Standard
--]]
local LTE_SERVER_IP2  = bind_add_param('SERVER_IP2',  6, 0)

--[[
    // @Param: LTE_SERVER_IP3
    // @DisplayName: Server IP 3
    // @Description: Fourth octet of the server IP address to connect to
    // @Range: 0 255
    // @User: Standard
--]]
local LTE_SERVER_IP3  = bind_add_param('SERVER_IP3',  7, 0)

--[[
    // @Param: LTE_SERVER_PORT
    // @DisplayName: Server Port
    // @Description: IPv4 Port of the server to connect to
    // @Range: 1 65525
    // @User: Standard
--]]
local LTE_SERVER_PORT = bind_add_param('SERVER_PORT',  8, 0)

--[[
    // @Param: LTE_BAUD
    // @DisplayName: Serial Baud Rate
    // @Description: Baud rate for the serial port to the LTE modem when connected. Initial power on baudrate is in LTE_IBAUD
    // @Values: 19200:19200,38400:38400,57600:57600,115200:115200,230400:230400,460800:460800,921600:921600,3686400:3686400
    // @User: Standard
--]]
local LTE_BAUD        = bind_add_param('BAUD',  9, 921600)

--[[
    // @Param: LTE_TIMEOUT
    // @DisplayName: Timeout
    // @Description: Timeout in seconds for the LTE connection. If no data is received for this time, the connection will be reset.
    // @Range: 1 60
    // @Units: s
    // @User: Standard
--]]
local LTE_TIMEOUT     = bind_add_param('TIMEOUT', 10, 10)

--[[
    // @Param: LTE_PROTOCOL
    // @DisplayName: LTE protocol
    // @Description: The protocol that we will use in communication with the LTE modem. If this is PPP then the LTE_SERVER parameters are not used and instead a PPP connection will be established and you should use the NET_ parameters to enable network ports. If this is MAVLink2 then the LTE_SERVER parameters are used to create a TCP connection to a single TCP server.
    // @Values: 2:MavLink2,48:PPP
    // @User: Standard
--]]
local LTE_PROTOCOL     = bind_add_param('PROTOCOL', 11, 48)

--[[
    // @Param: LTE_OPTIONS
    // @DisplayName: LTE options
    // @Description: Options to control the LTE modem driver. If VerboseSignalInfoGCS is set then additional NAMED_VALUE_FLOAT values are sent with verbose signal information
    // @Bitmask: 0:LogAllData,1:VerboseSignalInfoGCS
    // @User: Standard
--]]
local LTE_OPTIONS     = bind_add_param('OPTIONS', 12, 0)

--[[
    // @Param: LTE_IBAUD
    // @DisplayName: LTE initial baudrate
    // @Description: This is the initial baud rate on power on for the modem. This is set in the modem with the AT+IREX=baud command
    // @Values: 19200:19200,38400:38400,57600:57600,115200:115200,230400:230400,460800:460800,921600:921600,3686400:3686400
    // @User: Standard
--]]
local LTE_IBAUD       = bind_add_param('IBAUD', 13, 115200)

local LTE_OPTIONS_LOGALL = (1<<0)
local LTE_OPTIONS_SIGNALS = (1<<1)

--[[
    return true if an option is enabled
--]]
local function option_enabled(option)
    return (LTE_OPTIONS:get() & option) ~= 0
end

if LTE_ENABLE:get() == 0 then
    -- disabled
    return
end

local uart = serial:find_serial(LTE_SERPORT:get())
if not uart then
    gcs:send_text(MAV_SEVERITY.ERROR, 'LTE_modem: could not find serial port')
    return
end

local ser_device = serial:find_simulated_device(LTE_PROTOCOL:get(), LTE_SCRPORT:get())
if not ser_device then
    gcs:send_text(MAV_SEVERITY.ERROR, 'LTE_modem: could not find SCR_SDEV device')
    return
end

local step = "ATI"

local stats = { bytes_in = 0, bytes_out = 0 }

uart:begin(LTE_IBAUD:get())

--[[
    Open a log file to log the output from the modem
    This is useful for debugging the connection process
--]]
local log_file = io.open('LTE_modem.log', 'w')

--[[
    log data to log_file
--]]
local function log_data(s, marker)
    if s and #s > 0 and log_file then
        log_file:write(marker .. '[' .. s .. ']\n')
        log_file:flush()
    end
end

--[[
    Function to read from the UART and log the output
    This function reads up to 512 bytes at a time and writes it to the log file
    returns the string read or nil
--]]
local function uart_read()
    local s = uart:readstring(512)
    log_data(s, '<<<')
    stats.bytes_in = stats.bytes_in + #s
    return s
end

--[[
    Function to write to the UART and log the command
--]]
local function uart_write(s)
    uart:writestring(s)
    log_data(s, '>>>')
    stats.bytes_out = stats.bytes_out + #s
    return #s
end

-- Constants for GSM 07.10 CMUX framing
local FLAG = 0xF9
local UIH = 0xEF
local SABM = 0x2F
--local UA = 0x63
local EA = 0x01
local CR_SEND = 0x02

-- CMUX buffer state
local cmux = {}
cmux.buffers = {[1] = "", [2] = ""} -- DLC1=AT, DLC2=DATA(PPP or TCP)

--[[
    FCS lookup table for polynomial x^8 + x^2 + x^1 + 1 (0x07)
    This is the reverse of the standard CRC-8 table
--]]
local fcs_table = {
    0x00, 0x91, 0xe3, 0x72, 0x07, 0x96, 0xe4, 0x75,
    0x0e, 0x9f, 0xed, 0x7c, 0x09, 0x98, 0xea, 0x7b,
    0x1c, 0x8d, 0xff, 0x6e, 0x1b, 0x8a, 0xf8, 0x69,
    0x12, 0x83, 0xf1, 0x60, 0x15, 0x84, 0xf6, 0x67,
    0x38, 0xa9, 0xdb, 0x4a, 0x3f, 0xae, 0xdc, 0x4d,
    0x36, 0xa7, 0xd5, 0x44, 0x31, 0xa0, 0xd2, 0x43,
    0x24, 0xb5, 0xc7, 0x56, 0x23, 0xb2, 0xc0, 0x51,
    0x2a, 0xbb, 0xc9, 0x58, 0x2d, 0xbc, 0xce, 0x5f,
    0x70, 0xe1, 0x93, 0x02, 0x77, 0xe6, 0x94, 0x05,
    0x7e, 0xef, 0x9d, 0x0c, 0x79, 0xe8, 0x9a, 0x0b,
    0x6c, 0xfd, 0x8f, 0x1e, 0x6b, 0xfa, 0x88, 0x19,
    0x62, 0xf3, 0x81, 0x10, 0x65, 0xf4, 0x86, 0x17,
    0x48, 0xd9, 0xab, 0x3a, 0x4f, 0xde, 0xac, 0x3d,
    0x46, 0xd7, 0xa5, 0x34, 0x41, 0xd0, 0xa2, 0x33,
    0x54, 0xc5, 0xb7, 0x26, 0x53, 0xc2, 0xb0, 0x21,
    0x5a, 0xcb, 0xb9, 0x28, 0x5d, 0xcc, 0xbe, 0x2f,
    0xe0, 0x71, 0x03, 0x92, 0xe7, 0x76, 0x04, 0x95,
    0xee, 0x7f, 0x0d, 0x9c, 0xe9, 0x78, 0x0a, 0x9b,
    0xfc, 0x6d, 0x1f, 0x8e, 0xfb, 0x6a, 0x18, 0x89,
    0xf2, 0x63, 0x11, 0x80, 0xf5, 0x64, 0x16, 0x87,
    0xd8, 0x49, 0x3b, 0xaa, 0xdf, 0x4e, 0x3c, 0xad,
    0xd6, 0x47, 0x35, 0xa4, 0xd1, 0x40, 0x32, 0xa3,
    0xc4, 0x55, 0x27, 0xb6, 0xc3, 0x52, 0x20, 0xb1,
    0xca, 0x5b, 0x29, 0xb8, 0xcd, 0x5c, 0x2e, 0xbf,
    0x90, 0x01, 0x73, 0xe2, 0x97, 0x06, 0x74, 0xe5,
    0x9e, 0x0f, 0x7d, 0xec, 0x99, 0x08, 0x7a, 0xeb,
    0x8c, 0x1d, 0x6f, 0xfe, 0x8b, 0x1a, 0x68, 0xf9,
    0x82, 0x13, 0x61, 0xf0, 0x85, 0x14, 0x66, 0xf7,
    0xa8, 0x39, 0x4b, 0xda, 0xaf, 0x3e, 0x4c, 0xdd,
    0xa6, 0x37, 0x45, 0xd4, 0xa1, 0x30, 0x42, 0xd3,
    0xb4, 0x25, 0x57, 0xc6, 0xb3, 0x22, 0x50, 0xc1,
    0xba, 0x2b, 0x59, 0xc8, 0xbd, 0x2c, 0x5e, 0xcf
}

--[[
    Calculate FCS for a byte array
    data: table of bytes (numbers 0-255) or string
    Returns: FCS value (0-255)
--]]
local function fcs_calc(data)
    local fcs = 0xff  -- Initial value
    
    for i = 1, #data do
        local byte = string.byte(data, i)
        fcs = fcs_table[((fcs ~ byte) & 0xff) + 1] ~ (fcs >> 8)
    end

    return (~fcs) & 0xff
end

-- Construct a CMUX frame for a given DLC, data type and data
function cmux.encode_cmux_frame(dlc, dtype, data)
    local addr = string.char((dlc << 2) | EA | CR_SEND)
    local ctrl = string.char(dtype | 0x10)
    local len = #data
    local len_byte = string.char((len << 1) | EA)
    local header = addr .. ctrl .. len_byte
    local fcs = string.char(fcs_calc(header))
    return string.char(FLAG) .. header .. data .. fcs .. string.char(FLAG)
end

--[[
    send an AT command string with CMUX framing
--]]
local function cmux_AT_send(atcmd)
    local s = cmux.encode_cmux_frame(1, UIH, atcmd)
    return uart_write(s) == #s
end

--[[
    send an appropriate data reset for the protocol
--]]
local function send_data_reset()
    if LTE_PROTOCOL:get() == PPP then
        cmux_AT_send('ATH\r\n')
    else
        cmux_AT_send('AT+CRESET\r\n')
    end
end

--[[
    Function to handle errors in the response from the modem
    If an error is detected, it resets the modem
    returns true if an error was detected
--]]
local function handle_error(s)
    if s and s:find('\nERROR\r\n') then
        gcs:send_text(MAV_SEVERITY.ERROR, 'LTE_modem: error response from modem')
        send_data_reset()
        step = "ATI"
        return true
    end
    return false
end

-- Send SABM (Set Asynchronous Balanced Mode) for all DLCs
function cmux.send_sabm()
    uart_write(cmux.encode_cmux_frame(0, SABM, ""))
    uart_write(cmux.encode_cmux_frame(1, SABM, ""))
    uart_write(cmux.encode_cmux_frame(2, SABM, ""))
end

--[[
 Parses a single CMUX frame from a byte buffer.
 Returns: DLC number, extracted payload, and remaining buffer (or nils on failure)
--]]
function cmux.parse_cmux_frame(buf)
    local start_idx = buf:find(string.char(FLAG))
    if not start_idx then
        --gcs:send_text(MAV_SEVERITY.INFO, "no start idx")
        return nil, nil, nil
    end
    local end_idx = buf:find(string.char(FLAG), start_idx + 1)
    if not end_idx then
        --gcs:send_text(MAV_SEVERITY.INFO, "no end idx")
        return nil, nil, nil, "short"
    end

    local frame = buf:sub(start_idx + 1, end_idx - 1)
    if #frame < 4 then
        --gcs:send_text(MAV_SEVERITY.INFO, "too short")
        return nil, nil, nil
    end

    local addr = frame:byte(1)
    local ctrl = frame:byte(2)

    --gcs:send_text(MAV_SEVERITY.INFO, string.format("addr=0x%02x ctrl=0x%02x", addr, ctrl))

    if ctrl == SABM then
        return nil, nil, buf:sub(end_idx + 1)
    end

    if (ctrl & 0xef) ~= UIH then
        return nil, nil, nil
    end

    local len_byte = frame:byte(3)
    if (len_byte & EA) == 0 then
        gcs:send_text(MAV_SEVERITY.INFO, "mux multibyte")
        return nil, nil, nil -- we don't handle multi-byte length yet
    end
    local len = len_byte >> 1
    if #frame < 3 + len + 1 then return nil, nil, nil end

    local data = frame:sub(4, 3 + len)
    local fcs_field = frame:byte(3 + len + 1)
    local header = frame:sub(1, 3)
    local calc_fcs = fcs_calc(header)
    if calc_fcs ~= fcs_field then
        gcs:send_text(MAV_SEVERITY.INFO, "FCS mismatch")
        return nil, nil, nil -- FCS mismatch
    end

    local dlc = (addr >> 2) & 0x3F
    local remainder = buf:sub(end_idx + 1)
    --gcs:send_text(MAV_SEVERITY.INFO, string.format("CMUX got: dlc=%d ldata=%d lrem=%d", dlc, #data, #remainder))
    return dlc, data, remainder
end

-- Feeds raw UART data into CMUX frame parser and routes payloads to DLC buffers
function cmux.feed_uart_in(raw)
    while #raw > 0 do
        local dlc, data, rest, err = cmux.parse_cmux_frame(raw)
        if not dlc or not data or not rest then
            if err == "short" then
                return raw
            end
            -- discard
            return ""
        end
        if cmux.buffers[dlc] then
            cmux.buffers[dlc] = cmux.buffers[dlc] .. data
        end
        raw = rest
    end
    return raw
end

--[[
    send data with CMUX framing
--]]
local function cmux_data_send(data)
    local s = cmux.encode_cmux_frame(2, UIH, data)
    return uart_write(s) == #s
end

--[[
    send data with CMUX framing when connected (logging only if data
    logging enabled)
--]]
local function cmux_data_send_connected(data)
    local s = cmux.encode_cmux_frame(2, UIH, data)
    if option_enabled(LTE_OPTIONS_LOGALL) then
        log_data(s, '>>>')
    end
    local n = uart:writestring(s)
    stats.bytes_out = stats.bytes_out + n
    return n == #s
end

local ati_sequence = 0

--[[
    Function to confirm the connection to the modem
    it uses AIT command to get the modem info

    when we enter the ATI step the modem could be in one of several states:

    - in AT command mode
    - in muxed mode
    - in muxed mode at higher baudrate
--]]
local function step_ATI()
    local s = uart_read()
    if s and s:find('IMEI: ') then
        gcs:send_text(MAV_SEVERITY.INFO, 'LTE_modem: found modem')
        if cmux.parse_cmux_frame(s) then
            -- already in CMUX mode
            cmux.send_sabm()
            send_data_reset()
            step = "BAUD"
        else
            step = "CMUX"
        end
        return
    end
    if s and #s >= 4 and s:byte(1) == FLAG and s:byte(-1) == FLAG then
        -- already in mux mode
        send_data_reset()
        cmux_AT_send('ATI\r\n')
        return
    end
    if ati_sequence % 2 == 1 then
        uart_write('+++')
    else
        uart_write('\r\nATI\r\n')
    end
    if ati_sequence % 10 == 5 then
        uart:begin(LTE_BAUD:get())
    end
    if ati_sequence % 10 == 9 then
        uart:begin(LTE_IBAUD:get())
    end
    ati_sequence = ati_sequence + 1
end

local change_baud = nil

--[[
    change baud rate
--]]
local function step_BAUD()
    cmux_AT_send(string.format('AT+IPR=%u\r\n', LTE_BAUD:get()))
    step = "CREG"
    change_baud = LTE_BAUD:get()
end

--[[
    confirm we are registered on the network
--]]
local function step_CREG()
    local s = uart_read()
    if handle_error(s) then
        return
    end
    if s and s:find('CREG: 0,1\r\n') then
        gcs:send_text(MAV_SEVERITY.INFO, 'LTE_modem: CREG OK')
        if LTE_PROTOCOL:get() == PPP then
            step = "PPPOPEN"
        else
            step = "CIPMODE"
        end
        return
    end
    cmux_AT_send('AT+CREG?\r\n')
end

--[[
    set the modem to transparent mode
--]]
local function step_CIPMODE()
    local s = uart_read()
    if handle_error(s) then
        return
    end
    if s and s:find('CIPMODE=1\r') and s:find('\r\r\nOK\r') then
        gcs:send_text(MAV_SEVERITY.INFO, 'LTE_modem: transparent mode set')
        step = "NETOPEN"
        return
    end
    cmux_data_send('AT+CIPMODE=1\r\n')
end

--[[
    setup CMUX multiplexing mode
--]]
local function step_CMUX()
    local s = uart_read()
    if handle_error(s) then
        return
    end
    if s and s:find('CMUX=0\r\r\nOK\r') then
        gcs:send_text(MAV_SEVERITY.INFO, 'LTE_modem: CMUX mode set')
        -- send SABM frames to establish the DLCs
        send_data_reset()
        cmux.send_sabm()
        step = "BAUD"
        return
    end
    uart_write('AT+CMUX=0\r\n')
end

--[[
    open the network stack
    needed to be able to open a TCP connection
--]]
local function step_NETOPEN()
    local s = uart_read()
    if handle_error(s) then
        return
    end
    if s and s:find('NETOPEN\r') and s:find('\r\r\nOK\r\n') then
        gcs:send_text(MAV_SEVERITY.INFO, 'LTE_modem: network opened')
        step = "CIPOPEN"
        return
    end
    cmux_data_send('AT+NETOPEN\r\n')
end

local last_data_ms = millis()
local pending_to_modem = ""
local pending_to_fc = ""
local pending_to_parse = ""

--[[
    open PPP mode
--]]
local function step_PPPOPEN()
    local s = uart_read()
    if handle_error(s) then
        return
    end

    if s and s:find('CONNECT ') then
        gcs:send_text(MAV_SEVERITY.INFO, 'LTE_modem: connected')
        last_data_ms = millis()
        pending_to_modem = ""
        pending_to_fc = ""
        pending_to_parse = ""
        step = "CONNECTED"
        return
    end
    cmux_data_send('AT+CGDATA="PPP",1\r\n')
end

--[[
    open a TCP connection to the server
    the server IP and port are defined in the parameters
--]]
local function step_CIPOPEN()
    local s = uart_read()
    if handle_error(s) then
        return
    end

    if s and s:find('CONNECT ') then
        gcs:send_text(MAV_SEVERITY.INFO, 'LTE_modem: connected')
        last_data_ms = millis()
        pending_to_modem = ""
        pending_to_fc = ""
        pending_to_parse = ""
        step = "CONNECTED"
        return
    end
    if LTE_SERVER_PORT:get() <= 0 then
        gcs:send_text(MAV_SEVERITY.ERROR, "Must set LTE_SERVER_PORT")
        return
    end
    cmux_data_send(string.format('AT+CIPOPEN=0,"TCP","%d.%d.%d.%d",%d\r\n',
                                 LTE_SERVER_IP0:get(), LTE_SERVER_IP1:get(), LTE_SERVER_IP2:get(), LTE_SERVER_IP3:get(),
                                 LTE_SERVER_PORT:get()))
end

--[[
    handle AT replies in CMUX mode
--]]
local function handle_AT_reply(s)
    -- check for CSQ reply
    local rssi_raw, ber_raw = s:match("%+CSQ:%s*(%d+),(%d+)")
    if rssi_raw then
        gcs:send_named_float('LTE_RSSI', rssi_raw)
        logger:write("LTE",'RSSI,BER,Bin,Bout','iiII',
                     rssi_raw,
                     ber_raw,
                     stats.bytes_in,
                     stats.bytes_out)
        -- gcs:send_text(MAV_SEVERITY.INFO, string.format("RSSI:%d BER:%d", rssi_raw, ber_raw))
        return
    end
    -- check for CSPI reply
    -- example: +CPSI: LTE,Online,505-02,0xCBE8,36519691,101,EUTRAN-BAND3,1800,5,5,-147,-1143,-764,11
    local system_mode, operation_mode, mcc_mnc, tac_str, scell_id_str, pcid_str, earfcn_band, ul_freq_str, dl_freq_str, tdd_cfg_str, rsrp_str, rsrq_str, rssi_str, sinr_str =
    s:match("%+CPSI:%s*([^,]+),([^,]+),([^,]+),([^,]+),([^,]+),([^,]+),([^,]+),([^,]+),([^,]+),([^,]+),([%-]?%d+),([%-]?%d+),([%-]?%d+),([%-]?%d+)")

    if system_mode then
        -- Convert strings to numbers
        local tac = tonumber(tac_str:match("0x(%w+)"), 16) or tonumber(tac_str) or 0
        local scell_id = tonumber(scell_id_str) or 0
        local pcid = tonumber(pcid_str) or 0
        local ul_freq = tonumber(ul_freq_str) or 0
        local dl_freq = tonumber(dl_freq_str) or 0
        local tdd_cfg = tonumber(tdd_cfg_str) or 0
        local rsrp = tonumber(rsrp_str) or 0
        local rsrq = tonumber(rsrq_str) or 0
        local rssi = tonumber(rssi_str) or 0
        local sinr = tonumber(sinr_str) or 0
        local band = earfcn_band:match("[^%d]+(%d+)") or -1
        logger:write("LTES",'Md,Op,MCC,TAC,CID,PID,BND,F,DF,TDD,RP,RQ,RS,SR','nNNIIINHhhhhhh',
                     system_mode, operation_mode, mcc_mnc, tac, scell_id, pcid, earfcn_band,
                     ul_freq, dl_freq, tdd_cfg, rsrp, rsrq, rssi, sinr)
        if option_enabled(LTE_OPTIONS_SIGNALS) then
            gcs:send_named_float('LTE_RSRP', rsrp)
            gcs:send_named_float('LTE_RSRQ', rsrq)
            gcs:send_named_float('LTE_SINR', sinr)
            gcs:send_named_float('LTE_BAND', band)
            gcs:send_named_float('LTE_FREQ', ul_freq)
            gcs:send_named_float('LTE_CID', scell_id)
        end
        return
    end

    if s:find("PPPD: DISCONNECTED") then
        step = "PPPOPEN"
    end
end

local last_CSQ_ms = millis()
local last_CSQ_reply_ms = uint32_t(0)
local last_parse_ms = uint32_t(0)

--[[
    handle data while connected
--]]
local function step_CONNECTED()
    local s = uart:readstring(512)
    stats.bytes_in = stats.bytes_in + #s
    if option_enabled(LTE_OPTIONS_LOGALL) then
        log_data(s, '<<<')
    end
    if s and s:find('\r\nCLOSED\r\n') then
        gcs:send_text(MAV_SEVERITY.INFO, 'LTE_modem: connection closed, reconnecting')
        step = "CIPOPEN"
        return
    end
    if s and s:find('PPPD: DISCONNECTED\r\n') then
        gcs:send_text(MAV_SEVERITY.INFO, 'LTE_modem: PPP closed, reconnecting')
        step = "PPPOPEN"
        return
    end
    local now_ms = millis()
    if s and #s > 0 then
        pending_to_parse = pending_to_parse .. s
        pending_to_parse = cmux.feed_uart_in(pending_to_parse)
        if now_ms - last_parse_ms > 1000 then
            pending_to_parse = ""
        end
        if #cmux.buffers[1] > 0 then
            last_parse_ms = now_ms
            --gcs:send_text(MAV_SEVERITY.INFO, string.format("AT reply %d", #cmux.buffers[1]))
            handle_AT_reply(cmux.buffers[1])
            cmux.buffers[1] = ""
        end
        if #cmux.buffers[2] > 0 then
            last_data_ms = now_ms
            -- gcs:send_text(MAV_SEVERITY.INFO, string.format("data input %d", #cmux.buffers[2]))
            last_parse_ms = now_ms
            pending_to_fc = pending_to_fc .. cmux.buffers[2]
            cmux.buffers[2] = ""
        end
    elseif now_ms - last_data_ms > uint32_t(LTE_TIMEOUT:get() * 1000) then
        gcs:send_text(MAV_SEVERITY.ERROR, 'LTE_modem: timeout')
        step = "ATI"
        return
    end
    s = ser_device:readstring(512)
    if s then
        pending_to_modem = pending_to_modem .. s
    end

    --[[
        going via these pending buffers allows for rapid bursts of data and takes advantage
        of the hardware flow control
    --]]
    local buffer_limit = 10240 -- so we don't run out of memory
    if #pending_to_modem > buffer_limit then
        pending_to_modem = ""
    end
    if #pending_to_fc > buffer_limit then
        pending_to_fc = ""
    end
    
    while #pending_to_modem > 0 do
        local n = #pending_to_modem
        if n > 127 then
            n = 127
        end
        -- gcs:send_text(MAV_SEVERITY.INFO, string.format("data output %d", n))
        if not cmux_data_send_connected(pending_to_modem:sub(1, n)) then
            break
        end
        pending_to_modem = pending_to_modem:sub(n + 1)
    end
    if #pending_to_fc > 0 then
        local nwritten = ser_device:writestring(pending_to_fc)
        if nwritten > 0 then
            pending_to_fc = pending_to_fc:sub(nwritten + 1)
        end
    end
    -- request CSQ signal strength at 1Hz
    if now_ms - last_CSQ_ms > 1000 then
        last_CSQ_ms = now_ms
        cmux_AT_send("AT+CSQ\r\n")
        cmux_AT_send("AT+CPSI?\r\n")
    end
    if now_ms - last_CSQ_reply_ms > 5000 then
        last_CSQ_reply_ms = now_ms
        gcs:send_named_float('LTE_RSSI', -1)
    end
end

local step_count = 0
local last_step = nil

local function update()
    if LTE_ENABLE:get() == 0 then
        return update, 500
    end

    if change_baud then
        uart:begin(change_baud)
        change_baud = nil
    end

    if step == "CONNECTED" then
        -- run the connected step at 200Hz
        step_CONNECTED()
        step_count = 0
        return update, 5
    end

    -- prevent getting stuck
    if step == last_step and step ~= "ATI" then
        step_count = step_count + 1
        if step_count > 50 then
            gcs:send_text(MAV_SEVERITY.INFO, "LTE_modem: step reset")
            step = "ATI"
        end
    else
        step_count = 0
    end
    last_step = step

    gcs:send_text(MAV_SEVERITY.INFO, string.format('LTE_modem: step %s', step))

    if step == "ATI" then
        step_ATI()
        return update, 1100
    end

    if step == "BAUD" then
        step_BAUD()
        return update, 200
    end

    if step == "CREG" then
        step_CREG()
        return update, 500
    end
    
    if step == "CIPMODE" then
        step_CIPMODE()
        return update, 200
    end

    if step == "NETOPEN" then
        step_NETOPEN()
        return update, 200
    end

    if step == "CMUX" then
        step_CMUX()
        return update, 200
    end

    if step == "PPPOPEN" then
        step_PPPOPEN()
        return update, 200
    end
    
    if step == "CIPOPEN" then
        step_CIPOPEN()
        return update, 200
    end

    gcs:send_text(MAV_SEVERITY.ERROR, string.format("LTE_modem: bad step %s", step))
    step = "ATI"
end

gcs:send_text(MAV_SEVERITY.INFO, 'LTE_modem: starting')

return update,500
