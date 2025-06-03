--[[
    driver for SIM7600 LTE modems
--]]

local MAV_SEVERITY = {EMERGENCY=0, ALERT=1, CRITICAL=2, ERROR=3, WARNING=4, NOTICE=5, INFO=6, DEBUG=7}

local PARAM_TABLE_KEY = 106
local PARAM_TABLE_PREFIX = "LTE_"

-- add a parameter and bind it to a variable
local function bind_add_param(name, idx, default_value)
    assert(param:add_param(PARAM_TABLE_KEY, idx, name, default_value), string.format('could not add param %s', name))
    return Parameter(PARAM_TABLE_PREFIX .. name)
end

-- Setup Parameters
assert(param:add_table(PARAM_TABLE_KEY, PARAM_TABLE_PREFIX, 20), 'SIM7600: could not add param table')

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
local LTE_SERVER_IP0  = bind_add_param('SERVER_IP0',  4, 157)

--[[
    // @Param: LTE_SERVER_IP1
    // @DisplayName: Server IP 1
    // @Description: Second octet of the server IP address to connect to
    // @Range: 0 255
    // @User: Standard
--]]
local LTE_SERVER_IP1  = bind_add_param('SERVER_IP1',  5, 245)

--[[
    // @Param: LTE_SERVER_IP2
    // @DisplayName: Server IP 2
    // @Description: Third octet of the server IP address to connect to
    // @Range: 0 255
    // @User: Standard
--]]
local LTE_SERVER_IP2  = bind_add_param('SERVER_IP2',  6, 83)

--[[
    // @Param: LTE_SERVER_IP3
    // @DisplayName: Server IP 3
    // @Description: Fourth octet of the server IP address to connect to
    // @Range: 0 255
    // @User: Standard
--]]
local LTE_SERVER_IP3  = bind_add_param('SERVER_IP3',  7, 174)

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
    // @Description: Baud rate for the serial port to the LTE modem. If using something other than 115200 you need to connect to the modem and use AT+IPREX=BAUD to set the baud rate and save with AT&W
    // @Values: 19200:19200,38400:38400,57600:57600,115200:115200,230400:230400,460800:460800,921600:921600
    // @User: Standard
--]]
local LTE_BAUD        = bind_add_param('BAUD',  9, 115200)

--[[
    // @Param: LTE_TIMEOUT
    // @DisplayName: Timeout
    // @Description: Timeout in seconds for the LTE connection. If no data is received for this time, the connection will be reset.
    // @Range: 1 60
    // @Units: s
    // @User: Standard
--]]
local LTE_TIMEOUT     = bind_add_param('TIMEOUT', 10, 10)

local uart = serial:find_serial(LTE_SERPORT:get())
if not uart then
    gcs:send_text(MAV_SEVERITY.ERROR, 'SIM7600: could not find serial port')
    return
end

local MAVLINK2 = 2

local ser_device = serial:find_simulated_device(MAVLINK2, LTE_SCRPORT:get())
if not ser_device then
    gcs:send_text(MAV_SEVERITY.ERROR, 'SIM7600: could not find simulated device')
    return
end

--[[
    steps to connect the modem:
    1. send "ATI" to get modem info and confirm communication
    2. send AT+CIPMODE=1 to set to transparent mode
    3. send AT+NETOPEN to open the network stack
    4. send AT+CIPOPEN=0,"TCP","<server_ip>",<server_port> to open a TCP connection

    Once connected we have a transparent connection to the server.
    If we see "\r\nCLOSED\r\n" we need to reconnect.
--]]

local step = "ATI"

uart:begin(LTE_BAUD:get())

--[[
    Open a log file to log the output from the modem
    This is useful for debugging the connection process
--]]
local log_file = io.open('SIM7600.log', 'w')

--[[
    Function to read from the UART and log the output
    This function reads up to 512 bytes at a time and writes it to the log file
    returns the string read or nil
--]]
local function uart_read()
    local s = uart:readstring(512)
    if s and #s > 0 and log_file then
        log_file:write('[' .. s .. ']\n')
        log_file:flush()
    end
    return s
end

--[[
    Function to handle errors in the response from the modem
    If an error is detected, it resets the modem
    returns true if an error was detected
--]]
local function handle_error(s)
    if s and s:find('\nERROR\r\n') then
        gcs:send_text(MAV_SEVERITY.ERROR, 'SIM7600: error response from modem')
        uart:writestring('AT+CRESET\r\n')
        step = "ATI"
        return true
    end
    return false
end

local ati_sequence = 0

--[[
    Function to confirm the connection to the modem
    it uses AIT command to get the modem info, and +++ if needed
    to break out of transparent mode
--]]
local function confirm_connection()
    local s = uart_read()
    if s and s:find('IMEI: ') then
        gcs:send_text(MAV_SEVERITY.INFO, 'SIM7600: found modem')
        step = "CREG"
        return
    end
    if ati_sequence == 1 then
        uart:writestring('+++')
        ati_sequence = 0
    else
        uart:writestring('\r\nATI\r\n')
        ati_sequence = 1
    end
end

--[[
    confirm we are registered on the network
--]]
local function confirm_registration()
    local s = uart_read()
    if handle_error(s) then
        return
    end
    if s and s:find('CREG: 0,1\r\n') then
        gcs:send_text(MAV_SEVERITY.INFO, 'SIM7600: CREG OK')
        step = "CIPMODE"
        return
    end
    uart:writestring('AT+CREG?\r\n')
end

--[[
    set the modem to transparent mode
--]]
local function set_transparent()
    local s = uart_read()
    if handle_error(s) then
        return
    end
    if s and s:find('CIPMODE=1\r\r\nOK\r') then
        gcs:send_text(MAV_SEVERITY.INFO, 'SIM7600: transparent mode set')
        step = "NETOPEN"
        return
    end
    uart:writestring('AT+CIPMODE=1\r\n')
end

--[[
    open the network stack
    needed to be able to open a TCP connection
--]]
local function open_network()
    local s = uart_read()
    if handle_error(s) then
        return
    end
    if s and s:find('NETOPEN\r\r\nOK\r\n') then
        gcs:send_text(MAV_SEVERITY.INFO, 'SIM7600: network opened')
        step = "CIPOPEN"
        return
    end
    uart:writestring('AT+NETOPEN\r\n')
end

local last_data_ms = millis()
local pending_to_modem = ""
local pending_to_fc = ""

--[[
    open a TCP connection to the server
    the server IP and port are defined in the parameters
--]]
local function open_connection()
    local s = uart_read()
    if handle_error(s) then
        return
    end

    if s and s:find('CONNECT ') then
        gcs:send_text(MAV_SEVERITY.INFO, 'SIM7600: connected')
        last_data_ms = millis()
        pending_to_modem = ""
        pending_to_fc = ""
        step = "CONNECTED"
        return
    end
    if LTE_SERVER_PORT:get() <= 0 then
        gcs:send_text(MAV_SEVERITY.ERROR, "Must set LTE_SERVER_PORT")
        return
    end
    uart:writestring(string.format('AT+CIPOPEN=0,"TCP","%d.%d.%d.%d",%d\r\n',
                                   LTE_SERVER_IP0:get(), LTE_SERVER_IP1:get(), LTE_SERVER_IP2:get(), LTE_SERVER_IP3:get(),
                                   LTE_SERVER_PORT:get()))
end

--[[
    handle data while connected
--]]
local function handle_connection()
    local s = uart:readstring(512)
    if s and s:find('\r\nCLOSED\r\n') then
        gcs:send_text(MAV_SEVERITY.INFO, 'SIM7600: connection closed, reconnecting')
        step = "CIPOPEN"
        return
    end
    if s and #s > 0 then
        last_data_ms = millis()
        pending_to_fc = pending_to_fc .. s
    elseif millis() - last_data_ms > uint32_t(LTE_TIMEOUT:get() * 1000) then
        gcs:send_text(MAV_SEVERITY.ERROR, 'SIM7600: timeout')
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
    
    if #pending_to_modem > 0 then
        local nwritten = uart:writestring(pending_to_modem)
        if nwritten > 0 then
            pending_to_modem = pending_to_modem:sub(nwritten + 1)
        end
    end
    if #pending_to_fc > 0 then
        local nwritten = ser_device:writestring(pending_to_fc)
        if nwritten > 0 then
            pending_to_fc = pending_to_fc:sub(nwritten + 1)
        end
    end
end

local function update()
    if LTE_ENABLE:get() == 0 then
        return update, 500
    end
    if step ~= "CONNECTED" then
        gcs:send_text(MAV_SEVERITY.INFO, string.format('SIM7600: step %s', step))
    end

    if step == "ATI" then
        confirm_connection()
        return update, 1100
    end

    if step == "CREG" then
        confirm_registration()
        return update, 500
    end
    
    if step == "CIPMODE" then
        set_transparent()
        return update, 500
    end

    if step == "NETOPEN" then
        open_network()
        return update, 500
    end

    if step == "CIPOPEN" then
        open_connection()
        return update, 500
    end

    if step == "CONNECTED" then
        handle_connection()
        return update, 10
    end
end

gcs:send_text(MAV_SEVERITY.INFO, 'SIM7600: starting')

return update,500
