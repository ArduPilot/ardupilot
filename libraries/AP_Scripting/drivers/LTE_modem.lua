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
    // @Description: Baud rate for the serial port to the LTE modem. If using something other than 115200 you need to connect to the modem and use AT+IPREX=BAUD to set the baud rate and save with AT&W. The fastest supported baudrate is 3686400.
    // @Values: 19200:19200,38400:38400,57600:57600,115200:115200,230400:230400,460800:460800,921600:921600,3686400:3686400
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
    // @Description: Options to control the LTE modem driver
    // @Bitmask: 0:LogAllData
    // @User: Standard
--]]
local LTE_OPTIONS     = bind_add_param('OPTIONS', 12, 0)

local LTE_OPTIONS_LOGALL = (1<<0)

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

uart:begin(LTE_BAUD:get())

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
    return s
end

--[[
    Function to write to the UART and log the command
--]]
local function uart_write(s)
    uart:writestring(s)
    log_data(s, '>>>')
    return s
end

--[[
    Function to handle errors in the response from the modem
    If an error is detected, it resets the modem
    returns true if an error was detected
--]]
local function handle_error(s)
    if s and s:find('\nERROR\r\n') then
        gcs:send_text(MAV_SEVERITY.ERROR, 'LTE_modem: error response from modem')
        uart_write('ATH\r\nAT+CRESET;\r\n')
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
local function step_ATI()
    local s = uart_read()
    if s and s:find('IMEI: ') then
        gcs:send_text(MAV_SEVERITY.INFO, 'LTE_modem: found modem')
        step = "CREG"
        return
    end
    if ati_sequence == 1 then
        uart_write('+++')
        ati_sequence = 0
    else
        uart_write('\r\nATI\r\n')
        ati_sequence = 1
    end
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
    uart_write('AT+CREG?\r\n')
end

--[[
    setup automatic signal reporting
--]]
local function step_AUTOCSQ()
    local s = uart_read()
    if handle_error(s) then
        return
    end
    if s and s:find('AUTOCSQ=1,1\r\r\nOK\r') then
        gcs:send_text(MAV_SEVERITY.INFO, 'LTE_modem: AUTOCSQ OK')
        if LTE_PROTOCOL:get() == PPP then
            step = "PPPOPEN"
        else
            step = "CIPMODE"
        end
        return
    end
    uart_write('AT+AUTOCSQ=1,1\r\n')
end

--[[
    set the modem to transparent mode
--]]
local function step_CIPMODE()
    local s = uart_read()
    if handle_error(s) then
        return
    end
    if s and s:find('CIPMODE=1\r\r\nOK\r') then
        gcs:send_text(MAV_SEVERITY.INFO, 'LTE_modem: transparent mode set')
        step = "NETOPEN"
        return
    end
    uart_write('AT+CIPMODE=1\r\n')
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
    if s and s:find('NETOPEN\r\r\nOK\r\n') then
        gcs:send_text(MAV_SEVERITY.INFO, 'LTE_modem: network opened')
        step = "CIPOPEN"
        return
    end
    uart_write('AT+NETOPEN\r\n')
end

local last_data_ms = millis()
local pending_to_modem = ""
local pending_to_fc = ""

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
        step = "CONNECTED"
        return
    end
    uart_write('AT+CGDATA="PPP",1\r\n')
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
        step = "CONNECTED"
        return
    end
    if LTE_SERVER_PORT:get() <= 0 then
        gcs:send_text(MAV_SEVERITY.ERROR, "Must set LTE_SERVER_PORT")
        return
    end
    uart_write(string.format('AT+CIPOPEN=0,"TCP","%d.%d.%d.%d",%d\r\n',
                                   LTE_SERVER_IP0:get(), LTE_SERVER_IP1:get(), LTE_SERVER_IP2:get(), LTE_SERVER_IP3:get(),
                                   LTE_SERVER_PORT:get()))
end

--[[
    handle data while connected
--]]
local function step_CONNECTED()
    local s = uart:readstring(512)
    if LTE_OPTIONS:get() & LTE_OPTIONS_LOGALL then
        log_data(s, '<<<')
    end
    if s and s:find('\r\nCLOSED\r\n') then
        gcs:send_text(MAV_SEVERITY.INFO, 'LTE_modem: connection closed, reconnecting')
        step = "CIPOPEN"
        return
    end
    local now_ms = millis()
    if s and #s > 0 then
        last_data_ms = now_ms
        pending_to_fc = pending_to_fc .. s
    elseif now_ms - last_data_ms > uint32_t(LTE_TIMEOUT:get() * 1000) then
        gcs:send_text(MAV_SEVERITY.ERROR, 'LTE_modem: timeout')
        step = "ATI"
        return
    end
    s = ser_device:readstring(512)
    if LTE_OPTIONS:get() & LTE_OPTIONS_LOGALL then
        log_data(s, '>>>')
    end
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

    if step == "CONNECTED" then
        -- run the connected step at 200Hz
        step_CONNECTED()
        return update, 5
    end

    gcs:send_text(MAV_SEVERITY.INFO, string.format('LTE_modem: step %s', step))

    if step == "ATI" then
        step_ATI()
        return update, 1100
    end

    if step == "CREG" then
        step_CREG()
        return update, 500
    end

    if step == "AUTOCSQ" then
        step_AUTOCSQ()
        return update, 500
    end
    
    if step == "CIPMODE" then
        step_CIPMODE()
        return update, 500
    end

    if step == "NETOPEN" then
        step_NETOPEN()
        return update, 500
    end

    if step == "PPPOPEN" then
        step_PPPOPEN()
        return update, 500
    end
    
    if step == "CIPOPEN" then
        step_CIPOPEN()
        return update, 500
    end

    gcs:send_text(MAV_SEVERITY.ERROR, string.format("LTE_modem: bad step %s", step))
    step = "ATI"
end

gcs:send_text(MAV_SEVERITY.INFO, 'LTE_modem: starting')

return update,500
