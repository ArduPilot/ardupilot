-- Script decodes and logs wind sensor data for Trisonica LI-550 mini
-- https://www.licor.com/products/trisonica/LI-550-mini

-- Parameters:
-- SCR_ENABLE 1
-- SERIAL5_PROTOCOL 28

-- In SITL, you can enable serial ports to connect to the real device.
-- https://ardupilot.org/dev/docs/learning-ardupilot-uarts-and-the-console.html#the-8-uarts
-- ./Tools/autotest/sim_vehicle.py -v Plane --console --map -A "--serial5=uart:/dev/ttyUSB0" -D

-- Remember to change baud to 230k in the sensor setup and enable the fields you want.
-- Also, enable 10Hz output instead of the default 5Hz.

-- Example data string (excluding quotes, excluding the carriage return line feed ending)
-- "S  00.08 S2  00.07 D  245 DV  033 U  00.06 V  00.03 W  00.05 T  55889220.00 C  346.68 H  17.92 DP  03.68 P -099.70 AD  0.0000000 AX  -2913 AY  -3408 AZ -16600 PI  011.4 RO  009.8 MX   -619 MY    845 MZ    782 MD  337 TD  337"

-- Log severities
local MAV_SEVERITY = {EMERGENCY=0, ALERT=1, CRITICAL=2, ERROR=3, WARNING=4, NOTICE=5, INFO=6, DEBUG=7}

-- find the serial first (0) scripting serial port instance
local port = serial:find_serial(0)

if not port then
    gcs:send_text(MAV_SEVERITY.EMERGENCY, "no trisonica scripting port")
    return
end

-- begin the serial port
port:begin(230400)
port:set_flow_control(0)

local function parse_wind_data(buf)
    -- Split the string up into key and values splitting on the default space delimiter.

    local parsed_values = {}

    -- Match any key-value pair where key is a string and value is a number
    for key, value in buf:gmatch("(%a+)%s*([%-%.%d]+)") do
        parsed_values[key] = tonumber(value) -- Store key-value pair in the table
    end

    return parsed_values
end

local tag_ids = nil
local tag_id_str = ""
local value_format = ""
local last_keys = nil

local function log_wind_data(parsed)
    -- Given a table of parsed data, log it.

    -- Collect keys from parsed and store as a list
    local current_keys = {}
    for k in pairs(parsed) do
        table.insert(current_keys, k)
    end
    table.sort(current_keys)

    -- First packet, we must set last_keys.
    if not last_keys then
        last_keys = current_keys
        return
    end

    -- Bail early if the first packet parsed only contained some of the keys. 
    -- After the 2nd packet, it will stabilize.
    if #current_keys ~= #last_keys then
        last_keys = current_keys
        return
    end

    for i = 1, #current_keys do
        if current_keys[i] ~= last_keys[i] then
            last_keys = current_keys
            return
        end
    end

    -- Keys are now stable, so store them as tag_ids
    if not tag_ids then
        tag_ids = current_keys
        tag_id_str = table.concat(tag_ids, ',')
        value_format = string.rep('f', #tag_ids)
        gcs:send_text(MAV_SEVERITY.INFO, "Using tag_ids: " .. tag_id_str)
    end
    

    -- Build ordered values from tag_ids
    local values = {}
    for _, tag_id in ipairs(tag_ids) do
        table.insert(values, parsed[tag_id] or 0)
    end

    assert(#tag_ids < 15, "#tag_ids=" .. #tag_ids)
    logger:write('W3D', tag_id_str, value_format, table.unpack(values))
end

local buffer = ""
-- the main update function that is used to read in data from serial port
local function update()


    local n_bytes = port:available()
    while n_bytes > 0 do
        local byte = port:read()

        if byte > -1 then
            local c = string.char(byte)
            buffer = buffer .. c

            --ignore the \n because \r is sufficient to end parsing.
            if c == "\r" then
                local result = parse_wind_data(buffer)
                log_wind_data(result)
                buffer = ""
            end
        end
        n_bytes = n_bytes - 1
    end
    return update, 100
end

gcs:send_text(MAV_SEVERITY.INFO, "trisonica-mini.lua running...")

return update, 100
