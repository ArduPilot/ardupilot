-- Reads data in from UART and logs to dataflash

-- find the serial first (0) scripting serial port instance
local port = serial:find_serial(0)

if not port or baud == 0 then
    gcs:send_text(0, "No Scripting Serial Port")
    return
end

-- begin the serial port
port:begin(9600)
port:set_flow_control(0)

-- table for strings used in decoding
local log_data = {}
local term_number = 1
local valid = true
local term

-- number of terms we expect in the message
local num_terms = 3
-- maximum length of terms each term we expect
local max_length = 20

-- decode a basic string
local function decode(byte)
    local char = string.char(byte)
    if char == '\r' or char == '\n' or char == ',' then

        -- decode the term, note this assumes it is a number
        log_data[term_number] = tonumber(term)
        if not log_data[term_number] then
            -- could not convert to a number, discard this message
            valid = false
        end
        term = nil

        -- not got to the end yet
        if char == ',' then
            -- move onto next term
            if term_number < num_terms then
                term_number = term_number + 1
            end
            return false
        end

        -- make sure we have the correct number of terms
        if #log_data ~= num_terms then
            valid = false
        end

        if not valid then
            log_data = {}
        end

        -- reset for the next message
        local is_valid = valid
        term_number = 1
        valid = true

        return is_valid
    end

    -- ordinary characters are added to term
    -- if we have too many terms or they are too long then don't add to them
    if term_number <= num_terms then
        if term then
            if string.len(term) < max_length then
                term = term .. char
            else
                valid = false
            end
        else
            term = char
        end
    else
        valid = false
    end

    return false
end

-- the main update function that is used to read in data from serial port
function update()

    if not port then
        gcs:send_text(0, "no Scripting Serial Port")
        return update, 100
    end

    local n_bytes = port:available()
    while n_bytes > 0 do
        local byte = port:read()
        if decode(byte) then
            -- we have got a full line
            -- save to data flash

            -- care must be taken when selecting a name, must be less than four characters and not clash with an existing log type
            -- format characters specify the type of variable to be logged, see AP_Logger/README.md
            -- not all format types are supported by scripting only: i, L, e, f, n, M, B, I, E, N, and Z
            -- Note that Lua automatically adds a timestamp in micro seconds
            logger:write('SCR','Sensor1, Sensor2, Sensor3','fff',table.unpack(log_data))

            -- reset for the next message
            log_data = {}
        end
        n_bytes = n_bytes - 1
    end

    return update, 100
end

return update, 100
