-- Script decodes, checks and prints NMEA messages
-- luacheck: only 0

-- find the serial first (0) scripting serial port instance
local port = serial:find_serial(0)

if not port then
    gcs:send_text(0, "No Scripting Serial Port")
    return
end

-- begin the serial port
-- NMEA is usually 4800 or 9600
port:begin(4800)
port:set_flow_control(0)

-- table for strings used in decoding
local term = {}
local term_is_checksum = false
local term_number = 1
local checksum = 0
local string_complete = false

-- maximum number of terms we expect in the message
local max_terms = 15
-- maximum length of terms we expect
local max_term_length = 5

-- decode a basic NMEA string, only check the checksum
local function decode_NMEA(byte)
    local char = string.char(byte)
    if (char == ',' or char == '\r' or char == '\n' or char == '*') and not string_complete then
        if char == ',' then
            -- end of a term, but still counted for checksum
            checksum = checksum ~ byte
        end

        -- null terminate and decode latest term
        if term_is_checksum then
            -- test the checksum
            string_complete = true
            return checksum == tonumber(term[term_number],16)
        else
            -- we could further decode the message data here
        end

        if char == '*' then
            -- the next characters make up the checksum
            term_is_checksum = true
        end

        -- nothing in current term, add a space, makes the print work
        if not term[term_number] then
            term[term_number] = ' '
        end

        -- move onto next term
        term_number = term_number + 1

        return false
    end
    if char == '$' then
        -- sentence begin
        -- clear all flags, reset the term table and checksum
        term_is_checksum = false
        term_number = 1
        checksum = 0
        term = {}
        string_complete = false
        return false
    end
    
    -- ordinary characters are added to term
    -- if we have too many terms or they are too long then don't add to them
    if term_number < max_terms then
        if term[term_number] then
            if string.len(term[term_number]) < max_term_length then
                term[term_number] = term[term_number] .. char
            end
        else
            term[term_number] = char
        end
    end
    -- update the checksum
    if not term_is_checksum then
        -- checksum is bit wise xor of all characters in the string before the checksum
        checksum = checksum ~ byte
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
        if decode_NMEA(byte) then
            -- we have got a full NMEA message that has passed the checksum
            -- concatenate back to full message and print
            -- don't print the checksum
            gcs:send_text(0, table.concat(term,",",1,#term-1))

        end
        n_bytes = n_bytes - 1
    end

    return update, 100
end

return update, 100
