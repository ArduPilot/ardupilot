-- mavlink-tlog-reader.lua
--------------------------------------------------------------------------------
--[[
    This is a Wireshark file reader for Mavlink Telemetry Log (tlog) files.
    Author: Simon Hancock
--]]
--------------------------------------------------------------------------------

--------------------------------------------------------------------------------
-- set up debugging level
--------------------------------------------------------------------------------

local debug = {
    DISABLED = 0,
    LEVEL_1  = 1,
    LEVEL_2  = 2
}

-- set this DEBUG to debug.LEVEL_1 to enable printing debug info
-- set it to debug.LEVEL_2 to enable really verbose printing
local DEBUG = debug.LEVEL_1


local dprint = function() end
local dprint2 = function() end
local function reset_debug()
    if DEBUG > debug.DISABLED then
        dprint = function(...)
            print(table.concat({"Lua:", ...}," "))
        end

        if DEBUG > debug.LEVEL_1 then
            dprint2 = dprint
        end
    end
end
-- call it now
reset_debug()

--------------------------------------------------------------------------------
-- check for compatability with this version of Wireshark
--------------------------------------------------------------------------------

-- Set the name of Wireshark or Tshark application, for use in error
-- messages
local wireshark_name = "Wireshark"
if not GUI_ENABLED then
    wireshark_name = "Tshark"
end

-- verify Wireshark is new enough
local major, minor, micro = get_version():match("(%d+)%.(%d+)%.(%d+)")
if major and tonumber(major) <= 1 and ((tonumber(minor) <= 10) or (tonumber(minor) == 11 and tonumber(micro) < 3)) then
        error(  "Sorry, but your " .. wireshark_name .. " version (" .. get_version() .. ") is too old for this script!\n" ..
                "This script needs " .. wireshark_name .. "version 1.11.3 or higher.\n" )
end

-- verify we have the FileHandler class in wireshark
assert(FileHandler.new, wireshark_name .. " does not have the FileHandler class!")

--------------------------------------------------------------------------------
-- definitions
--------------------------------------------------------------------------------

-- declare name of functions we'll define later
local read_common

-- handy consts
MAVLINK1_MAGIC_BYTE = 0xfe
MAVLINK1_FRAME_OVERHEAD = 8
MAVLINK2_MAGIC_BYTE = 0xfd
MAVLINK2_FRAME_OVERHEAD = 12
MAVLINK2_SIGNATURE_LEN = 13

--------------------------------------------------------------------------------
-- file reader handling functions for Wireshark to use
--------------------------------------------------------------------------------

----------------------------------------
-- The read_open() is called by Wireshark once per file, to see if the file is this reader's type.
-- Wireshark passes in (1) a File object and (2) CaptureInfo object to this function
-- It expects in return either nil or false to mean it's not our file type, or true if it is
-- We do a quick check of the file to make sure that a Mavlink magic byte exists at location 9 -
-- i.e. directly after the 8-byte timestamp.
local function read_open(file, capture)
    dprint2("read_open() called")

    -- read 9 bytes from the start of the file
    local line = file:read(9)

    -- if we couldn't read these bytes, return false
    if not line then return false end

    -- unpack into 8 byte timestamp and 1 byte Mavlink magic byte
    local fields = { Struct.unpack(">E I1", line) }
    
    -- check we got what we expected (note that #fields is the number of returned fields PLUS ONE)
    if #fields ~= 3 then
        dprint2("Incorrect number of fields fields returned: " .. #fields)
        return false
    elseif fields[2] ~= MAVLINK2_MAGIC_BYTE and fields[2] ~= MAVLINK1_MAGIC_BYTE then
        dprint2("No Mavlink magic byte " .. fields[1])
        return false
    end
    
    dprint2("read_open: success, file is for us")

    -- if the file is for us, we MUST set the file position cursor to
    -- where we want the first call to read() function to get it the next time
    file:seek("set",0)

    -- these we can also set per record later during read operations
    capture.time_precision  = wtap_filetypes.TSPREC_USEC
    capture.encap           = wtap_encaps.USER0 -- we use USER0 for Mavlink

    return true
end

----------------------------------------
-- Wireshark/tshark calls read() for each frame/record in the file
-- It passes in (1) a File, (2) CaptureInfo, and (3) FrameInfo object to this function
-- It expects in return the file offset position the record starts at,
-- or nil/false if there's an error or end-of-file is reached.
-- The offset position is used later: wireshark remembers it and gives
-- it to seek_read() at various random times
local function read(file, capture, frame)
    dprint2("read() called")

    -- remember where we are in the file
    local position = file:seek()

    -- call our common reader function
    if not read_common("read", file, capture, frame) then
        -- this isnt' actually an error, because it might just mean we reached end-of-file
        -- so let's test for that (read(0) is a special case in Lua, see Lua docs)
        if file:read(0) ~= nil then
            dprint("read: failed to call read_common")
        else
            dprint2("read: reached end of file")
        end
        return false
    end

    dprint2("read: success")

    -- return the position we got to (or nil if we hit EOF/error)
    return position
end

----------------------------------------
-- Wireshark/tshark calls seek_read() for each frame/record in the file, at random times
-- It passes in (1) a File, (2) CaptureInfo, (3) FrameInfo object, and the offset position number
-- It expects in return true for successful parsing, or nil/false if there's an error.
local function seek_read(file, capture, frame, offset)
    dprint2("seek_read() called")

    -- first move to the right position in the file
    file:seek("set",offset)

    -- call our common reader function
    if not read_common("seek_read", file, capture, frame) then
        dprint("seek_read: failed to call read_common")
        return false
    end

    return true
end

----------------------------------------
-- Wireshark/tshark calls read_close() when it's closing the file completely
-- It passes in (1) a File and (2) CaptureInfo object to this function
-- this is a good opportunity to clean up any state you may have created during
-- file reading. (in our case there's no real state)
local function read_close(file, capture)
    dprint2("read_close() called")
    return true
end

----------------------------------------
-- An often unused function, Wireshark calls this when the sequential walk-through is over
-- (i.e., no more calls to read(), only to seek_read()).
-- It passes in (1) a File and (2) CaptureInfo object to this function
-- This gives you a chance to clean up any state you used during read() calls, but remember
-- that there will be calls to seek_read() after this (in Wireshark, though not Tshark)
local function seq_read_close(file, capture)
    dprint2("First pass of read() calls are over, but there may be seek_read() calls after this")
    return true
end

----------------------------------------
-- internal functions declared previously
----------------------------------------

----------------------------------------
-- this is used by both read() and seek_read()
-- the calling function to this should have already set the file position correctly
read_common = function(funcname, file, capture, frame)
    dprint2(funcname,": read_common() called")

    -- remember where we are in the file, as we'll need to come back here
    -- once we figure out the message length
    local position = file:seek()
    
    -- read 11 bytes from the start of the record
    local line = file:read(11)

    -- it's ok for us to not be able to read it, if it's end of file
    if not line then return false end

    -- this is: time_usec (8 bytes), Mavlink magic byte, message length, incompat_flags
    local fields = { Struct.unpack(">E I1 I1 I1", line) }
    if #fields ~= 5 then -- #fields is number of returned fields PLUS ONE
        dprint(funcname, ": read_common: incorrect number of fields returned: " .. #fields)
        return false
    end

    -- now move back to where we were, plus the 8 bytes of timestamp
    file:seek('set', position + 8)

    -- set the timestamp of this frame
    local ts = fields[1]:tonumber()
    local sec = math.floor(ts / 1000000.0)
    local usec = ((ts % 1000000) * 1000)
    frame.time = NSTime(sec, usec)

    -- set the captured length and original length of this frame
    -- (we add the Mavlink header length to the message length)
    if fields[2] == MAVLINK2_MAGIC_BYTE then
        if fields[4] & 0x01 ~= 0 then
            frame.captured_length = fields[3] + MAVLINK2_FRAME_OVERHEAD + MAVLINK2_SIGNATURE_LEN
        else
            frame.captured_length = fields[3] + MAVLINK2_FRAME_OVERHEAD
        end
    elseif fields[2] == MAVLINK1_MAGIC_BYTE then
        frame.captured_length = fields[3] + MAVLINK1_FRAME_OVERHEAD
    else
        dprint("Unrecognised Mavlink magic byte " .. fields[2])
        return false
    end
    frame.original_length = frame.captured_length

    -- set the flags to say we have a timestamp
    frame.flags = wtap_presence_flags.TS

    -- now we need to get the packet bytes from the file record into the frame...
    -- we *could* read them into a string using file:read(numbytes), and then
    -- set them to frame.data so that wireshark gets it...
    -- but that would mean the packet's string would be copied into Lua
    -- and then sent right back into wireshark, which is gonna slow things
    -- down; instead FrameInfo has a read_data() method, which makes
    -- wireshark read directly from the file into the frame buffer, so we use that
    if not frame:read_data(file, frame.captured_length) then
        dprint(funcname, ": read_common: failed to read data from file into buffer")
        return false
    end

    return true
end

--------------------------------------------------------------------------------
-- register the FileHandler with Wireshark
--------------------------------------------------------------------------------

----------------------------------------
-- ok, so let's create a FileHandler object
--
-- note the last argument defines our file handler type (r=reader), and its magic/heuristic
-- capability - this is used to determine the ordering of this file reader compared with
-- the built-in file readers in wireshark. When a user opens a file, wireshark automatically
-- tries to find a matching file reader for the file. The file extensions are used as hints,
-- but really the ordering is more important. So wireshark first tries all the "magic" readers,
-- meaning ones that can figure out if a file is their format based on a magic byte sequence
-- at the beginning of the file, and then wireshark runs through the "heuristic" file readers,
-- meaning ones that have to guess if the file is their format or not. The ordering of these
-- is very important, since a weak heuristic will incorrectly think a file is its format and
-- prevent a lower-ordered file reader from getting to check it. So if your file reader uses
-- a magic value, you should use the "m" flag, and if it uses a heuristic then no flag means
-- a weak heuristic, whereas a "s" means a strong one.
--
-- We consider our file format to have weak heuristic, as we rely only on byte 9 being a
-- Mavlink magic byte.
local fh = FileHandler.new("Mavlink Telemetry log file reader", "tlog", "A Lua-based file reader for Ardupilot DF log file","r")

-- set above functions to the FileHandler
fh.read_open = read_open
fh.read = read
fh.seek_read = seek_read
fh.read_close = read_close
fh.seq_read_close = seq_read_close
fh.extensions = "tlog" -- this is just a hint

-- and finally, register the FileHandler!
register_filehandler(fh)

dprint2("Mavlink TLOG FileHandler registered")
