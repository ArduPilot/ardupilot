-- how to use:
-- set SCR_SDEV_EN 1
-- set the port the device is on to protocol 28 (ensure no others are set to 28)
-- set SCR_SDEV1_PROTO to the original device protocol
local PROTO = -1 -- set to original device protocol
local BAUD = -1 -- set to original device baud
-- WARNING: latency or errors may crash the drone if this is a critical sensor!

local p_dev = serial:find_serial(0) -- first port with protocol 28
local p_fc = serial:find_simulated_device(PROTO, 0) -- first sim port with PROTO

if p_dev == nil or p_fc == nil then
    error("bad config")
end
p_dev:begin(BAUD)

-- takes data from the device and returns data for the flight controller
local function intercept_dev2fc(data)
    return data -- pass along input data unmodified, feel free to customize!
end

-- takes data from the flight controller and returns data for the device
local function intercept_fc2dev(data)
    return data -- pass along input data unmodified, feel free to customize!
end

local buf2dev = nil
local buf2fc = nil

local function process_side(port, func)
    -- 512 is the max size without an additional allocation
    local data = port:readstring(512)
    if data == nil or #data == 0 then
        return nil
    end

    data = func(data) -- process through the intercept function
    if data ~= nil and #data == 0 then
        data = nil
    end

    return data
end

local function flush_side(buf, port)
    local written = port:writestring(buf) -- write what we can
    if written == #buf then
        return nil -- wrote all of it
    end
    return buf:sub(written) -- return the remainder
end

local function update()
    -- if there is no data buffered, get new data and process it
    if buf2dev == nil then
        buf2dev = process_side(p_fc, intercept_fc2dev)
    end
    if buf2fc == nil then
        buf2fc = process_side(p_dev, intercept_dev2fc)
    end
    
    -- flush buffers
    if buf2dev ~= nil then
        buf2dev = flush_side(buf2dev, p_dev)
    end
    if buf2fc ~= nil then
        buf2fc = flush_side(buf2fc, p_fc)
    end

    return update, 1
end

print("intercept started")
return update()
