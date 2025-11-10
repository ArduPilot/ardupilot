--[[
    example of creating and sending DroneCAN messages
--]]


local MAGNETICFIELDSTRENGTHHIRES_ID = 1043
local MAGNETICFIELDSTRENGTHHIRES_SIGNATURE = uint64_t(0x3053EBE3, 0xD750286F)

local RAWAIRDATA_ID = 1027
local RAWAIRDATA_SIGNATURE = uint64_t(0xC77DF38B, 0xA122F5DA)

local PARAM_GETSET_ID = 11
local PARAM_GETSET_SIGNATURE = uint64_t(0xA7B622F9, 0x39D1A4D5)

local NODESTATUS_ID = 341
local NODESTATUS_SIGNATURE = uint64_t(0x0F0868D0, 0xC1A7C6F1)

local MAV_SEVERITY = {EMERGENCY=0, ALERT=1, CRITICAL=2, ERROR=3, WARNING=4, NOTICE=5, INFO=6, DEBUG=7}

local TARGET_NODE = Parameter('SCR_USER1')

-- a handle we will use for broadcasting, sent as CANFD
local dc_handle = DroneCAN_Handle(0, MAGNETICFIELDSTRENGTHHIRES_SIGNATURE, MAGNETICFIELDSTRENGTHHIRES_ID, true)

-- a handle for fetching parameters
local param_handle = DroneCAN_Handle(0, PARAM_GETSET_SIGNATURE, PARAM_GETSET_ID)

--[[
    setup subscription to NodeStatus
--]]
local nodestatus_handle = DroneCAN_Handle(0, NODESTATUS_SIGNATURE, NODESTATUS_ID)
nodestatus_handle:subscribe()

--[[
    setup subscription to raw air data
--]]
local airspeed_handle = DroneCAN_Handle(0, RAWAIRDATA_SIGNATURE, RAWAIRDATA_ID)
airspeed_handle:subscribe()

-- table of all nodes
local node_status = {}


--[[
    send highres mag using a global handle
--]]
local function send_mag_highres()
    local payload = string.pack("Bfff", 7, 1, 2, 3)
    dc_handle:broadcast(payload)
    gcs:send_text(MAV_SEVERITY.INFO, "mag highres broadcast done")
end

--[[
    send highres mag using a handle that will be closed after being used
--]]
local function send_mag_highres2()
    local h = DroneCAN_Handle(0, MAGNETICFIELDSTRENGTHHIRES_SIGNATURE, MAGNETICFIELDSTRENGTHHIRES_ID)
    local payload = string.pack("Bfff", 8, 10, 11, 12)
    h:broadcast(payload)
    gcs:send_text(MAV_SEVERITY.INFO, "mag highres broadcast2 done")
end

--[[
    unpack a float16 into a floating point number
--]]
local function unpackFloat16(v16)
    -- Extract the sign (bit 15), exponent (bits 10–14) and fraction (bits 0–9)
    local sign     = (v16 >> 15) & 0x1
    local exponent = (v16 >> 10) & 0x1F
    local fraction = v16 & 0x3FF

    local value
    if exponent == 0 then
        if fraction == 0 then
            -- Zero (positive or negative)
            value = 0.0
        else
            -- Subnormal numbers (exponent = -14, no implicit leading 1)
            value = (fraction / 1024.0) * 2.0^-14
        end
    elseif exponent == 0x1F then
        if fraction == 0 then
            -- Infinity (positive or negative)
            value = math.huge
        else
            -- NaN (Not a Number)
            value = 0/0
        end
    else
        -- Normalized numbers: implicit 1 before the fraction and exponent bias of 15.
        value = (1 + fraction / 1024.0) * 2.0^(exponent - 15)
    end

    -- Apply the sign bit
    if sign == 1 then
        value = -value
    end

    return value
end

--[[
    check for incoming airspeed broadcast messages
--]]
local function check_airspeed()
    local payload, nodeid = airspeed_handle:check_message()
    if not payload then
        return
    end
    local flags, static_pressure, differential_pressure, static_pressure_sensor_temperature,
        differential_pressure_sensor_temperature, static_air_temperature, pitot_temperature = string.unpack("BffHHHH", payload)
    if flags then
        local temp_C = unpackFloat16(static_air_temperature) - 273.15;
        gcs:send_text(MAV_SEVERITY.INFO, string.format("Rawairdata(%u): %f %.2fC",
                                                       nodeid, differential_pressure, temp_C))
        logger:write("ASPL", 'SP,DP,ST,DT,SAT,PIT', 'ffffff',
                     static_pressure, differential_pressure, static_pressure_sensor_temperature, differential_pressure_sensor_temperature,
                     static_air_temperature, pitot_temperature)
    end
end

--[[
    parse a parameter GetSet NumericValue
--]]
local function parse_param_NumericValue(payload, byte_offset)
    local vtype = string.unpack("B", payload, byte_offset[1])
    byte_offset[1] = byte_offset[1] + 1
    if vtype == 0 then
        return nil
    elseif vtype == 1 then
        -- integer (treat as 32 bit for now, actually 64 bit)
        local ret = string.unpack("i", payload, byte_offset[1])
        byte_offset[1] = byte_offset[1] + 8
        return ret
    elseif vtype == 2 then
        -- float32
        local ret = string.unpack("f", payload, byte_offset[1])
        byte_offset[1] = byte_offset[1] + 4
        return ret
    else
        return nil
    end
end

--[[
    parse a parameter GetSet Value
--]]
local function parse_param_Value(payload, byte_offset)
    local vtype = string.unpack("B", payload, byte_offset[1])
    byte_offset[1] = byte_offset[1] + 1
    if vtype == 0 then
        return nil
    elseif vtype == 1 then
        -- signed integer (64 bit), return as signed 32 bit number
        local ret_lo, ret_hi = string.unpack("II", payload, byte_offset[1])
        byte_offset[1] = byte_offset[1] + 8
        if ret_hi & 0x80000000 then
            return -ret_lo
        end
        return ret_lo
    elseif vtype == 2 then
        -- float32
        local ret = string.unpack("f", payload, byte_offset[1])
        byte_offset[1] = byte_offset[1] + 4
        return ret
    elseif vtype == 3 then
        -- bool
        local v = string.unpack("B", payload, byte_offset[1])
        byte_offset[1] = byte_offset[1] + 1
        return v == 1
    elseif vtype == 4 then
        -- string
        local slen = string.unpack("B", payload, byte_offset[1])
        local ret = string.sub(payload, byte_offset[1]+1, slen+2)
        byte_offset[1] = byte_offset[1] + 1 + slen
        return ret
    else
        return nil
    end
end


--[[
    parse a parameter GetSet reply
--]]
local function parse_param_reply(payload)
    local byte_offset = {1}
    local value = parse_param_Value(payload, byte_offset)
    local default_value = parse_param_Value(payload, byte_offset)
    local max_value = parse_param_NumericValue(payload, byte_offset)
    local min_value = parse_param_NumericValue(payload, byte_offset)
    local name = string.sub(payload, byte_offset[1], #payload)
    return name, value, default_value, min_value, max_value
end

local next_param_index = 0

--[[
    encode a 16 bit number as a DroneCAN int13
--]]
local function encode_int13(v)
    return (v & 0xFF) | (v&0xFF00)<<3
end

local function fetch_param()
    local payload, nodeid = param_handle:check_message()
    if payload then
        local pname, pvalue = parse_param_reply(payload)
        if not pname or not pvalue then
            gcs:send_text(MAV_SEVERITY.INFO, string.format("param restart loop %u", next_param_index))
            next_param_index = -1
        elseif type(pvalue) == "string" then
            gcs:send_text(MAV_SEVERITY.INFO, string.format("got param reply from %u idx=%u '%s' : %s", nodeid, next_param_index, pname, pvalue))
        else
            gcs:send_text(MAV_SEVERITY.INFO, string.format("got param reply from %u idx=%u '%s' : %f", nodeid, next_param_index, pname, pvalue))
        end
        next_param_index = next_param_index + 1
    end
    param_handle:request(TARGET_NODE:get(), string.pack("H",encode_int13(next_param_index)))
end

--[[
    check for new NodeStatus messages
--]]
local function check_node_status()
    local payload, nodeid = nodestatus_handle:check_message()
    if not payload then
        return
    end
    local uptime_sec, bits, _ = string.unpack("IBH", payload)
    local health = bits&3
    local mode = (bits>>2)&7
    local sub_mode = (bits>>5)&7
    if not node_status[nodeid] then
        gcs:send_text(MAV_SEVERITY.INFO, string.format("Found node %u", nodeid))
    end
    node_status[nodeid] = { uptime_sec=uptime_sec, health=health, mode=mode, sub_mode=sub_mode }
end

local last_low_rate_ms = uint32_t(0)

local function update()
    local now = millis()
    if now - last_low_rate_ms >= 1000 then
        last_low_rate_ms = now
        send_mag_highres()
        send_mag_highres2()
        check_airspeed()
    end
    check_node_status()
    fetch_param()

    return update, 10
end

return update, 1000
