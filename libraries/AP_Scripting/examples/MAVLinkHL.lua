--[[
Lua script to simulate a HL connection over a UART
Setup:
This script requires 1 serial port:
A "Script" serial port to connect directly to the GCS

Usage:
Use the "hl_mode" variable to control the behaviour of the script:
0 = start enabled, can be disabled via MAV_CMD_CONTROL_HIGH_LATENCY
1 = enabled on loss of telemetry link for 5 seconds (default)
2 = start disabled, can be enabled via MAV_CMD_CONTROL_HIGH_LATENCY

Use the MAVLink High Latency Control ("link hl on|off" in MAVProxy) to control
whether to send or not

Change the use_mavlink2 variable to true to use MAVLink 2 instead of MAVLink 1

Caveats:
-This will send HIGH_LATENCY2 packets in place of HEARTBEAT packets
-A single HIGH_LATENCY2 packet will be send every 5 sec
-The param SCR_VM_I_COUNT may need to be increased in some circumstances
-This script does not handle MAVLink 2 signing, so it will not work with GCS that require signed packets

Written by Stephen Dade (stephen_dade@hotmail.com)
--]]

---@diagnostic disable: need-check-nil


local port = serial:find_serial(0)

if not port then
    gcs:send_text(0, "No Scripting Serial Port")
    return
end

port:begin(19200)
port:set_flow_control(0)

local time_last_tx = millis():tofloat() * 0.001

local hl_mode = 0
local link_lost_for = 0

-- set to true to use MAVLink 2 instead of MAVLink 1
local use_mavlink2 = false

-- enable high latency mode from here, instead of having to enable from GCS
if hl_mode == 0 then
    gcs:enable_high_latency_connections(true)
end

--[[
Returns true if the value is NaN, false otherwise
--]]
local function isNaN (x)
    return (x ~= x)
  end

--[[
Lua Object for decoding and encoding MAVLink (V1 only) messages
--]]
local function MAVLinkProcessor()
    -- public fields
    local self = {
        -- define MAVLink message id's
        COMMAND_LONG = 76,
        COMMAND_INT = 75,
        HIGH_LATENCY2 = 235,
        MISSION_ITEM_INT = 73,
        SET_MODE = 11,
        MISSION_SET_CURRENT = 41
    }

    -- private fields
    local _mavbuffer = ""
    local _mavresult = {}
    local _payload_len = 0
    local _mavdecodestate = 0 -- 0=looking for marker, 1=getting header,2=getting payload,3=getting crc
    PROTOCOL_MARKER_V1 = 0xFE
    HEADER_LEN_V1 = 6
    PROTOCOL_MARKER_V2 = 0xFD
    HEADER_LEN_V2 = 10
    local _txseqid = 0

    -- AUTOGEN from MAVLink generator
    local _crc_extra = {}
    _crc_extra[75] = 0x9e
    _crc_extra[76] = 0x98
    _crc_extra[235] = 0xb3
    _crc_extra[73] = 0x26
    _crc_extra[11] = 0x59
    _crc_extra[41] = 0x1c

    local _payload_size = {}
    _payload_size[75] = 35
    _payload_size[76] = 33
    _payload_size[235] = 42
    _payload_size[73] = 38
    _payload_size[11] = 6
    _payload_size[41] = 4

    local _messages = {}
    _messages[75] = { -- COMMAND_INT
        {"param1", "<f"}, {"param2", "<f"}, {"param3", "<f"}, {"param4", "<f"},
        {"x", "<i4"}, {"y", "<i4"}, {"z", "<f"}, {"command", "<I2"},
        {"target_system", "<B"}, {"target_component", "<B"}, {"frame", "<B"},
        {"current", "<B"}, {"autocontinue", "<B"}
    }
    _messages[76] = { -- COMMAND_LONG
        {"param1", "<f"}, {"param2", "<f"}, {"param3", "<f"}, {"param4", "<f"},
        {"param5", "<f"}, {"param6", "<f"}, {"param7", "<f"},
        {"command", "<I2"}, {"target_system", "<B"}, {"target_component", "<B"},
        {"confirmation", "<B"}
    }
    _messages[235] = { -- HIGH_LATENCY2
        {"timestamp", "<I4"}, {"latitude", "<i4"}, {"longitude", "<i4"},
        {"custom_mode", "<I2"}, {"altitude", "<i2"}, {"target_altitude", "<i2"},
        {"target_distance", "<I2"}, {"wp_num", "<I2"}, {"failure_flags", "<I2"},
        {"type", "<B"}, {"autopilot", "<B"}, {"heading", "<B"},
        {"target_heading", "<B"}, {"throttle", "<B"}, {"airspeed", "<B"},
        {"airspeed_sp", "<B"}, {"groundspeed", "<B"}, {"windspeed", "<B"},
        {"wind_heading", "<B"}, {"eph", "<B"}, {"epv", "<B"},
        {"temperature_air", "<b"}, {"climb_rate", "<b"}, {"battery", "<b"},
        {"custom0", "<B"}, -- should be <b (int8), but we're hacking this into a uint8 instead
        {"custom1", "<b"}, {"custom2", "<b"}
    }
    _messages[73] = { -- MISSION_ITEM_INT
        {"param1", "<f"}, {"param2", "<f"}, {"param3", "<f"}, {"param4", "<f"},
        {"x", "<i4"}, {"y", "<i4"}, {"z", "<f"}, {"seq", "<I2"},
        {"command", "<I2"}, {"target_system", "<B"}, {"target_component", "<B"},
        {"frame", "<B"}, {"current", "<B"}, {"autocontinue", "<B"}
    }
    _messages[11] = { -- SET_MODE
        { "custom_mode", "<I4" }, { "target_system", "<B" }, { "base_mode", "<B" },
    }
    _messages[41] = { -- MISSION_SET_CURRENT
        { "seq", "<I2" }, { "target_system", "<B" }, { "target_component", "<B" },
    }
    function self.getSeqID() return _txseqid end

    function self.generateCRC(buffer)
        -- generate the x25crc for a given buffer. Make sure to include crc_extra!
        local crc = 0xFFFF
        for i = 1, #buffer do
            local tmp = string.byte(buffer, i, i) ~ (crc & 0xFF)
            tmp = (tmp ~ (tmp << 4)) & 0xFF
            crc = (crc >> 8) ~ (tmp << 8) ~ (tmp << 3) ~ (tmp >> 4)
            crc = crc & 0xFFFF
        end
        return string.pack("<H", crc)
    end

    function self.parseMAVLink(byte)
        -- parse a new byte and see if we've got MAVLink message
        -- returns true if a packet was decoded, false otherwise
        _mavbuffer = _mavbuffer .. string.char(byte)

        -- check if this is a start of packet
        if _mavdecodestate == 0 and ((use_mavlink2 and byte == PROTOCOL_MARKER_V2) or (not use_mavlink2 and byte == PROTOCOL_MARKER_V1)) then
            -- we have a packet start, discard the buffer before this byte
            _mavbuffer = string.char(byte)
            _mavdecodestate = 1
            return false
        end

        -- if we have a full header, try parsing
        if _mavdecodestate == 1 and ((not use_mavlink2 and #_mavbuffer == HEADER_LEN_V1) or (use_mavlink2 and #_mavbuffer == HEADER_LEN_V2)) then
            local read_marker = 1
            _, read_marker = string.unpack("<B", _mavbuffer, read_marker)
            _payload_len, read_marker = string.unpack("<B", _mavbuffer,
                                                      read_marker) -- payload is always the second byte
            if use_mavlink2 then
                -- ignore the 2 bytes of flags
                read_marker = read_marker + 2
            end
            -- fetch seq/sysid/compid
            _mavresult.seq, read_marker =
                string.unpack("<B", _mavbuffer, read_marker)
            _mavresult.sysid, read_marker =
                string.unpack("<B", _mavbuffer, read_marker)
            _mavresult.compid, read_marker =
                string.unpack("<B", _mavbuffer, read_marker)
            -- fetch the message id
            if use_mavlink2 then
                _mavresult.msgid, _ =
                    string.unpack("<I3", _mavbuffer, read_marker)
            else
                _mavresult.msgid, _ =
                    string.unpack("<B", _mavbuffer, read_marker)
            end

            _mavdecodestate = 2
            return false
        end

        -- get payload
        if _mavdecodestate == 2 and not use_mavlink2 and #_mavbuffer ==
            (_payload_len + HEADER_LEN_V1) then
            _mavdecodestate = 3
            _mavresult.payload = string.sub(_mavbuffer, HEADER_LEN_V1 + 1)
            return false
        elseif _mavdecodestate == 2 and use_mavlink2 and  #_mavbuffer ==
            (_payload_len + HEADER_LEN_V2) then
            _mavdecodestate = 3
            _mavresult.payload = string.sub(_mavbuffer, HEADER_LEN_V2 + 1)
            return false
        end

        -- get crc, then process if CRC ok
        if _mavdecodestate == 3 and ((not use_mavlink2 and #_mavbuffer ==
            (_payload_len + HEADER_LEN_V1 + 2)) or (use_mavlink2 and
            #_mavbuffer == (_payload_len + HEADER_LEN_V2 + 2))) then
            _mavdecodestate = 0
            _mavresult.crc = string.sub(_mavbuffer, -2, -1)

            local message_map = _messages[_mavresult.msgid]
            if not message_map then
                -- we don't know how to decode this message, bail on it
                _mavbuffer = ""
                return true
            end

            -- check CRC, if message defined
            local crc_extra_msg = _crc_extra[_mavresult.msgid]
            if crc_extra_msg ~= nil then
                local calccrc = self.generateCRC(
                                    string.sub(_mavbuffer, 2, -3) ..
                                        string.char(crc_extra_msg))
                if _mavresult.crc ~= calccrc then
                    gcs:send_text(3,
                                  "Bad CRC: " ..
                                      self.bytesToString(_mavbuffer, -2, -1) ..
                                      ", " .. self.bytesToString(calccrc, 1, 2))
                    _mavbuffer = ""
                    return true
                end
            end

            -- verify payload length before unpacking
            if #_mavresult.payload ~= _payload_size[_mavresult.msgid] then
                if use_mavlink2 then
                    -- re-add the truncated 0 bytes
                    _mavresult.payload = _mavresult.payload ..
                        string.rep("\0", _payload_size[_mavresult.msgid] - #_mavresult.payload)
                else
                    _mavbuffer = ""
                    _mavdecodestate = 0
                    return true
                end
            end

            -- map all the fields out
            local offset = 1
            for _, v in ipairs(message_map) do
                if v[3] then
                    _mavresult[v[1]] = {}
                    for j = 1, v[3] do
                        _mavresult[v[1]][j], offset = string.unpack(v[2],
                                                                    _mavresult.payload,
                                                                    offset)
                    end
                else
                    _mavresult[v[1]], offset = string.unpack(v[2],
                                                             _mavresult.payload,
                                                             offset)
                end
            end

            -- only process COMMAND_LONG and COMMAND_INT and  MISSION_ITEM_INT messages
            if _mavresult.msgid == self.MISSION_ITEM_INT then
                -- goto somewhere (guided mode target)
                if _mavresult.command == 16 then -- MAV_CMD_NAV_WAYPOINT
                    local loc = Location()
                    loc:lat(_mavresult.x)
                    loc:lng(_mavresult.y)
                    loc:alt(_mavresult.z * 100)
                    if _mavresult.frame == 10 then -- MAV_FRAME_GLOBAL_TERRAIN_ALT
                        loc:terrain_alt(true)
                    elseif _mavresult.frame == 3 then -- MAV_FRAME_GLOBAL_RELATIVE_ALT
                        loc:relative_alt(true)
                    end
                    vehicle:set_target_location(loc)
                end
            elseif _mavresult.msgid == self.SET_MODE then
                vehicle:set_mode(_mavresult.custom_mode)
            elseif _mavresult.msgid == self.COMMAND_LONG then
                --- need to do a little conversion here. Taken from convert_COMMAND_LONG_to_COMMAND_INT()
                local command_long_stores_location = 0
                if (_mavresult.command == 179 or       -- MAV_CMD_DO_SET_HOME
                    _mavresult.command ==  201 or      -- MAV_CMD_DO_SET_ROI
                    _mavresult.command ==  195 or      -- MAV_CMD_DO_SET_ROI_LOCATION
                    _mavresult.command ==  192 or      -- MAV_CMD_DO_REPOSITION
                    _mavresult.command ==  43003) then -- MAV_CMD_EXTERNAL_POSITION_ESTIMATE
                    command_long_stores_location = 1
                end
                local int_frame_conv = 0       -- MAV_FRAME_GLOBAL
                if (_mavresult.command ==  195) then       -- MAV_CMD_DO_SET_ROI_LOCATION
                    int_frame_conv = 3         -- MAV_FRAME_GLOBAL_RELATIVE_ALT
                elseif (_mavresult.command ==  179) then   -- MAV_CMD_DO_SET_HOME
                    int_frame_conv = 0        -- MAV_FRAME_GLOBAL
                elseif (_mavresult.command ==  201) then   -- MAV_CMD_DO_SET_ROI
                    int_frame_conv = 3        -- MAV_FRAME_GLOBAL_RELATIVE_ALT
                elseif (_mavresult.command ==  42006) then   -- MAV_CMD_FIXED_MAG_CAL_YAW
                    int_frame_conv = 3        -- MAV_FRAME_GLOBAL_RELATIVE_ALT
                end
                local int_x = _mavresult.param5
                local int_y = _mavresult.param6
                if isNaN(int_x) then
                    int_x = 0
                end
                if isNaN(int_y) then
                    int_y = 0
                end
                if command_long_stores_location == 1 then
                    int_x = 1E7 * int_x
                    int_y = 1E7 * int_y
                end
                gcs:run_command_int(_mavresult.command, { p1 = _mavresult.param1,
                                                          p2 = _mavresult.param2,
                                                          p3 = _mavresult.param3,
                                                          p4 = _mavresult.param4,
                                                          x = int_x,
                                                          y = int_y,
                                                          z = _mavresult.param7,
                                                          int_frame = int_frame_conv,
                                                          current = 0,
                                                          autocontinue = 0 })
            elseif _mavresult.msgid == self.COMMAND_INT then
                gcs:run_command_int(_mavresult.command, { p1 = _mavresult.param1,
                                                          p2 = _mavresult.param2,
                                                          p3 = _mavresult.param3,
                                                          p4 = _mavresult.param4,
                                                          x = _mavresult.x,
                                                          y = _mavresult.y,
                                                          z = _mavresult.z,
                                                          frame = _mavresult.frame })
            elseif _mavresult.msgid == self.MISSION_SET_CURRENT then
                mission:set_current_cmd(_mavresult.seq)
            end
            _mavbuffer = ""
            return true
        end

        -- packet too big ... start again
        if (not use_mavlink2 and #_mavbuffer > 263) or (use_mavlink2 and #_mavbuffer > 280) then 
            _mavbuffer = ""
            _mavdecodestate = 0
            return true
        end
        return false
    end

    function self.bytesToString(buf, start, stop)
        local ret = ""
        for idx = start, stop do
            ret = ret .. string.format("0x%x ", buf:byte(idx), 1, -1) .. " "
        end
        return ret
    end

    function self.createMAVLink(message, msgid)
        -- generate a mavlink message

        -- create the payload
        local message_map = _messages[msgid]
        if not message_map then
            -- we don't know how to encode this message, bail on it
            gcs:send_text(3, "Unknown MAVLink message " .. msgid)
            return nil
        end

        local packString = "<"
        local packedTable = {}
        local packedIndex = 1
        for i, v in ipairs(message_map) do
            if v[3] then
                packString = (packString ..
                                 string.rep(string.sub(v[2], 2), v[3]))
                for j = 1, v[3] do
                    packedTable[packedIndex] = message[message_map[i][1]][j]
                    packedIndex = packedIndex + 1
                end
            else
                packString = (packString .. string.sub(v[2], 2))
                packedTable[packedIndex] = message[message_map[i][1]]
                packedIndex = packedIndex + 1
            end
        end

        local payload = string.pack(packString, table.unpack(packedTable))

        --if mavlink2, truncate any trailing 0 bytes
        if use_mavlink2 then
            local payload_len = #payload
            while payload_len > 0 and payload:byte(payload_len) == 0 do
                payload = string.sub(payload, 1, payload_len - 1)
                payload_len = payload_len - 1
            end
        end

        -- create the header. Assume componentid of 1
        local header
        if use_mavlink2 then
            -- MAVLink 2 header
            header = string.pack('<BBBBBBBI3', PROTOCOL_MARKER_V2, #payload,
                                0, 0, _txseqid, param:get('MAV_SYSID'), 1,
                                msgid)
        else
            -- MAVLink 1 header
            header = string.pack('<BBBBBB', PROTOCOL_MARKER_V1, #payload,
                                _txseqid, param:get('MAV_SYSID'), 1,
                                msgid)
        end

        -- generate the CRC
        local crc_extra_msg = _crc_extra[msgid]
        local crc = self.generateCRC(string.sub(header, 2) .. payload ..
                                         string.char(crc_extra_msg))

        -- iterate sequence id
        _txseqid = (_txseqid + 1) % 255

        return header .. payload .. crc
    end

    -- return the instance
    return self
end

-- Transmitted HIGH_LATENCY2 packet
local hl2 = {}
hl2.timestamp = 0
hl2.latitude = 0
hl2.longitude = 0
hl2.custom_mode = 0
hl2.altitude = 0
hl2.target_altitude = 0
hl2.target_distance = 0
hl2.wp_num = 0
hl2.failure_flags = 0
hl2.type = gcs:frame_type()
hl2.autopilot = 3 -- MAV_AUTOPILOT_ARDUPILOTMEGA
hl2.heading = 0
hl2.target_heading = 0
hl2.throttle = 0
hl2.airspeed = 0
hl2.airspeed_sp = 0
hl2.groundspeed = 0
hl2.windspeed = 0
hl2.wind_heading = 0
hl2.eph = 0
hl2.epv = 0
hl2.temperature_air = 0
hl2.climb_rate = 0
hl2.battery = 0
hl2.custom0 = 1 -- MAV_MODE_FLAG_CUSTOM_MODE_ENABLED
hl2.custom1 = 0
hl2.custom2 = 0

function wrap_360(angle)
    local res = angle % 360
    if res < 0 then res = res + 360 end
    return res
end

-- Define the MAVLink processor
local mavlink = MAVLinkProcessor()

function HLSatcom()
    -- read in any bytes from GCS and and send to MAVLink processor, up to max of 100 bytes at a time to prevent timeouts
    local n_bytes = math.min(port:available():toint(), 100)
    while n_bytes > 0 do
        read = port:read()
        n_bytes = n_bytes - 1
        mavlink.parseMAVLink(read)
    end

    -- if mode 1 and there's been no mavlink traffic for 5000 ms, enable high latency
    if hl_mode == 1 then
        -- link lost time = boot time - GCS last seen time
        link_lost_for = (millis()- gcs:last_seen()):toint()
        -- gcs:last_seen() is set to millis() during boot (on plane). 0 on rover/copter
        -- So if it's less than 10000 assume no GCS packet received since boot
        if link_lost_for > 5000 and not gcs:get_high_latency_status() and gcs:last_seen() > 10000 then
            gcs:enable_high_latency_connections(true)
        elseif link_lost_for < 5000 and gcs:get_high_latency_status() then
            gcs:enable_high_latency_connections(false)
        end
    end

    -- send HL2 packet every 5 sec
    if gcs:get_high_latency_status() and (millis():tofloat() * 0.001) -
        time_last_tx > 5 then

        -- update HL2 packet
        hl2.timestamp = millis():tofloat()
        local position = ahrs:get_location()
        local wind = ahrs:wind_estimate()

        if position then
            hl2.latitude = tonumber(position:lat())
            hl2.longitude = tonumber(position:lng())
            hl2.altitude = math.floor(tonumber(position:alt()) * 0.01)
        end
        if wind then
            wind_xy = Vector2f()
            wind_xy:x(wind:x())
            wind_xy:y(wind:y())
            hl2.windspeed = math.abs(math.floor(wind_xy:length() * 5))
            hl2.wind_heading = math.floor(wrap_360(wind_xy:angle()) / 2)
        end
        hl2.custom_mode = vehicle:get_mode()

        if vehicle:get_wp_distance_m() ~= nil then
            hl2.target_distance = math.floor(vehicle:get_wp_distance_m() / 10)
        end
        if mission:get_current_nav_index() ~= nil then
            hl2.wp_num = mission:get_current_nav_index()
        end
        if vehicle:get_wp_bearing_deg() ~= nil then
            hl2.target_heading = math.floor(wrap_360(
                                                vehicle:get_wp_bearing_deg()) /
                                                2)
        end

        -- failure flags
        hl2.failure_flags = 0
        if not ahrs:healthy() then
            hl2.failure_flags = hl2.failure_flags + 4096 -- HL_FAILURE_FLAG_ESTIMATOR
        end
        if battery:num_instances() > 0 and not battery:healthy(0) then
            hl2.failure_flags = hl2.failure_flags + 128 -- HL_FAILURE_FLAG_BATTERY
        end
        if gps:num_sensors() > 0 and gps:status(0) <= gps.NO_FIX then
            hl2.failure_flags = hl2.failure_flags + 1 -- HL_FAILURE_FLAG_GPS
        end
        if (FWVersion:type() == 2 or FWVersion:type() == 3) and terrain:status() ==
            terrain.TerrainStatusUnhealthy then
            -- only for copter and plane
            hl2.failure_flags = hl2.failure_flags + 64 -- HL_FAILURE_FLAG_TERRAIN
        end
        if not rc:has_valid_input() then
            hl2.failure_flags = hl2.failure_flags + 256 -- HL_FAILURE_FLAG_RC_RECEIVER
        end

        hl2.heading = math.floor(wrap_360(math.deg(ahrs:get_yaw_rad())) / 2)
        hl2.throttle = math.floor(gcs:get_hud_throttle())
        if ahrs:airspeed_EAS() ~= nil then
            hl2.airspeed = math.abs(math.floor(ahrs:airspeed_EAS() * 5))
        end
        -- hl2.airspeed_sp = 0
        hl2.groundspeed = math.abs(math.floor(
                                       ahrs:groundspeed_vector():length() * 5))

        hl2.temperature_air = math.floor(baro:get_external_temperature())

        if battery:num_instances() > 0 and battery:capacity_remaining_pct(0) ~= nil then
            hl2.battery = battery:capacity_remaining_pct(0)
        else
            hl2.battery = 0
        end
        
        -- just sending armed state here for simplicity. Flight mode is in the custom_mode field
        if arming:is_armed() then
            hl2.custom0 = 129 -- MAV_MODE_FLAG_SAFETY_ARMED + MAV_MODE_FLAG_CUSTOM_MODE_ENABLED
        else
            hl2.custom0 = 1 -- MAV_MODE_FLAG_CUSTOM_MODE_ENABLED
        end

        local newpkt = mavlink.createMAVLink(hl2, mavlink.HIGH_LATENCY2)
        gcs:send_text(3,
                      "Sent HL2 packet, size: " .. tostring(#newpkt) .. ", seq " ..
                          mavlink.getSeqID())

        for idx = 1, #newpkt do port:write(newpkt:byte(idx)) end

        time_last_tx = millis():tofloat() * 0.001

    end

    return HLSatcom, 100
end

return HLSatcom, 100

