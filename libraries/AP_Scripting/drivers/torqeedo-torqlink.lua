 --[[
    Torqeedo TorqLink driver lua script

    How To Use:

    Connect the Torqeedo motor(s) to the autopilot's CAN ports.  If only one motor is used it should be connected to CAN1
    If two motors are used, connect the left motor to CAN1 and the right motor to CAN2

    Enable CAN1 by setting these parameters:
      - CAN_P1_DRIVER = 1 (First driver)
      - CAN_D1_PROTOCOL = 10 (Scripting)

    If CAN2 is being used set these parameters:
      - CAN_P2_DRIVER = 2 (Second driver)
      - CAN_D2_PROTOCOL = 12 (Scripting2)

    Copy this script to the autopilot's SD card in the APM/scripts directory and restart the autopilot
--]]

local PARAM_TABLE_KEY = 91
local PARAM_TABLE_PREFIX = "TRQL_"

-- add a parameter and bind it to a variable
function bind_add_param(name, idx, default_value)
    assert(param:add_param(PARAM_TABLE_KEY, idx, name, default_value), string.format('could not add param %s', name))
    return Parameter(PARAM_TABLE_PREFIX .. name)
end

-- setup script specific parameters
assert(param:add_table(PARAM_TABLE_KEY, PARAM_TABLE_PREFIX, 2), 'could not add param table')

--[[
  // @Param: TRQL_ENABLE
  // @DisplayName: Torqeedo TorqLink Enable
  // @Description: Torqeedo TorqLink Enable
  // @Values: 0:Disabled, 1:Enabled
  // @User: Standard
--]]
local TRQL_ENABLE = bind_add_param('ENABLE', 1, 1)

--[[
  // @Param: TRQL_DEBUG
  // @DisplayName: Torqeedo TorqLink Debug Level
  // @Description: Torqeedo TorqLink Debug Level
  // @Values: 0:None, 1:Low, 2:Medium, 3:High
  // @User: Standard
--]]
local TRQL_DEBUG = bind_add_param('DEBUG', 2, 0)

-- global definitions
local MAV_SEVERITY = {EMERGENCY=0, ALERT=1, CRITICAL=2, ERROR=3, WARNING=4, NOTICE=5, INFO=6, DEBUG=7}
local TEXT_PREFIX_STR = "torqeedo-torqlink:"    -- prefix for text messages sent to user
local UPDATE_MS = 10                            -- update interval in milliseconds, 10 = 100hz
local SRV_FN_THROTTLE = 70                      -- servo function for throttle
local SRV_FN_THROTTLE_LEFT = 73                 -- servo function for left motor
local SRV_FN_THROTTLE_RIGHT = 74                -- servo function for right motor
local CAN_BUF_LEN = 8                           -- CAN frame length
local CAN_WRITE_TIMEOUT_US = 10000              -- timeout in microseconds for writing a frame to the Torqeedo device
local TORQCAN_DIR_FORWARD = 126                 -- forward direction command
local TORQCAN_DIR_NEUTRAL = 125                 -- neutral direction command
local TORQCAN_DIR_BACKWARDS = 124               -- reverse direction command
local TORQCAN_PF_MASK = 0x00FF0000              -- bitmask used to extract health from PGN message
local TORQCAN_DA_MASK = 0x0000FF00              -- bitmask used to extract health from PGN message
local RECEIVE_TIMEOUT_MS = 1000                 -- timeout for receiving a frame from the Torqeedo device
local OUTPUT_INTERVAL_MS = 50					-- output interval in milliseconds, 50 = 20hz
local RECOVERY_DELAY_MS = 2000                  -- delay (in ms) after motor recovers before sending outputs

-- local variables
local left_device_last_recv_ms = uint32_t(0)    -- system time of last frame received from CAN1 device
local right_device_last_recv_ms = uint32_t(0)   -- system time of last frame received from CAN2 device
local left_device_timeout = true                -- flag to indicate if the left device is in a timeout state
local right_device_timeout = true               -- flag to indicate if the right device is in a timeout state
local last_send_ms = uint32_t(0)                -- system time of last command sent to Torqeedo devices
local recovery_start_ms = uint32_t(0)           -- system time motor communication was recovered (0 if not recovered recently)
local debug_send_speed_ms = uint32_t(0)         -- system time of last debug speed message

-- get CAN drivers with protocol configured for scripting, set buffer size to 8 frames
local left_device = CAN:get_device(CAN_BUF_LEN)
local right_device = CAN:get_device2(CAN_BUF_LEN)

-- swap two floats
function swap_float(f1, f2)
    return f2, f1
end

-- interpolate function
function interpolate(output_low, output_high, input_value, input_low, input_high)
-- support either polarity
    if (input_low > input_high) then
        input_low, input_high = swap_float(input_low, input_high)
        output_low, output_high = swap_float(output_low, output_high)
    end
    if (input_value <= input_low) then
        return output_low
    end
    if (input_value > input_high) then
        return output_high
    end
    local p = (input_value - input_low) / (input_high - input_low)
    return math.floor((output_low + p * (output_high - output_low)))
end

-- send speed and direction control commands
-- speed should be in the range of -1 to +1
function send_speed_and_direction(device, speed)

    -- sanity check device
    if not device then
        return
    end

    -- get absolute speed
    local speed_abs = math.abs(speed)

	-- convert speed into range 0 to 250
    local speed_cmd = interpolate(0.0, 250.0, speed_abs, 0.0, 1.0)

    -- extended frame, priority 12, PGN 0xF003, and node ID 208 - (0x8CF003d0)
    -- lua cannot handle numbers so large, so we have to use uint32_t userdata
    local speed_frame = CANFrame()
    speed_frame:id((uint32_t(1) << 31) | (uint32_t(12) << 24) | (uint32_t(tonumber("0xF003")) << 8) | uint32_t(208))
    speed_frame:dlc(8)
    speed_frame:data(0, 255)
    speed_frame:data(1, speed_cmd)
    speed_frame:data(2, 255)
    speed_frame:data(3, 255)
    speed_frame:data(4, 255)
    speed_frame:data(5, 255)
    speed_frame:data(6, 255)
    speed_frame:data(7, 255)

    -- write the speed frame with a 10000us timeout
    device:write_frame(speed_frame, CAN_WRITE_TIMEOUT_US)

    -- calculate direction
    local dir_cmd = TORQCAN_DIR_NEUTRAL
    if speed < 0 then
        dir_cmd = TORQCAN_DIR_BACKWARDS
    elseif speed > 0 then
        dir_cmd = TORQCAN_DIR_FORWARD
    end

    -- set transmission to forward
    local dir_frame = CANFrame()

    -- extended frame, priority 12, PGN 0xF005, and node ID 208 - (0x8CF005d0)
    -- lua cannot handle numbers so large, so we have to use uint32_t userdata
    dir_frame:id((uint32_t(1) << 31) | (uint32_t(12) << 24) | (uint32_t(tonumber("0xF005")) << 8) | uint32_t(208))
    dir_frame:dlc(8)
    dir_frame:data(0, dir_cmd) -- transmission gear (124-126)
    dir_frame:data(1, 255)
    dir_frame:data(2, 255)
    dir_frame:data(3, 255)
    dir_frame:data(4, 255)
    dir_frame:data(5, 255)
    dir_frame:data(6, 255)
    dir_frame:data(7, 255)

    -- write the direction frame with a 10000us timeout
    device:write_frame(dir_frame, CAN_WRITE_TIMEOUT_US)

    -- print frame debug at 1hz
    if TRQL_DEBUG:get() > 0 then
        local now_ms = millis()
        if (now_ms - debug_send_speed_ms) > 1000 then
            debug_send_speed_ms = now_ms
            gcs:send_text(MAV_SEVERITY.INFO, string.format("%s speed:%4.2f dir:%d", TEXT_PREFIX_STR, speed_cmd, dir_cmd))
        end
    end
end

-- get parameter group number (PGN) from frame id
function get_PGN_from_frameid(can_frame_id)
    local pf = (TORQCAN_PF_MASK & can_frame_id) >> 16
    local da = (TORQCAN_DA_MASK & can_frame_id) >> 8
    local pgn
    if pf >= 240 then
        pgn = pf * 256 + da
    else
        pgn = pf * 256
    end
    return pgn
end

-- parse a frame, returns true if motor is healthy
function parse_frame_for_health(frame)
    -- extract Parameter Group Number (PGN) from frame id
    local pgn = tostring(get_PGN_from_frameid(frame:id()))
    if pgn == "65299" then -- check if Torqeedo is ready
        local ready = frame:data(4) >> 7 -- check first bit of thruster status bitmap
        if ready == 1 then
            do return true end
        end
    end

    -- print incoming frame debug
    if TRQL_DEBUG:get() > 1 then
        gcs:send_text(MAV_SEVERITY.INFO, string.format("%s read id:%d PGN:%s", TEXT_PREFIX_STR, frame:id():toint(), pgn))
    end
    return false
end

-- read incoming frames
function read_incoming_frames()
    -- read incoming frame from left device
    if left_device then
        local frame = left_device:read_frame()
        if frame and parse_frame_for_health(frame) then
            left_device_last_recv_ms = millis()
        end
    end

    -- read incoming frame from right device
    if right_device then
        local frame = right_device:read_frame()
        if frame and parse_frame_for_health(frame) then
            right_device_last_recv_ms = millis()
        end
    end
end

-- print welcome message
gcs:send_text(MAV_SEVERITY.INFO, "torqeedo-torqlink script loaded")

-- update function runs at about 20hz
function update()

    -- check if script is enabled
    if TRQL_ENABLE:get() == 0 then
        return update, 1000
    end

    -- read incoming frames from devices
    read_incoming_frames()

    -- get current time
    local now_ms = millis()

    -- check for timeouts of left device
    local left_device_timeout_prev = left_device_timeout
    left_device_timeout = (now_ms - left_device_last_recv_ms) > RECEIVE_TIMEOUT_MS
    if left_device_timeout ~= left_device_timeout_prev then
        if left_device_timeout then
            gcs:send_text(MAV_SEVERITY.WARNING, TEXT_PREFIX_STR .. "device1 timeout")
        else
            gcs:send_text(MAV_SEVERITY.INFO, TEXT_PREFIX_STR .. "device1 healthy")
            recovery_start_ms = now_ms
        end
    end

    -- check for timeouts of right device
    local right_device_timeout_prev = right_device_timeout
    right_device_timeout = (now_ms - right_device_last_recv_ms) > RECEIVE_TIMEOUT_MS
    if right_device_timeout ~= right_device_timeout_prev then
        if right_device_timeout then
            gcs:send_text(MAV_SEVERITY.WARNING, TEXT_PREFIX_STR .. "device2 timeout")
        else
            gcs:send_text(MAV_SEVERITY.INFO, TEXT_PREFIX_STR .. "device2 healthy")
            recovery_start_ms = now_ms
        end
    end

    -- send commands to motors at specified interval
    if (now_ms - last_send_ms) >= OUTPUT_INTERVAL_MS then
        -- record last send time (we will certainly send a command below)
        last_send_ms = now_ms

        -- suppress output for 2 seconds after recovery
        local recovery_active = false
        if recovery_start_ms > 0 then
            if ((now_ms - recovery_start_ms) < RECOVERY_DELAY_MS) then
                recovery_active = true
            else
                -- clear recovery start time
                recovery_start_ms = uint32_t(0)
            end
        end

        -- determine whether we should send throttle or left and right throttle
        local throttle_defined = (SRV_Channels:find_channel(SRV_FN_THROTTLE) ~= nil)
        local left_throttle_defined = (SRV_Channels:find_channel(SRV_FN_THROTTLE_LEFT) ~= nil)
        local right_throttle_defined = (SRV_Channels:find_channel(SRV_FN_THROTTLE_RIGHT) ~= nil)

        -- default left and right speed to 0 (in range -1 to +1)
        local left_device_speed = 0
        local right_device_speed = 0

        -- use throttle by default but use left and/or right throttle if defined
        if throttle_defined then
            left_device_speed = SRV_Channels:get_output_scaled(SRV_FN_THROTTLE) * 0.01
            right_device_speed = SRV_Channels:get_output_scaled(SRV_FN_THROTTLE) * 0.01
        end
        if left_throttle_defined then
            left_device_speed = SRV_Channels:get_output_scaled(SRV_FN_THROTTLE_LEFT) * 0.001
        end
        if right_throttle_defined then
            right_device_speed = SRV_Channels:get_output_scaled(SRV_FN_THROTTLE_RIGHT) * 0.001
        end

        -- override speed if device timesout or in recovery
        if left_device_timeout or recovery_active then
            left_device_speed = 0
        end
        if right_device_timeout or recovery_active then
            right_device_speed = 0
        end

        -- if no outputs defined print warning to user
        if not (left_throttle_defined or right_throttle_defined or throttle_defined) then
            gcs:send_text(MAV_SEVERITY.WARNING, TEXT_PREFIX_STR .. "check SERVOx_FUNCTION")
        else
            -- send speed and direction to left and right devices
            send_speed_and_direction(left_device, left_device_speed)
            send_speed_and_direction(right_device, right_device_speed)
        end
    end

    return update, UPDATE_MS
end

return update()
