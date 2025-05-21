-- Move a Copter so as to reduce a slung payload's oscillation.  Requires the payload be capable of sending its position and velocity to the main vehicle
--
-- How To Use
-- 1. copy this script to the autopilot's "scripts" directory
-- 2. within the "scripts" directory create a "modules" directory
-- 3. copy the MAVLink/mavlink_msgs_xxx files to the "scripts" directory
-- 4. add an autopilot and GPS to the payload and configure it to send GLOBAL_POSITION_INT messages at 10hz to the vehicle
-- 5. create a mission with a SCRIPT_TIME or PAYLOAD_PLACE command included
-- 6. fly the mission and the vehicle should move so as to reduce the payload's oscillations while executing the SCRIPT_TIME or PAYLOAD_PLACE commands
-- 7. optionally set SLUP_SYSID to the system id of the payload autopilot
-- 8. optionally set WP_YAW_BEHAVIOR to 0 to prevent the vehicle from yawing while moving to the payload

-- load mavlink message definitions from modules/MAVLink directory
local mavlink_msgs = require("MAVLink/mavlink_msgs")

-- global definitions
local MAV_SEVERITY = {EMERGENCY=0, ALERT=1, CRITICAL=2, ERROR=3, WARNING=4, NOTICE=5, INFO=6, DEBUG=7}
local UPDATE_INTERVAL_MS = 10           -- update at about 100hz
local COPTER_MODE_AUTO = 3
local MAV_CMD_NAV_PAYLOAD_PLACE = 94
local MAV_CMD_NAV_SCRIPT_TIME = 42702
local PAYLOAD_OFFSET_COMP_VEL_MAX = 1   -- payload offset compensation will be active when the payload's horizontal velocity is no more than this speed in m/s
local PAYLOAD_UPDATE_TIMEOUT_MS = 1000  -- payload update timeout, used to warn user on loss of connection
local CONTROL_TIMEOUT_MS = 3000         -- control timeout, used to reset offsets if they have not been set for more than 3 seconds
local TEXT_PREFIX_STR = "copter-slung-payload:"    -- prefix for all text messages

 -- setup script specific parameters
local PARAM_TABLE_KEY = 82
local PARAM_TABLE_PREFIX = "SLUP_"
assert(param:add_table(PARAM_TABLE_KEY, PARAM_TABLE_PREFIX, 7), 'could not add param table')

-- add a parameter and bind it to a variable
function bind_add_param(name, idx, default_value)
    assert(param:add_param(PARAM_TABLE_KEY, idx, name, default_value), string.format('could not add param %s', PARAM_TABLE_PREFIX .. name))
    return Parameter(PARAM_TABLE_PREFIX .. name)
 end

--[[
  // @Param: SLUP_ENABLE
  // @DisplayName: Slung Payload enable
  // @Description: Slung Payload enable
  // @Values: 0:Disabled,1:Enabled
  // @User: Standard
--]]
local SLUP_ENABLE = bind_add_param("ENABLE", 1, 1)

--[[
  // @Param: SLUP_VEL_P
  // @DisplayName: Slung Payload Velocity P gain
  // @Description: Slung Payload Velocity P gain, higher values will result in faster movements in sync with payload
  // @Range: 0 0.8
  // @User: Standard
--]]
local SLUP_VEL_P = bind_add_param("VEL_P", 2, 0.5)

--[[
  // @Param: SLUP_DIST_MAX
  // @DisplayName: Slung Payload horizontal distance max
  // @Description: Oscillation is suppressed when vehicle and payload are no more than this distance horizontally.  Set to 0 to always suppress
  // @Range: 0 30
  // @User: Standard
--]]
local SLUP_DIST_MAX = bind_add_param("DIST_MAX", 3, 15)

--[[
  // @Param: SLUP_SYSID
  // @DisplayName: Slung Payload mavlink system id
  // @Description: Slung Payload mavlink system id.  0 to use any/all system ids
  // @Range: 0 255
  // @User: Standard
--]]
local SLUP_SYSID = bind_add_param("SYSID", 4, 0)

--[[
  // @Param: SLUP_WP_POS_P
  // @DisplayName: Slung Payload return to WP position P gain
  // @Description: WP position P gain. higher values will result in vehicle moving more quickly back to the original waypoint
  // @Range: 0 1
  // @User: Standard
--]]
local SLUP_WP_POS_P = bind_add_param("WP_POS_P", 5, 0.05)

--[[
  // @Param: SLUP_RESTOFS_TC
  // @DisplayName: Slung Payload resting offset estimate filter time constant
  // @Description: payload's position estimator's time constant used to compensate for GPS errors and wind.  Higher values result in smoother estimate but slower response
  // @Range: 1 20
  // @User: Standard
--]]
local SLUP_RESTOFS_TC = bind_add_param("RESTOFS_TC", 6, 10)

--[[
  // @Param: SLUP_DEBUG
  // @DisplayName: Slung Payload debug output
  // @Description: Slung payload debug output, set to 1 to enable debug
  // @Values: 0:Disabled,1:Enabled
  // @User: Standard
--]]
local SLUP_DEBUG = bind_add_param("DEBUG", 7, 0)

-- mavlink definitions
local GLOBAL_POSITION_INT_ID = 33
local msg_map = {}
msg_map[GLOBAL_POSITION_INT_ID] = "GLOBAL_POSITION_INT"

-- initialize MAVLink rx with buffer depth and number of rx message IDs to register
mavlink:init(5, 1)

-- register message id to receive
mavlink:register_rx_msgid(GLOBAL_POSITION_INT_ID)

-- variables
local payload_sysid = nil                       -- holds sysid of payload once a matching global position int message has been received
local payload_loc = Location()                  -- payload location
local payload_vel = Vector3f()                  -- payload velocity
local payload_acc = Vector3f()                  -- payload acceleration
local payload_update_timeout_prev = true        -- payload update timeout state from previous iteration, used to detect loss/recovery of payload updates
local payload_loc_update_ms = uint32_t(0)       -- system time that payload_loc was last updated
local payload_dist_NED = Vector3f()             -- distance between vehicle and payload in NED frame
local global_pos_int_timebootms_prev = 0        -- global position int message's time_boot_ms field from previous iteration (used to calc dt)
local resting_offset_NED = Vector3f()           -- estimated position offset between payload and vehicle
local resting_vel_NED = Vector3f()              -- estimated velocity offset.  should be near zero when hovering
local resting_offset_valid = false              -- true if resting_offset_NED and resting_vel_NED can be used
local resting_offset_update_ms = uint32_t(0)    -- system time that resting_offset_NED was last updated
local resting_offset_notify_ms = uint32_t(0)    -- system time that the user was sent the resting_offset_NED
local send_velocity_offsets = false             -- true if we should send vehicle velocity offset commands to reduce payload oscillation
local sent_velocity_offsets_ms = uint32_t(0)    -- system time that the last velocity offset was sent to the vehicle
local control_timeout_ms = uint32_t(0)          -- system time that the control timeout occurred
local payload_vel_prev = Vector3f()             -- previous iterations payload velocity used to calculate acceleration
local print_warning_ms = uint32_t(0)            -- system time that the last warning was printed.  used to prevent spamming the user with warnings

-- display a text warning to the user.  only displays a warning every second
-- prefix is automatically added
function print_warning(text_warning)
    local now_ms = millis()
    if (now_ms - print_warning_ms > 1000) then
        gcs:send_text(MAV_SEVERITY.WARNING, string.format("%s %s", TEXT_PREFIX_STR, text_warning))
        print_warning_ms = now_ms
    end
end

-- calculate sign of a number.  1 if positive, -1 if negative, 0 if exactly zero
function get_sign(value)
    if value > 0 then
        return 1
    elseif value < 0 then
        return -1
    end
    return 0
end

-- calculate an alpha for a first order low pass filter
function calc_lowpass_alpha(dt, time_constant)
    local rc = time_constant/(math.pi*2)
    return dt/(dt+rc)
end

-- handle global position int message
-- returns true if the message was from the payload and updates payload_loc, payload_vel, payload_acc and payload_loc_update_ms
function handle_global_position_int(msg)
    -- check if message is from the correct system id
    if (SLUP_SYSID:get() > 0 and msg.sysid ~= SLUP_SYSID:get()) then
        return false
    end

    -- lock onto the first matching system id
    if payload_sysid == nil then
        payload_sysid = msg.sysid
        gcs:send_text(MAV_SEVERITY.INFO, string.format("%s found sysid:%d", TEXT_PREFIX_STR, msg.sysid))
    elseif payload_sysid ~= msg.sysid then
        return false
    end

    -- check for duplicate messages and calculate dt
    local time_boot_ms = msg.time_boot_ms
    local dt = (time_boot_ms - global_pos_int_timebootms_prev) * 0.001
    global_pos_int_timebootms_prev = time_boot_ms
    if dt <= 0 or dt > 1 then
        return false
    end

    -- update payload location
    payload_loc:lat(msg.lat)
    payload_loc:lng(msg.lon)
    payload_loc:alt(msg.alt * 0.1)

    -- update payload velocity
    payload_vel:x(msg.vx * 0.01)
    payload_vel:y(msg.vy * 0.01)
    payload_vel:z(msg.vz * 0.01)

    -- calc payload acceleration
    payload_acc:x((payload_vel:x() - payload_vel_prev:x()) / dt)
    payload_acc:y((payload_vel:y() - payload_vel_prev:y()) / dt)
    payload_acc:z((payload_vel:z() - payload_vel_prev:z()) / dt)
    payload_vel_prev = payload_vel:copy()

    -- record time of update
    payload_loc_update_ms = millis()
    return true
end

-- estimate the payload's resting position offset based on its current offset and velocity
-- relies on payload_dist_NED and payload_vel being updated
function update_payload_resting_offset()

    -- calculate dt since last update
    local now_ms = millis()
    local dt = (now_ms - resting_offset_update_ms):tofloat() * 0.001
    resting_offset_update_ms = now_ms

    -- sanity check dt
    if (dt <= 0) then
        resting_offset_valid = false
        do return end
    end

    -- if not updated for more than 1 second, reset resting offset to current offset
    if (dt > 1) then
        resting_offset_NED = payload_dist_NED
        resting_vel_NED = payload_vel
        resting_offset_valid = false
        do return end
    end

    -- use a low-pass filter to move the resting offset NED towards the pos_offset_NED
    local alpha = calc_lowpass_alpha(dt, SLUP_RESTOFS_TC:get())
    resting_offset_NED:x(resting_offset_NED:x() + (payload_dist_NED:x() - resting_offset_NED:x()) * alpha)
    resting_offset_NED:y(resting_offset_NED:y() + (payload_dist_NED:y() - resting_offset_NED:y()) * alpha)
    resting_offset_NED:z(resting_offset_NED:z() + (payload_dist_NED:z() - resting_offset_NED:z()) * alpha)
    resting_vel_NED:x(resting_vel_NED:x() + (payload_vel:x() - resting_vel_NED:x()) * alpha)
    resting_vel_NED:y(resting_vel_NED:y() + (payload_vel:y() - resting_vel_NED:y()) * alpha)
    resting_vel_NED:z(resting_vel_NED:z() + (payload_vel:z() - resting_vel_NED:z()) * alpha)

    -- debug output every 3 seconds
    local print_debug = false
    if (SLUP_DEBUG:get() > 0) and (now_ms - resting_offset_notify_ms > 3000) then
        print_debug = true
        resting_offset_notify_ms = now_ms
    end

    -- validate that resting offsets are valid
    -- resting velocity should be low to ensure the resting position estimate is accurate
    if (resting_vel_NED:xy():length() > PAYLOAD_OFFSET_COMP_VEL_MAX) then
        resting_offset_valid = false
        if print_debug then
            print_warning("resting velocity too high")
        end
        return
    end

    -- resting position should be within SLUP_DIST_MAX of the vehicle
    if resting_offset_NED:xy():length() > SLUP_DIST_MAX:get() then
        resting_offset_valid = false
        if print_debug then
            print_warning("payload resting pos too far, ignoring");
        end
        return
    end

    -- update user
    if (print_debug) then
        gcs:send_text(MAV_SEVERITY.INFO, string.format("resting a:%f px:%4.1f py:%4.1f pz:%4.1f vx:%4.1f vy:%4.1f vz:%4.1f", alpha, resting_offset_NED:x(), resting_offset_NED:y(), resting_offset_NED:z(), resting_vel_NED:x(), resting_vel_NED:y(), resting_vel_NED:z()))
    end

    -- if set got this far the resting offsets must be valid
    resting_offset_valid = true
end

-- move vehicle to reduce payload oscillation
-- relies on payload_dist_NED, payload_vel, payload_acc, resting_offset_NED, resting_vel_NED being updated
function move_vehicle()

    -- check horizontal distance is less than SLUP_DIST_MAX
    if SLUP_DIST_MAX:get() > 0 then
        local dist_xy = payload_dist_NED:xy():length()
        if (dist_xy > SLUP_DIST_MAX:get()) then
            print_warning(string.format("payload too far %4.1fm", dist_xy));
            do return end
        end
    end

    -- get long-term payload offset used to compensate for GPS errors and wind
    local payload_offset_NED = Vector3f()
    if resting_offset_valid then
        payload_offset_NED = resting_offset_NED
    end

    -- get position offset (cumulative effect of velocity offsets) and use to slowly move back to waypoint
    local pos_offset_NED, _, _ = poscontrol:get_posvelaccel_offset()
    if pos_offset_NED == nil then
        print_warning("unable to get dist to waypoint")
        pos_offset_NED = Vector3f()
    end

    -- calculate send velocity offsets in m/s in NED frame
    local vel_offset_NED = Vector3f()
    vel_offset_NED:x(-payload_acc:x() * SLUP_VEL_P:get() + (-pos_offset_NED:x() - payload_offset_NED:x()) * SLUP_WP_POS_P:get())
    vel_offset_NED:y(-payload_acc:y() * SLUP_VEL_P:get() + (-pos_offset_NED:y() - payload_offset_NED:y()) * SLUP_WP_POS_P:get())
    if poscontrol:set_posvelaccel_offset(pos_offset_NED, vel_offset_NED, Vector3f()) then
        sent_velocity_offsets_ms = millis()
    end
end

-- display welcome message
gcs:send_text(MAV_SEVERITY.INFO, "copter-slung-payload script loaded")

-- update function to receive location from payload and move vehicle to reduce payload's oscillation
function update()

    -- exit immediately if not enabled
    if (SLUP_ENABLE:get() <= 0) then
        return update, 1000
    end

    -- get vehicle location
    local curr_loc = ahrs:get_location()
    if curr_loc == nil then
        return update, UPDATE_INTERVAL_MS
    end

    -- consume all available mavlink messages
    local payload_update_received = false
    local msg
    repeat
        msg, _ = mavlink:receive_chan()
        if (msg ~= nil) then
            local parsed_msg = mavlink_msgs.decode(msg, msg_map)
            if (parsed_msg ~= nil) then
                if parsed_msg.msgid == GLOBAL_POSITION_INT_ID then
                    if handle_global_position_int(parsed_msg) then
                        payload_update_received = true
                    end
                end
            end
        end
    until msg == nil

    -- warn user on loss of recovery of telemetry from payload
    local payload_timeout = millis() - payload_loc_update_ms > PAYLOAD_UPDATE_TIMEOUT_MS
    if payload_timeout ~= payload_update_timeout_prev then
        if payload_timeout then
            gcs:send_text(MAV_SEVERITY.WARNING, string.format("%s payload updates lost", TEXT_PREFIX_STR))
        else
            gcs:send_text(MAV_SEVERITY.INFO, string.format("%s payload updates received", TEXT_PREFIX_STR))
        end
    end
    payload_update_timeout_prev = payload_timeout

    if payload_update_received then
        -- calculate position difference vs vehicle
        payload_dist_NED = curr_loc:get_distance_NED(payload_loc)

        -- estimate the payload's resting position offset based on its current offset and velocity
        -- relies on payload_dist_NED and payload_vel being updated
        update_payload_resting_offset()
    end

    -- check if we can control the vehicle
    -- vehicle must be in Auto mode executing a SCRIPT_TIME or PAYLOAD_PLACE command
    local send_velocity_offsets_prev = send_velocity_offsets
    local armed_and_flying = arming:is_armed() and vehicle:get_likely_flying()
    local takingoff_or_landing = vehicle:is_landing() or vehicle:is_taking_off()
    local auto_mode = (vehicle:get_mode() == COPTER_MODE_AUTO)
    local scripting_or_payloadplace = (mission:get_current_nav_id() == MAV_CMD_NAV_SCRIPT_TIME) or (mission:get_current_nav_id() == MAV_CMD_NAV_PAYLOAD_PLACE)
    send_velocity_offsets = armed_and_flying and not takingoff_or_landing and auto_mode and scripting_or_payloadplace and not payload_timeout

    -- alert user if we start or stop sending velocity offsets
    if (send_velocity_offsets and not send_velocity_offsets_prev) then
        gcs:send_text(MAV_SEVERITY.INFO, string.format("%s activated", TEXT_PREFIX_STR))
    end
    if (not send_velocity_offsets and send_velocity_offsets_prev) then
        gcs:send_text(MAV_SEVERITY.INFO, string.format("%s deactivated", TEXT_PREFIX_STR))
        poscontrol:set_posvelaccel_offset(Vector3f(), Vector3f(), Vector3f())
    end

    -- move vehicle to reduce payload oscillation
    if send_velocity_offsets then
        move_vehicle()

        -- check for unexpected control timeout
        -- reset vehicle offsets if not sent within last 3 seconds
        local now_ms = millis()
        local time_since_vel_offset_sent = now_ms - sent_velocity_offsets_ms
        local time_since_last_control_timeout = now_ms - control_timeout_ms
        if (time_since_vel_offset_sent > CONTROL_TIMEOUT_MS) and (time_since_last_control_timeout > CONTROL_TIMEOUT_MS) then
            poscontrol:set_posvelaccel_offset(Vector3f(), Vector3f(), Vector3f())
            control_timeout_ms = now_ms
            print_warning("control timeout, clearing offsets")
        end
    end

    return update, UPDATE_INTERVAL_MS
end

return update()
