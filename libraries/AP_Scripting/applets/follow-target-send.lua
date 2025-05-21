-- Send the FOLLOW_TARGET mavlink message to allow other vehicles to follow this one
--
-- How To Use
-- 1. copy this script to the autopilot's "scripts" directory
-- 2. within the "scripts" directory create a "modules" directory
-- 3. copy the MAVLink/mavlink_msgs_xxx files to the "scripts" directory
-- 4. the FOLLOW_TARGET message will be published at 10hz

-- load mavlink message definitions from modules/MAVLink directory
local mavlink_msgs = require("MAVLink/mavlink_msgs")

-- global definitions
local MAV_SEVERITY = {EMERGENCY=0, ALERT=1, CRITICAL=2, ERROR=3, WARNING=4, NOTICE=5, INFO=6, DEBUG=7}
local UPDATE_INTERVAL_MS = 100                  -- update at about 10hz
local FOLLOW_TARGET_CAPABILITIES = {POS=2^0, VEL=2^1, ACCEL=2^2, ATT_RATES=2^3}

 -- setup script specific parameters
local PARAM_TABLE_KEY = 88
local PARAM_TABLE_PREFIX = "FOLT_"
assert(param:add_table(PARAM_TABLE_KEY, PARAM_TABLE_PREFIX, 2), 'could not add param table')

-- add a parameter and bind it to a variable
local function bind_add_param(name, idx, default_value)
    assert(param:add_param(PARAM_TABLE_KEY, idx, name, default_value), string.format('could not add param %s', PARAM_TABLE_PREFIX .. name))
    return Parameter(PARAM_TABLE_PREFIX .. name)
end

--[[
  // @Param: FOLT_ENABLE
  // @DisplayName: Follow Target Send Enable
  // @Description: Follow Target Send Enable
  // @Values: 0:Disabled,1:Enabled
  // @User: Standard
--]]
local FOLT_ENABLE = bind_add_param("ENABLE", 1, 1)

--[[
  // @Param: FOLT_MAV_CHAN
  // @DisplayName: Follow Target Send MAVLink Channel
  // @Description: MAVLink channel to which FOLLOW_TARGET should be sent
  // @Range: 0 10
  // @User: Standard
--]]
local FOLT_MAV_CHAN = bind_add_param("MAV_CHAN", 2, 0)

-- send FOLLOW_TARGET message
local function send_follow_target_msg()

    -- get vehicle location
    local curr_loc = ahrs:get_location()
    if curr_loc == nil then
        do return end
    end
    local capabilities = FOLLOW_TARGET_CAPABILITIES.POS

    -- get vehicle target velocity in m/s in NED frame
    local vel_target_NED = poscontrol:get_vel_target()
    if vel_target_NED ~= nil then
        capabilities = capabilities + FOLLOW_TARGET_CAPABILITIES.VEL
    else
        vel_target_NED = Vector3f()
    end

    -- get vehicle target acceleration in m/s/s in NED frame
    local accel_target_NED = poscontrol:get_accel_target()
    if accel_target_NED ~= nil then
        capabilities = capabilities + FOLLOW_TARGET_CAPABILITIES.ACCEL
    else
        accel_target_NED = Vector3f()
    end

    -- get vehicle current attitude as quaternion and rates
    local attitude_quat = ahrs:get_quaternion()
    local curr_rot_rate = ahrs:get_gyro()
    if attitude_quat ~= nil and curr_rot_rate ~= nil then
        capabilities = capabilities + FOLLOW_TARGET_CAPABILITIES.ATT_RATES
    else
        attitude_quat = Quaternion()
        curr_rot_rate = Vector3f()
    end
    local curr_rot_rate_NED = ahrs:body_to_earth(curr_rot_rate)

    -- prepare FOLLOW_TARGET message
    local follow_target_msg = {}
    follow_target_msg.timestamp = millis():toint()
    follow_target_msg.est_capabilities = capabilities
    follow_target_msg.lat = curr_loc:lat()
    follow_target_msg.lon = curr_loc:lng()
    follow_target_msg.alt = curr_loc:alt() * 0.01
    follow_target_msg.vel = {vel_target_NED:x(), vel_target_NED:y(), vel_target_NED:z()}
    follow_target_msg.acc = {accel_target_NED:x(), accel_target_NED:y(), accel_target_NED:z()}
    follow_target_msg.attitude_q = {attitude_quat:q1(), attitude_quat:q2(), attitude_quat:q3(), attitude_quat:q4()}
    follow_target_msg.rates = {curr_rot_rate_NED:x(), curr_rot_rate_NED:y(), curr_rot_rate_NED:z()}
    follow_target_msg.position_cov = {0, 0, 0}
    follow_target_msg.custom_state = 0

    -- send FOLLOW_TARGET message
    mavlink:send_chan(FOLT_MAV_CHAN:get(), mavlink_msgs.encode("FOLLOW_TARGET", follow_target_msg))
end

-- display welcome message
gcs:send_text(MAV_SEVERITY.INFO, "follow-target-send script loaded")

-- update function to receive location from payload and move vehicle to reduce payload's oscillation
local function update()

    -- exit immediately if not enabled
    if (FOLT_ENABLE:get() <= 0) then
        return update, 1000
    end

    -- send FOLLOW_TARGET message
    send_follow_target_msg()

    return update, UPDATE_INTERVAL_MS
end

return update()
