-- Inspect parameter sets received via MAVLink, determine action based
-- on whitelist.

-- When this script runs and ENABLE is true ArduPilot will stop
--   processing parameter-sets via the GCS library.  Instead, this
--   script becomes responsible for setting parameters, and it will
--   only set parameters which are whitelisted.  Setting ENABLE to
--   false will allow ArduPilot to set parameters normally.

-- Setting SCR_ENABLE to false while this script is running in the
--   ENABLE state is... not advised.

-- How To Use
-- 1. copy this script to the autopilot's "scripts" directory
-- 2. within the "scripts" directory create a "modules" directory
-- 3. copy the mavlink/mavlink_msgs_xxx files to the "scripts" directory in a "modules/mavlink" folder

-- load mavlink message definitions from modules/mavlink directory
local mavlink_msgs = require("mavlink/mavlink_msgs")

-- global definitions
local MAV_SEVERITY = {EMERGENCY=0, ALERT=1, CRITICAL=2, ERROR=3, WARNING=4, NOTICE=5, INFO=6, DEBUG=7}
local UPDATE_INTERVAL_MS = 10           -- update at about 100hz

-- get a reference to MAV_SYSID parameter so we can filter messages to
--   just those parameter-sets aimed at this vehicle:
MAV_SYSID = Parameter("MAV_SYSID")

-- mavlink definitions
local PARAM_SET_ID = 23
local msg_map = {}
msg_map[PARAM_SET_ID] = "PARAM_SET"

local TEXT_PREFIX_STR = "param-set"    -- prefix for all text messages

--
-- parameter setup
--
local PARAM_TABLE_KEY = 92
local PARAM_TABLE_PREFIX = "PS_"
assert(param:add_table(PARAM_TABLE_KEY, PARAM_TABLE_PREFIX, 7), 'could not add param table')

-- add a parameter and bind it to a variable
function bind_add_param(name, idx, default_value)
    assert(param:add_param(PARAM_TABLE_KEY, idx, name, default_value), string.format('could not add param %s', PARAM_TABLE_PREFIX .. name))
    return Parameter(PARAM_TABLE_PREFIX .. name)
end

--[[
  // @Param: PS_ENABLE
  // @DisplayName: Param Set enable
  // @Description: Param Set enable
  // @Values: 0:Disabled,1:Enabled
  // @User: Standard
--]]
local PS_ENABLE = bind_add_param("ENABLE", 1, 1)

-- initialize MAVLink rx with buffer depth and number of rx message IDs to register
mavlink:init(5, 1)

-- register message id to receive
mavlink:register_rx_msgid(PARAM_SET_ID)

-- handle PARAM_SET message
local parameters_which_can_be_set = {}
parameters_which_can_be_set["MAV_OPTIONS"] = true
parameters_which_can_be_set["PS_ENABLE"] = true
parameters_which_can_be_set["BATT_ARM_MAH"] = true
parameters_which_can_be_set["BATT_ARM_VOLT"] = true
parameters_which_can_be_set["BATT_CAPACITY"] = true
parameters_which_can_be_set["BATT_CRT_MAH"] = true
parameters_which_can_be_set["BATT_CRT_VOLT"] = true
parameters_which_can_be_set["BATT_FS_CRT_ACT"] = true
parameters_which_can_be_set["BATT_FS_LOW_ACT"] = true
parameters_which_can_be_set["BATT_LOW_MAH"] = true
parameters_which_can_be_set["BATT_LOW_VOLT"] = true
parameters_which_can_be_set["BRD_OPTIONS"] = true
parameters_which_can_be_set["COMPASS_USE3"] = true
parameters_which_can_be_set["FENCE_ACTION"] = true
parameters_which_can_be_set["FENCE_ALT_MAX"] = true
parameters_which_can_be_set["FENCE_ENABLE"] = true
parameters_which_can_be_set["FENCE_RADIUS"] = true
parameters_which_can_be_set["FENCE_TYPE"] = true
parameters_which_can_be_set["LIGHTS_ON"] = true
parameters_which_can_be_set["LOG_BITMASK"] = true
parameters_which_can_be_set["LOG_DISARMED"] = true
parameters_which_can_be_set["LOG_FILE_DSRMROT"] = true
parameters_which_can_be_set["RTL_ALT"] = true
parameters_which_can_be_set["RTL_LOIT_TIME"] = true
parameters_which_can_be_set["RTL_SPEED"] = true

function should_set_parameter_id(param_id)
    if parameters_which_can_be_set[param_id] == nil then
        return false
    end
    return true
end

function handle_param_set(msg)
    gcs:send_text(MAV_SEVERITY.WARNING, string.format("%s: param set received", TEXT_PREFIX_STR))

    -- ignore PARAM_SETs not aimed at our sysid:
    if msg.target_system ~= MAV_SYSID:get() then
        return
    end
    -- our component ID is assumed to be 1:
    if msg.target_component ~= 1 then
        return
    end
    -- check for this specific ID:
    if not should_set_parameter_id(msg.param_id) then
        gcs:send_text(MAV_SEVERITY.WARNING, string.format("%s: param set denied (%s)", TEXT_PREFIX_STR, msg.param_id))
        return
    end
    param:set(msg.param_id, msg.param_value)
end

-- display welcome message
gcs:send_text(MAV_SEVERITY.INFO, "param-set script loaded")

-- initialise our knowledge of the GCS's allow-set-parameters state.
--   We do not want to fight over setting this GCS state via other
--   mechanisms (eg. an auxiliary function), so we keep this state
--   around to track what we last set:
local gcs_allow_set = gcs:get_allow_param_set()

-- update function to receive param_set messages and perhaps act on them
function update()
    -- exit immediately if not enabled
    if (PS_ENABLE:get() <= 0) then
        -- this script is disabled, set allow-via-GCS (once):
        if not gcs_allow_set then
          gcs:set_allow_param_set(true)
          gcs_allow_set = true
        end
        return update, 1000
    end

    -- this script is enabled, disallow setting via normal means (once):
    if gcs_allow_set then
        gcs:set_allow_param_set(false)
        gcs_allow_set = false
    end

    -- consume all available mavlink messages
    while true do
        msg, _ = mavlink:receive_chan()
        if msg == nil then
            break
        end
        local parsed_msg = mavlink_msgs.decode(msg, msg_map)
        if parsed_msg == nil then
            goto continue
        end
        if parsed_msg.msgid == PARAM_SET_ID then
            handle_param_set(parsed_msg)
        end
        ::continue::
    end

    return update, UPDATE_INTERVAL_MS
end

return update()
