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
-- 2. set SCR_ENABLE to 1

-- global definitions
local MAV_SEVERITY = {EMERGENCY=0, ALERT=1, CRITICAL=2, ERROR=3, WARNING=4, NOTICE=5, INFO=6, DEBUG=7}
local UPDATE_INTERVAL_MS = 10           -- update at about 100hz

-- prefix for all text messages:
local TEXT_PREFIX_STR = "param-set"

--
-- parameter setup
--
local PARAM_TABLE_KEY = 92
local PARAM_TABLE_PREFIX = "PARAM_SET_"
assert(param:add_table(PARAM_TABLE_KEY, PARAM_TABLE_PREFIX, 7), 'could not add param table')

-- add a parameter and bind it to a variable
function bind_add_param(name, idx, default_value)
    assert(param:add_param(PARAM_TABLE_KEY, idx, name, default_value), string.format('could not add param %s', PARAM_TABLE_PREFIX .. name))
    return Parameter(PARAM_TABLE_PREFIX .. name)
end

--[[
  // @Param: PARAM_SET_ENABLE
  // @DisplayName: Param Set enable
  // @Description: Param Set enable
  // @Values: 0:Disabled,1:Enabled
  // @User: Standard
--]]
local PARAM_SET_ENABLE = bind_add_param("ENABLE", 1, 1)

-- initialize MAVLink rx with buffer depth and number of rx message IDs to register
mavlink:init(5, 1)

-- register message id to receive
local PARAM_SET_ID = 23
mavlink:register_rx_msgid(PARAM_SET_ID)

-- handle PARAM_SET message
local parameters_which_can_be_set = {}
parameters_which_can_be_set["MAV_OPTIONS"] = true
parameters_which_can_be_set["PARAM_SET_ENABLE"] = true
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

local function should_set_parameter_id(param_id)
    if parameters_which_can_be_set[param_id] == nil then
        return false
    end
    return parameters_which_can_be_set[param_id]
end

local function handle_param_set(name, value)
    -- we will not receive packets in here for the wrong system ID /
    --   component ID; this is handled by ArduPilot's MAVLink routing
    --   code

    -- check for this specific ID:
    if not should_set_parameter_id(name) then
        gcs:send_text(MAV_SEVERITY.WARNING, string.format("%s: param set denied (%s)", TEXT_PREFIX_STR, name))
        return
    end

    param:set_and_save(name, value)
    gcs:send_text(MAV_SEVERITY.WARNING, string.format("%s: param set applied", TEXT_PREFIX_STR))
end

-- display welcome message
gcs:send_text(MAV_SEVERITY.INFO, "param-set script loaded")

-- initialise our knowledge of the GCS's allow-set-parameters state.
--   We do not want to fight over setting this GCS state via other
--   mechanisms (eg. an auxiliary function), so we keep this state
--   around to track what we last set:
local gcs_allow_set = gcs:get_allow_param_set()

-- update function to receive param_set messages and perhaps act on them
local function update()
    -- return immediately if not enabled
    if (PARAM_SET_ENABLE:get() <= 0) then
        -- this script is disabled, set allow-via-GCS (once):
        if not gcs_allow_set then
          gcs:set_allow_param_set(true)
          gcs_allow_set = true
        end
        -- drain all mavlink messages to avoid processing them when enabled
        while true do
          local msg, _ = mavlink:receive_chan()
          if msg == nil then
            break
          end
        end
        return
    end

    -- this script is enabled, disallow setting via normal means (once):
    if gcs_allow_set then
        gcs:set_allow_param_set(false)
        gcs_allow_set = false
    end

    -- consume all available mavlink messages
    while true do
        local msg, _ = mavlink:receive_chan()
        if msg == nil then
            break
        end

        local param_value, _, _, param_id, _ = string.unpack("<fBBc16B", string.sub(msg, 13, 36))
        param_id = string.gsub(param_id, string.char(0), "")

        handle_param_set(param_id, param_value)
    end
end

-- wrapper around update(). This calls update() with an interval of
-- UPDATE_INTERVAL_MS, and if update faults then an error is
-- displayed, but the script is not stopped

function protected_wrapper()
  local success, err = pcall(update)
  if not success then
     gcs:send_text(MAV_SEVERITY.EMERGENCY, "Internal Error: " .. err)
     -- when we fault we run the update function again after 1s, slowing it
     -- down a bit so we don't flood the console with errors
     return protected_wrapper, 1000
  end
  return protected_wrapper, UPDATE_INTERVAL_MS
end

-- start running update loop
return protected_wrapper()
