--[[
    crash-actions.lua augments FS_CRASH_CHECK (Copter/Rover) and CRASH_DETECT (Plane).

    If crash detection is enabled, this script enables additional, configurable
        post-crash actions. See crash-actions.md for configuration details.

    -- Yuri Rage, May 2026
]]

local SCRIPT_NAME          = "Crash Actions"
local UPDATE_MS            = 500
local MAV_SEVERITY         = { EMERGENCY = 0, ALERT = 1, CRITICAL = 2, ERROR = 3, WARNING = 4, NOTICE = 5, INFO = 6, DEBUG = 7 }
local MAV_CMD_DO_SET_SERVO = 183
local ACTION               = {
    DISARM         = 1,
    SET_RELAY_OFF  = 2,
    SET_RELAY_ON   = 3,
    SET_SERVO_MIN  = 4,
    SET_SERVO_TRIM = 5,
    SET_SERVO_MAX  = 6,
}
local SERVO_PARAM          = {
    [ACTION.SET_SERVO_MIN]  = "MIN",
    [ACTION.SET_SERVO_TRIM] = "TRIM",
    [ACTION.SET_SERVO_MAX]  = "MAX",
}
local MAX_INSTANCES        = 32 -- max number of relay/servo instances

-- parameter table variables
local NUM_CRASH_ACTIONS    = 4
local PARAM_TABLE_PREFIX   = "CRASH_"
local PARAM_TABLE_SIZE     = NUM_CRASH_ACTIONS * 3
local PARAM_TABLE_KEY      = 188

-- parameter variables bound at runtime
local CRASH_ACTIONS        = {}

-- forward declarations for state machine functions
local standby              = nil
local do_crash_actions     = nil
local protected_wrapper    = nil

local valid_relays         = {}
local valid_servos         = {}

local crash_time_ms        = uint32_t(0)

-- determine valid (configured) servo/relay instances
for i = 1, MAX_INSTANCES do
    local val = param:get(string.format("RELAY%d_FUNCTION", i)) or 0
    if val > 0 then
        valid_relays[i] = true
    end

    val = param:get(string.format("SERVO%d_FUNCTION", i)) or 0
    if val > 0 then
        valid_servos[i] = true
    end
end

-- add a parameter and bind it to a variable
local function bind_add_param(name, idx, default_value)
    assert(param:add_param(PARAM_TABLE_KEY, idx, name, default_value),
        string.format("%s: could not add param %s", SCRIPT_NAME, name))
    return Parameter(PARAM_TABLE_PREFIX .. name)
end

-- add CRASH_ param table
assert(param:add_table(PARAM_TABLE_KEY, PARAM_TABLE_PREFIX, PARAM_TABLE_SIZE),
    string.format("%s: could not add param table", SCRIPT_NAME))

-- bind CRASH_ params
for i = 1, PARAM_TABLE_SIZE - 1, 3 do
    local idx = (i + 2) // 3
    CRASH_ACTIONS[idx] = {
        action = bind_add_param(string.format("ACT%d", idx), i, 0),
        instance = bind_add_param(string.format("ACT%d_INST", idx), i + 1, 0),
        timeout = bind_add_param(string.format("ACT%d_TIME", idx), i + 2, 0),
        complete = false,
    }
end

-- handle a crash action
-- sends GCS message on failure
local function handle_crash_action(action_type, instance)
    if action_type == ACTION.DISARM then
        gcs:send_text(MAV_SEVERITY.NOTICE, string.format("%s: Disarming", SCRIPT_NAME))
        arming:disarm()
        return
    end

    if action_type == ACTION.SET_RELAY_OFF or action_type == ACTION.SET_RELAY_ON then
        if not valid_relays[instance] then
            gcs:send_text(MAV_SEVERITY.ERROR, string.format("%s: RELAY%d not configured", SCRIPT_NAME, instance))
            return
        end
        local state = (action_type == ACTION.SET_RELAY_OFF) and "OFF" or "ON"
        gcs:send_text(MAV_SEVERITY.NOTICE, string.format("%s: Setting RELAY%d %s", SCRIPT_NAME, instance, state))
        if action_type == ACTION.SET_RELAY_OFF then
            relay:off(instance - 1)
        else
            relay:on(instance - 1)
        end
        return
    end

    if action_type == ACTION.SET_SERVO_MIN or action_type == ACTION.SET_SERVO_TRIM or action_type == ACTION.SET_SERVO_MAX then
        if not valid_servos[instance] then
            gcs:send_text(MAV_SEVERITY.ERROR, string.format("%s: SERVO%d not configured", SCRIPT_NAME, instance))
            return
        end

        local suffix = SERVO_PARAM[action_type]
        local val = param:get(string.format("SERVO%d_%s", instance, suffix))
        if not val then
            gcs:send_text(MAV_SEVERITY.ERROR,
                string.format("%s: Failed to get SERVO%d_%s", SCRIPT_NAME, instance, suffix))
            return
        end
        gcs:send_text(MAV_SEVERITY.NOTICE, string.format("%s: Setting SERVO%d %s", SCRIPT_NAME, instance, suffix))
        -- uses MAV_CMD_DO_SET_SERVO rather than SRV_Channels:set_() bindings
        -- gets ignore_small_rcin_changes() behavior used by AP_ServoRelayEvents
        gcs:run_command_int(MAV_CMD_DO_SET_SERVO, { p1 = instance, p2 = val })
        return
    end

    gcs:send_text(MAV_SEVERITY.ERROR, string.format("%s: Unknown action %d", SCRIPT_NAME, action_type))
end

-- state machine function to iterate over crash actions
-- returns next state function
do_crash_actions = function()
    if not vehicle:is_crashed() then
        gcs:send_text(MAV_SEVERITY.NOTICE, string.format("%s: Recovered", SCRIPT_NAME))
        return standby
    end

    for _, crash_action in ipairs(CRASH_ACTIONS) do
        local action = crash_action.action:get()

        if not crash_action.complete and
            action > 0 and
            millis() - crash_time_ms > (crash_action.timeout:get() * 1000) then
            handle_crash_action(action, crash_action.instance:get())
            crash_action.complete = true
        end
    end

    return do_crash_actions
end

-- state machine function active until crash detection
-- returns next state function
standby = function()
    if vehicle:is_crashed() then
        crash_time_ms = millis()
        for _, crash_action in ipairs(CRASH_ACTIONS) do
            crash_action.complete = (crash_action.action:get() == 0)
        end
        gcs:send_text(MAV_SEVERITY.EMERGENCY, string.format("%s: Executing...", SCRIPT_NAME))
        return do_crash_actions
    end
    return standby
end

-- wrapper around the state machine to handle errors
-- if a state machine function fails, an error message is sent and the function is retried after 1s
local result = standby
protected_wrapper = function()
    local last_fn = result
    local success
    success, result = pcall(result)
    if not success then
        gcs:send_text(MAV_SEVERITY.ERROR, "Internal Error: " .. result)
        result = standby -- reset the state machine
        return protected_wrapper, 1000
    end

    if last_fn == standby and result == do_crash_actions then
        -- execute crash actions ASAP if changing state from standby
        return protected_wrapper, 0
    end
    return protected_wrapper, UPDATE_MS
end

-- Copter/Rover firmware crash detection check/warning
if param:get("FS_CRASH_CHECK") == 0 then
    gcs:send_text(MAV_SEVERITY.WARNING, string.format("%s: FS_CRASH_CHECK is disabled", SCRIPT_NAME))
end

-- Plane firmware crash detection check/warning
if param:get("CRASH_DETECT") == 0 then
    gcs:send_text(MAV_SEVERITY.WARNING, string.format("%s: CRASH_DETECT is disabled", SCRIPT_NAME))
end

gcs:send_text(MAV_SEVERITY.INFO, string.format("%s: Script loaded", SCRIPT_NAME))

return protected_wrapper()
