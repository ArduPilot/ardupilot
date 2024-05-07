SYSTEM_STARTED = 0.0

-- The table key must be a number between 0 and 200. The key is persistent in storage
local PARAM_TABLE_KEY = 180
local PARAM_TABLE_PREFIX = "ROVER_SYSTEM_"

-- add a parameter and bind it to a variable
function bind_add_param(name, idx, default_value)
    assert(param:add_param(PARAM_TABLE_KEY, idx, name, default_value), string.format('could not add param %s', name))
    return Parameter(PARAM_TABLE_PREFIX .. name)
end

-- Severity for logging in GCS
local MAV_SEVERITY = {EMERGENCY=0, ALERT=1, CRITICAL=2, ERROR=3, WARNING=4, NOTICE=5, INFO=6, DEBUG=7}

-- Function to define the parameters table that will be used throughout the system execution
function system_start()

    -- If system is started, no need to run
    if SYSTEM_STARTED == 1.0 then
        gcs:send_text(MAV_SEVERITY.WARNING, string.format("System is all set, we can start now... %d",  SYSTEM_STARTED))
        return
    end

    -- Create a parameter table with N parameters in it. A table can have at most 63 parameters.
    -- The prefix is used to ensure another script doesn't use the same PARAM_TABLE_KEY.
    -- OBS: ALWAYS REMEMBER TO UPDATE THIS VALUE IF ADDING A NEW PARAMETER!
    local n = 4
    assert(param:add_table(PARAM_TABLE_KEY, PARAM_TABLE_PREFIX, n), 'Could not add ROVER SYSTEM param table')

    ACTIVATE_OBSTACLE_AVOIDANCE = bind_add_param('ACTIVATE_OBSTACLE_AVOIDANCE', 1, 0.0)
    OBSTACLE_AVOIDANCE_YAW = bind_add_param('OBSTACLE_AVOIDANCE_YAW', 2, 0.0)
    OBSTACLE_AVOIDANCE_THROTTLE = bind_add_param('OBSTACLE_AVOIDANCE_THROTTLE', 3, 0.0)
    -- After all parameters are created, we can enable the other scripts to run their control loops
    SYSTEM_STARTED_PARAM = bind_add_param('SYSTEM_STARTED', 4, 0.0)
    
    -- If the parameter is correctly set to 1, it is all set and we can skip this function in next calls
    if SYSTEM_STARTED_PARAM:get() == 1.0 then
        SYSTEM_STARTED = 1.0
        gcs:send_text(MAV_SEVERITY.WARNING, string.format("We set the system table with result = %d",  SYSTEM_STARTED))
    end

end

return system_start, 1000
