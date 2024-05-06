SYSTEM_STARTED = 0.0

-- Function to define the parameters table that will be used throughout the system execution
function system_start()

    -- If system is started, no need to run
    if SYSTEM_STARTED == 1.0 then
        return
    end

    -- The table key must be a number between 0 and 200. The key is persistent in storage
    local PARAM_TABLE_KEY = 180

    -- Create a parameter table with N parameters in it. A table can have at most 63 parameters.
    -- The prefix is used to ensure another script doesn't use the same PARAM_TABLE_KEY.
    -- OBS: ALWAYS REMEMBER TO UPDATE THIS VALUE IF ADDING A NEW PARAMETER!
    local n = 4
    assert(param:add_table(PARAM_TABLE_KEY, "ROVER_SYSTEM_", n), 'Could not add param table')

    -- Create the parameters. All added parameters are floats, with the given default values.
    assert(param:add_param(PARAM_TABLE_KEY, 1, 'ACTIVATE_OBSTACLE_AVOIDANCE', 0.0), 'could not add ACTIVATE_OBSTACLE_AVOIDANCE')
    assert(param:add_param(PARAM_TABLE_KEY, 2, 'OBSTACLE_AVOIDANCE_YAW', 0.0), 'could not add OBSTACLE_AVOIDANCE_YAW')
    assert(param:add_param(PARAM_TABLE_KEY, 3, 'OBSTACLE_AVOIDANCE_THROTTLE', 0.0), 'could not add OBSTACLE_AVOIDANCE_THROTTLE')

    -- After all parameters are created, we can enable the other scripts to run their control loops
    assert(param:add_param(PARAM_TABLE_KEY, n, 'SYSTEM_STARTED', 1.0), 'could not add SYSTEM_STARTED')
    local system_started_test = Parameter("ROVER_SYSTEM_SYSTEM_STARTED")
    -- If the parameter is correctly set to 1, it is all set and we can skip this function in next calls
    if system_started_test:get() == 1.0 then
        SYSTEM_STARTED = 1.0
        gcs:send_text(4, string.format("We set the system table with result = %d",  SYSTEM_STARTED))
    end

end

return system_start, 1000
