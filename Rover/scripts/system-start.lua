-- Function to define the parameters table that will be used throughout the system execution
function system_start()
    -- The table key must be a number between 0 and 200. The key is persistent in storage
    local PARAM_TABLE_KEY = 180

    -- Create a parameter table with N parameters in it. A table can have at most 63 parameters.
    -- The prefix is used to ensure another script doesn't use the same PARAM_TABLE_KEY.
    local n = 4
    assert(param:add_table(PARAM_TABLE_KEY, "ROVER_SYSTEM_", n), 'Could not add param table')

    -- Create the parameters. All added parameters are floats, with the given default values.
    assert(param:add_param(PARAM_TABLE_KEY, 1, 'ACTIVATE_OBSTACLE_AVOIDANCE', 0.0), 'could not add ACTIVATE_OBSTACLE_AVOIDANCE')
    assert(param:add_param(PARAM_TABLE_KEY, 2, 'OBSTACLE_AVOIDANCE_YAW', 0.0), 'could not add OBSTACLE_AVOIDANCE_YAW')
    assert(param:add_param(PARAM_TABLE_KEY, 3, 'OBSTACLE_AVOIDANCE_THROTTLE', 0.0), 'could not add OBSTACLE_AVOIDANCE_THROTTLE')

    -- After all parameters are created, we can enable the other scripts to run their control loops
    -- OBS - ALWAYS REMEMBER TO UPDATE THIS PARAMETER INDEX
    assert(param:add_param(PARAM_TABLE_KEY, 4, 'TEST2', 1.0), 'could not add param2')
end

return system_start
