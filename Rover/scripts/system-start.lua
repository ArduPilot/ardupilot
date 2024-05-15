-- Control the system start and set the parameters for the system
SYSTEM_STARTED = 0

-- Severity for logging in GCS
MAV_SEVERITY = {EMERGENCY=0, ALERT=1, CRITICAL=2, ERROR=3, WARNING=4, NOTICE=5, INFO=6, DEBUG=7}

--[[
This function will set the parameters for the system. 
All the other scripts must look for BATT_SOC_SYSSTART parameter to be set before running
--]]
function system_start()
    -- Check if it was all set at first so we can skip this function
    if SYSTEM_STARTED == 1 then
        gcs:send_text(MAV_SEVERITY.NOTICE, 'System already started')
        return
    end

    -- The table creation is very critical. We must use an existing table index, otherwise it will not work
    -- We are using the BATT_SOC table as it has many parameters left (32), but only 1 is used so far
    -- We cannot add more than 31 parameters then (index 32)
    local PARAM_TABLE_KEY = 14
    assert(param:add_table(PARAM_TABLE_KEY, "BATT_SOC", 32), 'could not add param table')

    -- Add the parameters to the table
    -- OBS: Keep the index in order, otherwise it will not work
    -- OBS: Always use the name as short as possible, as the table has a limit and the parameter wont be set
    assert(param:add_param(PARAM_TABLE_KEY, 2,  '_ACTOBSAVD', 0), 'could not add ACTIVATE_OBSTACLE_AVOIDANCE')
    assert(param:add_param(PARAM_TABLE_KEY, 3,  '_OBSYAW', 0), 'could not add OBSTACLE_AVOIDANCE_YAW')
    assert(param:add_param(PARAM_TABLE_KEY, 4,  '_OBSTHR', 0), 'could not add OBSTACLE_AVOIDANCE_THROTTLE')
    assert(param:add_param(PARAM_TABLE_KEY, 5,  '_SYSSTART', 1), 'could not add SYSTEM_STARTED')
    
    -- Check if the parameter was set properly
    local system_started_param = Parameter()
    system_started_param:init('BATT_SOC_SYSSTART')
    SYSTEM_STARTED = system_started_param:get()
    gcs:send_text(MAV_SEVERITY.NOTICE, string.format("SYSTEM STARTED was set with value = %d", SYSTEM_STARTED))

    -- Run the system start function faster if we were not able to set the parameter
    if SYSTEM_STARTED == 1 then
        gcs:send_text(MAV_SEVERITY.NOTICE, 'System started properly')
        return
    else
        return system_start, 100
    end
end

return system_start, 1000
