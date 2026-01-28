-- Sample health monitoring script that will monitor some key subsytems when the vehicle is armed.
-- If any of the subsystems are unhealthy, the script will prevent the vehicle from arming the next time by setting a bit in the HEALTH_STATUS bitmask.

-- Define parameter table keys and prefix
local PARAM_TABLE_KEY = 157
local PARAM_TABLE_PREFIX = "HEALTH_"

-- Global arming authorization ID
local auth_id = arming:get_aux_auth_id()

-- Bind a parameter to a variable
function bind_param(name)
    local p = Parameter()
    assert(p:init(name), string.format('Could not find %s parameter', name))
    return p
end

-- Add a parameter and bind it to a variable
function bind_add_param(name, idx, default_value)
    assert(param:add_param(PARAM_TABLE_KEY, idx, name, default_value), string.format('Could not add param %s', name))
    return bind_param(PARAM_TABLE_PREFIX .. name)
end

local STATS_FLTTIME = bind_param("STAT_FLTTIME")

-- Setup parameters
assert(param:add_table(PARAM_TABLE_KEY, PARAM_TABLE_PREFIX, 11), 'Could not add param table')

-- Define the enable parameter
--[[
  // @Param: HEALTH_ENABLE
  // @DisplayName: Enable Health Monitoring Script
  // @Description: Enable or disable the health monitoring script. Set to 1 to enable, 0 to disable.
  // @Values: 0:Disable, 1:Enable
  // @User: Standard
]]
local HEALTH_ENABLE = bind_add_param("ENABLE", 1, 0)

-- Define the bitmask parameter
--[[
  // @Param: HEALTH_STATUS
  // @DisplayName: System Health Bitmask
  // @Description: Bitmask representing the health of various subsystems. Each bit indicates the health of a subsystem.
  // @Bitmask: 0:EKF, 1:Total flight time, 2:Other subsystem
  // @User: Advanced
]]
local HEALTH_STATUS = bind_add_param("STATUS", 2, 0) -- Default is all systems healthy

------------------- Various subsystem health-check functions -------------------
-- Subsystem health-check functions
local function check_ekf_health()
    -- Get variances from AHRS
    local vel_variance, pos_variance, height_variance, mag_variance, airspeed_variance = ahrs:get_variances()
    -- Check if any variance exceeds 1.5
    if vel_variance > 1.5 or pos_variance > 1.5 or height_variance > 1.5 or mag_variance:length() > 1.5 or airspeed_variance > 1.5 then
        return false
    end
    return true
end

local function check_total_flight_time()
    -- Fetch total flight time from STAT_FLTTIME
    local total_flight_time = STATS_FLTTIME:get() -- Flight time in seconds

    -- Define a threshold for the maximum allowable flight time (e.g., 1,000,000 seconds, or ~277.7 hours)
    local max_flight_time = 1000000

    if total_flight_time > max_flight_time then
        return false
    end
    return true
end

local function check_other_health()
    -- Placeholder for additional subsystem checks
    return true
end

------------------- Subsystem definitions -------------------

-- Subsystem definitions (bit, description, check function). Add more subsystems as needed. Add corresponding check functions above.
local subsystems = {
    {bit = 0, name = "EKF health", check = check_ekf_health},
    {bit = 1, name = "Total flight time", check = check_total_flight_time},
    {bit = 2, name = "Other subsystem", check = check_other_health}
}

------------------- Main Logic -------------------

-- Utility function to set or clear a bit in the bitmask
local function set_bit(mask, bit, healthy)
    if healthy then
        return mask & ~(1 << bit) -- Clear the bit if healthy (0)
    else
        return mask | (1 << bit) -- Set the bit if unhealthy (1)
    end
end

-- Function to check health of subsystems and update the bitmask
local function check_health()
    local bitmask = HEALTH_STATUS:get() -- Fetch the current bitmask

    for _, subsystem in ipairs(subsystems) do
        local is_healthy = subsystem.check()
        bitmask = set_bit(bitmask, subsystem.bit, is_healthy)
    end

    -- Update the HEALTH_STATUS parameter
    HEALTH_STATUS:set(bitmask)

    -- Notify if any subsystem is unhealthy
    for _, subsystem in ipairs(subsystems) do
        if (bitmask & (1 << subsystem.bit)) ~= 0 then -- Bit is 1, unhealthy
            gcs:send_text(5, "Warning: " .. subsystem.name .. " is unhealthy!")
        end
    end
end

-- Function to handle arming checks
local function handle_arming()
    local bitmask = HEALTH_STATUS:get()
    local arming_failed = false

    for _, subsystem in ipairs(subsystems) do
        if (bitmask & (1 << subsystem.bit)) ~= 0 then -- Bit is 1, unhealthy
            gcs:send_text(5, "Arming denied: " .. subsystem.name .. " check failed.")
            arming_failed = true
        end
    end

    if arming_failed then
        arming:set_aux_auth_failed(auth_id, "Health checks failed")
    else
        arming:set_aux_auth_passed(auth_id)
    end
end

-- Main update function
function update()
    -- Check if health monitoring is enabled
    if HEALTH_ENABLE:get() == 0 then
        arming:set_aux_auth_passed(auth_id) -- Allow arming if script is disabled
        return update, 1000 -- Check again in 1 second
    end

    if arming:is_armed() then
        check_health() -- Periodic health check while armed
    else
        -- When disarmed, save the updated bitmask permanently
        HEALTH_STATUS:set_and_save(HEALTH_STATUS:get())
        -- Handle arming checks
        handle_arming()
    end

    return update, 1000 -- Run update every second
end

-- Initialize
gcs:send_text(6, "Health monitoring script initialized.")
return update()
