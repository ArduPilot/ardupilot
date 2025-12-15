--[[
    print_leigh_key.lua
    
    DESCRIPTION:
    This Lua script continuously prints the current LEIGH_KEY parameter value
    to the console once per second. Useful for monitoring the encryption key
    parameter value in real-time.
    
    USAGE:
    1. Copy this script to your autopilot's SD card in the APM/scripts/ directory
    2. Ensure SCR_ENABLE = 1 to enable Lua scripting
    3. The script will run automatically and print LEIGH_KEY every second
    
    REQUIREMENTS:
    - ArduPilot firmware with Lua scripting enabled (SCR_ENABLE = 1)
    - AP_Crypto library with LEIGH_KEY parameter support
    - SD card with APM/scripts/ directory
    
    OUTPUT:
    The script will send messages to the GCS console (Mission Planner) every second:
    - "LEIGH_KEY: <value>" if the parameter is set
    - "LEIGH_KEY: not found" if the parameter doesn't exist or isn't set
    
    NOTES:
    - The script runs continuously until SCR_ENABLE is set to 0
    - Update interval is 1000ms (1 second)
    - Uses severity level 6 (INFO) for console messages
--]]

--[[
    Update function that runs periodically
    This function is called repeatedly with a 1 second delay
--]]
function update()
    -- Wrap in error handling to catch any runtime errors
    local success, err = pcall(function()
        -- Get the current LEIGH_KEY parameter value
        local leigh_key_value = param:get('LEIGH_KEY')
        
        if leigh_key_value then
            -- Print the value to console
            gcs:send_text(6, string.format('LEIGH_KEY: %d', leigh_key_value))
        else
            -- Parameter not found or not set
            gcs:send_text(6, 'LEIGH_KEY: not found')
        end
    end)
    
    if not success then
        -- Report any errors that occurred
        gcs:send_text(3, 'print_leigh_key.lua ERROR: ' .. tostring(err))
    end
    
    -- Return the update function with a 1000ms (1 second) delay
    -- This schedules the function to run again after 1 second
    return update, 1000
end

-- Send initial message to confirm script is loaded
local success, err = pcall(function()
    gcs:send_text(6, 'print_leigh_key.lua: Script loaded and starting')
end)

if not success then
    -- If we can't even send a message, there's a serious problem
    -- Try with emergency severity
    gcs:send_text(0, 'print_leigh_key.lua: ERROR - ' .. tostring(err))
end

-- Start the update loop immediately
-- The function will run once, then reschedule itself every second
return update()



