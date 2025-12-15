--[[
    set_password.lua
    
    DESCRIPTION:
    This Lua script sets a new encryption key password in the LEIGH_KEY parameter
    and verifies that it was programmed successfully. The LEIGH_KEY parameter is
    used to derive the 32-byte encryption key for file encryption/decryption
    using BLAKE2b hashing with the salt "LEIGH_KEY_SALT_1".
    
    USAGE:
    1. Edit the NEW_PASSWORD value below (line 15) to your desired INT32 value
    2. Copy this script to your autopilot's SD card in the APM/scripts/ directory
    3. Ensure SCR_ENABLE = 1 to enable Lua scripting
    4. The script will run automatically when loaded
    
    REQUIREMENTS:
    - ArduPilot firmware with Lua scripting enabled (SCR_ENABLE = 1)
    - AP_Crypto library with LEIGH_KEY parameter support
    - SD card with APM/scripts/ directory
    
    CONFIGURATION:
    - NEW_PASSWORD: The INT32 value to set as the LEIGH_KEY parameter
      This value will be hashed with "LEIGH_KEY_SALT_1" using BLAKE2b to
      derive the 32-byte encryption key stored in secure storage.
      Range: -2147483648 to 2147483647 (INT32 limits)
    
    OUTPUT:
    The script will send messages to the GCS console (Mission Planner) with:
    - Current LEIGH_KEY value (if set)
    - Confirmation of setting attempt
    - Success message if verification passes
    - Error messages if setting or verification fails
    
    EXAMPLE OUTPUT:
    - "Current LEIGH_KEY: 74768360"
    - "Attempted to set LEIGH_KEY to: 74768361"
    - "SUCCESS: LEIGH_KEY verified as 74768361"
    
    NOTES:
    - The script uses param:set_and_save() to make the change persistent
    - The value is verified immediately after setting
    - If verification fails, check that the parameter exists and is writable
    - The derived encryption key will be stored in secure storage automatically
    - This script runs once when loaded (not a periodic update loop)
--]]

-- Wrap entire script in error handling
local success, err = pcall(function()
    -- Send initial message to confirm script is loaded
    gcs:send_text(6, 'set_password.lua: Script loaded and starting')
    
    -- Configure the new password value here (INT32)
    -- Change this to your desired password value
    -- This value will be used to derive the 32-byte encryption key
    local NEW_PASSWORD = 74768361  -- Change this to your desired password value
    
    --[[
        STEP 1: Read the current LEIGH_KEY parameter value
        This displays what the current value is before we change it
    --]]
    local old_value = param:get('LEIGH_KEY')
    
    if old_value then
        -- Display the current value to the console
        gcs:send_text(6, string.format('Current LEIGH_KEY: %d', old_value))
    else
        -- Parameter doesn't exist or isn't set yet
        gcs:send_text(6, 'LEIGH_KEY not found or not set')
    end
    
    --[[
        STEP 2: Set the new password value and save it permanently
        param:set_and_save() will:
        - Set the parameter value in RAM
        - Save it to non-volatile storage (EEPROM/FRAM)
        - Trigger key derivation and storage in secure storage
        Returns true on success, false on failure
    --]]
    local set_success = param:set_and_save('LEIGH_KEY', NEW_PASSWORD)
    
    if not set_success then
        -- Failed to set the parameter (parameter may not exist or be read-only)
        gcs:send_text(3, 'Failed to set LEIGH_KEY parameter')
    else
        -- Parameter set successfully, now verify it
        gcs:send_text(6, string.format('Attempted to set LEIGH_KEY to: %d', NEW_PASSWORD))
        
        --[[
            STEP 3: Verify the value was set correctly
            Read the parameter back immediately to confirm it matches what we set
        --]]
        local verify_value = param:get('LEIGH_KEY')
        
        if verify_value then
            -- Compare the read value with what we tried to set
            if verify_value == NEW_PASSWORD then
                -- Success! The value matches exactly
                gcs:send_text(6, string.format('SUCCESS: LEIGH_KEY verified as %d', verify_value))
            else
                -- Verification failed - value doesn't match (possible rounding issue or parameter constraint)
                gcs:send_text(3, string.format('VERIFICATION FAILED: Expected %d, got %d', NEW_PASSWORD, verify_value))
            end
        else
            -- Could not read the parameter after setting it
            gcs:send_text(3, 'VERIFICATION FAILED: Could not read LEIGH_KEY after setting')
        end
    end
end)

if not success then
    -- Report any errors that occurred during script execution
    gcs:send_text(0, 'set_password.lua FATAL ERROR: ' .. tostring(err))
end
