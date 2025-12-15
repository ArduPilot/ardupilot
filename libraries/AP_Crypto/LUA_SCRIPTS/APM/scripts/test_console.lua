-- Simple test script to verify console output works
-- This script sends a message immediately when loaded

gcs:send_text(0, "TEST: Console output test script loaded")

-- Try different severity levels
gcs:send_text(0, "TEST: Emergency severity (0)")
gcs:send_text(3, "TEST: Error severity (3)")
gcs:send_text(6, "TEST: Info severity (6)")

-- Test parameter access
local test_param = param:get('SCR_ENABLE')
if test_param then
    gcs:send_text(6, string.format("TEST: SCR_ENABLE = %d", test_param))
else
    gcs:send_text(3, "TEST: Could not read SCR_ENABLE")
end

-- Test LEIGH_KEY access
local leigh_key = param:get('LEIGH_KEY')
if leigh_key then
    gcs:send_text(6, string.format("TEST: LEIGH_KEY = %d", leigh_key))
else
    gcs:send_text(6, "TEST: LEIGH_KEY not found or not set")
end

gcs:send_text(6, "TEST: Script completed")



