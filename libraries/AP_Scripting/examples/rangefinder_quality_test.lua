
-- This test uses the Range Finder driver interface to simulate Range Finder
-- hardware and uses the Range Finder client interface to simulate a
-- client of the driver. The test sends distance data through the driver
-- interface and validates that it can be read through the client interface.

-- Parameters should be set as follows before this test is loaded.
-- "RNGFND1_TYPE": 36,
-- "RNGFND1_ORIENT": 25,
-- "RNGFND1_MIN": 0.10,
-- "RNGFND1_MAX": 50.00,

---@diagnostic disable: cast-local-type


-- UPDATE_PERIOD_MS is the time between when a distance is set and
-- when it is read. There is a periodic task that copies the set distance to
-- the state structure that it is read from. If UPDATE_PERIOD_MS is too short this periodic
-- task might not get a chance to run. A value of 25 seems to be too quick for sub.
local UPDATE_PERIOD_MS = 50
local TIMEOUT_MS = 5000

-- These strings must match the strings used by the test driver for interpreting the output from this test.
local TEST_ID_STR = "RQTL"
local COMPLETE_STR = "#complete#"
local SUCCESS_STR = "!!success!!"
local FAILURE_STR = "!!failure!!"


-- Copied from libraries/AP_Math/rotation.h enum Rotation {}.
local RNGFND_ORIENTATION_DOWN = 25
local RNGFND_ORIENTATION_FORWARD = 0
-- Copied from libraries/AP_RangeFinder/AP_RanggeFinder.h enum RangeFinder::Type {}.
local RNGFND_TYPE_LUA = 36.0
-- Copied from libraries/AP_RangeFinder/AP_RangeFinder.h enum RangeFinder::Status {}.
local RNDFND_STATUS_NOT_CONNECTED = 0
local RNDFND_STATUS_OUT_OF_RANGE_LOW = 2
local RNDFND_STATUS_OUT_OF_RANGE_HIGH = 3
local RNDFND_STATUS_GOOD = 4
-- Copied from libraries/AP_RangeFinder/AP_RangeFinder.h
local SIGNAL_QUALITY_MIN = 0
local SIGNAL_QUALITY_MAX = 100
local SIGNAL_QUALITY_UNKNOWN = -1

-- Read parameters for min and max valid range finder ranges.
local RNGFND1_MIN = Parameter("RNGFND1_MIN"):get()
local RNGFND1_MAX = Parameter("RNGFND1_MAX"):get()

local function send(str)
    gcs:send_text(3, string.format("%s %s", TEST_ID_STR, str))
end


-- The range finder backend is initialized in the update_prepare function.
---@type AP_RangeFinder_Backend_ud
local rngfnd_backend


local function test_dist_equal(dist_m_in, dist_in_factor, dist_out, signal_quality_pct_in, signal_quality_pct_out)
    if math.abs(dist_out - dist_m_in * dist_in_factor) > 1.0e-3 then
        return false
    end
    if signal_quality_pct_in < 0 and signal_quality_pct_out == -1 then
        return true
    end
    if signal_quality_pct_in > 100 and signal_quality_pct_out == -1 then
        return true
    end
    if signal_quality_pct_in == signal_quality_pct_out then
        return true
    end
    return false
end

local function get_and_eval(test_idx, dist_m_in, signal_quality_pct_in, status_expected)
    local status_actual = rangefinder:status_orient(RNGFND_ORIENTATION_DOWN)

    -- Check that the status is as expected
    if status_expected ~= status_actual then
        return string.format("Status test %i status incorrect - expected %i, actual %i", test_idx, status_expected, status_actual)
    end

    -- Not more checks if the status is poor
    if status_actual ~= RNDFND_STATUS_GOOD then
        send(string.format("Status test %i status correct - expected: %i actual: %i", test_idx, status_expected, status_actual))
        return nil
    end

    -- L U A   I N T E R F A C E   T E S T
    -- Check that the distance and signal_quality from the frontend are as expected
    local distance1_cm_out = rangefinder:distance_orient(RNGFND_ORIENTATION_DOWN) * 100
    local signal_quality1_pct_out = rangefinder:signal_quality_pct_orient(RNGFND_ORIENTATION_DOWN)

    -- Make sure data was returned
    if not distance1_cm_out or not signal_quality1_pct_out then
        return "No data returned from rangefinder:distance_orient()"
    end

    send(string.format("Frontend test %i dist in_m: %.2f out_cm: %.2f, signal_quality_pct in: %.1f out: %.1f",
        test_idx, dist_m_in, distance1_cm_out, signal_quality_pct_in, signal_quality1_pct_out))

    if not test_dist_equal(dist_m_in, 100.0, distance1_cm_out, signal_quality_pct_in, signal_quality1_pct_out) then
        return "Frontend expected and actual do not match"
    end

    -- L U A   I N T E R F A C E   T E S T
    -- Check that the distance and signal_quality from the backend are as expected
    local disttance2_m_out = rngfnd_backend:distance()
    local signal_quality2_pct_out = rngfnd_backend:signal_quality()

    send(string.format("Backend test %i dist in_m: %.2f out_m: %.2f, signal_quality_pct in: %.1f out: %.1f",
        test_idx, dist_m_in, disttance2_m_out, signal_quality_pct_in, signal_quality2_pct_out))

    if not test_dist_equal(dist_m_in, 1.0, disttance2_m_out, signal_quality_pct_in, signal_quality2_pct_out) then
        return "Backend expected and actual do not match"
    end

    -- L U A   I N T E R F A C E   T E S T
    -- Check that the state from the backend is as expected
    local rf_state = rngfnd_backend:get_state()
    local distance3_m_out = rf_state:distance()
    local signal_quality3_pct_out = rf_state:signal_quality()

    send(string.format("State test %i dist in_m: %.2f out_m: %.2f, signal_quality_pct in: %.1f out: %.1f",
        test_idx, dist_m_in, distance3_m_out, signal_quality_pct_in, signal_quality3_pct_out))

    if not test_dist_equal(dist_m_in, 1.0, distance3_m_out, signal_quality_pct_in, signal_quality3_pct_out) then
        return "State expected and actual do not match"
    end

    return nil
end

-- Test various status states
local function do_status_tests()
    send("Test initial status")
    local status_actual = rangefinder:status_orient(RNGFND_ORIENTATION_DOWN)
    if status_actual ~= RNDFND_STATUS_NOT_CONNECTED then
        return string.format("DOWN Status '%i' not NOT_CONNECTED on initialization.", status_actual)
    end
    status_actual = rangefinder:status_orient(RNGFND_ORIENTATION_FORWARD)
    if status_actual ~= RNDFND_STATUS_NOT_CONNECTED then
        return string.format("FORWARD Status '%i' not NOT_CONNECTED on initialization.", status_actual)
    end
    return nil
end


local test_data = {
    {20.0, -1, RNDFND_STATUS_GOOD},
    {20.5, -2, RNDFND_STATUS_GOOD},
    {21.0, 0, RNDFND_STATUS_GOOD},
    {22.0, 50, RNDFND_STATUS_GOOD},
    {23.0, 100, RNDFND_STATUS_GOOD},
    {24.0, 101, RNDFND_STATUS_GOOD},
    {25.0, -3, RNDFND_STATUS_GOOD},
    {26.0, 127, RNDFND_STATUS_GOOD},
    {27.0, 3, RNDFND_STATUS_GOOD},
    {28.0, 100, RNDFND_STATUS_GOOD},
    {29.0, 99, RNDFND_STATUS_GOOD},
    {100.0, 100, RNDFND_STATUS_OUT_OF_RANGE_HIGH},
    {0.0, 100, RNDFND_STATUS_OUT_OF_RANGE_LOW},
    {100.0, -2, RNDFND_STATUS_OUT_OF_RANGE_HIGH},
    {0.0, -2, RNDFND_STATUS_OUT_OF_RANGE_LOW},
}

-- Record the start time so we can timeout if initialization takes too long.
local time_start_ms = millis():tofloat()
local test_idx = 0


-- Called when tests are completed.
local function complete(error_str)
    -- Send a message indicating the success or failure of the test
    local status_str
    if not error_str or #error_str == 0 then
        status_str = SUCCESS_STR
    else
        send(error_str)
        status_str = FAILURE_STR
    end
    send(string.format("%s: %s", COMPLETE_STR, status_str))

    -- Returning nil will not queue an update routine so the test will stop running.
end


-- A state machine of update functions. The states progress:
--  prepare, wait, begin_test, eval_test, begin_test, eval_test, ... complete

local update_prepare
local update_wait
local update_begin_test
local update_eval_test

local function _update_prepare()
    if Parameter('RNGFND1_TYPE'):get() ~= RNGFND_TYPE_LUA then
        return complete("LUA range finder driver not enabled")
    end
    if rangefinder:num_sensors() < 1 then
        return complete("LUA range finder driver not connected")
    end
    rngfnd_backend = rangefinder:get_backend(0)
    if not rngfnd_backend then
        return complete("Range Finder 1 does not exist")
    end
    if (rngfnd_backend:type() ~= RNGFND_TYPE_LUA) then
        return complete("Range Finder 1 is not a LUA driver")
    end

    return update_wait()
end

local function _update_wait()
    -- Check for timeout while initializing
    if millis():tofloat() - time_start_ms > TIMEOUT_MS then
        return complete("Timeout while trying to initialize")
    end

    -- Wait until the prearm check passes. This ensures that the system is mostly initialized
    -- before starting the tests.
    if not arming:pre_arm_checks() then
        return update_wait, UPDATE_PERIOD_MS
    end

    -- Do some one time tests
    local error_str = do_status_tests()
    if error_str then
        return complete(error_str)
    end

    -- Continue on to the main list of tests.
    return update_begin_test()
end

local function _update_begin_test()
    test_idx = test_idx + 1
    if test_idx > #test_data then
        return complete()
    end

    local dist_m_in = test_data[test_idx][1]
    local signal_quality_pct_in = test_data[test_idx][2]

    -- L U A   I N T E R F A C E   T E S T
    -- Use the driver interface to simulate a data measurement being received and being passed to AP.
    local result
    -- -2 => use legacy interface
    if signal_quality_pct_in == -2 then
        result = rngfnd_backend:handle_script_msg(dist_m_in) -- number as arg (compatibility mode)

    else
        -- The full state udata must be initialized.
        local rf_state = RangeFinder_State()
        -- Set the status
        if dist_m_in < RNGFND1_MIN then
            rf_state:status(RNDFND_STATUS_OUT_OF_RANGE_LOW)
        elseif dist_m_in > RNGFND1_MAX then
            rf_state:status(RNDFND_STATUS_OUT_OF_RANGE_HIGH)
        else
            rf_state:status(RNDFND_STATUS_GOOD)
        end
        -- Sanitize signal_quality_pct_in
        if signal_quality_pct_in < SIGNAL_QUALITY_MIN or signal_quality_pct_in > SIGNAL_QUALITY_MAX then
            signal_quality_pct_in = SIGNAL_QUALITY_UNKNOWN
        end
        rf_state:last_reading(millis():toint())
        rf_state:range_valid_count(10)
        rf_state:distance(dist_m_in)
        rf_state:signal_quality(signal_quality_pct_in)
        rf_state:voltage(0)
        result = rngfnd_backend:handle_script_msg(rf_state) -- state as arg
    end

    if not result then
        return complete(string.format("Test %i, dist_m: %.2f, quality_pct: %3i failed to handle_script_msg2",
            test_idx, dist_m_in, signal_quality_pct_in))
    end

    return update_eval_test, UPDATE_PERIOD_MS
end

local function _update_eval_test()
    local dist_m_in = test_data[test_idx][1]
    local signal_quality_pct_in = test_data[test_idx][2]
    local status_expected = test_data[test_idx][3]

    -- Use the client interface to get distance data and ensure it matches the distance data
    -- that was sent through the driver interface.
    local error_str = get_and_eval(test_idx, dist_m_in, signal_quality_pct_in, status_expected)
    if error_str then
        return complete(string.format("Test %i, dist_m: %.2f, quality_pct: %3i failed because %s",
            test_idx, dist_m_in, signal_quality_pct_in, error_str))
    end

    -- Move to the next test in the list.
    return update_begin_test()
end

update_prepare = _update_prepare
update_wait = _update_wait
update_begin_test = _update_begin_test
update_eval_test = _update_eval_test

send("Loaded rangefinder_quality_test.lua")

return update_prepare, 0
