--[[
-- A script to perform ArduPilot calibrations using a custom CRSF menu.
-- This script uses the enhanced crsf_helper.lua library (v6.1+)
--]]

-- Load the CRSF helper library and its shared constants
local crsf_helper = require('crsf_helper')
local MAV_SEVERITY = crsf_helper.MAV_SEVERITY
local CRSF_COMMAND_STATUS = crsf_helper.CRSF_COMMAND_STATUS

-- MAVLink command constants
local MAV_CMD_PREFLIGHT_CALIBRATION = 241
local MAV_CMD_DO_START_MAG_CAL = 42424
local MAV_CMD_DO_ACCEPT_MAG_CAL = 42425
local MAV_CMD_DO_CANCEL_MAG_CAL = 42426

-- gcs:run_command_int results
local MAV_RESULT_ACCEPTED = 0
local MAV_RESULT_TEMPORARILY_REJECTED = 1  -- e.g. within 5s of the last calibration

-- This script-local variable tracks the state of the multi-step compass calibration
local compass_cal_running = false

-- ####################
-- # CALLBACK FUNCTIONS
-- ####################

-- These functions are called by the helper's event loop.
-- They must return the new status and info text for the command item.

--- Runs a one-shot calibration and reports how it went. Returns the command
--- item's new state: "Execute" if the calibration succeeded, "Failed" if not.
local function run_calibration(args, done_text, name)
    local result = gcs:run_command_int(MAV_CMD_PREFLIGHT_CALIBRATION, args)
    if result == MAV_RESULT_ACCEPTED then
        gcs:send_text(MAV_SEVERITY.INFO, done_text)
        return CRSF_COMMAND_STATUS.READY, "Execute"
    end
    if result == MAV_RESULT_TEMPORARILY_REJECTED then
        gcs:send_text(MAV_SEVERITY.WARNING, name .. " busy, try again")
    else
        gcs:send_text(MAV_SEVERITY.WARNING, name .. " failed")
    end
    return CRSF_COMMAND_STATUS.READY, "Failed"
end

--- Handles the multi-step compass calibration logic.
local function callback_compass_cal(command_action)
    if command_action == CRSF_COMMAND_STATUS.START then
        -- User pressed "Execute"
        if compass_cal_running then
            -- Was already running, cancel it first just in case
            gcs:run_command_int(MAV_CMD_DO_CANCEL_MAG_CAL, { p3 = 1 })
            compass_cal_running = false
        end

        -- Start the new calibration; if it cannot, the handler says why
        if gcs:run_command_int(MAV_CMD_DO_START_MAG_CAL, { p3 = 1 }) ~= MAV_RESULT_ACCEPTED then
            return CRSF_COMMAND_STATUS.READY, "Failed"
        end
        compass_cal_running = true
        gcs:send_text(MAV_SEVERITY.INFO, "Compass cal running...")

        -- Tell the helper to update the TX UI to "Accept"
        return CRSF_COMMAND_STATUS.CONFIRMATION_NEEDED, "Accept"

    elseif command_action == CRSF_COMMAND_STATUS.CONFIRM then
        -- User pressed "Accept"
        if compass_cal_running then
            -- accepting stops every calibrator, whether or not it had succeeded
            compass_cal_running = false
            if gcs:run_command_int(MAV_CMD_DO_ACCEPT_MAG_CAL, { p3 = 1 }) ~= MAV_RESULT_ACCEPTED then
                gcs:send_text(MAV_SEVERITY.WARNING, "Compass cal stopped: not finished or failed")
                return CRSF_COMMAND_STATUS.READY, "Failed"
            end
            gcs:send_text(MAV_SEVERITY.INFO, "Compass calibration accepted")
        end

        -- Tell the helper to update the TX UI back to "Execute"
        return CRSF_COMMAND_STATUS.READY, "Execute"

    elseif command_action == CRSF_COMMAND_STATUS.CANCEL then
        -- User pressed "Cancel" (e.g., long-press back button on TX)
        if compass_cal_running then
            gcs:run_command_int(MAV_CMD_DO_CANCEL_MAG_CAL, { p3 = 1 })
            gcs:send_text(MAV_SEVERITY.WARNING, "Calibration cancelled")
            compass_cal_running = false
        end

        -- Tell the helper to update the TX UI back to "Execute"
        return CRSF_COMMAND_STATUS.READY, "Execute"
    end

    -- Default fallback
    return CRSF_COMMAND_STATUS.READY, "Execute"
end

--- Handles the simple, one-shot gyro calibration.
local function callback_gyro_cal(command_action)
    if command_action == CRSF_COMMAND_STATUS.START then
        return run_calibration({ p1 = 1 }, "Gyros calibrated", "Gyro cal")
    end
    -- Always return to READY state
    return CRSF_COMMAND_STATUS.READY, "Execute"
end

--- Handles the simple, one-shot accel calibration.
local function callback_accel_cal(command_action)
    if command_action == CRSF_COMMAND_STATUS.START then
        -- param5 travels in x: run_command_int has no p5
        return run_calibration({ x = 4 }, "Accels calibrated", "Accel cal")
    end
    return CRSF_COMMAND_STATUS.READY, "Execute"
end

--- Handles the simple, one-shot "force" accel calibration.
local function callback_force_accel_cal(command_action)
    if command_action == CRSF_COMMAND_STATUS.START then
        return run_calibration({ x = 76 }, "Accels force calibrated", "Accel forcecal")
    end
    return CRSF_COMMAND_STATUS.READY, "Execute"
end

--- Handles the simple, one-shot "force" compass calibration.
local function callback_force_compass_cal(command_action)
    if command_action == CRSF_COMMAND_STATUS.START then
        return run_calibration({ p2 = 76 }, "Compass force calibrated", "Compass forcecal")
    end
    return CRSF_COMMAND_STATUS.READY, "Execute"
end

--- Handles the simple, one-shot AHRS trim.
local function callback_ahrs_trim(command_action)
    if command_action == CRSF_COMMAND_STATUS.START then
        return run_calibration({ x = 2 }, "AHRS trimmed", "AHRS trim")
    end
    return CRSF_COMMAND_STATUS.READY, "Execute"
end

-- ####################
-- # MENU DEFINITION
-- ####################

-- This table defines the entire menu structure.
-- The helper library will parse this and build the menu.
local menu_definition = {
    name = 'Calibrate',
    items = {
        {
            type = 'COMMAND',
            name = 'Calibrate Compass',
            callback = callback_compass_cal
            -- info = "Execute" -- This is the default
        },
        {
            type = 'COMMAND',
            name = 'Calibrate Accels',
            callback = callback_accel_cal
        },
        {
            type = 'COMMAND',
            name = 'Calibrate Gyros',
            callback = callback_gyro_cal
        },
        {
            type = 'COMMAND',
            name = 'Forcecal Accels',
            callback = callback_force_accel_cal
        },
        {
            type = 'COMMAND',
            name = 'Forcecal Compass',
            callback = callback_force_compass_cal
        },
        {
            type = 'COMMAND',
            name = 'Trim AHRS',
            callback = callback_ahrs_trim
        }
    }
}

-- ####################
-- # SCRIPT START
-- ####################

-- Register the menu definition with the helper.
-- This builds the menu and starts the helper's persistent event loop.
return crsf_helper.register_menu(menu_definition)