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
local MAV_RESULT_IN_PROGRESS = 5           -- gcs:command_int_result() gives the result later

-- a calibration's handler allows itself 30s (EXPECT_DELAY_MS in
-- _handle_command_preflight_calibration); stop waiting for its result 10s
-- after that, as another script may have taken it
local CALIBRATION_RESULT_TIMEOUT_MS = 40000

-- This script-local variable tracks the state of the multi-step compass calibration
local compass_cal_running = false

-- ####################
-- # CALLBACK FUNCTIONS
-- ####################

-- These functions are called by the helper's event loop.
-- They must return the new status and info text for the command item.

--- Tells the GCS how a one-shot calibration went. Returns whether it
--- succeeded and the text sent.
local function report_calibration(result, done_text, name)
    if result == MAV_RESULT_ACCEPTED then
        gcs:send_text(MAV_SEVERITY.INFO, done_text)
        return true, done_text
    end
    local text = name .. " failed"
    if result == MAV_RESULT_TEMPORARILY_REJECTED then
        text = name .. " busy, try again"
    end
    gcs:send_text(MAV_SEVERITY.WARNING, text)
    return false, text
end

--- Returns the callback for a one-shot calibration command item. Firmware
--- that runs the calibration on the main thread returns IN_PROGRESS, so the
--- item shows progress until gcs:command_int_result() has the result, then
--- asks the pilot to acknowledge it, as a plain READY closes the
--- transmitter's popup before its text can be read.
local function calibration_callback(args, done_text, name)
    -- when the calibration was handed over, while its result is awaited
    local waiting_since_ms = nil
    return function(command_action)
        if command_action == CRSF_COMMAND_STATUS.START then
            local result = gcs:run_command_int(MAV_CMD_PREFLIGHT_CALIBRATION, args)
            if result ~= MAV_RESULT_IN_PROGRESS then
                local succeeded = report_calibration(result, done_text, name)
                return CRSF_COMMAND_STATUS.READY, succeeded and "Execute" or "Failed"
            end
            waiting_since_ms = millis():tofloat()
            return CRSF_COMMAND_STATUS.PROGRESS, "Calibrating"
        elseif command_action == CRSF_COMMAND_STATUS.POLL then
            if waiting_since_ms == nil then
                -- keep showing the result until it is acknowledged
                return nil
            end
            local result = gcs:command_int_result(MAV_CMD_PREFLIGHT_CALIBRATION)
            if result == nil then
                if millis():tofloat() - waiting_since_ms < CALIBRATION_RESULT_TIMEOUT_MS then
                    return nil
                end
                waiting_since_ms = nil
                gcs:send_text(MAV_SEVERITY.WARNING, name .. " gave no result")
                return CRSF_COMMAND_STATUS.CONFIRMATION_NEEDED, name .. ": no result"
            end
            waiting_since_ms = nil
            local _, text = report_calibration(result, done_text, name)
            return CRSF_COMMAND_STATUS.CONFIRMATION_NEEDED, text
        end
        -- OK on the result, or backing out, which cannot stop a calibration
        waiting_since_ms = nil
        return CRSF_COMMAND_STATUS.READY, "Execute"
    end
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
            -- param5 travels in x: run_command_int has no p5
            callback = calibration_callback({ x = 4 }, "Accels calibrated", "Accel cal")
        },
        {
            type = 'COMMAND',
            name = 'Calibrate Gyros',
            callback = calibration_callback({ p1 = 1 }, "Gyros calibrated", "Gyro cal")
        },
        {
            type = 'COMMAND',
            name = 'Forcecal Accels',
            callback = calibration_callback({ x = 76 }, "Accels force calibrated", "Accel forcecal")
        },
        {
            type = 'COMMAND',
            name = 'Forcecal Compass',
            callback = calibration_callback({ p2 = 76 }, "Compass force calibrated", "Compass forcecal")
        },
        {
            type = 'COMMAND',
            name = 'Trim AHRS',
            callback = calibration_callback({ x = 2 }, "AHRS trimmed", "AHRS trim")
        }
    }
}

-- ####################
-- # SCRIPT START
-- ####################

-- Register the menu definition with the helper.
-- This builds the menu and starts the helper's persistent event loop.
return crsf_helper.register_menu(menu_definition)