--[[
-- A script to perform ArduPilot calibrations using a custom CRSF menu.
-- This script uses the enhanced crsf_helper.lua library (v6.4+)
--]]

-- Load the CRSF helper library and its shared constants
local crsf_helper = require('crsf_helper')
local MAV_SEVERITY = crsf_helper.MAV_SEVERITY
local CRSF_COMMAND_STATUS = crsf_helper.CRSF_COMMAND_STATUS

-- MAVLink command constants
local MAV_CMD_PREFLIGHT_CALIBRATION = 241
local MAV_CMD_DO_START_MAG_CAL = 42424
local MAV_CMD_DO_CANCEL_MAG_CAL = 42426

-- gcs:run_command_int results
local MAV_RESULT_ACCEPTED = 0
local MAV_RESULT_TEMPORARILY_REJECTED = 1  -- e.g. within 5s of the last calibration
local MAV_RESULT_IN_PROGRESS = 5           -- gcs:command_int_result() gives the result later

-- a calibration's handler allows itself 30s (EXPECT_DELAY_MS in
-- _handle_command_preflight_calibration); stop waiting for its result 10s
-- after that, as another script may have taken it
local CALIBRATION_RESULT_TIMEOUT_MS = 40000

-- true from Execute on Calibrate Compass until its result has been shown
local compass_cal_running = false

-- compasses one calibration can cover (COMPASS_MAX_INSTANCES)
local MAX_COMPASSES = 3

-- compasses seen calibrating since Execute; only they count towards the result
local compass_cal_seen = {}

-- CompassCalibrator::Status values compass:get_cal_state() returns
local MAG_CAL_NOT_STARTED = 0
local MAG_CAL_SUCCESS = 4

-- why a compass seen calibrating has ended in a status other than success
local MAG_CAL_FAILURES = {
    [0] = "stopped", [5] = "failed", [6] = "bad orientation", [7] = "bad radius",
    [8] = "bad offsets", [9] = "bad scaling", [10] = "poor fit",
}

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

--- Returns how the compass calibration started from this menu is going:
--- "running" and the lowest percent complete of the compasses still
--- running, "failed" and which compass failed and why, "saved", or nil if
--- no compass has been seen calibrating since Execute.
local function compass_cal_progress()
    local lowest_pct = nil
    local failure = nil
    local saved = false
    for i = 0, MAX_COMPASSES - 1 do
        local status, pct = compass:get_cal_state(i)
        if status ~= nil and status ~= MAG_CAL_NOT_STARTED and status < MAG_CAL_SUCCESS then
            compass_cal_seen[i] = true
            lowest_pct = math.min(lowest_pct or 100, pct)
        elseif compass_cal_seen[i] then
            -- a compass this calibration skipped keeps the state of an earlier one, so is not counted
            if status == MAG_CAL_SUCCESS then
                saved = true
            else
                failure = failure or string.format("Mag(%d): %s", i, MAG_CAL_FAILURES[status] or "failed")
            end
        end
    end
    if lowest_pct then
        return "running", lowest_pct
    elseif failure then
        return "failed", failure
    elseif saved then
        return "saved"
    end
    return nil
end

--- Handles the compass calibration. The transmitter shows its progress,
--- polling while it runs, then whether it saved (it autosaves) or failed.
--- Backing out of the progress popup cancels it.
local function callback_compass_cal(command_action)
    if command_action == CRSF_COMMAND_STATUS.START then
        -- User pressed "Execute". A calibration this menu started that is
        -- still running carries on, as starting again would restart any
        -- compass that has finished
        if compass_cal_running then
            local state, pct = compass_cal_progress()
            if state == "running" then
                return CRSF_COMMAND_STATUS.PROGRESS, string.format("Calibrating %.0f%%", pct)
            end
        end
        -- if a calibration cannot start, the handler says why
        if gcs:run_command_int(MAV_CMD_DO_START_MAG_CAL, { p3 = 1 }) ~= MAV_RESULT_ACCEPTED then
            return CRSF_COMMAND_STATUS.READY, "Failed"
        end
        compass_cal_running = true
        compass_cal_seen = {}
        gcs:send_text(MAV_SEVERITY.INFO, "Compass cal running...")
        return CRSF_COMMAND_STATUS.PROGRESS, "Calibrating 0%"

    elseif command_action == CRSF_COMMAND_STATUS.POLL then
        if not compass_cal_running then
            -- keep showing the result until it is acknowledged
            return nil
        end
        local state, detail = compass_cal_progress()
        if state == "running" then
            return CRSF_COMMAND_STATUS.PROGRESS, string.format("Calibrating %.0f%%", detail)
        end
        compass_cal_running = false
        if state == "saved" then
            gcs:send_text(MAV_SEVERITY.INFO, "Compass calibration saved, reboot to fly")
            return CRSF_COMMAND_STATUS.CONFIRMATION_NEEDED, "Saved, reboot to fly"
        end
        detail = detail or "Stopped"
        gcs:send_text(MAV_SEVERITY.WARNING, "Compass cal failed, " .. detail)
        return CRSF_COMMAND_STATUS.CONFIRMATION_NEEDED, detail

    elseif command_action == CRSF_COMMAND_STATUS.CONFIRM then
        -- User acknowledged the result
        return CRSF_COMMAND_STATUS.READY, "Execute"

    elseif command_action == CRSF_COMMAND_STATUS.CANCEL then
        -- User backed out of the progress popup; a calibration that has
        -- just finished keeps its result
        if compass_cal_running and compass_cal_progress() == "running" then
            gcs:run_command_int(MAV_CMD_DO_CANCEL_MAG_CAL, { p3 = 1 })
            gcs:send_text(MAV_SEVERITY.WARNING, "Compass calibration cancelled")
        end
        compass_cal_running = false
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