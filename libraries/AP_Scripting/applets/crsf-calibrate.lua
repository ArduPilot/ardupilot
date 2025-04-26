--[[
-- A script to perform ArduPilot calibrations using a custom CRSF menu
-- copy this script to the autopilot's "scripts" directory
--]]

SCRIPT_NAME = "ArduPilot Calibration"
SCRIPT_NAME_SHORT = "Calibration"
SCRIPT_VERSION = "0.1"

MAV_SEVERITY = {EMERGENCY=0, ALERT=1, CRITICAL=2, ERROR=3, WARNING=4, NOTICE=5, INFO=6, DEBUG=7}
CRSF_EVENT = {PARAMETER_READ=1, PARAMETER_WRITE=2}
CRSF_PARAM_TYPE = {
    UINT8 = 0,  -- deprecated
    INT8 = 1,   -- deprecated
    UINT16 = 2, -- deprecated
    INT16 = 3,  -- deprecated
    FLOAT = 8,
    TEXT_SELECTION = 9,
    STRING = 10,
    FOLDER = 11,
    INFO = 12,
    COMMAND = 13,
}

CRSF_COMMAND_STATUS = {
    READY = 0, --               --> feedback
    START = 1, --               <-- input
    PROGRESS = 2, --            --> feedback
    CONFIRMATION_NEEDED = 3, -- --> feedback
    CONFIRM = 4, --             <-- input
    CANCEL = 5, --              <-- input
    POLL = 6 --                 <-- input
}

MAV_CMD_PREFLIGHT_CALIBRATION = 241
MAV_CMD_DO_START_MAG_CAL = 42424
MAV_CMD_DO_ACCEPT_MAG_CAL = 42425
MAV_CMD_DO_CANCEL_MAG_CAL = 42426

-- create a CRSF menu float item
function create_float_entry(name, value, min, max, default, dpoint, step, unit)
    return string.pack(">BzllllBlz", CRSF_PARAM_TYPE.FLOAT, name, value, min, max, default, dpoint, step, unit)
end

-- create a CRSF menu text selection item
function create_text_entry(name, options, value, min, max, default, unit)
    return string.pack(">BzzBBBBz", CRSF_PARAM_TYPE.TEXT_SELECTION, name, options, value, min, max, default, unit)
end

-- create a CRSF menu string item
function create_string_entry(name, value, max)
    return string.pack(">BzzB", CRSF_PARAM_TYPE.STRING, name, value, max)
end

-- create a CRSF menu info item
function create_info_entry(name, info)
    return string.pack(">Bzz", CRSF_PARAM_TYPE.INFO, name, info)
end

-- create a CRSF command entry
function create_command_entry(name, status, timeout, info)
    timeout = timeout or 10 -- 1s
    return string.pack(">BzBBz", CRSF_PARAM_TYPE.COMMAND, name, status, timeout, info)
end

local compass_command, accel_command, gyro_command, forceaccel_command, forcecompass_command, ahrs_command

local menu = crsf:add_menu('Calibrate')
local compass_param = create_command_entry("Calibrate Compass", CRSF_COMMAND_STATUS.READY, 50, "Start Calibration")
local accel_param = create_command_entry("Calibrate Accels", CRSF_COMMAND_STATUS.READY, 50, "Calibrate Accels")
local gyro_param = create_command_entry("Calibrate Gyros", CRSF_COMMAND_STATUS.READY, 50, "Calibrate Gyros")
local forceaccel_param = create_command_entry("Forcecal Accels", CRSF_COMMAND_STATUS.READY, 50, "Forcecal Accels")
local forcecompass_param = create_command_entry("Forcecal Compass", CRSF_COMMAND_STATUS.READY, 50, "Forcecal Compass")
local ahrs_param = create_command_entry("Trim AHRS", CRSF_COMMAND_STATUS.READY, 50, "Trim AHRS")

if menu ~= nil then
    compass_command = menu:add_parameter(compass_param)
    accel_command = menu:add_parameter(accel_param)
    gyro_command = menu:add_parameter(gyro_param)
    forceaccel_command = menu:add_parameter(forceaccel_param)
    forcecompass_command = menu:add_parameter(forcecompass_param)
    ahrs_command = menu:add_parameter(ahrs_param)
    gcs:send_text(MAV_SEVERITY.INFO, string.format("Loaded CRSF calibration menu"))
end

local calibration_running = false

function update()
    local param, payload, events = crsf:get_menu_event(CRSF_EVENT.PARAMETER_WRITE)
    if (events & CRSF_EVENT.PARAMETER_WRITE) ~= 0 then
        if compass_command ~= nil and param == compass_command:id() then
            local command_action = string.unpack(">B", payload)
            if command_action == CRSF_COMMAND_STATUS.START then -- start calibration
                -- mag cal
                if calibration_running then
                    gcs:run_command_int(MAV_CMD_DO_CANCEL_MAG_CAL, { p3 = 1 })
                end
                calibration_running = true
                gcs:run_command_int(MAV_CMD_DO_START_MAG_CAL, { p3 = 1 })
                gcs:send_text(MAV_SEVERITY.INFO, "Compass calibration running")
                crsf:send_write_response(create_command_entry("Calibrate Compass", CRSF_COMMAND_STATUS.CONFIRMATION_NEEDED, 50, "Accept Calibration"))
            elseif command_action == CRSF_COMMAND_STATUS.CONFIRM then -- confirm acceptance
                gcs:run_command_int(MAV_CMD_DO_ACCEPT_MAG_CAL, { p3 = 1 })
                gcs:send_text(MAV_SEVERITY.INFO, "Compass calibration accepted")
                crsf:send_write_response(compass_param)
            elseif command_action == CRSF_COMMAND_STATUS.CANCEL and calibration_running then
                gcs:run_command_int(MAV_CMD_DO_CANCEL_MAG_CAL, { p3 = 1 })
                gcs:send_text(MAV_SEVERITY.WARNING, "Calibration cancelled")
                calibration_running = false
                crsf:send_write_response(compass_param)
            end
        elseif accel_command ~= nil and param == accel_command:id() then
            local command_action = string.unpack(">B", payload)
            if command_action == CRSF_COMMAND_STATUS.START then
                -- accelcalsimple
                gcs:run_command_int(MAV_CMD_PREFLIGHT_CALIBRATION, { p5 = 4 })
                gcs:send_text(MAV_SEVERITY.INFO, "Accels calibrated")
                crsf:send_write_response(accel_param)
            end
        elseif gyro_command ~= nil and param == gyro_command:id() then
            local command_action = string.unpack(">B", payload)
            if command_action == CRSF_COMMAND_STATUS.START then
                -- gyro cal
                gcs:run_command_int(MAV_CMD_PREFLIGHT_CALIBRATION, { p1 = 1 })
                gcs:send_text(MAV_SEVERITY.INFO, "Gyros calibrated")
                crsf:send_write_response(gyro_param)
            end
        elseif forceaccel_command ~= nil and param == forceaccel_command:id() then
            local command_action = string.unpack(">B", payload)
            if command_action == CRSF_COMMAND_STATUS.START then
                -- forcecal accel
                gcs:run_command_int(MAV_CMD_PREFLIGHT_CALIBRATION, { p5 = 76 })
                gcs:send_text(MAV_SEVERITY.INFO, "Accels force calibrated")
                crsf:send_write_response(forceaccel_param)
            end
        elseif forcecompass_command ~= nil and param == forcecompass_command:id() then
            local command_action = string.unpack(">B", payload)
            if command_action == CRSF_COMMAND_STATUS.START then
                -- forcecal compass
                gcs:run_command_int(MAV_CMD_PREFLIGHT_CALIBRATION, { p2 = 76 })
                gcs:send_text(MAV_SEVERITY.INFO, "Compass force calibrated")
                crsf:send_write_response(forcecompass_param)
            end
        elseif ahrs_command ~= nil and param == ahrs_command:id() then
            local command_action = string.unpack(">B", payload)
            if command_action == CRSF_COMMAND_STATUS.START then
                -- ahrs trim
                gcs:run_command_int(MAV_CMD_PREFLIGHT_CALIBRATION, { p5 = 2 })
                gcs:send_text(MAV_SEVERITY.INFO, "AHRS trimmed")
                crsf:send_write_response(ahrs_param)
            end
        end
    elseif (events & CRSF_EVENT.PARAMETER_READ) ~= 0 then
        gcs:send_text(MAV_SEVERITY.INFO, "Parameter read " .. param)
    end
    return update, 100
end

return update, 5000