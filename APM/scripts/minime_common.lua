--[[
   MiniMe Common Module
   Shared constants and conversion functions for AS-10 MiniMe ArduPilot scripts

   This module is required by all other MiniMe scripts:
   local common = require("minime_common")
]]--

local M = {}

-- Vehicle constants
M.HOVER_RPM = 1131.7
M.MOTOR_RPM = 4526.8
M.ERPM_HOVER = 45268
M.GEARBOX_RATIO = 4
M.POLE_PAIRS = 10

-- CAN constants
M.DTI_FWD_NODE = 0x01
M.DTI_AFT_NODE = 0x02
M.DTI_BROADCAST = 0xFF
M.CAN_BITRATE = 500000

-- GPIO constants (AUX pin numbers)
M.GPIO_FWD_MAIN = 50
M.GPIO_FWD_PRECHARGE = 51
M.GPIO_AFT_MAIN = 52
M.GPIO_AFT_PRECHARGE = 53
M.GPIO_FWD_STO = 54
M.GPIO_AFT_STO = 55

-- HV state machine states
M.STATE_DE_ENERGIZED = 0
M.STATE_PRECHARGING = 1
M.STATE_ENERGIZED = 2
M.STATE_ARMED = 3
M.STATE_FAULTED = 4

-- Timing constants (milliseconds)
M.DTI_COMMAND_RATE_MS = 10
M.DTI_TELEMETRY_RATE_MS = 20
M.DTI_TIMEOUT_MS = 500
M.PRECHARGE_MIN_TIME_MS = 1000
M.PRECHARGE_TIMEOUT_MS = 10000
M.PRECHARGE_THRESHOLD = 0.90

-- Desync alert states
M.DESYNC_NORMAL = 0
M.DESYNC_ALERT = 1
M.DESYNC_WARNING = 2
M.DESYNC_CRITICAL = 3

-- Desync thresholds (percentage of hover RPM)
M.DESYNC_ALERT_PCT = 0.25
M.DESYNC_WARNING_PCT = 0.50
M.DESYNC_HARD_PCT = 1.00

-- Desync thresholds (absolute RPM)
M.DESYNC_ALERT_RPM = 2.8
M.DESYNC_WARNING_RPM = 5.7
M.DESYNC_HARD_RPM = 11.3

-- Spin up suppression time after arming (milliseconds)
M.SPINUP_SUPPRESS_MS = 5000

-- Redline thresholds: motor winding temperature (Celsius)
M.REDLINE_MOTOR_WINDING = {
    caution = 80,
    warning = 95,
    hard = 100
}

-- Redline thresholds: motor part temperature (Celsius)
M.REDLINE_MOTOR_PART = {
    caution = 95,
    warning = 110,
    hard = 120
}

-- Redline thresholds: coolant inlet motor loop (Celsius)
M.REDLINE_COOLANT_MOTOR = {
    caution = 40,
    warning = 45,
    hard = 50
}

-- Redline thresholds: coolant inlet inverter loop (Celsius)
M.REDLINE_COOLANT_INVERTER = {
    caution = 45,
    warning = 55,
    hard = 60
}

-- Redline thresholds: phase current RMS (Amps)
M.REDLINE_PHASE_CURRENT = {
    caution = 200,
    warning = 300,
    hard = 380
}

-- Redline thresholds: inverter DC link voltage low (Volts)
M.REDLINE_DC_LINK_LOW = {
    caution = 100,
    warning = 50,
    hard = 30
}

-- Redline thresholds: inverter DC link voltage high (Volts)
M.REDLINE_DC_LINK_HIGH = {
    caution = 700,
    warning = 800,
    hard = 830
}

-- Redline thresholds: battery DC bus voltage low (Volts)
M.REDLINE_BATTERY_LOW = {
    caution = 130,
    warning = 128,
    hard = 120
}

-- Redline thresholds: battery DC bus voltage high (Volts)
M.REDLINE_BATTERY_HIGH = {
    caution = 154,
    warning = 156,
    hard = 165
}

-- Redline thresholds: motor RPM (mechanical)
M.REDLINE_MOTOR_RPM = {
    caution = 6000,
    warning = 6300,
    hard = 6500
}

-- Redline thresholds: rotor RPM
M.REDLINE_ROTOR_RPM = {
    caution = 1004,
    warning = 1050,
    hard = 1100
}

-- Redline thresholds: battery pack temperature (Celsius)
M.REDLINE_PACK_TEMP = {
    caution = 50,
    warning = 60,
    hard = 70
}

-- Redline thresholds: state of charge (percent)
M.REDLINE_SOC = {
    caution = 30,
    warning = 25,
    hard = 20
}

-- DTI fault codes
M.DTI_FAULT = {
    [0x00] = "No fault",
    [0x01] = "Over Voltage",
    [0x02] = "Under Voltage",
    [0x03] = "DRV Error",
    [0x04] = "ABS Over Current",
    [0x05] = "Controller Over Temperature",
    [0x06] = "Motor Over Temperature",
    [0x07] = "Sensor Wire Fault",
    [0x08] = "Sensor General Fault"
}

-- MAVLink severity levels
M.MAV_SEVERITY = {
    EMERGENCY = 0,
    ALERT = 1,
    CRITICAL = 2,
    ERROR = 3,
    WARNING = 4,
    NOTICE = 5,
    INFO = 6,
    DEBUG = 7
}

--[[
   Convert electrical RPM to rotor RPM
   @param erpm Electrical RPM from DTI controller
   @return Rotor RPM
]]--
function M.rpm_from_erpm(erpm)
    return erpm / M.POLE_PAIRS / M.GEARBOX_RATIO
end

--[[
   Convert rotor RPM to electrical RPM
   @param rotor_rpm Rotor RPM
   @return Electrical RPM for DTI command (integer)
]]--
function M.erpm_from_rpm(rotor_rpm)
    return math.floor(rotor_rpm * M.GEARBOX_RATIO * M.POLE_PAIRS + 0.5)
end

--[[
   Scale raw temperature value from DTI telemetry
   @param raw Raw temperature value (scaled by 10)
   @return Temperature in Celsius
]]--
function M.scale_temperature(raw)
    return raw / 10.0
end

--[[
   Scale raw current value from DTI telemetry
   @param raw Raw current value (scaled by 10)
   @return Current in Amps
]]--
function M.scale_current(raw)
    return raw / 10.0
end

--[[
   Scale raw voltage value from DTI telemetry
   @param raw Raw voltage value
   @return Voltage in Volts (unchanged)
]]--
function M.scale_voltage(raw)
    return raw
end

--[[
   Get fault name from fault code
   @param code DTI fault code (0x00 to 0x08)
   @return Fault name string
]]--
function M.fault_name(code)
    return M.DTI_FAULT[code] or string.format("Unknown (0x%02X)", code)
end

--[[
   Check if a value exceeds a redline threshold
   @param value Current value
   @param redline Redline table with caution/warning/hard fields
   @param invert If true, trigger when value is below threshold (for low voltage)
   @return "hard", "warning", "caution", or nil
]]--
function M.check_redline(value, redline, invert)
    if invert then
        if value <= redline.hard then
            return "hard"
        elseif value <= redline.warning then
            return "warning"
        elseif value <= redline.caution then
            return "caution"
        end
    else
        if value >= redline.hard then
            return "hard"
        elseif value >= redline.warning then
            return "warning"
        elseif value >= redline.caution then
            return "caution"
        end
    end
    return nil
end

return M
