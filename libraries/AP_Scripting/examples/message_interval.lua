-- This script is an example of manipulating the message stream rates
--
-- It will periodically run and adjust all the messages to their desired loop rates
-- It can be modified to only run once, however some GCS's will manipulate the rates
-- during the initial connection, so by resetting them periodically we ensure we get
-- the expected telemetry rate

local MAV_SEVERITY = {EMERGENCY=0, ALERT=1, CRITICAL=2, ERROR=3, WARNING=4, NOTICE=5, INFO=6, DEBUG=7}

local loop_time = 5000 -- number of ms between runs

-- mavlink message ids
local AHRS_ID                   = uint32_t(163)
local AHRS2_ID                  = uint32_t(178)
local ATTITUDE_ID               = uint32_t(30)
--local EKF_STATUS_REPORT_ID      = uint32_t(193)
--local GLOBAL_POSITION_INT_ID    = uint32_t(33)
--local GPS_RAW_INT_ID            = uint32_t(24)
--local LOCAL_POSITION_NED_ID     = uint32_t(32)
--local NAV_CONTROLLER_OUTPUT_ID  = uint32_t(62)
--local POWER_STATUS_ID           = uint32_t(125)
--local RAW_IMU_ID                = uint32_t(27)
--local RC_CHANNELS_ID            = uint32_t(65)
--local SERVO_OUTPUT_RAW_ID       = uint32_t(36)
--local SYS_STATUS_ID             = uint32_t(1)
--local SYSTEM_TIME_ID            = uint32_t(2)
--local VFR_HUD_ID                = uint32_t(74)
--local VIBRATION_ID              = uint32_t(241)
--local WIND_ID                   = uint32_t(168)

-- serial port
local serial_port = 0

-- intervals is a table which contains a table for each entry we want to adjust
-- each entry is arranged as {the serial link to adjust, the mavlink message ID, and the broadcast interval in Hz}
local intervals = {{serial_port, ATTITUDE_ID, 2.0}, -- ATTITUDE, 2Hz
                   {serial_port, AHRS_ID, 5.0},     -- AHRS, 5Hz
                   {serial_port, AHRS2_ID, 0},      -- AHRS2, 0hz
                  }

-- print welcome message
gcs:send_text(MAV_SEVERITY.INFO, "Loaded message_interval.lua")

function update() -- this is the loop which periodically runs
  for i = 1, #intervals do -- we want to iterate over every specified interval
    local channel, message, interval_hz = table.unpack(intervals[i]) -- this extracts the channel, MAVLink ID, and interval
    local interval_us = -1
    if interval_hz > 0 then
      interval_us = math.floor(1000000 / interval_hz) -- convert the interval to microseconds
    end
    gcs:set_message_interval(channel, message, interval_us) -- actually sets the interval as appropriate
  end
  return update, loop_time -- reschedules the loop
end

return update() -- run immediately before starting to reschedule
