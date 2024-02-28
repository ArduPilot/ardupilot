--[[
  Simple example that sends rpm info down the frsky link
  it works with SPort using SERIAL_PROTOCOL=4,10 and with
  FPort using SERIAL_PROTOCOL=23.
  
  We'll be using OpenTX genuine RPM data IDs (https://github.com/opentx/opentx/blob/2.3/radio/src/telemetry/frsky.h)
  
  RPM_FIRST_ID  0x0500
  RPM_LAST_ID   0x050F
  
  and we'll be responding to unused sensor IDs.
  This is a list of IDs we can't use:
  - serial protocol 4 uses IDs 0,2,3 and 6
  - serial protocol 10 uses ID 7,13,20,27
  - serial protocol 23, no IDs used
  
  For this test we'll use sensor ID 4 (0xE4), 
  Note: 4 is the index, 0xE4 is the actual ID
--]]

local loop_time = 250 -- number of ms between runs
local sport_data_frame = 0x10
local sensor_id = 0xE4
--[[
  we use the last 2 data ids so we hopefully don't interfere with pre existing sensors
  this is a suggestion but if we know our RPM sensors are going to be
  the only RPM sensors on the bus we can use 0x0500 and 0x0501
--]]
local data_ids = {
  [0] = 0x050E,
  [1] = 0x050F,
}

local function rpm_data_sent(instance, rpm)
  gcs:send_text(7, string.format("rpm_data_sent() %d: %s", instance, tostring(rpm)))
end

local function send_rpm_data(instance, callback)
  local rpm = RPM:get_rpm(instance)
  gcs:send_text(7,string.format("send_rpm_data() %d, %s", instance, tostring(rpm)))
  if rpm ~= nil then
    if frsky_sport:sport_telemetry_push(sensor_id, sport_data_frame, data_ids[instance], rpm) then
      callback(instance, rpm)
    end
  end
end

local rpm_idx = 0

function update()
    send_rpm_data(rpm_idx, rpm_data_sent)
    rpm_idx = (rpm_idx+1)%2
    return update, loop_time
end

return update() -- run immediately before starting to reschedule
