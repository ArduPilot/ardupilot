--[[
  Simple example that sends battery info down the frsky link
  it works with SPort using SERIAL_PROTOCOL=4,10 and with
  FPort using SERIAL_PROTOCOL=23.
  
  We'll be using OpenTX genuine data IDs (https://github.com/opentx/opentx/blob/2.3/radio/src/telemetry/frsky.h)
  
  CURR_FIRST_ID  0x0200
  VFAS_FIRST_ID  0x0210
  
  and we'll be responding to unused sensor IDs.
  This is a list of IDs we can't use:
  - serial protocol 4 uses IDs 0,2,3 and 6
  - serial protocol 10 uses ID 7,13,20,27
  - serial protocol 23, no IDs used
  
  For this test we'll use sensor ID 4 (0xE4), 
  Note: 4 is the index, 0xE4 is the actual ID
--]]

local loop_time = 500 -- number of ms between runs
local sport_data_frame = 0x10
local sensor_id = 0xE4

-- battery 1->instance 0, battery 2->instance 1, etc
local batt_instance = 0

local curr_id = 0x020E
local vfas_id = 0x021E

local function send_voltage(instance)
  local volts = battery:voltage(instance)
  gcs:send_text(7,string.format("FRSKY: batt %d, %.01fV", instance, volts))
  if volts ~= nil then
    frsky_sport:sport_telemetry_push(sensor_id, sport_data_frame, vfas_id, math.floor(volts*100+0.5)) -- centivolts
  end
end

local function send_current(instance)
  local amps = battery:current_amps(instance)
  gcs:send_text(7,string.format("FRSKY: batt %d, %.01fA", instance, amps))
  if amps ~= nil then
    frsky_sport:sport_telemetry_push(sensor_id, sport_data_frame, curr_id, math.floor(amps*10+0.5)) -- deciamps
  end
end

local alternate = 0

function update()
  if batt_instance > battery:num_instances() then
    error("Battery " .. batt_instance .. " does not exist")
  end
  
  if alternate % 2 == 0 then
    send_voltage(batt_instance)
  else
    send_current(batt_instance)
  end
  alternate = (alternate+1)%2
  return update, loop_time
end

return update() -- run immediately before starting to reschedule
