--[[
    This script reads a SN-GCJA5 panasonic particle sensor on i2c
    reading will be saved to data flash logs, CSV file and streamed as named value floats

    Development of this script was sponsored by Cubepilot

    the code is heavily based on the SparkFun arduino library
    https://github.com/sparkfun/SparkFun_Particle_Sensor_SN-GCJA5_Arduino_Library
]]--

-- search for a index without a file, this stops us overwriting from a previous run
local index = 0
local file_name
while true do
  file_name = string.format('Particle %i.csv',index)
  local file = io.open(file_name)
  local first_line = file:read(1) -- try and read the first character
  io.close(file)
  if first_line == nil then
    break
  end
  index = index + 1
end

-- open file and make header
file = assert(io.open(file_name, 'w'), 'Could not make file :' .. file_name)
file:write('Lattitude (°), Longitude (°), Absolute Altitude (m), PM 1.0, PM 2.5, PM 10, count 0.5, count 1, count 2.5, count 5, count 7.5, count 10\n')
file:close()

-- load the i2c driver, bus 0
local sensor = i2c.get_device(0,0x33)
sensor:set_retries(10)

-- register names
local SNGCJA5_PM1_0 = 0x00
local SNGCJA5_PM2_5 = 0x04
local SNGCJA5_PM10 = 0x08
local SNGCJA5_PCOUNT_0_5 = 0x0C
local SNGCJA5_PCOUNT_1_0 = 0x0E
local SNGCJA5_PCOUNT_2_5 = 0x10
local SNGCJA5_PCOUNT_5_0 = 0x14
local SNGCJA5_PCOUNT_7_5 = 0x16
local SNGCJA5_PCOUNT_10 = 0x18
local SNGCJA5_STATE = 0x26

-- Reads two consecutive bytes from a given location
local function readRegister16(addr)
  local lsb = sensor:read_registers(addr+0)
  local msb = sensor:read_registers(addr+1)
  if lsb and msb then
    return msb << 8 | lsb
  end
end

-- Reads four consecutive bytes from a given location
local function readRegister32(addr)
  local ll = sensor:read_registers(addr+0)
  local lh = sensor:read_registers(addr+1)
  local hl = sensor:read_registers(addr+2)
  local hh = sensor:read_registers(addr+3)
  if ll and lh and hl and hh then
    return (hh << 24) | (hl << 16) | (lh << 8) | (ll << 0)
  end
end

local function getPM(pmRegister)
  local count = readRegister32(pmRegister)
  if count then
    return count / 1000.0
  end
end

function update() -- this is the loop which periodically runs

  -- read status
  local state = sensor:read_registers(SNGCJA5_STATE)
  if not state then
    gcs:send_text(0, "Failed to read particle sensor state")
    return update, 10000
  end

  local Sensors = (state >> 6) & 3
  local PD =  (state >> 4) & 3
  local LD =  (state >> 2) & 3
  local Fan = (state >> 0) & 3

  -- report sensor errors
  if Sensors ~= 0 then
    if Sensors == 1 then
      gcs:send_text(0, "particle sensor: One sensor or fan abnormal")
    elseif Sensors == 2 then
      gcs:send_text(0, "particle sensor: Two sensors or fan abnormal")
    else
      gcs:send_text(0, "particle sensor: Both sensors and fan abnormal")
    end
    return update, 10000
  end

  -- report photo diode errors
  if PD ~= 0 then
    if statusPD == 1 then
      gcs:send_text(0, "particle sensor: Photo diode: Normal w/ software correction")
    elseif statusPD == 2 then
      gcs:send_text(0, "particle sensor: Photo diode: Abnormal, loss of function")
    else
      gcs:send_text(0, "particle sensor: Photo diode: Abnormal, with software correction")
    end
    return update, 10000
  end

  -- report laser diode errors
  if LD ~= 0 then
    if LD == 1 then
      gcs:send_text(0, "particle sensor: Laser diode: Normal w/ software correction")
    elseif LD == 2 then
      gcs:send_text(0, "particle sensor: Laser diode: Abnormal, loss of function")
    else
      gcs:send_text(0, "particle sensor: Laser diode: Abnormal, with software correction")
    end
    return update, 10000
  end

  -- report fan errors
  if Fan ~= 0 then
    if Fan == 1 then
      gcs:send_text(0, "particle sensor: Fan: Normal w/ software correction")
    elseif Fan == 2 then
      gcs:send_text(0, "particle sensor: Fan: In calibration")
    else
      gcs:send_text(0, "particle sensor: Fan: Abnormal, out of control")
    end
    return update, 10000
  end

  -- read mass density
  local PM1_0 = getPM(SNGCJA5_PM1_0)
  local PM2_5 = getPM(SNGCJA5_PM2_5)
  local PM10  = getPM(SNGCJA5_PM10)

  if (not PM1_0) or (not PM2_5) or (not PM10) then
    gcs:send_text(0, "Failed to read particle sensor mass density")
    return update, 10000
  end

  -- read particle counts
  local PC0_5 = readRegister16(SNGCJA5_PCOUNT_0_5)
  local PC1_0 = readRegister16(SNGCJA5_PCOUNT_1_0)
  local PC2_5 = readRegister16(SNGCJA5_PCOUNT_2_5)
  local PC5_0 = readRegister16(SNGCJA5_PCOUNT_5_0)
  local PC7_5 = readRegister16(SNGCJA5_PCOUNT_7_5)
  local PC10  = readRegister16(SNGCJA5_PCOUNT_10)

  if (not PC0_5) or (not PC1_0) or (not PC2_5) or (not PC5_0) or (not PC7_5) or (not PC10) then
    gcs:send_text(0, "Failed to read particle sensor counts")
    return update, 10000
  end

  local lat = 0
  local lng = 0
  local alt = 0

  -- try and get true position, but don't fail for no GPS lock
  local position = ahrs:get_position()
  if position then
    lat = position:lat()*10^-7
    lng = position:lng()*10^-7
    alt = position:alt()*0.01
  end

  -- write to csv
  file = io.open(file_name, 'a')
  file:write(string.format('%0.8f, %0.8f, %0.2f, %0.4f, %0.4f, %0.4f, %i, %i, %i, %i, %i, %i\n',lat,lng,alt,PM1_0,PM2_5,PM10,PC0_5,PC1_0,PC2_5,PC5_0,PC7_5,PC10))
  file:close()

  -- save to data flash
  logger.write('PART','PM1,PM2.5,PM10,Cnt0.5,Cnt1,Cnt2.5,Cnt5,Cnt7.5,Cnt10','fffffffff',PM1_0,PM2_5,PM10,PC0_5,PC1_0,PC2_5,PC5_0,PC7_5,PC10)

  -- send to GCS
  gcs:send_named_float('PM 1.0',PM1_0)
  gcs:send_named_float('PM 2.5',PM2_5)
  gcs:send_named_float('PM 10',PM10)

  gcs:send_named_float('count 0.5',PC0_5)
  gcs:send_named_float('count 1,',PC1_0)
  gcs:send_named_float('count 2.5,',PC2_5)
  gcs:send_named_float('count 5,',PC5_0)
  gcs:send_named_float('count 7.5,',PC7_5)
  gcs:send_named_float('count 10,',PC10)

  return update, 1000 -- reschedules the loop, 1hz
end

return update() -- run immediately before starting to reschedule
