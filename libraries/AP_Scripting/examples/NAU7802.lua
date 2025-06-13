-- Port of Sparkfun Qwiic Scale library for NAU7802
-- https://github.com/sparkfun/SparkFun_NAU7802_Scale_Arduino_Library

local i2c_bus = i2c:get_device_direct(0,0x2A)
local cal_started = false;
local cal_passed = false;

-- Mask & set a given bit within a register
local function setBit(bitNumber, registerAddress)
  local value = i2c_bus:read_registers(registerAddress)
  value = value | (1 << bitNumber) -- Set this bit
  return i2c_bus:write_register(registerAddress, value)
end

-- Mask & clear a given bit within a register
local function clearBit(bitNumber, registerAddress)
  local value = i2c_bus:read_registers(registerAddress)
  if value then
    value = value & (255 - (1 << bitNumber)) -- Set this bit
    return i2c_bus:write_register(registerAddress, value)
  end
  return false
end

-- Return a given bit within a register
local function getBit(bitNumber, registerAddress)
  local value = i2c_bus:read_registers(registerAddress)
  value = value & (1 << bitNumber) -- Clear all but this bit
  return value > 0
end

-- Returns true if Cycle Ready bit is set (conversion is complete)
local function available()
  return getBit(5, 0);
end

-- Check calibration status.
local function calAFEStatus()
  if getBit(2, 2) then
    return 1
  end

  if getBit(3, 2) then
   return 2
  end

  -- Calibration passed
  return 0;
end

-- Set the readings per second
-- 10, 20, 40, 80, and 320 samples per second is available
local function setSampleRate(rate)
  if rate > 7 then
    rate = 7
  end

  local value = i2c_bus:read_registers(2)
  value = value & 143 -- Clear CRS bits
  value = value | (rate << 4)  -- Mask in new CRS bits

  return i2c_bus:write_register(2, value)
end

-- Select between 1 and 2
local function setChannel(channelNumber)
  if (channelNumber == 0) then
    return clearBit(7, 2); -- Channel 1 (default)
  else
    return setBit(7, 2); -- Channel 2
  end
end

-- Power up digital and analog sections of scale
local function powerUp()
  setBit(1, 0)
  setBit(2, 0)

  -- Wait for Power Up bit to be set - takes approximately 200us
  local counter = 0;
  while (true) do
    if getBit(3, 0) then
      break -- Good to go
    end
    if counter > 100 then
      return false -- Error
    end
    counter = counter + 1
  end
  return true
end

-- Puts scale into low-power mode
local function powerDown()
  clearBit(1, 0)
  return (clearBit(2, 0))
end

-- Resets all registers to Power Of Defaults
local function reset()
  setBit(0, 0) -- Set RR
  return clearBit(0, 0) -- Clear RR to leave reset state
end

-- Set the onboard Low-Drop-Out voltage regulator to a given value
-- 2.4, 2.7, 3.0, 3.3, 3.6, 3.9, 4.2, 4.5V are available
local function setLDO(ldoValue)
  if ldoValue > 7 then
    ldoValue = 7 -- Error check
  end

  -- Set the value of the LDO
  local value = i2c_bus:read_registers(1)
  value = value & 199 -- Clear LDO bits
  value = value | (ldoValue << 3) -- Mask in new LDO bits
  i2c_bus:write_register(1, value);

  return setBit(7, 0) -- Enable the internal LDO
end

-- Set the gain
-- x1, 2, 4, 8, 16, 32, 64, 128 are avaialable
local function setGain(gainValue)
  if gainValue > 7 then
    gainValue = 7 -- Error check
  end

  local value = i2c_bus:read_registers(1)
  value = value & 248 -- Clear gain bits
  value = value | gainValue -- Mask in new bits

  return i2c_bus:write_register(1, value)
end

-- Get the revision code of this IC
local function getRevisionCode()
  revisionCode = i2c_bus:read_registers(31)
  return revisionCode & 0x0F
end

-- Returns 24-bit reading
-- Assumes CR Cycle Ready bit (ADC conversion complete) has been checked to be 1
local function getReading()

  local MSB = i2c_bus:read_registers(18)
  local MID = i2c_bus:read_registers(19)
  local LSB = i2c_bus:read_registers(20)

  if MSB and MID and LSB then

    local value = (MSB << 16) | (MID << 8) | LSB
  -- Quite likely to be a signed vs unsigned error here !! need to check
  
  --  local sign = (value & (1 << 23)) > 0
  --  if sign then
  --    value = 16777215 ~ value -- 23 bit not
  --   end
  -- gcs:send_text(0,string.format("%u, %u, %u, %u",MSB,MID,LSB,value));

    return value
  else
    return false
  end
  
end

-- Call when scale is setup, level, at running temperature, with nothing on it
local function calculateZeroOffset(averageAmount)
  setZeroOffset(getAverage(averageAmount))
end

-- Sets the internal variable. Useful for users who are loading values from NVM.
local function setZeroOffset(newZeroOffset)
  _zeroOffset = newZeroOffset
end

local function getZeroOffset()
  return _zeroOffset
end

-- Call after zeroing. Provide the float weight sitting on scale. Units do not matter.
local function calculateCalibrationFactor(weightOnScale, averageAmount)
  local onScale = getAverage(averageAmount)
  local newCalFactor = (onScale - _zeroOffset) / weightOnScale
  setCalibrationFactor(newCalFactor)
end

-- Pass a known calibration factor into library. Helpful if users is loading settings from NVM.
-- If you don't know your cal factor, call setZeroOffset(), then calculateCalibrationFactor() with a known weight
local function setCalibrationFactor(newCalFactor)
  _calibrationFactor = newCalFactor
end

local function getCalibrationFactor()
  return _calibrationFactor
end

-- Returns the y of y = mx + b using the current weight on scale, the cal factor, and the offset.
local function getWeight(allowNegativeWeights, samplesToTake)
  local onScale = getAverage(samplesToTake)

  -- Prevent the current reading from being less than zero offset
  -- This happens when the scale is zero'd, unloaded, and the load cell reports a value slightly less than zero value
  -- causing the weight to be negative or jump to millions of pounds
  if allowNegativeWeights == false then
   if onScale < _zeroOffset then
      onScale = _zeroOffset -- Force reading to zero
    end
  end

  return (onScale - _zeroOffset) / _calibrationFactor
end

-- Set Int pin to be high when data is ready (default)
local function setIntPolarityHigh()
  return clearBit(7, 1) -- 0 = CRDY pin is high active (ready when 1)
end

-- Set Int pin to be low when data is ready
local function setIntPolarityLow()
  return setBit(7, 1) -- 1 = CRDY pin is low active (ready when 0)
end



local function begin()

  local result = true;

  result = result and reset(); -- Reset all registers

  result = result and powerUp(); -- Power on analog and digital sections of the scale

  result = result and setLDO(4); -- Set LDO to 3.3V

  result = result and setGain(7); -- Set gain to 128

  result = result and setSampleRate(0); -- Set samples per second to 10

  result = result and i2c_bus:write_register(21, 48); -- Turn off CLK_CHP. From 9.1 power on sequencing.

  result = result and setBit(7, 28); -- Enable 330pF decoupling cap on chan 2. From 9.14 application circuit note.

  result = result and setBit(2, 2); -- Begin asynchronous calibration of the analog front end.
                                    -- Poll for completion with calAFEStatus() or wait with waitForCalibrateAFE()

  return result

end

----------------------------------------------------------------------------------------------------------------
-- Actual code to do Basic print reading using the 'library' ---------------------------------------------------
----------------------------------------------------------------------------------------------------------------

function update()
   if available() then
     local currentReading = getReading();
     if currentReading then
       --gcs:send_text(0,"Reading: " .. tostring(currentReading));
     else 
       gcs:send_text(0,"Reading failed");
     end
   end
   return update, 500
end

function init()
  if not cal_started then
    if begin() then
      gcs:send_text(0,"Setup done");
      cal_started = true
    else
      gcs:send_text(0,"Setup failed");
    end
  elseif not cal_passed then
    local cal_status = calAFEStatus()
    if cal_status == 0 then
      gcs:send_text(0,"cal done");
      cal_passed = true;
      return update, 500
    end
    gcs:send_text(0,"cal failed " .. tostring(cal_status));
  end

  return init, 1000
end

return init, 10000 -- Wait a while before starting so we have a better chance of seeing any error messages