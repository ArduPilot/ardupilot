-- This script scans for devices on the i2c bus

---@diagnostic disable: need-check-nil

local address = 0
local found = 0

local i2c_bus = i2c:get_device(0,0)
i2c_bus:set_retries(10)

function update() -- this is the loop which periodically runs

  i2c_bus:set_address(address)

  if i2c_bus:read_registers(0) then
      gcs:send_text(0, "Found I2C at " .. tostring(address))
      found = found + 1
  end

  address = address + 1
  if address == 127 then
    address = 0
    gcs:send_text(0, "Found " .. tostring(found) .. " devices")
    found = 0
  end

  return update, 100 -- reschedules the loop
end

return update() -- run immediately before starting to reschedule
