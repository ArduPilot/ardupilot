-- Show various CRC calculations

--CRC16 Xmodem
local inString = "Hello, world!"
crc = crc_xmodem(inString)
gcs:send_text(0, "CRC16 Xmodem of '" .. inString .. "' is: " .. string.format("0x%04X", crc))

--CRC16 Modbus
crc = crc_modbus(inString)
gcs:send_text(0, "CRC16 Modbus of '" .. inString .. "' is: " .. string.format("0x%04X", crc))
