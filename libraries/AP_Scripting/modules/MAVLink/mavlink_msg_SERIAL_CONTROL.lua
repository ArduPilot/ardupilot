local SERIAL_CONTROL = {}
SERIAL_CONTROL.id = 126
SERIAL_CONTROL.crc_extra = 220
SERIAL_CONTROL.fields = {
             { "baudrate", "<I4" },
             { "timeout", "<I2" },
             { "device", "<B" },
             { "flags", "<B" },
             { "count", "<B" },
             { "data", "<B", 70 },
             }
return SERIAL_CONTROL
