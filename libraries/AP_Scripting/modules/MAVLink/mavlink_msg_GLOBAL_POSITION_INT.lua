local GLOBAL_POSITION_INT = {}
GLOBAL_POSITION_INT.id = 33
GLOBAL_POSITION_INT.crc_extra = 104
GLOBAL_POSITION_INT.fields = {
             { "time_boot_ms", "<I4" },
             { "lat", "<i4" },
             { "lon", "<i4" },
             { "alt", "<i4" },
             { "relative_alt", "<i4" },
             { "vx", "<i2" },
             { "vy", "<i2" },
             { "vz", "<i2" },
             { "hdg", "<I2" },
             }
return GLOBAL_POSITION_INT
