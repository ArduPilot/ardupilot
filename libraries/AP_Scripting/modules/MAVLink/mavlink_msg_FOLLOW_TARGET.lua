local FOLLOW_TARGET = {}
FOLLOW_TARGET.id = 144
FOLLOW_TARGET.crc_extra = 127
FOLLOW_TARGET.fields = {
             { "timestamp", "<I8" },
             { "custom_state", "<I8" },
             { "lat", "<i4" },
             { "lon", "<i4" },
             { "alt", "<f" },
             { "vel", "<f", 3 },
             { "acc", "<f", 3 },
             { "attitude_q", "<f", 4 },
             { "rates", "<f", 3 },
             { "position_cov", "<f", 3 },
             { "est_capabilities", "<B" },
             }
return FOLLOW_TARGET
