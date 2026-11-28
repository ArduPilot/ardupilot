local COMMAND_LONG = {}
COMMAND_LONG.id = 76
COMMAND_LONG.crc_extra = 152
COMMAND_LONG.extensions_start = 11
COMMAND_LONG.fields = {
             { "param1", "<f" },
             { "param2", "<f" },
             { "param3", "<f" },
             { "param4", "<f" },
             { "param5", "<f" },
             { "param6", "<f" },
             { "param7", "<f" },
             { "command", "<I2" },
             { "target_system", "<B" },
             { "target_component", "<B" },
             { "confirmation", "<B" },
             { "command_opaque_id", "<I4" },
             }
return COMMAND_LONG
