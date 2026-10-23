local COMMAND_ACK = {}
COMMAND_ACK.id = 77
COMMAND_ACK.crc_extra = 143
COMMAND_ACK.fields = {
             { "command", "<I2" },
             { "result", "<B" },
             { "progress", "<B" },
             { "result_param2", "<i4" },
             { "target_system", "<B" },
             { "target_component", "<B" },
             }
return COMMAND_ACK
