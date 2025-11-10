local HEARTBEAT = {}
HEARTBEAT.id = 0
HEARTBEAT.crc_extra = 50
HEARTBEAT.fields = {
             { "custom_mode", "<I4" },
             { "type", "<B" },
             { "autopilot", "<B" },
             { "base_mode", "<B" },
             { "system_status", "<B" },
             { "mavlink_version", "<B" },
             }
return HEARTBEAT
