--a message that is not part of any standard mavlink definitions
--intended for use with the MAVLink_send_arbitrary.lua example
--not for normal use
local ARBITRARY = {}
ARBITRARY.id = 51000
ARBITRARY.crc_extra = 203
ARBITRARY.min_len=244   --required so that mavlink_msgs.encode will return 7 arguments instead of 4
ARBITRARY.max_len=244   --required so that mavlink_msgs.encode will return 7 arguments instead of 4
ARBITRARY.fields = {
             { "target_system", "<B" },
             { "target_component", "<B" },
             { "flags", "<B" },
             { "subtype", "<B" },
             { "payload", "<B", 240 },
             }
return ARBITRARY

