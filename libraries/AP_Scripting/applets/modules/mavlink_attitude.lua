--[[

   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program.  If not, see <http://www.gnu.org/licenses/>.

   MAVLinkAttitude
   A MAVlink message receiver for ATTITUDE messages specifically
--]]

local MAVLinkAttitude = {}

MAVLinkAttitude.SCRIPT_VERSION = "4.6.0-004"
MAVLinkAttitude.SCRIPT_NAME = "MAVLink Attitude"
MAVLinkAttitude.SCRIPT_NAME_SHORT = "MAVATT"

--[[
   import mavlink support for NAMED_VALUE_FLOAT, only used for
   DUAL_AIRCRAFT operation
--]]

ATTITUDE_MESSAGE = "ATTITUDE"

--[[
   a lua implementation of the jitter correction algorithm from libraries/AP_RTC

   note that the use of a 32 bit float lua number for a uint32_t
   milliseconds means we lose accuracy over time. At 9 hours we have
   an accuracy of about 1 millisecond
--]]
function MAVLinkAttitude.JitterCorrection(_max_lag_ms, _convergence_loops)
    local self = {}

    local max_lag_ms = _max_lag_ms
    local convergence_loops = _convergence_loops
    local link_offset_ms = 0
    local min_sample_ms = 0
    local initialised = false
    local min_sample_counter = 0

    function self.correct_offboard_timestamp_msec(offboard_ms, local_ms)
       local diff_ms = local_ms - offboard_ms
       if not initialised or diff_ms < link_offset_ms then
          --[[
             this message arrived from the remote system with a
             timestamp that would imply the message was from the
             future. We know that isn't possible, so we adjust down the
             correction value
          --]]
         link_offset_ms = diff_ms
         initialised = true
       end

       local estimate_ms = offboard_ms + link_offset_ms

       if estimate_ms + max_lag_ms < local_ms then
          --[[
             this implies the message came from too far in the past. clamp the lag estimate
             to assume the message had maximum lag
          --]]
          estimate_ms = local_ms - max_lag_ms
          link_offset_ms = estimate_ms - offboard_ms
       end
 
       if min_sample_counter == 0 then
          min_sample_ms = diff_ms
       end
 
       min_sample_counter = (min_sample_counter+1)
       if diff_ms < min_sample_ms then
          min_sample_ms = diff_ms
       end
       if min_sample_counter == convergence_loops then
          --[[
             we have the requested number of samples of the transport
             lag for convergence. To account for long term clock drift
             we set the diff we will use in future to this value
          --]]
          link_offset_ms = min_sample_ms
          min_sample_counter = 0
       end
       return estimate_ms
    end
    return self
 end

function MAVLinkAttitude.mavlink_attitude_receiver()
    local self = {}
    local ATTITUDE_map = {}
    ATTITUDE_map.id = 30
    ATTITUDE_map.fields = {
             { "time_boot_ms", "<I4" },
             { "roll", "<f" },
             { "pitch", "<f" },
             { "yaw", "<f" },
             { "rollspeed", "<f" },
             { "pitchspeed", "<f" },
             { "yawspeed", "<f" },
             }
    ATTITUDE_map[ATTITUDE_map.id] = ATTITUDE_MESSAGE
    local mavlink_msgs = require("mavlink_msgs")
    local jitter_correction = MAVLinkAttitude.JitterCorrection(5000, 100)

    -- initialise mavlink rx with number of messages, and buffer depth
    mavlink.init(1, 10)

    -- register message id to receive
    --mavlink.register_rx_msgid(ATTITUDE_msgid)
    mavlink.register_rx_msgid(ATTITUDE_map.id)

    function self.decode(message, message_map)
        local result, offset = mavlink_msgs.decode_header(message)
        -- map all the fields out
        for _,v in ipairs(message_map.fields) do
          if v[3] then
            result[v[1]] = {}
            for j=1,v[3] do
              result[v[1]][j], offset = string.unpack(v[2], message, offset)
            end
          else
            result[v[1]], offset = string.unpack(v[2], message, offset)
          end
        end
        return result;
    end
    --[[
       get an ATTITUDE incoming message from the target vehicle, handling jitter correction
    --]]
    function self.get_attitude(target_sysid)
       local msg,_,timestamp_ms = mavlink.receive_chan()
       if msg then
          local parsed_msg = self.decode(msg, ATTITUDE_map)

          if (parsed_msg ~= nil) and (parsed_msg.msgid == ATTITUDE_map.id) then
             local sysid = parsed_msg.sysid
             local attitude = {}
             attitude.timestamp_ms = jitter_correction.correct_offboard_timestamp_msec(parsed_msg.time_boot_ms, timestamp_ms:toint())
             attitude.roll = parsed_msg.roll
             attitude.pitch = parsed_msg.pitch
             attitude.yaw = parsed_msg.yaw
             attitude.rollspeed = parsed_msg.rollspeed
             attitude.pitchspeed = parsed_msg.pitchspeed
             attitude.yawspeed = parsed_msg.yawspeed
             if sysid == target_sysid then
                 return attitude
             end
          end
       end
       return nil
    end
    return self
end

MAVLinkAttitude.mavlink_handler = nil
MAVLinkAttitude.mavlink_handler = MAVLinkAttitude.mavlink_attitude_receiver()

gcs:send_text(MAV_SEVERITY.NOTICE, string.format("%s %s module loaded", MAVLinkAttitude.SCRIPT_NAME, MAVLinkAttitude.SCRIPT_VERSION) )

return MAVLinkAttitude