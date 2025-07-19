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

MAVLinkAttitude.SCRIPT_VERSION = "4.7.0-009"
MAVLinkAttitude.SCRIPT_NAME = "MAVLink Attitude"
MAVLinkAttitude.SCRIPT_NAME_SHORT = "MAVATT"

ATTITUDE_MESSAGE = "ATTITUDE"

MAV_SEVERITY = {EMERGENCY=0, ALERT=1, CRITICAL=2, ERROR=3, WARNING=4, NOTICE=5, INFO=6, DEBUG=7}

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
    local last_timestamp = millis():tofloat()
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
    local mavlink_msgs = require("mavlink/mavlink_msgs")
    local jitter_correction = MAVLinkAttitude.JitterCorrection(5000, 100)

    -- initialise mavlink rx with  buffer depth and number of messages
    mavlink:init(10, 1)

    -- register message id to receive
    --mavlink.register_rx_msgid(ATTITUDE_msgid)
    mavlink:register_rx_msgid(ATTITUDE_map.id)

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
       local msg,_,timestamp_ms = mavlink:receive_chan()
       if msg then
          local parsed_msg = self.decode(msg, ATTITUDE_map)

          if (parsed_msg ~= nil) and (parsed_msg.msgid == ATTITUDE_map.id) then
               local sysid = parsed_msg.sysid
             local attitude = {}
             attitude.timestamp_ms = jitter_correction.correct_offboard_timestamp_msec(parsed_msg.time_boot_ms, timestamp_ms:toint())
             if attitude.timestamp_ms == nil  -- not sure why this can happen but it can, so need to let the caller know to not use the results
               or attitude.timestamp_ms < last_timestamp then -- if we received a message older than the last most recent message ignore it
                  return nil
             end
             last_timestamp = attitude.timestamp_ms
             attitude.delay_ms = millis():tofloat() - attitude.timestamp_ms   -- the latency/delay from when the message was sent
             attitude.roll = parsed_msg.roll
             attitude.pitch = parsed_msg.pitch
             attitude.yaw = parsed_msg.yaw
             attitude.rollspeed = parsed_msg.rollspeed
             attitude.pitchspeed = parsed_msg.pitchspeed
             attitude.yawspeed = parsed_msg.yawspeed
             if sysid == target_sysid then
                -- Log ATI = Attitude In
                -- Time = Timestamp received in ms
                -- TimeJC = Timestamp received with Jitter Correction
                -- Delay = Delay between message sent and received in ms
                logger:write("ZATI",'Time,TimeJC,Delay,roll,ptch,yaw,rollspd,ptchspd,yawspd','iiiffffff','---rrrEEE','---------',
                        parsed_msg.time_boot_ms,
                        attitude.timestamp_ms,
                        attitude.delay_ms,
                        attitude.roll,
                        attitude.pitch,
                        attitude.yaw,
                        attitude.rollspeed,
                        attitude.pitchspeed,
                        attitude.yawspeed
                     )
                if attitude.delay_ms <= 600 then
                   return attitude
                end
             end
          end
       end
       return nil
    end
    return self
end

MAVLinkAttitude.mavlink_handler = nil
MAVLinkAttitude.mavlink_handler = MAVLinkAttitude.mavlink_attitude_receiver()

gcs:send_text(MAV_SEVERITY.INFO, string.format("%s %s module loaded", MAVLinkAttitude.SCRIPT_NAME, MAVLinkAttitude.SCRIPT_VERSION) )

return MAVLinkAttitude
