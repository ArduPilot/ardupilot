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

   MAVLink_command_int
   A MAVlink message manager for COMMAND_INT messages specifically
   This is for sending command_int messages to ANOTHER vehicle. To send to the current
   vehicle that the Lua is running on use gcs:run_command_int
--]]

local MAVLink_command_int = {}

MAVLink_command_int.SCRIPT_VERSION = "4.7.0-005"
MAVLink_command_int.SCRIPT_NAME = "MAVLink Command Int"
MAVLink_command_int.SCRIPT_NAME_SHORT = "MAVCMDINT"

MAVLink_command_int.COMMAND_INT_MESSAGE = "COMMAND_INT"

MAVLink_command_int.MAV_SEVERITY = {EMERGENCY=0, ALERT=1, CRITICAL=2, ERROR=3, WARNING=4, NOTICE=5, INFO=6, DEBUG=7}

MAVLink_command_int.MAV_CMD_INT = { ATTITUDE = 30, GLOBAL_POSITION_INT = 33, REQUEST_DATA_STREAM = 66,
                                    DO_SET_MODE = 176, DO_CHANGE_SPEED = 178, DO_REPOSITION = 192,
                                    CMD_SET_MESSAGE_INTERVAL = 511, CMD_REQUEST_MESSAGE = 512,
                                    GUIDED_CHANGE_SPEED = 43000, GUIDED_CHANGE_ALTITUDE = 43001, GUIDED_CHANGE_HEADING = 43002 }

MAVLink_command_int.map = {}
MAVLink_command_int.map.id = 75
MAVLink_command_int.map.fields = {
             { "param1", "<f" },
             { "param2", "<f" },
             { "param3", "<f" },
             { "param4", "<f" },
             { "x", "<i4" },
             { "y", "<i4" },
             { "z", "<f" },
             { "command", "<I2" },
             { "target_system", "<B" },
             { "target_component", "<B" },
             { "frame", "<B" },
             { "current", "<B" },
             { "autocontinue", "<B" },
             }
MAVLink_command_int.map[MAVLink_command_int.map.id] = MAVLink_command_int.COMMAND_INT_MESSAGE

function MAVLink_command_int.dump(o)
  if type(o) == 'table' then
     local s = '{ '
     for k,v in pairs(o) do
        if type(k) ~= 'number' then k = '"'..k..'"' end
        s = s .. '['..k..'] = ' .. MAVLink_command_int.dump(v) .. ','
     end
     return s .. '} '
  else
     return tostring(o)
  end
end

function MAVLink_command_int.mavlink_encode(msg_map, msg)
    local message_map = msg_map

    local packString = "<"
    local packedTable = {}
    local packedIndex = 1
    for i,v in ipairs(message_map.fields) do
      if v[3] then
        packString = (packString .. string.rep(string.sub(v[2], 2), v[3]))
        for j = 1, v[3] do
          packedTable[packedIndex] = msg[message_map.fields[i][1]][j]
          if packedTable[packedIndex] == nil then
            packedTable[packedIndex] = 0
          end
          packedIndex = packedIndex + 1
        end
      else
        packString = (packString .. string.sub(v[2], 2))
        packedTable[packedIndex] = msg[message_map.fields[i][1]]
        packedIndex = packedIndex + 1
      end
    end
    if next(packedTable) ~= nil then
        return message_map.id, string.pack(packString, table.unpack(packedTable))
    else
        return message_map.id, nil
    end
end

function MAVLink_command_int.send(channel, message)
    local command_id, encoded_message = MAVLink_command_int.mavlink_encode(MAVLink_command_int.map, message)
    if command_id == nil or encoded_message == nil then
        gcs:send_text(MAVLink_command_int.MAV_SEVERITY.WARNING, MAVLink_command_int.SCRIPT_NAME_SHORT .. " MAVLink encode failed")
        return false
    else
        if not mavlink:send_chan(channel, command_id, encoded_message) then
            gcs:send_text(MAVLink_command_int.MAV_SEVERITY.WARNING, MAVLink_command_int.SCRIPT_NAME_SHORT .. " MAVLink buffer is full")
            return false
        end
    end
    return true
end


-- request another vehicle to send specific mavlink messages at a particular interval
-- set_message_interval() Parameters
-- channel = the channel (telemetry link) on this autopilot to send out the request
-- target.sysid = the target vehicle to request messages from
-- target.componentid = the target component, defaults to the autopilot
-- target.message_id = the message id of the requested message
-- target.interval_ms = the interval in milliseconds for the target vehicle to send message_id messages 
function MAVLink_command_int.request_message_interval(channel, target)
    local target_sysid = target.sysid or -1
    if target_sysid < 1 then
        gcs:send_text(MAVLink_command_int.MAV_SEVERITY.ERROR, MAVLink_command_int.SCRIPT_NAME_SHORT .. ": no target: " .. target_sysid)
        return
    end
    if channel < 0 then -- -1 means "don't request"
        return
    end

    local message = {
        command = MAVLink_command_int.MAV_CMD_INT.CMD_SET_MESSAGE_INTERVAL,
        param1 = MAVLink_command_int.MAV_CMD_INT.ATTITUDE,
        param2 = target.interval_ms * 1000, -- request  interval (MAVLink uses microsecond units)
        param3 = 0,
        param4 = 0,
        x = 0, y = 0, z = 0, 
        frame = 0, current = 0, autocontinue = 0,
        target_system = target.sysid,
        target_component = (target.componentid or 1), -- the default to the autopilot
        confirmation = 0,
    }
    --return MAVLink_command_int.send(channel, message)
    local command_id, encoded = MAVLink_command_int.mavlink_encode(MAVLink_command_int.map, message)
    if command_id == nil or encoded == nil then
        return false
    else
        if not mavlink:send_chan(channel, command_id, encoded) then
            gcs:send_text(MAVLink_command_int.MAV_SEVERITY.INFO, MAVLink_command_int.SCRIPT_NAME_SHORT .. " MAVLink buffer is full")
            return false
        end
    end
    return true
end

gcs:send_text(MAV_SEVERITY.INFO, string.format("%s %s module loaded",
                    MAVLink_command_int.SCRIPT_NAME, MAVLink_command_int.SCRIPT_VERSION) )

return MAVLink_command_int
