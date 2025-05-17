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
   Only send is implemented at the moment, but its intended that receive could be added
--]]

local MAVLink_command_int = {}

MAVLink_command_int.SCRIPT_VERSION = "4.7.0-001"
MAVLink_command_int.SCRIPT_NAME = "MAVLink Command Int"
MAVLink_command_int.SCRIPT_NAME_SHORT = "MAVCMDINT"

MAVLink_command_int.COMMAND_INT_MESSAGE = "COMMAND_INT"

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

function MAVLink_command_int.mavlink_encode(msg_map, message)
    local message_map = msg_map

    local packString = "<"
    local packedTable = {}
    local packedIndex = 1
    for i,v in ipairs(message_map.fields) do
      if v[3] then
        packString = (packString .. string.rep(string.sub(v[2], 2), v[3]))
        for j = 1, v[3] do
          packedTable[packedIndex] = message[message_map.fields[i][1]][j]
          if packedTable[packedIndex] == nil then
            packedTable[packedIndex] = 0
          end
          packedIndex = packedIndex + 1
        end
      else
        packString = (packString .. string.sub(v[2], 2))
        packedTable[packedIndex] = message[message_map.fields[i][1]]
        packedIndex = packedIndex + 1
      end
    end
    return message_map.id, string.pack(packString, table.unpack(packedTable))
end

function MAVLink_command_int.send(channel, message)
    return mavlink:send_chan(channel, MAVLink_command_int.mavlink_encode(MAVLink_command_int.map, message))
end

gcs:send_text(MAV_SEVERITY.NOTICE, string.format("%s %s module loaded", 
                    MAVLink_command_int.SCRIPT_NAME, MAVLink_command_int.SCRIPT_VERSION) )

return MAVLink_command_int
