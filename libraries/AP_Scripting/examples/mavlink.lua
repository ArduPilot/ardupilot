function decode_header(message)
  -- build up a map of the result
  local result = {}

  local read_marker = 3

  -- id the MAVLink version
  result.protocol_version, read_marker = string.unpack("<B", message, read_marker)
  if (result.protocol_version == 0xFE) then -- mavlink 1
    result.protocol_version = 1
  elseif (result.protocol_version == 0XFD) then --mavlink 2
    result.protocol_version = 2
  else
    error("Invalid magic byte")
  end

  local payload_len, read_marker = string.unpack("<B", message, read_marker) -- payload is always the second byte

  -- strip the incompat/compat flags
  result.incompat_flags, result.compat_flags, read_marker = string.unpack("<BB", message, read_marker)

  -- fetch seq/sysid/compid
  result.seq, result.sysid, result.compid, read_marker = string.unpack("<BBB", message, read_marker)

  -- fetch the message id
  result.msgid, read_marker = string.unpack("<I3", message, read_marker)

  return result, read_marker
end

function decode(message, messages)
  local result, offset = decode_header(message)
  local message_map = messages[result.msgid]
  if not message_map then
    -- we don't know how to decode this message, bail on it
    return nil
  end

  -- map all the fields out
  for i,v in ipairs(message_map) do
    if v[3] then
      result[v[1]] = {}
      for j=1,v[3] do
        result[v[1]][j], offset = string.unpack(v[2], message, offset)
      end
    else
      result[v[1]], offset = string.unpack(v[2], message, offset)
    end
  end

  -- ignore the idea of a checksum

  return result;
end

function encode(msgid, message, messages)
  local message_map = messages[msgid]   
  if not message_map then                 
    -- we don't know how to encode this message, bail on it
    error("Unknown MAVLink message " .. msgid)
  end

  local packString = "<"
  local packedTable = {}                  
  local packedIndex = 1
  for i,v in ipairs(message_map) do
    if v[3] then
      packString = (packString .. string.rep(string.sub(v[2], 2), v[3]))
      for j = 1, v[3] do
        packedTable[packedIndex] = message[message_map[i][1]][j]
        packedIndex = packedIndex + 1
      end
    else                          
      packString = (packString .. string.sub(v[2], 2))
      packedTable[packedIndex] = message[message_map[i][1]]
      packedIndex = packedIndex + 1
    end
  end

  return string.pack(packString, table.unpack(packedTable))
end

messages = {}
messages[76] = { -- COMMAND_LONG
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
             }
messages[194] = { -- PID_TUNING
             { "desired", "<f" },
             { "achieved", "<f" },
             { "FF", "<f" },
             { "P", "<f" },
             { "I", "<f" },
             { "D", "<f" },
             { "axis", "<B" },
             } 

bar = {}
bar.desired = 12.5
bar.achieved = 13.5
bar.FF = 14.5
bar.P = 15.5
bar.I = 16.5
bar.D = 17.5
bar.axis = 6

mavlink.register_msgid(76)

function update()

  local message = mavlink.receive()
  local decoded_msg
  if message then
    decoded_msg = decode(message, messages)
    gcs:send_text(0, string.format("%d", decoded_msg.command))
  end

  bar.desired = math.random()

  mavlink.send(0, 194, encode(194, bar, messages))

  return update, 500
end

return update()
