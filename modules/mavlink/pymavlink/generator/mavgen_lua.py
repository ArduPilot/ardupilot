#!/usr/bin/env python3
'''
parse a MAVLink protocol XML file and generate a Ardupilot LUA mavlink module
'''

import os

def generate(basename, xml):
    '''generate complete python implemenation'''
    msgs = []
    enums = []
    filelist = []
    for x in xml:
        msgs.extend(x.message)
        enums.extend(x.enum)
        filelist.append(os.path.basename(x.filename))

    unpack_types = {
        'float'    : 'f',
        'double'   : 'd',
        'char'     : 'b',
        'int8_t'   : 'b',
        'uint8_t'  : 'B',
        'uint8_t_mavlink_version'  : 'B',
        'int16_t'  : 'i2',
        'uint16_t' : 'I2',
        'int32_t'  : 'i4',
        'uint32_t' : 'I4',
        'int64_t'  : 'i8',
        'uint64_t' : 'I8',
        }


    print("Generating %s/mavlink_msgs.lua" % basename)
    # create the output directory if needed
    if not os.path.exists(basename):
        os.makedirs(basename)
    outf = open("%s/mavlink_msgs.lua" % basename, "w")
    if 'modules' not in basename:
        print("ERROR: mavlink_msgs.lua must be generated in the modules directory or subdirectory")
        return
    # find the path relative to module directory
    module_root_rel = basename.split('modules')[1]
    module_root_rel = module_root_rel.strip('/')
    if module_root_rel != '':
        module_root_rel += '/'

    # dump the actual symbol table
    for m in msgs:
        mavlink_msg_file = open("%s/mavlink_msg_%s.lua" % (basename, m.name), "w")
        mavlink_msg_file.write("local %s = {}\n" % m.name)
        mavlink_msg_file.write("%s.id = %u\n" % (m.name, m.id))
        mavlink_msg_file.write("%s.crc_extra = %u\n" % (m.name, m.crc_extra))
        mavlink_msg_file.write("%s.fields = {\n" % m.name)
        for i in range(0, len(m.ordered_fields)):
            field = m.ordered_fields[i]
            if ((field.type == 'char') and (field.array_length > 0)):
                # Convert char array to lua string
                mavlink_msg_file.write("             { \"%s\", \"<c%s\" },\n" % (field.name, field.array_length))
            elif (field.array_length > 0):
                mavlink_msg_file.write("             { \"%s\", \"<%s\", %s },\n" % (field.name, unpack_types.get(field.type), field.array_length))
            else:
                mavlink_msg_file.write("             { \"%s\", \"<%s\" },\n" % (field.name, unpack_types.get(field.type)))
        mavlink_msg_file.write("             }\n")
        mavlink_msg_file.write("return %s\n" % m.name)
        mavlink_msg_file.close()
    outf.write(
"""-- Auto generated MAVLink parsing script
local mavlink_msgs = {{}}

---Lookup the message id for a given message name
---@param msgname string
---@return integer -- message id
function mavlink_msgs.get_msgid(msgname)
  local message_map = require("{module_root_rel}mavlink_msg_" .. msgname)
  if not message_map then
    error("Unknown MAVLink message " .. msgname)
  end
  return message_map.id
end

---Return a object containing everything that is not the payload
---@param message any -- encoded message
---@return table
function mavlink_msgs.decode_header(message)
  -- build up a map of the result
  local result = {{}}

  result.checksum = string.unpack("<H", message, 1)

  -- id the MAVLink version
  local magic = string.unpack("<B", message, 3)
  if (magic == 0xFE) then -- mavlink 1
    result.protocol_version = 1
  elseif (magic == 0XFD) then --mavlink 2
    result.protocol_version = 2
  else
    error("Invalid magic byte")
  end

  -- fetch payload length
  result.payload_length = string.unpack("<B", message, 4)

  -- fetch the incompat/compat flags
  result.incompat_flags, result.compat_flags = string.unpack("<BB", message, 5)

  -- fetch seq/sysid/compid
  result.seq, result.sysid, result.compid = string.unpack("<BBB", message, 7)

  -- fetch the message id
  result.msgid = string.unpack("<I3", message, 10)

  return result
end

-- generate the x25crc for a given buffer
---@param buffer string -- buffer to crc
---@return integer -- resulting crc 0 to 0xFFFF
function mavlink_msgs.generateCRC(buffer)
  -- generate the x25crc for a given buffer.
  local crc = 0xFFFF
  for i = 1, #buffer do
      local tmp = string.byte(buffer, i, i) ~ (crc & 0xFF)
      tmp = (tmp ~ (tmp << 4)) & 0xFF
      crc = (crc >> 8) ~ (tmp << 8) ~ (tmp << 3) ~ (tmp >> 4)
      crc = crc & 0xFFFF
  end
  return crc
end

-- Note that this does not parse the serial data, it parses the MAVLink 2 C structure `mavlink_message_t`
-- This structure is passed in by the ArduPilot bindings as a string
---@param message any -- encoded message
---@param msg_map table -- table containing message objects with keys of the message ID
---@return table|nil -- a table representing the contents of the message, or nill if decode failed
function mavlink_msgs.decode(message, msg_map)
  local result = mavlink_msgs.decode_header(message)
  local message_map = require("{module_root_rel}mavlink_msg_" .. msg_map[result.msgid])
  if not message_map then
    -- we don't know how to decode this message, bail on it
    return nil
  end

  -- If we have a crc extra for this message then check it
  -- This ensures compatibility with message definitions generated before the crc check was added
  if message_map.crc_extra then
    -- crc of payload and header values
    local crc_buffer
    if result.protocol_version == 2 then
      crc_buffer = string.sub(message, 4, 12 + result.payload_length)

    else
      -- V1 does not include all fields on the wire
      crc_buffer = string.char(result.payload_length)
      crc_buffer = crc_buffer .. string.char(result.seq)
      crc_buffer = crc_buffer .. string.char(result.sysid)
      crc_buffer = crc_buffer .. string.char(result.compid)
      crc_buffer = crc_buffer .. string.char(result.msgid)
      if result.payload_length > 0 then
        crc_buffer = crc_buffer .. string.sub(message, 13, 12 + result.payload_length)
      end

    end

    local crc = mavlink_msgs.generateCRC(crc_buffer .. string.char(message_map.crc_extra))

    if crc ~= result.checksum then
      -- crc failed
      return nil
    end
  end

  -- map all the fields out
  local offset = 13
  for _,v in ipairs(message_map.fields) do
    if v[3] then
      result[v[1]] = {{}}
      for j=1,v[3] do
        result[v[1]][j], offset = string.unpack(v[2], message, offset)
      end
    else
      result[v[1]], offset = string.unpack(v[2], message, offset)
      if string.sub(v[2],2,2) == 'c' then
        -- Got string, unpack includes 0 values to the set length
        -- this is annoying, so remove them
        result[v[1]] = string.gsub(result[v[1]], string.char(0), "")
      end
    end
  end

  return result
end

---Encode the payload section of a given message
---@param msgname string -- name of message to encode
---@param message table -- table containing key value pairs representing the data fields in the message
---@return integer -- message id
---@return string -- encoded payload
function mavlink_msgs.encode(msgname, message)
  local message_map = require("{module_root_rel}mavlink_msg_" .. msgname)
  if not message_map then
    -- we don't know how to encode this message, bail on it
    error("Unknown MAVLink message " .. msgname)
  end

  local packString = "<"
  local packedTable = {{}}
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

return mavlink_msgs
""".format(module_root_rel=module_root_rel))
    outf.close()
    print("Generated %s OK" % basename)

