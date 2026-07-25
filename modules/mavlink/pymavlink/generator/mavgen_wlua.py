'''
parse a MAVLink protocol XML file and generate a Wireshark Lua dissector

Example usage on Linux or macOS:

# assuming you have cloned the mavlink repo to /mavlink
MXML="/mavlink/message_definitions/v1.0/ardupilotmega.xml"
# Wireshark -> About Wireshark -> Folders -> Personal Lua Plugins
WIRESHARK_PLUGINS="~/.local/lib/wireshark/plugins"
mkdir -p $WIRESHARK_PLUGINS
mavgen.py --lang=WLua $MXML -o $WIRESHARK_PLUGINS/mavlink.lua

After doing this, Wireshark should be able to show details of MAVLink packets.

---

Copyright Holger Steinhaus 2012
Released under GNU GPL version 3 or later
'''

import os
from math import ceil
from . import mavparse, mavtemplate

t = mavtemplate.MAVTemplate()

def get_field_info(field):
    mavlink_type = field.type
    size = field.type_length
    count = field.array_length if field.array_length > 0 else 1

    if mavlink_type == "char":
        # char (string) types
        field_type = "ftypes.STRING"
        tvb_func = "string"
        size = count
        count = 1
    elif "int" in mavlink_type:
        # (u)int types
        field_type = "ftypes." + mavlink_type.replace("_t", "").upper()
        tvb_func = "le_" + ("u" if "u" in mavlink_type else "") + "int" + ("64" if "64" in mavlink_type else "")
    else:
        # float/double
        field_type = "ftypes." + mavlink_type.upper()
        tvb_func = "le_" + mavlink_type

    # If a multiplier is defined, then add this to the displayed mavlink type
    # and set the field type to DOUBLE to ensure that the full number is shown
    if field.multiplier != "":
        mavlink_type = mavlink_type + "*" + field.multiplier
        field_type = "ftypes.DOUBLE"

    return mavlink_type, field_type, tvb_func, size, count


def generate_preamble(outf):
    print("Generating preamble")
    t.write(outf, 
"""
-- Wireshark dissector for the MAVLink protocol (please see https://mavlink.io/en for details)

unknownFrameBeginOffset = 0
mavlink_proto = Proto("mavlink_proto", "MAVLink protocol")
f = mavlink_proto.fields

-- from http://lua-users.org/wiki/TimeZone
local function get_timezone()
    local now = os.time()
    return os.difftime(now, os.time(os.date("!*t", now)))
end
local signature_time_ref = get_timezone() + os.time{year=2015, month=1, day=1, hour=0}

-- threshold to decide if time is absolute or relative (some time in 2005)
time_usec_threshold = UInt64.new(0,0x40000)
-- function to append human-readable time onto unix_time_us fields
local function time_usec_decode(value)
    if value > time_usec_threshold then
        s = math.floor(value:tonumber() / 1000000.0)
        us = value % 1000000
        d = os.date("%Y-%m-%d %H:%M:%S",s)
        us = string.format("%06d",us:tonumber())
        ok, tz = pcall(os.date," %Z",s)
        if not ok then
            tz = os.date(" %z",s)
        end
        return " (" .. d .. "." .. us .. tz .. ")"
    elseif value < 1000000 then
        return ""
    elseif type(value) == "number" then
        return string.format(" (%.6f s)",value / 1000000.0)
    else
        return string.format(" (%.6f s)",value:tonumber() / 1000000.0)
    end
end

payload_fns = {}

protocolVersions = {
    [0xfd] = "MAVLink 2.0",
    [0xfe] = "MAVLink 1.0",
    [0x55] = "MAVLink 0.9"
}

""" )
    
    
def generate_body_fields(outf):
    t.write(outf, 
"""
f.magic = ProtoField.uint8("mavlink_proto.magic", "Magic value / version", base.HEX, protocolVersions)
f.length = ProtoField.uint8("mavlink_proto.length", "Payload length")
f.incompatibility_flag = ProtoField.uint8("mavlink_proto.incompatibility_flag", "Incompatibility flag", base.HEX_DEC)
f.compatibility_flag = ProtoField.uint8("mavlink_proto.compatibility_flag", "Compatibility flag", base.HEX_DEC)
f.sequence = ProtoField.uint8("mavlink_proto.sequence", "Packet sequence")
f.sysid = ProtoField.uint8("mavlink_proto.sysid", "System id", base.DEC)
f.compid = ProtoField.uint8("mavlink_proto.compid", "Component id", base.DEC, enumEntryName.MAV_COMPONENT)
f.msgid = ProtoField.uint24("mavlink_proto.msgid", "Message id", base.DEC, messageName)
f.payload = ProtoField.uint8("mavlink_proto.payload", "Payload", base.DEC, messageName)
f.crc = ProtoField.uint16("mavlink_proto.crc", "Message CRC", base.HEX)
f.signature_link = ProtoField.uint8("mavlink_proto.signature_link", "Link id", base.DEC)
f.signature_time = ProtoField.absolute_time("mavlink_proto.signature_time", "Time")
f.signature_signature = ProtoField.bytes("mavlink_proto.signature_signature", "Signature")
f.rawheader = ProtoField.bytes("mavlink_proto.rawheader", "Unparsable header fragment")
f.rawpayload = ProtoField.bytes("mavlink_proto.rawpayload", "Unparsable payload")

-- fields for FILE_TRANSFER_PROTOCOL.payload dissector
f.FILE_TRANSFER_PROTOCOL_payload_seq_number = ProtoField.uint16("mavlink_proto.FILE_TRANSFER_PROTOCOL_payload_seq_number", "seq_number")
f.FILE_TRANSFER_PROTOCOL_payload_session = ProtoField.uint8("mavlink_proto.FILE_TRANSFER_PROTOCOL_payload_session", "payload_session")
f.FILE_TRANSFER_PROTOCOL_payload_opcode = ProtoField.uint8("mavlink_proto.FILE_TRANSFER_PROTOCOL_payload_opcode", "opcode", base.DEC, enumEntryName.MAV_FTP_OPCODE)
f.FILE_TRANSFER_PROTOCOL_payload_size = ProtoField.uint8("mavlink_proto.FILE_TRANSFER_PROTOCOL_payload_size", "payload_size")
f.FILE_TRANSFER_PROTOCOL_payload_req_opcode = ProtoField.uint8("mavlink_proto.FILE_TRANSFER_PROTOCOL_payload_req_opcode", "req_opcode", base.DEC, enumEntryName.MAV_FTP_OPCODE)
f.FILE_TRANSFER_PROTOCOL_payload_burst_complete = ProtoField.uint8("mavlink_proto.FILE_TRANSFER_PROTOCOL_burst_complete", "burst_complete")
f.FILE_TRANSFER_PROTOCOL_payload_offset = ProtoField.int32("mavlink_proto.FILE_TRANSFER_PROTOCOL_payload_offset", "payload_offset")
f.FILE_TRANSFER_PROTOCOL_payload_data = ProtoField.bytes("mavlink_proto.FILE_TRANSFER_PROTOCOL_payload_data", "payload_data")

""")


def generate_msg_table(outf, msgs):
    t.write(outf, """
messageName = {
""")
    for msg in msgs:
        assert isinstance(msg, mavparse.MAVType)
        t.write(outf, """
    [${msgid}] = '${msgname}',
""", {'msgid':msg.id, 'msgname':msg.name})

    t.write(outf, """
}

""")


def is_power_of_2(n):
    assert isinstance(n, int)
    return (n & (n-1) == 0) and n != 0


def generate_enum_table(outf, enums):
    t.write(outf, """
local enumEntryName = {
""")
    for enum in enums:
        assert isinstance(enum, mavparse.MAVEnum)
        t.write(outf, """
    ["${enumname}"] = {
""", {'enumname': enum.name})

        for entry in enum.entry:
            if not entry.name.endswith("_ENUM_END"):
                t.write(outf, """
        [${entryvalue}] = "${entryname}",
""", {'entryvalue': entry.value, 'entryname': entry.name})

        t.write(outf, """
    },
""")

    t.write(outf, """
}
""")


def generate_field_or_param(outf, field_or_param, name, label, physical_type, field_type, enums):
    assert isinstance(field_or_param, mavparse.MAVEnumParam) or isinstance(field_or_param, mavparse.MAVField)
    values = "nil"
    enum_obj = None
    if field_or_param.enum:
        enum_obj = next((enum for enum in enums if enum.name == field_or_param.enum), None)

    if field_or_param.enum:
        # display name of enum instead of base type
        display_type = field_or_param.enum
        # show enum values for non-flags enums
        if not enum_obj.bitmask:
            values = "enumEntryName." + field_or_param.enum
        else:
            values = values + ", base.HEX_DEC"
            if field_type.startswith("ftypes.INT"):
                field_type = field_type.replace("INT", "UINT")
        # force display type of enums to uint32 so we can show the names
        if field_type in ("ftypes.FLOAT", "ftypes.DOUBLE"):
            field_type = "ftypes.UINT32"
    else:
        display_type = physical_type
        if isinstance(field_or_param, mavparse.MAVField) and field_or_param.display == "bitmask":
            values = values + ", base.HEX_DEC"
    unitstr = " " + field_or_param.units if field_or_param.units else ""
    t.write(outf,
"""
f.${fname} = ProtoField.new("${flabel} (${ftypename})${unitname}", "mavlink_proto.${fname}", ${ftype}, ${fvalues})
""", {'fname': name, 'flabel': label, 'ftypename': display_type, 'ftype': field_type, 'fvalues': values, 'unitname': unitstr})

    # generate flag enum subfields
    if enum_obj and enum_obj.bitmask:
        physical_bits = max(entry.value.bit_length() for entry in enum_obj.entry)
        physical_bits = ceil(physical_bits/4)*4
        for entry in enum_obj.entry:
            if not is_power_of_2(entry.value) or entry.name.endswith("_ENUM_END"):
                # omit flag enums have values like "0: None"
                continue

            t.write(outf,
"""
f.${fname}_flag${ename} = ProtoField.bool("mavlink_proto.${fname}.${ename}", "${ename}", ${fbits}, nil, ${evalue})
""", {'fname': name, 'ename': entry.name, 'fbits': physical_bits, 'evalue': entry.value})


def generate_msg_fields(outf, msg, enums):
    assert isinstance(msg, mavparse.MAVType)
    for f in msg.fields:
        assert isinstance(f, mavparse.MAVField)
        mavlink_type, field_type, _, _, count = get_field_info(f)

        for i in range(0,count):
            if count>1: 
                array_text = '[' + str(i) + ']'
                index_text = '_' + str(i)
            else:
                array_text = ''
                index_text = ''

            name = t.substitute("${fmsg}_${fname}${findex}", {'fmsg':msg.name, 'fname':f.name, 'findex':index_text})
            label = t.substitute("${fname}${farray}", {'fname':f.name, 'farray':array_text})
            generate_field_or_param(outf, f, name, label, mavlink_type, field_type, enums)

    t.write(outf, '\n\n')


def generate_cmd_params(outf, cmd, enums):
    assert isinstance(cmd, mavparse.MAVEnumEntry)

    for p in cmd.param:
        assert isinstance(p, mavparse.MAVEnumParam)
        # only save params that have a label
        if p.label:
            name = t.substitute("cmd_${pcname}_param${pindex}", {'pcname': cmd.name, 'pindex': p.index})
            label = t.substitute("param${pindex}: ${pname}", {'pindex': p.index, 'pname': p.label})
            generate_field_or_param(outf, p, name, label, "float", "ftypes.FLOAT", enums)
            pindex = int(p.index)
            if pindex >= 5:
                # On COMMAND_INT and MISSION_ITEM_INT, params 5,6,7 are named x,y,z ...
                intname = chr(pindex+115)
                name = t.substitute("cmd_${pcname}_${intname}", {'pcname': cmd.name, 'intname': intname})
                label = t.substitute("${intname}: ${pname}", {'intname': intname, 'pname': p.label})
                # ... and the x and y fields are integers
                if pindex == 5 or pindex == 6:
                    generate_field_or_param(outf, p, name, label, "int32_t", "ftypes.INT32", enums)
                else:
                    generate_field_or_param(outf, p, name, label, "float", "ftypes.FLOAT", enums)

    t.write(outf, '\n\n')


def generate_flag_enum_dissector(outf, enum):
    assert isinstance(enum, mavparse.MAVEnum)
    t.write(outf,
"""
-- dissect flag field
function dissect_flags_${enumname}(tree, name, tvbrange, value)
""", {'enumname': enum.name})

    for entry in enum.entry:
        if is_power_of_2(entry.value) and not entry.name.endswith("_ENUM_END"):
            t.write(outf,
"""
    tree:add_le(f[name .. "_flag${entryname}"], tvbrange, value)
""", {'entryname': entry.name})

    t.write(outf,
"""
end
""")

def generate_ftp_payload_dissector(outf, msg, field, offset):
    t.write(outf,
"""
    ftptree = tree:add(padded(offset + ${foffset}, ${fsize}), "payload (uint8_t[${fsize}])")
    tvbrange = padded(offset + ${foffset} + 0, 2)
    subtree = ftptree:add_le(f.FILE_TRANSFER_PROTOCOL_payload_seq_number, tvbrange)
    tvbrange = padded(offset + ${foffset} + 2, 1)
    subtree = ftptree:add_le(f.FILE_TRANSFER_PROTOCOL_payload_session, tvbrange)
    tvbrange = padded(offset + ${foffset} + 3, 1)
    subtree = ftptree:add_le(f.FILE_TRANSFER_PROTOCOL_payload_opcode, tvbrange)
    tvbrange = padded(offset + ${foffset} + 4, 1)
    payload_size = tvbrange:le_int()
    subtree = ftptree:add_le(f.FILE_TRANSFER_PROTOCOL_payload_size, tvbrange)
    tvbrange = padded(offset + ${foffset} + 5, 1)
    subtree = ftptree:add_le(f.FILE_TRANSFER_PROTOCOL_payload_req_opcode, tvbrange)
    tvbrange = padded(offset + ${foffset} + 6, 1)
    subtree = ftptree:add_le(f.FILE_TRANSFER_PROTOCOL_payload_burst_complete, tvbrange)
    tvbrange = padded(offset + ${foffset} + 8, 4)
    subtree = ftptree:add_le(f.FILE_TRANSFER_PROTOCOL_payload_offset, tvbrange)
    tvbrange = padded(offset + ${foffset} + 12, payload_size)
    subtree = ftptree:add_le(f.FILE_TRANSFER_PROTOCOL_payload_data, tvbrange)
""", {'foffset': offset, 'fsize': field.array_length})


# Define custom field dissectors
custom_field_dissector = {
    'FILE_TRANSFER_PROTOCOL.payload': generate_ftp_payload_dissector
}

unit_decoder_mapping = {
    'degE7': 'string.format(\" (%.7f deg)\",value/1E7)',
    'us': 'time_usec_decode(value)',
    'rad': 'string.format(\" (%g deg)\",value*180/math.pi)',
    'rad/s': 'string.format(\" (%g deg/s)\",value*180/math.pi)'
}

def generate_field_dissector(outf, msg, field, offset, enums, cmd=None, param=None):
    # field is the PHYSICAL type
    # but param may have a different LOGICAL type
    assert isinstance(field, mavparse.MAVField)
    assert cmd is None or isinstance(cmd, mavparse.MAVEnumEntry)
    assert param is None or isinstance(param, mavparse.MAVEnumParam)

    # If a custom dissector function is defined, use it instead
    msg_field = f"{msg.name}.{field.name}"
    if msg_field in custom_field_dissector:
        custom_field_dissector[msg_field](outf, msg, field, offset)
        return

    _, _, tvb_func, size, count = get_field_info(field)

    enum_name = param.enum if param else field.enum
    enum_obj = enum_name and next((e for e in enums if e.name == enum_name), None)

    # handle arrays, but not strings

    for i in range(0,count):
        if count>1:
            index_text = '_' + str(i)
        else:
            index_text = ''

        if param is not None:
            if msg.name.endswith("_INT") and int(param.index) >= 5:
                field_var = t.substitute("cmd_${cname}_${intname}", {'cname': cmd.name, 'intname': chr(int(param.index)+115)})
            else:
                field_var = t.substitute("cmd_${cname}_param${pindex}", {'cname': cmd.name, 'pindex': param.index})
        else:
            field_var = t.substitute("${fmsg}_${fname}${findex}", {'fmsg': msg.name, 'fname': field.name, 'findex': index_text})

        # If there is an associated enum and the datatype is not uint, we need to extract
        # and pass the value to add_le, as the raw and ProtoField types will not match.
        # This occurs in the case of using a command field to represent an enum or bitmask
        if enum_obj and tvb_func != "le_uint":
            value_extracted = True
            t.write(outf,
"""
    tvbrange = padded(offset + ${foffset}, ${fbytes})
    value = tvbrange:${ftvbfunc}()
    subtree = tree:add_le(f.${fvar}, tvbrange, value)
""", {'foffset': offset + i * size, 'fbytes': size, 'ftvbfunc': tvb_func, 'fvar': field_var})
        elif field.multiplier != "":
            value_extracted = True
            t.write(outf,
"""
    tvbrange = padded(offset + ${foffset}, ${fbytes})
    value = tvbrange:${ftvbfunc}() * ${mult}
    subtree = tree:add_le(f.${fvar}, tvbrange, value)
""", {'foffset': offset + i * size, 'fbytes': size, 'ftvbfunc': tvb_func, 'fvar': field_var, 'mult': field.multiplier})
        else:
            value_extracted = False
            t.write(outf,
"""
    tvbrange = padded(offset + ${foffset}, ${fbytes})
    subtree = tree:add_le(f.${fvar}, tvbrange)
""", {'foffset': offset + i * size, 'fbytes': size, 'ftvbfunc': tvb_func, 'fvar': field_var})

        unit = field.units.replace("[","").replace("]","")
        if unit in unit_decoder_mapping:
            if not value_extracted:
                t.write(outf,"    value = tvbrange:${ftvbfunc}()\n", {'ftvbfunc': tvb_func})
                value_extracted = True
            t.write(outf,"    subtree:append_text(" + unit_decoder_mapping[unit] + ")\n")

        if enum_obj and enum_obj.bitmask:
            if not value_extracted:
                t.write(outf,"    value = tvbrange:${ftvbfunc}()\n", {'ftvbfunc': tvb_func})
                value_extracted = True
            valuemethod = ":tonumber()" if tvb_func == "le_uint64" else ""
            t.write(outf,
"""
    dissect_flags_${enumname}(subtree, "${fvar}", tvbrange, value${vmeth})
""", {'enumname': enum_name, 'fvar': field_var, 'vmeth': valuemethod})


def generate_payload_dissector(outf, msg, cmds, enums, cmd=None):
    # detect command messages (but not in already command-specific dissectors)
    has_commands = cmds and msg.name in ("COMMAND_INT", "COMMAND_LONG", "COMMAND_ACK", "COMMAND_CANCEL","MISSION_ITEM","MISSION_ITEM_INT") and "command" in msg.field_offsets
    has_args = has_commands and msg.name in ("COMMAND_INT", "COMMAND_LONG","MISSION_ITEM","MISSION_ITEM_INT")

    # for command messages with args, generate command-specific dissectors
    if has_args:
        for subcmd in cmds:
            generate_payload_dissector(outf, msg, None,enums, cmd=subcmd)

    # function header
    if cmd is not None:
        t.write(outf, 
"""
-- dissect payload of message type ${msgname} with command ${cmdname}
function payload_fns.payload_${msgid}_cmd${cmdid}(buffer, tree, msgid, offset, limit, pinfo)
""", {'msgid': msg.id, 'msgname': msg.name, 'cmdid': cmd.value, 'cmdname': cmd.name})
    else:
        t.write(outf, 
"""
-- dissect payload of message type ${msgname}
function payload_fns.payload_${msgid}(buffer, tree, msgid, offset, limit, pinfo)
""", {'msgid': msg.id, 'msgname': msg.name})

    # validate and pad payload if necessary
    t.write(outf, 
"""
    local padded, field_offset, value, subtree, tvbrange
    if (offset + ${msgbytes} > limit) then
        padded = buffer(0, limit):bytes()
        padded:set_size(offset + ${msgbytes})
        padded = padded:tvb("Untruncated payload")
    else
        padded = buffer
    end
""", {'msgbytes': msg.wire_length})

    # for all command messages, show the command name in the info field
    if has_commands:
        t.write(outf,
"""
    local cmd_id = padded(offset + ${foffset}, 2):le_uint()
    local cmd_name = enumEntryName.MAV_CMD[cmd_id]
    if cmd_name ~= nil then
        pinfo.cols.info:append(": " .. cmd_name)
    end
""", {'foffset': msg.field_offsets['command']})

    # for command messages with args, call command-specific dissector if known
    if has_args:
        t.write(outf,
"""
    local cmd_fn = payload_fns["payload_${msgid}_cmd" .. tostring(cmd_id)]
    if cmd_fn ~= nil then
        cmd_fn(buffer, tree, msgid, offset, limit, pinfo)
        return
    end
""", {'msgid': msg.id})
    
    for field in msg.fields:
        # detect command params
        param = None
        if cmd is not None:
            param_index = {'param1': 1, 'param2': 2, 'param3': 3, 'param4': 4, 'param5': 5, 'param6': 6, 'param7': 7, 'x': 5, 'y': 6, 'z': 7}.get(field.name)

            for p in cmd.param:
                if int(p.index) == param_index:
                    param = p
                    break
            if param_index is not None:
                param = next((p for p in cmd.param if int(p.index) == param_index and p.label), None)

        generate_field_dissector(outf, msg, field, msg.field_offsets[field.name], enums, cmd, param)

    t.write(outf, 
"""
end
""")
    

def generate_packet_dis(outf):
    t.write(outf, 
"""
-- dissector function
function mavlink_proto.dissector(buffer,pinfo,tree)
    local offset = 0
    local msgCount = 0
    
    -- loop through the buffer to extract all the messages in the buffer
    while (offset < buffer:len()) 
    do
        msgCount = msgCount + 1
        local subtree = tree:add (mavlink_proto, buffer(), "MAVLink Protocol ("..buffer:len()..")")

        -- decode protocol version first
        local version = buffer(offset,1):uint()
        local protocolString = ""
    
        while (true)
        do
            protocolString = protocolVersions[version]
            if (protocolString ~= nil) then
                break
            else
                protocolString = "unknown"
                -- some unknown data found, record the begin offset
                if (unknownFrameBeginOffset == 0) then
                    unknownFrameBeginOffset = offset
                end
               
                offset = offset + 1
                
                if (offset < buffer:len()) then
                    version = buffer(offset,1):uint()
                else
                    -- no magic value found in the whole buffer. print the raw data and exit
                    if (unknownFrameBeginOffset ~= 0) then
                        if (msgCount == 1) then
                            pinfo.cols.info:set("Unknown message")
                        else
                            pinfo.cols.info:append("  Unknown message")
                        end
                        size = offset - unknownFrameBeginOffset
                        subtree:add(f.rawpayload, buffer(unknownFrameBeginOffset,size))
                        unknownFrameBeginOffset = 0
                    end
                    return
                end
            end
        end
        
        if (unknownFrameBeginOffset ~= 0) then
            pinfo.cols.info:append("Unknown message")
            size = offset - unknownFrameBeginOffset
            subtree:add(f.rawpayload, buffer(unknownFrameBeginOffset,size))
            unknownFrameBeginOffset = 0
            -- jump to next loop
            break
        end
        
        -- some Wireshark decoration
        pinfo.cols.protocol = protocolString

        -- HEADER ----------------------------------------
    
        local msgid
        local length
        local incompatibility_flag

        if (version == 0xfe) then
            if (buffer:len() - 2 - offset > 6) then
                -- normal header
                local header = subtree:add(buffer(offset, 6), "Header")
                header:add(f.magic, buffer(offset,1), version)
                offset = offset + 1
            
                length = buffer(offset,1)
                header:add(f.length, length)
                offset = offset + 1
            
                local sequence = buffer(offset,1)
                header:add(f.sequence, sequence)
                offset = offset + 1
            
                local sysid = buffer(offset,1)
                header:add(f.sysid, sysid)
                offset = offset + 1
        
                local compid = buffer(offset,1)
                header:add(f.compid, compid)
                offset = offset + 1
            
                pinfo.cols.src = "System: "..tostring(sysid:uint())..', Component: '..tostring(compid:uint())
        
                msgid = buffer(offset,1):uint()
                header:add(f.msgid, buffer(offset,1), msgid)
                offset = offset + 1
            else 
                -- handle truncated header
                local hsize = buffer:len() - 2 - offset
                subtree:add(f.rawheader, buffer(offset, hsize))
                offset = offset + hsize
            end
        elseif (version == 0xfd) then
            if (buffer:len() - 2 - offset > 10) then
                -- normal header
                local header = subtree:add(buffer(offset, 10), "Header")
                header:add(f.magic, buffer(offset,1), version)
                offset = offset + 1
                length = buffer(offset,1)
                header:add(f.length, length)
                offset = offset + 1
                incompatibility_flag = buffer(offset,1):uint()
                header:add(f.incompatibility_flag, buffer(offset,1), incompatibility_flag)
                offset = offset + 1
                local compatibility_flag = buffer(offset,1)
                header:add(f.compatibility_flag, compatibility_flag)
                offset = offset + 1
                local sequence = buffer(offset,1)
                header:add(f.sequence, sequence)
                offset = offset + 1
                local sysid = buffer(offset,1)
                header:add(f.sysid, sysid)
                offset = offset + 1
                local compid = buffer(offset,1)
                header:add(f.compid, compid)
                offset = offset + 1
                pinfo.cols.src = "System: "..tostring(sysid:uint())..', Component: '..tostring(compid:uint())
                msgid = buffer(offset,3):le_uint()
                header:add(f.msgid, buffer(offset,3), msgid)
                offset = offset + 3
            else 
                -- handle truncated header
                local hsize = buffer:len() - 2 - offset
                subtree:add(f.rawheader, buffer(offset, hsize))
                offset = offset + hsize
            end
        end


        -- BODY ----------------------------------------
    
        -- dynamically call the type-specific payload dissector    
        local msgnr = msgid
        local dissect_payload_fn = "payload_"..tostring(msgnr)
        local fn = payload_fns[dissect_payload_fn]
        local limit = buffer:len() - 2

        if (length) then
            length = length:uint()
        else
            length = 0
        end

        if (offset + length < limit) then
            limit = offset + length
        end
    
        if (fn == nil) then
            pinfo.cols.info:append ("Unknown message type   ")
            subtree:add_expert_info(PI_MALFORMED, PI_ERROR, "Unknown message type")
            size = buffer:len() - 2 - offset
            subtree:add(f.rawpayload, buffer(offset,size))
            offset = offset + size
        else
            local payload = subtree:add(f.payload, buffer(offset, limit - offset), msgid)
            pinfo.cols.dst:set(messageName[msgid])
            if (msgCount == 1) then
            -- first message should over write the TCP/UDP info
                pinfo.cols.info = messageName[msgid]
            else
                pinfo.cols.info:append("   "..messageName[msgid])
            end
            fn(buffer, payload, msgid, offset, limit, pinfo)
            offset = limit
        end

        -- CRC ----------------------------------------

        local crc = buffer(offset,2)
        subtree:add_le(f.crc, crc)
        offset = offset + 2

        -- SIGNATURE ----------------------------------

        if (version == 0xfd and incompatibility_flag == 0x01) then
            local signature = subtree:add("Signature")

            local link = buffer(offset,1)
            signature:add(f.signature_link, link)
            offset = offset + 1

            local signature_time = buffer(offset,6):le_uint64()
            local time_secs = signature_time / 100000
            local time_nsecs = (signature_time - (time_secs * 100000)) * 10000
            signature:add(f.signature_time, buffer(offset,6), NSTime.new(signature_time_ref + time_secs:tonumber(), time_nsecs:tonumber()))
            offset = offset + 6

            local signature_signature = buffer(offset,6)
            signature:add(f.signature_signature, signature_signature)
            offset = offset + 6
        end

    end
end


""")
    


def generate_epilog(outf):
    print("Generating epilog")
    t.write(outf, 
"""   
-- bind protocol dissector to USER0 linktype

wtap_encap = DissectorTable.get("wtap_encap")
wtap_encap:add(wtap.USER0, mavlink_proto)

-- bind protocol dissector to ports: 14550, 14580, 18570

local udp_dissector_table = DissectorTable.get("udp.port")
udp_dissector_table:add(14550, mavlink_proto)
udp_dissector_table:add(14580, mavlink_proto)
udp_dissector_table:add(18570, mavlink_proto)

-- register common Mavlink TCP ports

DissectorTable.get("tcp.port"):add("5760-5763", mavlink_proto)

""")

def generate(basename, xml):
    '''generate complete lua implemenation'''
    if basename.endswith('.lua'):
        filename = basename
    else:
        filename = basename + '.lua'

    msgs = []
    enums = []
    filelist = []
    for x in xml:
        msgs.extend(x.message)
        enums.extend(x.enum)
        filelist.append(os.path.basename(x.filename))

    # find the MAV_CMD enum
    cmds = next((enum.entry for enum in enums if enum.name == "MAV_CMD"), [])

    for m in msgs:
        m.order_map = [ 0 ] * len(m.fieldnames)
        for i in range(0, len(m.fieldnames)):
            m.order_map[i] = m.ordered_fieldnames.index(m.fieldnames[i])

    print("Generating %s" % filename)
    outf = open(filename, "w")
    generate_preamble(outf)
    generate_msg_table(outf, msgs)
    generate_enum_table(outf, enums)
    generate_body_fields(outf)

    for c in cmds:
        generate_cmd_params(outf, c, enums)
    
    for m in msgs:
        generate_msg_fields(outf, m, enums)

    for e in enums:
        if e.bitmask:
            generate_flag_enum_dissector(outf, e)
    
    for m in msgs:
        generate_payload_dissector(outf, m, cmds, enums)
    
    generate_packet_dis(outf)
#    generate_enums(outf, enums)
#    generate_message_ids(outf, msgs)
#    generate_classes(outf, msgs)
#    generate_mavlink_class(outf, msgs, xml[0])
#    generate_methods(outf, msgs)
    generate_epilog(outf)
    outf.close()
    print("Generated %s OK" % filename)

