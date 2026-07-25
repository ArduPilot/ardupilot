#!/usr/bin/env python3
'''
parse a MAVLink protocol XML file and generate a C# implementation

Copyright Michael Oborne 2018
Released under GNU GPL version 3 or later
'''

import sys, textwrap, os, time, re
from . import mavparse, mavtemplate

t = mavtemplate.MAVTemplate()

enumtypes = {}

map = {
        'float'    : 'float',
        'double'   : 'double',
        'char'     : 'byte',
        'int8_t'   : 'sbyte',
        'uint8_t'  : 'byte',
        'uint8_t_mavlink_version'  : 'B',
        'int16_t'  : 'short',
        'uint16_t' : 'ushort',
        'int32_t'  : 'int',
        'uint32_t' : 'uint',
        'int64_t'  : 'long',
        'uint64_t' : 'ulong',
    }
    
def generate_message_header(f, xml_list):
    dedupe = {}
    for xml in xml_list:
        print("generate_message_header " + xml.basename)
        if xml.little_endian:
            xml.mavlink_endian = "MAVLINK_LITTLE_ENDIAN"
        else:
            xml.mavlink_endian = "MAVLINK_BIG_ENDIAN"

        if xml.crc_extra:
            xml.crc_extra_define = "1"
        else:
            xml.crc_extra_define = "0"

        if xml.command_24bit:
            xml.command_24bit_define = "1"
        else:
            xml.command_24bit_define = "0"

        if xml.sort_fields:
            xml.aligned_fields_define = "1"
        else:
            xml.aligned_fields_define = "0"

        # work out the included headers
        xml.include_list = []
        for i in xml.include:
            base = i[:-4]
            xml.include_list.append(mav_include(base))

        if not hasattr(xml , 'message_names_enum'):
            xml.message_names_enum = ''

        # and message CRCs array
        if not hasattr(xml , 'message_infos_array'):
            xml.message_infos_array = ''
        if xml.command_24bit:
            # we sort with primary key msgid, secondary key dialect
            for msgid in sorted(xml.message_names.keys()):
                name = xml.message_names[msgid]
                if name not in dedupe:
                    dedupe[name] = 1
                    xml_list[0].message_infos_array += '        new message_info(%u, "%s", %u, %u, %u, typeof( mavlink_%s_t )),\n' % (msgid,
                                                                        name,
                                                                        xml.message_crcs[msgid],
                                                                        xml.message_min_lengths[msgid],
                                                                        xml.message_lengths[msgid],
                                                                        name.lower())
                    xml_list[0].message_names_enum += '\n        %s = %u,' % (name, msgid)
        else:
            for msgid in range(256):
                crc = xml.message_crcs.get(msgid, None)
                name = xml.message_names.get(msgid, None)
                length = xml.message_lengths.get(msgid, None)
                if name is not None and name not in dedupe:
                    dedupe[name] = 1
                    xml_list[0].message_infos_array += '        new message_info(%u, "%s", %u, %u, %u, typeof( mavlink_%s_t )), // none 24 bit\n' % (msgid, 
                                                                        name,
                                                                        crc,
                                                                        length,
                                                                        length,
                                                                        name.lower())
                    xml_list[0].message_names_enum += '\n        %s = %u,' % (name, msgid)

        # add some extra field attributes for convenience with arrays
        for m in xml.enum:
            for fe in m.entry[:]:
                fe.name = fe.name.replace("NAV_","")
           
    t.write(f, '''
using System;
using System.Collections.Generic;
using System.Text;
using System.Runtime.InteropServices;

public partial class MAVLink
{
    public const string MAVLINK_BUILD_DATE = "${parse_time}";
    public const string MAVLINK_WIRE_PROTOCOL_VERSION = "${wire_protocol_version}";
    public const int MAVLINK_MAX_PAYLOAD_LEN = ${largest_payload};

    public const byte MAVLINK_CORE_HEADER_LEN = 9;///< Length of core header (of the comm. layer)
    public const byte MAVLINK_CORE_HEADER_MAVLINK1_LEN = 5;///< Length of MAVLink1 core header (of the comm. layer)
    public const byte MAVLINK_NUM_HEADER_BYTES = (MAVLINK_CORE_HEADER_LEN + 1);///< Length of all header bytes, including core and stx
    public const byte MAVLINK_NUM_CHECKSUM_BYTES = 2;
    public const byte MAVLINK_NUM_NON_PAYLOAD_BYTES = (MAVLINK_NUM_HEADER_BYTES + MAVLINK_NUM_CHECKSUM_BYTES);

    public const int MAVLINK_MAX_PACKET_LEN = (MAVLINK_MAX_PAYLOAD_LEN + MAVLINK_NUM_NON_PAYLOAD_BYTES + MAVLINK_SIGNATURE_BLOCK_LEN);///< Maximum packet length
    public const byte MAVLINK_SIGNATURE_BLOCK_LEN = 13;

    public const int MAVLINK_LITTLE_ENDIAN = 1;
    public const int MAVLINK_BIG_ENDIAN = 0;

    public const byte MAVLINK_STX = ${protocol_marker};

    public const byte MAVLINK_STX_MAVLINK1 = 0xFE;

    public const byte MAVLINK_ENDIAN = ${mavlink_endian};

    public const bool MAVLINK_ALIGNED_FIELDS = (${aligned_fields_define} == 1);

    public const byte MAVLINK_CRC_EXTRA = ${crc_extra_define};
    
    public const byte MAVLINK_COMMAND_24BIT = ${command_24bit_define};
        
    public const bool MAVLINK_NEED_BYTE_SWAP = (MAVLINK_ENDIAN == MAVLINK_LITTLE_ENDIAN);
        
    // msgid, name, crc, minlength, length, type
    public static message_info[] MAVLINK_MESSAGE_INFOS = new message_info[] {
${message_infos_array}
    };

    public const byte MAVLINK_VERSION = ${version};

    public const byte MAVLINK_IFLAG_SIGNED=  0x01;
    public const byte MAVLINK_IFLAG_MASK   = 0x01;

    public struct message_info
    {
        public uint msgid { get; internal set; }
        public string name { get; internal set; }
        public byte crc { get; internal set; }
        public uint minlength { get; internal set; }
        public uint length { get; internal set; }
        public Type type { get; internal set; }

        public message_info(uint msgid, string name, byte crc, uint minlength, uint length, Type type)
        {
            this.msgid = msgid;
            this.name = name;
            this.crc = crc;
            this.minlength = minlength;
            this.length = length;
            this.type = type;
        }

        public override string ToString()
        {
            return String.Format("{0} - {1}",name,msgid);
        }
    }   

    public enum MAVLINK_MSG_ID 
    {
${message_names_enum}
    }
    
''', xml_list[0])


def generate_message_enum_types(xml):
    print("generate_message_enum_types: " + xml.filename)
    for m in xml.message:
        for fld in m.fields:
            if fld.array_length == 0:
                fld.type = map[fld.type]
            if fld.enum != "" and fld.array_length == 0:
                enumtypes[fld.enum] = fld.type
                print(fld.enum + " is type " + fld.type)

def cleanText(text):
    text = text.replace("\n"," ")
    text = text.replace("\r"," ")
    return text.replace("\"","'")

def generate_message_enums(f, xml): 
    print("generate_message_enums: " + xml.filename)
    # add some extra field attributes for convenience with arrays
    for m in xml.enum:
        m.description = cleanText(m.description)
        m.flags = ""
        if m.description.lower().find("bitmask") >= 0 or m.name.lower().find("_flags") >= 0:
            m.flags = "[Flags]\n\t"
        m.enumtype = enumtypes.get(m.name,"int /*default*/")
        for fe in m.entry:
            if fe.name.endswith('ENUM_END'):
                m.entry.remove(fe)
                continue
            fe.description = cleanText(fe.description)
            fe.name = fe.name.replace(m.name + "_","")
            firstchar = re.search('^([0-9])', fe.name )
            if firstchar != None and firstchar.group():
                fe.name = '_%s' % fe.name
            if hasattr(fe, "deprecated") and fe.deprecated is True:
                fe.name = '''[Obsolete]
        %s''' % fe.name
            for p in fe.param:
                p.description = cleanText(p.description)
            
    t.write(f, '''
    ${{enum:
    ///<summary> ${description} </summary>
    ${flags}public enum ${name}: ${enumtype}
    {
        ${{entry:///<summary> ${description} |${{param:${description}| }} </summary>
        [Description("${description}")]
        ${name}=${value}, 
        }}
    };
    }}
''', xml)


def generate_message_footer(f, xml):
    t.write(f, '''
}
''', xml)
    f.close()
             

def generate_message_h(f, directory, m):
    '''generate per-message header for a XML file'''
    
    m.obsolete = ""
    if hasattr(m, "deprecated") and m.deprecated is True:
        m.obsolete = "[Obsolete]"

    t.write(f, '''
    ${obsolete}
    /// extensions_start ${extensions_start}
    [StructLayout(LayoutKind.Sequential,Pack=1,Size=${wire_length})]
    ///<summary> ${description} </summary>
    public struct mavlink_${name_lower}_t
    {
        public mavlink_${name_lower}_t(${{ordered_fields:${type} ${name},}}) 
        {
            ${{ordered_fields:  this.${name} = ${name};
            }}
        }
${{ordered_fields:        /// <summary>${description} ${enum} ${units} ${display}</summary>
        [Units("${units}")]
        [Description("${description}")]
        ${array_prefix} ${type} ${name};
    }}
    };

''', m)


class mav_include(object):
    def __init__(self, base):
        self.base = base

def generate_one(fh, basename, xml):
    '''generate headers for one XML file'''
    
    directory = os.path.join(basename, xml.basename)

    print("Generating CSharp implementation for %s" % xml.basename)

    # add some extra field attributes for convenience with arrays
    for m in xml.message:
        m.msg_name = m.name
        if xml.crc_extra:
            m.crc_extra_arg = ", %s" % m.crc_extra
        else:
            m.crc_extra_arg = ""
        m.msg_nameid = "MAVLINK_MSG_ID_${name} = ${id}"
        m.description = cleanText(m.description)
        if m.extensions_start is None:
            m.extensions_start = 0;
        for f in m.fields:
            f.description = cleanText(f.description)
            if f.array_length != 0:
                f.array_prefix = '[MarshalAs(UnmanagedType.ByValArray,SizeConst=%u)]\n\t\tpublic' % f.array_length
                f.array_arg = ', %u' % f.array_length
                f.array_return_arg = '%u, ' % (f.array_length)
                f.array_tag = ''
                f.array_const = 'const '
                f.decode_left = "%s.%s = " % (m.name_lower, f.name)
                f.decode_right = ''
                f.return_type = 'void'
                f.return_value = 'void'
                f.type = "%s%s" % (map[f.type], '[]')
            else:
                if f.enum != "":
                    f.type = "/*" +f.enum + "*/" + f.type;
                    #f.type = "/*" +f.type + "*/" + f.enum;
                f.array_suffix = ''
                f.array_prefix = 'public '
                f.array_tag = 'BitConverter.To%s' % f.type
                if f.type == 'byte':
                    f.array_tag = 'getByte'
                if f.name == 'fixed':   # this is a keyword
                    f.name = '@fixed' 
                f.array_arg = ''
                f.array_return_arg = ''
                f.array_const = ''
                f.decode_left = "%s.%s = " % (m.name_lower, f.name)
                f.decode_right = ''
                f.get_arg = ''
                f.c_test_value = f.test_value
                f.return_type = f.type

    # cope with uint8_t_mavlink_version
    for m in xml.message:
        m.arg_fields = []
        m.array_fields = []
        m.scalar_fields = []
        for f in m.ordered_fields:
            if f.array_length != 0:
                m.array_fields.append(f)
            else:
                m.scalar_fields.append(f)
        for f in m.fields:
            if not f.omit_arg:
                m.arg_fields.append(f)
                f.putname = f.name
            else:
                f.putname = f.const_value
    
    for m in xml.message:
        generate_message_h(fh, directory, m)

def copy_fixed_headers(directory, xml):
    '''copy the fixed protocol headers to the target directory'''
    import shutil, filecmp
    hlist = {
        "1.0": [ 'MavlinkCRC.cs', 'MAVLinkMessage.cs', 'MavlinkParse.cs', 'MavlinkUtil.cs', 'MAVLink.csproj' ],
        "2.0": [ 'MavlinkCRC.cs', 'MAVLinkMessage.cs', 'MavlinkParse.cs', 'MavlinkUtil.cs', 'MAVLink.csproj' ]
        }
    basepath = os.path.dirname(os.path.realpath(__file__))
    srcpath = os.path.join(basepath, 'CS')
    print("Copying fixed headers for protocol %s to %s" % (xml.wire_protocol_version, directory))
    for h in hlist[xml.wire_protocol_version]:
        src = os.path.realpath(os.path.join(srcpath, h))
        dest = os.path.realpath(os.path.join(directory, h))
        if src == dest or (os.path.exists(dest) and filecmp.cmp(src, dest)):
            continue
        shutil.copy(src, dest)


def generate(basename, xml_list):
    '''generate complete MAVLink CSharp implemenation'''
    print("generate for protocol %s to %s" % (xml_list[0].wire_protocol_version, basename))
    
    directory = basename

    if not os.path.exists(directory): 
        os.makedirs(directory) 

    f = open(os.path.join(directory, "mavlink.cs"), mode='w')

    generate_message_header(f, xml_list)

    for xml1 in xml_list:
        generate_message_enum_types(xml1)

    for xml2 in xml_list:
        generate_message_enums(f, xml2)
        
    for xml3 in xml_list:
        generate_one(f, basename, xml3)
    
    generate_message_footer(f,xml_list[0])

    copy_fixed_headers(basename, xml_list[0])
