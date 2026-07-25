#!/usr/bin/env python3
'''
mavlink python parse functions

Copyright Andrew Tridgell 2011
Released under GNU GPL version 3 or later
'''

import errno
import operator
import os
import sys
import time
import xml.parsers.expat

PROTOCOL_0_9 = "0.9"
PROTOCOL_1_0 = "1.0"
PROTOCOL_2_0 = "2.0"

# message flags
FLAG_HAVE_TARGET_SYSTEM    = 1
FLAG_HAVE_TARGET_COMPONENT = 2

class MAVParseError(Exception):
    def __init__(self, message, inner_exception=None):
        self.message = message
        self.inner_exception = inner_exception
        self.exception_info = sys.exc_info()
    def __str__(self):
        return self.message

class MAVField(object):
    def __init__(self, name, type, print_format, xml, description='', enum='', display='', units='', multiplier='', instance=False, invalid=''):
        self.name = name
        self.name_upper = name.upper()
        self.description = description
        self.array_length = 0
        self.enum = enum
        self.display = display
        self.units = units
        self.multiplier = multiplier
        self.omit_arg = False
        self.const_value = None
        self.print_format = print_format
        self.instance = instance
        self.invalid = invalid
        lengths = {
        'float'    : 4,
        'double'   : 8,
        'char'     : 1,
        'int8_t'   : 1,
        'uint8_t'  : 1,
        'uint8_t_mavlink_version'  : 1,
        'int16_t'  : 2,
        'uint16_t' : 2,
        'int32_t'  : 4,
        'uint32_t' : 4,
        'int64_t'  : 8,
        'uint64_t' : 8,
        }

        if type=='uint8_t_mavlink_version':
            type = 'uint8_t'
            self.omit_arg = True
            self.const_value = xml.version

        aidx = type.find("[")
        if aidx != -1:
            assert type[-1:] == ']'
            self.array_length = int(type[aidx+1:-1])
            type = type[0:aidx]
            if type == 'array':
                type = 'int8_t'
        if type in lengths:
            self.type_length = lengths[type]
            self.type = type
        elif (type+"_t") in lengths:
            self.type_length = lengths[type+"_t"]
            self.type = type+'_t'
        else:
            raise MAVParseError("unknown type '%s'" % type)
        if self.array_length != 0:
            self.wire_length = self.array_length * self.type_length
        else:
            self.wire_length = self.type_length
        self.type_upper = self.type.upper()

    def gen_test_value(self, i):
        '''generate a testsuite value for a MAVField'''
        if self.const_value:
            return self.const_value
        elif self.type == 'float':
            return 17.0 + self.wire_offset*7 + i
        elif self.type == 'double':
            return 123.0 + self.wire_offset*7 + i
        elif self.type == 'char':
            return chr(ord('A') + (self.wire_offset + i)%26)
        elif self.type in [ 'int8_t', 'uint8_t' ]:
            return (5 + self.wire_offset*67 + i) & 0xFF
        elif self.type in ['int16_t', 'uint16_t']:
            return (17235 + self.wire_offset*52 + i) & 0xFFFF
        elif self.type in ['int32_t', 'uint32_t']:
            return (963497464 + self.wire_offset*52 + i)&0xFFFFFFFF
        elif self.type in ['int64_t', 'uint64_t']:
            return 93372036854775807 + self.wire_offset*63 + i
        else:
            raise MAVParseError('unknown type %s' % self.type)

    def set_test_value(self):
        '''set a testsuite value for a MAVField'''
        if self.array_length:
            self.test_value = []
            for i in range(self.array_length):
                self.test_value.append(self.gen_test_value(i))
        else:
                self.test_value = self.gen_test_value(0)
        if self.type == 'char' and self.array_length:
            v = ""
            for c in self.test_value:
                v += c
            self.test_value = v[:-1]


class MAVDeprecated(object):
    def __init__(self, since, replaced_by, explanation=''):
        self.since = since
        self.replaced_by = replaced_by
        self.explanation = explanation

class MAVType(object):
    def __init__(self, name, id, linenumber, description='', wip=False):
        self.name = name
        self.name_lower = name.lower()
        self.linenumber = linenumber
        self.id = int(id)
        self.deprecated = None
        self.description = description
        self.wip = wip
        self.fields = []
        self.fieldnames = []
        self.extensions_start = None
        self.needs_pack = False

    def base_fields(self):
        '''return number of non-extended fields'''
        if self.extensions_start is None:
            return len(self.fields)
        return len(self.fields[:self.extensions_start])

class MAVEnumParam(object):
    def __init__(self, index, description='', label='', units='', enum='', increment='', minValue='', maxValue='', reserved=False, default='', multiplier=''):
        self.index = index
        self.description = description
        self.label = label
        self.units = units
        self.enum = enum
        self.increment = increment
        self.minValue = minValue
        self.maxValue = maxValue
        self.reserved = reserved
        self.default = default
        self.multiplier = multiplier
        if self.reserved and not self.default:
            self.default = '0'
        self.set_description(description)

    def set_description(self, description):
        if not description.strip() and self.reserved:
            self.description = 'Reserved (default:%s)' % self.default
        else:
            self.description = description

class MAVEnumEntry(object):
    def __init__(self, name, value, description='', wip=False, end_marker=False, autovalue=False, origin_file='', origin_line=0, has_location=False):
        self.name = name
        self.value = value
        self.deprecated = None
        self.description = description
        self.wip = wip
        self.param = []
        self.end_marker = end_marker
        self.autovalue = autovalue  # True if value was *not* specified in XML
        self.origin_file = origin_file
        self.origin_line = origin_line
        self.has_location = has_location

class MAVEnum(object):
    def __init__(self, name, linenumber, description='', bitmask=False):
        self.name = name
        self.deprecated = None
        self.description = description
        self.entry = []
        self.start_value = None
        self.highest_value = 0
        self.linenumber = linenumber
        self.bitmask = bitmask

class MAVXML(object):
    '''parse a mavlink XML file'''
    def __init__(self, filename, wire_protocol_version=PROTOCOL_0_9):
        self.filename = filename
        self.basename = os.path.basename(filename)
        if self.basename.lower().endswith(".xml"):
            self.basename = self.basename[:-4]
        self.basename_upper = self.basename.upper()
        self.message = []
        self.enum = []
        # we use only the day for the parse_time, as otherwise
        # it causes a lot of unnecessary cache misses with ccache
        self.parse_time = time.strftime("%a %b %d %Y")
        self.version = 2
        self.include = []
        self.wire_protocol_version = wire_protocol_version

        # setup the protocol features for the requested protocol version
        if wire_protocol_version == PROTOCOL_0_9:
            self.protocol_marker = ord('U')
            self.sort_fields = False
            self.little_endian = False
            self.crc_extra = False
            self.crc_struct = False
            self.command_24bit = False
            self.allow_extensions = False
        elif wire_protocol_version == PROTOCOL_1_0:
            self.protocol_marker = 0xFE
            self.sort_fields = True
            self.little_endian = True
            self.crc_extra = True
            self.crc_struct = False
            self.command_24bit = False
            self.allow_extensions = False
        elif wire_protocol_version == PROTOCOL_2_0:
            self.protocol_marker = 0xFD
            self.sort_fields = True
            self.little_endian = True
            self.crc_extra = True
            self.crc_struct = True
            self.command_24bit = True
            self.allow_extensions = True
        else:
            print("Unknown wire protocol version")
            print("Available versions are: %s %s %s" % (PROTOCOL_0_9, PROTOCOL_1_0, PROTOCOL_2_0))
            raise MAVParseError('Unknown MAVLink wire protocol version %s' % wire_protocol_version)

        in_element_list = []

        def check_attrs(attrs, check, where):
            for c in check:
                if c not in attrs:
                    raise MAVParseError('expected missing %s "%s" attribute at %s:%u' % (
                        where, c, filename, p.CurrentLineNumber))

        def start_element(name, attrs):
            in_element_list.append(name)
            in_element = '.'.join(in_element_list)
            #print in_element
            if in_element == "mavlink.messages.message":
                check_attrs(attrs, ['name', 'id'], 'message')
                self.message.append(MAVType(attrs['name'], attrs['id'], p.CurrentLineNumber))
            elif in_element == "mavlink.messages.message.wip":
                self.message[-1].wip = True
            elif in_element == "mavlink.messages.message.extensions":
                self.message[-1].extensions_start = len(self.message[-1].fields)
            elif in_element == "mavlink.messages.message.deprecated":
                check_attrs(attrs, ['since', 'replaced_by'], 'deprecated')
                self.message[-1].deprecated = MAVDeprecated(attrs['since'], attrs['replaced_by'])
            elif in_element == "mavlink.messages.message.field":
                check_attrs(attrs, ['name', 'type'], 'field')
                print_format = attrs.get('print_format', None)
                enum = attrs.get('enum', '')
                display = attrs.get('display', '')
                units = attrs.get('units', '')
                if units:
                    units = '[' + units + ']'
                invalid = attrs.get('invalid', '')
                multiplier = attrs.get('multiplier', '')
                instance = attrs.get('instance', False)
                new_field = MAVField(attrs['name'], attrs['type'], print_format, self, enum=enum, display=display, units=units, multiplier=multiplier, instance=instance, invalid=invalid)
                if self.message[-1].extensions_start is None or self.allow_extensions:
                    self.message[-1].fields.append(new_field)
            elif in_element == "mavlink.enums.enum":
                check_attrs(attrs, ['name'], 'enum')
                bitmask = 'bitmask' in attrs and attrs['bitmask'] == 'true'
                self.enum.append(MAVEnum(attrs['name'], p.CurrentLineNumber, bitmask=bitmask))
            elif in_element == "mavlink.enums.enum.deprecated":
                check_attrs(attrs, ['since', 'replaced_by'], 'deprecated')
                self.enum[-1].deprecated = MAVDeprecated(attrs['since'], attrs['replaced_by'])
            elif in_element == "mavlink.enums.enum.entry":
                check_attrs(attrs, ['name'], 'enum entry')
                # determine value and if it was automatically assigned (for possible merging later)
                if 'value' in attrs:
                    value = eval(attrs['value'])
                    autovalue = False
                else:
                    value = self.enum[-1].highest_value + 1
                    autovalue = True
                # check lowest value
                if (self.enum[-1].start_value is None or value < self.enum[-1].start_value):
                    self.enum[-1].start_value = value
                # check highest value
                if (value > self.enum[-1].highest_value):
                    self.enum[-1].highest_value = value
                has_location = attrs.get('hasLocation', False)
                if has_location == 'true':
                    has_location = True
                elif has_location == 'false':
                    has_location = False
                if type(has_location) != bool:
                    raise MAVParseError("invalid has_location value %s" % has_location)

                # check bitmask value
                if self.enum[-1].bitmask:
                    # values should always be a power of 2.  Py3.10
                    # has value.bit_count()
                    if bin(value).count("1") != 1:
                        print(f"{attrs['name']} has invalid values (bitmask must have powers of 2)")

                # append the new entry
                self.enum[-1].entry.append(MAVEnumEntry(attrs['name'], value, '', False, False, autovalue, self.filename, p.CurrentLineNumber, has_location=has_location))
            elif in_element == "mavlink.enums.enum.entry.wip":
                self.enum[-1].entry[-1].wip = True
            elif in_element == "mavlink.enums.enum.entry.param":
                check_attrs(attrs, ['index'], 'enum param')
                self.enum[-1].entry[-1].param.append(
                                                MAVEnumParam(attrs['index'], 
                                                        label=attrs.get('label', ''), units=attrs.get('units', ''), 
                                                        enum=attrs.get('enum', ''), increment=attrs.get('increment', ''), 
                                                        minValue=attrs.get('minValue', ''), 
                                                        maxValue=attrs.get('maxValue', ''), default=attrs.get('default', '0'), 
                                                        reserved=attrs.get('reserved', False), multiplier=attrs.get('multiplier','')))
            elif in_element == "mavlink.enums.enum.entry.deprecated":
                check_attrs(attrs, ['since', 'replaced_by'], 'deprecated')
                self.enum[-1].entry[-1].deprecated = MAVDeprecated(attrs['since'], attrs['replaced_by'])

        def is_target_system_field(m, f):
            if f.name == 'target_system':
                return True
            if m.name == "MANUAL_CONTROL" and f.name == "target":
                return True
            return False

        def end_element(name):
            in_element_list.pop()

        def char_data(data):
            in_element = '.'.join(in_element_list)
            if in_element == "mavlink.messages.message.description":
                self.message[-1].description += data
            elif in_element == "mavlink.messages.message.deprecated":
                self.message[-1].deprecated.explanation += data
            elif in_element == "mavlink.messages.message.field":
                if self.message[-1].extensions_start is None or self.allow_extensions:
                    self.message[-1].fields[-1].description += data
            elif in_element == "mavlink.enums.enum.description":
                self.enum[-1].description += data
            elif in_element == "mavlink.enums.enum.deprecated":
                self.enum[-1].deprecated.explanation += data
            elif in_element == "mavlink.enums.enum.entry.description":
                self.enum[-1].entry[-1].description += data
            elif in_element == "mavlink.enums.enum.entry.deprecated":
                self.enum[-1].entry[-1].deprecated.explanation += data
            elif in_element == "mavlink.enums.enum.entry.param":
                self.enum[-1].entry[-1].param[-1].description += data
            elif in_element == "mavlink.version":
                self.version = int(data)
            elif in_element == "mavlink.include":
                self.include.append(data)

        f = open(filename, mode='rb')
        p = xml.parsers.expat.ParserCreate()
        p.StartElementHandler = start_element
        p.EndElementHandler = end_element
        p.CharacterDataHandler = char_data
        p.ParseFile(f)
        f.close()
   

        #Post process to add reserved params (for docs)
        for current_enum in self.enum:
            if not 'MAV_CMD' in current_enum.name:
                continue
            for enum_entry in current_enum.entry:
                if len(enum_entry.param) == 7:
                    continue
                params_dict=dict()
                for param_index in range (1,8):
                    params_dict[param_index] = MAVEnumParam(param_index, label='', units='', enum='', increment='', 
                                                        minValue='', maxValue='', default='0', reserved='True')

                for a_param in enum_entry.param:
                    params_dict[int(a_param.index)] = a_param
                enum_entry.param=params_dict.values()
                


        self.message_lengths = {}
        self.message_min_lengths = {}
        self.message_flags = {}
        self.message_target_system_ofs = {}
        self.message_target_component_ofs = {}
        self.message_crcs = {}
        self.message_names = {}
        self.largest_payload = 0

        if not self.command_24bit:
            # remove messages with IDs > 255
            m2 = []
            for m in self.message:
                if m.id <= 255:
                    m2.append(m)
            self.message = m2

        for m in self.message:
            if not self.command_24bit and m.id > 255:
                continue

            m.wire_length = 0
            m.wire_min_length = 0
            m.fieldnames = []
            m.fieldlengths = []
            m.ordered_fieldnames = []
            m.ordered_fieldtypes = []
            m.fieldtypes = []
            m.message_flags = 0
            m.target_system_ofs = 0
            m.target_component_ofs = 0
            m.field_offsets = {}
            
            if self.sort_fields:
                # when we have extensions we only sort up to the first extended field
                sort_end = m.base_fields()
                m.ordered_fields = sorted(m.fields[:sort_end],
                                                   key=operator.attrgetter('type_length'),
                                                   reverse=True)
                m.ordered_fields.extend(m.fields[sort_end:])
            else:
                m.ordered_fields = m.fields
            for f in m.fields:
                m.fieldnames.append(f.name)
                L = f.array_length
                if L == 0:
                    m.fieldlengths.append(1)
                elif L > 1 and f.type == 'char':
                    m.fieldlengths.append(1)
                else:
                    m.fieldlengths.append(L)
                m.fieldtypes.append(f.type)
            for i in range(len(m.ordered_fields)):
                f = m.ordered_fields[i]
                f.wire_offset = m.wire_length
                m.field_offsets[f.name] = f.wire_offset
                m.wire_length += f.wire_length
                field_el_length = f.wire_length
                if f.array_length > 1:
                    field_el_length = f.wire_length / f.array_length
                if f.wire_offset % field_el_length != 0:
                    # misaligned field, structure will need packing in C
                    m.needs_pack = True
                if m.extensions_start is None or i < m.extensions_start:
                    m.wire_min_length = m.wire_length
                m.ordered_fieldnames.append(f.name)
                m.ordered_fieldtypes.append(f.type)
                f.set_test_value()
                if f.name.find('[') != -1:
                    raise MAVParseError("invalid field name with array descriptor %s" % f.name)
                # having flags for target_system and target_component helps a lot for routing code
                if is_target_system_field(m, f):
                    m.message_flags |= FLAG_HAVE_TARGET_SYSTEM
                    m.target_system_ofs = f.wire_offset
                elif f.name == 'target_component':
                    m.message_flags |= FLAG_HAVE_TARGET_COMPONENT
                    m.target_component_ofs = f.wire_offset
            m.num_fields = len(m.fieldnames)
            if m.num_fields > 64:
                raise MAVParseError("num_fields=%u : Maximum number of field names allowed is %u" % (
                    m.num_fields, 64))
            m.crc_extra = message_checksum(m)

            key = m.id
            self.message_crcs[key] = m.crc_extra
            self.message_lengths[key] = m.wire_length
            self.message_min_lengths[key] = m.wire_min_length
            self.message_names[key] = m.name
            self.message_flags[key] = m.message_flags
            self.message_target_system_ofs[key] = m.target_system_ofs
            self.message_target_component_ofs[key] = m.target_component_ofs

            if m.wire_length > self.largest_payload:
                self.largest_payload = m.wire_length

    def __str__(self):
        return "MAVXML for %s from %s (%u message, %u enums)" % (
            self.basename, self.filename, len(self.message), len(self.enum))


def message_checksum(msg):
    '''calculate CRC-16/MCRF4XX checksum of the key fields of a message, so we
       can detect incompatible XML changes'''
    from .mavcrc import x25crc
    crc = x25crc()
    crc.accumulate_str(msg.name + ' ')
    # in order to allow for extensions the crc does not include
    # any field extensions
    crc_end = msg.base_fields()
    for i in range(crc_end):
        f = msg.ordered_fields[i]
        crc.accumulate_str(f.type + ' ')
        crc.accumulate_str(f.name + ' ')
        if f.array_length:
            crc.accumulate([f.array_length])
    return (crc.crc&0xFF) ^ (crc.crc>>8)

def merge_enums(xml):
    '''merge enums between XML files'''
    emap = {}
    for x in xml:
        newenums = []
        for enum in x.enum:
            if enum.name in emap:
                emapitem = emap[enum.name]
                # check for possible conflicting auto-assigned values after merge
                if (emapitem.start_value <= enum.highest_value and emapitem.highest_value >= enum.start_value):
                    for entry in emapitem.entry:
                        # correct the value if necessary, but only if it was auto-assigned to begin with
                        if entry.value <= enum.highest_value and entry.autovalue is True:
                            entry.value = enum.highest_value + 1
                            enum.highest_value = entry.value
                # merge the entries
                emapitem.entry.extend(enum.entry)
                if not emapitem.description:
                    emapitem.description = enum.description
                print("Merged enum %s" % enum.name)
            else:
                newenums.append(enum)
                emap[enum.name] = enum
        x.enum = newenums
    for e in emap:
        # sort by value
        emap[e].entry = sorted(emap[e].entry,
                               key=operator.attrgetter('value'),
                               reverse=False)
        # add a ENUM_END
        emap[e].entry.append(MAVEnumEntry("%s_ENUM_END" % emap[e].name,
                                            emap[e].entry[-1].value+1, end_marker=True))

def check_duplicates(xml):
    '''check for duplicate message IDs'''

    merge_enums(xml)

    msgmap = {}
    msg_name_map = {}
    enummap = {}
    for x in xml:
        for m in x.message:
            key = m.id
            if key in msgmap:
                print("ERROR: Duplicate message id %u for %s (%s:%u) also used by %s" % (
                    m.id,
                    m.name,
                    x.filename, m.linenumber,
                    msgmap[key]))
                return True
            fieldset = set()
            for f in m.fields:
                if f.name in fieldset:
                    print("ERROR: Duplicate field %s in message %s (%s:%u)" % (
                        f.name, m.name,
                        x.filename, m.linenumber))
                    return True
                fieldset.add(f.name)
            msgmap[key] = '%s (%s:%u)' % (m.name, x.filename, m.linenumber)
            # Check for duplicate message names
            if m.name in msg_name_map:
                print("ERROR: Duplicate message name %s for id:%u (%s:%u) also used by %s" % (
                    m.name,
                    m.id,
                    x.filename, m.linenumber,
                    msg_name_map[m.name]))
                return True
            msg_name_map[m.name] = '%s (%s:%u)' % (m.id, x.filename, m.linenumber)
        for enum in x.enum:
            for entry in enum.entry:
                if entry.autovalue is True and "common.xml" not in entry.origin_file:
                    print("Note: An enum value was auto-generated: %s = %u" % (entry.name, entry.value))
                s1 = "%s.%s" % (enum.name, entry.name)
                s2 = "%s.%s" % (enum.name, entry.value)
                if s1 in enummap or s2 in enummap:
                    print("ERROR: Duplicate enum %s:\n\t%s = %s @ %s:%u\n\t%s" % (
                        "names" if s1 in enummap else "values",
                        s1, entry.value, entry.origin_file, entry.origin_line,
                        enummap.get(s1) or enummap.get(s2)))
                    return True
                enummap[s1] = enummap[s2] = "%s.%s = %s @ %s:%u" % (enum.name, entry.name, entry.value, entry.origin_file, entry.origin_line)

    return False

def check_missing_enum(xml):
    '''check for enum fields pointing to invalid enums'''

    all_enums = set()
    for x in xml:
        for enum in x.enum:
            all_enums.add(enum.name)
    for x in xml:
        for m in x.message:
            for f in m.fields:
                if f.enum and f.enum not in all_enums:
                    print('Enum %s in %s.%s does not exist' % (f.enum, m.name, f.name))
                    return True
    return False



def total_msgs(xml):
    '''count total number of msgs'''
    count = 0
    for x in xml:
        count += len(x.message)
    return count

def mkdir_p(dir):
    try:
        os.makedirs(dir)
    except OSError as exc:
        if exc.errno != errno.EEXIST:
            raise

# check version consistent
# add test.xml
# finish test suite
# printf style error macro, if defined call errors
