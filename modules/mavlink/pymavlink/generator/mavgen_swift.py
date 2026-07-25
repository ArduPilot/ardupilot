#!/usr/bin/env python3
"""
Parse a MAVLink protocol XML file and generate Swift implementation

Copyright Max Odnovolyk 2015
Released under GNU GPL version 3 or later
"""
import os
from . import mavparse, mavtemplate

abbreviations = ["MAV", "PX4", "UDB", "PPZ", "PIXHAWK", "SLUGS", "FP", "ASLUAV", "VTOL", "ROI", "UART", "UDP", "IMU", "IMU2", "3D", "RC", "GPS", "GPS1", "GPS2", "NED", "RTK", "ADSB"]
swift_keywords = ["associatedtype", "class", "deinit", "enum", "extension", "fileprivate", "func", "import", "init", "inout", "internal", "let", "open", "operator", "private", "protocol",
                  "public", "static", "struct", "subscript", "typealias", "var", "break" "case", "continue", "default", "defer", "do", "else", "fallthrough", "for", "guard", "if", "in", "repeat", "return", "switch",
                  "where", "while", "Any", "catch", "false", "is", "nil", "rethrows", "super", "self", "Self", "throw", "throws", "true", "try"]
swift_types = {'char' : ("String", '"\\0"', "string(at: %u, length: %u)", "set(%s, at: %u, length: %u)"),
               'uint8_t' : ("UInt8", 0, "number(at: %u)", "set(%s, at: %u)"),
               'int8_t' : ("Int8", 0, "number(at: %u)", "set(%s, at: %u)"),
               'uint16_t' : ("UInt16", 0, "number(at: %u)", "set(%s, at: %u)"),
               'int16_t' : ("Int16", 0, "number(at: %u)", "set(%s, at: %u)"),
               'uint32_t' : ("UInt32", 0, "number(at: %u)", "set(%s, at: %u)"),
               'int32_t' : ("Int32", 0, "number(at: %u)", "set(%s, at: %u)"),
               'uint64_t' : ("UInt64", 0, "number(at: %u)", "set(%s, at: %u)"),
               'int64_t' : ("Int64", 0, "number(at: %u)", "set(%s, at: %u)"),
               'float' : ("Float", 0, "number(at: %u)", "set(%s, at: %u)"),
               'double' : ("Double", 0, "number(at: %u)", "set(%s, at: %u)"),
               'uint8_t_mavlink_version' : ("UInt8", 0, "number(at: %u)", "set(%s, at: %u)")}

t = mavtemplate.MAVTemplate()

def generate_mavlink(directory, filelist, xml_list, msgs):
    print("Generating MAVLink.swift file")

    mavparse.mkdir_p(directory)
    filename = 'MAVLink.swift'
    filepath = os.path.join(directory, filename)
    outf = open(filepath, "w")
    generate_header(outf, filelist, xml_list, filename)
    append_static_code('MAVLink.swift', outf)
    generate_message_mappings_array(outf, msgs)
    generate_message_lengths_array(outf, msgs)
    generate_message_crc_extra_array(outf, msgs)
    outf.close()

def generate_header(outf, filelist, xml_list, filename):
    """Generate Swift file header with source files list and creation date"""

    t.write(outf, """
//
//  ${FILENAME}
//  MAVLink Protocol Swift Library
//
//  Generated from ${FILELIST} on ${PARSE_TIME} by mavgen_swift.py 
//  https://github.com/modnovolyk/MAVLinkSwift
//

""", {'FILENAME' : filename,
      'FILELIST' : ", ".join(filelist),
      'PARSE_TIME' : xml_list[0].parse_time})

def generate_enums(directory, filelist, xml_list, enums):
    """Iterate through all enums and create Swift equivalents"""

    print("Generating Enumerations")

    for enum in enums:
        if enum.is_a_bitmask:
            continue
        filename = "%s%sEnum.swift" % (enum.swift_name, enum.basename)
        filepath = os.path.join(directory, filename)
        outf = open(filepath, "w")
        generate_header(outf, filelist, xml_list, filename)
        t.write(outf, """
${formatted_description}public enum ${swift_name}: ${raw_value_type} {
${{entry:${formatted_description}\tcase ${swift_name} = ${value}\n}}
}

extension ${swift_name}: Enumeration {
    public static var typeName = "${name}"
    public static var typeDescription = "${entity_description}"
    public static var allMembers = [${all_entities}]
    public static var membersDescriptions = [${entities_info}]
    public static var enumEnd = UInt(${enum_end})
}
""", enum)
        outf.close()

def generate_optionsets(directory, filelist, xml_list, enums):
    """Iterate through all enums and create Swift equivalents"""

    print("Generating Enumerations")

    for enum in enums:
        if not enum.is_a_bitmask:
            continue
        for entry in enum.entry:
            entry.parent_swift_name = enum.swift_name
        filename = "%s%sOptionSet.swift" % (enum.swift_name, enum.basename)
        filepath = os.path.join(directory, filename)
        outf = open(filepath, "w")
        generate_header(outf, filelist, xml_list, filename)
        t.write(outf, """
${formatted_description}public struct ${swift_name}: OptionSet {
\tpublic let rawValue: ${raw_value_type}

\tpublic init(rawValue: ${raw_value_type}) {
\t\tself.rawValue = rawValue
\t}
${{entry:${formatted_description}\tpublic static let ${swift_name} = ${parent_swift_name}(rawValue: ${value})\n}}
}

extension ${swift_name}: MAVLinkBitmask {
    public static var typeName = "${name}"
    public static var typeDescription = "${entity_description}"
    public static var allMembers = [${all_entities}]
    public static var membersDescriptions = [${entities_info}]
    public static var enumEnd = UInt(${enum_end})
}
""", enum)
        outf.close()

def get_enum_raw_type(enum, msgs):
    """Search appropriate raw type for enums in messages fields"""

    for msg in msgs:
        for field in msg.fields:
            if  field.enum == enum.name:
                return swift_types[field.type][0]
    return "Int"

def generate_messages(directory, filelist, xml_list, msgs):
    """Generate Swift structs to represent all MAVLink messages"""

    print("Generating Messages")

    for msg in msgs:
        filename = "%s%sMsg.swift" % (msg.swift_name, msg.basename)
        filepath = os.path.join(directory, filename)
        outf = open(filepath, "w")
        generate_header(outf, filelist, xml_list, filename)
        t.write(outf, """
import Foundation

${formatted_description}public struct ${swift_name} {
${{fields:${formatted_description}\tpublic let ${swift_name}: ${return_type}\n}}
}

extension ${swift_name}: Message {
    public static let id = UInt8(${id})
    public static var typeName = "${name}"
    public static var typeDescription = "${message_description}"
    public static var fieldDefinitions: [FieldDefinition] = [${fields_info}]

    public init(data: Data) throws {
${{ordered_fields:\t\t${init_accessor} = ${initial_value}\n}}
    }

    public func pack() throws -> Data {
        var payload = Data(count: ${wire_length})
${{ordered_fields:\t\ttry payload.${payload_setter}\n}}
        return payload
    }
}
""", msg)
        outf.close()

def append_static_code(filename, outf):
    """Open and copy static code from specified file"""

    basepath = os.path.dirname(os.path.realpath(__file__))
    filepath = os.path.join(basepath, 'swift/%s' % filename)
    
    print("Appending content of %s" % filename)
    
    with open(filepath) as inf:
        for line in inf:
            outf.write(line) 

def generate_message_mappings_array(outf, msgs):
    """Create array for mapping message Ids to proper structs"""

    classes = []
    for msg in msgs:
        classes.append("%u: %s.self" % (msg.id, msg.swift_name))
    t.write(outf, """

/// Array for mapping message id to proper struct
private let messageIdToClass: [UInt8: Message.Type] = [${ARRAY_CONTENT}]
""", {'ARRAY_CONTENT' : ", ".join(classes)})

def generate_message_lengths_array(outf, msgs):
    """Create array with message lengths to validate known message lengths"""

    # form message lengths array
    lengths = []
    for msg in msgs:
        lengths.append("%u: %u" % (msg.id, msg.wire_length))

    t.write(outf, """

/// Message lengths array for known messages length validation
private let messageLengths: [UInt8: UInt8] = [${ARRAY_CONTENT}]
""", {'ARRAY_CONTENT' : ", ".join(lengths)})

def generate_message_crc_extra_array(outf, msgs):
    """Add array with CRC extra values to detect incompatible XML changes"""

    crcs = []
    for msg in msgs:
        crcs.append("%u: %u" % (msg.id, msg.crc_extra))

    t.write(outf, """

/// Message CRSs extra for detection incompatible XML changes
private let messageCRCsExtra: [UInt8: UInt8] = [${ARRAY_CONTENT}]
""", {'ARRAY_CONTENT' : ", ".join(crcs)})

def camel_case_from_underscores(string):
    """Generate a CamelCase string from an underscore_string"""

    components = string.split('_')
    string = ''

    for component in components:
        if component in abbreviations:
            string += component
        else:
            string += component[0].upper() + component[1:].lower()

    return string

def lower_camel_case_from_underscores(string):
    """Generate a lower-cased camelCase string from an underscore_string"""

    components = string.split('_')
    string = components[0].lower()
    for component in components[1:]:
        string += component[0].upper() + component[1:].lower()

    return string

def contains_a_bitmask(enums, enumName):
    for enum in enums:
        if enum.name == enumName:
            return enum.is_a_bitmask
    return False

def enum_is_a_bitmask(enum):
    values = []
    for entry in enum.entry:
        values.append(entry.value)
    values.sort()
    for i, value in enumerate(values):
        if 2 ** i != value:
            return False
    return True


def generate_enums_type_info(enums, msgs):
    """Add camel case swift names for enums an entries, descriptions and sort enums alphabetically"""

    for enum in enums:
        enum.swift_name = camel_case_from_underscores(enum.name)
        enum.raw_value_type = get_enum_raw_type(enum, msgs)

        enum.formatted_description = ""
        if enum.description:
            enum.description = " ".join(enum.description.split())
            enum.formatted_description = "/// %s\n" % enum.description

        for index, entry in enumerate(enum.entry):
            if entry.name.endswith("_ENUM_END"):
                enum.enum_end = entry.value
                del enum.entry[index]

        all_entities = []
        entities_info = []

        for entry in enum.entry:
            name = entry.name.replace(enum.name + '_', '')
            """Ensure that enums entry name does not start from digit"""
            if name[0].isdigit():
                name = "MAV_" + name
            entry.swift_name = lower_camel_case_from_underscores(name)

            """Ensure that enums entry name does not match any swift keyword"""
            if entry.swift_name in swift_keywords:
                entry.swift_name = lower_camel_case_from_underscores("MAV_" + name)

            entry.formatted_description = ""
            if entry.description:
                entry.description = " ".join(entry.description.split())
                entry.formatted_description = "\n\t/// " + entry.description + "\n"

            all_entities.append(entry.swift_name)
            entities_info.append('("%s", "%s")' % (entry.name, entry.description.replace('"','\\"')))

        enum.all_entities = ", ".join(all_entities)
        enum.entities_info = ", ".join(entities_info)
        enum.entity_description = enum.description.replace('"','\\"')
        enum.is_a_bitmask = enum_is_a_bitmask(enum)

    enums.sort(key = lambda enum : enum.swift_name)

def generate_messages_type_info(msgs, enums):
    """Add proper formatted variable names, initializers and type names to use in templates"""

    for msg in msgs:
        msg.swift_name = camel_case_from_underscores(msg.name)

        msg.formatted_description = ""
        if msg.description:
            msg.description = " ".join(msg.description.split())
            msg.formatted_description = "/// %s\n" % " ".join(msg.description.split())
        msg.message_description = msg.description.replace('"','\\"')

        for field in msg.ordered_fields:
            field.swift_name = lower_camel_case_from_underscores(field.name)
            field.init_accessor = field.swift_name if field.swift_name != "data" else "self.%s" % field.swift_name
            field.pack_accessor = field.swift_name if field.swift_name != "payload" else "self.%s" % field.swift_name
            field.return_type = swift_types[field.type][0]
            
            # configure fields initializers
            if field.enum:
                # handle enums


                field.return_type = camel_case_from_underscores(field.enum)
                if contains_a_bitmask(enums, field.enum):
                    field.initial_value = "try data.bitmask(at: %u)" % field.wire_offset
                else:
                    field.initial_value = "try data.enumeration(at: %u)" % field.wire_offset
                field.payload_setter = "set(%s, at: %u)" % (field.pack_accessor, field.wire_offset)
            elif field.array_length > 0:
                if field.return_type == "String":
                    # handle strings
                    field.initial_value = "try data." + swift_types[field.type][2] % (field.wire_offset, field.array_length)
                    field.payload_setter = swift_types[field.type][3] % (field.pack_accessor, field.wire_offset, field.array_length)
                else:
                    # other array types
                    field.return_type = "[%s]" % field.return_type
                    field.initial_value = "try data.array(at: %u, capacity: %u)" % (field.wire_offset, field.array_length)
                    field.payload_setter = "set(%s, at: %u, capacity: %u)" % (field.pack_accessor, field.wire_offset, field.array_length)
            else:
                # simple type field
                field.initial_value = "try data." + swift_types[field.type][2] % field.wire_offset
                field.payload_setter = swift_types[field.type][3] % (field.pack_accessor, field.wire_offset)

            field.formatted_description = ""
            if field.description:
                field.description = " ".join(field.description.split())
                field.formatted_description = "\n\t/// " + field.description + "\n"
         
        fields_info = ['("%s", %u, "%s", %u, "%s")' % (field.swift_name, field.wire_offset, field.return_type, field.array_length, field.description.replace('"','\\"')) for field in msg.fields]
        msg.fields_info = ", ".join(fields_info)

    msgs.sort(key = lambda msg : msg.id)

def generate(basename, xml_list):
    """Generate complete MAVLink Swift implementation"""

    msgs = []
    enums = []
    filelist = []

    for xml in xml_list:
        for msg in xml.message: msg.basename = xml.basename.title()
        for enum in xml.enum: enum.basename = xml.basename.title()

        msgs.extend(xml.message)
        enums.extend(xml.enum)
        filelist.append(os.path.basename(xml.filename))
    
    generate_enums_type_info(enums, msgs)
    generate_messages_type_info(msgs, enums)

    generate_mavlink(basename, filelist, xml_list, msgs)
    generate_enums(basename, filelist, xml_list, enums)
    generate_optionsets(basename, filelist, xml_list, enums)
    generate_messages(basename, filelist, xml_list, msgs)