#!/usr/bin/env python3
'''
parse a MAVLink protocol XML file and generate a python implementation

Copyright Andrii Fil root.fi36@gmail.com 2022
Based on mavgen_python.py
Released under GNU GPL version 3 or later
'''
import os
import os.path
import math
import shutil
from operator import attrgetter
from . import mavparse

v1 = "MAVLink.V1"
v1_file = "mavlink-v1-"
v2 = "MAVLink.V2"
v2_file = "mavlink-v2-"

NAN = "0"

copy = """--  Copyright Fil Andrii root.fi36@gmail.com 2022-2025

"""

reserved_words = (
    'abort', 'else', 'new', 'return', 'abs', 'elsif', 'not', 'reverse', 'abstract', 'end', 'null',
    'accept', 'entry', 'select', 'access', 'exception', 'of', 'separate', 'aliased', 'exit', 'or',
    'subtype', 'all', 'others', 'synchronized', 'and', 'for', 'out', 'array', 'function', 'overriding',
    'tagged', 'at', 'task', 'generic', 'package', 'terminate', 'begin', 'goto', 'pragma', 'then',
    'body', 'private', 'type', 'if', 'procedure', 'case', 'in', 'protected', 'until', 'constant',
    'interface', 'use', 'is', 'raise', 'declare', 'range', 'when', 'delay', 'limited', 'record',
    'while', 'delta', 'loop', 'rem', 'with', 'digits', 'renames', 'do', 'mod', 'requeue', 'xor')

reserved_fields = ('sequence', 'system_id', 'component_id', 'message_id')

# mavlink developers cannot understand that there are not only c-like languages :(
try:
    math.log2(2)

    def is_position(val):
        return math.log2(val).is_integer()
except:
    def is_position(val):
        return 2 ** int(math.log(val, 2)) == val

def normalize_name(name, suffix):
    if name.lower() in reserved_words:
        return name + "_" + suffix
    if not name[0].isalpha():
        return "A_" + name
    return name

def normalize_field_name(name):
    n = normalize_name(name, "Field")
    if n.lower() in reserved_fields:
        return n + "_Field"
    return n

def normalize_message_name(name):
    return normalize_name(name, "Message").title()

def normalize_enum_name(name):
    return normalize_name(name, "Enum").title()

def normalize_entry_name(name):
    return normalize_name(name, "Entry")

field_types = {
    'uint8_t' : 'Interfaces.Unsigned_8',
    'uint16_t' : 'Interfaces.Unsigned_16',
    'uint32_t' : 'Interfaces.Unsigned_32',
    'uint64_t' : 'Interfaces.Unsigned_64',
    'int8_t' : 'Interfaces.Integer_8',
    'int16_t' : 'Interfaces.Integer_16',
    'int32_t' : 'Interfaces.Integer_32',
    'int64_t' : 'Interfaces.Integer_64',
    'float' : 'Raw_Float',
    'double' : 'Raw_Long_Float'}

field_array_types = {
    'uint8_t' : 'Unsigned_8_Array',
    'uint16_t' : 'Unsigned_16_Array',
    'uint32_t' : 'Unsigned_32_Array',
    'uint64_t' : 'Unsigned_64_Array',
    'int8_t' : 'Integer_8_Array',
    'int16_t' : 'Integer_16_Array',
    'int32_t' : 'Integer_32_Array',
    'int64_t' : 'Integer_64_Array',
    'float' : 'Short_Float_Array',
    'double' : 'Long_Float_Array'}

array_to_float = {
    'Short_Float_Array' : 'Raw_Float',
    'Long_Float_Array' : 'Raw_Long_Float'}

field_values = {
    'uint8_t' : '1',
    'uint16_t' : '2',
    'uint32_t' : '3',
    'uint64_t' : '4',
    'int8_t' : '5',
    'int16_t' : '6',
    'int32_t' : '7',
    'int64_t' : '8',
    'float' : 'To_Raw (9.9)',
    'double' : 'To_Raw (10.10)'}

field_array_values = {
    'uint8_t' : '[others => 1]',
    'uint16_t' : '[others => 2]',
    'uint32_t' : '[others => 3]',
    'uint64_t' : '[others => 4]',
    'int8_t' : '[others => 5]',
    'int16_t' : '[others => 6]',
    'int32_t' : '[others => 7]',
    'int64_t' : '[others => 8]',
    'float' : '[others => To_Raw (9.9)]',
    'double' : '[others => To_Raw (10.10)]'}

# Default invalid values for fields
invalid = {
    'UINT8_MAX'  : "Interfaces.Unsigned_8'Last",
    'UINT16_MAX' : "Interfaces.Unsigned_16'Last",
    'UINT32_MAX' : "Interfaces.Unsigned_32'Last",
    'INT8_MAX'   : "Interfaces.Integer_8'Last",
    'INT16_MAX'  : "Interfaces.Integer_16'Last",
    'INT32_MAX'  : "Interfaces.Integer_32'Last"
}

float_types_conversions = {
    'Raw_Float' : 'Interfaces.IEEE_Float_32',
    'Raw_Long_Float' : 'Interfaces.IEEE_Float_64',
    'Interfaces.IEEE_Float_32' : 'Interfaces.IEEE_Float_32'}

GEN = """-------------------------------------------
--  DO NOT EDIT. This file is generated. --
-------------------------------------------\n\n"""

# format text as a comment with lines < 79
def format_comment(text, tab):
    sp = text.split()
    if len(sp) < 1: return ""

    s = "".ljust(tab) + "--  "
    start = len (s)
    l = start
    for i in sp:
        if l + len(i) + 1 > 79:
            s += "\n".ljust(tab + 1) + "--  "
            l = start
      
        s += i + " "
        l = l + len(i) + 1
    s += "\n"
    return s

#return information about deprication as a comment
def get_deprecated(deprecated, tab):
    s  = "".ljust(tab) + "------------\n"
    s += "".ljust(tab) + "--  DEPRECATED SINCE: %s REPLACED BY: %s\n" % (deprecated.since, deprecated.replaced_by)
    s += format_comment (deprecated.explanation, tab)
    s += "".ljust(tab) + "------------\n\n"
    return s

def pkg_name(x):
    name = os.path.splitext(os.path.basename(x.filename))[0].lower().title()
    if name == "All":
        return "For_All"
    else:
        return name

def file_name(x):
    name = os.path.splitext(os.path.basename(x.filename))[0].lower()
    if name == "all":
        return "for_all"
    else:
        return name

# Returns package filename and name for the message
def get_pakage_name(x, m, directory, ver, f_ver):
    name = file_name(x)
    pkg_name = normalize_message_name(m.name)

    if pkg_name[-1] == 's': pkg_name += 'es'
    else: pkg_name += 's'

    f_name = os.path.join(directory, "mavlink-%s-%s-%s" % (f_ver, name, pkg_name.lower()))
    p_name = "%s.%s.%s" % (ver, name.title(), pkg_name)
    return f_name, p_name

# Returns package name where the tp type declared
def find_pkg (xml, tp, ver):
    pkg = ""
    for x in xml:
        for t in x.enum:
            if normalize_enum_name(t.name) == tp:
                pkg = pkg_name(x)

    return "%s.%s.Types.%s" % (ver, pkg, tp)

# Generate record type
def repr_bitmask(enum, size):
    enum_name = normalize_enum_name(enum.name)
    s = "   type %s is record\n" % enum_name
    names = [i.name for i in enum.entry if not i.end_marker]
    common_prefix = os.path.commonprefix(names) if len(names) > 1 else ""

    max_len = 0
    for i in enum.entry:
        if i.end_marker:
            name = "Reserved_%i" % (size * 8 - 1)
        elif i.value == 0 or not is_position(i.value):
            continue
        else:
            name = i.name[len(common_prefix):] # strip common prefix
        name_len = len(normalize_entry_name(name))
        if name_len > max_len:
            max_len = name_len

    for i in enum.entry:
        if i.end_marker:
            for j in range(math.ceil(math.log(i.value, 2)), size * 8):
                name = "Reserved_%i" % j
                s += "      %s : Boolean := False;\n" % name.ljust(max_len)
            break
        elif i.value == 0:
            continue
        elif not is_position(i.value):
            print("%s ignored because the composite value!" % i.name)
            continue
        name = i.name[len(common_prefix):] # strip common prefix
        name = normalize_entry_name(name).title()
        s += "      %s : Boolean := False;\n" % name.ljust(max_len)

    s += "   end record with Size => %i;\n" % (size * 8)
    s += format_comment (enum.description, 3) + '\n'

    parts = ""
    s += "   for %s use record\n" % enum_name
    for i in enum.entry:
        if i.end_marker:
            for j in range(math.ceil(math.log(i.value, 2)), size * 8):
                name = "Reserved_%i" % j
                s += "      %s at 0 range %i .. %i;\n" % (name.ljust(max_len), j, j)
            break
        elif i.value == 0 or not is_position(i.value):
            continue
        name = i.name[len(common_prefix):] # strip common prefix
        image = name
        name = normalize_entry_name(name).title()
        pos = int(math.log(i.value, 2))
        s += "      %s at 0 range %i .. %i;\n" % (name.ljust(max_len), pos, pos)
        parts += '\n      & (if V.%s then "%s " else "")' % (name, image)
    s += "   end record;\n"
    s += """
   function Image (V : {}) return String is
     ("["{}
      & "]");
""".format (enum_name, parts)
    return s

# Generate enumeration type
def repr_enum(enum, size, types_files):
    enum_name = normalize_enum_name(enum.name)
    s = "   type %s is new Interfaces.Unsigned_%i;\n" % (enum_name, size * 8)
    if enum.deprecated:
        s += "   pragma Obsolescent (%s);\n" % enum_name
        s += get_deprecated(enum.deprecated, 3)
    s += format_comment (enum.description, 3) + '\n'

    names = [i.name for i in enum.entry if not i.end_marker]
    common_prefix = os.path.commonprefix(names) if len(names) > 1 else ""
    first = ""
    last = ""
    last_value = 0
    predicate = ""
    choises = ""

    for i in enum.entry:
        if i.end_marker:
            break
        name = i.name[len(common_prefix):] # strip common prefix
        if name in types_files: name = i.name
        name = normalize_entry_name(name).title()

        if first == "":
            first = name
            last = name
            last_value = i.value
        elif last_value + 1 == i.value:
            last = name
            last_value = i.value
        else:
            predicate += ("\n       | " if predicate else "")
            predicate += (first if first == last else first + " .. " + last)
            first = name
            last = name
            last_value = i.value

        choises += ("," if choises else "")
        choises += '\n        when %s => "%s"' % (name, name)
        fun = "   function %s return %s is (%i)\n     with Static;\n" % (name, enum_name, i.value)
        s += fun
        if i.deprecated:
            s += "   pragma Obsolescent (%s);\n" % name
            s += get_deprecated(i.deprecated, 3)
        s += format_comment (i.description, 3) + '\n'

    predicate += ("\n       | " if predicate else "")
    predicate += (first if first == last else first + " .. " + last)
    subtype = """   subtype {0}_Well_Known is {0}
     with Static_Predicate => {0}_Well_Known in
       {1};

   function Well_Known_Image
     (Value : {0}_Well_Known) return String is
       (case Value is{2});

   function Image (Value : {0}) return String is
     (if Value in {0}_Well_Known
      then Well_Known_Image (Value) else "Unknown:" & Value'Image);
"""
    s += subtype.format(enum_name, predicate, choises)

    return s

# Create file for the types package
def create_dialect_spec (directory, file_prefix, ver, x, xml, header=None):
    def process_ids(ids, id_name, x):
        for m in x.message:
            if int(m.id) not in ids: ids.append(int(m.id))
            if int(m.id) not in id_name: id_name[int(m.id)] = normalize_message_name(m.name)

    def process_dialect(ids, id_name, x):
        process_ids(ids, id_name, x)
        for i in x.include:
            for c in xml:
                if i in c.filename:
                    process_dialect(ids, id_name, c)

    dialect = file_name(x)
    print("Generate " + dialect.title())

    # Create file
    spec = open(os.path.join(directory, file_prefix + dialect + ".ads"), "w")
    spec.write(GEN)
    if header: spec.write(header)
    spec.write("pragma Ada_2022;\n\n")

    spec.write("package %s.%s is\n\n" % (ver, dialect.title()))
    spec.write("   pragma Pure;\n\n")

    ids = []
    id_name = {}
    process_dialect(ids, id_name, x)

    if len(ids) > 0:
        n_len = 0
        for i in id_name:
            n_len = max(len(id_name[i]), n_len)

        n_len += 3
        for i in sorted(id_name):
            spec.write("   %s : constant Msg_Id := %i;\n" % ((id_name[i] + '_Id').ljust(n_len), i))
        spec.write("\n")

        ids.sort()
        prev = -1

        n_len = len(str(ids[-1]))
        spec.write("   subtype Valid_Msg_Id is Msg_Id\n     with Static_Predicate => Valid_Msg_Id in\n")
        for id in ids:
            if prev == -1:
                spec.write("       %s .. " % str(id).ljust(n_len))
            else:
                if prev + 1 != id:
                    spec.write("%s |\n       %s .. " % (str(prev).ljust(n_len), str(id).ljust(n_len)))
            prev = id

        spec.write("%s;\n\n" % str(ids[-1]).ljust(n_len))
        spec.write("   function Image (Id : Valid_Msg_Id) return String is\n")
        spec.write("     (case Id is")
        for id in ids:
            if id != ids[0]:
                spec.write(",")
            spec.write('\n         when %s => "%s"' % (str(id).ljust(n_len), id_name[id]))
        spec.write(");\n\n")
    spec.write("end %s.%s;\n" % (ver, dialect.title()))

# Create file for the types package
def create_types_spec (directory, file_prefix, ver, x, header=None):
    dialect = file_name(x)

    # Create file
    spec = open(os.path.join(directory, file_prefix + dialect + "-types.ads"), "w")
    spec.write(GEN)
    if header: spec.write(header)
    spec.write("pragma Ada_2022;\n\n")

    spec.write("package %s.%s.Types is\n\n" % (ver, dialect.title()))
    spec.write("   pragma Pure;\n\n")
    return spec

# Create file for the message spec
def create_message_pkg (x, f_name, p_name, ver, m, types_files, header=None):
    spec = open(f_name + ".ads", "w")
    spec.write(GEN)
    if header: spec.write(header)
    if m.deprecated:
        spec.write(get_deprecated(m.deprecated, 0))
    spec.write(format_comment (m.description, 0))
    spec.write("\npragma Ada_2022;\n")

    # indirect includes
    included = []
    f_types = [field.enum for field in m.fields if field.enum and not field.array_length]
    for t in f_types:
        if t in types_files and types_files[t] not in included and types_files[t].filename not in x.include:
            if len(included) == 0: spec.write("\n")
            included.append(types_files[t])
            inc = os.path.splitext(os.path.basename(types_files[t].filename))[0].title()
            spec.write("with %s.%s.Types; use %s.%s.Types;\n" % (ver, inc, ver, inc))

    spec.write("\npackage %s is\n\n" % p_name)
    spec.write("   pragma Pure;\n\n")
    return spec

# Create file for the message body
def create_message_body (f_name, p_name):
    body = open(f_name + ".adb", "w")
    body.write(GEN)
    body.write("package body %s is\n" % p_name)
    return body

# Prepare information about types
def calculate_types_size(xml):
    types = []
    types_size = {}
    types_files = {}
    msgs = []

    for x in xml:
        msgs.extend(x.message)
        types.extend(x.enum)
        for enum in x.enum:
            types_files |= {enum.name: x}

    msgs.sort(key=attrgetter('id'))

    types_size = {t.name: None for t in types}
    for m in msgs:
        for f in m.fields:
            if f.enum:
                if types_size[f.enum] is None:
                    types_size[f.enum] = f.type_length
                else:
                    assert types_size[f.enum] == f.type_length, "Different size for one enum"
    return types, types_size, types_files

# Generate v1 types
def generate_v1_types(directory, x, types_files, types_size):
    dialect = pkg_name(x)
    outf = create_types_spec (directory, v1_file, v1, x, copy)

    for t in x.enum:
        size = types_size[t.name]

        if size is None:
            if t.bitmask: size = calculate_bitmask_size(t)
            else: size = calculate_enum_size(t)

        if size is not None:
            if t.bitmask:
                outf.write(repr_bitmask(t, size))
            else:
                outf.write(repr_enum(t, size, types_files))
            outf.write("\n")
    outf.write("end %s.%s.Types;\n" % (v1, dialect.title()))

# return default value for record field
def get_default(value, type_name, array_to_float, types):
    if value[0] == '[':
        if type_name in array_to_float:
            return "[others => %s]" % get_default(value[1:-1], array_to_float[type_name], array_to_float, types)
        else:
            return "[others => %s]" % get_default(value[1:-1], type_name, array_to_float, types)

    elif value[:2] == '0x':
        res = "16#%s#" % value[2:]
        if type_name.find("IEEE") != -1: return "%s (%s)" % (type_name, res)
        if type_name.find("Float") != -1: return "To_Raw (%s (%s))" % (float_types_conversions[type_name], res)
        return res

    else:
        if value in invalid:
            res = invalid[value]
            if type_name.find("IEEE") != -1: return "%s (%s)" % (type_name, res)
            if type_name.find("Float") != -1: return "To_Raw (%s (%s))" % (float_types_conversions[type_name], res)
            return res

        elif value[:3].upper() == "NAN":
            return get_default(NAN, type_name, array_to_float, types)

        else:
            if type_name.find("IEEE") != -1:
                if value.find(".") == -1:
                    return "%s.0" % value
                else:
                    return value

            elif type_name.find("Float") != -1:
                if value.find(".") == -1:
                    return "To_Raw (%s.0)" % value
                else:
                    return "To_Raw (%s)" % value

            elif type_name == "String":
                return "Character'Val (%s)" % value

            else:
                for t in types:
                    if normalize_enum_name(t.name) == type_name and not t.bitmask:
                        values = [i.name for i in t.entry if not i.end_marker]
                        if value in values:
                            common_prefix = os.path.commonprefix(values) if len(values) > 1 else ""
                            return normalize_entry_name(value[len(common_prefix):]).title()

                return value

def generate_message_representation(msg, spec, max_len):
    name = normalize_message_name(msg.name)
    offset = 0
    max_offset = 0
    for field in msg.ordered_fields:
        field_size = (field.array_length or 1) * field.type_length * 8
        pp = "%i" % offset
        max_offset = max(max_offset, len(pp))
        offset += field_size / 8

    offset = 0
    spec.write("   for %s use record\n" % name)
    for field in msg.ordered_fields:
        field_name = normalize_field_name(field.name).title()
        field_size = (field.array_length or 1) * field.type_length * 8
        off = "%i" % offset
        spec.write("      %s at %s range 0 .. %i;\n" % (field_name.ljust(max_len), off.ljust(max_offset), field_size - 1))
        offset += field_size / 8
    spec.write("   end record;\n\n")

def generate_message_methods(name, rev, spec, is_v1):
    if is_v1:
        mode = "in out "
    else:
        mode = ""

    out = """   procedure Encode
     (Message : {0};
      Connect : in out {1}.Out_Connection;
      Buffer  : out Data_Buffer;
      Last    : out Positive);
   --  Put the message in the buffer ready for send

   procedure Encode
     (Message : {0};
      Connect : in out {1}.Connection;
      Buffer  : out Data_Buffer;
      Last    : out Positive);

   procedure Decode
     (Message   : out {0};
      Connect   : {2}{1}.Connection;
      CRC_Valid : out Boolean);
   --  Get the message from the Connect and delete it
   --  from the Connect's buffer. CRC_Valid is set to
   --  True if x25crc is valid for the message.

   procedure Decode
     (Message : out {0};
      Connect : {1}.Connection);
   --  Same as Above but does not check CRC

   procedure Decode
     (Message   : out {0};
      Connect   : {2}{1}.In_Connection;
      CRC_Valid : out Boolean);
   --  Get the message from the Connect and delete it
   --  from the Connect's buffer. CRC_Valid is set to
   --  True if x25crc is valid for the message.

   procedure Decode
     (Message : out {0};
      Connect : {1}.In_Connection);
   --  Same as Above but does not check CRC

   function Check_CRC
     (Connect : {2}{1}.Connection)
      return Boolean with Inline;
   --  Returns True if CRC is valid

   function Check_CRC
     (Connect : {2}{1}.In_Connection)
      return Boolean with Inline;
   --  Returns True if CRC is valid

"""
    spec.write(out.format(name, rev, mode))

def generate_message_methods_V2(name, spec):
    out = """   procedure Encode
     (Message : {0};
      Connect : in out {1}.Connection;
      Sign    : in out Signature;
      Buffer  : out Data_Buffer;
      Last    : out Positive);

   procedure Encode
     (Message : {0};
      Connect : in out {1}.Out_Connection;
      Sign    : in out Signature;
      Buffer  : out Data_Buffer;
      Last    : out Positive);

"""
    spec.write(out.format(name, v2))

# Generate message body
def generate_message_body_encode (name, rev, extra, body, is_v1, connection):
    out = """
   ------------
   -- Encode --
   ------------

   procedure Encode
     (Message : {0};
      Connect : in out {1}.{2};
      Buffer  : out Data_Buffer;
      Last    : out Positive)
   is
      Local : Data_Buffer (1 .. {0}'Value_Size / 8)
        with Import, Address => Message'Address,
        Convention => Ada;

   begin
      Last := Buffer'First +
        Packet_Payload_First +
        ({0}'Value_Size / 8) - 1;
      Buffer (Buffer'First + Packet_Payload_First .. Last) := Local;
      Encode (Connect, {0}_Id, {3}, Buffer, Last);
   end Encode;
"""
    body.write(out.format(name, rev, connection, extra))

    if not is_v1:
        out = """
   ------------
   -- Encode --
   ------------

   procedure Encode
     (Message : {0};
      Connect : in out {1}.{2};
      Sign    : in out {1}.Signature;
      Buffer  : out Data_Buffer;
      Last    : out Positive)
   is
      Local : Data_Buffer (1 .. {0}'Value_Size / 8)
        with Import, Address => Message'Address,
        Convention => Ada;

   begin
      Last := Buffer'First +
        Packet_Payload_First +
        ({0}'Value_Size / 8) - 1;
      Buffer (Buffer'First + Packet_Payload_First .. Last) := Local;
      Encode (Connect, {0}_Id, {3}, Sign, Buffer, Last);
   end Encode;
"""
        body.write(out.format(name, rev, connection, extra))

def generate_message_body_decode(name, rev, extra, body, is_v1, connection):
    if is_v1:
        mode = "in out "
    else:
        mode = ""

    body.write("""
   ------------
   -- Decode --
   ------------

   procedure Decode
     (Message   : out %s;
      Connect   : %s%s.%s;
      CRC_Valid : out Boolean)
   is
""" % (name, mode, rev, connection))

    if is_v1:
        body.write("""      Buf : Data_Buffer
        (1 .. %s'Size / 8) := [others => 0]
        with Address => Message'Address,
        Convention   => Ada;
   begin
      pragma Assert
        (Get_Msg_Len (Connect) =
             Unsigned_8 (Integer (%s'Value_Size) / 8));
      Get_Message_Data (Connect, Buf);
      CRC_Valid := Check_CRC (Connect);
""" % (name, name))
    else:
        body.write("""      Data : Data_Buffer (1 .. %s'Value_Size / 8);
      Last : Natural;
      Buf  : Data_Buffer
        (1 .. %s'Size / 8) := [others => 0]
        with Address => Message'Address,
        Convention   => Ada;
   begin
      Get_Message_Data (Connect, Data, Last);
      Buf (1 .. Last) := Data (1 .. Last);
      CRC_Valid := Check_CRC (Connect);
""" % (name, name))
    body.write("""   end Decode;

   ------------
   -- Decode --
   ------------

   procedure Decode
     (Message : out %s;
      Connect : %s.%s)
   is
""" % (name, rev, connection))
    if is_v1:
        body.write("""      Buf : Data_Buffer
        (1 .. %s'Size / 8) := [others => 0]
        with Address => Message'Address,
        Convention   => Ada;
   begin
      pragma Assert
        (Get_Msg_Len (Connect) =
             Unsigned_8 (Integer (%s'Value_Size / 8)));
      Get_Message_Data (Connect, Buf);
""" % (name, name))
    else:
        body.write("""      Data : Data_Buffer (1 .. %s'Value_Size / 8);
      Last : Natural;
      Buf  : Data_Buffer
        (1 .. %s'Size / 8) := [others => 0]
        with Address => Message'Address,
        Convention   => Ada;
   begin
      Get_Message_Data (Connect, Data, Last);
      Buf (1 .. Last) := Data (1 .. Last);
""" % (name, name))
    body.write("   end Decode;\n")

def generate_message_body_crc(rev, extra, body, connection, is_v1):
    if is_v1:
        mode = "in out "
    else:
        mode = ""

    body.write("""
   ---------------
   -- Check_CRC --
   ---------------

   function Check_CRC
     (Connect : %s%s.%s) return Boolean is
   begin
      return Is_CRC_Valid (Connect, %s);
   end Check_CRC;
""" % (mode, rev, connection, extra))

def generate_message_body(name, rev, extra, body, is_v1):
    generate_message_body_encode(name, rev, extra, body, is_v1, "Connection")
    generate_message_body_encode(name, rev, extra, body, is_v1, "Out_Connection")
    generate_message_body_decode(name, rev, extra, body, is_v1, "Connection")
    generate_message_body_decode(name, rev, extra, body, is_v1, "In_Connection")
    generate_message_body_crc(rev, extra, body, "Connection", is_v1)
    generate_message_body_crc(rev, extra, body, "In_Connection", is_v1)

# Generate message for the V1
def generate_v1_message(msg, spec, body, xml, types):
    name = normalize_message_name(msg.name)

    max_len = 0
    payload_length = 0
    for field in msg.fields:
        payload_length += (field.array_length or 1) * field.type_length
        field_name_len = len(normalize_field_name(field.name))
        if field_name_len > max_len:
            max_len = field_name_len

    spec.write("   %s_Len : constant Interfaces.Unsigned_8 := %i;\n\n" % (name, payload_length))

    spec.write("   type %s is record\n" % name)
    for field in msg.fields:
        field_name = normalize_field_name(field.name).title()
        spec.write("      %s : " % field_name.ljust(max_len))
        if field.type == 'char':
            tp = "String"
            spec.write("String (1 .. %i)" % field.array_length)
        else:
            if field.array_length:
                tp = field_array_types[field.type]
                spec.write("%s (1 .. %i)" % (tp, field.array_length))
            elif field.enum:
                tp = normalize_enum_name(field.enum)
                if field_name == tp: tp = find_pkg (xml, tp, v1)
                spec.write(tp)
            else:
                tp = field_types[field.type]
                spec.write(tp)

        if field.invalid:
            s = get_default(field.invalid, tp, array_to_float, types)
            if s != "":
                spec.write(" :=\n        %s" % s)

        spec.write(";\n")
        if field.units: spec.write("      --  Units: %s\n" % field.units)
        spec.write(format_comment(field.description, 6))
    spec.write("   end record;\n\n")

    if msg.deprecated:
        spec.write("   pragma Obsolescent (%s);\n\n" % name)

    generate_message_representation(msg, spec, max_len)
    generate_message_methods(name, v1, spec, True)
    generate_message_body(name, v1, str(msg.crc_extra), body, True)

# Generate test project
def generate_test_project(directory, ver):
    dir = os.path.join(directory, 'tests')
    mavparse.mkdir_p(dir)

    f = open(os.path.join(dir, "test.gpr"), "w")
    f.write("""with "../mavlink_%s.gpr";
project Test is

   for Source_Dirs use (".");
   for Object_Dir use "./obj/";
   for Main use ("test.adb");

   package Compiler is
      for Switches ("ada") use
        ("-gnat2022", "-gnata", "-g", "-gnatf", "-gnatwaJ");
   end Compiler;

   package Builder is
      for Switches ("ada") use ("-g");
   end Builder;

   package Linker is
      for Switches ("ada") use ("-g");
   end Linker;

end Test;""" % ver)

# Generate test project for V1
def generate_test_v1(directory, xml, bitmasks):
    dir = os.path.join(directory, 'tests')
    generate_test_project(directory, "v1")

    f = open(os.path.join(dir, "test.adb"), "w")
    f.write("\npragma Warnings (Off); --  prevent `not used`\n")
    for x in xml:
        f.write("with %s.%s.Types;\n" % (v1, file_name(x).title()))
    f.write("pragma Warnings (On);\n\n")

    for x in xml:
        for m in x.message:
            f_name, p_name = get_pakage_name(x, m, directory, v1, "v1")
            f.write("with %s;\n" % p_name)

    f.write("""\nwith Ada.Text_IO;

pragma Warnings (Off); --  prevent "not used"
with MAVLink.Raw_Floats; use MAVLink.Raw_Floats;
with MAVLink.Raw_Long_Floats; use MAVLink.Raw_Long_Floats;
pragma Warnings (On);

use MAVLink.V1;

procedure Test
is
   In_Connect  : MAVLink.V1.Connection;
   Out_Connect : MAVLink.V1.Out_Connection;
   Res         : Boolean;
   Buffer      : Data_Buffer (1 .. MAVLink.V1.Maximum_Buffer_Len);
   Last        : Positive;

begin
   Set_System_Id (In_Connect, 1);
   Set_Component_Id (In_Connect, 1);
   Set_System_Id (Out_Connect, 1);
   Set_Component_Id (Out_Connect, 1);
""")
    for x in xml:
        for m in x.message:
            msg = normalize_message_name(m.name)
            f_name, p_name = get_pakage_name(x, m, directory, v1, "v1")
            f.write("""   declare
      use %s;
      I : %s;
      O : constant %s :=
         (""" % (p_name, msg, msg))
            first = True
            for field in m.fields:
                field_name = normalize_field_name(field.name).title()
                if field.invalid:
                    value = "<>"

                else:
                    if field.type == 'char':
                        value = "[others => 'A']"

                    else:
                        if field.array_length:
                            value = field_array_values[field.type]

                        elif field.enum:
                            if field.enum in bitmasks:
                                value = "<>"

                            else:
                                value = "MAVLink.V1.%s.%s'First" % (
                                    find_package(xml, field.enum),
                                    normalize_enum_name(field.enum))
                        else:
                            value = field_values[field.type]

                if first:
                    first = False
                    f.write("%s => %s" % (field_name, value))
                else:
                    f.write(",\n          %s => %s" % (field_name, value))

            f.write(""");
   begin
      Encode (O, Out_Connect, Buffer, Last);

      for Index in Buffer'First .. Last loop
         Res := Parse_Byte (In_Connect, Buffer (Index));
      end loop;
      pragma Assert (Res);

      Decode (I, In_Connect, Res);
      pragma Assert (Res);
      pragma Assert (I = O);
   end;

""")
    f.write("""   begin
      pragma Warnings (Off);
      --  Should be raised to ensure we compiled
      --  with Asserts
      pragma Assert (0 = 1);
   exception
      when others =>
         Ada.Text_IO.Put_Line ("Ok");
   end;
end Test;""")

#
# Generate V1
#
pre_files_v1 = (
    'mavlink_v1.gpr',
    'mavlink.ads',
    'mavlink-v1.ads',
    'mavlink-v1.adb',
    'mavlink-x25crc.ads',
    'mavlink-x25crc.adb',
    'mavlink-raw_floats.ads',
    'mavlink-raw_long_floats.ads')

def generate(directory, xml):
    '''generate complete Ada implementation'''
    mavparse.mkdir_p(directory)

    bitmasks = []

    types, types_size, types_files = calculate_types_size(xml)

    basepath = os.path.dirname(os.path.realpath(__file__))
    srcpath = os.path.join(basepath, 'Ada')
    for f in pre_files_v1:
        shutil.copy(os.path.join(srcpath, f), directory)

    shutil.copytree(os.path.join(srcpath, "examples"), os.path.join(directory, "examples"), dirs_exist_ok=True)

    for x in xml:
        for t in x.enum:
            if t.bitmask: bitmasks.append(t.name)

        create_dialect_spec (directory, v1_file, v1, x, xml, copy)
        generate_v1_types(directory, x, types_files, types_size)

        #Generate messages
        for m in x.message:
            f_name, p_name = get_pakage_name(x, m, directory, v1, "v1")

            spec = create_message_pkg (x, f_name, p_name, v1, m, types_files, copy)
            body = create_message_body (f_name, p_name)
            generate_v1_message(m, spec, body, xml, types)
            spec.write("end %s;\n" % p_name)
            body.write("\nend %s;\n" % p_name)
    generate_test_v1(directory, xml, bitmasks)

#
# Generate V2
#

# calculate size of enum type based on the max value
def calculate_enum_size(enum):
    max_value = 0
    for i in enum.entry:
        if i.end_marker: break
        max_value = max(max_value, i.value)

    size = math.ceil(math.ceil(math.log2(max_value + 1)) / 8)
    if size < 1: size = 1
    return size

# calculate size of bitmask type based on the fields count
def calculate_bitmask_size(bitmask):
    c = 0
    for i in bitmask.entry:
        if i.end_marker:
            c += math.ceil(math.log2(i.value))
            break

        elif i.value == 0:
            continue

        elif not is_position(i.value):
            print("%s ignored because the composite value!" % i.name)
            continue

        c += 1
    return math.ceil(c / 8)

# generate types and place them into Mavlink.`dialect` package
def generate_v2_types (directory, x, types_files, types_size):
    name = pkg_name(x)
    spec = create_types_spec (directory, v2_file, v2, x)

    # Generate types
    for t in x.enum:
        size = types_size[t.name]

        if t.bitmask:
            if size is None: size = calculate_bitmask_size(t)
            spec.write(repr_bitmask(t, size))

        else:
            if size is None: size = calculate_enum_size(t)
            spec.write(repr_enum(t, size, types_files))
        spec.write("\n")

    spec.write("end %s.%s.Types;\n" % (v2, name))

#Generate message
def generate_v2_message(msg, spec, body, xml, types):
    name = normalize_message_name(msg.name)

    max_len = 0
    for field in msg.fields:
        max_len = max(max_len, len(normalize_field_name(field.name)))

    spec.write("   type %s is record\n" % name)
    for field in msg.fields:
        field_name = normalize_field_name(field.name).title()
        spec.write("      %s : " % field_name.ljust(max_len))
        tp = ""

        if field.type == 'char':
            tp = "String"
            spec.write("String (1 .. %i)" % field.array_length)

        else:
            if field.array_length:
                tp = field_array_types[field.type]
                spec.write("%s (1 .. %i)" % (tp, field.array_length))

            elif field.enum:
                tp = normalize_enum_name(field.enum)
                if field_name == tp: tp = find_pkg (xml, tp, v2)
                spec.write(tp)

            else:
                tp = field_types[field.type]
                spec.write(tp)

        if field.invalid:
            s = get_default(field.invalid, tp, array_to_float, types)
            if s != "":
                spec.write(" :=\n        %s" % s)

        spec.write(";\n")
        if field.units: spec.write("      --  Units: %s\n" % field.units)
        spec.write(format_comment(field.description, 6))
    spec.write("   end record;\n\n")

    if msg.deprecated:
        spec.write("   pragma Obsolescent (%s);\n\n" % name)

    generate_message_representation(msg, spec, max_len)
    generate_message_methods(name, v2, spec, False)
    generate_message_methods_V2(name, spec)
    generate_message_body(name, v2, str(msg.crc_extra), body, False)

# Find package name
def find_package(xml, name):
    for x in xml:
        for t in x.enum:
            if not t.bitmask and t.name == name:
                return pkg_name(x) + ".Types"
    return ""

# Generate test project for V2
def generate_test_v2(directory, xml, bitmasks):
    Has_Common = False
    dir = os.path.join(directory, 'tests')
    generate_test_project(directory, "v2")

    f = open(os.path.join(dir, "test.adb"), "w")

    f.write("\npragma Warnings (Off); --  prevent `not used`\n")
    for x in xml:
        f.write("with %s.%s.Types;\n" % (v2, file_name(x).title()))
    f.write("pragma Warnings (On);\n\n")

    for x in xml:
        for m in x.message:
            f_name, p_name = get_pakage_name(x, m, directory, v2, "v2")
            f.write("with %s;\n" % p_name)

    f.write("""\nwith Ada.Text_IO;

pragma Warnings (Off); --  prevent "not used"
with MAVLink.Raw_Floats;      use MAVLink.Raw_Floats;
with MAVLink.Raw_Long_Floats; use MAVLink.Raw_Long_Floats;
with MAVLink.SHA_256;
pragma Warnings (On);

use MAVLink.V2;

procedure Test
is""");

    for x in xml:
        if "common.xml" in x.filename:
            Has_Common = True

    if Has_Common:
        f.write("""
   D1 : constant MAVLink.SHA_256.Data  := [16#61#, 16#62#, 16#63#];
   R1 : constant MAVLink.SHA_256.State :=
     [16#ba7816bf#, 16#8f01cfea#, 16#414140de#, 16#5dae2223#,
      16#b00361a3#, 16#96177a9c#, 16#b410ff61#, 16#f20015ad#];

   D2 : constant MAVLink.SHA_256.Data :=
     [16#61#, 16#62#, 16#63#, 16#64#, 16#62#, 16#63#, 16#64#, 16#65#, 16#63#,
      16#64#, 16#65#, 16#66#, 16#64#, 16#65#, 16#66#, 16#67#, 16#65#, 16#66#,
      16#67#, 16#68#, 16#66#, 16#67#, 16#68#, 16#69#, 16#67#, 16#68#, 16#69#,
      16#6a#, 16#68#, 16#69#, 16#6a#, 16#6b#, 16#69#, 16#6a#, 16#6b#, 16#6c#,
      16#6a#, 16#6b#, 16#6c#, 16#6d#, 16#6b#, 16#6c#, 16#6d#, 16#6e#, 16#6c#,
      16#6d#, 16#6e#, 16#6f#, 16#6d#, 16#6e#, 16#6f#, 16#70#, 16#6e#, 16#6f#,
      16#70#, 16#71#];
   R2 : constant MAVLink.SHA_256.State :=
     [16#248d6a61#, 16#d20638b8#, 16#e5c02693#, 16#0c3e6039#,
      16#a33ce459#, 16#64ff2167#, 16#f6ecedd4#, 16#19db06c1#];

   D3 : constant MAVLink.SHA_256.Data :=
     [16#6C#, 16#6F#, 16#6E#, 16#67#, 16#5F#, 16#70#, 16#61#, 16#73#, 16#73#,
      16#77#, 16#6F#, 16#72#, 16#64#, 16#fd#, 16#05#, 16#01#, 16#00#, 16#00#,
      16#01#, 16#01#, 16#78#, 16#32#, 16#00#, 16#01#, 16#00#, 16#00#, 16#00#,
      16#01#, 16#08#, 16#98#, 16#01#, 16#c8#, 16#00#, 16#00#, 16#00#, 16#00#,
      16#00#];
   R3 : constant MAVLink.SHA_256.State :=
     [16#48c298bd#, 16#a123637a#, 16#f1103486#, 16#180a716a#,
      16#9c41e4b1#, 16#42293472#, 16#ea587ff5#, 16#247d5943#];
   T  : Data_Buffer (1 .. 32) with Import, Address => R3'Address;

   procedure Do_SHA_256_Test (D : MAVLink.SHA_256.Data; R : MAVLink.SHA_256.State);
   procedure Do_SHA_256_Test (D : MAVLink.SHA_256.Data; R : MAVLink.SHA_256.State)
   is
      use type MAVLink.SHA_256.State;

      Checksum : MAVLink.SHA_256.Context;
      Result   : MAVLink.SHA_256.Digest_Type;
      Res      : MAVLink.SHA_256.State (1 .. 8) with Import,
        Address => Result'Address;
   begin
      MAVLink.SHA_256.Update (Checksum, D);
      MAVLink.SHA_256.Digest (Checksum, Result);
      pragma Assert (Res = R);
   end Do_SHA_256_Test;

   Hygrometer_Sensors_Data : constant Data_Buffer :=
     [253, 5, 1, 0, 0, 1, 1, 120, 50,  0, --  Header
      1, 0, 0, 0, 1, --  Message
      8, 152, --  CRC
      1, -- Link_Id
      200, 0, 0, 0, 0, 0, --  Timestamp
      189, 152, 194, 72,  122, 99]; --  SHA
""")

    f.write("""
   Sig         : MAVLink.V2.Signature;
   In_Connect  : MAVLink.V2.Connection;
   Out_Connect : MAVLink.V2.Out_Connection;
   Pass        : constant String := "long_password";
   Pass_Data   : Signature_Key (1 .. Pass'Length) with Import,
     Address => Pass'Address;
   Res         : Boolean;

   Seq         : Sequence_Id_Type;
   Sys_Id      : System_Id_Type;
   Comp_Id     : Component_Id_Type;
   Id          : Msg_Id;
   Link_Id     : Link_Id_Type;
   Timestamp   : Timestamp_Type;
   Signature   : Three_Boolean;
   Buffer      : Data_Buffer (1 .. MAVLink.V2.Maximum_Buffer_Len);
   Last        : Positive;

begin
   Initialize (Sig,  1, Pass_Data, 200);
   Set_System_Id (In_Connect, 1);
   Set_Component_Id (In_Connect, 1);
   Set_System_Id (Out_Connect, 1);
   Set_Component_Id (Out_Connect, 1);
""")

    if Has_Common:
        f.write("""
   Do_SHA_256_Test (D1, R1);
   Do_SHA_256_Test (D2, R2);
   Do_SHA_256_Test (D3, R3);

   -- Income
   for Index in Hygrometer_Sensors_Data'First ..
     Hygrometer_Sensors_Data'Last
   loop
      Res := Parse_Byte (In_Connect, Hygrometer_Sensors_Data (Index));
   end loop;
   pragma Assert (Res);

   Get_Message_Information
     (In_Connect, Sig, Seq, Sys_Id, Comp_Id, Id, Link_Id, Timestamp, Signature);

   pragma Assert (Seq = 0);
   pragma Assert (Sys_Id = 1);
   pragma Assert (Comp_Id = 1);
   pragma Assert
     (Id =
        MAVLink.V2.Common.Hygrometer_Sensor_Id);
   pragma Assert (Link_Id = 1);
   pragma Assert (Timestamp = 200);
   pragma Assert (Signature = True);

   -- In / Out
   declare
      use MAVLink.V2.Common.Hygrometer_Sensors;
      M : constant Hygrometer_Sensor :=
        (Id => 1, Temperature => 1, Humidity => 0);
      O : Hygrometer_Sensor;
   begin
      Encode (M, Out_Connect, Sig, Buffer, Last);
      pragma Assert (Last = 30);

      for Index in Buffer'First .. Last loop
         Res := Parse_Byte (In_Connect, Buffer (Index));
      end loop;
      pragma Assert (Res);

      Get_Message_Information
        (In_Connect, Sig, Seq, Sys_Id, Comp_Id, Id, Link_Id, Timestamp, Signature);

      pragma Assert (Seq = 0);
      pragma Assert (Sys_Id = 1);
      pragma Assert (Comp_Id = 1);
      pragma Assert (Id = MAVLink.V2.Common.Hygrometer_Sensor_Id);
      pragma Assert (Link_Id = 1);
      pragma Assert (Timestamp = 200);
      pragma Assert (Signature = True);

      Decode (O, In_Connect, Res);
      pragma Assert (M = O);
   end;

""")

    for x in xml:
        for m in x.message:
            msg = normalize_message_name(m.name)
            f_name, p_name = get_pakage_name(x, m, directory, v2, "v2")
            f.write("""   declare
      use %s;
      I : %s;
      O : constant %s :=
         (""" % (p_name, msg, msg))
            first = True
            for field in m.fields:
                field_name = normalize_field_name(field.name).title()
                if field.invalid:
                    value = "<>"

                else:
                    if field.type == 'char':
                        value = "[others => 'A']"

                    else:
                        if field.array_length:
                            value = field_array_values[field.type]

                        elif field.enum:
                            if field.enum in bitmasks:
                                value = "<>"

                            else:
                                value = "MAVLink.V2.%s.%s'First" % (
                                    find_package(xml, field.enum),
                                    normalize_enum_name(field.enum))
                        else:
                            value = field_values[field.type]

                if first: 
                    first = False
                    f.write("%s => %s" % (field_name, value))
                else:
                    f.write(",\n          %s => %s" % (field_name, value))

            f.write(""");
   begin
      Encode (O, Out_Connect, Sig, Buffer, Last);

      for Index in Buffer'First .. Last - 1 loop
         Res := Parse_Byte (In_Connect, Buffer (Index));
         pragma Assert (not Res);
      end loop;
      Res := Parse_Byte (In_Connect, Buffer (Last));
      pragma Assert (Res);

      Get_Message_Information
        (In_Connect, Sig, Seq, Sys_Id, Comp_Id, Id, Link_Id, Timestamp, Signature);
      pragma Assert (Signature = True);

      Decode (I, In_Connect, Res);
      pragma Assert (Res);
      pragma Assert (I = O);
   end;

""")
    f.write("""   begin
      pragma Warnings (Off);
      --  Should be raised to ensure we compiled
      --  with Asserts
      pragma Assert (0 = 1);
   exception
      when others =>
         Ada.Text_IO.Put_Line ("Ok");
   end;
end Test;""")

# Generate V2
pre_files_v1_for_v2 = (
    'mavlink.ads',
    'mavlink-x25crc.ads',
    'mavlink-x25crc.adb',
    'mavlink-raw_floats.ads',
    'mavlink-raw_long_floats.ads'
)

pre_files_v2 = (
    'mavlink_v2.gpr',
    'mavlink-v2.ads',
    'mavlink-v2.adb',
    'mavlink-sha_256.ads',
    'mavlink-sha_256.adb'
)

def generate_v2(directory, xml):
    '''generate complete Ada implementation for v2'''
    mavparse.mkdir_p(directory)

    bitmasks = []

    types, types_size, types_files = calculate_types_size(xml)

    basepath = os.path.dirname(os.path.realpath(__file__))
    srcpath = os.path.join(basepath, 'Ada')
    for f in pre_files_v1_for_v2:
        shutil.copy(os.path.join(srcpath, f), directory)

    srcpath = os.path.join(basepath, 'Ada', 'v2')
    for f in pre_files_v2:
        shutil.copy(os.path.join(srcpath, f), directory)

    for x in xml:
        for t in x.enum:
            if t.bitmask: bitmasks.append(t.name)

        # Mavlink.`dialect`
        create_dialect_spec (directory, v2_file, v2, x, xml)
        generate_v2_types (directory, x, types_files, types_size)

        #Generate messages
        for m in x.message:
            f_name, p_name = get_pakage_name(x, m, directory, v2, "v2")

            spec = create_message_pkg (x, f_name, p_name, v2, m, types_files)
            body = create_message_body (f_name, p_name)
            generate_v2_message(m, spec, body, xml, types)
            spec.write("end %s;\n" % p_name)
            body.write("\nend %s;\n" % p_name)

        def create_renames (c):
            for m in c.message:
                f_name, p_name = get_pakage_name(x, m, directory, v2, "v2")
                fc_name, pc_name = get_pakage_name(c, m, directory, v2, "v2")
                spec = open(f_name + ".ads", "w")
                spec.write(GEN)
                if m.deprecated:
                    spec.write(get_deprecated(m.deprecated, 0))
                spec.write(format_comment (m.description, 0))
                spec.write("\npragma Ada_2022;\n")
                spec.write("\nwith %s;\n" % pc_name)
                spec.write("\npackage %s\nrenames %s;\n" % (p_name, pc_name))

        def process_dialect(c, rename):
            if rename: create_renames(c)
            for i in c.include:
                for t in xml:
                    if i in t.filename:
                        process_dialect(t, True)

        process_dialect (x, False)

    generate_test_v2(directory, xml, bitmasks)