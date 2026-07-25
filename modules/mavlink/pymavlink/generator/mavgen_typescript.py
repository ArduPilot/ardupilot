#!/usr/bin/env python3
"""
parse a MAVLink protocol XML file and generate a Node.js typescript module implementation

Based on original work Copyright Andrew Tridgell 2011
Released under GNU GPL version 3 or later
"""
import os
from . import mavtemplate

t = mavtemplate.MAVTemplate()


def camelcase(str):
    parts = str.split('_')
    result = ''
    for part in parts:
        result += part.lower().capitalize()
    return result


def generate_enums(dir, enums):
    print("Generating enums")

    if not os.path.isdir(dir):
        os.mkdir(dir)
        
    for e in enums:
        filename = e.name.replace('_', '-')
        filename = filename.lower()
        with open('{}/{}.ts'.format(dir, filename), "w") as f:
            f.write("export enum {} {{\n".format(camelcase(e.name)))
            for entry in e.entry:
                f.write(
                    "\t{} = {}, // {}\n".format(entry.name, entry.value, entry.description.rstrip("\r").rstrip("\n")))
            f.write("}")


def generate_classes(dir, registry, msgs, xml):
    print("Generating class definitions")

    ts_types = {"uint8_t": "number", "uint16_t": "number", "uint32_t": "number", "uint64_t": "number",
                "int8_t": "number", "int16_t": "number", "int32_t": "number", "int64_t": "number",
                "float": "number", "double": "number", "char": "string"}

    if not os.path.isdir(dir):
        os.mkdir(dir)

    with open(registry, "w") as registry_f:
        registry_f.write("import {MAVLinkMessage} from 'node-mavlink';\n")
        for m in msgs:
            filename = m.name.replace('_', '-')
            filename = filename.lower()

            m.order_map = [0] * len(m.fieldnames)
            for i in range(0, len(m.fieldnames)):
                m.order_map[i] = m.ordered_fieldnames.index(m.fieldnames[i])

            with open('{}/{}.ts'.format(dir, filename), "w") as f:
                if xml.wire_protocol_version == '1.0':
                    raise Exception('WireProtocolException', 'Please use WireProtocol = 2.0 only.')

                f.write("import {MAVLinkMessage} from 'node-mavlink';\n")
                f.write("import {readInt64LE, readUInt64LE} from 'node-mavlink';\n")
                registry_f.write("import {{{}}} from './messages/{}';\n".format(camelcase(m.name), filename))
                imported_enums = []
                for enum in [field.enum for field in m.fields if field.enum != '']:
                    if enum not in imported_enums:
                        f.write("import {{{}}} from '../enums/{}';\n".format(camelcase(enum),
                                                                             enum.replace('_', '-').lower()))
                        imported_enums.append(enum)

                f.write("/*\n{}\n*/\n".format(m.description.strip()))
                for field in m.fields:
                    f.write("// {} {} {}\n".format(field.name, field.description.strip(), field.type))

                f.write("export class {} extends MAVLinkMessage {{\n".format(camelcase(m.name)))

                for field in m.fields:
                    if field.enum:
                        f.write("\tpublic {}!: {};\n".format(field.name, camelcase(field.enum)))
                    else:
                        f.write("\tpublic {}!: {};\n".format(field.name, ts_types[field.type]))

                f.write("\tpublic _message_id: number = {};\n".format(m.id))
                f.write("\tpublic _message_name: string = '{}';\n".format(m.name))
                f.write("\tpublic _crc_extra: number = {};\n".format(m.crc_extra))

                i = 0
                f.write("\tpublic _message_fields: [string, string, boolean][] = [\n")
                for fieldname in m.ordered_fieldnames:
                    field = next(field for field in m.fields if field.name == fieldname)
                    if m.extensions_start is not None and i >= m.extensions_start:
                        extension = 'true'
                    else:
                        extension = 'false'
                    f.write("\t\t['{}', '{}', {}],\n".format(field.name, field.type, extension))
                    i += 1
                f.write("\t];\n".format("', '".join(m.ordered_fieldnames)))

                f.write("}")

        registry_f.write(
            "export const messageRegistry: Array<[number, new (system_id: number, component_id: number) "
            "=> MAVLinkMessage]> = [\n")
        for m in msgs:
            registry_f.write("\t[{}, {}],\n".format(m.id, camelcase(m.name)))
        registry_f.write("];")


def generate_tsconfig(basename):
    with open('{}/tsconfig.json'.format(basename), "w") as f:
        f.write(
            "{\n  \"compilerOptions\": {\n    \"target\": \"es5\",\n    \"module\": \"commonjs\","
            "\n    \"declaration\": true,\n    \"declarationMap\": true,\n    \"sourceMap\": true,"
            "\n    \"outDir\": \"./\",\n    \"strict\": true,\n    \"esModuleInterop\": true\n  },"
            "\n  \"include\": [\n    \"./\"\n  ],\n  \"exclude\": [\n    \"**/*.d.ts\","
            "\n    \"**/*.d.ts.map\",\n    \"**/*.js\",\n    \"**/*.js.map\"\n  ]\n}")


def generate(basename, xml):
    enums_dir = basename + '/enums'
    messages_dir = basename + '/messages'
    message_registry = basename + '/message-registry.ts'

    msgs = []
    enums = []
    filelist = []
    for x in xml:
        msgs.extend(x.message)
        enums.extend(x.enum)
        filelist.append(os.path.basename(x.filename))

    generate_enums(enums_dir, enums)
    generate_classes(messages_dir, message_registry, msgs, xml[0])
    generate_tsconfig(basename)
