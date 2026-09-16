'''
AP_FLAKE8_CLEAN
'''
import json

import emitter


class JSONEmitter(emitter.Emitter):
    def emit(self, doccos, enumerations):
        message_file = {}
        if self.git_sha is not None or self.git_branch is not None:
            firmware = {}
            if self.git_sha is not None:
                firmware['git_sha'] = self.git_sha
            if self.git_branch is not None:
                firmware['git_branch'] = self.git_branch
            message_file['firmware'] = firmware

        logformats = []
        for docco in doccos:
            logformat = {'name': docco.name}
            if docco.url is not None:
                logformat['url'] = docco.url
            if docco.description is not None:
                logformat['description'] = docco.description

            fields = []
            for field_name in docco.fields_order:
                field_data = docco.fields[field_name]
                field = {
                    'name': field_name,
                    'units': field_data.get('units', ''),
                    'type': field_data.get('fmt', ''),
                }
                if 'description' in field_data:
                    field['description'] = field_data['description']

                enum_key = None
                enum_type = None
                if 'bitmaskenum' in field_data:
                    enum_key = 'bitmaskenum'
                    enum_type = 'bitmask'
                elif 'valueenum' in field_data:
                    enum_key = 'valueenum'
                    enum_type = 'enum'

                if enum_key is not None:
                    enum_name = field_data[enum_key]
                    if enum_name not in enumerations:
                        raise Exception("Unknown enum (%s) (have %s)" %
                                        (enum_name, "\n".join(sorted(enumerations.keys()))))
                    enum = {
                        'name': enum_name,
                        'entries': [],
                    }
                    for entry in enumerations[enum_name].entries:
                        enum_entry = {
                            'name': entry.name,
                            'value': entry.value,
                        }
                        if entry.comment is not None:
                            enum_entry['description'] = entry.comment
                        enum['entries'].append(enum_entry)
                    field[enum_type] = enum

                fields.append(field)

            logformat['fields'] = fields
            logformats.append(logformat)

        message_file['logformats'] = logformats
        with open('LogMessages.json', mode='w', encoding='utf-8') as output_file:
            json.dump(message_file, output_file, indent=4, ensure_ascii=False)
            output_file.write('\n')
