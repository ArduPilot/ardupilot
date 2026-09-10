'''
AP_FLAKE8_CLEAN
'''

from toon import encode

import emit_json


class TOONEmitter(emit_json.JSONEmitter):
    def emit(self, doccos, enumerations):
        with open('LogMessages.toon', mode='w', encoding='utf-8') as output_file:
            output_file.write(encode(self.message_file(doccos, enumerations)))
            output_file.write('\n')
