'''
AP_FLAKE8_CLEAN
'''

import bson

import emit_json


class BSONEmitter(emit_json.JSONEmitter):
    def emit(self, doccos, enumerations):
        with open('LogMessages.bson', mode='wb') as output_file:
            output_file.write(bson.encode(self.message_file(doccos, enumerations)))
