# flake8: noqa

import bson

from jsonemit import JSONEmit


# Emit ArduPilot documentation in BSON format
class BSONEmit(JSONEmit):
    def output_fname(self):
        return 'apm.pdef.bson'

    def close(self):
        self.add_firmware_metadata()
        with open(self.output_fname(), mode='wb') as output_file:
            output_file.write(bson.encode(self.content))
