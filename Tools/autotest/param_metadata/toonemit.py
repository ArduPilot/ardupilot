# flake8: noqa

from toon import encode

from jsonemit import JSONEmit


# Emit ArduPilot documentation in TOON format
class TOONEmit(JSONEmit):
    def output_fname(self):
        return 'apm.pdef.toon'

    def close(self):
        self.add_firmware_metadata()
        # python-toon cannot encode an empty object key. Parameter metadata
        # uses one for parameters not associated with a named group.
        toon_content = '\n'.join(
            encode({'__TOON_EMPTY_KEY__' if key == '' else key: value}).replace(
                '__TOON_EMPTY_KEY__:', '"":', 1)
            for key, value in self.content.items())
        with open(self.output_fname(), mode='w', encoding='utf-8') as output_file:
            output_file.write(toon_content)
            output_file.write('\n')
