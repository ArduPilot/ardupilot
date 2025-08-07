#!/usr/bin/env python3
'''
setup board.h for ESP32

AP_FLAKE8_CLEAN

'''

import argparse
# import shlex
import sys
import os

sys.path.append(os.path.join(os.path.dirname(os.path.realpath(__file__)), '../../../../libraries/AP_HAL/hwdef/scripts'))
import hwdef  # noqa:E402


class ESP32HWDef(hwdef.HWDef):

    def __init__(self, quiet=False, outdir=None, hwdef=[]):
        super(ESP32HWDef, self).__init__(quiet=quiet, outdir=outdir, hwdef=hwdef)

    def write_hwdef_header_content(self, f):
        for d in self.alllines:
            if d.startswith('define '):
                f.write('#define %s\n' % d[7:])

        # self.write_SPI_config(f)
        self.write_IMU_config(f)
        self.write_MAG_config(f)
        self.write_BARO_config(f)

        self.write_env_py(os.path.join(self.outdir, "env.py"))

    def process_line(self, line, depth):
        '''process one line of pin definition file'''
        # keep all config lines for later use
        self.all_lines.append(line)
        self.alllines.append(line)

        # a = shlex.split(line, posix=False)

        super(ESP32HWDef, self).process_line(line, depth)


if __name__ == '__main__':

    parser = argparse.ArgumentParser("esp32_hwdef.py")
    parser.add_argument(
        '-D', '--outdir', type=str, default="/tmp", help='Output directory')
    parser.add_argument(
        'hwdef', type=str, nargs='+', default=None, help='hardware definition file')
    parser.add_argument(
        '--quiet', action='store_true', default=False, help='quiet running')

    args = parser.parse_args()

    c = ESP32HWDef(
        outdir=args.outdir,
        hwdef=args.hwdef,
        quiet=args.quiet,
    )
    c.run()
