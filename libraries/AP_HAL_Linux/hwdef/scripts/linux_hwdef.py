#!/usr/bin/env python
'''
setup board.h for Linux

AP_FLAKE8_CLEAN

'''

import argparse
import sys
import os

sys.path.append(os.path.join(os.path.dirname(os.path.realpath(__file__)), '../../../../libraries/AP_HAL/hwdef/scripts'))
import hwdef  # noqa:E402


class LinuxHWDef(hwdef.HWDef):

    def __init__(self, quiet=False, outdir=None, hwdef=[]):
        super(LinuxHWDef, self).__init__(quiet=quiet, outdir=outdir, hwdef=hwdef)

    def write_hwdef_header_content(self, f):
        for d in self.alllines:
            if d.startswith('define '):
                f.write('#define %s\n' % d[7:])


if __name__ == '__main__':

    parser = argparse.ArgumentParser("linux_hwdef.py")
    parser.add_argument(
        '-D', '--outdir', type=str, default="/tmp", help='Output directory')
    parser.add_argument(
        'hwdef', type=str, nargs='+', default=None, help='hardware definition file')
    parser.add_argument(
        '--quiet', action='store_true', default=False, help='quiet running')

    args = parser.parse_args()

    c = LinuxHWDef(
        outdir=args.outdir,
        hwdef=args.hwdef,
        quiet=args.quiet,
    )
    c.run()
