#!/usr/bin/env python3
# AP_FLAKE8_CLEAN

"""
Turn any upstream Zephyr sample into a flashable .apj for an ArduPilot Zephyr board.

Why this exists
---------------
When a board misbehaves it is enormously useful to run a *stock Zephyr sample* on it,
with ArduPilot removed from the picture entirely - e.g. samples/drivers/uart/echo_bot to
test a UART, or samples/basic/blinky to prove the board boots at all. Doing that by hand
is fiddly for three reasons, all handled here:

  1. This repo deliberately does not use west, so Zephyr's module list is not discoverable
     the usual way. We scrape it from the CMakeCache.txt of a completed ArduPilot build.
  2. The board lives out-of-tree, so cmake needs BOARD_ROOT.
  3. The bootloader expects the app's vector table at a specific offset, and the padding
     needed is NOT the same number as CONFIG_FLASH_LOAD_OFFSET (see PAD_BYTES below).

Typical use
-----------
    # build the board once so the cmake cache exists
    ./waf configure --board=mr_vmu_rt1176 && ./waf copter

    # then turn any sample into a flashable image
    ./Tools/scripts/zephyr_make_sample_into_apj_flashable.py \\
        modules/zephyr/samples/drivers/uart/echo_bot

    # ...and optionally flash it in the same step
    ./Tools/scripts/zephyr_make_sample_into_apj_flashable.py \\
        modules/zephyr/samples/basic/blinky --flash

Restoring ArduPilot afterwards is just a normal upload of the ArduPilot .apj.
"""

import argparse
import base64
import json
import os
import re
import shutil
import subprocess
import sys
import zlib

# Distance from the start of the uploaded file to where the bootloader expects the app's
# vector table, i.e. the bootloader's own APP_VECTOR_OFFSET constant.
#
# This is deliberately NOT CONFIG_FLASH_LOAD_OFFSET. That Kconfig value is "distance from
# the true flash base to where the linker puts code", which already includes the
# bootloader's reserved region. uploader.py writes the file starting at the bootloader's
# APP_LOAD_ADDRESS, so padding by the full Kconfig value double-counts that region and
# lands the vector table exactly one bootloader-size too far in. Keep this table in sync
# with _BOOTLOADER_UPLOAD_PAD_BYTES in Tools/ardupilotwaf/zephyr.py.
PAD_BYTES = {
    'mr_vmu_rt1176': 0x2000,
}

# Config a sample needs to be laid out the same way an ArduPilot app is: linked behind the
# bootloader, and NOT carrying its own FCB/IVT boot header (the bootloader owns that).
BOOTLOADER_RELATIVE_CONF = """
# Added by zephyr_make_sample_into_apj_flashable.py so this sample links the same way an
# ArduPilot app does on this board: behind the bootloader, without its own boot header.
CONFIG_FLASH_LOAD_OFFSET={flash_load_offset}
CONFIG_NXP_IMXRT_BOOT_HEADER=n
CONFIG_XIP=y
CONFIG_SERIAL=y
CONFIG_UART_INTERRUPT_DRIVEN=y
"""


def repo_root():
    return os.path.normpath(os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', '..'))


def die(msg):
    print('ERROR: %s' % msg, file=sys.stderr)
    sys.exit(1)


def scrape_cmake_cache(cache_path):
    """Pull the Zephyr module set and toolchain out of a completed ArduPilot build.

    Zephyr needs the full module list; without west there is nothing else to derive it
    from. Returns (cmake_args, board_string).
    """
    if not os.path.isfile(cache_path):
        die('no CMakeCache.txt at %s\n'
            '       Build the board first:  ./waf configure --board=<board> && ./waf copter'
            % cache_path)

    args = []
    board = None
    wanted = re.compile(r'^(ZEPHYR_MODULES|ZEPHYR_TOOLCHAIN_VARIANT|'
                        r'ZEPHYR_[A-Z0-9_]+_(?:CMAKE_DIR|KCONFIG)):[^=]*=(.*)$')
    with open(cache_path, 'r', encoding='utf-8', errors='replace') as f:
        for line in f:
            line = line.rstrip('\n')
            m = wanted.match(line)
            if m:
                args.append('-D%s=%s' % (m.group(1), m.group(2)))
                continue
            m = re.match(r'^BOARD:[^=]*=(.*)$', line)
            if m:
                board = m.group(1)
    if board is None:
        die('could not read BOARD from %s' % cache_path)
    return args, board


def read_flash_load_offset(root, board):
    """Take CONFIG_FLASH_LOAD_OFFSET from the board's own Kconfig fragment."""
    for name in ('prj.%s.conf' % board, 'boards/%s.conf' % board):
        path = os.path.join(root, 'libraries/AP_HAL_Zephyr/zephyr', name)
        if not os.path.isfile(path):
            continue
        with open(path, 'r', encoding='utf-8') as f:
            for line in f:
                m = re.match(r'^\s*CONFIG_FLASH_LOAD_OFFSET\s*=\s*(\S+)', line)
                if m:
                    return m.group(1)
    return None


def read_board_id(root, board):
    """APJ_BOARD_ID from the board's hwdef.dat - the bootloader refuses a mismatch."""
    path = os.path.join(root, 'libraries/AP_HAL_Zephyr/hwdef', board, 'hwdef.dat')
    if os.path.isfile(path):
        with open(path, 'r', encoding='utf-8') as f:
            for line in f:
                m = re.match(r'^\s*APJ_BOARD_ID\s+(\S+)', line)
                if m:
                    return int(m.group(1), 0)
    return None


def make_apj(bin_path, apj_path, board, board_id, pad_bytes, description):
    """Pad the raw .bin and wrap it as an .apj, exactly as Tools/ardupilotwaf/zephyr.py does."""
    with open(bin_path, 'rb') as f:
        image = f.read()

    if len(image) < 8:
        die('%s is only %d bytes - too small to contain a vector table' % (bin_path, len(image)))

    # A Cortex-M vector table starts with the initial SP, which must point into RAM. If it
    # does not, objcopy produced something other than the layout we expect and padding it
    # would silently produce an unbootable image.
    initial_sp = int.from_bytes(image[0:4], 'little')
    if (initial_sp & 0x20000000) == 0:
        die('%s does not start with a plausible vector table (first word 0x%08x is not a RAM '
            'address) - refusing to pad' % (bin_path, initial_sp))

    if pad_bytes:
        image = b'\xff' * pad_bytes + image

    flash_total = max(len(image), 1024 * 1024)
    desc = {
        'board_id': board_id,
        'magic': 'APJFWv1',
        'description': description,
        'image': base64.b64encode(zlib.compress(image, 9)).decode('utf-8'),
        'summary': board,
        'version': '0.1',
        'image_size': len(image),
        'flash_total': flash_total,
        'image_maxsize': flash_total,
        'flash_free': max(0, flash_total - len(image)),
        'extflash_total': 0,
        'extflash_free': 0,
        'git_identity': 'zephyr-sample',
        'board_revision': 0,
        'USBID': '0x0000/0x0000',
    }
    with open(apj_path, 'w', encoding='utf-8') as f:
        f.write(json.dumps(desc, indent=4))

    return len(image), initial_sp


def main():
    root = repo_root()
    parser = argparse.ArgumentParser(
        description='Turn an upstream Zephyr sample into a flashable .apj',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog='Example:\n'
               '  %(prog)s modules/zephyr/samples/drivers/uart/echo_bot --flash\n')
    parser.add_argument('sample', help='path to the Zephyr sample directory (contains CMakeLists.txt)')
    parser.add_argument('--board', default='mr_vmu_rt1176',
                        help='ArduPilot board name (default: %(default)s)')
    parser.add_argument('--build-dir', default=None,
                        help='where to build (default: build/<board>/zephyr_sample_<name>)')
    parser.add_argument('--output', default=None, help='output .apj path')
    parser.add_argument('--extra-conf', default=None,
                        help='additional Kconfig fragment to merge in')
    parser.add_argument('--flash', action='store_true',
                        help='upload the result with uploader.py once built')
    parser.add_argument('--jobs', '-j', default=str(os.cpu_count() or 4), help='parallel build jobs')
    args = parser.parse_args()

    sample = os.path.abspath(args.sample)
    if not os.path.isfile(os.path.join(sample, 'CMakeLists.txt')):
        die('%s does not look like a Zephyr sample (no CMakeLists.txt)' % sample)

    zephyr_base = os.path.join(root, 'modules/zephyr')
    board_root = os.path.join(root, 'libraries/AP_HAL_Zephyr/zephyr')
    cache = os.path.join(root, 'build', args.board, 'zephyr_build', 'CMakeCache.txt')

    for tool in ('cmake', 'ninja'):
        if shutil.which(tool) is None:
            die('%s not found in PATH' % tool)

    name = os.path.basename(sample.rstrip('/'))
    build_dir = args.build_dir or os.path.join(root, 'build', args.board, 'zephyr_sample_' + name)
    apj_path = args.output or os.path.join(build_dir, name + '.apj')

    print('sample     : %s' % sample)
    print('board      : %s' % args.board)
    print('build dir  : %s' % build_dir)

    cmake_args, board_string = scrape_cmake_cache(cache)
    print('board str  : %s   (%d cmake args scraped from the ArduPilot build)'
          % (board_string, len(cmake_args)))

    flash_load_offset = read_flash_load_offset(root, args.board)
    board_id = read_board_id(root, args.board)
    pad = PAD_BYTES.get(args.board, 0)
    if board_id is None:
        die('could not find APJ_BOARD_ID in hwdef.dat for board %s' % args.board)
    print('board_id   : %d' % board_id)
    print('pad bytes  : 0x%x   (APP_VECTOR_OFFSET, NOT CONFIG_FLASH_LOAD_OFFSET)' % pad)

    # Kconfig fragment giving the sample an ArduPilot-compatible flash layout.
    os.makedirs(build_dir, exist_ok=True)
    conf_path = os.path.join(build_dir, 'ap_sample_overlay.conf')
    with open(conf_path, 'w', encoding='utf-8') as f:
        if flash_load_offset is not None:
            f.write(BOOTLOADER_RELATIVE_CONF.format(flash_load_offset=flash_load_offset))
            print('flash off  : %s   (from the board Kconfig fragment)' % flash_load_offset)
        else:
            print('flash off  : none found - building the sample as a DIRECT-FLASH image')
        if args.extra_conf:
            with open(args.extra_conf, 'r', encoding='utf-8') as extra:
                f.write('\n' + extra.read())

    env = dict(os.environ, ZEPHYR_BASE=zephyr_base)

    cmd = ['cmake', '-GNinja', '-B', build_dir, '-S', sample,
           '-DBOARD=%s' % board_string,
           '-DBOARD_ROOT=%s' % board_root,
           '-DEXTRA_CONF_FILE=%s' % conf_path] + cmake_args
    print('\n=== cmake configure ===')
    if subprocess.call(cmd, env=env, cwd=root) != 0:
        die('cmake configure failed')

    print('\n=== ninja build ===')
    if subprocess.call(['ninja', '-C', build_dir, '-j', args.jobs], env=env, cwd=root) != 0:
        die('build failed')

    bin_path = os.path.join(build_dir, 'zephyr', 'zephyr.bin')
    if not os.path.isfile(bin_path):
        die('no zephyr.bin produced at %s' % bin_path)

    size, sp = make_apj(bin_path, apj_path, args.board, board_id, pad,
                        'Zephyr sample %s for %s' % (name, args.board))
    print('\n=== packaged ===')
    print('  image      : %d bytes (including 0x%x pad)' % (size, pad))
    print('  initial SP : 0x%08x  (RAM address - vector table looks sane)' % sp)
    print('  apj        : %s' % apj_path)

    upload = ['python3', os.path.join(root, 'Tools/scripts/uploader.py'), apj_path]
    if args.flash:
        print('\n=== uploading ===')
        print('NOTE: if it sits on "waiting for the bootloader", reset the board now')
        print('      (power cycle, or: pyocd reset -t mimxrt1170_cm7 -m hw)')
        rc = subprocess.call(upload, cwd=root)
        if rc != 0:
            die('upload failed (rc=%d)' % rc)
        print('\nFlashed. Reflash ArduPilot with a normal uploader.py run when finished.')
    else:
        print('\nTo flash it:\n    %s' % ' '.join(upload))
        print('If it waits on the bootloader, reset the board:')
        print('    pyocd reset -t mimxrt1170_cm7 -m hw')

    return 0


if __name__ == '__main__':
    sys.exit(main())
