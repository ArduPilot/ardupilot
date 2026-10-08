#!/usr/bin/env python3
# AP_FLAKE8_CLEAN
"""Move chosen objects' code into RAM by renaming their text sections.

WHY THIS AND NOT A LINKER SCRIPT: hot code that does not fit in ITCM belongs in
the 1 MB at 0x20200000, which is only ~38% used. Every route through the linker
script is blocked - zephyr_linker_sources() hooks are either after the .text rule
or inside an output section, zephyr_code_relocate() needs a CMake target and
ArduPilot is a waf-built archive, and ld's INSERT BEFORE cannot augment a script
supplied with -T, which is how Zephyr passes linker.cmd. See the note in
AP_HAL_Zephyr/zephyr/CMakeLists.txt for all four.

What is NOT blocked: Zephyr's own .ramfunc section already does this job. The
generated script has

    .ramfunc : { *(.ramfunc) *(".ramfunc.*") ... } > RAM AT > FLASH

and arch/common/xip.c copies it at boot, so anything landing in a section named
.ramfunc.* runs from RAM with no script change and no copy code of our own. The
__ramfunc flash primitives prove the path works - they link at 0x2021xxxx.

So: rename each .text* section of the named objects to .ramfunc.text*, in place
inside the archive. Only text sections are touched; .rodata/.data/.bss keep their
names, because .ramfunc is a code region and moving .bss into it would break its
zero-initialisation.
"""
import argparse
import os
import shutil
import re
import subprocess
import sys
import tempfile

OBJDUMP = os.environ.get('OBJDUMP', 'arm-none-eabi-objdump')
OBJCOPY = os.environ.get('OBJCOPY', 'arm-none-eabi-objcopy')
AR = os.environ.get('AR', 'arm-none-eabi-ar')


def text_sections(obj, prefix='.text'):
    out = subprocess.run([OBJDUMP, '-h', obj], capture_output=True, text=True).stdout
    names = []
    for line in out.splitlines():
        parts = line.split()
        if len(parts) >= 3 and parts[0].isdigit():
            n = parts[1]
            if n == prefix or n.startswith(prefix + '.'):
                names.append(n)
    return names


def members_with_ramfunc(archive):
    """Members that currently carry .ramfunc.text* sections.

    One objdump over the whole archive, not one extract per member: the archive
    holds about a thousand objects and extracting each one every build would
    cost more than the link.
    """
    out = subprocess.run([OBJDUMP, '-h', archive], capture_output=True, text=True).stdout
    found, cur = [], None
    for line in out.splitlines():
        if line.endswith(':     file format') or '(ex ' in line:
            pass
        m = re.match(r'^(\S+\.o(?:bj)?):\s+file format', line)
        if m:
            cur = m.group(1)
            continue
        parts = line.split()
        if cur and len(parts) >= 3 and parts[0].isdigit():
            if parts[1].startswith('.ramfunc.text'):
                if cur not in found:
                    found.append(cur)
    return found


def restore_in_archive(archive, keep_renamed, verbose=True):
    """Put .ramfunc.text* back to .text* for members no longer on the list.

    WHY THIS EXISTS: the rename edits the archive IN PLACE, and waf only
    recompiles a source when it changes. Take an object off the list and its
    member keeps the .ramfunc names from the last build, so it stays in RAM and
    an itcm_hot_code.ld entry that selects .text cannot match it - the list
    silently stops being what the image does. Deleting a line from
    ramtext_objects.txt must mean something on the very next build.
    """
    stale = [m for m in members_with_ramfunc(archive) if m not in keep_renamed]
    if not stale:
        return 0
    restored = 0
    tmp = tempfile.mkdtemp(prefix='ramtext_restore.')
    try:
        for m in stale:
            subprocess.run([AR, 'x', os.path.abspath(archive), m], cwd=tmp, check=True)
            obj = os.path.join(tmp, m)
            secs = text_sections(obj, '.ramfunc.text')
            if not secs:
                continue
            cmd = [OBJCOPY]
            for sec in secs:
                cmd += ['--rename-section', f'{sec}={sec[len(".ramfunc"):]}']
            cmd.append(obj)
            subprocess.run(cmd, check=True)
            subprocess.run([AR, 'r', os.path.abspath(archive), m], cwd=tmp, check=True)
            restored += len(secs)
            if verbose:
                print(f'  {m}: {len(secs)} sections restored to .text*')
    finally:
        shutil.rmtree(tmp, ignore_errors=True)
    return restored


def rename_in_archive(archive, patterns, verbose=True):
    members = subprocess.run([AR, 't', archive], capture_output=True, text=True).stdout.split()
    targets = [m for m in members if any(p in m for p in patterns)]
    restored = restore_in_archive(archive, targets, verbose)
    if not targets:
        # Not an error by itself: the caller may hand us several archives and only
        # one of them holds the vehicle code.
        print(f'no members matched in {os.path.basename(archive)}')
        return 0
    tmp = tempfile.mkdtemp(prefix='ramtext.')
    moved = 0
    try:
        for m in targets:
            subprocess.run([AR, 'x', os.path.abspath(archive), m], cwd=tmp, check=True)
            obj = os.path.join(tmp, m)
            secs = text_sections(obj)
            if not secs:
                continue
            cmd = [OBJCOPY]
            for s in secs:
                cmd += ['--rename-section', f'{s}=.ramfunc{s}']
            cmd.append(obj)
            subprocess.run(cmd, check=True)
            subprocess.run([AR, 'r', os.path.abspath(archive), m], cwd=tmp, check=True)
            moved += len(secs)
            if verbose:
                print(f'  {m}: {len(secs)} text sections -> .ramfunc.*')
    finally:
        shutil.rmtree(tmp, ignore_errors=True)
    # moved == 0 with targets found means they were renamed by an earlier run.
    # This has to be success, or an incremental build reports failure every time.
    print(f'renamed {moved} sections across {len(targets)} objects'
          f'{" (already done)" if moved == 0 else ""}'
          f'{f", restored {restored}" if restored else ""}')
    return 0


if __name__ == '__main__':
    ap = argparse.ArgumentParser()
    ap.add_argument('archive')
    ap.add_argument('patterns', nargs='+', help='substrings matching archive member names')
    a = ap.parse_args()
    sys.exit(rename_in_archive(a.archive, a.patterns))
