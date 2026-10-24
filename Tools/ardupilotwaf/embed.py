#!/usr/bin/env python3

# flake8: noqa

'''
script to create ap_romfs_embedded.h from a set of static files

Andrew Tridgell
May 2017
'''

import os, re, struct, sys, zlib


APMOD_MAGIC = b'APMOD07\0'
APMOD_HEADER = struct.Struct('<8s14I16s2I')
APMOD_FIXUP = struct.Struct('<I')
APMOD_IMPORT = struct.Struct('<QII')
APMOD_HEADER_SIZE = APMOD_HEADER.size
APMOD_MACHINE_ARM = 40
APMOD_POINTER_WIDTH_ARM = 4
APMOD_ALIGNMENT = 4096
APMOD_MAX_IMAGE = 16 * 1024 * 1024
APMOD_MAX_IMPORTS = 4096
APMOD_MAX_IMPORT_NAME = 63
ROMFS_STORED_SIZE_MASK = 0x3FFFFFFF
ROMFS_FLAG_UNCOMPRESSED = 1 << 31
ROMFS_FLAG_PRELINKED = 1 << 30

def write_encode(out, s):
    out.write(s.encode())

def write_bytes(out, contents):
    write_encode(out, ",".join(str(c) for c in contents))


def apmod_details(contents, embedded_name):
    '''validate an APMOD07 module and return its machine and fixups'''
    if len(contents) < APMOD_HEADER_SIZE:
        raise ValueError("Invalid AP_Module module %s: truncated header" % embedded_name)
    fields = APMOD_HEADER.unpack_from(contents)
    (magic, version, machine, width, alignment, file_size, memory_size, rx_size,
     entry, count, abi, profile, stack, heap, import_count, fingerprint, priority, crc) = fields
    if magic != APMOD_MAGIC or version != 7:
        raise ValueError("Invalid AP_Module module %s: expected APMOD07" % embedded_name)
    if (machine, width) not in ((40, 4), (62, 8)):
        raise ValueError("Invalid AP_Module module %s: unsupported target" % embedded_name)
    rw_alignment, rx_alignment = alignment & 0xffff, alignment >> 16
    if (any(not width <= value <= APMOD_ALIGNMENT or value & (value - 1) != 0
            for value in (rw_alignment, rx_alignment)) or
            file_size == 0 or file_size > memory_size or
            memory_size > APMOD_MAX_IMAGE or rx_size == 0):
        raise ValueError("Invalid AP_Module module %s: invalid image sizes" % embedded_name)
    boundary = width if machine == APMOD_MACHINE_ARM else APMOD_ALIGNMENT
    if rx_size > file_size or rx_size % boundary != 0 or entry >= rx_size:
        raise ValueError("Invalid AP_Module module %s: invalid RX image" % embedded_name)
    if machine == APMOD_MACHINE_ARM and (entry & 1) == 0:
        raise ValueError("Invalid AP_Module module %s: ARM entry is not Thumb" % embedded_name)
    if (abi == 0 or (machine, profile) not in ((40, 1), (62, 2), (40, 3)) or
            stack < 256 or stack > APMOD_MAX_IMAGE or heap < 256 or heap > APMOD_MAX_IMAGE or
            fingerprint == bytes(len(fingerprint)) or not 1 <= priority <= 254):
        raise ValueError("Invalid AP_Module module %s: invalid runtime contract" % embedded_name)
    fixup_start = APMOD_HEADER_SIZE + file_size
    fixup_end = fixup_start + count * APMOD_FIXUP.size
    import_end = fixup_end + import_count * APMOD_IMPORT.size
    if (count > file_size // width or import_count > min(file_size // 4, APMOD_MAX_IMPORTS) or
            import_end > len(contents)):
        raise ValueError("Invalid AP_Module module %s: truncated metadata" % embedded_name)
    canonical_crc = zlib.crc32(contents[:APMOD_HEADER_SIZE - 4] + bytes(4) +
                               contents[APMOD_HEADER_SIZE:])
    if canonical_crc != crc:
        raise ValueError("Invalid AP_Module module %s: checksum mismatch" % embedded_name)

    fixups = []
    previous_end = 0
    for offset in range(fixup_start, fixup_end, APMOD_FIXUP.size):
        destination, = APMOD_FIXUP.unpack_from(contents, offset)
        if (destination < previous_end or destination % width != 0 or
                destination + width > file_size):
            raise ValueError("Invalid AP_Module module %s: invalid fixup" % embedded_name)
        target = int.from_bytes(contents[APMOD_HEADER_SIZE + destination:
                                         APMOD_HEADER_SIZE + destination + width], 'little')
        if (target > memory_size or
                (machine == APMOD_MACHINE_ARM and destination < rx_size and target >= rx_size)):
            raise ValueError("Invalid AP_Module module %s: invalid fixup target" % embedded_name)
        previous_end = destination + width
        fixups.append((destination, target))

    names = contents[import_end:]
    previous_end = rx_size
    previous_name_end = 0
    hashes = set()
    fixup_index = 0
    for offset in range(fixup_end, import_end, APMOD_IMPORT.size):
        binding_hash, destination, name_offset = APMOD_IMPORT.unpack_from(contents, offset)
        if (destination < previous_end or destination % 4 != 0 or destination + 4 > file_size or
                contents[APMOD_HEADER_SIZE + destination:APMOD_HEADER_SIZE + destination + 4] != bytes([255]) * 4):
            raise ValueError("Invalid AP_Module module %s: invalid import" % embedded_name)
        if binding_hash in hashes:
            raise ValueError("Invalid AP_Module module %s: duplicate import hash" % embedded_name)
        if name_offset != previous_name_end or name_offset >= len(names):
            raise ValueError("Invalid AP_Module module %s: invalid import name offset" % embedded_name)
        name_end = names.find(b'\0', name_offset)
        if (name_end <= name_offset or name_end - name_offset > APMOD_MAX_IMPORT_NAME or
                re.fullmatch(rb'[A-Za-z0-9_.]+', names[name_offset:name_end]) is None):
            raise ValueError("Invalid AP_Module module %s: invalid import name" % embedded_name)
        while fixup_index < len(fixups) and fixups[fixup_index][0] + width <= destination:
            fixup_index += 1
        if fixup_index < len(fixups) and fixups[fixup_index][0] < destination + 4:
            raise ValueError("Invalid AP_Module module %s: import overlaps fixup" % embedded_name)
        hashes.add(binding_hash)
        previous_end = destination + 4
        previous_name_end = name_end + 1
    if previous_name_end != len(names):
        raise ValueError("Invalid AP_Module module %s: invalid import names length" % embedded_name)
    return machine, width, rx_size, fixups


def write_apmod_asm(out, contents, idx, embedded_name, fixups, rx_size):
    '''emit an ARM module with RX absolute fixups resolved by the firmware linker'''
    replacements = []
    for destination, target in fixups:
        if destination >= rx_size:
            continue
        if target >= rx_size:
            raise ValueError("Invalid AP_Module module %s: RX fixup targets RW data" % embedded_name)
        replacements.append((APMOD_HEADER_SIZE + destination, target))

    symbol = 'ap_romfs_%u' % idx
    write_encode(out, 'extern "C" const uint8_t %s[];\n' % symbol)
    write_encode(out, 'asm(\n')

    def asm_line(line):
        escaped = line.replace('\\', '\\\\').replace('"', '\\"')
        write_encode(out, '    "%s\\n"\n' % escaped)

    asm_line('.pushsection .ap_romfs.%u,"ax",%%progbits' % idx)
    alignment_word = APMOD_HEADER.unpack_from(contents)[4]
    rx_alignment = alignment_word >> 16
    asm_line('.balign %u' % rx_alignment)
    padding = (-APMOD_HEADER_SIZE) % rx_alignment
    if padding:
        asm_line('.space %u' % padding)
    asm_line('.global %s' % symbol)
    asm_line('.type %s, %%object' % symbol)
    asm_line('%s:' % symbol)
    position = 0
    for file_offset, target in replacements:
        for start in range(position, file_offset, 64):
            asm_line('.byte ' + ','.join(str(c) for c in contents[start:min(file_offset, start + 64)]))
        asm_line('.word %s + %u' % (symbol, APMOD_HEADER_SIZE + target))
        position = file_offset + APMOD_POINTER_WIDTH_ARM
    for start in range(position, len(contents), 64):
        asm_line('.byte ' + ','.join(str(c) for c in contents[start:start + 64]))
    # Preserve the string-friendly AP_ROMFS contract outside the logical size.
    if contents[-1] != 0:
        asm_line('.byte 0')
    asm_line('.size %s, . - %s' % (symbol, symbol))
    asm_line('.popsection')
    write_encode(out, ');\n\n')


def embed_file(out, f, idx, embedded_name, uncompressed, prelink_apmods=False):
    '''embed one file'''
    try:
        contents = open(f,'rb').read()
    except Exception:
        raise Exception("Failed to embed %s" % f)

    if embedded_name.endswith("bootloader.bin"):
        # round size to a multiple of 32 bytes for bootloader, this ensures
        # it can be flashed on a STM32H7 chip
        blen = len(contents)
        pad = (32 - (blen % 32)) % 32
        if pad != 0:
            contents += bytes([0xff]*pad)
            print("Padded %u bytes for %s to %u" % (pad, embedded_name, len(contents)))

    crc = crc32(contents)
    is_apmod = embedded_name.endswith('.apmod')
    raw = uncompressed or is_apmod
    prelink = prelink_apmods and is_apmod and not uncompressed
    apmod = None
    if is_apmod:
        apmod = apmod_details(contents, embedded_name)

    if prelink:
        machine, width, rx_size, fixups = apmod
        if machine != APMOD_MACHINE_ARM or width != APMOD_POINTER_WIDTH_ARM:
            raise ValueError("Invalid AP_Module module %s: ChibiOS ROMFS requires an ARM module" % embedded_name)
        write_apmod_asm(out, contents, idx, embedded_name, fixups, rx_size)
        return crc, len(contents), len(contents), raw, True

    write_encode(out, '__EXTFLASHFUNC__ static const uint8_t ap_romfs_%u[] = {' % idx)

    if raw:
        # Terminate the physical array without changing the logical file size.
        null_terminate = not contents or contents[-1] != 0
        b = contents
    else:
        # compress it (max level, max window size, raw stream, max mem usage)
        z = zlib.compressobj(level=9, method=zlib.DEFLATED, wbits=-15, memLevel=9)
        b = z.compress(contents)
        b += z.flush()
        # decompressed data will be null terminated at runtime, nothing to do here
        null_terminate = False

    write_bytes(out, b)
    if null_terminate:
        write_encode(out, ",0" if b else "0")
    write_encode(out, '};\n\n');
    return crc, len(contents), len(b), raw, False

def crc32(bytes, crc=0):
    '''crc32 equivalent to crc32_small() from AP_Math/crc.cpp'''
    # the same polynomial as zlib's, which inverts the register on the way in
    # and out; undoing both gives crc32_small() at C speed rather than a bit
    # at a time in Python
    return zlib.crc32(bytes, crc ^ 0xFFFFFFFF) ^ 0xFFFFFFFF

def create_embedded_h(filename, files, uncompressed=False, prelink_apmods=False):
    '''create a ap_romfs_embedded.h file'''

    done = set()

    out = open(filename, "wb")
    write_encode(out, '''// generated embedded files for AP_ROMFS\n\n''')

    # remove duplicates and sort
    files = sorted(list(set(files)))
    metadata = []
    for i in range(len(files)):
        (name, filename) = files[i]
        if name in done:
            print("Duplicate ROMFS file %s" % name)
            sys.exit(1)
        done.add(name)
        try:
            metadata.append(embed_file(out, filename, i, name, uncompressed, prelink_apmods))
        except Exception as e:
            print(e)
            return False

    write_encode(out, '''const AP_ROMFS::embedded_file AP_ROMFS::files[] = {\n''')

    for i in range(len(files)):
        (name, filename) = files[i]
        crc, decompressed_size, stored_size, raw, prelinked = metadata[i]
        if stored_size > ROMFS_STORED_SIZE_MASK:
            print("ROMFS file %s is too large" % name)
            return False
        encoded_size = stored_size
        if raw:
            encoded_size |= ROMFS_FLAG_UNCOMPRESSED
        if prelinked:
            encoded_size |= ROMFS_FLAG_PRELINKED
        if raw:
            ustr = ' (uncompressed)'
        else:
            ustr = ''
        print("Embedding file %s:%s%s" % (name, filename, ustr))
        write_encode(out, '{ "%s", 0x%08x, %d, 0x%08x, ap_romfs_%u },\n' % (
            name, encoded_size, decompressed_size, crc, i))
    write_encode(out, '};\n')
    out.close()
    return True

if __name__ == '__main__':
    import sys
    flist = []
    for i in range(1, len(sys.argv)):
        f = sys.argv[i]
        flist.append((f, f))
    create_embedded_h("/tmp/ap_romfs_embedded.h", flist)
