# MCUBoot A/B on Zephyr, and the state of bootloader security

MCUBoot is the Zephyr project's own bootloader, and its image format is what
the whole Zephyr update ecosystem speaks. On `mr_vmu_rt1176` AP_Bootloader
implements MCUBoot's A/B update path against that format exactly: what
`imgtool` signs is what this bootloader parses, and mcumgr over the SMP server
delivers it. There is no private variant and no ArduPilot-only step in that
chain.

The A/B path checks integrity but not authenticity: this tree carries no
verifier for the signature `imgtool` writes.

## Only one of the two boards can use A/B

Two Zephyr boards carry an AP_Bootloader of their own: `mr_vmu_rt1176` and
`CubeOrangeZephyr`. Both compile `mcuboot_ab.cpp`
(`Tools/AP_Bootloader/wscript` lists it in `bootloader_source`), but its whole
body sits behind `#if AP_BOOTLOADER_MCUBOOT_AB`, and
`Tools/AP_Bootloader/hwdef_zephyr.h` sets that to 1 for
`CONFIG_SOC_SERIES_IMXRT11XX` and 0 everywhere else. The staging side matches:
`Tools/ardupilotwaf/zephyr.py` emits the `ap_firmware_<board>.img` that goes
into slot 1 only for boards in its `_BOOTLOADER_UPLOAD_PAD_BYTES` table, and
that table lists `mr_vmu_rt1176` and nothing else. `CONFIG_MCUMGR=y` likewise
appears in only one board configuration,
`libraries/AP_HAL_Zephyr/zephyr/prj.mr_vmu_rt1176.conf`, so the H7 has no SMP
server to receive an image with.

The reason is flash, not policy. A/B needs room for two complete copies of
the application, so the application has to fit in half of what the bootloader
leaves it, rounded down to whole erase sectors.

`mr_vmu_rt1176` boots from external NOR. Its devicetree, under
`libraries/AP_HAL_Zephyr/zephyr/boards/arm/mr_vmu_rt1176/`, gives
`slot0_partition` and `slot1_partition` 3 MB each, `mcuboot_ab.cpp` uses the
same 3 MB for `AP_MCUBOOT_SLOT_SIZE`, and `zephyr.py` signs the app image with
`--slot-size 0x300000`. Those three have to agree or the copy lands at the
wrong offset. A Copter build for this board is about 1370 KB, so it sits in a
3072 KB slot with more than half of the slot still spare.

CubeOrangeZephyr does not fit. The H7 has 2 MB of internal flash, the
bootloader takes the first 128 KB (`FLASH_RESERVE_START_KB 128` in
`libraries/AP_HAL_Zephyr/hwdef/CubeOrangeZephyr/hwdef.inc`, matching
`CONFIG_FLASH_LOAD_OFFSET=0x20000` in
`libraries/AP_HAL_Zephyr/zephyr/prj.CubeOrangeZephyr.conf`), and the linker
gets one 1920 KB region at `0x08020000`. That part erases in 128 KB sectors
(`erase-block-size` in Zephyr's `st/h7/stm32h742.dtsi`, which the board's
`stm32h743Xi.dtsi` inherits), so the region is fifteen sectors: an odd number,
which cannot be split into two equal slots on sector boundaries at all. The
largest equal pair is seven sectors each, 896 KB, with one sector left over.
In 1024-byte KB:

| | KB |
| --- | --- |
| App flash region | 1920 |
| Erase sector | 128 |
| Largest equal sector-aligned slot | 896 |
| Idealised half-region slot, not attainable here | 960 |
| Copter image, `zephyr_upload.bin`, built from this tree | about 1080 |

The image is 56% of the whole region. It overruns an 896 KB slot by about
180 KB, a sixth of the image, before an MCUBoot header or TLV trailer is
counted at all, and it overruns even the unattainable 960 KB half by about
120 KB. No partition layout fixes that. That is why the non-IMXRT branch of
`hwdef_zephyr.h` leaves A/B off instead of giving it a layout that cannot
work.

Both image sizes come from local builds; the repo records neither, and this
one has already moved once, so re-measure before quoting it. The size of
`build/CubeOrangeZephyr/zephyr_upload.bin` is the image itself, and
`arm-none-eabi-size` on
`build/CubeOrangeZephyr/zephyr_build/zephyr/zephyr.elf` gives a text+data that
tracks it within a few dozen bytes.

So CubeOrangeZephyr updates its single app slot over the uploader.py protocol,
which is not an A/B path: the bootloader erases the app region and writes what
it is handed. The only check is the CRC32 the host asks for after the write
(`PROTO_GET_CRC`, `Tools/AP_Bootloader/bl_protocol.cpp`), which shows the
transfer arrived intact and says nothing about where it came from. An upload
that fails part way leaves no previous application to fall back to.

## What the bootloader does check

`Tools/AP_Bootloader/mcuboot_ab.cpp` implements overwrite-only A/B. An image
staged in slot 1 is walked as a real MCUBoot image, so the magic, the hashed
span and the TLV trailer all have to be right, and its SHA256 TLV has to match
what the bootloader computes before the image is copied over slot 0.

That is an integrity check. It catches a truncated or corrupted upload. It
does not establish who produced the image, so anything that can write slot 1
can get its image adopted.

## What is missing

The ED25519 TLV that `imgtool sign` writes is not verified. Doing so needs an
RFC 8032 Ed25519 verifier, and the monocypher in this tree is the Blake2b
flavour that ArduPilot secure boot uses, which cannot check an imgtool
signature.

Two Ed25519 flavours are involved and they are not interchangeable:

| | ArduPilot secure boot | MCUBoot / imgtool |
| --- | --- | --- |
| Flavour | Ed25519 with Blake2b | RFC 8032 Ed25519, SHA-512 |
| Signature lives in | `.apj`, APSE block | image TLV trailer |
| Key file format | `PRIVATE_KEYV1:` and base64 | PKCS#8 PEM |

The same 32-byte seed therefore yields two different public keys, one per
flavour, which is the trap to remember whenever this work restarts.

## Until then

Treat A/B as an integrity-checked update path, not a secure-boot one, and do
not describe these boards as having verified boot. Adding the verifier, the
key provisioning and the trust store belongs in its own change where it can
be reviewed as security work rather than as part of a HAL port.
