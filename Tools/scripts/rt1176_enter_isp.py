#!/usr/bin/env python3
# AP_FLAKE8_CLEAN
"""
Put a RUNNING mr_vmu_rt1176 into the ROM Serial Downloader (ISP) mode over
SWD - the LinkServer/blhost-flashable state - WITHOUT the physical BOOT0
button (which needs the FMU module disassembled).

How: the i.MX RT1170 boot ROM exports a `runBootloader(void *arg)` function
via the ROM API tree at 0x0021001C (RM rev3, ROM APIs / Figure 10-27). Called
with arg=0xEB100000 it re-enters the ROM in Serial Downloader mode over USB
(RM Table 10-68: tag 0xEB, boot-mode[23:20]=1 serial downloader,
media[19:16]=1 USB - the RM's own Example 1). We invoke it on the live core
over SWD: halt, stage the arg in RAM, point PC at the ROM function with r0 =
&arg (AAPCS), resume. No firmware change is needed - this works on any running
image because the entry lives in immutable boot ROM.

This is the same class of mechanism PX4's v6xrt uses for "reboot to
bootloader". After it runs, the board enumerates as an NXP SDP device
(1FC9:013D) and LinkServer's SFDP flash driver sees the clean boot-state
FlexSPI it needs - so a bootloader reflash then works without BOOT0.

Limitation: needs SOME code running (or at least a responsive core) to be
halted over SWD. For a fully dead board the physical BOOT0 strap remains the
last resort.

STATUS 2026-08-14, bench-tested, NOT yet working end-to-end: the ROM call is
reached and the app is left cleanly (two real bugs fixed on the way - the
original SP was unmapped under the app's 15/1 FlexRAM banking, and the app's
interrupts were left enabled across the resume). But the ROM's own re-entry
then parks at the KNOWN warm-entry trap (pc=0x00223104, IOMUXC_LPSR_GPR26
re-arms to 0x4000 even after being cleared in-session) instead of bringing
up SDP USB - the same trap soft resets hit on this chip. The serial
downloader appears to require a genuine power-on ROM pass. Recovery is safe:
flash is untouched; a plain power-cycle boots the resident firmware. Next
step if pursued: RM ch.10 warm-entry semantics of GPR26/BootROM, and how
PX4/NXP tooling requests serial download across a reset (SRC GPR tags).

Usage:  python3 Tools/scripts/rt1176_enter_isp.py
"""

import sys

from pyocd.core.helpers import ConnectHelper

# Pin the MCU-Link by serial. Other CMSIS-DAP probes are routinely attached to
# this bench on other boards; an unpinned pyocd will prompt or pick one of them.
PROBE_SERIAL = "JUHP1E4TMRVGD"
ROM_API_TREE_ROOT = 0x0021001C   # RM Figure 10-27 (Bootloader API Tree root)
# RM Table 10-68: tag 0xEB | boot-mode 1 (serial downloader) | media 1 (USB)
RUN_BOOTLOADER_ARG = 0xEB100000
# a scratch DTCM word to hold the arg (arg is passed by POINTER)
ARG_SCRATCH_ADDR = 0x20001000     # DTCM, valid under both FlexRAM maps
# Stack for the ROM call. MUST be valid under the APP's live FlexRAM map:
# the ArduPilot app reprograms FlexRAM to 15 ITCM / 1 DTCM banks, leaving
# only 32 KB of DTCM (0x20000000..0x20008000). The first version of this
# tool used 0x20020000 - valid under the 256 KB fuse-default map, UNMAPPED
# under the app's - so the ROM function faulted on its first stack push and
# the chip lockup-reset into a normal boot instead of the serial downloader
# (observed 2026-08-14: board re-enumerated as the AP bootloader, not SDP).
ROM_CALL_MSP = 0x20007FF0         # top of the 32 KB DTCM, 8-byte aligned
# IOMUXC_GPR_GPR16 (RM ch.11): bit 2 = FLEXRAM_BANK_CFG_SEL. The app sets it
# for its 15/1 banking; clear it so the ROM runs under the fuse-default map -
# the same restore AP_HAL_Zephyr Scheduler::reboot() does before a reset.
IOMUXC_GPR_GPR16 = 0x400AC040
FLEXRAM_BANK_CFG_SEL = 1 << 2


def main():
    # target_override MUST stay the generic 'cortex_m': the mimxrt1170_cm7
    # target enumerates the dormant CM4's AHB-AP#1, which answers WAIT forever
    # and wedges the whole DAP until the target is pin-reset - the same trap
    # valid_aps=[0] exists to avoid (commit 5cdf6c6bbd). Verified the hard way
    # 2026-08-16: switching to the "correct" RT1170 target re-broke SWD until
    # zephyr_pin_reset.py cleared it.
    session = ConnectHelper.session_with_chosen_probe(
        unique_id=PROBE_SERIAL,
        target_override="cortex_m", connect_mode="attach",
        options={"frequency": 4000000, "valid_aps": [0]})
    with session:
        t = session.target
        t.halt()

        # Resolve runBootloader: *(0x0021001C) -> tree; tree[0] -> runBootloader
        tree = t.read32(ROM_API_TREE_ROOT)
        run_bootloader = t.read32(tree)          # first member of bootloader_tree_t
        print("ROM API tree @0x%08x -> tree 0x%08x -> runBootloader 0x%08x"
              % (ROM_API_TREE_ROOT, tree, run_bootloader))
        if run_bootloader == 0 or run_bootloader == 0xFFFFFFFF:
            sys.exit("runBootloader pointer looks invalid - aborting")

        # Restore the fuse-default FlexRAM banking FIRST: scratch/stack
        # addresses below must be interpreted under the map the ROM will see.
        gpr16 = t.read32(IOMUXC_GPR_GPR16)
        if gpr16 & FLEXRAM_BANK_CFG_SEL:
            t.write32(IOMUXC_GPR_GPR16, gpr16 & ~FLEXRAM_BANK_CFG_SEL)
            print("FlexRAM restored to fuse-default map (GPR16 was 0x%08x)" % gpr16)

        # Stage arg in RAM and pass its ADDRESS in r0 (AAPCS first argument).
        t.write32(ARG_SCRATCH_ADDR, RUN_BOOTLOADER_ARG)
        t.write_core_register("r0", ARG_SCRATCH_ADDR)
        # Thumb entry: set PC (bit0 is ignored by write_core_register/masked by core)
        t.write_core_register("pc", run_bootloader & ~1)
        # ROM runBootloader does not return; a valid SP keeps its own prologue
        # happy. Use the MSP explicitly (CONTROL=0) - the app may be running
        # on the PSP - and mask the app's still-armed interrupts so no app ISR
        # runs between resume and the ROM taking over.
        t.write_core_register("control", 0)
        t.write_core_register("msp", ROM_CALL_MSP)
        t.write_core_register("primask", 1)
        print("entering serial downloader (arg=0x%08x) ..." % RUN_BOOTLOADER_ARG)
        t.resume()

    print("done - board should now enumerate as NXP SDP (lsusb: 1fc9:013d).")
    print("flash the bootloader with Tools/scripts/rt1176_linkserver_flash.py "
          "<bootloader.bin> (BOOT0 no longer needed).")


if __name__ == "__main__":
    main()
