#!/usr/bin/env python3

# AP_FLAKE8_CLEAN

"""Exercise the RT1176 eDMA, DMAMUX and LPUART DMA helper without firmware.

They run entirely from the Renode monitor, no vehicle build and no simulation,
and each one localises a different piece, in increasing order of what has to be
working before it can pass. See Tools/renode/README.md, "NXP i.MX RT1176
models", for how these models are put together:

  1  the platform still builds, and nothing after edma0 fell out of it
  2  memory to memory - byte-width SERQ, the minor loop, CITER/BITER, DREQ, INT
  3  scatter/gather - the whole 32-byte TCD is replaced, DLAST_SGA included
  4  LPUART1 RX - the synthetic ReceiveDMA request, and one FIFO pop per byte
  5  LPUART1 TX - the LPUART's own UpdateTxDMA handshake terminates
  -  the synthetic idle line goes up AND comes back down again

The last two are the two hang-shaped behaviours, and both are cheap enough to
test from the monitor. A TX whose ERQ never clears is an infinite C# loop inside
one guest store; an idle line that is raised and never lowered leaves the guest
re-entering an empty LPUART ISR forever, because Renode's NVIC re-pends a
completed interrupt whose input is still high.

Rung 6 and up need a firmware image and are not run here.
"""

import os
import re
import subprocess
import sys

from pathlib import Path

import pytest

HERE = Path(__file__).resolve().parents[1]
ROOT = HERE.parents[1]
sys.path.insert(0, str(HERE))

from launch import clean_monitor_text  # noqa: E402
from process_utils import terminate_process_group  # noqa: E402

VALUE_RE = re.compile(r'^0x([0-9A-Fa-f]+)$', re.M)

# eDMA control block, 0x40070000.
CR = 0x40070000
ERQ = 0x4007000C
SERQ = 0x4007001B
CINT = 0x4007001F
INT = 0x40070024
HRS = 0x40070034
# TCD n at 0x40071000 + n*0x20; DMAMUX CHCFG[n] at 0x40074000 + n*4.
TCD = 0x40071000
CHCFG = 0x40074000
# LPUART1.
BAUD = 0x4007C010
STAT = 0x4007C014
CTRL = 0x4007C018
DATA = 0x4007C01C
FIFO = 0x4007C028
WATER = 0x4007C02C


def tcd(channel, field):
    return TCD + channel * 0x20 + field


def prelude():
    """The .cs include list the flight script uses, verbatim.

    Taken from the script rather than repeated here so that a model added to
    the platform but forgotten in the includes fails these tests too.
    """
    text = (HERE / 'scripts/ardupilot_imxrt1176.resc').read_text()
    return text.split('mach create')[0]


def tcd_image(saddr, daddr, count, next_tcd, csr, soff=1, doff=1, nbytes=1):
    """The eight little-endian words of a 32-byte TCD, as the guest builds it."""
    return [
        saddr,                          # SADDR
        soff & 0xFFFF,                  # SOFF | ATTR<<16, ATTR 0 = 8-bit/8-bit
        nbytes,                         # NBYTES
        0,                              # SLAST
        daddr,                          # DADDR
        (doff & 0xFFFF) | (count << 16),  # DOFF | CITER<<16
        next_tcd,                       # DLAST_SGA
        csr | (count << 16),            # CSR | BITER<<16
    ]


@pytest.fixture
def renode(tmp_path):
    executable = Path(os.environ.get('RENODE', ROOT / 'build/renode/renode'))
    if not executable.is_file():
        pytest.skip('build Renode or set RENODE')

    def run(script):
        path = tmp_path / 'test.resc'
        path.write_text('$repo=@%s\n%s\n%s' % (ROOT, prelude(), script))
        process = subprocess.Popen(
            [str(executable.resolve()), '--disable-xwt', '--console',
             '-e', 'include @%s' % path, '-e', 'quit'],
            cwd=ROOT, stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
            env=dict(os.environ, TMPDIR=str(tmp_path),
                     XDG_CONFIG_HOME=str(tmp_path / 'xdg')),
            start_new_session=True)
        try:
            stdout, _ = process.communicate(timeout=180)
        except subprocess.TimeoutExpired:
            terminate_process_group(process, graceful_timeout=0.2)
            stdout, _ = process.communicate()
            pytest.fail(clean_monitor_text(stdout))
        output = clean_monitor_text(stdout)
        assert process.returncode == 0, output
        assert 'There was an error' not in output, output
        assert 'error CS' not in output, output
        return output

    return run


def values(output):
    """Every bare `0x...` line the monitor printed, in order."""
    return [int(match, 16) for match in VALUE_RE.findall(output)]


MACHINE = '''
mach create "imxrt1176"
machine LoadPlatformDescription $platform
'''


def test_platform_still_builds_around_the_edma(renode):
    # Renode stops building a platform at the first bad entry and drops every
    # peripheral after it without saying so, so the check that matters is not
    # that edma0 appeared but that the entries defined after it did too.
    output = renode(MACHINE + '\nperipherals\n')
    for name in ('edma0', 'dmamux0', 'lpuart1', 'lpuart1_dmafix',
                 'lpuart10_dmafix', 'gpt2', 'usbphy2'):
        assert name in output, output


def test_memory_to_memory_transfer(renode):
    # DMAMUX A_ON is the path Zephyr's mem2mem takes, and it needs no UART.
    # CR is 0x80 because that is what EDMA_Init leaves behind: CR[EMLM] is
    # permanently set on this part, so NBYTES is decoded through the minor-loop
    # mapping mask.
    script = MACHINE + '''
sysbus WriteDoubleWord 0x20200000 0xDEADBEEF
sysbus WriteDoubleWord 0x%X 0x00000080
sysbus WriteDoubleWord 0x%X 0x20200000
sysbus WriteWord       0x%X 0x0001
sysbus WriteWord       0x%X 0x0000
sysbus WriteDoubleWord 0x%X 0x00000001
sysbus WriteDoubleWord 0x%X 0x00000000
sysbus WriteDoubleWord 0x%X 0x20201000
sysbus WriteWord       0x%X 0x0001
sysbus WriteWord       0x%X 0x0004
sysbus WriteDoubleWord 0x%X 0x00000000
sysbus WriteWord       0x%X 0x0000
sysbus WriteWord       0x%X 0x000A
sysbus WriteWord       0x%X 0x0004
sysbus WriteDoubleWord 0x%X 0xA0000000
sysbus WriteByte       0x%X 0x00
sysbus ReadDoubleWord  0x20201000
sysbus ReadDoubleWord  0x%X
sysbus ReadWord        0x%X
sysbus ReadWord        0x%X
sysbus ReadDoubleWord  0x%X
sysbus WriteByte       0x%X 0x00
sysbus ReadDoubleWord  0x%X
''' % (CR,
       tcd(0, 0x00), tcd(0, 0x04), tcd(0, 0x06), tcd(0, 0x08), tcd(0, 0x0C),
       tcd(0, 0x10), tcd(0, 0x14), tcd(0, 0x16), tcd(0, 0x18),
       tcd(0, 0x1C), tcd(0, 0x1C), tcd(0, 0x1E),
       CHCFG, SERQ,
       INT, tcd(0, 0x1C), tcd(0, 0x16), ERQ, CINT, INT)
    output = renode(script)
    assert 'Unhandled' not in output, output
    moved, interrupts, csr, citer, erq, cleared = values(output)
    assert moved == 0xDEADBEEF, output           # four 8-bit beats
    assert interrupts == 0x1, output             # CSR[INTMAJOR] -> INT bit 0
    assert csr & 0x80, output                    # DONE set at major completion
    assert not csr & 0x40, output                # ACTIVE never observable
    assert citer == 4, output                    # reloaded from BITER
    assert erq == 0, output                      # CSR[DREQ] cleared ERQ
    assert cleared == 0, output                  # CINT dropped the flag


def test_scatter_gather_replaces_the_whole_tcd(renode):
    # Three TCDs: A is installed in the registers, B and C sit in OCRAM. A links
    # to B, B links to C. B carries DREQ, so the run stops after B - which
    # leaves C loaded in the registers and lets us read back every field the
    # fetch was supposed to replace, DLAST_SGA included. edma_reload_loop reads
    # DLAST_SGA back as the identity of the TCD in hardware, so a stale one
    # sends it down the wrong branch forever.
    source = 0x20200100
    b_image, c_image = 0x20202000, 0x20202020
    a_dest, b_dest, c_dest = 0x20201100, 0x20201110, 0x20201120
    token = 0xCAFE0000

    lines = [
        'sysbus WriteDoubleWord 0x%X 0x44332211' % source,
        'sysbus WriteDoubleWord 0x%X 0x88776655' % (source + 4),
        'sysbus WriteDoubleWord 0x%X 0x00000080' % CR,
    ]
    # ESG | DREQ | INTMAJOR on B; INTMAJOR alone on C.
    for base, image in ((b_image, tcd_image(source + 2, b_dest, 2, c_image, 0x1A)),
                        (c_image, tcd_image(source + 4, c_dest, 3, token, 0x02))):
        for index, word in enumerate(image):
            lines.append('sysbus WriteDoubleWord 0x%X 0x%08X' % (base + index * 4, word))
    lines += [
        'sysbus WriteDoubleWord 0x%X 0x%08X' % (tcd(0, 0x00), source),
        'sysbus WriteWord       0x%X 0x0001' % tcd(0, 0x04),
        'sysbus WriteWord       0x%X 0x0000' % tcd(0, 0x06),
        'sysbus WriteDoubleWord 0x%X 0x00000001' % tcd(0, 0x08),
        'sysbus WriteDoubleWord 0x%X 0x00000000' % tcd(0, 0x0C),
        'sysbus WriteDoubleWord 0x%X 0x%08X' % (tcd(0, 0x10), a_dest),
        'sysbus WriteWord       0x%X 0x0001' % tcd(0, 0x14),
        'sysbus WriteWord       0x%X 0x0002' % tcd(0, 0x16),
        'sysbus WriteDoubleWord 0x%X 0x%08X' % (tcd(0, 0x18), b_image),
        # CSR = 0 first: the reference manual forces ESG to 0 when it is written
        # while DONE is set, which is why EDMA_InstallTCD writes it twice.
        'sysbus WriteWord       0x%X 0x0000' % tcd(0, 0x1C),
        'sysbus WriteWord       0x%X 0x0012' % tcd(0, 0x1C),
        'sysbus WriteWord       0x%X 0x0002' % tcd(0, 0x1E),
        'sysbus WriteDoubleWord 0x%X 0xA0000000' % CHCFG,
        'sysbus WriteByte       0x%X 0x00' % SERQ,
        'sysbus ReadDoubleWord  0x%X' % a_dest,
        'sysbus ReadDoubleWord  0x%X' % b_dest,
        'sysbus ReadDoubleWord  0x%X' % c_dest,
        'sysbus ReadDoubleWord  0x%X' % tcd(0, 0x00),
        'sysbus ReadDoubleWord  0x%X' % tcd(0, 0x18),
        'sysbus ReadWord        0x%X' % tcd(0, 0x1C),
        'sysbus ReadWord        0x%X' % tcd(0, 0x16),
        'sysbus ReadWord        0x%X' % tcd(0, 0x1E),
        'sysbus ReadDoubleWord  0x%X' % ERQ,
    ]
    output = renode(MACHINE + '\n'.join(lines) + '\n')
    assert 'Unhandled' not in output, output
    (first, second, third, saddr, next_tcd,
     csr, citer, biter, erq) = values(output)
    assert first == 0x2211, output               # A moved two bytes
    assert second == 0x4433, output              # B moved the next two
    assert third == 0, output                    # C was fetched, never run
    assert saddr == source + 4, output           # SADDR came from C's image
    assert next_tcd == token, output             # so did DLAST_SGA
    assert csr == 0x0002, output                 # and CSR - with DONE clear
    assert citer == 3 and biter == 3, output     # and CITER and BITER
    assert erq == 0, output                      # B's DREQ stopped the channel


def test_lpuart_rx_moves_bytes_into_memory(renode):
    # LPUART1 set up the way LPUART_Init leaves it - RX FIFO enabled, watermark
    # zero - which is exactly the state in which Renode's own NXP_LPUART can
    # never reach BufferState.Full and so can never assert ReceiveDMA. If this
    # moves bytes, AP_IMXRT_LPUART_DmaFix is doing its job.
    #
    # Channel 6 and DMAMUX source 9 are lpuart1's rx pair in the board
    # devicetree: dmas = <&edma0 6 9>.
    channel, source = 6, 9
    destination = 0x20201200
    script = MACHINE + '''
sysbus WriteDoubleWord 0x%X 0x00000080
sysbus WriteDoubleWord 0x%X 0x0F000010
sysbus WriteDoubleWord 0x%X 0x00000088
sysbus WriteDoubleWord 0x%X 0x00000000
sysbus WriteDoubleWord 0x%X 0x00040000
sysbus WriteDoubleWord 0x%X 0x%08X
sysbus WriteWord       0x%X 0x0000
sysbus WriteWord       0x%X 0x0000
sysbus WriteDoubleWord 0x%X 0x00000001
sysbus WriteDoubleWord 0x%X 0x00000000
sysbus WriteDoubleWord 0x%X 0x%08X
sysbus WriteWord       0x%X 0x0001
sysbus WriteWord       0x%X 0x0080
sysbus WriteDoubleWord 0x%X 0x00000000
sysbus WriteWord       0x%X 0x0000
sysbus WriteWord       0x%X 0x0002
sysbus WriteWord       0x%X 0x0080
sysbus WriteDoubleWord 0x%X 0x8000000%X
sysbus WriteByte       0x%X 0x%02X
sysbus WriteDoubleWord 0x%X 0x0F200010
sysbus.lpuart1 WriteChar 0x41
sysbus.lpuart1 WriteChar 0x42
sysbus ReadDoubleWord  0x%X
sysbus ReadWord        0x%X
sysbus ReadDoubleWord  0x%X
''' % (CR,
       BAUD,                                    # OSR 16, SBR 16
       FIFO,                                    # TXFE | RXFE, before CTRL[RE]
       WATER,                                   # watermark 0, as LPUART_Init
       CTRL,                                    # RE
       tcd(channel, 0x00), DATA,                # SADDR = &LPUART1->DATA
       tcd(channel, 0x04),                      # SOFF = 0, the FIFO does not move
       tcd(channel, 0x06),                      # ATTR = 8-bit / 8-bit
       tcd(channel, 0x08),                      # NBYTES = 1
       tcd(channel, 0x0C),
       tcd(channel, 0x10), destination,
       tcd(channel, 0x14),                      # DOFF = 1
       tcd(channel, 0x16),                      # CITER = 128
       tcd(channel, 0x18),
       tcd(channel, 0x1C),                      # clear DONE first
       tcd(channel, 0x1C),                      # INTMAJOR
       tcd(channel, 0x1E),                      # BITER = 128
       CHCFG + channel * 4, source,             # ENBL | SOURCE
       SERQ, channel,
       BAUD,                                    # BAUD |= RDMAE
       destination, tcd(channel, 0x16), HRS)
    output = renode(script)
    assert 'Unhandled' not in output, output
    # An empty read of the data register would mean the request level was not
    # withdrawn when the FIFO drained.
    assert 'empty fifo' not in output, output
    received, citer, hrs = values(output)
    assert received == 0x4241, output            # 'A' then 'B', DOFF applied
    assert citer == 126, output                  # decremented live, per byte
    assert hrs == 0, output                      # level dropped with the FIFO


def test_lpuart_tx_drains_the_whole_buffer_in_one_request(renode):
    # Rung 5. The LPUART raises TransmitDMA from inside its own BAUD write and
    # then loops `while(TransmitDmaState)`, re-pulsing the line until nobody
    # answers. The only thing that ends that loop is our DREQ handling clearing
    # ERQ at major completion - so a missed DREQ, or a CITER that never reaches
    # zero, is an infinite C# loop with virtual time stopped. This test
    # terminating at all is half of what it proves.
    #
    # Channel 7 and DMAMUX source 8 are lpuart1's tx pair in the board
    # devicetree: dmas = <&edma0 6 9>, <&edma0 7 8>.
    channel, source, length = 7, 8, 4
    buffer = 0x20200200
    script = MACHINE + '''
sysbus WriteDoubleWord 0x%X 0x44434241
sysbus WriteDoubleWord 0x%X 0x00000080
sysbus WriteDoubleWord 0x%X 0x0F000010
sysbus WriteDoubleWord 0x%X 0x00000088
sysbus WriteDoubleWord 0x%X 0x00000000
sysbus WriteDoubleWord 0x%X 0x00080000
sysbus WriteDoubleWord 0x%X 0x%08X
sysbus WriteWord       0x%X 0x0001
sysbus WriteWord       0x%X 0x0000
sysbus WriteDoubleWord 0x%X 0x00000001
sysbus WriteDoubleWord 0x%X 0x00000000
sysbus WriteDoubleWord 0x%X 0x%08X
sysbus WriteWord       0x%X 0x0000
sysbus WriteWord       0x%X 0x000%X
sysbus WriteDoubleWord 0x%X 0x00000000
sysbus WriteWord       0x%X 0x0000
sysbus WriteWord       0x%X 0x000A
sysbus WriteWord       0x%X 0x000%X
sysbus WriteDoubleWord 0x%X 0x8000000%X
sysbus WriteByte       0x%X 0x%02X
sysbus WriteDoubleWord 0x%X 0x0F800010
sysbus ReadDoubleWord  0x%X
sysbus ReadWord        0x%X
sysbus ReadWord        0x%X
sysbus ReadWord        0x%X
sysbus ReadDoubleWord  0x%X
sysbus ReadDoubleWord  0x%X
''' % (buffer,
       CR,
       BAUD,                                    # OSR 16, SBR 16, TDMAE off
       FIFO,                                    # TXFE | RXFE
       WATER,
       CTRL,                                    # transmitter enable, so the bytes go
       tcd(channel, 0x00), buffer,              # SADDR = the buffer
       tcd(channel, 0x04),                      # SOFF = 1
       tcd(channel, 0x06),                      # ATTR = 8-bit / 8-bit
       tcd(channel, 0x08),                      # NBYTES = 1
       tcd(channel, 0x0C),
       tcd(channel, 0x10), DATA,                # DADDR = &LPUART1->DATA
       tcd(channel, 0x14),                      # DOFF = 0, the FIFO does not move
       tcd(channel, 0x16), length,              # CITER
       tcd(channel, 0x18),
       tcd(channel, 0x1C),                      # clear DONE first
       tcd(channel, 0x1C),                      # DREQ | INTMAJOR
       tcd(channel, 0x1E), length,              # BITER
       CHCFG + channel * 4, source,             # ENBL | SOURCE
       SERQ, channel,
       BAUD,                                    # BAUD |= TDMAE - this starts it
       ERQ, tcd(channel, 0x16), tcd(channel, 0x1E), tcd(channel, 0x1C),
       INT, tcd(channel, 0x00))
    output = renode(script)
    assert 'Unhandled' not in output, output
    erq, citer, biter, csr, interrupts, saddr = values(output)
    assert erq == 0, output                      # DREQ cleared it, ending the loop
    assert citer == length and biter == length, output   # reloaded from BITER
    assert csr & 0x80, output                    # DONE
    assert not csr & 0x40, output                # ACTIVE never observable
    assert interrupts == 1 << channel, output    # INTMAJOR -> NVIC line 7
    assert saddr == buffer + length, output      # every byte moved, once


def test_synthetic_idle_line_is_lowered_when_the_guest_clears_it(renode):
    # The other hang. NXP_LPUART recomputes its interrupt line in a private
    # UpdateInterrupt(), and its STAT register has no write callback - so the
    # one register the guest's idle handler writes,
    # LPUART_ClearStatusFlags(base, kLPUART_IdleLineFlag), recomputes nothing.
    # A helper that only ever raised the line would leave it high: Renode's NVIC
    # re-pends a completed interrupt whose input is still asserted, so
    # mcux_lpuart_isr would re-enter forever on a STAT with no flag set and the
    # guest would stop making progress after its first short RX frame.
    #
    # CTRL is RE | ILIE, which is what mcux_lpuart_rx_enable leaves behind, and
    # the CPU is halted because this needs virtual time to advance and not a
    # core executing an empty ITCM.
    #
    # No 'Unhandled' assertion here on purpose: STAT[IDLE] is a TaggedFlag in
    # NXP_LPUART, so the guest's own acknowledgement always logs one warning
    # before our after-write hook sees the value.
    script = MACHINE + '''
cpu IsHalted true
sysbus WriteDoubleWord 0x%X 0x0F000010
sysbus WriteDoubleWord 0x%X 0x00000088
sysbus WriteDoubleWord 0x%X 0x00000000
sysbus WriteDoubleWord 0x%X 0x00140000
sysbus.lpuart1 WriteChar 0x41
emulation RunFor "0.01"
echo "MARK-RAISED"
sysbus.lpuart1 IRQ
sysbus ReadDoubleWord 0x%X
sysbus WriteDoubleWord 0x%X 0x00100000
echo "MARK-CLEARED"
sysbus.lpuart1 IRQ
sysbus ReadDoubleWord 0x%X
''' % (BAUD, FIFO, WATER, CTRL,     # OSR/SBR, TXFE|RXFE, watermark 0, RE|ILIE
       STAT, STAT, STAT)
    output = renode(script)
    raised = output.split('MARK-RAISED')[1].split('MARK-CLEARED')[0]
    cleared = output.split('MARK-CLEARED')[1]
    assert 'GPIO: set' in raised, output          # one character gap, then IDLE
    assert '0x00F00000' in raised, output         # STAT[IDLE] visible to the ISR
    assert 'GPIO: unset' in cleared, output       # and the line comes back down
    assert '0x00E00000' in cleared, output        # with the flag gone
