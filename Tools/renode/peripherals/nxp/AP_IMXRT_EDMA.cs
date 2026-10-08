//
// The RT1176 classic eDMA and its DMAMUX, to the depth ArduPilot's UART
// traffic needs.
//
// Renode ships DMA.NXP_eDMA, but that is the eDMA4 found on the RT700/RT798:
// it reserves 4 KiB of channel registers per channel, so mapping 32 of them
// from 0x40071000 runs straight over lpuart1 at 0x4007c000. The RT1176 has the
// earlier design - one 32-byte TCD per channel at 0x1000 + n*0x20, a separate
// DMAMUX at 0x40074000 - which that model does not describe at all. Hence a
// from-scratch decode here.
//
// Which guest code depends on what is spelled out at each rule below. The four
// that a "tidier" rewrite would get wrong, and what each one costs:
//
//   * No [AllowedTranslations]. dma_start() issues a single `strb` to SERQ at
//     0x4007001B. Widening that into a read-modify-write of the word at 0x18
//     would execute CEEI, SEEI and CERQ on channel 0 as a side effect, because
//     each byte in 0x18..0x1F is an independent command register, not a field.
//     All three access widths are therefore implemented natively.
//
//   * No DoubleWordRegisterCollection for the TCDs. 32 TCDs x 12 fields at
//     0x1000 + n*0x20 collides with RegisterCollection.AddRegisterInner's
//     silent RegisterSelector aliasing, and buys nothing here: plain arrays and
//     a hand-written offset switch are shorter and say what they mean.
//
//   * No DmaEngine/IssueCopy. IssueCopy throws when Size % width != 0, and its
//     bulk path would read a UART data register once per byte *address* rather
//     than once per beat. The minor loop below is an explicit beat loop at
//     exactly SSIZE/DSIZE width, with this peripheral as the bus context.
//
//   * No machine.ScheduleAction for the request itself. A peripheral's DMA
//     request here is a re-pulsed LEVEL that Renode's NXP_LPUART raises from
//     inside its own bus access and holds high until somebody moves a byte
//     (NXP_LPUART.UpdateTxDMA / UpdateRxDMA). The transfer has to run
//     synchronously inside OnGPIO or that handshake never completes: the
//     LPUART's `while(TransmitDmaState)` loop only exits because our DREQ
//     handling clears ERQ. Deferring the copy gives exactly one byte and then
//     silence. The per-channel level latch below IS the HRS register, which is
//     also what lets a channel start on SERQ when the line is already high.
//     (ScheduleAction is right for the synthetic idle line in
//     AP_IMXRT_LPUART_DmaFix.cs - that one is a genuine timeout.)
//
// Known divergence from silicon, accepted: after a completed TX the LPUART
// leaves TransmitDMA Set, so requestLevel stays true and the next SERQ starts
// the channel one BAUD write earlier than hardware would. Functionally
// identical - the LPUART accepts data writes with TDMAE clear and TDRE is
// genuinely true - and only visible to someone instrumenting byte timing.
//
// Errors are never fabricated. ES and ERR are pinned to 0 and the error output
// is never asserted, because EDMA_GetDefaultConfig sets enableHaltOnError, so
// CR[HOE] is 1 and a single invented error would set CR[HALT] and stop all 32
// channels - and dma_mcux_edma_error_irq_handler aborts every channel the shim
// believes is busy without ever reading ERR. An unserviceable TCD logs a
// warning and drops the request instead.
//
using System;
using System.Collections.Generic;
using Antmicro.Renode.Core;
using Antmicro.Renode.Exceptions;
using Antmicro.Renode.Logging;
using Antmicro.Renode.Peripherals;
using Antmicro.Renode.Peripherals.Bus;

namespace Antmicro.Renode.Peripherals.DMA
{
    public class AP_IMXRT_EDMA : IDoubleWordPeripheral, IWordPeripheral, IBytePeripheral,
                                 IKnownSize, INumberedGPIOOutput
    {
        public AP_IMXRT_EDMA(IMachine machine, int numberOfChannels = 32)
        {
            if(numberOfChannels < 1 || numberOfChannels > MaxChannels)
            {
                throw new ConstructionException(
                    "AP_IMXRT_EDMA supports 1 to 32 channels, given: " + numberOfChannels);
            }
            this.machine = machine;
            this.numberOfChannels = numberOfChannels;
            channelMask = numberOfChannels == 32 ? 0xFFFFFFFFu : ((1u << numberOfChannels) - 1);

            tcd = new Tcd[MaxChannels];
            for(var i = 0; i < MaxChannels; i++)
            {
                tcd[i] = new Tcd();
            }
            dchpri = new byte[MaxChannels];
            requestLevel = new bool[MaxChannels];
            servicing = new bool[MaxChannels];
            beatsMoved = new ulong[MaxChannels];

            // 0..15 are the channel-complete lines - channel n and channel n+16
            // share line n mod 16, which is what irq-shared-offset = <16> in
            // nxp_rt11xx.dtsi means. 16 is the single error line for all 32
            // channels and is never asserted; see the file header.
            var connections = new Dictionary<int, IGPIO>();
            for(var line = 0; line <= ErrorLine; line++)
            {
                connections[line] = new GPIO();
            }
            Connections = connections;

            Reset();
        }

        public IReadOnlyDictionary<int, IGPIO> Connections { get; }

        public long Size => 0x4000;

        // Diagnostics, not silicon. The question "has this model ever moved a
        // byte for a real guest, as opposed to for a monitor script?" cannot be
        // answered from the register file - the guest's own counters live in its
        // memory, and a boot that reaches a heartbeat does not by itself say
        // whether the bytes went through the eDMA or the interrupt path. These
        // are readable from the monitor as `sysbus.edma0 BytesMoved` and
        // `sysbus.edma0 BeatsPerChannel`. They are how you tell a heartbeat that
        // came through the eDMA from one that came through the interrupt path;
        // Tools/renode/tests/zephyr_boot_check.py --assert-edma checks them.
        // One add per beat; nothing reads them but a human.
        public ulong BytesMoved { get; private set; }

        public string BeatsPerChannel
        {
            get
            {
                var parts = new List<string>();
                for(var i = 0; i < numberOfChannels; i++)
                {
                    if(beatsMoved[i] != 0)
                    {
                        parts.Add(string.Format("ch{0}={1}", i, beatsMoved[i]));
                    }
                }
                return parts.Count == 0 ? "no channel has moved a beat" : string.Join(" ", parts);
            }
        }

        public void Reset()
        {
            cr = 0;
            erq = 0;
            eei = 0;
            intFlags = 0;
            ears = 0;
            BytesMoved = 0;
            for(var i = 0; i < MaxChannels; i++)
            {
                tcd[i].Clear();
                requestLevel[i] = false;
                servicing[i] = false;
                beatsMoved[i] = 0;
                // DCHPRIn is byte-addressed with each group of four swapped:
                // channel 3 sits at 0x100, channel 0 at 0x103. dchpri is indexed
                // by raw offset, and the reset value of each register is its own
                // channel number, so offset i holds i ^ 3.
                dchpri[i] = (byte)(i ^ 3);
            }
            foreach(var connection in Connections)
            {
                connection.Value.Unset();
            }
        }

        // Called by AP_IMXRT_DMAMUX only. `level` is a level, not an edge: it
        // stays true until the peripheral withdraws it, which for RX happens
        // from inside our own read of the LPUART data register.
        public void SetRequest(int channel, bool level)
        {
            if(channel < 0 || channel >= numberOfChannels)
            {
                return;
            }
            requestLevel[channel] = level;
            if(level)
            {
                Reevaluate(channel);
            }
        }

        public uint ReadDoubleWord(long offset)
        {
            if(offset >= TcdBase)
            {
                return TcdRead(offset - TcdBase, 4);
            }
            if(offset >= DchpriBase && offset < DchpriBase + MaxChannels)
            {
                // Bounds-checked per byte: a misaligned 32-bit access near the
                // top of the array would otherwise index past it and throw
                // inside a bus access. Nothing else in this file faults.
                return Dchpri(offset - DchpriBase)
                    | ((uint)Dchpri(offset - DchpriBase + 1) << 8)
                    | ((uint)Dchpri(offset - DchpriBase + 2) << 16)
                    | ((uint)Dchpri(offset - DchpriBase + 3) << 24);
            }
            switch(offset)
            {
            case CrOffset:
                // VERSION and ACTIVE read 0 deliberately. EDMA_Init does a
                // read-modify-write of the whole register (fsl_edma.c), so any
                // bit we invent here is written straight back into the field.
                return cr;
            case EsOffset:
                return 0;
            case ErqOffset:
                return erq;
            case EeiOffset:
                return eei;
            case IntOffset:
                return intFlags;
            case ErrOffset:
                return 0;
            case HrsOffset:
                // HRS is the latched request level ANDed with ERQ - the same
                // latch the engine services from. Reviewers split on whether
                // silicon gates HRS on ERQ or reports the raw line; the RM's own
                // note is that the bit indicates a request that is asserted AND
                // enabled, so the AND stays. Nothing but dma_mcux_edma's debug
                // dump reads this register, so either reading is only ever
                // visible in a CONFIG_LOG=y build.
                return HardwareRequestStatus();
            case EarsOffset:
                return ears;
            case CommandsOffset:
            case CommandsOffset + 4:
                // The command bytes read back as zero on silicon.
                return 0;
            default:
                this.LogUnhandledRead(offset);
                return 0;
            }
        }

        public void WriteDoubleWord(long offset, uint value)
        {
            if(offset >= TcdBase)
            {
                TcdWrite(offset - TcdBase, 4, value);
                return;
            }
            if(offset >= DchpriBase && offset < DchpriBase + MaxChannels)
            {
                for(var i = 0; i < 4; i++)
                {
                    SetDchpri(offset - DchpriBase + i, (byte)(value >> (8 * i)));
                }
                return;
            }
            switch(offset)
            {
            case CrOffset:
                WriteControl(value);
                return;
            case EsOffset:
            case ErrOffset:
                // ES is read-only and ERR is write-1-to-clear over a register
                // that is always 0. EDMA_Init writes 0xFFFFFFFF to ERR as a
                // clear, so this must be silently accepted, not logged.
                return;
            case ErqOffset:
                erq = value & channelMask;
                ReevaluateAll();
                return;
            case EeiOffset:
                // A real read/write register: EDMA_EnableChannelInterrupts does
                // a plain 32-bit read-modify-write of it.
                eei = value & channelMask;
                return;
            case CommandsOffset:
            case CommandsOffset + 4:
                // Four independent command registers in one word. Each byte
                // carries its own NOP bit and each lane is decoded on its own -
                // that is exactly what the NOP bit is for.
                for(var lane = 0; lane < 4; lane++)
                {
                    Command(offset + lane, (byte)(value >> (8 * lane)));
                }
                return;
            case IntOffset:
                // Write-1-to-clear. EDMA_Init writes 0xFFFFFFFF here as a clear.
                intFlags &= ~value;
                UpdateIrqLines();
                return;
            case HrsOffset:
                return;
            case EarsOffset:
                ears = value & channelMask;
                return;
            default:
                this.LogUnhandledWrite(offset, value);
                return;
            }
        }

        public ushort ReadWord(long offset)
        {
            if(offset >= TcdBase)
            {
                return (ushort)TcdRead(offset - TcdBase, 2);
            }
            if(offset >= CommandsOffset && offset < CommandsOffset + 8)
            {
                return 0;
            }
            // Anything else in the control block is a half of a 32-bit register.
            // The guest never does this; support it rather than fault.
            var aligned = offset & ~3L;
            var shift = (int)(offset & 2) * 8;
            return (ushort)(ReadDoubleWord(aligned) >> shift);
        }

        public void WriteWord(long offset, ushort value)
        {
            if(offset >= TcdBase)
            {
                TcdWrite(offset - TcdBase, 2, value);
                return;
            }
            if(offset >= CommandsOffset && offset < CommandsOffset + 8)
            {
                // Two independent command registers, decoded separately - never
                // a read-modify-write of the containing word. See the header.
                Command(offset, (byte)value);
                Command(offset + 1, (byte)(value >> 8));
                return;
            }
            if(offset >= DchpriBase && offset < DchpriBase + MaxChannels)
            {
                SetDchpri(offset - DchpriBase, (byte)value);
                SetDchpri(offset - DchpriBase + 1, (byte)(value >> 8));
                return;
            }
            var aligned = offset & ~3L;
            var shift = (int)(offset & 2) * 8;
            var merged = (ReadDoubleWord(aligned) & ~(0xFFFFu << shift)) | ((uint)value << shift);
            this.Log(LogLevel.Debug, "16-bit write to control register 0x{0:X}, merged into 0x{1:X}", offset, aligned);
            WriteDoubleWord(aligned, merged);
        }

        public byte ReadByte(long offset)
        {
            if(offset >= TcdBase)
            {
                return (byte)TcdRead(offset - TcdBase, 1);
            }
            if(offset >= CommandsOffset && offset < CommandsOffset + 8)
            {
                return 0;
            }
            if(offset >= DchpriBase && offset < DchpriBase + MaxChannels)
            {
                return Dchpri(offset - DchpriBase);
            }
            var aligned = offset & ~3L;
            var shift = (int)(offset & 3) * 8;
            return (byte)(ReadDoubleWord(aligned) >> shift);
        }

        public void WriteByte(long offset, byte value)
        {
            if(offset >= TcdBase)
            {
                TcdWrite(offset - TcdBase, 1, value);
                return;
            }
            if(offset >= CommandsOffset && offset < CommandsOffset + 8)
            {
                // dma_start() is a single strb to SERQ at 0x4007001B and this is
                // where it lands. Never widen it.
                Command(offset, value);
                return;
            }
            if(offset >= DchpriBase && offset < DchpriBase + MaxChannels)
            {
                SetDchpri(offset - DchpriBase, value);
                return;
            }
            var aligned = offset & ~3L;
            var shift = (int)(offset & 3) * 8;
            var merged = (ReadDoubleWord(aligned) & ~(0xFFu << shift)) | ((uint)value << shift);
            this.Log(LogLevel.Debug, "8-bit write to control register 0x{0:X}, merged into 0x{1:X}", offset, aligned);
            WriteDoubleWord(aligned, merged);
        }

        private byte Dchpri(long index)
        {
            return (index >= 0 && index < MaxChannels) ? dchpri[index] : (byte)0;
        }

        private void SetDchpri(long index, byte value)
        {
            if(index >= 0 && index < MaxChannels)
            {
                dchpri[index] = value;
            }
        }

        private void WriteControl(uint value)
        {
            cr = value & CrWriteMask;
            // ECX and CX are self-clearing cancel requests. Nothing here can be
            // cancelled - transfers are instantaneous - so drop them at once
            // rather than leaving a bit set that the guest would read back.
            cr &= ~(CrEcx | CrCx);
            if((cr & CrHalt) != 0)
            {
                this.Log(LogLevel.Warning, "CR[HALT] set by software - all channels are stalled until it is cleared");
            }
        }

        private uint HardwareRequestStatus()
        {
            var status = 0u;
            for(var channel = 0; channel < numberOfChannels; channel++)
            {
                if(requestLevel[channel] && (erq & (1u << channel)) != 0)
                {
                    status |= 1u << channel;
                }
            }
            return status;
        }

        private void Command(long offset, byte value)
        {
            if((value & CommandNop) != 0)
            {
                return;
            }
            var all = (value & CommandAll) != 0;
            var first = all ? 0 : (value & 0x1F);
            var last = all ? numberOfChannels - 1 : (value & 0x1F);
            for(var channel = first; channel <= last; channel++)
            {
                if(channel >= numberOfChannels)
                {
                    continue;
                }
                switch(offset)
                {
                case CommandsOffset + 0:     // CEEI
                    eei &= ~(1u << channel);
                    break;
                case CommandsOffset + 1:     // SEEI
                    eei |= 1u << channel;
                    break;
                case CommandsOffset + 2:     // CERQ
                    erq &= ~(1u << channel);
                    break;
                case CommandsOffset + 3:     // SERQ - the one bit that starts a channel
                    erq |= 1u << channel;
                    Reevaluate(channel);
                    break;
                case CommandsOffset + 4:     // CDNE
                    tcd[channel].csr &= unchecked((ushort)~CsrDone);
                    break;
                case CommandsOffset + 5:     // SSRT - software start, one minor loop
                    SoftwareRequest(channel);
                    break;
                case CommandsOffset + 6:     // CERR - ERR is always 0, nothing to clear
                    break;
                case CommandsOffset + 7:     // CINT
                    intFlags &= ~(1u << channel);
                    UpdateIrqLines();
                    break;
                }
            }
        }

        // A software start (SSRT, or CSR[START]) is a one-shot, not a level. Run
        // one minor loop, then let any level that is genuinely pending drain.
        private void SoftwareRequest(int channel)
        {
            // CR[HALT] stops every channel, software starts included. Reevaluate
            // refuses for the same reason; the two paths have to agree. Nothing
            // in this firmware sets HALT - errors are never fabricated - so this
            // is consistency, not a live path.
            if((cr & CrHalt) != 0)
            {
                return;
            }
            if(servicing[channel])
            {
                return;
            }
            servicing[channel] = true;
            try
            {
                RunMinorLoop(channel);
            }
            finally
            {
                servicing[channel] = false;
            }
            Reevaluate(channel);
        }

        private void ReevaluateAll()
        {
            for(var channel = 0; channel < numberOfChannels; channel++)
            {
                Reevaluate(channel);
            }
        }

        private void Reevaluate(int channel)
        {
            if((cr & CrHalt) != 0)
            {
                return;
            }
            // servicing[] suppresses recursion only. The level itself is always
            // updated by SetRequest, because the peripheral withdraws it from
            // inside our own bus access - that is how the loop below ends.
            if(servicing[channel])
            {
                return;
            }
            servicing[channel] = true;
            try
            {
                var guard = 0;
                while(requestLevel[channel] && (erq & (1u << channel)) != 0)
                {
                    RunMinorLoop(channel);
                    if(++guard > MaxMinorLoopsPerRequest)
                    {
                        // Without this a missed DREQ or a CITER that never
                        // reaches zero becomes an infinite C# loop with virtual
                        // time frozen, which reads as a hung guest and is not
                        // one.
                        this.Log(LogLevel.Error,
                            "channel {0}: {1} minor loops without the request being withdrawn - dropping it",
                            channel, guard);
                        requestLevel[channel] = false;
                        break;
                    }
                }
            }
            finally
            {
                servicing[channel] = false;
            }
        }

        private void RunMinorLoop(int channel)
        {
            var t = tcd[channel];
            var sourceSize = TransferSize((t.attr >> 8) & 7);
            var destinationSize = TransferSize(t.attr & 7);
            // CR[EMLM] is permanently 1 here - EDMA_Init sets it - so NBYTES
            // carries the minor-loop-offset fields and has to be masked out of
            // the byte count. The offsets themselves are not modelled; nothing
            // in Zephyr's shim reaches EDMA_SetMinorOffsetConfig. Note what the
            // mask does NOT do: 0x3FFFFFFF is still a gigabyte, which is what
            // the size cap below is for.
            var minorBytes = (cr & CrEmlm) != 0
                ? (((t.nbytes & (NbytesSmloe | NbytesDmloe)) != 0) ? (t.nbytes & 0x3FFu) : (t.nbytes & 0x3FFFFFFFu))
                : t.nbytes;
            var current = CountOf(t.citer);

            // ATTR SSIZE/DSIZE 5 is a 32-byte burst; nothing on this part
            // programs one, so refuse rather than invent a transfer.
            if(minorBytes == 0 || current == 0 || sourceSize == 0 || destinationSize == 0
               || sourceSize > 8 || destinationSize > 8)
            {
                this.Log(LogLevel.Warning,
                    "channel {0}: unserviceable TCD (NBYTES {1}, CITER {2}, ATTR 0x{3:X}) - dropping the request",
                    channel, minorBytes, current, t.attr);
                requestLevel[channel] = false;
                return;
            }
            if(sourceSize != destinationSize)
            {
                this.Log(LogLevel.Warning,
                    "channel {0}: SSIZE {1} != DSIZE {2}; the beat loop steps by SSIZE",
                    channel, sourceSize, destinationSize);
            }
            // The iteration cap in Reevaluate counts minor LOOPS, and a single
            // minor loop is not bounded by it: with CR[EMLM] set and the
            // minor-loop-offset bits clear, NBYTES decodes to anything up to
            // 0x3FFFFFFF. A stale scatter/gather fetch out of uninitialised
            // memory would then run a billion bus accesses inside one guest
            // store, with virtual time stopped - which reads as a hung firmware
            // and is not one. Nothing on this part programs a minor loop
            // anywhere near this size (UART and SPI use 1, I2C 2), so refuse
            // rather than move a partial, inconsistent transfer.
            if(minorBytes > MaxBytesPerMinorLoop)
            {
                this.Log(LogLevel.Warning,
                    "channel {0}: NBYTES {1} exceeds the {2}-byte minor-loop cap - dropping the request",
                    channel, minorBytes, MaxBytesPerMinorLoop);
                requestLevel[channel] = false;
                return;
            }

            // Silicon clears DONE once the channel is running again; clearing
            // it on every minor loop is the same thing, since it can only be set
            // at a major completion. Every path this firmware takes happens to
            // clear it some other way first - TX through EDMA_CreateHandle's
            // zeroing, RX through the scatter/gather fetch, edma_reload_loop
            // explicitly - but a path that did not would fail silently and
            // oddly: EDMA_GetRemainingMajorLoopCount short-circuits to 0 while
            // DONE is set, so dma_get_status reports pending_length 0 and
            // mcux_lpuart_async_rx_flush stops delivering partial frames while
            // the channel still looks perfectly healthy.
            t.csr &= unchecked((ushort)~CsrDone);

            for(uint moved = 0; moved < minorBytes; moved += (uint)sourceSize)
            {
                CopyBeat(t.saddr, t.daddr, sourceSize, destinationSize);
                BytesMoved += (ulong)sourceSize;
                beatsMoved[channel]++;
                t.saddr = (uint)(t.saddr + (short)t.soff);
                t.daddr = (uint)(t.daddr + (short)t.doff);
            }

            current--;
            StoreCount(t, current);
            if(current != 0)
            {
                if((t.csr & CsrIntHalf) != 0 && current == (ushort)(CountOf(t.biter) >> 1))
                {
                    intFlags |= 1u << channel;
                    UpdateIrqLines();
                }
                return;
            }

            // Major loop complete. DREQ belongs to the TCD that just COMPLETED,
            // not to the one scatter/gather is about to fetch - which is why RX
            // keeps running: edma_reload_loop clears the installed TCD's DREQ
            // (EDMA_EnableAutoStopRequest false) before this point is reached.
            var autoStop = (t.csr & CsrDreq) != 0;
            var scatterGather = (t.csr & CsrEsg) != 0;
            var interruptOnMajor = (t.csr & CsrIntMajor) != 0;
            if(autoStop)
            {
                erq &= ~(1u << channel);
            }
            if(scatterGather)
            {
                LoadTcdFromMemory(channel, (uint)t.dlastSga);
            }
            else
            {
                t.saddr = (uint)(t.saddr + t.slast);
                t.daddr = (uint)(t.daddr + t.dlastSga);
                // Mandatory, and the reason is not the one it is easy to
                // assume. EDMA_SubmitTransfer does refuse while CITER != BITER,
                // but it cannot see a stale count: dma_mcux_edma_configure calls
                // reset_channel -> EDMA_CreateHandle before every submit, and
                // that zeroes CITER and BITER first. What does read the live
                // count is EDMA_GetRemainingMajorLoopCount, which dma_get_status
                // reports as pending_length and mcux_lpuart_async_rx_flush turns
                // into "how much of this buffer arrived". Without the reload the
                // count reads 0 after every major loop and partial RX delivery
                // silently stops - and edma_reload_dynamic, which no LPUART path
                // reaches today, would get the -EFAULT.
                StoreCount(t, CountOf(t.biter));
                t.csr |= CsrDone;
            }
            if(interruptOnMajor)
            {
                intFlags |= 1u << channel;
                UpdateIrqLines();
            }
        }

        // One beat, at exactly SSIZE and DSIZE, with this peripheral as the bus
        // context. Sizes above 8 are rejected before the beat loop starts.
        private void CopyBeat(uint source, uint destination, int sourceSize, int destinationSize)
        {
            var bus = machine.SystemBus;
            ulong value;
            switch(sourceSize)
            {
            case 1:
                value = bus.ReadByte(source, this);
                break;
            case 2:
                value = bus.ReadWord(source, this);
                break;
            case 4:
                value = bus.ReadDoubleWord(source, this);
                break;
            default:
                value = bus.ReadDoubleWord(source, this)
                    | ((ulong)bus.ReadDoubleWord(source + 4, this) << 32);
                break;
            }
            switch(destinationSize)
            {
            case 1:
                bus.WriteByte(destination, (byte)value, this);
                break;
            case 2:
                bus.WriteWord(destination, (ushort)value, this);
                break;
            case 4:
                bus.WriteDoubleWord(destination, (uint)value, this);
                break;
            default:
                bus.WriteDoubleWord(destination, (uint)value, this);
                bus.WriteDoubleWord(destination + 4, (uint)(value >> 32), this);
                break;
            }
        }

        // Replaces the WHOLE 32-byte TCD, including DLAST_SGA, CSR and BITER.
        // edma_reload_loop reads DLAST_SGA back through EDMA_GetNextTCDAddress
        // and uses it as the identity of the TCD currently in hardware; leaving
        // a stale value there sends it down the wrong branch forever. The
        // fetched CSR arrives with DONE clear, which is what keeps
        // mcux_lpuart_async_rx_flush's "buf_len - CITER" honest.
        private void LoadTcdFromMemory(int channel, uint address)
        {
            var bus = machine.SystemBus;
            var words = new uint[8];
            for(var i = 0; i < 8; i++)
            {
                words[i] = bus.ReadDoubleWord(address + (ulong)(i * 4), this);
            }
            var t = tcd[channel];
            t.saddr = words[0];
            t.soff = (ushort)words[1];
            t.attr = (ushort)(words[1] >> 16);
            t.nbytes = words[2];
            t.slast = (int)words[3];
            t.daddr = words[4];
            t.doff = (ushort)words[5];
            t.citer = (ushort)(words[5] >> 16);
            t.dlastSga = (int)words[6];
            t.csr = (ushort)(words[7] & unchecked((uint)~CsrActive));
            t.biter = (ushort)(words[7] >> 16);
        }

        private void UpdateIrqLines()
        {
            for(var line = 0; line < SharedLines; line++)
            {
                Connections[line].Set(((intFlags >> line) & 1) != 0
                                      || ((intFlags >> (line + SharedLines)) & 1) != 0);
            }
            // Never asserted - see the file header.
            Connections[ErrorLine].Unset();
        }

        private uint TcdRead(long offset, int width)
        {
            var channel = (int)(offset / TcdStride);
            var field = offset % TcdStride;
            if(channel >= numberOfChannels)
            {
                this.LogUnhandledRead(TcdBase + offset);
                return 0;
            }
            if(width == 4)
            {
                return TcdReadWord(channel, field & ~3L);
            }
            var half = TcdReadHalf(channel, field & ~1L);
            return width == 2 ? half : (uint)((half >> ((int)(field & 1) * 8)) & 0xFF);
        }

        // Byte and half-word writes are resolved against the natural 16-bit
        // field, never against the containing 32-bit word. Merging BITER into
        // the word at +0x1C and writing that back would re-run the CSR write
        // rule with the stored CSR value, which can quietly strip ESG once DONE
        // has been set.
        private void TcdWrite(long offset, int width, uint value)
        {
            var channel = (int)(offset / TcdStride);
            var field = offset % TcdStride;
            if(channel >= numberOfChannels)
            {
                this.LogUnhandledWrite(TcdBase + offset, value);
                return;
            }
            if(width == 4)
            {
                TcdWriteWord(channel, field & ~3L, value);
                return;
            }
            if(width == 1)
            {
                var pair = field & ~1L;
                var shift = (int)(field & 1) * 8;
                var mergedByte = (ushort)((TcdReadHalf(channel, pair) & ~(0xFFu << shift))
                                          | ((value & 0xFF) << shift));
                this.Log(LogLevel.Debug, "8-bit write to TCD {0} field 0x{1:X}", channel, field);
                TcdWriteHalf(channel, pair, mergedByte);
                return;
            }
            TcdWriteHalf(channel, field & ~1L, (ushort)value);
        }

        private uint TcdReadHalf(int channel, long field)
        {
            var t = tcd[channel];
            switch(field)
            {
            case 0x04:
                return t.soff;
            case 0x06:
                return t.attr;
            case 0x14:
                return t.doff;
            case 0x16:
                return t.citer;
            case 0x1C:
                // ACTIVE reads 0 always - see TcdReadWord.
                return (uint)(t.csr & unchecked((ushort)~CsrActive));
            case 0x1E:
                return t.biter;
            default:
                return (TcdReadWord(channel, field & ~3L) >> ((int)(field & 2) * 8)) & 0xFFFF;
            }
        }

        private void TcdWriteHalf(int channel, long field, ushort value)
        {
            var t = tcd[channel];
            switch(field)
            {
            case 0x04:
                t.soff = value;
                return;
            case 0x06:
                t.attr = value;
                return;
            case 0x14:
                t.doff = value;
                return;
            case 0x16:
                t.citer = value;
                return;
            case 0x1C:
                WriteControlAndStatus(channel, value);
                return;
            case 0x1E:
                t.biter = value;
                return;
            default:
                var aligned = field & ~3L;
                var shift = (int)(field & 2) * 8;
                var merged = (TcdReadWord(channel, aligned) & ~(0xFFFFu << shift))
                             | ((uint)value << shift);
                TcdWriteWord(channel, aligned, merged);
                return;
            }
        }

        // TCD layout, PERI_DMA.h: +0x00 SADDR(32) +0x04 SOFF(16) +0x06 ATTR(16)
        // +0x08 NBYTES(32) +0x0C SLAST(32) +0x10 DADDR(32) +0x14 DOFF(16)
        // +0x16 CITER(16) +0x18 DLAST_SGA(32) +0x1C CSR(16) +0x1E BITER(16).
        private uint TcdReadWord(int channel, long field)
        {
            var t = tcd[channel];
            switch(field)
            {
            case 0x00:
                return t.saddr;
            case 0x04:
                return t.soff | ((uint)t.attr << 16);
            case 0x08:
                return t.nbytes;
            case 0x0C:
                return (uint)t.slast;
            case 0x10:
                return t.daddr;
            case 0x14:
                return t.doff | ((uint)t.citer << 16);
            case 0x18:
                return (uint)t.dlastSga;
            case 0x1C:
                // ACTIVE reads 0 always. edma_reload_loop spins on it with no
                // timeout, in thread context under irq_lock, during begin().
                return (uint)(t.csr & unchecked((ushort)~CsrActive)) | ((uint)t.biter << 16);
            default:
                this.LogUnhandledRead(TcdBase + channel * TcdStride + field);
                return 0;
            }
        }

        private void TcdWriteWord(int channel, long field, uint value)
        {
            var t = tcd[channel];
            switch(field)
            {
            case 0x00:
                t.saddr = value;
                return;
            case 0x04:
                t.soff = (ushort)value;
                t.attr = (ushort)(value >> 16);
                return;
            case 0x08:
                t.nbytes = value;
                return;
            case 0x0C:
                t.slast = (int)value;
                return;
            case 0x10:
                t.daddr = value;
                return;
            case 0x14:
                t.doff = (ushort)value;
                t.citer = (ushort)(value >> 16);
                return;
            case 0x18:
                t.dlastSga = (int)value;
                return;
            case 0x1C:
                // BITER first: CSR[START] inside WriteControlAndStatus can run
                // the TCD, and the major-loop reload would then use the old
                // BITER. fsl_edma only ever writes the 16-bit CSR, so no guest
                // here reaches this, but the order costs nothing.
                t.biter = (ushort)(value >> 16);
                WriteControlAndStatus(channel, (ushort)value);
                return;
            default:
                this.LogUnhandledWrite(TcdBase + channel * TcdStride + field, value);
                return;
            }
        }

        private void WriteControlAndStatus(int channel, ushort value)
        {
            var t = tcd[channel];
            if((t.csr & CsrDone) != 0)
            {
                // The reference manual forces both to 0 when they are written
                // while DONE is set. EDMA_InstallTCD writes CSR = 0 and then
                // CSR = tcd->CSR for exactly this reason, commenting "Clear DONE
                // bit first, otherwise ESG cannot be set" - so DONE has to take
                // the written value, it is not write-1-to-clear.
                value &= unchecked((ushort)~(CsrMajorElink | CsrEsg));
            }
            value &= unchecked((ushort)~CsrActive);
            t.csr = value;
            if((value & CsrStart) != 0)
            {
                t.csr &= unchecked((ushort)~CsrStart);
                SoftwareRequest(channel);
            }
        }

        // CITER and BITER keep the channel-linking bits in the top of the same
        // 16-bit field, so the count is 9 bits wide when ELINK is set and 15
        // otherwise. Channel linking itself is stored and never acted on -
        // nothing in Zephyr's shim sets source_chaining_en or dest_chaining_en -
        // but the bits still have to survive a decrement.
        private static ushort CountOf(ushort field)
        {
            return (ushort)((field & Elink) != 0 ? field & 0x1FF : field & 0x7FFF);
        }

        private static void StoreCount(Tcd t, ushort count)
        {
            if((t.citer & Elink) != 0)
            {
                t.citer = (ushort)((t.citer & 0xFE00) | (count & 0x1FF));
            }
            else
            {
                t.citer = (ushort)((t.citer & Elink) | (count & 0x7FFF));
            }
        }

        private static int TransferSize(int code)
        {
            switch(code)
            {
            case 0:
                return 1;
            case 1:
                return 2;
            case 2:
                return 4;
            case 3:
                return 8;
            case 5:
                return 32;
            default:
                return 0;
            }
        }

        private class Tcd
        {
            public void Clear()
            {
                saddr = 0;
                soff = 0;
                attr = 0;
                nbytes = 0;
                slast = 0;
                daddr = 0;
                doff = 0;
                citer = 0;
                dlastSga = 0;
                csr = 0;
                biter = 0;
            }

            public uint saddr;
            public ushort soff;
            public ushort attr;
            public uint nbytes;
            public int slast;
            public uint daddr;
            public ushort doff;
            public ushort citer;
            public int dlastSga;
            public ushort csr;
            public ushort biter;
        }

        private uint cr;
        private uint erq;
        private uint eei;
        private uint intFlags;
        private uint ears;

        private readonly IMachine machine;
        private readonly int numberOfChannels;
        private readonly uint channelMask;
        private readonly Tcd[] tcd;
        private readonly byte[] dchpri;
        private readonly bool[] requestLevel;   // this IS the HRS latch
        private readonly bool[] servicing;      // re-entrancy guard, nothing more
        private readonly ulong[] beatsMoved;    // diagnostics only, see BeatsPerChannel

        private const int MaxChannels = 32;
        private const int SharedLines = 16;
        private const int ErrorLine = 16;
        // Comfortably above any legitimate run: one request is one minor loop,
        // and CITER cannot exceed 32767, so even a whole major loop drained
        // inside a single held request stays well under this. Low enough that
        // hitting it produces a log line rather than another kind of hang.
        private const int MaxMinorLoopsPerRequest = 65536;
        // Bounds one minor loop the way MaxMinorLoopsPerRequest bounds a held
        // request. 64 KiB is four orders of magnitude above anything this SoC's
        // peripherals program and four orders below the 0x3FFFFFFF that a
        // corrupt NBYTES can decode to.
        private const uint MaxBytesPerMinorLoop = 0x10000;

        private const long CrOffset = 0x00;
        private const long EsOffset = 0x04;
        private const long ErqOffset = 0x0C;
        private const long EeiOffset = 0x14;
        private const long CommandsOffset = 0x18;   // CEEI SEEI CERQ SERQ CDNE SSRT CERR CINT
        private const long IntOffset = 0x24;
        private const long ErrOffset = 0x2C;
        private const long HrsOffset = 0x34;
        private const long EarsOffset = 0x44;
        private const long DchpriBase = 0x100;
        private const long TcdBase = 0x1000;
        private const long TcdStride = 0x20;

        private const uint CrWriteMask = 0x000305FE;
        private const uint CrErga = 0x00000008;
        private const uint CrHoe = 0x00000010;
        private const uint CrHalt = 0x00000020;
        private const uint CrEmlm = 0x00000080;
        private const uint CrEcx = 0x00010000;
        private const uint CrCx = 0x00020000;

        private const uint NbytesDmloe = 0x40000000;
        private const uint NbytesSmloe = 0x80000000;

        private const ushort CsrStart = 0x0001;
        private const ushort CsrIntMajor = 0x0002;
        private const ushort CsrIntHalf = 0x0004;
        private const ushort CsrDreq = 0x0008;
        private const ushort CsrEsg = 0x0010;
        private const ushort CsrMajorElink = 0x0020;
        private const ushort CsrActive = 0x0040;
        private const ushort CsrDone = 0x0080;

        private const ushort Elink = 0x8000;

        private const byte CommandAll = 0x40;
        private const byte CommandNop = 0x80;
    }

    //
    // The DMAMUX in front of it. Two things about this block decide whether
    // anything ever starts:
    //
    //   * CHCFG must read back. dma_mcux_edma.c programs it as two separate
    //     read-modify-writes - DMAMUX_SetSource then DMAMUX_EnableChannel - so a
    //     write-only register silently loses either SOURCE or ENBL.
    //
    //   * The GPIO input index here is the request SOURCE number (0..255), not a
    //     channel. Which channel a source reaches is decided at run time by
    //     whichever CHCFG the guest programmed with that source, so the platform
    //     file names sources and never channels, and stays correct if the
    //     devicetree reassigns them.
    //
    // A_ON is a request source in its own right, and the always-on path
    // deliberately leaves SOURCE at 0 (dma_mcux_edma.c takes it for
    // kEDMA_MemoryToMemory). That is why A_ON asserts by itself and why source 0
    // - "no source" - is excluded from routing.
    //
    [GPIO(NumberOfInputs = 256)]
    public class AP_IMXRT_DMAMUX : IDoubleWordPeripheral, IKnownSize, IGPIOReceiver
    {
        public AP_IMXRT_DMAMUX(IMachine machine, AP_IMXRT_EDMA dma, int numberOfChannels = 32)
        {
            if(dma == null)
            {
                throw new ConstructionException("AP_IMXRT_DMAMUX needs the eDMA it feeds");
            }
            if(numberOfChannels < 1 || numberOfChannels > MaxChannels)
            {
                throw new ConstructionException(
                    "AP_IMXRT_DMAMUX supports 1 to 32 channels, given: " + numberOfChannels);
            }
            this.dma = dma;
            this.numberOfChannels = numberOfChannels;
            chcfg = new uint[MaxChannels];
            sourceLevel = new bool[NumberOfSources];
        }

        public long Size => 0x4000;

        public void Reset()
        {
            for(var i = 0; i < MaxChannels; i++)
            {
                chcfg[i] = 0;
            }
            for(var i = 0; i < NumberOfSources; i++)
            {
                sourceLevel[i] = false;
            }
        }

        public void OnGPIO(int source, bool value)
        {
            if(source < 0 || source >= NumberOfSources)
            {
                this.Log(LogLevel.Warning, "request source {0} is out of range", source);
                return;
            }
            sourceLevel[source] = value;
            if(source == 0)
            {
                // SOURCE 0 means "no source", and it is also what the always-on
                // channels leave behind. Routing it would start them from any
                // stray line.
                return;
            }
            for(var channel = 0; channel < numberOfChannels; channel++)
            {
                var config = chcfg[channel];
                if((config & Enbl) == 0 || (config & AlwaysOn) != 0)
                {
                    continue;
                }
                if((config & SourceMask) == (uint)source)
                {
                    dma.SetRequest(channel, value);
                }
            }
        }

        public uint ReadDoubleWord(long offset)
        {
            var channel = (int)(offset / 4);
            if((offset % 4) != 0 || channel >= numberOfChannels)
            {
                this.LogUnhandledRead(offset);
                return 0;
            }
            return chcfg[channel];
        }

        public void WriteDoubleWord(long offset, uint value)
        {
            var channel = (int)(offset / 4);
            if((offset % 4) != 0 || channel >= numberOfChannels)
            {
                this.LogUnhandledWrite(offset, value);
                return;
            }
            chcfg[channel] = value & (Enbl | Trig | AlwaysOn | SourceMask);
            var enabled = (chcfg[channel] & Enbl) != 0;
            var alwaysOn = (chcfg[channel] & AlwaysOn) != 0;
            var source = chcfg[channel] & SourceMask;
            dma.SetRequest(channel, enabled && (alwaysOn || (source != 0 && sourceLevel[source])));
        }

        private readonly AP_IMXRT_EDMA dma;
        private readonly int numberOfChannels;
        private readonly uint[] chcfg;
        private readonly bool[] sourceLevel;

        private const int MaxChannels = 32;
        private const int NumberOfSources = 256;

        private const uint SourceMask = 0x000000FF;
        private const uint AlwaysOn = 0x20000000;
        private const uint Trig = 0x40000000;
        private const uint Enbl = 0x80000000;
    }
}
