//
// Makes Renode's NXP_LPUART usable as a DMA source, and gives it the idle line.
//
// Two separate gaps, both fatal to RX, neither fixable in the platform file:
//
// (a) NXP_LPUART can NEVER assert ReceiveDMA for this guest. Its
//     ReceiveDmaState is `receiverDMAEnabled && BufferState == Full`, and
//     UpdateBufferState() returns Ready - not Full - for any count above the
//     receive watermark while the RX FIFO is enabled. LPUART_Init leaves the
//     watermark at 0 and then sets FIFO[RXFE], so the count is always above it
//     and Full is unreachable. A perfect eDMA model therefore receives exactly
//     zero bytes. This helper drives uart.ReceiveDMA itself, from the only
//     condition that actually matters: RX DMA enabled and the FIFO not empty.
//
// (b) STAT[IDLE] and CTRL[ILIE] are TaggedFlags in NXP_LPUART and appear
//     nowhere in its UpdateInterrupt. The idle line is the ONLY trigger for
//     partial RX delivery: mcux_lpuart_isr's kLPUART_IdleLineFlag branch is
//     what schedules mcux_lpuart_async_rx_flush, and without it a MAVLink frame
//     shorter than the 128-byte RX buffer is never handed up. This helper sets
//     the status bit one character gap after the last byte and raises IRQ when
//     the guest has enabled the interrupt.
//
// This is a repo-local .cs and not a patch to the Renode submodule on purpose:
// CI fetches a pinned portable Renode build (RENODE_SOURCE_REVISION in
// .github/workflows/test_renode_zephyr.yml), so a submodule change would not be
// present where the tests actually run.
//
// Raising the interrupt line is only half of (b). NXP_LPUART recomputes IRQ in
// its private UpdateInterrupt(), and the ONLY paths that reach it are the
// BAUD/CTRL/DATA/FIFO/WATER write callbacks, the DATA read callback and
// WriteChar. Its STAT register has no write callback at all - so the one
// register the guest's idle handler touches, LPUART_ClearStatusFlags writing
// STAT, recomputes nothing. Without help the synthetic idle would go up and
// never come down: Renode's NVIC re-pends a completed interrupt whose input is
// still high, so mcux_lpuart_isr would re-enter forever reading a STAT with no
// flag set, and the guest would stop making progress after the first short RX
// frame. AP_STM32F7_USART_Idle hit exactly this and records the symptom. The
// STAT hook below therefore calls UpdateInterrupt itself, which also repairs
// the same gap for the LPUART's own W1C error flags (OR/FE/NF/PF): clearing one
// of those through STAT recomputes nothing in stock NXP_LPUART either.
//
// Reflection is used three times, all checked at construction. `registers` and
// `UpdateInterrupt` are private on NXP_LPUART, which exposes neither a
// RegistersCollection property nor any way to recompute its interrupt, and
// `Count` is protected on UARTBase. If Count ever disappears the fallback is
// BufferState != Empty, which is exactly equivalent under an enabled RX FIFO
// with a zero watermark - a deliberate deviation from "every reflection lookup
// is fatal", because that one has a correct substitute and the other two do
// not.
//
// Expect one "Unhandled write to offset 0x14 ... Unhandled bits: [20]" warning
// from NXP_LPUART per idle clear - STAT[IDLE] is a TaggedFlag there, so the
// register logs before our after-write hook ever sees the value. It is noise,
// one line per frame boundary per UART, not a fault.
//
// Note on ScheduleAction: it is correct HERE, because the idle line is a
// genuine timeout measured in character times. It is wrong for the DMA request
// itself - see the header of AP_IMXRT_EDMA.cs.
//
using System;
using System.Reflection;
using Antmicro.Renode.Core;
using Antmicro.Renode.Core.Structure.Registers;
using Antmicro.Renode.Exceptions;
using Antmicro.Renode.Logging;
using Antmicro.Renode.Peripherals;
using Antmicro.Renode.Peripherals.Bus;
using Antmicro.Renode.Peripherals.UART;
using Antmicro.Renode.Time;

namespace Antmicro.Renode.Peripherals.Miscellaneous
{
    public class AP_IMXRT_LPUART_DmaFix : IDoubleWordPeripheral, IKnownSize
    {
        public AP_IMXRT_LPUART_DmaFix(IMachine machine, NXP_LPUART uart, bool hasGlobalRegisters = true)
        {
            if(uart == null)
            {
                throw new ConstructionException("AP_IMXRT_LPUART_DmaFix needs an NXP_LPUART");
            }
            this.machine = machine;
            this.uart = uart;

            var registersField = typeof(NXP_LPUART).GetField("registers",
                BindingFlags.NonPublic | BindingFlags.Instance);
            if(registersField == null)
            {
                throw new ConstructionException(
                    "NXP_LPUART no longer has a private `registers` field - revisit this fix");
            }
            var registers = registersField.GetValue(uart) as DoubleWordRegisterCollection;
            if(registers == null)
            {
                throw new ConstructionException(
                    "NXP_LPUART.registers is no longer a DoubleWordRegisterCollection - revisit this fix");
            }

            // The only way to make NXP_LPUART re-derive its interrupt line from
            // its own sources. Needed because a STAT write - the guest's idle
            // acknowledgement - reaches no callback inside the model. See the
            // file header for what happens without it.
            updateInterrupt = typeof(NXP_LPUART).GetMethod("UpdateInterrupt",
                BindingFlags.NonPublic | BindingFlags.Instance);
            if(updateInterrupt == null)
            {
                throw new ConstructionException(
                    "NXP_LPUART no longer has a private UpdateInterrupt() - revisit this fix");
            }

            var countProperty = typeof(UARTBase).GetProperty("Count",
                BindingFlags.NonPublic | BindingFlags.Instance);
            if(countProperty == null)
            {
                this.Log(LogLevel.Warning,
                    "UARTBase.Count is gone; falling back to BufferState for the RX fill level");
            }
            countGetter = countProperty;

            // hasGlobalRegisters shifts every common register by 0x10. The
            // platform constructs NXP_LPUART with the default (true), so BAUD is
            // at 0x10 and DATA at 0x1C; the parameter is here so a platform that
            // sets it false does not silently hook the wrong offsets.
            var common = hasGlobalRegisters ? 0x10L : 0x00L;
            baudOffset = common + 0x00;
            statusOffset = common + 0x04;
            controlOffset = common + 0x08;
            dataOffset = common + 0x0C;
            fifoOffset = common + 0x18;
            waterOffset = common + 0x1C;

            // (a) RX request generation. Only ONE hook of each kind may exist per
            // offset - RegisterCollection throws on a second - so each offset
            // gets a single hook that does everything that offset needs.
            registers.AddAfterWriteHook(baudOffset, (offset, value) =>
            {
                receiveDmaEnabled = (value & ReceiveDmaEnable) != 0;
                Refresh();
                RestoreIdleInterrupt();
            });

            // The eDMA has just taken a byte out of the FIFO. Dropping the level
            // here, from inside the eDMA's own bus access, is what breaks its
            // service loop when the FIFO empties; leaving it high would make the
            // next minor loop read an empty FIFO and set RXUF.
            registers.AddAfterReadHook(dataOffset, (offset, value) =>
            {
                Refresh();
                RearmIdle();
                RestoreIdleInterrupt();
                return value;
            });

            uart.BufferStateChanged += state =>
            {
                Refresh();
                if(state != BufferState.Empty)
                {
                    // A byte arrived. Going Empty is the eDMA draining the FIFO
                    // in the same bus access, and the data-register hook above
                    // has already re-armed for that - one scheduled action per
                    // byte is enough.
                    RearmIdle();
                }
                RestoreIdleInterrupt();
            };

            // (b) idle line.
            registers.AddAfterReadHook(statusOffset, (offset, value) =>
                idlePending ? value | IdleFlag : value);

            registers.AddAfterWriteHook(statusOffset, (offset, value) =>
            {
                // LPUART_ClearStatusFlags writes (STAT & 0x3E000000) | mask, so
                // the idle bit appears in the written value when, and only when,
                // the guest is clearing it.
                if((value & IdleFlag) != 0)
                {
                    idlePending = false;
                }
                // Unconditional, and it must come before the re-assert. This is
                // the register the ISR uses to acknowledge, and it is the one
                // register in NXP_LPUART that recomputes nothing on its own -
                // so this call is what actually LOWERS the line. Without it the
                // guest's handler re-enters forever on a line the NVIC re-pends.
                RecomputeUartInterrupt();
                RestoreIdleInterrupt();
            });

            registers.AddAfterWriteHook(controlOffset, (offset, value) =>
            {
                idleInterruptEnabled = (value & IdleInterruptEnable) != 0;
                RestoreIdleInterrupt();
            });

            // Every write below makes NXP_LPUART recompute IRQ from its own
            // sources, which would drop a synthetic idle assertion on the floor.
            // Re-assert afterwards.
            registers.AddAfterWriteHook(dataOffset, (offset, value) => RestoreIdleInterrupt());
            // A character arriving through WriteChar also makes the LPUART
            // recompute IRQ, and that path is not a register write, so a
            // synthetic idle assertion can be dropped there with nothing to
            // re-assert it. That is harmless: the arriving byte re-arms the idle
            // timer, so the interrupt is simply delivered one character gap
            // after the LAST byte of the burst instead of in the middle of it,
            // which is what an idle line means in the first place. The status
            // bit itself is never lost - only the guest can clear it.
            registers.AddAfterWriteHook(fifoOffset, (offset, value) => RestoreIdleInterrupt());
            registers.AddAfterWriteHook(waterOffset, (offset, value) => RestoreIdleInterrupt());
        }

        public long Size => 4;

        public uint ReadDoubleWord(long offset)
        {
            return 0;
        }

        public void WriteDoubleWord(long offset, uint value)
        {
        }

        public void Reset()
        {
            idlePending = false;
            idleInterruptEnabled = false;
            receiveDmaEnabled = false;
            generation++;
            if(uart.ReceiveDMA.IsSet)
            {
                uart.ReceiveDMA.Unset();
            }
            // Peripheral reset order is not defined, so do not rely on the
            // LPUART's own Reset() to drop a synthetic idle assertion made
            // before it: recompute the line from the LPUART's real sources now
            // that idlePending is false.
            RecomputeUartInterrupt();
        }

        // The eDMA treats ReceiveDMA as a level and loops on it, so this is
        // re-pulsed rather than merely held: NXP_LPUART's own UpdateRxDMA does
        // the same, and a fresh edge is what a stock DMA model would need.
        //
        // This deliberately has NO re-entrancy guard of its own. The call that
        // matters most is the nested one, from the data-register after-read hook
        // that fires inside the eDMA's own bus access: dropping the level THERE
        // is the only thing that ends the eDMA's service loop when the FIFO
        // empties. A guard that suppressed the nested call would leave the level
        // high and the eDMA would keep reading an empty FIFO until its own
        // iteration cap fired. Recursion is bounded instead by the eDMA, whose
        // per-channel servicing flag makes the nested Set() a no-op - so the
        // nesting never goes deeper than one bus access.
        private void Refresh()
        {
            var wanted = receiveDmaEnabled && ReceiveFillLevel() > 0;
            if(uart.ReceiveDMA.IsSet)
            {
                uart.ReceiveDMA.Unset();
            }
            if(wanted)
            {
                uart.ReceiveDMA.Set();
            }
        }

        private int ReceiveFillLevel()
        {
            if(countGetter != null)
            {
                return (int)countGetter.GetValue(uart);
            }
            return uart.BufferState == BufferState.Empty ? 0 : 1;
        }

        // Drive NXP_LPUART's own UpdateInterrupt(). Lowers the line when nothing
        // real is pending, and is the only thing in this file that CAN lower it:
        // uart.IRQ.Set(false) would be wrong, because it would also swallow the
        // LPUART's genuine RDRF/TDRE/overrun/framing interrupts - and the guest
        // enables ORIE/NEIE/FEIE/PEIE in mcux_lpuart_rx_enable.
        private void RecomputeUartInterrupt()
        {
            updateInterrupt.Invoke(uart, null);
        }

        private void RestoreIdleInterrupt()
        {
            if(idlePending && idleInterruptEnabled)
            {
                uart.IRQ.Set(true);
            }
        }

        // One character gap after the last byte, superseded by the next one.
        //
        // "The RX FIFO went empty" is not an idle condition here: the eDMA
        // drains it inside the same bus access that filled it, so the FIFO is
        // empty between every pair of consecutive bytes. The wait is deliberately
        // two frames rather than one because the host-side terminal delivers a
        // socket burst one frame apart per byte - the same reason
        // AP_STM32F7_USART_Idle waits 20 bits. Firing early is not harmful
        // (mcux_lpuart_async_rx_flush tracks a monotonic counter and re-notifies
        // nothing), only noisy; firing never means MAVLink arrives in 128-byte
        // chunks or not at all.
        private void RearmIdle()
        {
            var scheduled = ++generation;
            var baudRate = Math.Max(uart.BaudRate, 1u);
            var delay = (uint)Math.Ceiling(IdleBits * 1000000.0 / baudRate);
            machine.ScheduleAction(TimeInterval.FromMicroseconds(delay), _ =>
            {
                if(scheduled != generation)
                {
                    return;
                }
                idlePending = true;
                RestoreIdleInterrupt();
            }, name: "LPUART idle line");
        }

        private bool idlePending;
        private bool idleInterruptEnabled;
        private bool receiveDmaEnabled;
        private uint generation;

        private readonly IMachine machine;
        private readonly NXP_LPUART uart;
        private readonly MethodInfo updateInterrupt;
        private readonly PropertyInfo countGetter;
        private readonly long baudOffset;
        private readonly long statusOffset;
        private readonly long controlOffset;
        private readonly long dataOffset;
        private readonly long fifoOffset;
        private readonly long waterOffset;

        private const uint ReceiveDmaEnable = 1u << 21;      // BAUD[RDMAE]
        private const uint IdleFlag = 1u << 20;              // STAT[IDLE]
        private const uint IdleInterruptEnable = 1u << 20;   // CTRL[ILIE]
        private const uint IdleBits = 20;
    }
}
