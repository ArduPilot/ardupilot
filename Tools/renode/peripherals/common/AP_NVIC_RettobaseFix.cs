//
// Renode's NVIC (through at least 1.16.1 and current master) leaves
// ICSR.RETTOBASE (bit 11) as a tag that always reads 0. ChibiOS's
// _port_irq_epilogue() requires RETTOBASE=1 before it will arrange a
// preemptive context switch on exception return, so with the stock
// model every interrupt-driven thread wakeup is silently dropped: the
// ISR runs, chSchReadyI() readies the thread, and the CPU returns to
// whatever was running. The first casualty is the very first
// chThdSleep() in boot, which never wakes.
//
// The NVIC tracks its active-exception stack ("activeIRQs"), which is
// exactly the state RETTOBASE needs: returning-to-base means no active
// exception besides the current one. The register framework allows
// after-read hooks on any register, so rather than forking the NVIC
// this helper installs a hook on ICSR that ORs in bit 11 when at most
// one exception is active. The only reflection needed is reading the
// private activeIRQs field once at construction.
//
// Instantiate it in the platform with a reference to the nvic; the bus
// registration is a formality (reads as zero).
//
using System;
using System.Collections.Generic;
using System.Reflection;
using Antmicro.Renode.Core;
using Antmicro.Renode.Core.Structure.Registers;
using Antmicro.Renode.Exceptions;
using Antmicro.Renode.Logging;
using Antmicro.Renode.Peripherals;
using Antmicro.Renode.Peripherals.Bus;
using Antmicro.Renode.Peripherals.IRQControllers;

namespace Antmicro.Renode.Peripherals.Miscellaneous
{
    public class AP_NVIC_RettobaseFix : IDoubleWordPeripheral, IKnownSize
    {
        public AP_NVIC_RettobaseFix(NVIC nvic)
        {
            var field = typeof(NVIC).GetField("activeIRQs", BindingFlags.NonPublic | BindingFlags.Instance);
            if(field == null)
            {
                throw new ConstructionException("NVIC no longer has an activeIRQs field - revisit this fix");
            }
            var active = (Stack<int>)field.GetValue(nvic);
            nvic.RegisterCollection.AddAfterReadHook(InterruptControlState, (offset, value) =>
            {
                return active.Count <= 1 ? (uint?)(value | Rettobase) : value;
            });
        }

        public long Size => 4;
        public uint ReadDoubleWord(long offset) => 0;
        public void WriteDoubleWord(long offset, uint value) { }
        public void Reset() { }

        private const long InterruptControlState = 0xD04;
        private const uint Rettobase = 1u << 11;
    }
}
