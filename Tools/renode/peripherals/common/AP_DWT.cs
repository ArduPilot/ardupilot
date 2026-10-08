//
// Cortex-M DWT: a cycle counter following virtual CPU time, plus four
// guest watchpoint comparators when the CPU supports them. Renode leaves the DWT block
// unimplemented, so CYCCNT reads 0 forever and every ChibiOS
// chSysPolledDelayX() - the OTG PHY delays in usb_lld_start are the
// first in the ArduPilot boot - spins for good. Granularity is the
// virtual-time resolution at the point of read, which is enough for
// the wait-at-least semantics polled delays want.
//
using System;
using Antmicro.Renode.Core;
using Antmicro.Renode.Logging;
using Antmicro.Renode.Logging.Profiling;
using Antmicro.Renode.Peripherals;
using Antmicro.Renode.Peripherals.Bus;
using Antmicro.Renode.Peripherals.CPU;

namespace Antmicro.Renode.Peripherals.Miscellaneous
{
    [AllowedTranslations(AllowedTranslation.ByteToDoubleWord | AllowedTranslation.WordToDoubleWord)]
    public class AP_DWT : IDoubleWordPeripheral, IKnownSize, IHasFrequency
    {
        public AP_DWT(IMachine machine, uint frequency = 168000000, CortexM cpu = null)
        {
            this.machine = machine;
            this.frequency = frequency;
            // Keep the timer model usable with packaged Renode versions which
            // predate guest watchpoint support; advertise no comparators there.
            var method = cpu?.GetType().GetMethod("RequestDWTTrap");
            if(method != null)
            {
                setMemoryHook = (Action<MemoryAccessHook>)Delegate.CreateDelegate(
                    typeof(Action<MemoryAccessHook>), cpu, "SetGuestMemoryAccessHook");
                requestTrap = (Action)Delegate.CreateDelegate(typeof(Action), cpu, "RequestDWTTrap");
            }
        }

        public long Size => 0x1000;

        public ulong Frequency
        {
            get { return frequency; }
            set
            {
                if(value == 0)
                {
                    throw new ArgumentException("DWT frequency must be greater than zero");
                }
                var current = CycleCount() + cyccntOffset;
                frequency = value;
                cyccntOffset = current - CycleCount();
            }
        }

        public void Reset()
        {
            Array.Clear(comparators, 0, 4);
            Array.Clear(masks, 0, 4);
            Array.Clear(functions, 0, 4);
            UpdateHooks();
            control = 0;
            cyccntOffset = 0;
        }

        public uint ReadDoubleWord(long offset)
        {
            int index, register;
            if(ComparatorOffset(offset, out index, out register))
            {
                uint result = register == 0 ? comparators[index] : register == 1 ? masks[index] : functions[index];
                if(register == 2)
                {
                    functions[index] &= ~(1u << 24);
                }
                return result;
            }
            switch(offset)
            {
            case CTRL:
                return control | (requestTrap == null ? 0u : 4u << 28);
            case CYCCNT:
                return CycleCount() + cyccntOffset;
            default:
                return 0;
            }
        }

        public void WriteDoubleWord(long offset, uint value)
        {
            int index, register;
            if(ComparatorOffset(offset, out index, out register))
            {
                if(register == 0)
                {
                    comparators[index] = value;
                }
                else if(register == 1)
                {
                    masks[index] = Math.Min(value & 31, 5u);
                }
                else
                {
                    functions[index] = value & 15;
                }
                UpdateHooks();
                return;
            }
            switch(offset)
            {
            case CTRL:
                control = value & 1;
                return;
            case CYCCNT:
                cyccntOffset = value - CycleCount();
                return;
            }
        }

        // Guest DWT comparators use memory notifications to observe guest memory
        // accesses. The CPU delivers the debug event at the next instruction,
        // independently of Renode's host GDB breakpoints.
        private bool ComparatorOffset(long offset, out int index, out int register)
        {
            index = (int)(offset - 0x20) / 16;
            register = (int)(offset & 15) / 4;
            return requestTrap != null && offset >= 0x20 && offset < 0x60 && (offset & 3) == 0 && register < 3;
        }

        private void UpdateHooks()
        {
            if(requestTrap == null)
            {
                return;
            }
            bool enabled = Array.Exists(functions, value => (value & 15) >= 5 && (value & 15) <= 7);
            setMemoryHook(enabled ? (MemoryAccessHook)ObserveAccess : null);
        }

        private void ObserveAccess(ulong pc, MemoryOperation operation, ulong virtualAddress,
            ulong physicalAddress, uint width, ulong value)
        {
            bool read = operation == MemoryOperation.MemoryRead || operation == MemoryOperation.MemoryIORead;
            bool write = operation == MemoryOperation.MemoryWrite || operation == MemoryOperation.MemoryIOWrite;
            for(int i = 0; i < 4; i++)
            {
                uint function = functions[i] & 15;
                if(!(read && (function == 5 || function == 7)) && !(write && (function == 6 || function == 7)))
                {
                    continue;
                }
                ulong start = comparators[i] & ~((1u << (int)masks[i]) - 1u);
                ulong end = start + (1u << (int)masks[i]);
                if(physicalAddress < end && physicalAddress + width > start)
                {
                    functions[i] |= 1u << 24;
                    requestTrap();
                }
            }
        }

        private readonly Action requestTrap;
        private readonly Action<MemoryAccessHook> setMemoryHook;
        private readonly uint[] comparators = new uint[4], masks = new uint[4], functions = new uint[4];

        private uint CycleCount()
        {
            var us = (ulong)machine.ElapsedVirtualTime.TimeElapsed.TotalMicroseconds;
            return (uint)(us * frequency / 1000000);
        }

        private const long CTRL = 0x0;
        private const long CYCCNT = 0x4;

        private readonly IMachine machine;
        private ulong frequency;
        private uint control;
        private uint cyccntOffset;
    }
}
