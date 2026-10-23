//
// RP2350 SIO: per-core CPUID, GPIO output registers, the inter-core FIFOs,
// hardware spinlocks and doorbells.
//
// SIO is private to each core: what a register returns depends on which core
// reads it. The accessing core is taken from the system bus.
//
// There is no boot ROM model, so this also plays core1's ROM: until it is
// launched, core1 answers core0's FIFO words by echoing them, as the ROM's
// wait-for-launch loop does. After the sequence 0, 0, 1, VTOR, SP, entry it
// sets core1's registers and releases it. ChibiOS start_core1() (hal_lld.c)
// drives this.
//
// XIP lockout: before erasing or programming flash, core0 sets c1_xip_lock
// to 1 and rings core1's doorbell, and core1's handler parks in SRAM and
// answers 2 (board_rp2350.c, rp2350/c1_main.c). The emulated flash never
// takes XIP away, so parking protects nothing. With XipLockAddress set to
// &c1_xip_lock, a core0 doorbell answers the request directly and core1 is
// never interrupted.
//
// GPIO outputs: 0 = SIO_IRQ_FIFO for core0, 1 = for core1,
//               2 = SIO_IRQ_BELL for core0, 3 = for core1,
//               100 + n = level of GPIO n as driven by GPIO_OUT (chip
//               selects, LEDs), for n = 0..47.
//
using System.Collections.Generic;
using System.Collections.ObjectModel;
using Antmicro.Renode.Core;
using Antmicro.Renode.Logging;
using Antmicro.Renode.Peripherals;
using Antmicro.Renode.Peripherals.Bus;
using Antmicro.Renode.Peripherals.CPU;

namespace Antmicro.Renode.Peripherals.Miscellaneous
{
    [AllowedTranslations(AllowedTranslation.ByteToDoubleWord | AllowedTranslation.WordToDoubleWord)]
    public class AP_RP2350_SIO : IDoubleWordPeripheral, IKnownSize, INumberedGPIOOutput
    {
        public AP_RP2350_SIO(IMachine machine, CortexM core1)
        {
            this.machine = machine;
            this.core1 = core1;
            var outputs = new Dictionary<int, IGPIO>();
            for(var i = 0; i < 4; i++)
            {
                outputs[i] = new GPIO();
            }
            for(var pin = 0; pin < NumPins; pin++)
            {
                outputs[PinOutputBase + pin] = new GPIO();
            }
            Connections = new ReadOnlyDictionary<int, IGPIO>(outputs);
            Reset();
        }

        public long Size => 0x200;

        public IReadOnlyDictionary<int, IGPIO> Connections { get; }

        // address of c1_xip_lock; 0 leaves the doorbell to core1 as on hardware
        public ulong XipLockAddress { get; set; }

        public void Reset()
        {
            for(var i = 0; i < 2; i++)
            {
                rx[i] = new Queue<uint>();
                wof[i] = false;
                roe[i] = false;
                doorbellIn[i] = 0;
            }
            regs = new uint[Size / 4];
            spinlocks = 0;
            core1Launched = false;
            launchSeq.Clear();
            UpdateInterrupts();
            UpdatePins();
        }

        // Renode runs each core on its own host thread, so every access is
        // serialised: a spinlock claim or FIFO pop must be atomic.
        public uint ReadDoubleWord(long offset)
        {
            lock(accessLock)
            {
                return ReadLocked(offset);
            }
        }

        public void WriteDoubleWord(long offset, uint value)
        {
            lock(accessLock)
            {
                WriteLocked(offset, value);
            }
        }

        private uint ReadLocked(long offset)
        {
            var core = CurrentCore;
            switch(offset)
            {
                case CpuId:
                    return (uint)core;
                case GpioIn:
                    return regs[GpioOut >> 2];
                case GpioHiIn:
                    return regs[GpioHiOut >> 2];
                case FifoSt:
                    return FifoStatus(core);
                case FifoRd:
                    if(rx[core].Count == 0)
                    {
                        roe[core] = true;
                        UpdateInterrupts();
                        return 0;
                    }
                    var value = rx[core].Dequeue();
                    UpdateInterrupts();
                    return value;
                case SpinlockSt:
                    return spinlocks;
                case DoorbellOutSet:
                case DoorbellOutClr:
                    return doorbellIn[1 - core];
                case DoorbellInSet:
                case DoorbellInClr:
                    return doorbellIn[core];
                default:
                    if(offset >= Spinlock0 && offset < Spinlock0 + 4 * 32)
                    {
                        // claim on read: returns nonzero only if it was free
                        var bit = 1u << (int)((offset - Spinlock0) >> 2);
                        if((spinlocks & bit) != 0)
                        {
                            return 0;
                        }
                        spinlocks |= bit;
                        return bit;
                    }
                    return regs[offset >> 2];
            }
        }

        private void WriteLocked(long offset, uint value)
        {
            var core = CurrentCore;
            switch(offset)
            {
                case GpioOutSet: regs[GpioOut >> 2] |= value; UpdatePins(); return;
                case GpioOutClr: regs[GpioOut >> 2] &= ~value; UpdatePins(); return;
                case GpioOutXor: regs[GpioOut >> 2] ^= value; UpdatePins(); return;
                case GpioHiOutSet: regs[GpioHiOut >> 2] |= value; UpdatePins(); return;
                case GpioHiOutClr: regs[GpioHiOut >> 2] &= ~value; UpdatePins(); return;
                case GpioHiOutXor: regs[GpioHiOut >> 2] ^= value; UpdatePins(); return;
                case GpioOeSet: regs[GpioOe >> 2] |= value; return;
                case GpioOeClr: regs[GpioOe >> 2] &= ~value; return;
                case GpioOeXor: regs[GpioOe >> 2] ^= value; return;
                case GpioHiOeSet: regs[GpioHiOe >> 2] |= value; return;
                case GpioHiOeClr: regs[GpioHiOe >> 2] &= ~value; return;
                case GpioHiOeXor: regs[GpioHiOe >> 2] ^= value; return;
                case FifoSt:
                    // write 1 to clear the sticky error flags
                    if((value & StWof) != 0)
                    {
                        wof[core] = false;
                    }
                    if((value & StRoe) != 0)
                    {
                        roe[core] = false;
                    }
                    UpdateInterrupts();
                    return;
                case FifoWr:
                    FifoWrite(core, value);
                    return;
                case DoorbellOutSet:
                    if(core == 0 && XipLockAddress != 0)
                    {
                        var bus = machine.SystemBus;
                        if(bus.ReadDoubleWord(XipLockAddress) == 1)
                        {
                            bus.WriteDoubleWord(XipLockAddress, 2);
                        }
                        return;
                    }
                    doorbellIn[1 - core] |= value & DoorbellMask;
                    UpdateInterrupts();
                    return;
                case DoorbellOutClr:
                    doorbellIn[1 - core] &= ~(value & DoorbellMask);
                    UpdateInterrupts();
                    return;
                case DoorbellInSet:
                    doorbellIn[core] |= value & DoorbellMask;
                    UpdateInterrupts();
                    return;
                case DoorbellInClr:
                    doorbellIn[core] &= ~(value & DoorbellMask);
                    UpdateInterrupts();
                    return;
                default:
                    if(offset >= Spinlock0 && offset < Spinlock0 + 4 * 32)
                    {
                        spinlocks &= ~(1u << (int)((offset - Spinlock0) >> 2));
                        return;
                    }
                    regs[offset >> 2] = value;
                    if(offset == GpioOut || offset == GpioHiOut)
                    {
                        UpdatePins();
                    }
                    return;
            }
        }

        private void UpdatePins()
        {
            var lo = regs[GpioOut >> 2];
            var hi = regs[GpioHiOut >> 2];
            for(var pin = 0; pin < NumPins; pin++)
            {
                var level = pin < 32 ? (lo >> pin) & 1 : (hi >> (pin - 32)) & 1;
                Connections[PinOutputBase + pin].Set(level != 0);
            }
        }

        private void FifoWrite(int core, uint value)
        {
            if(core == 0 && !core1Launched)
            {
                RomLaunchStep(value);
                return;
            }
            var dest = rx[1 - core];
            if(dest.Count >= FifoDepth)
            {
                wof[core] = true;
            }
            else
            {
                dest.Enqueue(value);
            }
            UpdateInterrupts();
        }

        // Core1's boot ROM, reduced to the launch protocol: echo every word and
        // launch once the last six received are 0, 0, 1, VTOR, SP, entry.
        private void RomLaunchStep(uint value)
        {
            launchSeq.Add(value);
            if(launchSeq.Count > 6)
            {
                launchSeq.RemoveAt(0);
            }
            if(rx[0].Count < FifoDepth)
            {
                rx[0].Enqueue(value);
            }
            var n = launchSeq.Count;
            if(n >= 6 && launchSeq[n - 6] == 0 && launchSeq[n - 5] == 0 && launchSeq[n - 4] == 1)
            {
                var vtor = launchSeq[n - 3];
                var sp = launchSeq[n - 2];
                var entry = launchSeq[n - 1];
                launchSeq.Clear();
                core1Launched = true;
                this.Log(LogLevel.Info, "core1 launch: VTOR=0x{0:X8} SP=0x{1:X8} entry=0x{2:X8}", vtor, sp, entry);
                machine.LocalTimeSource.ExecuteInNearestSyncedState(_ =>
                {
                    core1.VectorTableOffset = vtor;
                    core1.SP = sp;
                    core1.PC = entry;
                    core1.IsHalted = false;
                });
            }
            UpdateInterrupts();
        }

        private uint FifoStatus(int core)
        {
            var status = 0u;
            if(rx[core].Count > 0)
            {
                status |= StVld;
            }
            var peerFull = core == 0 && !core1Launched ? false : rx[1 - core].Count >= FifoDepth;
            if(!peerFull)
            {
                status |= StRdy;
            }
            if(wof[core])
            {
                status |= StWof;
            }
            if(roe[core])
            {
                status |= StRoe;
            }
            return status;
        }

        private void UpdateInterrupts()
        {
            for(var core = 0; core < 2; core++)
            {
                var fifoIrq = rx[core].Count > 0 || wof[core] || roe[core];
                Connections[core].Set(fifoIrq);
                Connections[2 + core].Set(doorbellIn[core] != 0);
            }
        }

        private int CurrentCore
        {
            get
            {
                var cpu = machine.SystemBus.GetCurrentCPU();
                return ReferenceEquals(cpu, core1) ? 1 : 0;
            }
        }

        private readonly IMachine machine;
        private readonly CortexM core1;
        private readonly object accessLock = new object();
        private readonly Queue<uint>[] rx = new Queue<uint>[2];
        private readonly bool[] wof = new bool[2];
        private readonly bool[] roe = new bool[2];
        private readonly uint[] doorbellIn = new uint[2];
        private readonly List<uint> launchSeq = new List<uint>();
        private uint[] regs;
        private uint spinlocks;
        private bool core1Launched;

        private const int FifoDepth = 4;
        private const int NumPins = 48;
        private const int PinOutputBase = 100;
        private const uint StVld = 1u << 0;
        private const uint StRdy = 1u << 1;
        private const uint StWof = 1u << 2;
        private const uint StRoe = 1u << 3;
        private const uint DoorbellMask = 0xFF;

        private const long CpuId = 0x000;
        private const long GpioIn = 0x004;
        private const long GpioHiIn = 0x008;
        private const long GpioOut = 0x010;
        private const long GpioHiOut = 0x014;
        private const long GpioOutSet = 0x018;
        private const long GpioHiOutSet = 0x01C;
        private const long GpioOutClr = 0x020;
        private const long GpioHiOutClr = 0x024;
        private const long GpioOutXor = 0x028;
        private const long GpioHiOutXor = 0x02C;
        private const long GpioOe = 0x030;
        private const long GpioHiOe = 0x034;
        private const long GpioOeSet = 0x038;
        private const long GpioHiOeSet = 0x03C;
        private const long GpioOeClr = 0x040;
        private const long GpioHiOeClr = 0x044;
        private const long GpioOeXor = 0x048;
        private const long GpioHiOeXor = 0x04C;
        private const long FifoSt = 0x050;
        private const long FifoWr = 0x054;
        private const long FifoRd = 0x058;
        private const long SpinlockSt = 0x05C;
        private const long Spinlock0 = 0x100;
        private const long DoorbellOutSet = 0x180;
        private const long DoorbellOutClr = 0x184;
        private const long DoorbellInSet = 0x188;
        private const long DoorbellInClr = 0x18C;
    }
}
