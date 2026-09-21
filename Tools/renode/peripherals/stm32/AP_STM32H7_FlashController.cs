// STM32H7 flash programming fixup for CPU memory-write hooks.
//
// Renode's stock controller validates 32-byte write sequences, but writes from
// the Cortex-M7 path used here are not committed to the backing MappedMemory.
// Preserve the stock register/erase behavior and commit the value delivered by
// the CPU hook while programming is enabled.
using System.Linq;

using Antmicro.Renode.Core;
using Antmicro.Renode.Logging.Profiling;
using Antmicro.Renode.Peripherals.Bus;
using Antmicro.Renode.Peripherals.CPU;
using Antmicro.Renode.Peripherals.Memory;

namespace Antmicro.Renode.Peripherals.MTD
{
    [AllowedTranslations(AllowedTranslation.ByteToDoubleWord | AllowedTranslation.WordToDoubleWord)]
    public class AP_STM32H7_FlashController : STM32H7_FlashController
    {
        public AP_STM32H7_FlashController(IMachine machine, MappedMemory flash1,
            MappedMemory flash2) : base(machine, flash1, flash2)
        {
            this.machine = machine;
            this.flash1 = flash1;
            this.flash2 = flash2;
        }

        public override void Reset()
        {
            base.Reset();
            bank1Programming = false;
            bank2Programming = false;
            UpdateMemoryHook();
        }

        public override void WriteDoubleWord(long offset, uint value)
        {
            // ArduPilot clears all status bits with ~0. Pass only implemented
            // clear fields to the stock model to avoid a warning per flash line.
            if(offset == ClearControlBank1 || offset == ClearControlBank2)
            {
                value &= ImplementedClearBits;
            }
            base.WriteDoubleWord(offset, value);

            if(offset == ControlBank1)
            {
                bank1Programming = (value & ProgramBit) != 0;
                UpdateMemoryHook();
            }
            else if(offset == ControlBank2)
            {
                bank2Programming = (value & ProgramBit) != 0;
                UpdateMemoryHook();
            }
        }

        private void UpdateMemoryHook()
        {
            if(machine == null)
            {
                return;
            }
            var hook = bank1Programming || bank2Programming ?
                (MemoryAccessHook)HandleMemoryWrite : null;
            foreach(var cpu in machine.SystemBus.GetCPUs().OfType<ICPUWithMemoryAccessHooks>())
            {
                cpu.SetHookAtMemoryAccess(hook);
            }
        }

        private void HandleMemoryWrite(ulong _, MemoryOperation operation, ulong __,
            ulong physicalAddress, uint width, ulong value)
        {
            if(operation != MemoryOperation.MemoryWrite &&
                operation != MemoryOperation.MemoryIOWrite)
            {
                return;
            }

            if(bank1Programming && physicalAddress >= FlashBase &&
                physicalAddress < FlashBase + (ulong)flash1.Size)
            {
                Commit(flash1, physicalAddress - FlashBase, width, value);
            }
            else if(bank2Programming && physicalAddress >= FlashBank2Base &&
                    physicalAddress < FlashBank2Base + (ulong)flash2.Size)
            {
                Commit(flash2, physicalAddress - FlashBank2Base, width, value);
            }
        }

        private static void Commit(MappedMemory memory, ulong offset, uint width, ulong value)
        {
            switch(width)
            {
            case 1:
                memory.WriteByte((long)offset, (byte)value);
                break;
            case 2:
                memory.WriteWord((long)offset, (ushort)value);
                break;
            case 4:
                memory.WriteDoubleWord((long)offset, (uint)value);
                break;
            case 8:
                memory.WriteQuadWord((long)offset, value);
                break;
            }
        }

        private bool bank1Programming;
        private bool bank2Programming;

        private readonly IMachine machine;
        private readonly MappedMemory flash1;
        private readonly MappedMemory flash2;

        private const long ControlBank1 = 0x00C;
        private const long ClearControlBank1 = 0x014;
        private const long ControlBank2 = 0x10C;
        private const long ClearControlBank2 = 0x114;
        private const uint ProgramBit = 1U << 1;
        private const uint ImplementedClearBits = 0x06650000;
        private const ulong FlashBase = 0x08000000;
        private const ulong FlashBank2Base = 0x08100000;
    }
}
