// STM32F4 flash controller with the five-bit sector number used by 2 MiB F42x.
using System;

using Antmicro.Renode.Core;
using Antmicro.Renode.Logging;
using Antmicro.Renode.Peripherals;
using Antmicro.Renode.Peripherals.Bus;

namespace Antmicro.Renode.Peripherals.MTD
{
    [AllowedTranslations(AllowedTranslation.ByteToDoubleWord |
        AllowedTranslation.WordToDoubleWord)]
    public class AP_STM32F4_FlashController : IDoubleWordPeripheral, IKnownSize
    {
        public AP_STM32F4_FlashController(IMachine machine)
        {
            this.machine = machine;
        }

        public void Reset()
        {
            accessControl = 0;
            control = 1U << 31;
        }

        public uint ReadDoubleWord(long offset)
        {
            switch(offset)
            {
            case AccessControl:
                return accessControl;
            case Status:
                return 0;
            case Control:
                return control & ~(1U << 16);
            case OptionControl:
                return 0x0FFFAAED;
            default:
                return 0;
            }
        }

        public void WriteDoubleWord(long offset, uint value)
        {
            switch(offset)
            {
            case AccessControl:
                accessControl = value;
                break;
            case Control:
                control = value & ~(1U << 16);
                if((value & (1U << 16)) != 0)
                {
                    if((value & (1U << 2)) != 0)
                    {
                        Erase(0x08000000, 0x200000);
                    }
                    else if((value & (1U << 1)) != 0)
                    {
                        EraseSector((value >> 3) & 0x1F);
                    }
                }
                break;
            }
        }

        [ConnectionRegion("optionBytes")]
        public uint ReadDoubleWordFromOptionBytes(long offset)
        {
            return offset == 0 ? 0xAAECU : 0xFFFFFFFFU;
        }

        [ConnectionRegion("optionBytes")]
        public void WriteDoubleWordToOptionBytes(long offset, uint value)
        {
        }

        public long Size => 0x400;

        private void EraseSector(uint sector)
        {
            ulong offset;
            int size;
            var bankSector = sector;
            if(sector >= 12)
            {
                offset = 0x100000;
                bankSector -= 12;
            }
            else
            {
                offset = 0;
            }

            if(bankSector < 4)
            {
                offset += bankSector * 0x4000;
                size = 0x4000;
            }
            else if(bankSector == 4)
            {
                offset += 0x10000;
                size = 0x10000;
            }
            else if(bankSector < 12)
            {
                offset += 0x20000 + (bankSector - 5) * 0x20000;
                size = 0x20000;
            }
            else
            {
                this.Log(LogLevel.Warning,
                    "Tried to erase invalid STM32F4 sector {0}", sector);
                return;
            }
            Erase(0x08000000UL + offset, size);
        }

        private void Erase(ulong address, int size)
        {
            var erased = new byte[size];
            Array.Fill(erased, (byte)0xFF);
            machine.SystemBus.WriteBytes(erased, address);
        }

        private readonly IMachine machine;
        private uint accessControl;
        private uint control;

        private const long AccessControl = 0x00;
        private const long Status = 0x0C;
        private const long Control = 0x10;
        private const long OptionControl = 0x14;
    }
}
