// STM32F1 flash page erasure used by the IOMCU bootloader. Program halfword
// writes already reach the backing MappedMemory through the CPU memory path;
// this model supplies the controller state and page erase operation around
// those writes.
using System;

using Antmicro.Renode.Core;
using Antmicro.Renode.Peripherals;
using Antmicro.Renode.Peripherals.Bus;
using Antmicro.Renode.Peripherals.Memory;

namespace Antmicro.Renode.Peripherals.MTD
{
    [AllowedTranslations(AllowedTranslation.ByteToDoubleWord |
        AllowedTranslation.WordToDoubleWord)]
    public class AP_STM32F1_FlashController : IDoubleWordPeripheral, IKnownSize
    {
        public AP_STM32F1_FlashController(MappedMemory flash)
        {
            this.flash = flash;
            erasedPage = new byte[PageSize];
            Array.Fill(erasedPage, (byte)0xFF);
            Reset();
        }

        public void Reset()
        {
            accessControl = 0;
            address = 0;
            control = Lock;
            keyIndex = 0;
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
                return control & ~Start;
            case Address:
                return address;
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
            case Key:
                HandleKey(value);
                break;
            case Status:
                break;
            case Control:
                WriteControl(value);
                break;
            case Address:
                address = value;
                break;
            }
        }

        public long Size => 0x400;

        private void HandleKey(uint value)
        {
            if((control & Lock) == 0)
            {
                return;
            }
            if(value != UnlockKeys[keyIndex])
            {
                keyIndex = 0;
                return;
            }
            keyIndex++;
            if(keyIndex == UnlockKeys.Length)
            {
                control &= ~Lock;
                keyIndex = 0;
            }
        }

        private void WriteControl(uint value)
        {
            var wasLocked = (control & Lock) != 0;
            control = value & ~Start;
            if(wasLocked && (value & Lock) == 0)
            {
                control |= Lock;
            }
            if((value & (PageErase | Start)) == (PageErase | Start) &&
                (control & Lock) == 0)
            {
                ErasePage(address);
            }
        }

        private void ErasePage(uint pageAddress)
        {
            if(pageAddress < FlashBase ||
                pageAddress >= FlashBase + flash.Size)
            {
                return;
            }
            var offset = (long)(pageAddress - FlashBase) & ~(PageSize - 1);
            flash.WriteBytes(offset, erasedPage);
        }

        private readonly MappedMemory flash;
        private readonly byte[] erasedPage;
        private uint accessControl;
        private uint address;
        private uint control;
        private int keyIndex;

        private static readonly uint[] UnlockKeys = { 0x45670123, 0xCDEF89AB };
        private const uint FlashBase = 0x08000000;
        private const int PageSize = 1024;
        private const uint PageErase = 1U << 1;
        private const uint Start = 1U << 6;
        private const uint Lock = 1U << 7;
        private const long AccessControl = 0x00;
        private const long Key = 0x04;
        private const long Status = 0x0C;
        private const long Control = 0x10;
        private const long Address = 0x14;
    }
}
