// STM32H7 QUADSPI compatibility for the ChibiOS driver used by ArduPilot.
//
// STM32H7 rev. Y devices require command-only transfers to put the command in
// ABR and use the alternate-byte phase (STM32 erratum 2.2.9).  Renode's stock
// controller otherwise waits forever because its transfer trigger assumes an
// instruction phase.  Keep the stock data-path model and translate that one
// hardware workaround back into the equivalent SPI transaction.
using Antmicro.Renode.Peripherals;
using Antmicro.Renode.Peripherals.Bus;
using Antmicro.Renode.Peripherals.SPI;

namespace Antmicro.Renode.Peripherals.Miscellaneous
{
    public class AP_STM32H7_QuadSPI : IDoubleWordPeripheral, IWordPeripheral,
        IBytePeripheral, IKnownSize
    {
        public AP_STM32H7_QuadSPI(STM32H7_QuadSPI controller)
        {
            this.controller = controller;
        }

        public void Reset()
        {
            alternateBytes = 0;
            CommandCount = 0;
            WorkaroundCount = 0;
            LastOriginalCommunicationConfiguration = 0;
            LastCommunicationConfiguration = 0;
            controller.Reset();
        }

        public uint ReadDoubleWord(long offset)
        {
            return controller.ReadDoubleWord(offset);
        }

        public ushort ReadWord(long offset)
        {
            return controller.ReadWord(offset);
        }

        public byte ReadByte(long offset)
        {
            return controller.ReadByte(offset);
        }

        public void WriteDoubleWord(long offset, uint value)
        {
            if(offset == CommunicationConfiguration)
            {
                LastOriginalCommunicationConfiguration = value;
            }
            if(offset == AlternateBytes)
            {
                alternateBytes = value;
            }
            else if(offset == CommunicationConfiguration &&
                    (value & InstructionModeMask) == 0 &&
                    (value & AlternateBytesModeMask) != 0 &&
                    (value & AddressModeMask) == 0 &&
                    (value & DataModeMask) == 0)
            {
                WorkaroundCount++;
                var instructionMode = (value & AlternateBytesModeMask) >> 6;
                value = (value & ~AlternateBytesModeMask) |
                    instructionMode | (alternateBytes & 0xFF);
            }
            if(offset == CommunicationConfiguration)
            {
                LastCommunicationConfiguration = value;
                CommandCount++;
                // The SPI flash adapter models dummy clocks as a phase with
                // no byte transfer.  Avoid the stock controller converting
                // them into data bytes at this model boundary.
                value &= ~DummyCyclesMask;
            }
            controller.WriteDoubleWord(offset, value);
        }

        public void WriteWord(long offset, ushort value)
        {
            controller.WriteWord(offset, value);
        }

        public void WriteByte(long offset, byte value)
        {
            controller.WriteByte(offset, value);
        }

        public long Size => controller.Size;
        public ulong CommandCount { get; private set; }
        public ulong WorkaroundCount { get; private set; }
        public uint LastOriginalCommunicationConfiguration { get; private set; }
        public uint LastCommunicationConfiguration { get; private set; }

        private readonly STM32H7_QuadSPI controller;
        private uint alternateBytes;

        private const long CommunicationConfiguration = 0x14;
        private const long AlternateBytes = 0x1C;
        private const uint InstructionModeMask = 0x3U << 8;
        private const uint AddressModeMask = 0x3U << 10;
        private const uint AlternateBytesModeMask = 0x3U << 14;
        private const uint DummyCyclesMask = 0x1FU << 18;
        private const uint DataModeMask = 0x3U << 24;
    }
}
