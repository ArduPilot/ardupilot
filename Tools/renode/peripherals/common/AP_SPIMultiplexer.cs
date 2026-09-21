// Renode's generic SPI multiplexer defaults its GPIO chip selects to active
// high. ArduPilot STM32 hwdefs use active-low GPIO chip selects, so configure
// every possible input before platform wiring supplies its initial level.
//
using Antmicro.Renode.Core;
using Antmicro.Renode.Peripherals.SPI;

namespace Antmicro.Renode.Peripherals.Miscellaneous
{
    public class AP_SPIMultiplexer : SPIMultiplexer, ISPIPeripheral
    {
        public AP_SPIMultiplexer(IMachine machine, bool frameOnTransfer = false) :
            base(machine, suppressExplicitFinishTransmission: !frameOnTransfer)
        {
            for(var index = 0; index < MaximumChipSelects; index++)
            {
                SetActiveLow(index);
            }
        }

        public new byte Transmit(byte data)
        {
            var received = base.Transmit(data);
            if(Analyzer != null)
            {
                Analyzer.ObserveSPI(data, received);
            }
            return received;
        }

        public new void FinishTransmission()
        {
            base.FinishTransmission();
        }

        public IAPSigrok Analyzer { get; set; }

        private const int MaximumChipSelects = 32;
    }
}
