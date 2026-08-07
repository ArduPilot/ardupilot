// Renode's generic SPI multiplexer defaults its GPIO chip selects to active
// high. ArduPilot STM32 hwdefs use active-low GPIO chip selects, so configure
// every possible input before platform wiring supplies its initial level.
//
using Antmicro.Renode.Core;
using Antmicro.Renode.Peripherals.SPI;

namespace Antmicro.Renode.Peripherals.Miscellaneous
{
    public class AP_SPIMultiplexer : SPIMultiplexer
    {
        public AP_SPIMultiplexer(IMachine machine) : base(machine)
        {
            for(var index = 0; index < MaximumChipSelects; index++)
            {
                SetActiveLow(index);
            }
        }

        private const int MaximumChipSelects = 32;
    }
}
