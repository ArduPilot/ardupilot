// STM32 ADC model with the 16-bit data-register access used by ChibiOS DMA.
// Renode's transfer model is otherwise sufficient, so retain it unchanged.
using Antmicro.Renode.Core;
using Antmicro.Renode.Peripherals.Bus;

namespace Antmicro.Renode.Peripherals.Analog
{
    public class AP_STM32_ADC : STM32_ADC, IWordPeripheral
    {
        public AP_STM32_ADC(IMachine machine) : base(machine)
        {
        }

        public ushort ReadWord(long offset)
        {
            return (ushort)ReadDoubleWord(offset);
        }

        public void WriteWord(long offset, ushort value)
        {
            WriteDoubleWord(offset, value);
        }
    }
}
