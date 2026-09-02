//
// Adds the STM32 timer update-DMA request omitted by Renode's stock
// STM32_Timer model. ArduPilot's DShot and serial-ESC output use TIMx_DIER.UDE
// to request one DMA transfer on every timer overflow. Without this signal the
// DMA stream remains at its initial NDTR and firmware can only leave the send
// through its timeout path.
//
// The stock timer already schedules overflows at the programmed PSC/ARR rate,
// so subscribe to its LimitReached event and emit the missing request only
// while UDE is set. DMAMUX then routes this fixed peripheral request to the
// stream selected by firmware, just as on the H743.
//
using Antmicro.Renode.Core;
using Antmicro.Renode.Peripherals;
using Antmicro.Renode.Peripherals.Bus;
using Antmicro.Renode.Peripherals.Timers;

namespace Antmicro.Renode.Peripherals.Miscellaneous
{
    public class AP_STM32_Timer_UpdateDMA : IDoubleWordPeripheral, IKnownSize
    {
        public AP_STM32_Timer_UpdateDMA(STM32_Timer timer)
        {
            timer.LimitReached += () =>
            {
                if((timer.ReadDoubleWord(DmaOrInterruptEnable) & UpdateDmaEnable) != 0)
                {
                    UpdateDMA.Blink();
                }
            };
        }

        public GPIO UpdateDMA { get; } = new GPIO();

        public long Size => 4;
        public uint ReadDoubleWord(long offset) => 0;
        public void WriteDoubleWord(long offset, uint value) { }
        public void Reset() { }

        private const long DmaOrInterruptEnable = 0x0C;
        private const uint UpdateDmaEnable = 1u << 8;
    }
}
