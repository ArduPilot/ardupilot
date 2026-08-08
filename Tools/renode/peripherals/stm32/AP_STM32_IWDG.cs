//
// Independent watchdog stub, taken from the AM32 Renode harness
// (AM32_STM32_IWDG). ArduPilot arms the IWDG by default (BRD_OPTIONS
// bit 0, 2048ms) and Renode's stock model really fires, which turns
// every pause-at-a-breakpoint into a reset. Kicks are accepted and
// counted but never enforced, so a test can still assert the firmware
// is feeding it.
//
using Antmicro.Renode.Core;
using Antmicro.Renode.Logging;
using Antmicro.Renode.Peripherals;
using Antmicro.Renode.Peripherals.Bus;

namespace Antmicro.Renode.Peripherals.Miscellaneous
{
    [AllowedTranslations(AllowedTranslation.ByteToDoubleWord | AllowedTranslation.WordToDoubleWord)]
    public class AP_STM32_IWDG : IDoubleWordPeripheral, IKnownSize
    {
        public long Size => 0x400;

        public void Reset()
        {
            prescaler = 0;
            reload = 0xFFF;
            window = 0xFFF;
            Kicks = 0;
        }

        // how many times the firmware has reloaded the counter
        public ulong Kicks { get; private set; }

        public uint ReadDoubleWord(long offset)
        {
            switch(offset)
            {
            case KR: return 0;
            case PR: return prescaler;
            case RLR: return reload;
            // PVU/RVU/WVU: polled waiting for a write to take effect.
            // Always settled here, so the loops fall straight through.
            case SR: return 0;
            case WINR: return window;
            default: return 0;
            }
        }

        public void WriteDoubleWord(long offset, uint value)
        {
            switch(offset)
            {
            case KR:
                switch(value & 0xFFFF)
                {
                case 0xAAAA: Kicks++; break;
                // 0x5555 unlocks PR/RLR, 0xCCCC starts the countdown.
                // Neither is enforced, so both are simply accepted.
                case 0x5555: break;
                case 0xCCCC: break;
                default:
                    this.Log(LogLevel.Warning, "unknown key 0x{0:X} to IWDG_KR", value);
                    break;
                }
                return;
            case PR: prescaler = value & 7; return;
            case RLR: reload = value & 0xFFF; return;
            case WINR: window = value & 0xFFF; return;
            }
        }

        private const long KR = 0x00;
        private const long PR = 0x04;
        private const long RLR = 0x08;
        private const long SR = 0x0C;
        private const long WINR = 0x10;

        private uint prescaler;
        private uint reload = 0xFFF;
        private uint window = 0xFFF;
    }
}
