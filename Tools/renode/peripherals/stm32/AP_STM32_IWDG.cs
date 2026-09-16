//
// Independent watchdog model. ArduPilot arms the IWDG by default, which
// interferes with debugging when Renode advances peripheral time while the
// CPU is paused. Reset enforcement therefore remains opt-in. Flash crashdump
// tests enable it because the legacy flash backend relies on the watchdog to
// reboot after recording a fault.
//
using Antmicro.Renode.Core;
using Antmicro.Renode.Logging;
using Antmicro.Renode.Peripherals;
using Antmicro.Renode.Peripherals.Bus;
using Antmicro.Renode.Peripherals.Timers;

namespace Antmicro.Renode.Peripherals.Miscellaneous
{
    [AllowedTranslations(AllowedTranslation.ByteToDoubleWord | AllowedTranslation.WordToDoubleWord)]
    public class AP_STM32_IWDG : IDoubleWordPeripheral, IKnownSize
    {
        public AP_STM32_IWDG(IMachine machine, ulong frequency = LsiFrequency)
        {
            this.machine = machine;
            watchdogTimer = new LimitTimer(machine.ClockSource, frequency, this,
                "AP_STM32_IWDG", reload + 1, enabled: false,
                eventEnabled: true, autoUpdate: true, divider: 4);
            watchdogTimer.LimitReached += () =>
            {
                this.Log(LogLevel.Info, "Watchdog timed out; resetting the machine");
                machine.RequestReset();
            };
            Reset();
        }

        public long Size => 0x400;

        public void Reset()
        {
            watchdogTimer.Reset();
            prescaler = 0;
            reload = 0xFFF;
            window = 0xFFF;
            started = false;
            Kicks = 0;
        }

        // how many times the firmware has reloaded the counter
        public ulong Kicks { get; private set; }

        public bool EnforceReset
        {
            get => enforceReset;
            set
            {
                enforceReset = value;
                watchdogTimer.Enabled = value && started;
                if(watchdogTimer.Enabled)
                {
                    Reload();
                }
            }
        }

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
                case 0xAAAA:
                    Kicks++;
                    if(watchdogTimer.Enabled)
                    {
                        Reload();
                    }
                    break;
                case 0x5555: break;
                case 0xCCCC:
                    started = true;
                    if(enforceReset)
                    {
                        Reload();
                        watchdogTimer.Enabled = true;
                    }
                    break;
                default:
                    this.Log(LogLevel.Warning, "unknown key 0x{0:X} to IWDG_KR", value);
                    break;
                }
                return;
            case PR:
                prescaler = value & 7;
                watchdogTimer.Divider = PrescalerDivider;
                return;
            case RLR:
                reload = value & 0xFFF;
                if(watchdogTimer.Enabled)
                {
                    Reload();
                }
                return;
            case WINR: window = value & 0xFFF; return;
            }
        }

        private void Reload()
        {
            watchdogTimer.Limit = reload + 1UL;
        }

        private ulong PrescalerDivider
        {
            get
            {
                var divider = 4UL << (int)prescaler;
                return divider > 256 ? 256 : divider;
            }
        }

        private const long KR = 0x00;
        private const long PR = 0x04;
        private const long RLR = 0x08;
        private const long SR = 0x0C;
        private const long WINR = 0x10;

        private uint prescaler;
        private bool enforceReset;
        private bool started;
        private uint reload = 0xFFF;
        private uint window = 0xFFF;

        private readonly IMachine machine;
        private readonly LimitTimer watchdogTimer;

        private const ulong LsiFrequency = 32000;
    }
}
