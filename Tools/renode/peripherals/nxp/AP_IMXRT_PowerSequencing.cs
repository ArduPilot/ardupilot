//
// The RT1176 analog blocks that the NXP SDK spins on during boot, and that
// Renode does not model: DCDC and ANADIG_PMU. Each one is written and then
// polled for a "done" bit that plain memory can never produce, because the
// firmware's own write clears it.
//
// This is a stub, not a model. It stores what is written and forces the ready
// bits on read, which is all a boot needs: nothing in ArduPilot reads back a
// regulator voltage, it only waits for the regulator to settle.
//
// Without it the board spins in PMU_EnableBodyBias() or in
// DCDC_SetVDD1P0BuckModeTargetVoltage() and never reaches main().
//
using System.Collections.Generic;
using Antmicro.Renode.Core;
using Antmicro.Renode.Peripherals;
using Antmicro.Renode.Peripherals.Bus;

namespace Antmicro.Renode.Peripherals.Miscellaneous
{
    [AllowedTranslations(AllowedTranslation.ByteToDoubleWord | AllowedTranslation.WordToDoubleWord)]
    public class AP_IMXRT_PowerSequencing : IDoubleWordPeripheral, IKnownSize
    {
        public AP_IMXRT_PowerSequencing(IMachine machine)
        {
            this.machine = machine;
            registers = new Dictionary<long, uint>();
        }

        public long Size => 0x1000;

        public void Reset()
        {
            registers.Clear();
        }

        public uint ReadDoubleWord(long offset)
        {
            uint value;
            if(!registers.TryGetValue(offset, out value))
            {
                value = 0;
            }
            uint forced;
            if(ReadyBits.TryGetValue(offset, out forced))
            {
                value |= forced;
            }
            return value;
        }

        public void WriteDoubleWord(long offset, uint value)
        {
            if(ToggleRegisters.Contains(offset))
            {
                // ANATOP's analog interface is a handshake: the firmware flips
                // AITOGGLE and spins until AITOGGLE_DONE flips to match. Mirror
                // it on the write, so the wait ends on the first read.
                if((value & AiToggle) != 0)
                {
                    value |= AiToggleDone;
                }
                else
                {
                    value &= ~AiToggleDone;
                }
            }
            registers[offset] = value;
        }

        // offset -> bits that always read as set
        private static readonly Dictionary<long, uint> ReadyBits = new Dictionary<long, uint>
        {
            // DCDC REG0.STS_DC_OK - the buck converter has settled
            { 0x008, 0x80000000 },
            // ANADIG_PMU PMU_BIAS_CTRL2.WB_OK - body bias is applied
            { 0x560, 0x04000000 },
        };

        // ANADIG_MISC VDDSOC/VDDLPSR analog-interface control registers. They
        // share a layout: AITOGGLE is bit 8, AITOGGLE_DONE is bit 9.
        private const uint AiToggle = 0x100;
        private const uint AiToggleDone = 0x200;

        private static readonly HashSet<long> ToggleRegisters = new HashSet<long>
        {
            0x820,  // VDDSOC_AI_CTRL
            0x850,  // VDDSOC2PLL_AI_CTRL_1G
            0x880,  // VDDSOC2PLL_AI_CTRL_AUDIO
            0x8b0,  // VDDSOC2PLL_AI_CTRL_VIDEO
            0x8e0,  // VDDLPSR_AI_CTRL
            0x920,  // VDDLPSR_AI400M_CTRL
        };

        private readonly IMachine machine;
        private readonly Dictionary<long, uint> registers;
    }
}
