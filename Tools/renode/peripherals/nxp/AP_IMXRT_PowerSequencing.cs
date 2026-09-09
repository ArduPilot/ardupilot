//
// The RT1176 analog blocks that the NXP SDK spins on during boot, and that
// Renode does not model: DCDC, and the ANADIG block that carries PMU, OSC and
// PLL. Each one is written and then polled for a "done" bit that plain memory
// can never produce, because the firmware's own write clears it. ANADIG_PMU,
// ANADIG_OSC and ANADIG_PLL all sit at 0x40c84000, so one instance covers the
// three of them.
//
// This is a stub, not a model. It stores what is written and forces the ready
// bits on read, which is all a boot needs: nothing in ArduPilot reads back a
// regulator voltage, it only waits for the regulator to settle.
//
// Without it the board spins in PMU_EnableBodyBias(), in
// DCDC_SetVDD1P0BuckModeTargetVoltage(), in clock_init() waiting for
// OSC_24M_STABLE, or in CLOCK_InitArmPll() and its siblings waiting for a PLL
// to lock, and never reaches main().
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
            if(PfdRegisters.Contains(offset))
            {
                // CLOCK_InitPfd() reads the PFD's STABLE bit, reconfigures the
                // fraction, and then waits for that bit to come back DIFFERENT.
                // A bit forced permanently high would hang it just as surely as
                // one stuck low, so flip these on every write instead. The
                // routine writes the register an odd number of times per PFD,
                // which leaves the bit changed by the time it looks.
                uint previous;
                if(!registers.TryGetValue(offset, out previous))
                {
                    previous = 0;
                }
                value = (value & ~PfdStableBits) | (~previous & PfdStableBits);
            }
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
            // ANADIG_OSC OSC_24M_CTRL.OSC_24M_STABLE - the crystal has started.
            // soc.c clock_init() spins here before anything else is clocked, so
            // without this bit the board never leaves SystemInit().
            { 0x020, 0x40000000 },
            // ANADIG_PLL <pll>_CTRL.<pll>_STABLE - the PLL has locked. Every
            // one of these is polled for the bit being set and never for it
            // being clear, including on the CLOCK_DeinitSysPll1() path, so
            // reading them as permanently locked ends each wait at once.
            { 0x200, 0x20000000 },  // ARM_PLL_CTRL
            { 0x210, 0x20000000 },  // SYS_PLL3_CTRL
            { 0x240, 0x20000000 },  // SYS_PLL2_CTRL
            { 0x2c0, 0x20000000 },  // SYS_PLL1_CTRL
            { 0x300, 0x20000000 },  // PLL_AUDIO_CTRL
            { 0x350, 0x20000000 },  // PLL_VIDEO_CTRL
        };

        // ANADIG_MISC VDDSOC/VDDLPSR analog-interface control registers. They
        // share a layout: AITOGGLE is bit 8, AITOGGLE_DONE is bit 9.
        private const uint AiToggle = 0x100;
        private const uint AiToggleDone = 0x200;

        // SYS_PLL2_PFD and SYS_PLL3_PFD. Each holds four PFDs, and each PFD's
        // STABLE bit is bit 6 of its own byte.
        private const uint PfdStableBits = 0x40404040;

        private static readonly HashSet<long> PfdRegisters = new HashSet<long>
        {
            0x230,  // SYS_PLL3_PFD
            0x270,  // SYS_PLL2_PFD
        };

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
