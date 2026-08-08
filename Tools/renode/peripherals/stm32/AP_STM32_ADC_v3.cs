// Minimal STM32 ADC-v3 model for the L4 and H7 AP_Periph analog inputs.
using System.Collections.Generic;

using Antmicro.Renode.Core;
using Antmicro.Renode.Peripherals;
using Antmicro.Renode.Peripherals.Bus;

namespace Antmicro.Renode.Peripherals.Analog
{
    public class AP_STM32_ADC_v3 : IDoubleWordPeripheral, IWordPeripheral,
        IKnownSize
    {
        public AP_STM32_ADC_v3(IMachine machine)
        {
            converter = machine.ObtainManagedThread(
                Convert, ConversionFrequency,
                name: "AP STM32 ADC-v3 conversion", owner: this);
            converter.Start();
        }

        public void Reset()
        {
            registers.Clear();
            sequenceIndex = 0;
            DMARequest.Unset();
            IRQ.Unset();
        }

        public uint ReadDoubleWord(long offset)
        {
            var value = registers.TryGetValue(offset, out var stored) ? stored : 0;
            if(offset == Data)
            {
                SetRegister(InterruptStatus, Register(InterruptStatus) & ~EndOfConversion);
            }
            return value;
        }

        public void WriteDoubleWord(long offset, uint value)
        {
            switch(offset)
            {
            case InterruptStatus:
                SetRegister(offset, Register(offset) & ~value);
                UpdateInterrupt();
                return;
            case Control:
                if((value & Calibration) != 0)
                {
                    value &= ~Calibration;
                }
                if((value & Disable) != 0)
                {
                    value &= ~(Disable | Enable | Start);
                }
                if((value & Stop) != 0)
                {
                    value &= ~(Stop | Start);
                }
                if((value & Enable) != 0)
                {
                    SetRegister(InterruptStatus,
                        Register(InterruptStatus) | Ready);
                }
                if((value & Start) != 0)
                {
                    sequenceIndex = 0;
                }
                break;
            }
            SetRegister(offset, value);
            UpdateInterrupt();
        }

        public ushort ReadWord(long offset)
        {
            return (ushort)ReadDoubleWord(offset);
        }

        public void WriteWord(long offset, ushort value)
        {
            WriteDoubleWord(offset, value);
        }

        public void FeedSample(uint value, uint channel, int repeat = -1)
        {
            samples[channel] = value & 0xFFFF;
        }

        public GPIO DMARequest { get; } = new GPIO();
        public GPIO IRQ { get; } = new GPIO();
        public long Size => 0x400;

        private void Convert()
        {
            if((Register(Control) & Start) == 0)
            {
                return;
            }

            var sequence = Register(Sequence);
            var count = (int)(sequence & 0xF) + 1;
            var slot = sequenceIndex % count;
            uint channel;
            if(slot < 4)
            {
                channel = (sequence >> (6 + slot * 6)) & 0x1F;
            }
            else
            {
                var laterSlot = slot - 4;
                var sequenceRegister = Sequence + 4 + laterSlot / 5 * 4;
                channel = (Register(sequenceRegister) >> (laterSlot % 5 * 6)) & 0x1F;
            }
            var sample = samples.TryGetValue(channel, out var stored) ? stored : 0;
            SetRegister(Data, sample);

            var status = Register(InterruptStatus) | EndOfConversion;
            sequenceIndex++;
            if(sequenceIndex >= count)
            {
                sequenceIndex = 0;
                status |= EndOfSequence;
            }
            SetRegister(InterruptStatus, status);
            UpdateInterrupt();
            if((Register(Configuration) & DmaEnable) != 0)
            {
                DMARequest.Blink();
            }
        }

        private uint Register(long offset)
        {
            return registers.TryGetValue(offset, out var value) ? value : 0;
        }

        private void SetRegister(long offset, uint value)
        {
            registers[offset] = value;
        }

        private void UpdateInterrupt()
        {
            IRQ.Set((Register(InterruptStatus) & Register(InterruptEnable)) != 0);
        }

        private readonly IManagedThread converter;
        private readonly Dictionary<long, uint> registers = new Dictionary<long, uint>();
        private readonly Dictionary<uint, uint> samples = new Dictionary<uint, uint>();
        private int sequenceIndex;

        private const uint Ready = 1U;
        private const uint EndOfConversion = 1U << 2;
        private const uint EndOfSequence = 1U << 3;
        private const uint Enable = 1U;
        private const uint Disable = 1U << 1;
        private const uint Start = 1U << 2;
        private const uint Stop = 1U << 4;
        private const uint Calibration = 1U << 31;
        private const uint DmaEnable = 1U;
        private const long InterruptStatus = 0x00;
        private const long InterruptEnable = 0x04;
        private const long Control = 0x08;
        private const long Configuration = 0x0C;
        private const long Sequence = 0x30;
        private const long Data = 0x40;
        private const uint ConversionFrequency = 10000;
    }
}
