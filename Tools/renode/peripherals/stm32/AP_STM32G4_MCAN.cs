// Adapt Renode's programmable Bosch MCAN model to STM32G4's fixed message RAM.
using System;
using Antmicro.Renode.Core;
using Antmicro.Renode.Core.CAN;
using Antmicro.Renode.Peripherals;
using Antmicro.Renode.Peripherals.Bus;

namespace Antmicro.Renode.Peripherals.CAN
{
    public class AP_STM32G4_MCAN : IDoubleWordPeripheral, IKnownSize, ICAN
    {
        public AP_STM32G4_MCAN(IMachine machine,
                              IMultibyteWritePeripheral messageRAM,
                              uint messageRAMOffset)
        {
            this.messageRAMOffset = messageRAMOffset;
            can = new MCAN(machine, messageRAM);
            can.FrameSent += frame =>
            {
                var handler = FrameSent;
                if(handler != null)
                {
                    handler(frame);
                }
            };
            ConfigureFixedMessageRAM();
        }

        public void Reset()
        {
            can.Reset();
            ConfigureFixedMessageRAM();
        }

        public uint ReadDoubleWord(long offset)
        {
            return can.ReadDoubleWord(TranslateRegister(offset));
        }

        public void WriteDoubleWord(long offset, uint value)
        {
            // AP_HAL writes zero to TXBC because these values are fixed in G4
            // silicon. Keep the equivalent configuration in the generic model.
            if(offset == TxBufferConfiguration)
            {
                value = messageRAMOffset + TxFIFOOffset | 3U << 24;
            }
            can.WriteDoubleWord(TranslateRegister(offset), value);
        }

        public void OnFrameReceived(CANMessageFrame message)
        {
            can.OnFrameReceived(message);
        }

        public long Size => can.Size;
        public GPIO Line0 => can.Line0;
        public GPIO Line1 => can.Line1;
        public GPIO Calibration => can.Calibration;
        public event Action<CANMessageFrame> FrameSent;

        private void ConfigureFixedMessageRAM()
        {
            can.WriteDoubleWord(StandardIDFilterConfiguration,
                messageRAMOffset | 28U << 16);
            can.WriteDoubleWord(ExtendedIDFilterConfiguration,
                messageRAMOffset + ExtendedFilterOffset | 8U << 16);
            can.WriteDoubleWord(RxFIFO0Configuration,
                messageRAMOffset + RxFIFO0Offset | 3U << 16);
            can.WriteDoubleWord(RxFIFO1Configuration,
                messageRAMOffset + RxFIFO1Offset | 3U << 16);
            can.WriteDoubleWord(RxBufferElementSizeConfiguration, 0x777);
            can.WriteDoubleWord(TxBufferConfiguration,
                messageRAMOffset + TxFIFOOffset | 3U << 24);
            can.WriteDoubleWord(GenericTxBufferElementSizeConfiguration, 0x7);
            can.WriteDoubleWord(GenericTxEventFIFOConfiguration,
                messageRAMOffset + TxEventFIFOOffset | 3U << 16);
        }

        private static long TranslateRegister(long offset)
        {
            // STM32G4 omits the programmable message-RAM configuration
            // registers, so registers after GFC do not use the Bosch M_CAN
            // offsets implemented by Renode's generic model.
            switch(offset)
            {
            case 0x84: return 0x90; // XIDAM
            case 0x88: return 0x94; // HPMS
            case 0x90: return 0xA4; // RXF0S
            case 0x94: return 0xA8; // RXF0A
            case 0x98: return 0xB4; // RXF1S
            case 0x9C: return 0xB8; // RXF1A
            case 0xC8: return 0xCC; // TXBRP
            case 0xCC: return 0xD0; // TXBAR
            case 0xD0: return 0xD4; // TXBCR
            case 0xD4: return 0xD8; // TXBTO
            case 0xD8: return 0xDC; // TXBCF
            case 0xDC: return 0xE0; // TXBTIE
            case 0xE0: return 0xE4; // TXBCIE
            case 0xE4: return 0xF4; // TXEFS
            case 0xE8: return 0xF8; // TXEFA
            default: return offset;
            }
        }

        private readonly MCAN can;
        private readonly uint messageRAMOffset;

        private const uint ExtendedFilterOffset = 0x70;
        private const uint RxFIFO0Offset = 0xB0;
        private const uint RxFIFO1Offset = 0x188;
        private const uint TxEventFIFOOffset = 0x260;
        private const uint TxFIFOOffset = 0x278;

        private const long StandardIDFilterConfiguration = 0x84;
        private const long ExtendedIDFilterConfiguration = 0x88;
        private const long RxFIFO0Configuration = 0xA0;
        private const long RxFIFO1Configuration = 0xB0;
        private const long RxBufferElementSizeConfiguration = 0xBC;
        private const long TxBufferConfiguration = 0xC0;
        private const long GenericTxBufferElementSizeConfiguration = 0xC8;
        private const long GenericTxEventFIFOConfiguration = 0xF0;
    }
}
