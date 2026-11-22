// Minimal STM32H7 MDMA model for the byte-wide QUADSPI transfers used by
// ChibiOS.  The production driver configures one channel with a fixed QSPI DR
// endpoint and an incrementing memory endpoint.
using Antmicro.Renode.Core;
using Antmicro.Renode.Peripherals;
using Antmicro.Renode.Peripherals.Bus;

namespace Antmicro.Renode.Peripherals.DMA
{
    public class AP_STM32H7_MDMA : IDoubleWordPeripheral, IKnownSize
    {
        public AP_STM32H7_MDMA(IMachine machine)
        {
            bus = machine.SystemBus;
            IRQ = new GPIO();
        }

        public void Reset()
        {
            foreach(var state in channels)
            {
                state.Reset();
            }
            TransferCount = 0;
            BytesTransferred = 0;
            IRQ.Unset();
        }

        public uint ReadDoubleWord(long offset)
        {
            if(offset == GlobalInterruptStatus)
            {
                uint result = 0;
                for(var i = 0; i < channels.Length; i++)
                {
                    if(channels[i].Status != 0)
                    {
                        result |= 1U << i;
                    }
                }
                return result;
            }
            if(!TryRegister(offset, out var channel, out var register))
            {
                return 0;
            }
            var state = channels[channel];
            switch(register)
            {
            case ChannelStatus: return state.Status;
            case ChannelErrorStatus: return 0;
            case ChannelConfiguration: return state.Configuration;
            case ChannelTransferConfiguration: return state.TransferConfiguration;
            case ChannelBlockDataLength: return state.BlockDataLength;
            case ChannelSourceAddress: return state.SourceAddress;
            case ChannelDestinationAddress: return state.DestinationAddress;
            case ChannelBlockRepeat: return state.BlockRepeat;
            case ChannelLinkAddress: return state.LinkAddress;
            case ChannelTrigger: return state.Trigger;
            case ChannelMaskAddress: return state.MaskAddress;
            case ChannelMaskData: return state.MaskData;
            default: return 0;
            }
        }

        public void WriteDoubleWord(long offset, uint value)
        {
            if(!TryRegister(offset, out var channel, out var register))
            {
                return;
            }
            var state = channels[channel];
            switch(register)
            {
            case ChannelInterruptClear:
                state.Status &= ~value;
                UpdateInterrupt();
                break;
            case ChannelConfiguration:
                state.Configuration = value;
                if((value & ChannelEnable) != 0)
                {
                    Transfer(state);
                }
                break;
            case ChannelTransferConfiguration: state.TransferConfiguration = value; break;
            case ChannelBlockDataLength: state.BlockDataLength = value; break;
            case ChannelSourceAddress: state.SourceAddress = value; break;
            case ChannelDestinationAddress: state.DestinationAddress = value; break;
            case ChannelBlockRepeat: state.BlockRepeat = value; break;
            case ChannelLinkAddress: state.LinkAddress = value; break;
            case ChannelTrigger: state.Trigger = value; break;
            case ChannelMaskAddress: state.MaskAddress = value; break;
            case ChannelMaskData: state.MaskData = value; break;
            }
        }

        public GPIO IRQ { get; }
        public long Size => 0x1000;
        public ulong TransferCount { get; private set; }
        public ulong BytesTransferred { get; private set; }

        private void Transfer(Channel state)
        {
            var count = state.BlockDataLength & 0x1FFFFU;
            TransferCount++;
            BytesTransferred += count;
            var source = state.SourceAddress;
            var destination = state.DestinationAddress;
            var sourceIncrement = (state.TransferConfiguration & 0x3U) == 0x2U;
            var destinationIncrement = ((state.TransferConfiguration >> 2) & 0x3U) == 0x2U;
            for(uint i = 0; i < count; i++)
            {
                bus.WriteByte(destination, bus.ReadByte(source));
                if(sourceIncrement)
                {
                    source++;
                }
                if(destinationIncrement)
                {
                    destination++;
                }
            }
            state.SourceAddress = source;
            state.DestinationAddress = destination;
            state.BlockDataLength &= ~0x1FFFFU;
            state.Configuration &= ~ChannelEnable;
            state.Status |= ChannelTransferComplete;
            UpdateInterrupt();
        }

        private void UpdateInterrupt()
        {
            foreach(var state in channels)
            {
                if((state.Status & ChannelTransferComplete) != 0 &&
                    (state.Configuration & ChannelTransferCompleteInterrupt) != 0)
                {
                    IRQ.Set();
                    return;
                }
            }
            IRQ.Unset();
        }

        private static bool TryRegister(long offset, out int channel, out long register)
        {
            var relative = offset - FirstChannel;
            channel = (int)(relative / ChannelStride);
            register = relative % ChannelStride;
            return relative >= 0 && channel < ChannelCount;
        }

        private sealed class Channel
        {
            public void Reset()
            {
                Status = 0;
                Configuration = 0;
                TransferConfiguration = 0;
                BlockDataLength = 0;
                SourceAddress = 0;
                DestinationAddress = 0;
                BlockRepeat = 0;
                LinkAddress = 0;
                Trigger = 0;
                MaskAddress = 0;
                MaskData = 0;
            }

            public uint Status;
            public uint Configuration;
            public uint TransferConfiguration;
            public uint BlockDataLength;
            public uint SourceAddress;
            public uint DestinationAddress;
            public uint BlockRepeat;
            public uint LinkAddress;
            public uint Trigger;
            public uint MaskAddress;
            public uint MaskData;
        }

        private readonly Channel[] channels = {
            new Channel(), new Channel(), new Channel(), new Channel(),
            new Channel(), new Channel(), new Channel(), new Channel(),
            new Channel(), new Channel(), new Channel(), new Channel(),
            new Channel(), new Channel(), new Channel(), new Channel(),
        };
        private readonly IBusController bus;

        private const int ChannelCount = 16;
        private const long GlobalInterruptStatus = 0x0;
        private const long FirstChannel = 0x40;
        private const long ChannelStride = 0x40;
        private const long ChannelStatus = 0x00;
        private const long ChannelInterruptClear = 0x04;
        private const long ChannelErrorStatus = 0x08;
        private const long ChannelConfiguration = 0x0C;
        private const long ChannelTransferConfiguration = 0x10;
        private const long ChannelBlockDataLength = 0x14;
        private const long ChannelSourceAddress = 0x18;
        private const long ChannelDestinationAddress = 0x1C;
        private const long ChannelBlockRepeat = 0x20;
        private const long ChannelLinkAddress = 0x24;
        private const long ChannelTrigger = 0x28;
        private const long ChannelMaskAddress = 0x30;
        private const long ChannelMaskData = 0x34;
        private const uint ChannelEnable = 1U;
        private const uint ChannelTransferCompleteInterrupt = 1U << 2;
        private const uint ChannelTransferComplete = 1U << 1;
    }
}
