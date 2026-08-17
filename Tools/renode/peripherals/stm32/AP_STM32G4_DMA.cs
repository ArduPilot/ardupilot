// Adapt Renode's STM32G0 DMA model to the STM32G4 register behavior.
using System.Collections.Generic;

using Antmicro.Renode.Core;
using Antmicro.Renode.Peripherals;
using Antmicro.Renode.Peripherals.Bus;

namespace Antmicro.Renode.Peripherals.DMA
{
    public class AP_STM32G4_DMA : IDoubleWordPeripheral, IKnownSize,
        IGPIOReceiver, INumberedGPIOOutput
    {
        public AP_STM32G4_DMA(IMachine machine, int numberOfChannels)
        {
            dma = new STM32G0DMA(machine, numberOfChannels);
            channelEnabled = new bool[numberOfChannels];
            channelConfiguration = new uint[numberOfChannels];
            requestActive = new bool[numberOfChannels];
        }

        public void Reset()
        {
            dma.Reset();
            requestSelection = 0;
            for(var i = 0; i < channelEnabled.Length; i++)
            {
                channelEnabled[i] = false;
                channelConfiguration[i] = 0;
                requestActive[i] = false;
            }
        }

        public uint ReadDoubleWord(long offset)
        {
            if(offset == RequestSelection)
            {
                return requestSelection;
            }
            var value = dma.ReadDoubleWord(offset);
            if(TryGetChannelConfiguration(offset, out var channel) &&
                channel < channelEnabled.Length && channelEnabled[channel])
            {
                value |= ChannelEnable;
            }
            return value;
        }

        public void WriteDoubleWord(long offset, uint value)
        {
            if(offset == RequestSelection)
            {
                // STM32L4 selects fixed peripheral requests through CSELR.
                // The generated platform wires those requests directly.
                requestSelection = value;
                return;
            }
            var innerValue = value;
            if(offset == InterruptFlagClear)
            {
                // STM32G0DMA does not model transfer errors. Avoid warnings
                // for the corresponding G4 interrupt-clear bits.
                innerValue &= TransferFlagMask;
            }

            if(!TryGetChannelConfiguration(offset, out var channel))
            {
                dma.WriteDoubleWord(offset, innerValue);
                return;
            }
            if(channel >= channelEnabled.Length)
            {
                dma.WriteDoubleWord(offset, innerValue);
                return;
            }

            // Error interrupts and channel priority do not affect transfer
            // behavior in the wrapped model.
            innerValue &= ~UnsupportedConfigurationBits;
            var memoryToMemory = (value & MemoryToMemory) != 0;
            var memoryToPeripheral = (value & MemoryToPeripheral) != 0;
            if(!memoryToMemory && !memoryToPeripheral)
            {
                // STM32G0DMA starts peripheral-to-memory transfers as soon as
                // EN is written. STM32L4/G4 channels wait for a peripheral
                // request, so latch EN here and assert it from OnGPIO. Keep
                // memory-to-peripheral transfers immediate because Renode's
                // USART model has no transmit-DMA request output.
                innerValue &= ~ChannelEnable;
            }
            dma.WriteDoubleWord(offset, innerValue);

            channelEnabled[channel] = (value & ChannelEnable) != 0;
            channelConfiguration[channel] = innerValue;
            if(channelEnabled[channel] && requestActive[channel])
            {
                HandleRequest(channel);
            }
        }

        public void OnGPIO(int number, bool value)
        {
            if(number < 1 || number > channelEnabled.Length)
            {
                dma.OnGPIO(number, value);
                return;
            }

            var channel = number - 1;
            requestActive[channel] = value;
            if(!value)
            {
                dma.OnGPIO(number, false);
                return;
            }

            HandleRequest(channel);
        }

        private void HandleRequest(int channel)
        {
            var remaining = dma.ReadDoubleWord(ChannelDataCount(channel)) & 0xFFFF;
            if(!channelEnabled[channel] || remaining == 0)
            {
                // ChibiOS can rearm a channel between the request edge and
                // this callback. The next UART request will run normally.
                return;
            }

            // The wrapped STM32G0 model does not expose the enable bit on
            // register reads. Reassert the latched configuration before a
            // peripheral request so fixed-channel DMA remains
            // armed after its request selection bits have been discarded.
            dma.WriteDoubleWord(ChannelConfiguration(channel),
                channelConfiguration[channel] | ChannelEnable);
            remaining = dma.ReadDoubleWord(ChannelDataCount(channel)) & 0xFFFF;
            if(remaining != 0)
            {
                dma.OnGPIO(channel + 1, true);
            }
            remaining = dma.ReadDoubleWord(ChannelDataCount(channel)) & 0xFFFF;
            if(remaining == 0)
            {
                channelEnabled[channel] = false;
            }
        }

        public IReadOnlyDictionary<int, IGPIO> Connections => dma.Connections;
        public long Size => dma.Size;

        private static bool TryGetChannelConfiguration(long offset, out int channel)
        {
            var relative = offset - Channel1Configuration;
            channel = (int)(relative / ChannelRegisterStride);
            return relative >= 0 && relative % ChannelRegisterStride == 0;
        }

        private static long ChannelDataCount(int channel)
        {
            return Channel1DataCount + channel * ChannelRegisterStride;
        }

        private static long ChannelConfiguration(int channel)
        {
            return Channel1Configuration + channel * ChannelRegisterStride;
        }

        private readonly STM32G0DMA dma;
        private readonly bool[] channelEnabled;
        private readonly uint[] channelConfiguration;
        private readonly bool[] requestActive;
        private uint requestSelection;

        private const uint ChannelEnable = 1U;
        private const uint UnsupportedConfigurationBits =
            (1U << 3) | (3U << 12) | (0xFU << 16);
        private const uint MemoryToMemory = 1U << 14;
        private const uint MemoryToPeripheral = 1U << 4;
        private const uint TransferFlagMask = 0x77777777U;
        private const long InterruptFlagClear = 0x4;
        private const long RequestSelection = 0xA8;
        private const long Channel1Configuration = 0x8;
        private const long Channel1DataCount = 0xC;
        private const long ChannelRegisterStride = 0x14;
    }
}
