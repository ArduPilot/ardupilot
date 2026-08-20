// The generic Synopsys DWC QoS model writes all four receive descriptor
// words back after a frame. STM32H7 hardware preserves RDES0, and ChibiOS
// relies on that behaviour: it initializes the buffer address once and only
// rewrites RDES3 when returning a descriptor to DMA ownership.
//
// Cache RDES0 when reception starts and restore it before a receive-tail write
// resumes the model. The model continues to provide all MAC, DMA and PHY
// behaviour; this helper only supplies the STM32-specific descriptor detail.
//
// The model also treats SARC=0 as reserved when a descriptor requests no
// source-address replacement. STM32 leaves it at zero, so select MAC0 replace
// in the model; MAC0 is the same address the firmware put in the frame.
using System.Reflection;
using Antmicro.Renode.Core;
using Antmicro.Renode.Core.Structure.Registers;
using Antmicro.Renode.Exceptions;
using Antmicro.Renode.Peripherals;
using Antmicro.Renode.Peripherals.Bus;
using Antmicro.Renode.Peripherals.Network;

namespace Antmicro.Renode.Peripherals.Miscellaneous
{
    public class AP_STM32H7_Ethernet : IDoubleWordPeripheral, IKnownSize
    {
        public AP_STM32H7_Ethernet(IMachine machine,
                                  SynopsysDWCEthernetQualityOfService ethernet)
        {
            this.machine = machine;
            this.ethernet = ethernet;

            var registersField = typeof(SynopsysDWCEthernetQualityOfService).GetField(
                "dmaRegisters", BindingFlags.NonPublic | BindingFlags.Instance);
            if(registersField == null)
            {
                throw new ConstructionException(
                    "Synopsys DWC Ethernet no longer has DMA registers - revisit this fix");
            }
            var registers = (DoubleWordRegisterCollection)registersField.GetValue(ethernet);
            registers.AddAfterWriteHook(ReceiveControl, CacheBufferAddresses);
            registers.AddBeforeWriteHook(ReceiveDescriptorTail, RestoreBufferAddresses);

            registersField = typeof(SynopsysDWCEthernetQualityOfService).GetField(
                "macAndMmcRegisters", BindingFlags.NonPublic | BindingFlags.Instance);
            if(registersField == null)
            {
                throw new ConstructionException(
                    "Synopsys DWC Ethernet no longer has MAC registers - revisit this fix");
            }
            registers = (DoubleWordRegisterCollection)registersField.GetValue(ethernet);
            registers.AddBeforeWriteHook(OperatingModeConfiguration, FixSourceAddressControl);
        }

        public long Size => 4;
        public uint ReadDoubleWord(long offset) => 0;
        public void WriteDoubleWord(long offset, uint value) { }
        public void Reset()
        {
            receiveDescriptorBase = 0;
            receiveBufferAddresses = null;
        }

        private void CacheBufferAddresses(long offset, uint value)
        {
            if((value & ReceiveStart) == 0)
            {
                return;
            }

            receiveDescriptorBase = ethernet.ReadDoubleWordFromDMA(ReceiveDescriptorList);
            var descriptorCount =
                (ethernet.ReadDoubleWordFromDMA(ReceiveDescriptorRingLength) & RingLengthMask) + 1;
            receiveBufferAddresses = new uint[descriptorCount];
            var bus = machine.GetSystemBus(ethernet);
            for(var index = 0; index < descriptorCount; index++)
            {
                receiveBufferAddresses[index] = bus.ReadDoubleWord(
                    receiveDescriptorBase + (uint)index * DescriptorSize);
            }
        }

        private uint? RestoreBufferAddresses(long offset, uint value)
        {
            if(receiveBufferAddresses == null)
            {
                return null;
            }

            var bus = machine.GetSystemBus(ethernet);
            for(var index = 0; index < receiveBufferAddresses.Length; index++)
            {
                bus.WriteDoubleWord(receiveDescriptorBase + (uint)index * DescriptorSize,
                                    receiveBufferAddresses[index]);
            }
            return null;
        }

        private uint? FixSourceAddressControl(long offset, uint value)
        {
            if((value & SourceAddressControlMask) == 0)
            {
                return value | SourceAddressMac0Replace;
            }
            return null;
        }

        private readonly IMachine machine;
        private readonly SynopsysDWCEthernetQualityOfService ethernet;
        private uint receiveDescriptorBase;
        private uint[] receiveBufferAddresses;

        private const long OperatingModeConfiguration = 0x0;
        private const long ReceiveControl = 0x108;
        private const long ReceiveDescriptorList = 0x11C;
        private const long ReceiveDescriptorTail = 0x128;
        private const long ReceiveDescriptorRingLength = 0x130;
        private const uint ReceiveStart = 1;
        private const uint RingLengthMask = 0x3FF;
        private const uint DescriptorSize = 16;
        private const uint SourceAddressControlMask = 0x7u << 28;
        private const uint SourceAddressMac0Replace = 0x3u << 28;
    }
}
