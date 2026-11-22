// Winbond W25Q flash adapter for Renode's STM32H7 QuadSPI controller.
//
// Dummy clocks belong to the controller transaction and do not transfer a
// byte to the flash.  GenericSpiFlash normally consumes a dummy byte itself,
// which duplicates that phase when it is attached to a controller model.
using System;

using Antmicro.Renode.Peripherals.Memory;
using Antmicro.Renode.Peripherals.SPI;
using Antmicro.Renode.Peripherals.SPI.NORFlash;

namespace Antmicro.Renode.Peripherals.Miscellaneous
{
    public class AP_W25QFlash : GenericSpiFlash
    {
        public AP_W25QFlash(MappedMemory underlyingMemory,
            byte manufacturerId, byte memoryType, byte? capacityCode = null)
            : base(underlyingMemory, manufacturerId, memoryType, capacityCode)
        {
            // ArduPilot supports JESD216 revisions 1.5 and 1.6.  Renode's
            // generated table uses its library revision (currently 1.12) and
            // stops after DWORD 11.  Complete the JESD216D fields consumed by
            // AP_FlashIface_JEDEC while retaining the generated density.
            var signature = new byte[ParameterTableOffset + ParameterDwords * 4];
            Array.Copy(SFDPSignature, signature, SFDPSignature.Length);
            signature[4] = 6;
            signature[11] = ParameterDwords;
            SetParameter(signature, 0, 1U << 21);              // 1-4-4 read
            SetParameter(signature, 2, 0xEBU << 8 | 2U << 5 | 4U);
            SetParameter(signature, 7, 0x20U << 8 | 12U);      // 4 KiB erase
            SetParameter(signature, 10, 8U << 4);              // 256-byte page
            SetParameter(signature, 13, 1U << 2);              // legacy status
            SetParameter(signature, 14,
                4U << 20 | 1U << 16 | 1U << 9);               // QE and XIP method 1
            SetParameter(signature, 15, 1U);                   // WREN 0x06
            SFDPSignature = signature;
        }

        protected override void RecognizeOperation(byte firstByte)
        {
            if(firstByte == ReadStatus2)
            {
                currentOperation.Operation = DecodedOperation.OperationType.ReadRegister;
                currentOperation.State = DecodedOperation.OperationState.HandleCommand;
                currentOperation.Register = (uint)Register.FirstNonstandardRegister;
                return;
            }
            base.RecognizeOperation(firstByte);
        }

        protected override byte ReadRegister(Register register)
        {
            return register == Register.FirstNonstandardRegister ?
                status2 : base.ReadRegister(register);
        }

        protected override void WriteRegister(Register register, byte data)
        {
            if(register == Register.Configuration)
            {
                status2 = data;
                return;
            }
            base.WriteRegister(register, data);
        }

        protected override int GetDummyBytes(Commands command)
        {
            // AP_FlashIface sends the 1-4-4 mode clocks as one alternate byte.
            // GenericSpiFlash has no separate alternate-byte phase, so consume
            // it here before returning flash contents.
            return command == Commands.QuadInputOutputFastRead ? 1 : 0;
        }

        protected override byte ReadFromMemory()
        {
            var address = currentOperation.ExecutionAddress +
                (uint)currentOperation.CommandBytesHandled;
            if(address == 0 && CurrentReadCount != 0)
            {
                LastReadHash = CurrentReadHash;
                LastReadCount = CurrentReadCount;
                CurrentReadHash = FnvOffsetBasis;
                CurrentReadCount = 0;
            }
            var value = base.ReadFromMemory();
            CurrentReadHash ^= value;
            CurrentReadHash *= FnvPrime;
            CurrentReadCount++;
            return value;
        }

        public ulong CurrentReadCount { get; private set; }
        public ulong CurrentReadHash { get; private set; } = FnvOffsetBasis;
        public ulong LastReadCount { get; private set; }
        public ulong LastReadHash { get; private set; }

        private static void SetParameter(byte[] signature, int index, uint value)
        {
            var offset = ParameterTableOffset + index * 4;
            signature[offset] = (byte)value;
            signature[offset + 1] = (byte)(value >> 8);
            signature[offset + 2] = (byte)(value >> 16);
            signature[offset + 3] = (byte)(value >> 24);
        }

        private const byte ParameterDwords = 20;
        private const byte ReadStatus2 = 0x35;
        private const int ParameterTableOffset = 0x18;
        private const ulong FnvOffsetBasis = 14695981039346656037UL;
        private const ulong FnvPrime = 1099511628211UL;
        private byte status2;
    }
}
