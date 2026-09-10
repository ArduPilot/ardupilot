// Minimal register transports for the two ADIS protocol variants used by
// ArduPilot. They model product identity, writable configuration registers
// and clear diagnostic state. Sample/burst reads return stationary data.
//
using System;
using Antmicro.Renode.Core;
using Antmicro.Renode.Peripherals;
using Antmicro.Renode.Peripherals.SPI;

namespace Antmicro.Renode.Peripherals.Sensors
{
    public class AP_ADIS1647x : ISPIPeripheral, IGPIOReceiver
    {
        public AP_ADIS1647x(ushort productId = ProductId16470)
        {
            registers = new byte[RegisterCount];
            this.productId = productId;
            Reset();
        }

        public void Reset()
        {
            Array.Clear(registers, 0, registers.Length);
            WriteWord(ProductIdRegister, productId);
            transferByte = 0;
            pendingRead = false;
        }

        public byte Transmit(byte value)
        {
            if(pendingRead)
            {
                var result = transferByte == 0
                    ? registers[(pendingRegister + 1) & RegisterMask]
                    : registers[pendingRegister];
                transferByte++;
                return result;
            }
            if(transferByte++ == 0)
            {
                command = value;
            }
            else if((command & WriteFlag) != 0)
            {
                registers[command & RegisterMask] = value;
            }
            return 0;
        }

        public void FinishTransmission()
        {
            if(pendingRead)
            {
                pendingRead = false;
            }
            else if((command & WriteFlag) == 0)
            {
                pendingRegister = (byte)(command & RegisterMask);
                pendingRead = true;
            }
            transferByte = 0;
        }

        public void OnGPIO(int number, bool value)
        {
            if(value)
            {
                FinishTransmission();
            }
        }

        private void WriteWord(byte register, ushort value)
        {
            registers[register] = (byte)value;
            registers[register + 1] = (byte)(value >> 8);
        }

        private readonly byte[] registers;
        private readonly ushort productId;
        private int transferByte;
        private byte command;
        private byte pendingRegister;
        private bool pendingRead;

        private const int RegisterCount = 128;
        private const byte WriteFlag = 0x80;
        private const byte RegisterMask = 0x7F;
        private const byte ProductIdRegister = 0x72;
        private const ushort ProductId16470 = 0x4056;
    }

    public class AP_ADIS16607 : ISPIPeripheral, IGPIOReceiver
    {
        public AP_ADIS16607()
        {
            registers = new ushort[RegisterCount];
            Reset();
        }

        public void Reset()
        {
            Array.Clear(registers, 0, registers.Length);
            registers[DeviceId] = DeviceIdValue;
            registers[WriteLock] = 1;
            transferByte = 0;
        }

        public byte Transmit(byte value)
        {
            if(transferByte == 0)
            {
                command = value;
                transferByte++;
                return 0;
            }
            if((command & ReadFlag) != 0)
            {
                var word = registers[command & RegisterMask];
                var result = transferByte < 2 ? (byte)0 :
                    (transferByte == 2 ? (byte)(word >> 8) : (byte)word);
                transferByte++;
                return result;
            }
            if(transferByte == 1)
            {
                writeHigh = value;
            }
            else if(transferByte == 2)
            {
                registers[command & RegisterMask] = (ushort)((writeHigh << 8) | value);
            }
            transferByte++;
            return 0;
        }

        public void FinishTransmission()
        {
            transferByte = 0;
        }

        public void OnGPIO(int number, bool value)
        {
            if(value)
            {
                FinishTransmission();
            }
        }

        private readonly ushort[] registers;
        private int transferByte;
        private byte command;
        private byte writeHigh;

        private const int RegisterCount = 128;
        private const byte ReadFlag = 0x80;
        private const byte RegisterMask = 0x7F;
        private const byte DeviceId = 0x00;
        private const byte WriteLock = 0x7F;
        private const ushort DeviceIdValue = 0x6000;
    }
}
