// Register-pointer models for barometers used by the wider ArduPilot hwdef
// set. Each exposes a stable bench sample and the identity/status behavior
// required by its production backend.
//
using System;
using Antmicro.Renode.Core;
using Antmicro.Renode.Peripherals;
using Antmicro.Renode.Peripherals.I2C;
using Antmicro.Renode.Peripherals.SPI;

namespace Antmicro.Renode.Peripherals.Sensors
{
    public class AP_SPIBarometer : ISPIPeripheral, IGPIOReceiver
    {
        public AP_SPIBarometer(byte chipId, byte chipIdRegister)
        {
            this.chipId = chipId;
            this.chipIdRegister = (byte)(chipIdRegister & RegisterMask);
            registers = new byte[256];
            Reset();
        }

        public void Reset()
        {
            Array.Clear(registers, 0, registers.Length);
            registers[chipIdRegister] = chipId;
            registers[0x03] = 0x60;
            registers[0x08] = 0x10;
            registers[0x11] = 0x02;
            registers[0x27] = 0x13;
            registers[0x28] = 0x60;
            ConfigureSample();
            transferByte = 0;
        }

        public byte Transmit(byte value)
        {
            if(transferByte++ == 0)
            {
                reading = (value & ReadFlag) != 0;
                currentRegister = (byte)(value & RegisterMask);
                return 0;
            }
            if(reading)
            {
                return registers[currentRegister++];
            }
            registers[currentRegister++] = value;
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

        private void ConfigureSample()
        {
            if(chipIdRegister == Bmp280ChipId)
            {
                // BMP280 datasheet calibration and raw sample, with the SPI
                // read flag removed from the logical 0x80..0xFF addresses.
                WriteU16LE(0x08, 27504);
                WriteS16LE(0x0A, 26435);
                WriteS16LE(0x0C, -1000);
                WriteU16LE(0x0E, 36477);
                WriteS16LE(0x10, -10685);
                WriteS16LE(0x12, 3024);
                WriteS16LE(0x14, 2855);
                WriteS16LE(0x16, 140);
                WriteS16LE(0x18, -7);
                WriteS16LE(0x1A, 15500);
                WriteS16LE(0x1C, -14600);
                WriteS16LE(0x1E, 6000);
                registers[0x77] = 0x65;
                registers[0x78] = 0x5C;
                registers[0x79] = 0xC0;
                registers[0x7A] = 0x7E;
                registers[0x7B] = 0xED;
                return;
            }
            if(chipIdRegister == Bmp388ChipId)
            {
                WriteU16LE(0x31, 0);
                WriteU16LE(0x33, 0xFFFF);
                WriteS16LE(0x36, 0x7FFF);
                WriteS16LE(0x38, 0x4000);
                WriteU24LE(0x04, 6485000);
                WriteU24LE(0x07, 409600);
                return;
            }
            if(chipIdRegister == Bmp581ChipId)
            {
                WriteU24LE(0x1D, 25U << 16);
                WriteU24LE(0x20, 101325U << 6);
                return;
            }
            if(chipIdRegister == DpsChipId)
            {
                // C0=50, C1=0, C00=100000; zero raw inputs describe a
                // stable 100 kPa, 25 C sample.
                registers[0x10] = 0x03;
                registers[0x11] = 0x20;
                registers[0x12] = 0x00;
                registers[0x13] = 0x18;
                registers[0x14] = 0x6A;
                registers[0x15] = 0x00;
                registers[0x28] = 0x00;
                return;
            }
            if(chipIdRegister == LpsChipId)
            {
                WriteU24LE(0x28, 101325U * 4096U / 100U);
                var temperature = (short)(25 * 100);
                registers[0x2B] = (byte)temperature;
                registers[0x2C] = (byte)(temperature >> 8);
            }
        }

        private void WriteU16LE(int register, ushort value)
        {
            registers[register] = (byte)value;
            registers[register + 1] = (byte)(value >> 8);
        }

        private void WriteS16LE(int register, short value)
        {
            WriteU16LE(register, (ushort)value);
        }

        private void WriteU24LE(int register, uint value)
        {
            registers[register] = (byte)value;
            registers[register + 1] = (byte)(value >> 8);
            registers[register + 2] = (byte)(value >> 16);
        }

        private readonly byte chipId;
        private readonly byte chipIdRegister;
        private readonly byte[] registers;
        private int transferByte;
        private byte currentRegister;
        private bool reading;

        private const byte ReadFlag = 0x80;
        private const byte RegisterMask = 0x7F;
        private const byte Bmp388ChipId = 0x00;
        private const byte Bmp581ChipId = 0x01;
        private const byte DpsChipId = 0x0D;
        private const byte LpsChipId = 0x0F;
        private const byte Bmp280ChipId = 0x50;
    }

    public abstract class AP_I2CRegisterDevice : II2CPeripheral
    {
        protected AP_I2CRegisterDevice()
        {
            Registers = new byte[256];
        }

        public virtual void Reset()
        {
            Array.Clear(Registers, 0, Registers.Length);
            Pointer = 0;
        }

        public virtual void Write(byte[] data)
        {
            if(data.Length == 0)
            {
                return;
            }
            Pointer = data[0];
            for(var index = 1; index < data.Length; index++)
            {
                WriteRegister(Pointer, data[index]);
                Pointer = (Pointer + 1) & 0xFF;
            }
        }

        public virtual byte[] Read(int count = 1)
        {
            var result = new byte[count];
            for(var index = 0; index < count; index++)
            {
                result[index] = ReadRegister(Pointer);
                Pointer = (Pointer + 1) & 0xFF;
            }
            return result;
        }

        public void FinishTransmission()
        {
        }

        protected virtual byte ReadRegister(int register)
        {
            return Registers[register];
        }

        protected virtual void WriteRegister(int register, byte value)
        {
            Registers[register] = value;
        }

        protected void WriteU16(int register, ushort value)
        {
            Registers[register] = (byte)(value >> 8);
            Registers[register + 1] = (byte)value;
        }

        protected void WriteS16(int register, short value)
        {
            WriteU16(register, (ushort)value);
        }

        protected void WriteU24LE(int register, uint value)
        {
            Registers[register] = (byte)value;
            Registers[register + 1] = (byte)(value >> 8);
            Registers[register + 2] = (byte)(value >> 16);
        }

        protected byte[] Registers;
        protected int Pointer;
    }

    public class AP_BMP085 : AP_I2CRegisterDevice
    {
        public AP_BMP085()
        {
            Reset();
        }

        public override void Reset()
        {
            base.Reset();
            Registers[0xD0] = 0x55;
            // BMP085 datasheet example calibration, big-endian.
            WriteS16(0xAA, 408);
            WriteS16(0xAC, -72);
            WriteS16(0xAE, -14383);
            WriteU16(0xB0, 32741);
            WriteU16(0xB2, 32757);
            WriteU16(0xB4, 23153);
            WriteS16(0xB6, 6190);
            WriteS16(0xB8, 4);
            WriteS16(0xBA, -32768);
            WriteS16(0xBC, -8711);
            WriteS16(0xBE, 2868);
            SetTemperatureSample();
        }

        protected override void WriteRegister(int register, byte value)
        {
            base.WriteRegister(register, value);
            if(register != 0xF4)
            {
                return;
            }
            if(value == 0x2E)
            {
                SetTemperatureSample();
            }
            else
            {
                // UP=23843 from the datasheet example, left-aligned for OSS=3.
                var raw = 23843U << 5;
                Registers[0xF6] = (byte)(raw >> 16);
                Registers[0xF7] = (byte)(raw >> 8);
                Registers[0xF8] = (byte)raw;
            }
        }

        private void SetTemperatureSample()
        {
            WriteU16(0xF6, 27898);
        }
    }

    public class AP_BMP581 : AP_I2CRegisterDevice
    {
        public AP_BMP581()
        {
            Reset();
        }

        public override void Reset()
        {
            base.Reset();
            Registers[0x01] = 0x50;
            Registers[0x11] = 0x02;
            Registers[0x27] = 0x10;
            Registers[0x28] = 0x60;
            WriteU24LE(0x1D, 25U << 16);
            WriteU24LE(0x20, 101325U << 6);
        }
    }

    public class AP_LPS2XH : AP_I2CRegisterDevice
    {
        public AP_LPS2XH()
        {
            Reset();
        }

        public override void Reset()
        {
            base.Reset();
            Registers[0x0F] = 0xB1;
            Registers[0x27] = 0x03;
            WriteU24LE(0x28, 101325U * 4096U / 100U);
            var temperature = (short)(25 * 100);
            Registers[0x2B] = (byte)temperature;
            Registers[0x2C] = (byte)(temperature >> 8);
        }
    }

    public class AP_ICP201XX : AP_I2CRegisterDevice
    {
        public AP_ICP201XX()
        {
            Reset();
        }

        public override void Reset()
        {
            base.Reset();
            Registers[0x0C] = 0x63;
            Registers[0xD3] = 0xB2;
            Registers[0xBF] = 0x01;
            Registers[0xCD] = 0x01;
        }

        protected override byte ReadRegister(int register)
        {
            if(register == 0xC4)
            {
                return 1;
            }
            // One pressure/temperature FIFO packet. Zero raw values mean
            // 70 kPa and 25 C in the backend's transfer function.
            if(register >= 0xFA)
            {
                return 0;
            }
            return base.ReadRegister(register);
        }
    }
}
