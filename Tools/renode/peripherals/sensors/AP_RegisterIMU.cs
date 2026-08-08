// Register-oriented SPI IMU used for Bosch BMI055/BMI088/BMI160/BMI270 and
// ST LSM6DSV devices. These parts use the same high-bit read convention for
// their normal register maps. Identity, reset, checked-register readback and
// ready/status behavior are modeled; an empty FIFO represents a stationary
// device until a richer sample source is connected.
//
using System;
using Antmicro.Renode.Core;
using Antmicro.Renode.Peripherals;
using Antmicro.Renode.Peripherals.I2C;
using Antmicro.Renode.Peripherals.SPI;

namespace Antmicro.Renode.Peripherals.Sensors
{
    public class AP_SCHA63T : ISPIPeripheral, IGPIOReceiver
    {
        public byte Transmit(byte value)
        {
            if(transferByte == 0)
            {
                register = (byte)((value >> 2) & 0x1F);
                response1 = register == SummaryStatus ? (byte)0x98 : (byte)0;
                response2 = register == SummaryStatus ? (byte)0x02 : (byte)0;
            }
            byte result;
            switch(transferByte)
            {
            case 1:
                result = response1;
                break;
            case 2:
                result = response2;
                break;
            case 3:
                result = Crc(0, response1, response2);
                break;
            default:
                result = 0;
                break;
            }
            transferByte++;
            return result;
        }

        public void Reset()
        {
            transferByte = 0;
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

        private static byte Crc(byte first, byte second, byte third)
        {
            byte crc = 0xFF;
            foreach(var value in new[] { first, second, third })
            {
                crc ^= value;
                for(var bit = 0; bit < 8; bit++)
                {
                    crc = (byte)((crc & 0x80) != 0 ? (crc << 1) ^ 0x1D : crc << 1);
                }
            }
            return (byte)(crc ^ 0xFF);
        }

        private int transferByte;
        private byte register;
        private byte response1;
        private byte response2;

        private const byte SummaryStatus = 0x0E;
    }

    public class AP_I2CRegisterIMU : II2CPeripheral
    {
        public AP_I2CRegisterIMU(byte whoAmI)
        {
            this.whoAmI = whoAmI;
            registers = new byte[256];
            Reset();
        }

        public void Reset()
        {
            Array.Clear(registers, 0, registers.Length);
            registers[0x00] = whoAmI;
            registers[0x0F] = whoAmI;
            registers[0x75] = whoAmI;
            registers[0x03] = 0xC0;
            registers[0x1E] = 0x07;
            pointer = 0;
        }

        public void Write(byte[] data)
        {
            if(data.Length == 0)
            {
                return;
            }
            pointer = data[0];
            for(var index = 1; index < data.Length; index++)
            {
                registers[pointer++] = data[index];
            }
        }

        public byte[] Read(int count = 1)
        {
            var result = new byte[count];
            for(var index = 0; index < count; index++)
            {
                result[index] = registers[pointer++];
            }
            return result;
        }

        public void FinishTransmission()
        {
        }

        private readonly byte whoAmI;
        private readonly byte[] registers;
        private int pointer;
    }

    public class AP_RegisterIMU : ISPIPeripheral, IGPIOReceiver
    {
        public AP_RegisterIMU(byte whoAmI)
        {
            this.whoAmI = whoAmI;
            registers = new byte[RegisterCount];
            Reset();
        }

        public void Reset()
        {
            Array.Clear(registers, 0, registers.Length);
            registers[WhoAmI] = whoAmI;
            registers[LsmWhoAmI] = whoAmI;
            registers[Bmi270InternalStatus] = Bmi270InitOk;
            transferByte = 0;
            currentRegister = 0;
            reading = false;
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
                var result = ReadRegister(currentRegister);
                currentRegister++;
                return result;
            }

            WriteRegister(currentRegister, value);
            currentRegister++;
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

        private byte ReadRegister(byte register)
        {
            switch(register)
            {
            case BmiStatus:
                return BmiAccelReady | BmiGyroReady;
            case LsmStatus:
                return LsmAccelReady | LsmGyroReady | LsmTemperatureReady;
            case Bmi270InternalStatus:
                return Bmi270InitOk;
            default:
                return registers[register];
            }
        }

        private void WriteRegister(byte register, byte value)
        {
            // Reset commands self-clear. Preserve the immutable identity and
            // make the BMI270 configuration-loader completion visible.
            if((register == BoschCommand && value == BoschSoftReset) ||
               (register == LsmControl3 && (value & LsmSoftwareReset) != 0))
            {
                Reset();
                return;
            }
            if(register == Bmi270InitControl && value != 0)
            {
                registers[Bmi270InternalStatus] = Bmi270InitOk;
            }
            registers[register] = value;
        }

        private readonly byte whoAmI;
        private readonly byte[] registers;
        private int transferByte;
        private byte currentRegister;
        private bool reading;

        private const int RegisterCount = 256;
        private const byte ReadFlag = 0x80;
        private const byte RegisterMask = 0x7F;
        private const byte WhoAmI = 0x00;
        private const byte LsmWhoAmI = 0x0F;
        private const byte BmiStatus = 0x03;
        private const byte Bmi270InternalStatus = 0x21;
        private const byte Bmi270InitControl = 0x59;
        private const byte LsmControl3 = 0x12;
        private const byte LsmStatus = 0x1E;
        private const byte BoschCommand = 0x7E;
        private const byte BoschSoftReset = 0xB6;
        private const byte LsmSoftwareReset = 0x01;
        private const byte BmiAccelReady = 0x80;
        private const byte BmiGyroReady = 0x40;
        private const byte LsmAccelReady = 0x01;
        private const byte LsmGyroReady = 0x02;
        private const byte LsmTemperatureReady = 0x04;
        private const byte Bmi270InitOk = 0x01;
    }
}
