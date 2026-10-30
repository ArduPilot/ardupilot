// SPI transport for the physics-driven register-pointer barometer models,
// selected by the identity the generator passes. It applies each backend's
// SPI addressing: BMP280 drops address bit 7 and BMP388 returns a dummy
// byte before read data.
//
using System;
using Antmicro.Renode.Core;
using Antmicro.Renode.Peripherals;
using Antmicro.Renode.Peripherals.SPI;

namespace Antmicro.Renode.Peripherals.Sensors
{
    public class AP_SPIBarometer : ISPIPeripheral, IGPIOReceiver
    {
        // chipId is implied by the model and kept for the generated platform
        public AP_SPIBarometer(IMachine machine, byte chipId, byte chipIdRegister)
        {
            this.chipIdRegister = (byte)(chipIdRegister & RegisterMask);
            switch(this.chipIdRegister)
            {
            case Bmp280ChipId:
                inner = new AP_BMP280(machine);
                break;
            case Bmp388ChipId:
                inner = new AP_BMP388(machine);
                break;
            case Bmp581ChipId:
                inner = new AP_BMP581(machine);
                break;
            case DpsChipId:
                inner = new AP_DPS310(machine);
                break;
            case LpsChipId:
                inner = new AP_LPS2XH(machine);
                break;
            default:
                throw new ArgumentException(string.Format(
                    "unsupported SPI barometer chip ID register 0x{0:X2}", chipIdRegister));
            }
            Reset();
        }

        public void Reset()
        {
            inner.Reset();
            transferByte = 0;
        }

        public byte Transmit(byte value)
        {
            if(transferByte == 0)
            {
                transferByte++;
                reading = (value & ReadFlag) != 0;
                register = (byte)(value & RegisterMask);
                if(chipIdRegister == Bmp280ChipId)
                {
                    register |= 0x80;
                }
                dataIndex = reading && chipIdRegister == Bmp388ChipId ? -1 : 0;
                return 0;
            }
            transferByte++;
            if(!reading)
            {
                inner.Write(new byte[] { register++, value });
                return 0;
            }
            if(dataIndex < 0)
            {
                dataIndex++;
                return 0;
            }
            if(dataIndex == 0)
            {
                // one burst reads one consistent sample
                inner.Write(new byte[] { register });
                block = inner.Read(BlockLength);
            }
            return dataIndex < block.Length ? block[dataIndex++] : (byte)0;
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

        private readonly AP_I2CRegisterDevice inner;
        private readonly byte chipIdRegister;
        private byte[] block = new byte[0];
        private int transferByte;
        private int dataIndex;
        private byte register;
        private bool reading;

        private const int BlockLength = 32;
        private const byte ReadFlag = 0x80;
        private const byte RegisterMask = 0x7F;
        private const byte Bmp388ChipId = 0x00;
        private const byte Bmp581ChipId = 0x01;
        private const byte DpsChipId = 0x0D;
        private const byte LpsChipId = 0x0F;
        private const byte Bmp280ChipId = 0x50;
    }
}
