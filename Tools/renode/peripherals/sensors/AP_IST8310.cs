// Stationary IST8310 compass model for the ArduPilot I2C backend.
using Antmicro.Renode.Peripherals.I2C;

namespace Antmicro.Renode.Peripherals.Sensors
{
    public class AP_IST8310 : II2CPeripheral
    {
        public AP_IST8310()
        {
            Reset();
        }

        public void Reset()
        {
            pointer = 0;
            registers = new byte[256];
            registers[WhoAmI] = DeviceId;

            // A fixed, valid magnetic field.  The driver converts each LSB to
            // 3 milligauss and flips Z before applying the board rotation.
            SetWord(OutputX, 67);
            SetWord(OutputY, 0);
            SetWord(OutputZ, -150);
        }

        public void Write(byte[] data)
        {
            if(data.Length == 0)
            {
                return;
            }

            pointer = data[0];
            for(var i = 1; i < data.Length; i++)
            {
                if(pointer == Control2 && (data[i] & SoftwareReset) != 0)
                {
                    Reset();
                }
                else
                {
                    registers[pointer] = data[i];
                    pointer = (pointer + 1) & 0xFF;
                }
            }
        }

        public byte[] Read(int count = 1)
        {
            var result = new byte[count];
            for(var i = 0; i < count; i++)
            {
                result[i] = registers[pointer];
                pointer = (pointer + 1) & 0xFF;
            }
            return result;
        }

        public void FinishTransmission()
        {
        }

        private void SetWord(int register, short value)
        {
            registers[register] = (byte)value;
            registers[register + 1] = (byte)(value >> 8);
        }

        private byte[] registers;
        private int pointer;

        private const int WhoAmI = 0x00;
        private const int OutputX = 0x03;
        private const int OutputY = 0x05;
        private const int OutputZ = 0x07;
        private const int Control2 = 0x0B;
        private const byte DeviceId = 0x10;
        private const byte SoftwareReset = 0x01;
    }
}
