// BMP388 barometer as an I2C register-pointer device. The calibration and
// raw samples describe a stable bench environment near 101325 Pa and 25 C.
//
using System;
using Antmicro.Renode.Peripherals.I2C;

namespace Antmicro.Renode.Peripherals.Sensors
{
    public class AP_BMP388 : II2CPeripheral
    {
        public AP_BMP388()
        {
            registers = new byte[RegisterCount];
            Reset();
        }

        public void Reset()
        {
            Array.Clear(registers, 0, registers.Length);
            pointer = 0;
            registers[ChipId] = ChipIdValue;
            registers[Status] = PressureReady | TemperatureReady;

            // Temperature is (raw - T1) * T2. These values produce 25 C.
            WriteU16(CalibrationTemperature, 0);
            WriteU16(CalibrationTemperature + 2, 0xFFFF);
            registers[CalibrationTemperature + 4] = 0;

            // Pressure is raw * P1 with all higher order terms disabled.
            WriteS16(CalibrationPressure, 0x7FFF);
            WriteS16(CalibrationPressure + 2, 0x4000);
            WriteU24(Pressure, 6485000);
            WriteU24(Temperature, 409600);
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
                if(pointer == Command && data[index] == SoftReset)
                {
                    Reset();
                }
                else
                {
                    registers[pointer] = data[index];
                    pointer = (pointer + 1) & 0xFF;
                }
            }
        }

        public byte[] Read(int count = 1)
        {
            var result = new byte[count];
            for(var index = 0; index < count; index++)
            {
                result[index] = registers[pointer];
                pointer = (pointer + 1) & 0xFF;
            }
            return result;
        }

        public void FinishTransmission()
        {
        }

        private void WriteU16(int offset, ushort value)
        {
            registers[offset] = (byte)value;
            registers[offset + 1] = (byte)(value >> 8);
        }

        private void WriteS16(int offset, short value)
        {
            WriteU16(offset, (ushort)value);
        }

        private void WriteU24(int offset, uint value)
        {
            registers[offset] = (byte)value;
            registers[offset + 1] = (byte)(value >> 8);
            registers[offset + 2] = (byte)(value >> 16);
        }

        private readonly byte[] registers;
        private int pointer;

        private const int RegisterCount = 256;
        private const int ChipId = 0x00;
        private const int Status = 0x03;
        private const int Pressure = 0x04;
        private const int Temperature = 0x07;
        private const int CalibrationTemperature = 0x31;
        private const int CalibrationPressure = 0x36;
        private const int Command = 0x7E;
        private const byte ChipIdValue = 0x50;
        private const byte PressureReady = 0x20;
        private const byte TemperatureReady = 0x40;
        private const byte SoftReset = 0xB6;
    }
}
