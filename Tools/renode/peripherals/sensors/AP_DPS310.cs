//
// DPS310 barometer as an I2C register-pointer device. This serves the
// AP_Baro_DPS310 probe, checked configuration registers, and periodic
// pressure/temperature reads. The deliberately simple calibration has
// zero raw inputs produce 100000 Pa and 25 C, a stable bench condition.
//
using System;
using Antmicro.Renode.Peripherals.I2C;

namespace Antmicro.Renode.Peripherals.Sensors
{
    public class AP_DPS310 : II2CPeripheral
    {
        public AP_DPS310()
        {
            Reset();
        }

        public void Reset()
        {
            pointer = 0;
            registers = new byte[256];

            registers[ProductId] = 0x10;

            // C0=50, C1=0, C00=100000, all remaining coefficients 0.
            registers[CoefficientBase + 0] = 0x03;
            registers[CoefficientBase + 1] = 0x20;
            registers[CoefficientBase + 2] = 0x00;
            registers[CoefficientBase + 3] = 0x18;
            registers[CoefficientBase + 4] = 0x6A;
            registers[CoefficientBase + 5] = 0x00;
            registers[TemperatureCoefficientSource] = 0x00;

            // Raw pressure and temperature are both zero.
            for(var i = Pressure; i < Pressure + 6; i++)
            {
                registers[i] = 0;
            }
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
                if(pointer == ResetRegister && data[i] == ResetCommand)
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
                if(pointer == MeasurementConfig)
                {
                    result[i] |= PressureReady;
                }
                pointer = (pointer + 1) & 0xFF;
            }
            return result;
        }

        public void FinishTransmission()
        {
        }

        private byte[] registers;
        private int pointer;

        private const int Pressure = 0x00;
        private const int MeasurementConfig = 0x08;
        private const int ResetRegister = 0x0C;
        private const int ProductId = 0x0D;
        private const int CoefficientBase = 0x10;
        private const int TemperatureCoefficientSource = 0x28;
        private const byte PressureReady = 1 << 4;
        private const byte ResetCommand = 0x09;
    }
}
