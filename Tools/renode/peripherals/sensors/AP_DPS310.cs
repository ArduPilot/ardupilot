//
// DPS310 barometer as an I2C register-pointer device. This serves the
// AP_Baro_DPS310 probe, checked configuration registers, and periodic
// pressure/temperature reads. The deliberately simple calibration has
// zero raw inputs produce 100000 Pa and 25 C, a stable bench condition.
//
using System;
using Antmicro.Renode.Core;
using Antmicro.Renode.Peripherals.I2C;
using Antmicro.Renode.Peripherals.Miscellaneous;

namespace Antmicro.Renode.Peripherals.Sensors
{
    public class AP_DPS310 : II2CPeripheral
    {
        public AP_DPS310(IMachine machine)
        {
            physics = AP_PhysicsState.ForMachine(machine);
            Reset();
        }

        public void Reset()
        {
            pointer = 0;
            pressureSampleNumber = 0;
            registers = new byte[256];

            registers[ProductId] = 0x10;

            // C0=0, C1=1, C00=0 and C10=253952. With the driver's 16x
            // oversampling divisor this makes pressure equal raw pressure,
            // while raw temperature / 253952 gives degrees Celsius.
            registers[CoefficientBase + 0] = 0x00;
            registers[CoefficientBase + 1] = 0x00;
            registers[CoefficientBase + 2] = 0x01;
            registers[CoefficientBase + 3] = 0x00;
            registers[CoefficientBase + 4] = 0x00;
            registers[CoefficientBase + 5] = 0x03;
            registers[CoefficientBase + 6] = 0xE0;
            registers[CoefficientBase + 7] = 0x00;
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
            UpdateSample();
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

        private void UpdateSample()
        {
            var truth = physics.Current;
            pressureSampleNumber++;
            SetSigned24(Pressure, (int)Math.Round(Math.Max(
                -8388608.0, Math.Min(8388607.0,
                    AP_SensorNoise.Pressure(
                        truth, pressureSampleNumber, 0xD310U, 1.0f)))));
            var temperatureC = truth.TemperatureK - 273.15;
            SetSigned24(Temperature, (int)Math.Round(Math.Max(
                -8388608.0, Math.Min(8388607.0, temperatureC * 253952.0))));
        }

        private void SetSigned24(int register, int value)
        {
            registers[register] = (byte)(value >> 16);
            registers[register + 1] = (byte)(value >> 8);
            registers[register + 2] = (byte)value;
        }

        private readonly AP_PhysicsState physics;
        private byte[] registers;
        private int pointer;
        private uint pressureSampleNumber;

        private const int Pressure = 0x00;
        private const int Temperature = 0x03;
        private const int MeasurementConfig = 0x08;
        private const int ResetRegister = 0x0C;
        private const int ProductId = 0x0D;
        private const int CoefficientBase = 0x10;
        private const int TemperatureCoefficientSource = 0x28;
        private const byte PressureReady = 1 << 4;
        private const byte ResetCommand = 0x09;
    }
}
