//
// BMP280 barometer as an I2C register-pointer device. Serves what
// AP_Baro_BMP280 needs: ID, the 24-byte calibration block, read-back
// CTRL_MEAS/CONFIG (checked registers, re-read every 20 transfers) and
// the 6-byte pressure/temperature data block. Calibration constants
// and the 6-byte pressure/temperature data block. Raw readings are generated
// from the physics truth by inverting the integer compensation functions used
// by AP_Baro_BMP280.
//
using System;
using Antmicro.Renode.Core;
using Antmicro.Renode.Peripherals.Miscellaneous;

namespace Antmicro.Renode.Peripherals.Sensors
{
    public class AP_BMP280 : AP_I2CRegisterDevice
    {
        public AP_BMP280(IMachine machine)
        {
            physics = AP_PhysicsState.ForMachine(machine);
            Reset();
        }

        public override void Reset()
        {
            base.Reset();
            pressureSampleNumber = 0;

            Registers[ID] = 0x58;

            // dig_T1..dig_P9, little-endian, datasheet worked example
            WriteU16(0x88, 27504);
            WriteS16(0x8A, 26435);
            WriteS16(0x8C, -1000);
            WriteU16(0x8E, 36477);
            WriteS16(0x90, -10685);
            WriteS16(0x92, 3024);
            WriteS16(0x94, 2855);
            WriteS16(0x96, 140);
            WriteS16(0x98, -7);
            WriteS16(0x9A, 15500);
            WriteS16(0x9C, -14600);
            WriteS16(0x9E, 6000);

            UpdateSample();
        }

        protected override void WriteRegister(int register, byte value)
        {
            switch(register)
            {
            case RESET:
                if(value == 0xB6)
                {
                    Registers[CTRL_MEAS] = 0;
                    Registers[CONFIG] = 0;
                }
                return;
            case STATUS:
                return;
            default:
                base.WriteRegister(register, value);
                return;
            }
        }

        public override byte[] Read(int count = 1)
        {
            if(!FreezeSample)
            {
                UpdateSample();
            }
            return base.Read(count);
        }

        public bool FreezeSample { get; set; }

        private void UpdateSample()
        {
            var truth = physics.Current;
            pressureSampleNumber++;
            var temperatureRaw = FindTemperatureRaw((truth.TemperatureK - 273.15) * 100.0);
            var temperatureFine = CompensateTemperatureFine(temperatureRaw);
            var pressure = AP_SensorNoise.Pressure(
                truth, pressureSampleNumber, 0xB280U, 0.5f);
            var pressureRaw = FindPressureRaw(pressure, temperatureFine);
            Set20(Pressure, pressureRaw);
            Set20(Temperature, temperatureRaw);
        }

        private int FindTemperatureRaw(double targetHundredthsC)
        {
            var low = 0;
            var high = 0xFFFFF;
            while(low < high)
            {
                var middle = low + (high - low) / 2;
                var temperature = (CompensateTemperatureFine(middle) * 5 + 128) >> 8;
                if(temperature < targetHundredthsC)
                {
                    low = middle + 1;
                }
                else
                {
                    high = middle;
                }
            }
            return low;
        }

        private int CompensateTemperatureFine(int raw)
        {
            var var1 = ((((raw >> 3) - DigT1 * 2) * DigT2) >> 11);
            var difference = (raw >> 4) - DigT1;
            var var2 = (((difference * difference >> 12) * DigT3) >> 14);
            return var1 + var2;
        }

        private int FindPressureRaw(double targetPa, int temperatureFine)
        {
            var low = 0;
            var high = 0xFFFFF;
            while(low < high)
            {
                var middle = low + (high - low) / 2;
                // Compensated pressure falls as the raw ADC value rises.
                if(CompensatePressure(middle, temperatureFine) > targetPa)
                {
                    low = middle + 1;
                }
                else
                {
                    high = middle;
                }
            }
            return low;
        }

        private double CompensatePressure(int raw, int temperatureFine)
        {
            long var1 = temperatureFine - 128000;
            long var2 = var1 * var1 * DigP6;
            var2 += var1 * DigP5 << 17;
            var2 += (long)DigP4 << 35;
            var1 = ((var1 * var1 * DigP3) >> 8) + ((var1 * DigP2) << 12);
            var1 = (((1L << 47) + var1) * DigP1) >> 33;
            if(var1 == 0)
            {
                return 0.0;
            }
            long pressure = 1048576 - raw;
            pressure = (((pressure << 31) - var2) * 3125) / var1;
            var1 = DigP9 * (pressure >> 13) * (pressure >> 13) >> 25;
            var2 = DigP8 * pressure >> 19;
            pressure = ((pressure + var1 + var2) >> 8) + ((long)DigP7 << 4);
            return pressure / 256.0;
        }

        private void Set20(int offset, int value)
        {
            Registers[offset] = (byte)(value >> 12);
            Registers[offset + 1] = (byte)(value >> 4);
            Registers[offset + 2] = (byte)(value << 4);
        }

        private void WriteU16(int offset, ushort value)
        {
            Registers[offset] = (byte)(value & 0xFF);
            Registers[offset + 1] = (byte)(value >> 8);
        }

        private void WriteS16(int offset, short value)
        {
            WriteU16(offset, (ushort)value);
        }

        private readonly AP_PhysicsState physics;
        private uint pressureSampleNumber;

        private const int ID = 0xD0;
        private const int RESET = 0xE0;
        private const int STATUS = 0xF3;
        private const int CTRL_MEAS = 0xF4;
        private const int CONFIG = 0xF5;
        private const int Pressure = 0xF7;
        private const int Temperature = 0xFA;
        private const int DigT1 = 27504;
        private const int DigT2 = 26435;
        private const int DigT3 = -1000;
        private const int DigP1 = 36477;
        private const int DigP2 = -10685;
        private const int DigP3 = 3024;
        private const int DigP4 = 2855;
        private const int DigP5 = 140;
        private const int DigP6 = -7;
        private const int DigP7 = 15500;
        private const int DigP8 = -14600;
        private const int DigP9 = 6000;
    }
}
