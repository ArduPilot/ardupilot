//
// DPS310 barometer as an I2C register-pointer device. This serves the
// AP_Baro_DPS310 probe, checked configuration registers, and periodic
// pressure/temperature reads. The deliberately simple calibration has
// zero raw inputs produce 100000 Pa and 25 C, a stable bench condition.
//
using System;
using Antmicro.Renode.Core;
using Antmicro.Renode.Peripherals.Miscellaneous;

namespace Antmicro.Renode.Peripherals.Sensors
{
    public class AP_DPS310 : AP_I2CRegisterDevice
    {
        public AP_DPS310(IMachine machine)
        {
            physics = AP_PhysicsState.ForMachine(machine);
            Reset();
        }

        public override void Reset()
        {
            base.Reset();
            pressureSampleNumber = 0;

            Registers[ProductId] = 0x10;

            // C0=0, C1=256, C00=0 and C10=253952. The raw generators below
            // account for the distinct DPS280 and SPL06 oversampling scales.
            Registers[CoefficientBase + 0] = 0x00;
            Registers[CoefficientBase + 1] = 0x01;
            Registers[CoefficientBase + 2] = 0x00;
            Registers[CoefficientBase + 3] = 0x00;
            Registers[CoefficientBase + 4] = 0x00;
            Registers[CoefficientBase + 5] = 0x03;
            Registers[CoefficientBase + 6] = 0xE0;
            Registers[CoefficientBase + 7] = 0x00;
            Registers[TemperatureCoefficientSource] = 0x00;

            // Raw pressure and temperature are both zero.
            for(var i = Pressure; i < Pressure + 6; i++)
            {
                Registers[i] = 0;
            }
        }

        protected override void WriteRegister(int register, byte value)
        {
            if(register == ResetRegister && value == ResetCommand)
            {
                Reset();
                return;
            }
            if(register == PressureConfig)
            {
                spl06Mode = (value & OversamplingMask) == Spl06Oversampling;
            }
            base.WriteRegister(register, value);
        }

        protected override byte ReadRegister(int register)
        {
            if(SuppressData && register >= Pressure && register < Temperature + 3)
            {
                return 0;
            }
            var value = base.ReadRegister(register);
            if(register == MeasurementConfig && !SuppressReady)
            {
                value |= AllReady;
            }
            return value;
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
        public bool SuppressData { get; set; }
        public bool SuppressReady { get; set; }

        private void UpdateSample()
        {
            var truth = physics.Current;
            pressureSampleNumber++;
            var scale = spl06Mode ? Spl06Scale : Dps280Scale;
            SetSigned24(Pressure, (int)Math.Round(Math.Max(
                -8388608.0, Math.Min(8388607.0,
                    AP_SensorNoise.Pressure(
                        truth, pressureSampleNumber, 0xD310U, 1.0f) *
                    scale / PressureCoefficient))));
            var temperatureC = truth.TemperatureK - 273.15;
            SetSigned24(Temperature, (int)Math.Round(Math.Max(
                -8388608.0, Math.Min(8388607.0,
                    temperatureC * scale / TemperatureCoefficient))));
        }

        private void SetSigned24(int register, int value)
        {
            Registers[register] = (byte)(value >> 16);
            Registers[register + 1] = (byte)(value >> 8);
            Registers[register + 2] = (byte)value;
        }

        private readonly AP_PhysicsState physics;
        private uint pressureSampleNumber;
        private bool spl06Mode;

        private const int Pressure = 0x00;
        private const int Temperature = 0x03;
        private const int MeasurementConfig = 0x08;
        private const int PressureConfig = 0x06;
        private const int ResetRegister = 0x0C;
        private const int ProductId = 0x0D;
        private const int CoefficientBase = 0x10;
        private const int TemperatureCoefficientSource = 0x28;
        private const byte AllReady = 0xF0;
        private const byte ResetCommand = 0x09;
        private const byte OversamplingMask = 0x0F;
        private const byte Spl06Oversampling = 3;
        private const double Dps280Scale = 253952.0;
        private const double Spl06Scale = 7864320.0;
        private const double PressureCoefficient = 253952.0;
        private const double TemperatureCoefficient = 256.0;
    }
}
