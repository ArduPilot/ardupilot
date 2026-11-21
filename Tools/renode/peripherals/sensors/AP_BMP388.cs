// BMP388 barometer as an I2C register-pointer device. Raw samples are generated
// from physics truth by inverting the deliberately simple calibration below.
//
using System;
using Antmicro.Renode.Core;
using Antmicro.Renode.Peripherals.Miscellaneous;

namespace Antmicro.Renode.Peripherals.Sensors
{
    public class AP_BMP388 : AP_I2CRegisterDevice
    {
        public AP_BMP388(IMachine machine)
        {
            physics = AP_PhysicsState.ForMachine(machine);
            Reset();
        }

        public override void Reset()
        {
            base.Reset();
            Registers[ChipId] = ChipIdValue;
            Registers[Status] = PressureReady | TemperatureReady;
            pressureSampleNumber = 0;

            // Temperature is (raw - T1) * T2. These values produce 25 C.
            WriteU16LE(CalibrationTemperature, 0);
            WriteU16LE(CalibrationTemperature + 2, 0xFFFF);
            Registers[CalibrationTemperature + 4] = 0;

            // Pressure is raw * P1 with all higher order terms disabled.
            WriteS16LE(CalibrationPressure, 0x7FFF);
            WriteS16LE(CalibrationPressure + 2, 0x4000);
            UpdateSample();
        }

        public override byte[] Read(int count = 1)
        {
            if(Pointer == Status && !SuppressReady)
            {
                UpdateSample();
            }
            return base.Read(count);
        }

        public bool SuppressReady { get; set; }

        protected override byte ReadRegister(int register)
        {
            if(register == Status && SuppressReady)
            {
                return 0;
            }
            return base.ReadRegister(register);
        }

        protected override void WriteRegister(int register, byte value)
        {
            if(register == Command && value == SoftReset)
            {
                Reset();
                return;
            }
            base.WriteRegister(register, value);
        }

        private void UpdateSample()
        {
            var truth = physics.Current;
            pressureSampleNumber++;
            var pressurePa = AP_SensorNoise.Pressure(
                truth, pressureSampleNumber, 0xB388U, 0.5f);
            var temperatureC = truth.TemperatureK - 273.15;
            WriteU24LE(Pressure, LimitU24(pressurePa / PressureScale));
            WriteU24LE(Temperature, LimitU24(temperatureC / TemperatureScale));
        }

        private uint LimitU24(double value)
        {
            return (uint)Math.Max(0, Math.Min(0xFFFFFF, Math.Round(value)));
        }

        private readonly AP_PhysicsState physics;
        private uint pressureSampleNumber;

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
        private const double TemperatureScale = 65535.0 / 1073741824.0;
        private const double PressureScale = 16383.0 / 1048576.0;
    }
}
