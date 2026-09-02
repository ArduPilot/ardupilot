// Physics-driven IST8310 compass model for the ArduPilot I2C backend.
using System;
using Antmicro.Renode.Core;
using Antmicro.Renode.Peripherals.Miscellaneous;

namespace Antmicro.Renode.Peripherals.Sensors
{
    public class AP_IST8310 : AP_I2CRegisterDevice
    {
        public AP_IST8310(IMachine machine, byte rotation = 12)
        {
            physics = AP_PhysicsState.ForMachine(machine);
            this.rotation = rotation;
            Reset();
        }

        public override void Reset()
        {
            base.Reset();
            Registers[WhoAmI] = DeviceId;

            // A fixed, valid magnetic field.  The driver converts each LSB to
            // 3 milligauss and flips Z before applying the board rotation.
            SetWord(OutputX, 67);
            SetWord(OutputY, 0);
            SetWord(OutputZ, -150);
        }

        public override byte[] Read(int count = 1)
        {
            UpdateSample();
            return base.Read(count);
        }

        protected override void WriteRegister(int register, byte value)
        {
            if(register == Control2 && (value & SoftwareReset) != 0)
            {
                Reset();
                return;
            }
            base.WriteRegister(register, value);
        }

        private void SetWord(int register, short value)
        {
            WriteS16LE(register, value);
        }

        private void UpdateSample()
        {
            var field = AP_SensorOrientation.BodyToSensor(
                physics.Current.MagneticFieldBodyMgauss, rotation);
            SetWord(OutputX, ScaleField(field[0]));
            SetWord(OutputY, ScaleField(field[1]));
            SetWord(OutputZ, ScaleField(-field[2]));
        }

        private static short ScaleField(float value)
        {
            return (short)Math.Max(Int16.MinValue,
                Math.Min(Int16.MaxValue, Math.Round(value / MilligaussPerLsb)));
        }

        private readonly AP_PhysicsState physics;
        private readonly byte rotation;
        private const int WhoAmI = 0x00;
        private const int OutputX = 0x03;
        private const int OutputY = 0x05;
        private const int OutputZ = 0x07;
        private const int Control2 = 0x0B;
        private const byte DeviceId = 0x10;
        private const byte SoftwareReset = 0x01;
        private const double MilligaussPerLsb = 3.0;
    }
}
