// Physics-driven MS4525 differential-pressure sensor. The DLVR backend uses
// the same four-byte pressure and temperature layout at address 0x28; writes
// trigger a conversion on MS4525 and are harmless for DLVR.
using System;
using Antmicro.Renode.Core;
using Antmicro.Renode.Peripherals.I2C;
using Antmicro.Renode.Peripherals.Miscellaneous;

namespace Antmicro.Renode.Peripherals.Sensors
{
    internal static class AP_AirspeedPhysics
    {
        public static double DifferentialPressurePa(AP_PhysicsState physics)
        {
            var truth = physics.Current;
            var density = truth.PressurePa > 0.0f && truth.TemperatureK > 0.0f
                ? truth.PressurePa / (SpecificGasConstant * truth.TemperatureK)
                : StandardDensity;
            return 0.5 * density * truth.AirspeedMS * truth.AirspeedMS;
        }

        public static double TemperatureC(AP_PhysicsState physics)
        {
            return physics.Current.TemperatureK - 273.15;
        }

        private const double SpecificGasConstant = 287.05;
        private const double StandardDensity = 1.225;
    }

    public class AP_Airspeed : II2CPeripheral
    {
        public AP_Airspeed(IMachine machine, double dlvrRangeInH2O = 0.0)
        {
            physics = AP_PhysicsState.ForMachine(machine);
            this.dlvrRangeInH2O = dlvrRangeInH2O;
            Reset();
        }

        public void Reset()
        {
            // Mid-scale pressure is zero differential pressure.  768 maps to
            // approximately 25 C in both firmware backends.
            pressure = 8192;
            temperature = 768;
            readIndex = 0;
            readSinceFinish = false;
        }

        public void Write(byte[] data)
        {
            WriteCount++;
            readIndex = 0;
        }

        public byte[] Read(int count = 1)
        {
            UpdateSample();
            var packed = ((uint)(pressure & 0x3FFF) << 16)
                | ((uint)(temperature & 0x7FF) << 5);
            var sample = new byte[] {
                (byte)(packed >> 24), (byte)(packed >> 16),
                (byte)(packed >> 8), (byte)packed,
            };
            var result = new byte[count];
            for(var i = 0; i < count; i++)
            {
                result[i] = (byte)(sample[(readIndex + i) % sample.Length] ^ ReadXorMask);
            }
            ReadCount += (ulong)count;
            readIndex = (readIndex + count) % sample.Length;
            readSinceFinish = true;
            return result;
        }

        public void FinishTransmission()
        {
            if(readSinceFinish)
            {
                readIndex = 0;
                readSinceFinish = false;
            }
        }

        public byte ReadXorMask { get; set; }
        public ulong ReadCount { get; private set; }
        public ulong WriteCount { get; private set; }

        private void UpdateSample()
        {
            var differentialPressurePa = AP_AirspeedPhysics.DifferentialPressurePa(physics);
            double rawPressure;
            if(dlvrRangeInH2O > 0.0)
            {
                var pressureInH2O = differentialPressurePa / InchH2OToPa;
                rawPressure = DlvrOffset + pressureInH2O * DlvrScale /
                    (2.5 * dlvrRangeInH2O);
            }
            else
            {
                rawPressure = PressureMidpoint - differentialPressurePa /
                    PsiToPa * PressureSpan / SensorRangePsi;
            }
            pressure = (ushort)Math.Max(1, Math.Min(0x3FFE, Math.Round(rawPressure)));
            var temperatureC = AP_AirspeedPhysics.TemperatureC(physics);
            var rawTemperature = (temperatureC + 50.0) * 2047.0 / 200.0;
            temperature = (ushort)Math.Max(1, Math.Min(0x7FE, Math.Round(rawTemperature)));
        }

        private readonly AP_PhysicsState physics;
        private readonly double dlvrRangeInH2O;
        private ushort pressure;
        private ushort temperature;
        private int readIndex;
        private bool readSinceFinish;

        private const double PsiToPa = 6894.757;
        private const double InchH2OToPa = 249.08891;
        private const double SensorRangePsi = 1.0;
        private const double PressureMidpoint = 8191.5;
        private const double DlvrOffset = 8192.0;
        private const double PressureSpan = 0.4 * 16383.0;
        private const double DlvrScale = 16384.0;
    }

    // AUAV differential pressure sensor at 0x26. Zeroed compensation
    // coefficients and a midpoint pressure sample produce 0 Pa at 25 C.
    public class AP_Airspeed_AUAV : II2CPeripheral
    {
        public AP_Airspeed_AUAV(IMachine machine)
        {
            physics = AP_PhysicsState.ForMachine(machine);
            Reset();
        }

        public void Reset()
        {
            command = 0xAE;
            readIndex = 0;
            readSinceFinish = false;
        }

        public void Write(byte[] data)
        {
            if(data.Length != 0)
            {
                WriteCount++;
                command = data[0];
                readIndex = 0;
                readSinceFinish = false;
            }
        }

        public byte[] Read(int count = 1)
        {
            UpdateSample();
            byte[] sample;
            var status = Absolute ? (byte)0x40 : (byte)0x50;
            if(command >= 0x2B && command <= 0x38)
            {
                sample = new byte[] { status, 0, 0 };
            }
            else if(Absolute)
            {
                sample = new byte[] {
                    status, 0xB5, 0xE9, 0xE2, 0x73, 0x9C, 0xE7,
                };
            }
            else
            {
                sample = new byte[] {
                    status,
                    (byte)(pressure >> 16), (byte)(pressure >> 8), (byte)pressure,
                    (byte)(temperature >> 16), (byte)(temperature >> 8), (byte)temperature,
                };
            }
            var result = new byte[count];
            for(var i = 0; i < count; i++)
            {
                result[i] = (byte)(sample[(readIndex + i) % sample.Length] ^ ReadXorMask);
            }
            ReadCount += (ulong)count;
            readIndex = (readIndex + count) % sample.Length;
            readSinceFinish = true;
            return result;
        }

        public void FinishTransmission()
        {
            if(readSinceFinish)
            {
                readIndex = 0;
                readSinceFinish = false;
            }
        }

        public byte ReadXorMask { get; set; }
        public ulong ReadCount { get; private set; }
        public ulong WriteCount { get; private set; }

        private void UpdateSample()
        {
            if(Absolute)
            {
                return;
            }
            var pressurePa = AP_AirspeedPhysics.DifferentialPressurePa(physics);
            var normalized = pressurePa / (PressureScalePa * RangeInH2O);
            pressure = (uint)Math.Max(1, Math.Min(0xFFFFFE,
                Math.Round(PressureMidpoint + normalized * PressureSpan)));
            var temperatureC = AP_AirspeedPhysics.TemperatureC(physics);
            temperature = (uint)Math.Max(1, Math.Min(0xFFFFFE,
                Math.Round((temperatureC + 45.0) * TemperatureSpan / 155.0)));
        }

        protected bool Absolute;
        private readonly AP_PhysicsState physics;
        private byte command;
        private int readIndex;
        private bool readSinceFinish;
        private uint pressure;
        private uint temperature;

        private const double PressureScalePa = 248.8 * 1.25 * 2.0;
        private const double RangeInH2O = 10.0;
        private const double PressureMidpoint = 8388608.0;
        private const double PressureSpan = 16777216.0;
        private const double TemperatureSpan = 16777216.0;
    }

    public class AP_Baro_AUAV : AP_Airspeed_AUAV
    {
        public AP_Baro_AUAV(IMachine machine) : base(machine)
        {
            Absolute = true;
        }
    }

    // QioTek ASP5033 register interface at 0x6C. It reports a ready,
    // stationary 0 Pa sample at 25 C and implements the mutable WHOAMI probe.
    public class AP_Airspeed_ASP5033 : AP_I2CRegisterDevice
    {
        public AP_Airspeed_ASP5033(IMachine machine)
        {
            physics = AP_PhysicsState.ForMachine(machine);
            Reset();
        }

        public override void Reset()
        {
            base.Reset();
            identity = 0;
        }

        public override byte[] Read(int count = 1)
        {
            UpdateSample();
            return base.Read(count);
        }

        protected override byte ReadRegister(int register)
        {
            if(register == Identity || register == IdentitySet)
            {
                return identity;
            }
            if(register == Command)
            {
                return Ready;
            }
            return base.ReadRegister(register);
        }

        protected override void WriteRegister(int register, byte value)
        {
            if(register == IdentitySet)
            {
                identity = value;
            }
            base.WriteRegister(register, value);
        }

        private void UpdateSample()
        {
            var pressure = (int)Math.Round(
                AP_AirspeedPhysics.DifferentialPressurePa(physics) * PressureScale);
            pressure = Math.Max(-0x800000, Math.Min(0x7FFFFF, pressure));
            Registers[Pressure] = (byte)(pressure >> 16);
            Registers[Pressure + 1] = (byte)(pressure >> 8);
            Registers[Pressure + 2] = (byte)pressure;
            var temperature = (short)Math.Round(
                AP_AirspeedPhysics.TemperatureC(physics) * TemperatureScale);
            Registers[Temperature] = (byte)(temperature >> 8);
            Registers[Temperature + 1] = (byte)temperature;
        }

        private readonly AP_PhysicsState physics;
        private byte identity;

        private const byte Identity = 0x01;
        private const byte Pressure = 0x06;
        private const byte Temperature = 0x09;
        private const byte Command = 0x30;
        private const byte IdentitySet = 0xA4;
        private const byte Ready = 0x08;
        private const double PressureScale = 128.0;
        private const double TemperatureScale = 256.0;
    }
}
