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
            if(SuppressData)
            {
                packed |= 0xC0000000;
            }
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
        public bool SuppressData { get; set; }
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

    public class AP_Airspeed_DLVR : AP_Airspeed
    {
        public AP_Airspeed_DLVR(IMachine machine) : base(machine, RangeInH2O)
        {
        }

        private const double RangeInH2O = 5.0;
    }

    // MS5525DSO pressure sensor at 0x76. The PROM coefficients deliberately
    // remove the temperature cross-terms, allowing the model to invert the
    // production compensation equations without approximating the protocol.
    public class AP_Airspeed_MS5525 : II2CPeripheral
    {
        public AP_Airspeed_MS5525(IMachine machine)
        {
            physics = AP_PhysicsState.ForMachine(machine);
            Reset();
        }

        public void Reset()
        {
            command = ResetCommand;
            readIndex = 0;
            readSinceFinish = false;
            prom[0] = 1;
            prom[1] = SensitivityCoefficient;
            prom[2] = OffsetCoefficient;
            prom[3] = 0;
            prom[4] = 0;
            prom[5] = TemperatureReference;
            prom[6] = TemperatureCoefficient;
            prom[7] = 0;
            prom[7] = CalculateCrc4(prom);
        }

        public void Write(byte[] data)
        {
            if(data.Length == 0)
            {
                return;
            }
            WriteCount++;
            command = data[0];
            readIndex = 0;
            readSinceFinish = false;
        }

        public byte[] Read(int count = 1)
        {
            byte[] sample;
            if(command >= PromBase && command <= PromEnd && (command & 1) == 0)
            {
                var word = prom[(command - PromBase) / 2];
                sample = new byte[] { (byte)(word >> 8), (byte)word };
            }
            else
            {
                var value = SuppressData ? 0U : GetAdcValue();
                sample = new byte[] {
                    (byte)(value >> 16), (byte)(value >> 8), (byte)value,
                };
            }
            var result = new byte[count];
            for(var i = 0; i < count; i++)
            {
                result[i] = sample[(readIndex + i) % sample.Length];
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

        public bool SuppressData { get; set; }
        public ulong ReadCount { get; private set; }
        public ulong WriteCount { get; private set; }

        private uint GetAdcValue()
        {
            if(command == ConvertTemperature)
            {
                var temperatureC = AP_AirspeedPhysics.TemperatureC(physics);
                var delta = (temperatureC * 100.0 - 2000.0) *
                    (1 << 21) / TemperatureCoefficient;
                return ClampAdc(TemperatureReference * (1 << 7) + delta);
            }

            var pressurePa = AP_AirspeedPhysics.DifferentialPressurePa(physics);
            var compensatedPressure = pressurePa / PsiToPa / 1.0e-4;
            // With the selected coefficients, P = D1 / 64 - 40000.
            return ClampAdc((compensatedPressure + PressureAdcOffset) * 64.0);
        }

        private static uint ClampAdc(double value)
        {
            return (uint)Math.Max(1, Math.Min(0xFFFFFF, Math.Round(value)));
        }

        private static ushort CalculateCrc4(ushort[] words)
        {
            ushort remainder = 0;
            for(var byteIndex = 0; byteIndex < 16; byteIndex++)
            {
                remainder ^= (ushort)((byteIndex & 1) != 0
                    ? words[byteIndex >> 1] & 0xFF
                    : words[byteIndex >> 1] >> 8);
                for(var bit = 0; bit < 8; bit++)
                {
                    remainder = (ushort)((remainder & 0x8000) != 0
                        ? (remainder << 1) ^ 0x3000
                        : remainder << 1);
                }
            }
            return (ushort)((remainder >> 12) & 0xF);
        }

        private readonly AP_PhysicsState physics;
        private readonly ushort[] prom = new ushort[8];
        private byte command;
        private int readIndex;
        private bool readSinceFinish;

        private const byte ResetCommand = 0x1E;
        private const byte ConvertTemperature = 0x54;
        private const byte PromBase = 0xA0;
        private const byte PromEnd = 0xAE;
        private const ushort SensitivityCoefficient = 32768;
        private const ushort OffsetCoefficient = 10000;
        private const ushort TemperatureReference = 30000;
        private const ushort TemperatureCoefficient = 30000;
        private const double PressureAdcOffset = 40000.0;
        private const double PsiToPa = 6894.757;
    }

    // Sensirion SDP31 at 0x21, including its command sequence and CRC-protected
    // pressure, temperature and scale words. The encoded sensor pressure
    // inverts ArduPilot's pitot-tube correction so the shared physics value is
    // the pressure ultimately presented by the airspeed frontend.
    public class AP_Airspeed_SDP3X : II2CPeripheral
    {
        public AP_Airspeed_SDP3X(IMachine machine)
        {
            physics = AP_PhysicsState.ForMachine(machine);
            Reset();
        }

        public void Reset()
        {
            command = StopCommand;
            readIndex = 0;
            readSinceFinish = false;
        }

        public void Write(byte[] data)
        {
            if(data.Length == 0)
            {
                return;
            }
            WriteCount++;
            if(data.Length >= 2)
            {
                command = (ushort)((data[0] << 8) | data[1]);
            }
            readIndex = 0;
            readSinceFinish = false;
        }

        public byte[] Read(int count = 1)
        {
            var pressurePa = InvertPressureCorrection(
                AP_AirspeedPhysics.DifferentialPressurePa(physics));
            var pressure = ClampInt16(pressurePa * PressureScale);
            var temperature = ClampInt16(
                AP_AirspeedPhysics.TemperatureC(physics) * TemperatureScale);
            var sample = new byte[9];
            StoreWord(sample, 0, pressure);
            StoreWord(sample, 3, temperature);
            StoreWord(sample, 6, PressureScale);
            if(SuppressData || command != StartCommand)
            {
                sample[2] ^= 0xFF;
                sample[5] ^= 0xFF;
            }
            var sampleLength = count > 6 ? 9 : 6;
            var result = new byte[count];
            for(var i = 0; i < count; i++)
            {
                result[i] = sample[(readIndex + i) % sampleLength];
            }
            ReadCount += (ulong)count;
            readIndex = (readIndex + count) % sampleLength;
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

        public bool SuppressData { get; set; }
        public ulong ReadCount { get; private set; }
        public ulong WriteCount { get; private set; }

        private double InvertPressureCorrection(double target)
        {
            if(target <= 0.0)
            {
                return 0.0;
            }
            var low = 0.0;
            var high = Math.Max(target, 1.0);
            while(CorrectPressure(high) < target && high < 10000.0)
            {
                high *= 2.0;
            }
            for(var iteration = 0; iteration < 32; iteration++)
            {
                var middle = (low + high) * 0.5;
                if(CorrectPressure(middle) < target)
                {
                    low = middle;
                }
                else
                {
                    high = middle;
                }
            }
            return (low + high) * 0.5;
        }

        private double CorrectPressure(double pressure)
        {
            var truth = physics.Current;
            var density = truth.PressurePa /
                (SpecificGasConstant * truth.TemperatureK);
            if(density <= 0.0)
            {
                return pressure;
            }
            var flow = (300.805 - 300.878 /
                (0.00344205 * Math.Pow(pressure, 0.68698) + 1.0)) *
                1.29 / density;
            flow = Math.Max(0.0, flow);
            var pitotPressure = 28557670.0 * (1.0 - 1.0 /
                (1.0 + Math.Pow(flow / 5027611.0, 1.227924)));
            var uncorrected = (pressure + pitotPressure) / StandardDensity;
            var speedCorrection = 0.0331582 * flow;
            return Math.Pow(Math.Sqrt(uncorrected * AirspeedRatio) +
                speedCorrection, 2.0) / AirspeedRatio;
        }

        private static short ClampInt16(double value)
        {
            return (short)Math.Max(short.MinValue,
                Math.Min(short.MaxValue, Math.Round(value)));
        }

        private static void StoreWord(byte[] sample, int offset, int value)
        {
            sample[offset] = (byte)(value >> 8);
            sample[offset + 1] = (byte)value;
            sample[offset + 2] = CalculateCrc(sample[offset], sample[offset + 1]);
        }

        private static byte CalculateCrc(byte first, byte second)
        {
            var crc = 0xFF;
            foreach(var value in new byte[] { first, second })
            {
                crc ^= value;
                for(var bit = 0; bit < 8; bit++)
                {
                    crc = (crc & 0x80) != 0 ? (crc << 1) ^ 0x31 : crc << 1;
                    crc &= 0xFF;
                }
            }
            return (byte)crc;
        }

        private readonly AP_PhysicsState physics;
        private ushort command;
        private int readIndex;
        private bool readSinceFinish;

        private const int PressureScale = 60;
        private const ushort StartCommand = 0x3615;
        private const ushort StopCommand = 0x3FF9;
        private const double TemperatureScale = 200.0;
        private const double SpecificGasConstant = 287.26;
        private const double StandardDensity = 1.225;
        private const double AirspeedRatio = 2.0;
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
            pressureSampleNumber = 0;
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
                var pressurePa = AP_SensorNoise.Pressure(
                    physics.Current, pressureSampleNumber++, 0xA0A7U, 0.5f);
                var pressureCompensated = PressureCompensatedOffset +
                    (pressurePa / 100.0 - AbsolutePressureOffsetMbar) *
                    PressureSpan / AbsolutePressureSpanMbar;
                var normalized = pressureCompensated / PressureMaximum;
                pressure = (uint)Math.Max(1, Math.Min(0xFFFFFE,
                    Math.Round((normalized * 2.0 - 1.0) * PressureMidpoint +
                               PressureMidpoint + 1.0)));
            }
            else
            {
                var pressurePa = AP_AirspeedPhysics.DifferentialPressurePa(physics);
                var normalized = pressurePa / (PressureScalePa * RangeInH2O);
                pressure = (uint)Math.Max(1, Math.Min(0xFFFFFE,
                    Math.Round(PressureMidpoint + normalized * PressureSpan)));
            }
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
        private uint pressureSampleNumber;

        private const double PressureScalePa = 248.8 * 1.25 * 2.0;
        private const double RangeInH2O = 10.0;
        private const double PressureMidpoint = 8388608.0;
        private const double PressureSpan = 16777216.0;
        private const double TemperatureSpan = 16777216.0;
        private const double PressureMaximum = 16777215.0;
        private const double PressureCompensatedOffset = 1677722.0;
        private const double AbsolutePressureOffsetMbar = 250.0;
        private const double AbsolutePressureSpanMbar = 1250.0;
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
