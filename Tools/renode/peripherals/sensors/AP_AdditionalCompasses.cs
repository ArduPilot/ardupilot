// Physics-driven I2C compass models used by ArduPilot production backends.
using System;
using Antmicro.Renode.Core;
using Antmicro.Renode.Peripherals.Miscellaneous;

namespace Antmicro.Renode.Peripherals.Sensors
{
    public abstract class AP_CompassModelBase : AP_I2CRegisterDevice
    {
        protected AP_CompassModelBase(IMachine machine)
        {
            physics = AP_PhysicsState.ForMachine(machine);
        }

        public override byte[] Read(int count = 1)
        {
            UpdateSample();
            return base.Read(count);
        }

        public bool SuppressData { get; set; }

        public byte Rotation { get; set; }

        protected float[] Field
        {
            get
            {
                return AP_SensorOrientation.BodyToSensor(
                    physics.Current.MagneticFieldBodyMgauss, Rotation);
            }
        }

        protected abstract void UpdateSample();

        protected static short ScaleS16(float value, double milligaussPerLsb)
        {
            return (short)Math.Max(Int16.MinValue, Math.Min(Int16.MaxValue,
                Math.Round(value / milligaussPerLsb)));
        }

        protected void WriteS24(int register, int value)
        {
            value = Math.Max(-0x800000, Math.Min(0x7FFFFF, value));
            Registers[register] = (byte)(value >> 16);
            Registers[register + 1] = (byte)(value >> 8);
            Registers[register + 2] = (byte)value;
        }

        private readonly AP_PhysicsState physics;
    }

    public class AP_MAG3110 : AP_CompassModelBase
    {
        public AP_MAG3110(IMachine machine) : base(machine)
        {
            Reset();
        }

        public override void Reset()
        {
            base.Reset();
            Registers[WhoAmI] = DeviceId;
            UpdateSample();
        }

        protected override byte ReadRegister(int register)
        {
            if(register == Status)
            {
                return SuppressData ? (byte)0 : Ready;
            }
            return base.ReadRegister(register);
        }

        protected override void UpdateSample()
        {
            var x = ScaleS16(Field[0], MilligaussPerLsb);
            var y = ScaleS16(Field[1], MilligaussPerLsb);
            var z = ScaleS16(Field[2], MilligaussPerLsb);
            Registers[Output] = (byte)(x >> 8);
            Registers[Output + 1] = (byte)x;
            Registers[Output + 2] = (byte)(y >> 8);
            Registers[Output + 3] = (byte)y;
            Registers[Output + 4] = (byte)(z >> 8);
            Registers[Output + 5] = (byte)z;
        }

        private const int Status = 0x00;
        private const int Output = 0x01;
        private const int WhoAmI = 0x07;
        private const byte Ready = 1 << 3;
        private const byte DeviceId = 0xC4;
        // The MAG3110 data sheet specifies 0.1 microtesla (1 milligauss) per
        // LSB, but the ArduPilot backend uses 1000 milligauss per LSB. Match
        // the production backend here so Renode exercises its existing scale.
        private const double MilligaussPerLsb = 1000.0;
    }

    public class AP_IIS2MDC : AP_CompassModelBase
    {
        public AP_IIS2MDC(IMachine machine) : base(machine)
        {
            Reset();
        }

        public override void Reset()
        {
            base.Reset();
            Registers[WhoAmI] = DeviceId;
            Registers[Status] = Ready;
            UpdateSample();
        }

        protected override byte ReadRegister(int register)
        {
            if(register == Status && SuppressData)
            {
                return 0;
            }
            return base.ReadRegister(register);
        }

        protected override void UpdateSample()
        {
            WriteS16LE(Output, ScaleS16(Field[0], MilligaussPerLsb));
            WriteS16LE(Output + 2, ScaleS16(Field[1], MilligaussPerLsb));
            WriteS16LE(Output + 4, ScaleS16(-Field[2], MilligaussPerLsb));
            WriteS16LE(Output + 6, 0);
        }

        private const int WhoAmI = 0x4F;
        private const int Status = 0x67;
        private const int Output = 0x68;
        private const byte DeviceId = 0x40;
        private const byte Ready = 0x0F;
        private const double MilligaussPerLsb = 100.0 / 65.535;
    }

    public class AP_LIS3MDL : AP_CompassModelBase
    {
        public AP_LIS3MDL(IMachine machine) : base(machine)
        {
            Reset();
        }

        public override void Reset()
        {
            base.Reset();
            Registers[WhoAmI] = DeviceId;
            Registers[Status] = Ready;
            UpdateSample();
        }

        protected override byte ReadRegister(int register)
        {
            if(register == Status && SuppressData)
            {
                return 0;
            }
            return base.ReadRegister(register);
        }

        protected override void UpdateSample()
        {
            // ArduPilot's external-I2C auto probe applies ROTATION_YAW_90.
            WriteS16LE(Output, ScaleS16(Field[1], MilligaussPerLsb));
            WriteS16LE(Output + 2, ScaleS16(-Field[0], MilligaussPerLsb));
            WriteS16LE(Output + 4, ScaleS16(Field[2], MilligaussPerLsb));
        }

        private const int WhoAmI = 0x0F;
        private const int Status = 0x27;
        private const int Output = 0x28;
        private const byte DeviceId = 0x3D;
        private const byte Ready = 0x08;
        private const double MilligaussPerLsb = 1000.0 / 6842.0;
    }

    public class AP_IST8308 : AP_CompassModelBase
    {
        public AP_IST8308(IMachine machine) : base(machine)
        {
            Reset();
        }

        public override void Reset()
        {
            base.Reset();
            Registers[WhoAmI] = DeviceId;
            Registers[Status] = Ready;
            UpdateSample();
        }

        protected override byte ReadRegister(int register)
        {
            if(register == Status && SuppressData)
            {
                return 0;
            }
            return base.ReadRegister(register);
        }

        protected override void WriteRegister(int register, byte value)
        {
            if(register == Control3 && (value & SoftwareReset) != 0)
            {
                Reset();
                return;
            }
            base.WriteRegister(register, value);
        }

        protected override void UpdateSample()
        {
            WriteS16LE(Output, ScaleS16(Field[0], MilligaussPerLsb));
            WriteS16LE(Output + 2, ScaleS16(Field[1], MilligaussPerLsb));
            WriteS16LE(Output + 4, ScaleS16(-Field[2], MilligaussPerLsb));
        }

        private const int WhoAmI = 0x00;
        private const int Status = 0x10;
        private const int Output = 0x11;
        private const int Control3 = 0x32;
        private const byte DeviceId = 0x08;
        private const byte Ready = 0x01;
        private const byte SoftwareReset = 0x01;
        private const double MilligaussPerLsb = 1.515;
    }

    public class AP_QMC5883L : AP_CompassModelBase
    {
        public AP_QMC5883L(IMachine machine) : base(machine)
        {
            Reset();
        }

        public override void Reset()
        {
            base.Reset();
            Registers[SecondaryId] = SecondaryDeviceId;
            Registers[Id] = DeviceId;
            Registers[Status] = Ready;
            UpdateSample();
        }

        protected override byte ReadRegister(int register)
        {
            if(register == Status && SuppressData)
            {
                return 0;
            }
            return base.ReadRegister(register);
        }

        protected override void UpdateSample()
        {
            // Undo the driver's axis signs, external YAW_90 adjustment and
            // default external ROTATION_ROLL_180 orientation.
            WriteS16LE(Output, ScaleS16(Field[1], MilligaussPerLsb));
            WriteS16LE(Output + 2, ScaleS16(-Field[0], MilligaussPerLsb));
            WriteS16LE(Output + 4, ScaleS16(Field[2], MilligaussPerLsb));
        }

        private const int Output = 0x00;
        private const int Status = 0x06;
        private const int SecondaryId = 0x0C;
        private const int Id = 0x0D;
        private const byte SecondaryDeviceId = 0x01;
        private const byte DeviceId = 0xFF;
        private const byte Ready = 0x04;
        private const double MilligaussPerLsb = 1000.0 / 3000.0;
    }

    public class AP_QMC5883P : AP_CompassModelBase
    {
        public AP_QMC5883P(IMachine machine) : base(machine)
        {
            Reset();
        }

        public override void Reset()
        {
            base.Reset();
            Registers[Id] = DeviceId;
            Registers[Status] = Ready;
            UpdateSample();
        }

        protected override byte ReadRegister(int register)
        {
            if(register == Status && SuppressData)
            {
                return 0;
            }
            return base.ReadRegister(register);
        }

        protected override void UpdateSample()
        {
            // AP_Compass_QMC5883P applies ROTATION_ROLL_180 to external
            // sensors. Present the inverse transform in the sensor frame.
            WriteS16LE(Output, ScaleS16(Field[0], MilligaussPerLsb));
            WriteS16LE(Output + 2, ScaleS16(-Field[1], MilligaussPerLsb));
            WriteS16LE(Output + 4, ScaleS16(-Field[2], MilligaussPerLsb));
        }

        private const int Id = 0x00;
        private const int Output = 0x01;
        private const int Status = 0x09;
        private const byte DeviceId = 0x80;
        private const byte Ready = 0x01;
        private const double MilligaussPerLsb = 1000.0 / 3000.0;
    }

    public class AP_RM3100 : AP_CompassModelBase
    {
        public AP_RM3100(IMachine machine) : base(machine)
        {
            Reset();
        }

        public override void Reset()
        {
            base.Reset();
            for(var register = CycleX; register <= CycleZ + 1; register += 2)
            {
                Registers[register] = 0x00;
                Registers[register + 1] = 0xC8;
            }
            Registers[Status] = Ready;
            UpdateSample();
        }

        protected override byte ReadRegister(int register)
        {
            if(register == Status && SuppressData)
            {
                return 0;
            }
            return base.ReadRegister(register);
        }

        protected override void UpdateSample()
        {
            WriteS24(Output, ScaleField(Field[0]));
            WriteS24(Output + 3, ScaleField(Field[1]));
            WriteS24(Output + 6, ScaleField(Field[2]));
        }

        private static int ScaleField(float value)
        {
            return (int)Math.Round(value / MilligaussPerLsb);
        }

        private const int CycleX = 0x04;
        private const int CycleZ = 0x08;
        private const int Output = 0x24;
        private const int Status = 0x34;
        private const byte Ready = 0x80;
        private const double MilligaussPerLsb = 10.0 / 75.0;
    }

    public class AP_AK09916 : AP_CompassModelBase
    {
        public AP_AK09916(IMachine machine) : base(machine)
        {
            Reset();
        }

        public override void Reset()
        {
            base.Reset();
            Registers[CompanyId] = 0x48;
            Registers[DeviceId] = 0x09;
            Registers[Status1] = Ready;
            UpdateSample();
        }

        protected override byte ReadRegister(int register)
        {
            if(register == Status1 && SuppressData)
            {
                return 0;
            }
            return base.ReadRegister(register);
        }

        protected override void WriteRegister(int register, byte value)
        {
            if(register == Control3 && (value & SoftwareReset) != 0)
            {
                Reset();
                return;
            }
            base.WriteRegister(register, value);
        }

        protected override void UpdateSample()
        {
            var x = Field[0];
            var y = Field[1];
            if(CompensateExternalProbeRotation)
            {
                // External auto-probing applies ROTATION_YAW_270.
                x = -Field[1];
                y = Field[0];
            }
            WriteS16LE(Output, ScaleS16(x, MilligaussPerLsb));
            WriteS16LE(Output + 2, ScaleS16(y, MilligaussPerLsb));
            WriteS16LE(Output + 4, ScaleS16(Field[2], MilligaussPerLsb));
            Registers[Output + 6] = 0;
            Registers[Output + 7] = 0;
        }

        public bool CompensateExternalProbeRotation { get; set; } = true;

        private const int CompanyId = 0x00;
        private const int DeviceId = 0x01;
        private const int Status1 = 0x10;
        private const int Output = 0x11;
        private const int Control3 = 0x32;
        private const byte Ready = 0x01;
        private const byte SoftwareReset = 0x01;
        private const double MilligaussPerLsb = 1.5;
    }

    public class AP_AK8963 : AP_CompassModelBase
    {
        public AP_AK8963(IMachine machine) : base(machine)
        {
            Reset();
        }

        public override void Reset()
        {
            base.Reset();
            Registers[WhoAmI] = DeviceId;
            Registers[Adjustment] = NeutralAdjustment;
            Registers[Adjustment + 1] = NeutralAdjustment;
            Registers[Adjustment + 2] = NeutralAdjustment;
            UpdateSample();
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

        protected override void UpdateSample()
        {
            if(SuppressData)
            {
                for(var index = 0; index < 6; index++)
                {
                    Registers[Output + index] = 0;
                }
                return;
            }
            WriteS16LE(Output, ScaleS16(Field[0], MilligaussPerLsb));
            WriteS16LE(Output + 2, ScaleS16(Field[1], MilligaussPerLsb));
            WriteS16LE(Output + 4, ScaleS16(Field[2], MilligaussPerLsb));
            Registers[Output + 6] = 0;
        }

        private const int WhoAmI = 0x00;
        private const int Output = 0x03;
        private const int Control2 = 0x0B;
        private const int Adjustment = 0x10;
        private const byte DeviceId = 0x48;
        private const byte NeutralAdjustment = 128;
        private const byte SoftwareReset = 0x01;
        private const double MilligaussPerLsb = 1.5;
    }

    public class AP_HMC5843 : AP_CompassModelBase
    {
        public AP_HMC5843(IMachine machine) : base(machine)
        {
            Reset();
        }

        public override void Reset()
        {
            base.Reset();
            Registers[Id] = (byte)'H';
            Registers[Id + 1] = (byte)'4';
            Registers[Id + 2] = (byte)'3';
            UpdateSample();
        }

        protected override void WriteRegister(int register, byte value)
        {
            base.WriteRegister(register, value);
            if(register == ConfigA || register == Mode)
            {
                UpdateSample();
            }
        }

        protected override void UpdateSample()
        {
            if(SuppressData)
            {
                WriteS16(Output, InvalidSample);
                WriteS16(Output + 2, InvalidSample);
                WriteS16(Output + 4, InvalidSample);
                return;
            }
            if((Registers[ConfigA] & BiasMask) != NormalBias)
            {
                // Values chosen so ArduPilot's startup calibration produces
                // unit scaling on all three axes.
                WriteS16(Output, -696);
                WriteS16(Output + 2, -696);
                WriteS16(Output + 4, 648);
                return;
            }

            // The production backend reads X/Z/Y, applies sensor signs,
            // external YAW_90 and the auto-probe ROLL_180 orientation.
            WriteS16(Output, ScaleS16(Field[1], MilligaussPerLsb));
            WriteS16(Output + 2, ScaleS16(Field[2], MilligaussPerLsb));
            WriteS16(Output + 4, ScaleS16(-Field[0], MilligaussPerLsb));
        }

        private const int ConfigA = 0x00;
        private const int Mode = 0x02;
        private const int Output = 0x03;
        private const int Id = 0x0A;
        private const int BiasMask = 0x03;
        private const int NormalBias = 0;
        private const short InvalidSample = -4096;
        private const double MilligaussPerLsb = 1000.0 / 1090.0;
    }

    public class AP_MMC3416 : AP_CompassModelBase
    {
        public AP_MMC3416(IMachine machine) : base(machine)
        {
            Reset();
        }

        public override void Reset()
        {
            base.Reset();
            Registers[ProductId] = DeviceId;
            polarity = 1;
            UpdateSample();
        }

        protected override byte ReadRegister(int register)
        {
            if(register == Status && SuppressData)
            {
                return 0;
            }
            return base.ReadRegister(register);
        }

        protected override void WriteRegister(int register, byte value)
        {
            if(register == Control1 && (value & SoftwareReset) != 0)
            {
                Reset();
                return;
            }
            base.WriteRegister(register, value);
            if(register != Control0)
            {
                return;
            }
            if((value & Set) != 0)
            {
                polarity = 1;
            }
            else if((value & ResetBridge) != 0)
            {
                polarity = -1;
            }
            if((value & TakeMeasurement) != 0)
            {
                UpdateSample();
                Registers[Status] = Ready;
            }
        }

        protected override void UpdateSample()
        {
            WriteAxis(Output, Field[0]);
            WriteAxis(Output + 2, -Field[1]);
            WriteAxis(Output + 4, Field[2]);
        }

        private void WriteAxis(int register, float value)
        {
            var counts = (int)Math.Round(value / MilligaussPerLsb);
            counts = Math.Max(-32767, Math.Min(32767, counts));
            WriteU16LE(register, (ushort)(ZeroOffset + polarity * counts));
        }

        private int polarity;
        private const int Output = 0x00;
        private const int Status = 0x06;
        private const int Control0 = 0x07;
        private const int Control1 = 0x08;
        private const int ProductId = 0x20;
        private const int ZeroOffset = 32768;
        private const byte DeviceId = 0x06;
        private const byte Ready = 0x01;
        private const byte Set = 0x20;
        private const byte ResetBridge = 0x40;
        private const byte TakeMeasurement = 0x01;
        private const byte SoftwareReset = 0x80;
        private const double MilligaussPerLsb = 1000.0 / 2048.0;
    }

    public class AP_BMM150 : AP_CompassModelBase
    {
        public AP_BMM150(IMachine machine) : base(machine)
        {
            Reset();
        }

        public override void Reset()
        {
            base.Reset();
            Registers[ChipId] = DeviceId;
            Registers[TrimZ2] = 0x00;
            Registers[TrimZ2 + 1] = 0x40;
            Registers[TrimXyz1] = 0x00;
            Registers[TrimXyz1 + 1] = 0x20;
            UpdateSample();
        }

        protected override void WriteRegister(int register, byte value)
        {
            if(register == Power && (value & SoftReset) == SoftReset)
            {
                Reset();
                return;
            }
            base.WriteRegister(register, value);
        }

        protected override void UpdateSample()
        {
            WriteS16LE(Output, ScaleRaw(Field[0], 3, XyCompensation));
            WriteS16LE(Output + 2, ScaleRaw(Field[1], 3, XyCompensation));
            WriteS16LE(Output + 4, ScaleRaw(Field[2], 1, ZCompensation));
            WriteU16LE(Output + 6,
                (ushort)((HallResistance << 2) | (SuppressData ? 0 : Ready)));
        }

        private static short ScaleRaw(float value, int shift, double compensation)
        {
            var raw = Math.Round(value * DriverCountsPerMilligauss / compensation);
            var shifted = raw * (1 << shift);
            return (short)Math.Max(Int16.MinValue, Math.Min(Int16.MaxValue, shifted));
        }

        private const int ChipId = 0x40;
        private const int Output = 0x42;
        private const int Power = 0x4B;
        private const int TrimZ2 = 0x68;
        private const int TrimXyz1 = 0x6C;
        private const byte DeviceId = 0x32;
        private const byte SoftReset = 0x82;
        private const ushort HallResistance = 8192;
        private const ushort Ready = 1;
        private const double DriverCountsPerMilligauss = 16.0 / 10.0;
        private const double XyCompensation = 5.0;
        private const double ZCompensation = 2.0;
    }

    public class AP_BMM350 : AP_CompassModelBase
    {
        public AP_BMM350(IMachine machine) : base(machine)
        {
            Reset();
        }

        public override void Reset()
        {
            base.Reset();
            // BMM350 I2C reads contain two dummy bytes.
            Registers[ChipId + DummyBytes] = DeviceId;
            Registers[OtpStatus + DummyBytes] = OtpDone;
            SetPmuCommand(SuspendMode);
            UpdateSample();
        }

        protected override void WriteRegister(int register, byte value)
        {
            if(register == Command && value == SoftReset)
            {
                Reset();
                return;
            }
            base.WriteRegister(register, value);
            if(register == PmuCommand)
            {
                SetPmuCommand(value);
            }
        }

        protected override void UpdateSample()
        {
            var x = SuppressData ? 0 : ScaleRaw(Field[0], XyScale);
            var y = SuppressData ? 0 : ScaleRaw(Field[1] / YSensitivity, XyScale);
            var z = SuppressData ? 0 : ScaleRaw(Field[2], ZScale);
            WriteS24LE(Data, x);
            WriteS24LE(Data + 3, y);
            WriteS24LE(Data + 6, z);
            WriteS24LE(Data + 9, TemperatureRaw);
        }

        private void SetPmuCommand(byte command)
        {
            Registers[PmuCommand + DummyBytes] = command;
            Registers[PmuStatus + DummyBytes] = (byte)(command << 5);
            if(command == NormalMode)
            {
                Registers[PmuStatus + DummyBytes] |= NormalModeStatus;
            }
        }

        private void WriteS24LE(int register, int value)
        {
            value = Math.Max(-0x800000, Math.Min(0x7FFFFF, value));
            WriteU24LE(register, (uint)value & 0xFFFFFF);
        }

        private static int ScaleRaw(float milligauss, double scale)
        {
            return (int)Math.Round((milligauss / 10.0) / scale);
        }

        private const int ChipId = 0x00;
        private const int PmuCommand = 0x06;
        private const int PmuStatus = 0x07;
        private const int Data = 0x31 + DummyBytes;
        private const int OtpStatus = 0x55;
        private const int Command = 0x7E;
        private const int DummyBytes = 2;
        private const byte DeviceId = 0x33;
        private const byte OtpDone = 0x01;
        private const byte SuspendMode = 0x00;
        private const byte NormalMode = 0x01;
        private const byte NormalModeStatus = 0x08;
        private const byte SoftReset = 0xB6;
        private const float YSensitivity = 1.01f;
        private const double Power = 1000000.0 / 1048576.0;
        private const double AdcGain = 1.0 / 1.5;
        private const double LutGain = 0.714607238769531;
        private const double XyScale = Power / (14.55 * 19.46 * AdcGain * LutGain);
        private const double ZScale = Power / (9.0 * 31.0 * AdcGain * LutGain);
        private const double TemperatureScale = 1.0 / (0.00204 * AdcGain * LutGain * 1048576.0);
        private static readonly int TemperatureRaw =
            (int)Math.Round((23.0 + 25.49) / TemperatureScale);
    }

    public abstract class AP_LSMCompassBase : AP_CompassModelBase
    {
        protected AP_LSMCompassBase(IMachine machine) : base(machine)
        {
        }

        public override void Write(byte[] data)
        {
            if(data.Length == 0)
            {
                return;
            }
            var transaction = (byte[])data.Clone();
            transaction[0] &= RegisterMask;
            base.Write(transaction);
        }

        private const byte RegisterMask = 0x3F;
    }

    public class AP_LSM303DCompass : AP_LSMCompassBase
    {
        public AP_LSM303DCompass(IMachine machine) : base(machine)
        {
            Reset();
        }

        public override void Reset()
        {
            base.Reset();
            Registers[WhoAmI] = DeviceId;
            UpdateSample();
        }

        protected override void UpdateSample()
        {
            Registers[Status] = 0;
            if(SuppressData)
            {
                for(var index = 0; index < 6; index++)
                {
                    Registers[Output + index] = 0;
                }
                return;
            }
            WriteS16LE(Output, ScaleS16(Field[0], MilligaussPerLsb));
            WriteS16LE(Output + 2, ScaleS16(Field[1], MilligaussPerLsb));
            WriteS16LE(Output + 4, ScaleS16(Field[2], MilligaussPerLsb));
        }

        private const int WhoAmI = 0x0F;
        private const int Status = 0x07;
        private const int Output = 0x08;
        private const byte DeviceId = 0x49;
        private const double MilligaussPerLsb = 0.080;
    }

    public class AP_LSM9DS1Compass : AP_LSMCompassBase
    {
        public AP_LSM9DS1Compass(IMachine machine) : base(machine)
        {
            Reset();
        }

        public override void Reset()
        {
            base.Reset();
            Registers[WhoAmI] = DeviceId;
            UpdateSample();
        }

        protected override void UpdateSample()
        {
            Registers[Status] = SuppressData ? Invalid : (byte)0;
            WriteS16LE(Output, ScaleS16(Field[0], MilligaussPerLsb));
            WriteS16LE(Output + 2, ScaleS16(Field[1], MilligaussPerLsb));
            WriteS16LE(Output + 4, ScaleS16(Field[2], MilligaussPerLsb));
        }

        private const int WhoAmI = 0x0F;
        private const int Status = 0x27;
        private const int Output = 0x28;
        private const byte DeviceId = 0x3D;
        private const byte Invalid = 0x80;
        private const double MilligaussPerLsb = 0.58;
    }

    public class AP_MMC5983 : AP_CompassModelBase
    {
        public AP_MMC5983(IMachine machine) : base(machine)
        {
            Reset();
        }

        public override void Reset()
        {
            base.Reset();
            Registers[ProductId] = DeviceId;
            polarity = 1;
            UpdateSample();
        }

        protected override byte ReadRegister(int register)
        {
            if(register == Status && SuppressData)
            {
                return 0;
            }
            return base.ReadRegister(register);
        }

        protected override void WriteRegister(int register, byte value)
        {
            if(register == Control1 && (value & SoftwareReset) != 0)
            {
                Reset();
                return;
            }
            base.WriteRegister(register, value);
            if(register != Control0)
            {
                return;
            }
            if((value & Set) != 0)
            {
                polarity = -1;
            }
            else if((value & ResetBridge) != 0)
            {
                polarity = 1;
            }
            if((value & TakeMeasurement) != 0)
            {
                UpdateSample();
                Registers[Status] = Ready;
            }
        }

        protected override void UpdateSample()
        {
            WriteAxis(Output, Field[0]);
            WriteAxis(Output + 2, Field[1]);
            WriteAxis(Output + 4, Field[2]);
        }

        private void WriteAxis(int register, float value)
        {
            var counts = (int)Math.Round(value / MilligaussPerLsb);
            counts = Math.Max(-32767, Math.Min(32767, counts));
            WriteU16(register, (ushort)(ZeroOffset + polarity * counts));
        }

        private int polarity;
        private const int Output = 0x00;
        private const int Status = 0x08;
        private const int Control0 = 0x09;
        private const int Control1 = 0x0A;
        private const int ProductId = 0x2F;
        private const int ZeroOffset = 32768;
        private const byte DeviceId = 0x30;
        private const byte Ready = 0x01;
        private const byte Set = 0x08;
        private const byte ResetBridge = 0x10;
        private const byte TakeMeasurement = 0x01;
        private const byte SoftwareReset = 0x80;
        private const double MilligaussPerLsb = 1000.0 / 4096.0;
    }
}
