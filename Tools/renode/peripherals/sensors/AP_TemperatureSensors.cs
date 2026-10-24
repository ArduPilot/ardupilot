// Physics-driven I2C temperature-sensor models used by ArduPilot backends.
using System;
using Antmicro.Renode.Core;
using Antmicro.Renode.Peripherals.Miscellaneous;

namespace Antmicro.Renode.Peripherals.Sensors
{
    public abstract class AP_TemperatureSensorBase : AP_I2CRegisterDevice
    {
        protected AP_TemperatureSensorBase(IMachine machine)
        {
            physics = AP_PhysicsState.ForMachine(machine);
        }

        public override byte[] Read(int count = 1)
        {
            UpdateSample();
            return base.Read(count);
        }

        public bool SuppressData { get; set; }

        protected double TemperatureC
        {
            get { return physics.Current.TemperatureK - 273.15; }
        }

        protected abstract void UpdateSample();

        protected static short ScaleS16(double value, double degreesPerLsb)
        {
            return (short)Math.Max(Int16.MinValue, Math.Min(Int16.MaxValue,
                Math.Round(value / degreesPerLsb)));
        }

        protected static byte Crc8(byte initial, byte first, byte second)
        {
            var crc = initial;
            crc = Crc8Byte(crc, first);
            return Crc8Byte(crc, second);
        }

        private static byte Crc8Byte(byte crc, byte value)
        {
            crc ^= value;
            for(var bit = 0; bit < 8; bit++)
            {
                crc = (byte)((crc & 0x80) != 0 ? (crc << 1) ^ 0x31 : crc << 1);
            }
            return crc;
        }

        private readonly AP_PhysicsState physics;
    }

    public class AP_MCP9600 : AP_TemperatureSensorBase
    {
        public AP_MCP9600(IMachine machine) : base(machine)
        {
            Reset();
        }

        public override void Reset()
        {
            base.Reset();
            Registers[DeviceId] = WhoAmI;
            UpdateSample();
        }

        protected override void UpdateSample()
        {
            Registers[Status] = SuppressData ? (byte)0 : UpdateReady;
            WriteS16(Temperature, ScaleS16(TemperatureC, DegreesPerLsb));
        }

        private const int Temperature = 0x00;
        private const int Status = 0x04;
        private const int DeviceId = 0x20;
        private const byte UpdateReady = 0x40;
        private const byte WhoAmI = 0x40;
        private const double DegreesPerLsb = 0.0625;
    }

    public class AP_MLX90614 : AP_TemperatureSensorBase
    {
        public AP_MLX90614(IMachine machine) : base(machine)
        {
            Reset();
        }

        public override void Reset()
        {
            base.Reset();
            UpdateSample();
        }

        protected override void UpdateSample()
        {
            var raw = SuppressData ? (ushort)0 :
                (ushort)Math.Max(1, Math.Min(UInt16.MaxValue,
                    Math.Round((TemperatureC + 273.15) / KelvinPerLsb)));
            WriteU16LE(AmbientTemperature, raw);
            Registers[AmbientTemperature + 2] = 0;
        }

        private const int AmbientTemperature = 0x06;
        private const double KelvinPerLsb = 0.02;
    }

    public class AP_TMP119 : AP_TemperatureSensorBase
    {
        public AP_TMP119(IMachine machine) : base(machine)
        {
            Reset();
        }

        public override void Reset()
        {
            base.Reset();
            WriteU16(DeviceId, WhoAmI);
            UpdateSample();
        }

        protected override void UpdateSample()
        {
            if(!SuppressData)
            {
                WriteS16(Temperature, ScaleS16(TemperatureC, DegreesPerLsb));
            }
        }

        private const int Temperature = 0x00;
        private const int DeviceId = 0x0F;
        private const ushort WhoAmI = 0x2117;
        private const double DegreesPerLsb = 0.0078125;
    }

    public class AP_SHT3x : AP_TemperatureSensorBase
    {
        public AP_SHT3x(IMachine machine) : base(machine)
        {
            Reset();
        }

        public override void Reset()
        {
            base.Reset();
            serialResponse = false;
            UpdateSample();
        }

        public override void Write(byte[] data)
        {
            if(data.Length >= 2)
            {
                var command = (ushort)((data[0] << 8) | data[1]);
                if(command == ReadSerialNumber)
                {
                    serialResponse = true;
                }
                else if(command == StartMeasurement)
                {
                    serialResponse = false;
                }
            }
            base.Write(data);
        }

        public override byte[] Read(int count = 1)
        {
            UpdateSample();
            Pointer = 0;
            return base.Read(count);
        }

        protected override void UpdateSample()
        {
            if(serialResponse)
            {
                WriteWordWithCrc(0, 0x1234);
                WriteWordWithCrc(3, 0x5678);
                return;
            }

            var encoded = (ushort)Math.Max(0, Math.Min(UInt16.MaxValue,
                Math.Round((TemperatureC + 45.0) * UInt16.MaxValue / 175.0)));
            WriteWordWithCrc(0, encoded);
            WriteWordWithCrc(3, 0x8000);
            if(SuppressData)
            {
                Registers[2] ^= 0xFF;
            }
        }

        private void WriteWordWithCrc(int register, ushort value)
        {
            Registers[register] = (byte)(value >> 8);
            Registers[register + 1] = (byte)value;
            Registers[register + 2] = Crc8(
                0xFF, Registers[register], Registers[register + 1]);
        }

        private bool serialResponse;
        private const ushort ReadSerialNumber = 0x3780;
        private const ushort StartMeasurement = 0x2C06;
    }

    public class AP_TSYS01 : AP_TemperatureSensorBase
    {
        public AP_TSYS01(IMachine machine) : base(machine)
        {
            Reset();
        }

        public override void Reset()
        {
            base.Reset();
            WriteU16(CoefficientK1, K1);
            WriteU16(CoefficientK0, K0);
            UpdateSample();
        }

        protected override void UpdateSample()
        {
            var adc16 = (TemperatureC + OffsetC) / DegreesPerAdc16;
            var raw = SuppressData ? 0 : (int)Math.Round(adc16 * 256.0);
            raw = Math.Max(0, Math.Min(0xFFFFFF, raw));
            Registers[Adc] = (byte)(raw >> 16);
            Registers[Adc + 1] = (byte)(raw >> 8);
            Registers[Adc + 2] = (byte)raw;
        }

        private const int Adc = 0x00;
        private const int CoefficientK1 = 0xA8;
        private const int CoefficientK0 = 0xAA;
        private const ushort K1 = 10000;
        private const ushort K0 = 30000;
        private const double DegreesPerAdc16 = K1 * 1.0e-6;
        private const double OffsetC = K0 * 1.5e-2;
    }

    public class AP_TSYS03 : AP_TemperatureSensorBase
    {
        public AP_TSYS03(IMachine machine) : base(machine)
        {
            Reset();
        }

        public override void Reset()
        {
            base.Reset();
            UpdateSample();
        }

        protected override void UpdateSample()
        {
            var raw = (ushort)Math.Max(0, Math.Min(UInt16.MaxValue,
                Math.Round((TemperatureC + 40.0) * UInt16.MaxValue / 165.0)));
            Registers[Adc] = (byte)(raw >> 8);
            Registers[Adc + 1] = (byte)raw;
            Registers[Adc + 2] = Crc8(0, Registers[Adc], Registers[Adc + 1]);
            if(SuppressData)
            {
                Registers[Adc + 2] ^= 0xFF;
            }
        }

        private const int Adc = 0x00;
    }
}
