// Physics-driven I2C power monitors used by ArduPilot battery backends.
using System;
using Antmicro.Renode.Core;
using Antmicro.Renode.Peripherals.I2C;
using Antmicro.Renode.Peripherals.Miscellaneous;

namespace Antmicro.Renode.Peripherals.Sensors
{
    public abstract class AP_BattMonitor_INA2xx : II2CPeripheral
    {
        protected AP_BattMonitor_INA2xx(IMachine machine, DeviceType deviceType)
        {
            physics = AP_PhysicsState.ForMachine(machine);
            this.deviceType = deviceType;
            Reset();
        }

        public void Reset()
        {
            command = 0;
            configuration = deviceType == DeviceType.INA231
                ? DefaultConfiguration : (ushort)0;
        }

        public void Write(byte[] data)
        {
            if(data.Length == 0)
            {
                return;
            }
            command = data[0];
            if(data.Length >= 3 && command == Configuration)
            {
                configuration = (ushort)(data[1] << 8 | data[2]);
            }
        }

        public byte[] Read(int count = 1)
        {
            switch(deviceType)
            {
            case DeviceType.INA226:
                return ReadINA226(count);
            case DeviceType.INA228:
                return ReadINA228(count);
            case DeviceType.INA238:
                return ReadINA238(count);
            case DeviceType.INA231:
                return ReadINA231(count);
            case DeviceType.INA260:
                return ReadINA260(count);
            default:
                return new byte[count];
            }
        }

        public void FinishTransmission()
        {
        }

        private byte[] ReadINA226(int count)
        {
            switch(command)
            {
            case Configuration:
                return Word(configuration, count);
            case 0x02:
                return MeasurementWord(
                    physics.Current.BatteryVoltageV / 0.00125, count);
            case 0x04:
                return MeasurementWord(physics.Current.BatteryCurrentA /
                    (MaximumCurrentA / 32768.0), count);
            case 0xFE:
                return Word(ManufacturerId, count);
            default:
                return new byte[count];
            }
        }

        private byte[] ReadINA228(int count)
        {
            switch(command)
            {
            case 0x05:
                return MeasurementWord24(
                    physics.Current.BatteryVoltageV / 195.3125e-6 * 16.0, count);
            case 0x06:
                return MeasurementWord(
                    (physics.Current.TemperatureK - 273.15) / 7.8125e-3, count);
            case 0x07:
                return MeasurementWord24(
                    physics.Current.BatteryCurrentA /
                    (MaximumCurrentA / (1U << 19)) * 16.0, count);
            case 0x3E:
                return Word(ManufacturerId, count);
            case 0x3F:
                return Word(0x2280, count);
            default:
                return new byte[count];
            }
        }

        private byte[] ReadINA238(int count)
        {
            switch(command)
            {
            case 0x05:
                return MeasurementWord(physics.Current.BatteryVoltageV / 0.003125, count);
            case 0x06:
                return MeasurementWord(
                    Math.Round((physics.Current.TemperatureK - 273.15) / 0.125) *
                    16.0, count);
            case 0x07:
                return MeasurementWord(physics.Current.BatteryCurrentA /
                    (MaximumCurrentA / 32768.0), count);
            case 0x3E:
                return Word(ManufacturerId, count);
            case 0x3F:
                return Word(0x2380, count);
            default:
                return new byte[count];
            }
        }

        private byte[] ReadINA231(int count)
        {
            switch(command)
            {
            case Configuration:
                return Word(configuration, count);
            case 0x01:
                return MeasurementWord(physics.Current.BatteryVoltageV / 0.00125, count);
            case 0x04:
                return MeasurementWord(physics.Current.BatteryCurrentA /
                    (MaximumCurrentA / 32768.0), count);
            default:
                return new byte[count];
            }
        }

        private byte[] ReadINA260(int count)
        {
            switch(command)
            {
            case 0x01:
                return MeasurementWord(physics.Current.BatteryCurrentA / 0.00125, count);
            case 0x02:
                return MeasurementWord(physics.Current.BatteryVoltageV / 0.00125, count);
            case 0xFE:
                return Word(ManufacturerId, count);
            case 0xFF:
                return Word(0x2270, count);
            default:
                return new byte[count];
            }
        }

        private static byte[] MeasurementWord(double value, int count)
        {
            var raw = (int)Math.Round(value);
            raw = Math.Max(short.MinValue, Math.Min(short.MaxValue, raw));
            return Word((ushort)(short)raw, count);
        }

        private static byte[] MeasurementWord24(double value, int count)
        {
            var raw = (int)Math.Round(value);
            raw = Math.Max(-0x800000, Math.Min(0x7FFFFF, raw));
            var result = new byte[count];
            if(count > 0)
            {
                result[0] = (byte)(raw >> 16);
            }
            if(count > 1)
            {
                result[1] = (byte)(raw >> 8);
            }
            if(count > 2)
            {
                result[2] = (byte)raw;
            }
            return result;
        }

        private static byte[] Word(ushort value, int count)
        {
            var result = new byte[count];
            if(count > 0)
            {
                result[0] = (byte)(value >> 8);
            }
            if(count > 1)
            {
                result[1] = (byte)value;
            }
            return result;
        }

        protected enum DeviceType
        {
            INA226,
            INA228,
            INA238,
            INA231,
            INA260,
        }

        private readonly AP_PhysicsState physics;
        private readonly DeviceType deviceType;
        private byte command;
        private ushort configuration;

        private const byte Configuration = 0x00;
        private const ushort DefaultConfiguration = 0x4127;
        private const ushort ManufacturerId = 0x5449;
        private const double MaximumCurrentA = 90.0;
    }

    public class AP_INA226 : AP_BattMonitor_INA2xx
    {
        public AP_INA226(IMachine machine) : base(machine, DeviceType.INA226)
        {
        }
    }

    public class AP_INA228 : AP_BattMonitor_INA2xx
    {
        public AP_INA228(IMachine machine) : base(machine, DeviceType.INA228)
        {
        }
    }

    public class AP_INA238 : AP_BattMonitor_INA2xx
    {
        public AP_INA238(IMachine machine) : base(machine, DeviceType.INA238)
        {
        }
    }

    public class AP_INA231 : AP_BattMonitor_INA2xx
    {
        public AP_INA231(IMachine machine) : base(machine, DeviceType.INA231)
        {
        }
    }

    public class AP_INA260 : AP_BattMonitor_INA2xx
    {
        public AP_INA260(IMachine machine) : base(machine, DeviceType.INA260)
        {
        }
    }

    public class AP_INA3221 : II2CPeripheral
    {
        public AP_INA3221(IMachine machine)
        {
            physics = AP_PhysicsState.ForMachine(machine);
            Reset();
        }

        public void Reset()
        {
            command = 0;
            configuration = 0;
        }

        public void Write(byte[] data)
        {
            if(data.Length == 0)
            {
                return;
            }
            command = data[0];
            if(data.Length >= 3 && command == Configuration)
            {
                configuration = (ushort)(data[1] << 8 | data[2]);
            }
        }

        public byte[] Read(int count = 1)
        {
            if(command == ManufacturerIdRegister)
            {
                return Word(ManufacturerId, count);
            }
            if(command == DieIdRegister)
            {
                return Word(DieId, count);
            }
            if(command == Configuration)
            {
                return Word(configuration, count);
            }
            if(command >= Channel1Shunt && command <= Channel3Bus)
            {
                var busVoltage = (command & 1) == 0;
                var raw = busVoltage
                    ? physics.Current.BatteryVoltageV / BusVoltageLsbV
                    : physics.Current.BatteryCurrentA / CurrentLsbA;
                var value = (int)Math.Round(raw) << 3;
                value = Math.Max(short.MinValue, Math.Min(short.MaxValue, value));
                return Word((ushort)(short)value, count);
            }
            return new byte[count];
        }

        public void FinishTransmission()
        {
        }

        private static byte[] Word(ushort value, int count)
        {
            var result = new byte[count];
            if(count > 0)
            {
                result[0] = (byte)(value >> 8);
            }
            if(count > 1)
            {
                result[1] = (byte)value;
            }
            return result;
        }

        private readonly AP_PhysicsState physics;
        private byte command;
        private ushort configuration;

        private const byte Configuration = 0x00;
        private const byte Channel1Shunt = 0x01;
        private const byte Channel3Bus = 0x06;
        private const byte ManufacturerIdRegister = 0xFE;
        private const byte DieIdRegister = 0xFF;
        private const ushort ManufacturerId = 0x5449;
        private const ushort DieId = 0x3220;
        private const double BusVoltageLsbV = 0.008;
        private const double CurrentLsbA = 0.04;
    }

    public class AP_LTC2946 : II2CPeripheral
    {
        public AP_LTC2946(IMachine machine)
        {
            physics = AP_PhysicsState.ForMachine(machine);
            Reset();
        }

        public void Reset()
        {
            command = 0;
            controlA = 0;
            controlB = 0;
        }

        public void Write(byte[] data)
        {
            if(data.Length == 0)
            {
                return;
            }
            command = data[0];
            if(data.Length < 2)
            {
                return;
            }
            if(command == ControlA)
            {
                controlA = data[1];
            }
            else if(command == ControlB)
            {
                controlB = data[1];
            }
        }

        public byte[] Read(int count = 1)
        {
            switch(command)
            {
            case ControlA:
                return Bytes(controlA, count);
            case ControlB:
                return Bytes(controlB, count);
            case ManufacturerId:
                return Bytes(DeviceId, count);
            case CurrentDelta:
                return MeasurementWord(
                    physics.Current.BatteryCurrentA / CurrentLsbA, count);
            case InputVoltage:
                return MeasurementWord(
                    physics.Current.BatteryVoltageV / VoltageLsbV, count);
            default:
                return new byte[count];
            }
        }

        public void FinishTransmission()
        {
        }

        private static byte[] MeasurementWord(double value, int count)
        {
            var raw = (int)Math.Round(value);
            raw = Math.Max(0, Math.Min(0xFFF, raw));
            return Word((ushort)(raw << 4), count);
        }

        private static byte[] Word(ushort value, int count)
        {
            var result = new byte[count];
            if(count > 0)
            {
                result[0] = (byte)(value >> 8);
            }
            if(count > 1)
            {
                result[1] = (byte)value;
            }
            return result;
        }

        private static byte[] Bytes(byte value, int count)
        {
            var result = new byte[count];
            if(count > 0)
            {
                result[0] = value;
            }
            return result;
        }

        private readonly AP_PhysicsState physics;
        private byte command;
        private byte controlA;
        private byte controlB;

        private const byte ControlA = 0x00;
        private const byte ControlB = 0x01;
        private const byte CurrentDelta = 0x14;
        private const byte InputVoltage = 0x1E;
        private const byte ManufacturerId = 0xE7;
        private const byte DeviceId = 0x60;
        private const double VoltageLsbV = 102.4 / 4095.0;
        private const double CurrentLsbA = (0.1024 / 0.0005) / 4095.0;
    }

    public class AP_AD7091R5 : II2CPeripheral
    {
        public AP_AD7091R5(IMachine machine)
        {
            physics = AP_PhysicsState.ForMachine(machine);
        }

        public void Reset()
        {
        }

        public void Write(byte[] data)
        {
        }

        public byte[] Read(int count = 1)
        {
            var result = new byte[count];
            WriteChannel(result, 0,
                physics.Current.BatteryVoltageV / VoltageMultiplier);
            WriteChannel(result, 1,
                physics.Current.BatteryCurrentA / AmpsPerVolt);
            WriteChannel(result, 2, 0.0);
            WriteChannel(result, 3, 0.0);
            return result;
        }

        public void FinishTransmission()
        {
        }

        private static void WriteChannel(byte[] result, int channel,
                                         double voltage)
        {
            var offset = channel * 2;
            if(offset >= result.Length)
            {
                return;
            }
            var raw = (int)Math.Round(voltage / ReferenceVoltageV * 4096.0);
            raw = Math.Max(0, Math.Min(0xFFF, raw));
            result[offset] = (byte)(channel << 5 | raw >> 8);
            if(offset + 1 < result.Length)
            {
                result[offset + 1] = (byte)raw;
            }
        }

        private readonly AP_PhysicsState physics;

        private const double ReferenceVoltageV = 3.3;
        private const double VoltageMultiplier = 10.0;
        private const double AmpsPerVolt = 10.0;
    }

    public class AP_SMBusBattery : II2CPeripheral
    {
        public AP_SMBusBattery(IMachine machine) : this(machine,
            DefaultManufacturer, 0.0)
        {
        }

        protected AP_SMBusBattery(IMachine machine, byte[] manufacturer,
                                  double internalTemperatureOffsetK,
                                  bool neoDesign = false,
                                  int suiCellCount = 0,
                                  bool solo = false)
        {
            physics = AP_PhysicsState.ForMachine(machine);
            this.manufacturer = manufacturer;
            this.internalTemperatureOffsetK = internalTemperatureOffsetK;
            this.neoDesign = neoDesign;
            this.suiCellCount = suiCellCount;
            this.solo = solo;
            Reset();
        }

        public void Reset()
        {
            command = 0;
        }

        public void Write(byte[] data)
        {
            if(data.Length > 0)
            {
                command = data[0];
            }
        }

        public byte[] Read(int count = 1)
        {
            if(solo)
            {
                switch(command)
                {
                case SoloCellVoltage:
                    return SoloCells(4, 0.125, count);
                case SoloCellVoltageExtended:
                    return SoloCells(6, 1.0 / 6.0, count);
                case SoloCurrent:
                    return SoloCurrentBlock(count);
                case ManufacturerData:
                    return Block(SoloManufacturerData, count);
                }
            }
            if(suiCellCount != 0)
            {
                if(command == SuiCellVoltage)
                {
                    return SuiCells(count);
                }
                if(command == SuiCurrent)
                {
                    return BareCurrent(count);
                }
            }
            if(neoDesign)
            {
                if(command == NeoDesignCellCount)
                {
                    return Word(CellCount, count);
                }
                if(command >= NeoDesignCell1 &&
                   command < NeoDesignCell1 + CellCount)
                {
                    return Word(physics.Current.BatteryVoltageV * 1000.0 /
                        CellCount, count);
                }
            }
            switch(command)
            {
            case Temperature:
                return Word((physics.Current.TemperatureK +
                    internalTemperatureOffsetK) * 10.0, count);
            case ExternalTemperature:
                return Word(physics.Current.TemperatureK * 10.0, count);
            case Voltage:
                return Word(physics.Current.BatteryVoltageV * 1000.0, count);
            case Current:
                return SignedWord(-physics.Current.BatteryCurrentA * 1000.0,
                    count);
            case RemainingCapacity:
                return Word(4000, count);
            case FullChargeCapacity:
                return Word(5000, count);
            case CycleCount:
                return Word(42, count);
            case SpecificationInfo:
                return Word(0x0030, count);
            case SerialNumber:
                return Word(1234, count);
            case ManufacturerName:
                return Block(manufacturer, count);
            default:
                if(command <= Cell1 && command >= Cell12)
                {
                    var cell = Cell1 - command;
                    var millivolts = cell < CellCount
                        ? physics.Current.BatteryVoltageV * 1000.0 / CellCount
                        : 0.0;
                    return Word(millivolts, count);
                }
                return new byte[count];
            }
        }

        public void FinishTransmission()
        {
        }

        private byte[] Word(double value, int count)
        {
            var raw = (int)Math.Round(value);
            raw = Math.Max(0, Math.Min(ushort.MaxValue, raw));
            return Word((ushort)raw, count);
        }

        private byte[] SignedWord(double value, int count)
        {
            var raw = (int)Math.Round(value);
            raw = Math.Max(short.MinValue, Math.Min(short.MaxValue, raw));
            return Word((ushort)(short)raw, count);
        }

        private byte[] Word(ushort value, int count)
        {
            var result = new byte[count];
            if(count > 0)
            {
                result[0] = (byte)value;
            }
            if(count > 1)
            {
                result[1] = (byte)(value >> 8);
            }
            if(count > 2)
            {
                result[2] = Pec(command, result, 2);
            }
            return result;
        }

        private byte[] Block(byte[] value, int count)
        {
            var result = new byte[count];
            if(count == 0)
            {
                return result;
            }
            result[0] = (byte)value.Length;
            var copied = Math.Min(value.Length, count - 1);
            Array.Copy(value, 0, result, 1, copied);
            if(count > value.Length + 1)
            {
                result[value.Length + 1] = Pec(command, result,
                    value.Length + 1);
            }
            return result;
        }

        private byte[] SuiCells(int count)
        {
            var result = new byte[count];
            if(count == 0)
            {
                return result;
            }
            result[0] = SuiCellBlockLength;
            for(var cell = 0; cell < 4; cell++)
            {
                var fraction = suiCellCount == 6
                    ? Sui6DirectCellFractions[cell]
                    : 1.0 / suiCellCount;
                var cellMillivolts = (int)Math.Round(
                    physics.Current.BatteryVoltageV * 1000.0 * fraction);
                cellMillivolts = Math.Max(0,
                    Math.Min(ushort.MaxValue, cellMillivolts));
                var offset = 1 + cell * 2;
                if(offset < count)
                {
                    result[offset] = (byte)cellMillivolts;
                }
                if(offset + 1 < count)
                {
                    result[offset + 1] = (byte)(cellMillivolts >> 8);
                }
            }
            return result;
        }

        private byte[] BareCurrent(int count)
        {
            var result = new byte[count];
            var raw = (long)Math.Round(-physics.Current.BatteryCurrentA *
                1000.0);
            raw = Math.Max(int.MinValue, Math.Min(int.MaxValue, raw));
            for(var index = 0; index < Math.Min(4, count); index++)
            {
                result[index] = (byte)(raw >> (index * 8));
            }
            return result;
        }

        private byte[] SoloCells(int cells, double fraction, int count)
        {
            var data = new byte[cells * 2];
            var millivolts = (int)Math.Round(
                physics.Current.BatteryVoltageV * 1000.0 * fraction);
            millivolts = Math.Max(0, Math.Min(ushort.MaxValue, millivolts));
            for(var cell = 0; cell < cells; cell++)
            {
                data[cell * 2] = (byte)millivolts;
                data[cell * 2 + 1] = (byte)(millivolts >> 8);
            }
            return Block(data, count);
        }

        private byte[] SoloCurrentBlock(int count)
        {
            var data = new byte[4];
            var raw = (long)Math.Round(-physics.Current.BatteryCurrentA *
                1000.0);
            raw = Math.Max(int.MinValue, Math.Min(int.MaxValue, raw));
            for(var index = 0; index < data.Length; index++)
            {
                data[index] = (byte)(raw >> (index * 8));
            }
            return Block(data, count);
        }

        private static byte Pec(byte register, byte[] data, int length)
        {
            var crc = (byte)0;
            crc = CrcByte(crc, Address << 1);
            crc = CrcByte(crc, register);
            crc = CrcByte(crc, Address << 1 | 1);
            for(var index = 0; index < length; index++)
            {
                crc = CrcByte(crc, data[index]);
            }
            return crc;
        }

        private static byte CrcByte(byte crc, int value)
        {
            for(var bit = 0; bit < 8; bit++)
            {
                var invert = ((crc ^ value) & 0x80) != 0;
                crc <<= 1;
                value <<= 1;
                if(invert)
                {
                    crc ^= 0x07;
                }
            }
            return crc;
        }

        private readonly AP_PhysicsState physics;
        private readonly byte[] manufacturer;
        private readonly double internalTemperatureOffsetK;
        private readonly bool neoDesign;
        private readonly int suiCellCount;
        private readonly bool solo;
        private byte command;

        private const byte Address = 0x0B;
        private const byte Temperature = 0x08;
        private const byte Voltage = 0x09;
        private const byte Current = 0x0A;
        private const byte RemainingCapacity = 0x0F;
        private const byte FullChargeCapacity = 0x10;
        private const byte CycleCount = 0x17;
        private const byte SpecificationInfo = 0x1A;
        private const byte SerialNumber = 0x1C;
        private const byte ManufacturerName = 0x20;
        private const byte Cell1 = 0x3F;
        private const byte Cell12 = 0x34;
        private const byte ExternalTemperature = 0x48;
        private const byte NeoDesignCell1 = 0x30;
        private const byte NeoDesignCellCount = 0x5C;
        private const byte SuiCellVoltage = 0x28;
        private const byte SuiCurrent = 0x2A;
        private const byte SuiCellBlockLength = 8;
        private const byte SoloCellVoltage = 0x28;
        private const byte SoloCellVoltageExtended = 0x29;
        private const byte SoloCurrent = 0x2A;
        private const byte ManufacturerData = 0x23;
        private const int CellCount = 4;
        private static readonly byte[] DefaultManufacturer = {
            (byte)'R', (byte)'e', (byte)'n', (byte)'o', (byte)'d', (byte)'e'
        };
        private static readonly double[] Sui6DirectCellFractions = {
            0.10, 0.12, 0.14, 0.16
        };
        private static readonly byte[] SoloManufacturerData = {
            0, 0, 0, 0, 0, 0
        };
    }

    public class AP_SMBusMaxellBattery : AP_SMBusBattery
    {
        public AP_SMBusMaxellBattery(IMachine machine) : base(machine,
            MaxellManufacturer, 0.0)
        {
        }

        private static readonly byte[] MaxellManufacturer = {
            (byte)'H', (byte)'i', (byte)'t', (byte)'a', (byte)'c', (byte)'h',
            (byte)'i', (byte)' ', (byte)'m', (byte)'a', (byte)'x', (byte)'e',
            (byte)'l', (byte)'l'
        };
    }

    public class AP_SMBusRotoyeBattery : AP_SMBusBattery
    {
        public AP_SMBusRotoyeBattery(IMachine machine) : base(machine,
            RotoyeManufacturer, -2.0)
        {
        }

        private static readonly byte[] RotoyeManufacturer = {
            (byte)'R', (byte)'o', (byte)'t', (byte)'o', (byte)'y', (byte)'e'
        };
    }

    public class AP_SMBusNeoDesignBattery : AP_SMBusBattery
    {
        public AP_SMBusNeoDesignBattery(IMachine machine) : base(machine,
            NeoDesignManufacturer, 0.0, true)
        {
        }

        private static readonly byte[] NeoDesignManufacturer = {
            (byte)'N', (byte)'e', (byte)'o', (byte)'D', (byte)'e', (byte)'s',
            (byte)'i', (byte)'g', (byte)'n'
        };
    }

    public class AP_SMBusSUI3Battery : AP_SMBusBattery
    {
        public AP_SMBusSUI3Battery(IMachine machine) : base(machine,
            SuiManufacturer, 0.0, false, 3)
        {
        }

        private static readonly byte[] SuiManufacturer = {
            (byte)'S', (byte)'U', (byte)'I', (byte)'3'
        };
    }

    public class AP_SMBusSUI6Battery : AP_SMBusBattery
    {
        public AP_SMBusSUI6Battery(IMachine machine) : base(machine,
            SuiManufacturer, 0.0, false, 6)
        {
        }

        private static readonly byte[] SuiManufacturer = {
            (byte)'S', (byte)'U', (byte)'I', (byte)'6'
        };
    }

    public class AP_SMBusSoloBattery : AP_SMBusBattery
    {
        public AP_SMBusSoloBattery(IMachine machine) : base(machine,
            SoloManufacturer, 0.0, false, 0, true)
        {
        }

        private static readonly byte[] SoloManufacturer = {
            (byte)'S', (byte)'o', (byte)'l', (byte)'o'
        };
    }

    public class AP_BQ76952 : II2CPeripheral
    {
        public AP_BQ76952(IMachine machine)
        {
            physics = AP_PhysicsState.ForMachine(machine);
            Reset();
        }

        public void Reset()
        {
            command = 0;
            indirectAddress = 0;
        }

        public void Write(byte[] data)
        {
            if(data.Length == 0)
            {
                return;
            }
            command = data[0];
            if(data.Length < 2)
            {
                return;
            }
            if(command == IndirectCommandLow)
            {
                indirectAddress = (ushort)((indirectAddress & 0xFF00) |
                    data[1]);
            }
            else if(command == IndirectCommandHigh)
            {
                indirectAddress = (ushort)((indirectAddress & 0x00FF) |
                    data[1] << 8);
            }
        }

        public byte[] Read(int count = 1)
        {
            if(command == IndirectResponse)
            {
                return indirectAddress == DeviceNumber
                    ? DWord(DeviceId, count) : new byte[count];
            }
            if(command == AlarmRawStatus)
            {
                return Word(AlarmWake | AlarmFullScan, count);
            }
            if(command == BatteryStatus)
            {
                return Word(0, count);
            }
            if(command == StackVoltage || command == PackVoltage)
            {
                return MeasurementWord(
                    physics.Current.BatteryVoltageV * 100.0, false, count);
            }
            if(command >= Cell1Voltage &&
               command < Cell1Voltage + CellCount * 2 &&
               (command - Cell1Voltage) % 2 == 0)
            {
                return MeasurementWord(
                    physics.Current.BatteryVoltageV * 1000.0 / CellCount,
                    false, count);
            }
            if(command == Current)
            {
                return MeasurementWord(
                    -physics.Current.BatteryCurrentA * 1000.0, true, count);
            }
            if(command == InternalTemperature)
            {
                return MeasurementWord(
                    physics.Current.TemperatureK * 10.0, true, count);
            }
            return new byte[count];
        }

        public void FinishTransmission()
        {
        }

        private static byte[] MeasurementWord(double value, bool signed,
                                              int count)
        {
            var raw = (int)Math.Round(value);
            raw = signed
                ? Math.Max(short.MinValue, Math.Min(short.MaxValue, raw))
                : Math.Max(0, Math.Min(ushort.MaxValue, raw));
            return Word((ushort)(signed ? (short)raw : raw), count);
        }

        private static byte[] Word(ushort value, int count)
        {
            var result = new byte[count];
            if(count > 0)
            {
                result[0] = (byte)value;
            }
            if(count > 1)
            {
                result[1] = (byte)(value >> 8);
            }
            return result;
        }

        private static byte[] DWord(uint value, int count)
        {
            var result = new byte[count];
            for(var index = 0; index < Math.Min(4, count); index++)
            {
                result[index] = (byte)(value >> (index * 8));
            }
            return result;
        }

        private readonly AP_PhysicsState physics;
        private byte command;
        private ushort indirectAddress;

        private const byte BatteryStatus = 0x12;
        private const byte Cell1Voltage = 0x14;
        private const byte StackVoltage = 0x34;
        private const byte PackVoltage = 0x36;
        private const byte Current = 0x3A;
        private const byte IndirectCommandLow = 0x3E;
        private const byte IndirectCommandHigh = 0x3F;
        private const byte IndirectResponse = 0x40;
        private const byte AlarmRawStatus = 0x64;
        private const byte InternalTemperature = 0x68;
        private const ushort DeviceNumber = 0x0001;
        private const uint DeviceId = 0x00007695;
        private const ushort AlarmWake = 1 << 0;
        private const ushort AlarmFullScan = 1 << 7;
        private const int CellCount = 6;
    }
}
