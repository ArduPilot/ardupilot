// Physics-driven I2C rangefinders used by the ArduPilot production backends.
using System;
using System.Collections.Generic;
using Antmicro.Renode.Core;
using Antmicro.Renode.Peripherals.I2C;
using Antmicro.Renode.Peripherals.Miscellaneous;

namespace Antmicro.Renode.Peripherals.Sensors
{
    internal static class AP_I2CRangefinderPhysics
    {
        public static double DistanceMetres(AP_PhysicsState physics, uint index)
        {
            var truth = physics.Current;
            return truth.TimestampUs == 0
                ? DefaultDistanceMetres
                : truth.RangefinderM[index];
        }

        public static ushort DistanceCentimetres(AP_PhysicsState physics, uint index)
        {
            return ClampUInt16(DistanceMetres(physics, index) * 100.0);
        }

        public static ushort DistanceMillimetres(AP_PhysicsState physics, uint index)
        {
            return ClampUInt16(DistanceMetres(physics, index) * 1000.0);
        }

        public static ushort ClampUInt16(double value)
        {
            return (ushort)Math.Max(1, Math.Min(ushort.MaxValue, Math.Round(value)));
        }

        private const double DefaultDistanceMetres = 5.0;
    }

    // MaxBotix I2CXL at 0x70: command 0x51 followed by a big-endian cm word.
    public class AP_MaxSonarI2C : II2CPeripheral
    {
        public AP_MaxSonarI2C(IMachine machine)
        {
            physics = AP_PhysicsState.ForMachine(machine);
            Reset();
        }

        public void Reset()
        {
            readIndex = 0;
            readSinceFinish = false;
        }

        public void Write(byte[] data)
        {
            if(data.Length > 0)
            {
                WriteCount++;
                readIndex = 0;
                readSinceFinish = false;
            }
        }

        public byte[] Read(int count = 1)
        {
            var distance = AP_I2CRangefinderPhysics.DistanceCentimetres(
                physics, RangefinderIndex);
            var sample = new byte[] { (byte)(distance >> 8), (byte)distance };
            return CopySample(sample, count);
        }

        public void FinishTransmission()
        {
            if(readSinceFinish)
            {
                readIndex = 0;
                readSinceFinish = false;
            }
        }

        public uint RangefinderIndex { get; set; }
        public ulong ReadCount { get; private set; }
        public ulong WriteCount { get; private set; }

        private byte[] CopySample(byte[] sample, int count)
        {
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

        private readonly AP_PhysicsState physics;
        private int readIndex;
        private bool readSinceFinish;
    }

    // Benewake TFS20-L at 0x10 using its little-endian register map.
    public class AP_TFS20L : AP_I2CRegisterDevice
    {
        public AP_TFS20L(IMachine machine)
        {
            physics = AP_PhysicsState.ForMachine(machine);
            Reset();
        }

        public override void Reset()
        {
            base.Reset();
            Registers[VersionMajor] = 1;
            Registers[VersionMajor + 1] = 0;
            Registers[VersionMajor + 2] = 0;
        }

        public override byte[] Read(int count = 1)
        {
            var distance = AP_I2CRangefinderPhysics.DistanceCentimetres(
                physics, RangefinderIndex);
            Registers[Distance] = (byte)distance;
            Registers[Distance + 1] = (byte)(distance >> 8);
            var strength = SuppressData ? 0 : SignalStrength;
            Registers[Strength] = (byte)strength;
            Registers[Strength + 1] = (byte)(strength >> 8);
            return base.Read(count);
        }

        public uint RangefinderIndex { get; set; }
        public bool SuppressData { get; set; }

        private readonly AP_PhysicsState physics;
        private const byte Distance = 0x00;
        private const byte Strength = 0x02;
        private const byte VersionMajor = 0x0C;
        private const ushort SignalStrength = 200;
    }

    // Benewake TFMini Plus command protocol at 0x10.
    public class AP_TFMiniPlusI2C : II2CPeripheral
    {
        public AP_TFMiniPlusI2C(IMachine machine)
        {
            physics = AP_PhysicsState.ForMachine(machine);
            Reset();
        }

        public void Reset()
        {
            command = 0;
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
            command = data.Length > 2 ? data[2] : (byte)0;
            readIndex = 0;
            readSinceFinish = false;
        }

        public byte[] Read(int count = 1)
        {
            byte[] sample;
            if(command == FirmwareVersionCommand)
            {
                sample = new byte[] { 0x5A, 0x07, 0x01, 0x03, 0x00, 0x02, 0x00 };
            }
            else
            {
                var distance = AP_I2CRangefinderPhysics.DistanceCentimetres(
                    physics, RangefinderIndex);
                var strength = SuppressData ? 0 : SignalStrength;
                sample = new byte[] {
                    0x59, 0x59,
                    (byte)distance, (byte)(distance >> 8),
                    (byte)strength, (byte)(strength >> 8),
                    0, 0, 0, 0, 0,
                };
            }
            byte checksum = 0;
            for(var i = 0; i < sample.Length - 1; i++)
            {
                checksum += sample[i];
            }
            sample[sample.Length - 1] = checksum;
            return CopySample(sample, count);
        }

        public void FinishTransmission()
        {
            if(readSinceFinish)
            {
                readIndex = 0;
                readSinceFinish = false;
            }
        }

        public uint RangefinderIndex { get; set; }
        public bool SuppressData { get; set; }
        public ulong ReadCount { get; private set; }
        public ulong WriteCount { get; private set; }

        private byte[] CopySample(byte[] sample, int count)
        {
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

        private readonly AP_PhysicsState physics;
        private byte command;
        private int readIndex;
        private bool readSinceFinish;
        private const byte FirmwareVersionCommand = 0x01;
        private const ushort SignalStrength = 200;
    }

    // TeraRanger Evo at 0x31: WHOAMI register and CRC-protected millimetres.
    public class AP_TeraRangerI2C : II2CPeripheral
    {
        public AP_TeraRangerI2C(IMachine machine)
        {
            physics = AP_PhysicsState.ForMachine(machine);
            Reset();
        }

        public void Reset()
        {
            command = 0;
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
            command = data[0];
            readIndex = 0;
            readSinceFinish = false;
        }

        public byte[] Read(int count = 1)
        {
            byte[] sample;
            if(command == WhoAmIRegister)
            {
                sample = new byte[] { WhoAmIValue };
            }
            else
            {
                var distance = AP_I2CRangefinderPhysics.DistanceMillimetres(
                    physics, RangefinderIndex);
                sample = new byte[] { (byte)(distance >> 8), (byte)distance, 0 };
                sample[2] = CalculateCrc8(sample[0], sample[1]);
                if(SuppressData)
                {
                    sample[2] ^= 0xFF;
                }
            }
            return CopySample(sample, count);
        }

        public void FinishTransmission()
        {
            if(readSinceFinish)
            {
                readIndex = 0;
                readSinceFinish = false;
            }
        }

        public uint RangefinderIndex { get; set; }
        public bool SuppressData { get; set; }
        public ulong ReadCount { get; private set; }
        public ulong WriteCount { get; private set; }

        private byte[] CopySample(byte[] sample, int count)
        {
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

        private static byte CalculateCrc8(byte first, byte second)
        {
            byte crc = 0;
            foreach(var value in new byte[] { first, second })
            {
                crc ^= value;
                for(var bit = 0; bit < 8; bit++)
                {
                    crc = (byte)((crc & 0x80) != 0
                        ? (crc << 1) ^ 0x07
                        : crc << 1);
                }
            }
            return crc;
        }

        private readonly AP_PhysicsState physics;
        private byte command;
        private int readIndex;
        private bool readSinceFinish;
        private const byte WhoAmIRegister = 0x01;
        private const byte WhoAmIValue = 0xA1;
    }

    // Nooploop TOFSense F2 at 0x08 using its little-endian eight-byte packet.
    public class AP_TOFSenseF_I2C : II2CPeripheral
    {
        public AP_TOFSenseF_I2C(IMachine machine)
        {
            physics = AP_PhysicsState.ForMachine(machine);
        }

        public void Reset()
        {
        }

        public void Write(byte[] data)
        {
            if(data.Length > 0)
            {
                WriteCount++;
            }
        }

        public byte[] Read(int count = 1)
        {
            var distance = (uint)Math.Round(
                AP_I2CRangefinderPhysics.DistanceMetres(
                    physics, RangefinderIndex) * 1000.0);
            var statusAndStrength = SuppressData ? 0U : 100U << 16 | 1U;
            var sample = new byte[] {
                (byte)distance, (byte)(distance >> 8),
                (byte)(distance >> 16), (byte)(distance >> 24),
                (byte)statusAndStrength, (byte)(statusAndStrength >> 8),
                (byte)(statusAndStrength >> 16), (byte)(statusAndStrength >> 24),
            };
            var result = new byte[count];
            for(var i = 0; i < count; i++)
            {
                result[i] = sample[i % sample.Length];
            }
            ReadCount += (ulong)count;
            return result;
        }

        public void FinishTransmission()
        {
        }

        public uint RangefinderIndex { get; set; }
        public bool SuppressData { get; set; }
        public ulong ReadCount { get; private set; }
        public ulong WriteCount { get; private set; }

        private readonly AP_PhysicsState physics;
    }

    // LightWare GRF-250/500 at 0x66 using its register-like message IDs.
    public class AP_LightWareGRF_I2C : II2CPeripheral
    {
        public AP_LightWareGRF_I2C(IMachine machine)
        {
            physics = AP_PhysicsState.ForMachine(machine);
            Reset();
        }

        public void Reset()
        {
            messageId = ProductName;
            updateRate = 50;
            distanceOutput = 5;
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
            messageId = data[0];
            if(data.Length >= 5)
            {
                var value = (uint)(data[1] | data[2] << 8 |
                    data[3] << 16 | data[4] << 24);
                if(messageId == UpdateRate)
                {
                    updateRate = value;
                }
                else if(messageId == DistanceOutput)
                {
                    distanceOutput = value;
                }
            }
            readIndex = 0;
            readSinceFinish = false;
        }

        public byte[] Read(int count = 1)
        {
            byte[] sample;
            switch(messageId)
            {
            case ProductName:
                sample = new byte[16];
                Array.Copy(new byte[] { (byte)'G', (byte)'R', (byte)'F',
                    (byte)'-', (byte)'2', (byte)'5', (byte)'0' }, sample, 7);
                break;
            case UpdateRate:
                sample = UInt32LittleEndian(updateRate);
                break;
            case DistanceOutput:
                sample = UInt32LittleEndian(distanceOutput);
                break;
            case DistanceDataCm:
                var distanceDecimetres = SuppressData ? 0U : (uint)Math.Round(
                    AP_I2CRangefinderPhysics.DistanceMetres(
                        physics, RangefinderIndex) * 10.0);
                sample = new byte[8];
                Array.Copy(UInt32LittleEndian(distanceDecimetres), sample, 4);
                Array.Copy(UInt32LittleEndian(100), 0, sample, 4, 4);
                break;
            default:
                sample = new byte[count];
                break;
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

        public uint RangefinderIndex { get; set; }
        public bool SuppressData { get; set; }
        public ulong ReadCount { get; private set; }
        public ulong WriteCount { get; private set; }

        private static byte[] UInt32LittleEndian(uint value)
        {
            return new byte[] {
                (byte)value, (byte)(value >> 8),
                (byte)(value >> 16), (byte)(value >> 24),
            };
        }

        private readonly AP_PhysicsState physics;
        private byte messageId;
        private uint updateRate;
        private uint distanceOutput;
        private int readIndex;
        private bool readSinceFinish;
        private const byte ProductName = 0;
        private const byte DistanceOutput = 27;
        private const byte DistanceDataCm = 44;
        private const byte UpdateRate = 74;
    }

    // Legacy LightWare I2C protocol at a configurable address. Returning no
    // response bytes to the newer ASCII handshake deliberately selects the
    // production driver's two-byte legacy register path.
    public class AP_LightWareLegacyI2C : II2CPeripheral
    {
        public AP_LightWareLegacyI2C(IMachine machine)
        {
            physics = AP_PhysicsState.ForMachine(machine);
            Reset();
        }

        public void Reset()
        {
            register = 0;
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
            register = data.Length == 1 ? data[0] : AsciiCommand;
            readIndex = 0;
            readSinceFinish = false;
        }

        public byte[] Read(int count = 1)
        {
            ushort value;
            if(register == LostSignalTimeout)
            {
                value = 20;
            }
            else if(register == DistanceRegister)
            {
                value = AP_I2CRangefinderPhysics.DistanceCentimetres(
                    physics, RangefinderIndex);
            }
            else
            {
                value = 0;
            }
            var sample = new byte[] { (byte)(value >> 8), (byte)value };
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

        public uint RangefinderIndex { get; set; }
        public ulong ReadCount { get; private set; }
        public ulong WriteCount { get; private set; }

        private readonly AP_PhysicsState physics;
        private byte register;
        private int readIndex;
        private bool readSinceFinish;
        private const byte DistanceRegister = 0;
        private const byte LostSignalTimeout = 22;
        private const byte AsciiCommand = 0xFF;
    }

    // Garmin/PulsedLight LidarLite V3 at 0x62. Register 0x0F is addressed
    // with the auto-increment bit set for the big-endian distance word.
    public class AP_LidarLiteI2C : AP_I2CRegisterDevice
    {
        public AP_LidarLiteI2C(IMachine machine)
        {
            physics = AP_PhysicsState.ForMachine(machine);
            Reset();
        }

        public override void Reset()
        {
            base.Reset();
            Registers[HardwareVersion] = 0x15;
            Registers[SoftwareVersion] = 1;
        }

        public override void Write(byte[] data)
        {
            if(data.Length > 0)
            {
                var normalized = (byte[])data.Clone();
                normalized[0] &= 0x7F;
                base.Write(normalized);
                return;
            }
            base.Write(data);
        }

        public override byte[] Read(int count = 1)
        {
            var distance = AP_I2CRangefinderPhysics.DistanceCentimetres(
                physics, RangefinderIndex);
            Registers[DistanceRegister] = (byte)(distance >> 8);
            Registers[DistanceRegister + 1] = (byte)distance;
            return base.Read(count);
        }

        public uint RangefinderIndex { get; set; }

        private readonly AP_PhysicsState physics;
        private const byte DistanceRegister = 0x0F;
        private const byte HardwareVersion = 0x41;
        private const byte SoftwareVersion = 0x4F;
    }

    // VL53L0X at 0x29. The model retains the sensor's paged 8-bit register
    // state while supplying hardware-owned SPAD, ready and range registers.
    public class AP_VL53L0X : II2CPeripheral
    {
        public AP_VL53L0X(IMachine machine)
        {
            physics = AP_PhysicsState.ForMachine(machine);
            Reset();
        }

        public void Reset()
        {
            registers.Clear();
            page = 0;
            register = 0;
            readIndex = 0;
            readSinceFinish = false;
            SetRegister(0, 0xC0, 0xEE);
            SetRegister(0, 0xC1, 0xAA);
            for(var offset = 0; offset < 6; offset++)
            {
                SetRegister(0, 0xB0 + offset, 0xFF);
            }
        }

        public void Write(byte[] data)
        {
            if(data.Length == 0)
            {
                return;
            }
            WriteCount++;
            register = data[0];
            if(data.Length > 1)
            {
                for(var offset = 1; offset < data.Length; offset++)
                {
                    var target = (byte)(register + offset - 1);
                    if(target == PageSelect)
                    {
                        page = data[offset];
                    }
                    else
                    {
                        SetRegister(page, target, data[offset]);
                    }
                }
            }
            readIndex = 0;
            readSinceFinish = false;
        }

        public byte[] Read(int count = 1)
        {
            var result = new byte[count];
            for(var i = 0; i < count; i++)
            {
                result[i] = ReadRegister(page, (byte)(register + readIndex + i));
            }
            ReadCount += (ulong)count;
            readIndex = (readIndex + count) & 0xFF;
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

        public uint RangefinderIndex { get; set; }
        public bool SuppressData { get; set; }
        public ulong ReadCount { get; private set; }
        public ulong WriteCount { get; private set; }

        private byte ReadRegister(byte selectedPage, byte address)
        {
            if(selectedPage == 0 && address == InterruptStatus)
            {
                return SuppressData ? (byte)0 : (byte)7;
            }
            if(selectedPage == 0 &&
                (address == RangeHigh || address == RangeHigh + 1))
            {
                var distance = AP_I2CRangefinderPhysics.DistanceMillimetres(
                    physics, RangefinderIndex);
                return address == RangeHigh
                    ? (byte)(distance >> 8)
                    : (byte)distance;
            }
            if(selectedPage == 7 && address == SpadReady)
            {
                return 1;
            }
            if(selectedPage == 7 && address == SpadInfo)
            {
                return 0x8C;
            }
            return GetRegister(selectedPage, address);
        }

        private byte GetRegister(byte selectedPage, int address)
        {
            byte value;
            return registers.TryGetValue(selectedPage << 8 | address, out value)
                ? value : (byte)0;
        }

        private void SetRegister(byte selectedPage, int address, byte value)
        {
            registers[selectedPage << 8 | address] = value;
        }

        private readonly AP_PhysicsState physics;
        private readonly Dictionary<int, byte> registers = new Dictionary<int, byte>();
        private byte page;
        private byte register;
        private int readIndex;
        private bool readSinceFinish;
        private const byte PageSelect = 0xFF;
        private const byte InterruptStatus = 0x13;
        private const byte RangeHigh = 0x1E;
        private const byte SpadReady = 0x83;
        private const byte SpadInfo = 0x92;
    }

    // VL53L1X at 0x29 with a flat 16-bit register address space. Startup
    // defaults are sufficient for the production timing calculations; all
    // configuration writes remain readable by later calibration steps.
    public class AP_VL53L1X : II2CPeripheral
    {
        public AP_VL53L1X(IMachine machine)
        {
            physics = AP_PhysicsState.ForMachine(machine);
            Reset();
        }

        public void Reset()
        {
            registers.Clear();
            register = 0;
            readIndex = 0;
            readSinceFinish = false;
            SetRegister(IdentityModel, 0xEA);
            SetRegister(IdentityModule, 0xCC);
            SetRegister16(FastOscFrequency, 0x0A00);
            SetRegister16(OscCalibrate, 0x00A0);
            SetRegister(VcselPeriodA, 0x0F);
            SetRegister(VcselPeriodB, 0x0D);
            SetRegister16(TimeoutMacroA, 0x000B);
            SetRegister(RangeStatus, RangeComplete);
        }

        public void Write(byte[] data)
        {
            if(data.Length < 2)
            {
                return;
            }
            WriteCount++;
            register = (ushort)((data[0] << 8) | data[1]);
            for(var offset = 2; offset < data.Length; offset++)
            {
                SetRegister(register + offset - 2, data[offset]);
            }
            readIndex = 0;
            readSinceFinish = false;
        }

        public byte[] Read(int count = 1)
        {
            var result = new byte[count];
            for(var i = 0; i < count; i++)
            {
                result[i] = ReadRegister(register + readIndex + i);
            }
            ReadCount += (ulong)count;
            readIndex += count;
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

        public uint RangefinderIndex { get; set; }
        public bool SuppressData { get; set; }
        public ulong ReadCount { get; private set; }
        public ulong WriteCount { get; private set; }

        private byte ReadRegister(int address)
        {
            if(address == GpioStatus)
            {
                return SuppressData ? (byte)1 : (byte)0;
            }
            if(address == RangeHigh || address == RangeHigh + 1)
            {
                var desired = AP_I2CRangefinderPhysics.DistanceMillimetres(
                    physics, RangefinderIndex);
                var raw = AP_I2CRangefinderPhysics.ClampUInt16(
                    desired * 2048.0 / 2011.0);
                return address == RangeHigh ? (byte)(raw >> 8) : (byte)raw;
            }
            byte value;
            return registers.TryGetValue(address, out value) ? value : (byte)0;
        }

        private void SetRegister(int address, byte value)
        {
            registers[address] = value;
        }

        private void SetRegister16(int address, ushort value)
        {
            SetRegister(address, (byte)(value >> 8));
            SetRegister(address + 1, (byte)value);
        }

        private readonly AP_PhysicsState physics;
        private readonly Dictionary<int, byte> registers = new Dictionary<int, byte>();
        private ushort register;
        private int readIndex;
        private bool readSinceFinish;
        private const int FastOscFrequency = 0x0006;
        private const int GpioStatus = 0x0031;
        private const int TimeoutMacroA = 0x005E;
        private const int VcselPeriodA = 0x0060;
        private const int VcselPeriodB = 0x0063;
        private const int RangeStatus = 0x0089;
        private const int RangeHigh = 0x0096;
        private const int OscCalibrate = 0x00DE;
        private const int IdentityModel = 0x010F;
        private const int IdentityModule = 0x0110;
        private const byte RangeComplete = 9;
    }
}
