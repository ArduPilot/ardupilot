// Register-pointer models for barometers used by the wider ArduPilot hwdef
// set. Each exposes a stable bench sample and the identity/status behavior
// required by its production backend.
//
using System;
using Antmicro.Renode.Core;
using Antmicro.Renode.Peripherals;
using Antmicro.Renode.Peripherals.I2C;
using Antmicro.Renode.Peripherals.Miscellaneous;
using Antmicro.Renode.Peripherals.SPI;

namespace Antmicro.Renode.Peripherals.Sensors
{
    public class AP_SPIBarometer : ISPIPeripheral, IGPIOReceiver
    {
        public AP_SPIBarometer(byte chipId, byte chipIdRegister)
        {
            this.chipId = chipId;
            this.chipIdRegister = (byte)(chipIdRegister & RegisterMask);
            registers = new byte[256];
            Reset();
        }

        public void Reset()
        {
            Array.Clear(registers, 0, registers.Length);
            registers[chipIdRegister] = chipId;
            registers[0x03] = 0x60;
            registers[0x08] = 0xD0;
            registers[0x11] = 0x02;
            registers[0x27] = 0x13;
            registers[0x28] = 0x60;
            ConfigureSample();
            transferByte = 0;
        }

        public byte Transmit(byte value)
        {
            if(transferByte++ == 0)
            {
                reading = (value & ReadFlag) != 0;
                currentRegister = (byte)(value & RegisterMask);
                return 0;
            }
            if(reading)
            {
                return registers[currentRegister++];
            }
            registers[currentRegister++] = value;
            return 0;
        }

        public void FinishTransmission()
        {
            transferByte = 0;
        }

        public void OnGPIO(int number, bool value)
        {
            if(value)
            {
                FinishTransmission();
            }
        }

        private void ConfigureSample()
        {
            if(chipIdRegister == Bmp280ChipId)
            {
                // BMP280 datasheet calibration and raw sample, with the SPI
                // read flag removed from the logical 0x80..0xFF addresses.
                WriteU16LE(0x08, 27504);
                WriteS16LE(0x0A, 26435);
                WriteS16LE(0x0C, -1000);
                WriteU16LE(0x0E, 36477);
                WriteS16LE(0x10, -10685);
                WriteS16LE(0x12, 3024);
                WriteS16LE(0x14, 2855);
                WriteS16LE(0x16, 140);
                WriteS16LE(0x18, -7);
                WriteS16LE(0x1A, 15500);
                WriteS16LE(0x1C, -14600);
                WriteS16LE(0x1E, 6000);
                registers[0x77] = 0x65;
                registers[0x78] = 0x5C;
                registers[0x79] = 0xC0;
                registers[0x7A] = 0x7E;
                registers[0x7B] = 0xED;
                return;
            }
            if(chipIdRegister == Bmp388ChipId)
            {
                WriteU16LE(0x31, 0);
                WriteU16LE(0x33, 0xFFFF);
                WriteS16LE(0x36, 0x7FFF);
                WriteS16LE(0x38, 0x4000);
                WriteU24LE(0x04, 6485000);
                WriteU24LE(0x07, 409600);
                return;
            }
            if(chipIdRegister == Bmp581ChipId)
            {
                WriteU24LE(0x1D, 25U << 16);
                WriteU24LE(0x20, 101325U << 6);
                return;
            }
            if(chipIdRegister == DpsChipId)
            {
                // C0=50, C1=0, C00=100000; zero raw inputs describe a
                // stable 100 kPa, 25 C sample.
                registers[0x10] = 0x03;
                registers[0x11] = 0x20;
                registers[0x12] = 0x00;
                registers[0x13] = 0x18;
                registers[0x14] = 0x6A;
                registers[0x15] = 0x00;
                registers[0x28] = 0x00;
                return;
            }
            if(chipIdRegister == LpsChipId)
            {
                WriteU24LE(0x28, 101325U * 4096U / 100U);
                var temperature = (short)(25 * 100);
                registers[0x2B] = (byte)temperature;
                registers[0x2C] = (byte)(temperature >> 8);
            }
        }

        private void WriteU16LE(int register, ushort value)
        {
            registers[register] = (byte)value;
            registers[register + 1] = (byte)(value >> 8);
        }

        private void WriteS16LE(int register, short value)
        {
            WriteU16LE(register, (ushort)value);
        }

        private void WriteU24LE(int register, uint value)
        {
            registers[register] = (byte)value;
            registers[register + 1] = (byte)(value >> 8);
            registers[register + 2] = (byte)(value >> 16);
        }

        private readonly byte chipId;
        private readonly byte chipIdRegister;
        private readonly byte[] registers;
        private int transferByte;
        private byte currentRegister;
        private bool reading;

        private const byte ReadFlag = 0x80;
        private const byte RegisterMask = 0x7F;
        private const byte Bmp388ChipId = 0x00;
        private const byte Bmp581ChipId = 0x01;
        private const byte DpsChipId = 0x0D;
        private const byte LpsChipId = 0x0F;
        private const byte Bmp280ChipId = 0x50;
    }

    public class AP_BMP085 : AP_I2CRegisterDevice
    {
        public AP_BMP085(IMachine machine)
        {
            physics = AP_PhysicsState.ForMachine(machine);
            Reset();
        }

        public override void Reset()
        {
            base.Reset();
            Registers[0xD0] = 0x55;
            // A valid, deliberately simple calibration. The production
            // compensation routine is mirrored below to invert live truth.
            WriteS16(0xAA, Ac1);
            WriteS16(0xAC, Ac2);
            WriteS16(0xAE, Ac3);
            WriteU16(0xB0, Ac4);
            WriteU16(0xB2, Ac5);
            WriteU16(0xB4, Ac6);
            WriteS16(0xB6, B1);
            WriteS16(0xB8, B2);
            WriteS16(0xBA, 0);
            WriteS16(0xBC, Mc);
            WriteS16(0xBE, Md);
            pressureSampleNumber = 0;
            SetTemperatureSample();
        }

        protected override void WriteRegister(int register, byte value)
        {
            base.WriteRegister(register, value);
            if(register != 0xF4)
            {
                return;
            }
            if(value == 0x2E)
            {
                SetTemperatureSample();
            }
            else
            {
                SetPressureSample();
            }
        }

        public bool SuppressAdc { get; set; }

        protected override byte ReadRegister(int register)
        {
            if(SuppressAdc && register >= Data && register < Data + 3)
            {
                return 0;
            }
            return base.ReadRegister(register);
        }

        private void SetTemperatureSample()
        {
            var target = (int)Math.Round((physics.Current.TemperatureK - 273.15) * 10.0);
            var low = 0;
            var high = 0xFFFF;
            while(low < high)
            {
                var middle = low + (high - low) / 2;
                if(CompensateTemperature(middle) < target)
                {
                    low = middle + 1;
                }
                else
                {
                    high = middle;
                }
            }
            temperatureRaw = low;
            WriteU16(Data, (ushort)temperatureRaw);
        }

        private void SetPressureSample()
        {
            pressureSampleNumber++;
            var pressure = AP_SensorNoise.Pressure(
                physics.Current, pressureSampleNumber, 0xB085U, 0.5f);
            var low = 0x10000;
            var high = 0x7FFFF;
            while(low < high)
            {
                var middle = low + (high - low) / 2;
                if(CompensatePressure(middle, temperatureRaw) < pressure)
                {
                    low = middle + 1;
                }
                else
                {
                    high = middle;
                }
            }
            var encoded = (uint)low << (8 - Oversampling);
            Registers[Data] = (byte)(encoded >> 16);
            Registers[Data + 1] = (byte)(encoded >> 8);
            Registers[Data + 2] = (byte)encoded;
        }

        private int CompensateTemperature(int rawTemperature)
        {
            var b5 = CalculateB5(rawTemperature);
            return (b5 + 8) >> 4;
        }

        private int CompensatePressure(int rawPressure, int rawTemperature)
        {
            var b5 = CalculateB5(rawTemperature);
            var b6 = b5 - 4000;
            var x1 = (B2 * ((b6 * b6) >> 12)) >> 11;
            var x2 = (Ac2 * b6) >> 11;
            var x3 = x1 + x2;
            var b3 = (((Ac1 * 4 + x3) << Oversampling) + 2) / 4;
            x1 = (Ac3 * b6) >> 13;
            x2 = (B1 * ((b6 * b6) >> 12)) >> 16;
            x3 = (x1 + x2 + 2) >> 2;
            var b4 = (uint)(Ac4 * (x3 + 32768)) >> 15;
            var b7 = unchecked((uint)(rawPressure - b3) *
                               (50000U >> Oversampling));
            var pressure = b7 < 0x80000000U
                ? (int)((b7 * 2U) / b4)
                : (int)((b7 / b4) * 2U);
            x1 = (pressure >> 8) * (pressure >> 8);
            x1 = (x1 * 3038) >> 16;
            x2 = (-7357 * pressure) >> 16;
            return pressure + ((x1 + x2 + 3791) >> 4);
        }

        private int CalculateB5(int rawTemperature)
        {
            var x1 = ((rawTemperature - Ac6) * Ac5) >> 15;
            var x2 = (Mc << 11) / (x1 + Md);
            return x1 + x2;
        }

        private readonly AP_PhysicsState physics;
        private int temperatureRaw;
        private uint pressureSampleNumber;

        private const int Data = 0xF6;
        private const int Oversampling = 3;
        private const short Ac1 = 1000;
        private const short Ac2 = 2;
        private const short Ac3 = 2;
        private const ushort Ac4 = 32768;
        private const ushort Ac5 = 32768;
        private const ushort Ac6 = 20000;
        private const short B1 = 0;
        private const short B2 = 0;
        private const short Mc = 0;
        private const short Md = 1;
    }

    public class AP_BMP581 : AP_I2CRegisterDevice
    {
        public AP_BMP581(IMachine machine)
        {
            physics = AP_PhysicsState.ForMachine(machine);
            Reset();
        }

        public override void Reset()
        {
            base.Reset();
            Registers[0x01] = 0x50;
            Registers[0x11] = 0x02;
            Registers[0x27] = 0x10;
            Registers[0x28] = 0x02;
            dataReadPhase = 0;
            pressureSampleNumber = 0;
            UpdateSample();
        }

        public override byte[] Read(int count = 1)
        {
            if(Pointer == TemperatureData && !SuppressData)
            {
                // The production backend rejects a sample unless two
                // consecutive block reads match exactly.
                if(dataReadPhase == 0)
                {
                    UpdateSample();
                }
                dataReadPhase = (dataReadPhase + 1) % 2;
            }
            return base.Read(count);
        }

        public bool SuppressData { get; set; }

        protected override byte ReadRegister(int register)
        {
            if(SuppressData && register >= TemperatureData &&
                    register < TemperatureData + SampleLength)
            {
                return InvalidData;
            }
            return base.ReadRegister(register);
        }

        private void UpdateSample()
        {
            var truth = physics.Current;
            var temperatureC = truth.TemperatureK - 273.15;
            var temperatureRaw = (int)Math.Round(Math.Max(
                -8388608.0, Math.Min(8388607.0, temperatureC * 65536.0)));
            WriteS24LE(TemperatureData, temperatureRaw);
            pressureSampleNumber++;
            var pressurePa = AP_SensorNoise.Pressure(
                truth, pressureSampleNumber, 0xB581U, 0.5f);
            var pressureRaw = (uint)Math.Round(Math.Max(
                0.0, Math.Min(0xFFFFFF, pressurePa * 64.0)));
            WriteU24LE(PressureData, pressureRaw);
        }

        private void WriteS24LE(int register, int value)
        {
            WriteU24LE(register, (uint)value & 0xFFFFFF);
        }

        private readonly AP_PhysicsState physics;
        private int dataReadPhase;
        private uint pressureSampleNumber;

        private const int TemperatureData = 0x1D;
        private const int PressureData = 0x20;
        private const int SampleLength = 6;
        private const byte InvalidData = 0x7F;
    }

    public class AP_LPS2XH : AP_I2CRegisterDevice
    {
        public AP_LPS2XH(IMachine machine)
        {
            physics = AP_PhysicsState.ForMachine(machine);
            Reset();
        }

        public override void Reset()
        {
            base.Reset();
            Registers[0x0F] = 0xB1;
            Registers[0x27] = 0x03;
            pressureSampleNumber = 0;
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

        private void UpdateSample()
        {
            var truth = physics.Current;
            pressureSampleNumber++;
            var pressure = AP_SensorNoise.Pressure(
                truth, pressureSampleNumber, 0x2A22U, 0.5f);
            WriteU24LE(Pressure, (uint)Math.Round(pressure * 4096.0 / 100.0));
            var temperature = (short)Math.Round((truth.TemperatureK - 273.15) * 100.0);
            Registers[0x2B] = (byte)temperature;
            Registers[0x2C] = (byte)(temperature >> 8);
        }

        private readonly AP_PhysicsState physics;
        private uint pressureSampleNumber;

        private const int Status = 0x27;
        private const int Pressure = 0x28;
    }

    // Keller 4LD--9LD pressure transducer. Calibration and metadata reads use
    // command-specific three-byte replies; measurement requests return the
    // latest absolute pressure and temperature in a five-byte frame.
    public class AP_Baro_KellerLD : II2CPeripheral
    {
        public AP_Baro_KellerLD(IMachine machine)
        {
            physics = AP_PhysicsState.ForMachine(machine);
            Reset();
        }

        public void Reset()
        {
            command = 0;
            pressureSampleNumber = 0;
        }

        public void Write(byte[] data)
        {
            if(data.Length == 0)
            {
                return;
            }
            command = data[0];
        }

        public byte[] Read(int count = 1)
        {
            if(command == RequestMeasurement)
            {
                return ReadMeasurement(count);
            }

            var result = new byte[count];
            if(count == 0)
            {
                return result;
            }
            result[0] = StatusNormal;
            var value = CalibrationWord(command);
            if(count > 1)
            {
                result[1] = (byte)(value >> 8);
            }
            if(count > 2)
            {
                result[2] = (byte)value;
            }
            return result;
        }

        public void FinishTransmission()
        {
        }

        public bool SuppressData { get; set; }

        private byte[] ReadMeasurement(int count)
        {
            var result = new byte[count];
            if(count == 0)
            {
                return result;
            }
            result[0] = SuppressData ? StatusChecksumError : StatusNormal;

            pressureSampleNumber++;
            var pressurePa = AP_SensorNoise.Pressure(
                physics.Current, pressureSampleNumber, 0x4B4CU, 0.5f);
            var pressureRaw = (int)Math.Round(
                16384.0 + pressurePa / PascalPerBar *
                32768.0 / (PressureMaximumBar - PressureMinimumBar));
            pressureRaw = Math.Max(1, Math.Min(0xFFFF, pressureRaw));

            var temperatureC = physics.Current.TemperatureK - 273.15;
            var temperatureRaw = (int)Math.Round(
                ((temperatureC + 50.0) / 0.05 + 24.0) * 16.0);
            temperatureRaw = Math.Max(1, Math.Min(0xFFFF, temperatureRaw));

            if(count > 1)
            {
                result[1] = (byte)(pressureRaw >> 8);
            }
            if(count > 2)
            {
                result[2] = (byte)pressureRaw;
            }
            if(count > 3)
            {
                result[3] = (byte)(temperatureRaw >> 8);
            }
            if(count > 4)
            {
                result[4] = (byte)temperatureRaw;
            }
            return result;
        }

        private static ushort CalibrationWord(byte register)
        {
            switch(register)
            {
            case PressureMinimumMsb:
                return (ushort)(PressureMinimumBits >> 16);
            case PressureMinimumLsb:
                return (ushort)(PressureMinimumBits & 0xFFFF);
            case PressureMaximumMsb:
                return (ushort)(PressureMaximumBits >> 16);
            case PressureMaximumLsb:
                return (ushort)(PressureMaximumBits & 0xFFFF);
            case MetadataPressureMode:
                return PressureModeAbsolute;
            default:
                return 0;
            }
        }

        private readonly AP_PhysicsState physics;
        private byte command;
        private uint pressureSampleNumber;

        private const byte MetadataPressureMode = 0x12;
        private const byte PressureMinimumMsb = 0x13;
        private const byte PressureMinimumLsb = 0x14;
        private const byte PressureMaximumMsb = 0x15;
        private const byte PressureMaximumLsb = 0x16;
        private const byte RequestMeasurement = 0xAC;
        private const byte StatusNormal = 0x40;
        private const byte StatusChecksumError = 0x44;
        private const ushort PressureModeAbsolute = 2;
        private const uint PressureMinimumBits = 0x00000000;
        private const uint PressureMaximumBits = 0x40000000;
        private const double PressureMinimumBar = 0.0;
        private const double PressureMaximumBar = 2.0;
        private const double PascalPerBar = 100000.0;
    }

    // ICP-101xx command protocol. The synthetic OTP constants come from the
    // datasheet conversion example; raw samples invert the same conversion
    // used by the production backend.
    public class AP_ICP101XX : II2CPeripheral
    {
        public AP_ICP101XX(IMachine machine)
            : this(machine, false)
        {
        }

        protected AP_ICP101XX(IMachine machine, bool pressureFirst)
        {
            this.pressureFirst = pressureFirst;
            physics = AP_PhysicsState.ForMachine(machine);
            Reset();
        }

        public void Reset()
        {
            command = 0;
            otpIndex = 0;
            pressureSampleNumber = 0;
            response = null;
            responseOffset = 0;
            OtpReadCount = 0;
            MeasurementCount = 0;
        }

        public void Write(byte[] data)
        {
            if(data.Length < 2)
            {
                return;
            }
            command = (ushort)(data[0] << 8 | data[1]);
            if(command == SoftReset)
            {
                Reset();
            }
            else if(command == SetAddress)
            {
                otpIndex = 0;
            }
        }

        public byte[] Read(int count = 1)
        {
            if(response == null)
            {
                response = BuildResponse();
                responseOffset = 0;
            }
            var result = new byte[count];
            var available = Math.Max(0, response.Length - responseOffset);
            var copied = Math.Min(count, available);
            Array.Copy(response, responseOffset, result, 0, copied);
            responseOffset += copied;
            return result;
        }

        public void FinishTransmission()
        {
            response = null;
            responseOffset = 0;
        }

        public bool SuppressData { get; set; }
        public uint OtpReadCount { get; private set; }
        public uint MeasurementCount { get; private set; }

        private byte[] BuildResponse()
        {
            switch(command)
            {
            case ReadId:
                return new byte[] { 0, ProductId };
            case ReadOtp:
                return ReadOtpWord();
            case MeasureLowPower:
            case MeasureNormal:
            case MeasureLowNoise:
            case MeasureUltraLowNoise:
            case MeasurePressureFirst:
                return ReadMeasurement();
            default:
                return new byte[0];
            }
        }

        private byte[] ReadOtpWord()
        {
            var value = OtpConstants[Math.Min(otpIndex,
                                              OtpConstants.Length - 1)];
            otpIndex++;
            OtpReadCount++;
            var first = (byte)(value >> 8);
            var second = (byte)value;
            return new byte[] { first, second, Crc(first, second) };
        }

        private byte[] ReadMeasurement()
        {
            MeasurementCount++;
            if(SuppressData)
            {
                return new byte[9];
            }

            var truth = physics.Current;
            var temperatureC = truth.TemperatureK - 273.15;
            var temperatureRaw = (int)Math.Round(
                (temperatureC + 45.0) * 65536.0 / 175.0);
            temperatureRaw = Math.Max(0, Math.Min(0xFFFF, temperatureRaw));

            pressureSampleNumber++;
            var pressure = AP_SensorNoise.Pressure(
                truth, pressureSampleNumber, 0x1C10U, 0.5f);
            double a;
            double b;
            double c;
            CalculateConversionConstants(temperatureRaw, out a, out b, out c);
            var pressureRaw = (int)Math.Round(b / (pressure - a) - c);
            pressureRaw = Math.Max(0, Math.Min(0xFFFFFF, pressureRaw));

            var sample = new byte[9];
            if(pressureFirst)
            {
                StoreThreeByteWord(sample, 0, pressureRaw);
                StoreTwoByteWord(sample, 6, temperatureRaw);
            }
            else
            {
                StoreTwoByteWord(sample, 0, temperatureRaw);
                StoreThreeByteWord(sample, 3, pressureRaw);
            }
            return sample;
        }

        private static void StoreTwoByteWord(byte[] sample, int offset,
                                              int value)
        {
            sample[offset] = (byte)(value >> 8);
            sample[offset + 1] = (byte)value;
            sample[offset + 2] = Crc(sample[offset], sample[offset + 1]);
        }

        private static void StoreThreeByteWord(byte[] sample, int offset,
                                                int value)
        {
            sample[offset] = (byte)(value >> 16);
            sample[offset + 1] = (byte)(value >> 8);
            sample[offset + 2] = Crc(sample[offset], sample[offset + 1]);
            sample[offset + 3] = (byte)value;
            sample[offset + 4] = 0;
            sample[offset + 5] = Crc(sample[offset + 3], sample[offset + 4]);
        }

        private static void CalculateConversionConstants(
            int temperatureRaw, out double a, out double b, out double c)
        {
            var t = temperatureRaw - 32768.0;
            var square = t * t / 16777216.0;
            var s0 = 3.5 * (1 << 20) + OtpConstants[0] * square;
            var s1 = 2048.0 * OtpConstants[3] + OtpConstants[1] * square;
            var s2 = 11.5 * (1 << 20) + OtpConstants[2] * square;
            c = (s0 * s1 * (PressureCalibration[0] - PressureCalibration[1]) +
                 s1 * s2 * (PressureCalibration[1] - PressureCalibration[2]) +
                 s2 * s0 * (PressureCalibration[2] - PressureCalibration[0])) /
                (s2 * (PressureCalibration[0] - PressureCalibration[1]) +
                 s0 * (PressureCalibration[1] - PressureCalibration[2]) +
                 s1 * (PressureCalibration[2] - PressureCalibration[0]));
            a = (PressureCalibration[0] * s0 - PressureCalibration[1] * s1 -
                 (PressureCalibration[1] - PressureCalibration[0]) * c) /
                (s0 - s1);
            b = (PressureCalibration[0] - a) * (s0 + c);
        }

        private static byte Crc(byte first, byte second)
        {
            var crc = (byte)0xFF;
            foreach(var initial in new byte[] { first, second })
            {
                var value = initial;
                for(var bit = 0; bit < 8; bit++)
                {
                    var xor = ((crc & 0x80) ^ (value & 0x80)) != 0;
                    crc = (byte)(crc << 1);
                    if(xor)
                    {
                        crc ^= 0x31;
                    }
                    value = (byte)(value << 1);
                }
            }
            return crc;
        }

        private readonly AP_PhysicsState physics;
        private readonly bool pressureFirst;
        private int otpIndex;
        private ushort command;
        private uint pressureSampleNumber;
        private byte[] response;
        private int responseOffset;

        private static readonly short[] OtpConstants = { 1000, 2000, 3000, 4000 };
        private static readonly double[] PressureCalibration = { 45000, 80000, 105000 };

        private const byte ProductId = 0x08;
        private const ushort ReadId = 0xEFC8;
        private const ushort SetAddress = 0xC595;
        private const ushort ReadOtp = 0xC7F7;
        private const ushort MeasureLowPower = 0x609C;
        private const ushort MeasureNormal = 0x6825;
        private const ushort MeasureLowNoise = 0x70DF;
        private const ushort MeasureUltraLowNoise = 0x7866;
        private const ushort MeasurePressureFirst = 0x5059;
        private const ushort SoftReset = 0x805D;
    }

    public class AP_ICM20789Barometer : AP_ICP101XX
    {
        public AP_ICM20789Barometer(IMachine machine)
            : base(machine, true)
        {
        }
    }

    public class AP_ICP201XX : AP_I2CRegisterDevice
    {
        public AP_ICP201XX(IMachine machine)
        {
            physics = AP_PhysicsState.ForMachine(machine);
            Reset();
        }

        public override void Reset()
        {
            base.Reset();
            Registers[0x0C] = 0x63;
            Registers[0xD3] = 0xB2;
            Registers[0xBF] = 0x01;
            Registers[0xCD] = 0x01;
            pressureSampleNumber = 0;
            UpdateSample();
        }

        public override byte[] Read(int count = 1)
        {
            if(Pointer == FifoBase && !SuppressData)
            {
                UpdateSample();
            }
            return base.Read(count);
        }

        public bool SuppressData { get; set; }

        protected override byte ReadRegister(int register)
        {
            if(register == FifoFill)
            {
                return SuppressData ? (byte)0 : (byte)1;
            }
            if(SuppressData && register >= FifoBase)
            {
                return 0;
            }
            return base.ReadRegister(register);
        }

        private void UpdateSample()
        {
            var truth = physics.Current;
            pressureSampleNumber++;
            var pressurePa = AP_SensorNoise.Pressure(
                truth, pressureSampleNumber, 0x1C20U, 0.5f);
            var pressureRaw = (int)Math.Round(
                (pressurePa * 0.001 - 70.0) * 131072.0 / 40.0);
            var temperatureRaw = (int)Math.Round(
                (truth.TemperatureK - 273.15 - 25.0) * 262144.0 / 65.0);
            WriteSigned20LE(FifoBase, pressureRaw);
            WriteSigned20LE(FifoBase + 3, temperatureRaw);
        }

        private void WriteSigned20LE(int register, int value)
        {
            value = Math.Max(-0x80000, Math.Min(0x7FFFF, value));
            Registers[register] = (byte)value;
            Registers[register + 1] = (byte)(value >> 8);
            Registers[register + 2] = (byte)((value >> 16) & 0x0F);
        }

        private readonly AP_PhysicsState physics;
        private uint pressureSampleNumber;

        private const int FifoFill = 0xC4;
        private const int FifoBase = 0xFA;
    }
}
