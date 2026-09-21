// Register-oriented IMUs used by ArduPilot. The I2C MPU6000 model supplies
// physics-driven FIFO samples; the generic SPI model below supplies identity,
// reset, checked-register readback and ready/status behavior for Bosch
// BMI055/BMI088/BMI160/BMI270 and ST LSM6DSV devices.
//
using System;
using System.Collections.Generic;
using Antmicro.Renode.Core;
using Antmicro.Renode.Peripherals;
using Antmicro.Renode.Peripherals.I2C;
using Antmicro.Renode.Peripherals.Miscellaneous;
using Antmicro.Renode.Peripherals.SPI;

namespace Antmicro.Renode.Peripherals.Sensors
{
    public class AP_SCHA63T : ISPIPeripheral, IGPIOReceiver
    {
        public byte Transmit(byte value)
        {
            if(transferByte == 0)
            {
                register = (byte)((value >> 2) & 0x1F);
                response1 = register == SummaryStatus ? (byte)0x98 : (byte)0;
                response2 = register == SummaryStatus ? (byte)0x02 : (byte)0;
            }
            byte result;
            switch(transferByte)
            {
            case 1:
                result = response1;
                break;
            case 2:
                result = response2;
                break;
            case 3:
                result = Crc(0, response1, response2);
                break;
            default:
                result = 0;
                break;
            }
            transferByte++;
            return result;
        }

        public void Reset()
        {
            transferByte = 0;
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

        private static byte Crc(byte first, byte second, byte third)
        {
            byte crc = 0xFF;
            foreach(var value in new[] { first, second, third })
            {
                crc ^= value;
                for(var bit = 0; bit < 8; bit++)
                {
                    crc = (byte)((crc & 0x80) != 0 ? (crc << 1) ^ 0x1D : crc << 1);
                }
            }
            return (byte)(crc ^ 0xFF);
        }

        private int transferByte;
        private byte register;
        private byte response1;
        private byte response2;

        private const byte SummaryStatus = 0x0E;
    }

    public class AP_I2CRegisterIMU : II2CPeripheral
    {
        public AP_I2CRegisterIMU(IMachine machine, byte whoAmI = 0x68,
            byte rotation = 0, double temperatureZeroC = 36.53,
            double temperatureSensitivity = 1.0 / 340.0)
        {
            this.whoAmI = whoAmI;
            this.rotation = rotation;
            this.temperatureZeroC = temperatureZeroC;
            this.temperatureSensitivity = temperatureSensitivity;
            physics = AP_PhysicsState.ForMachine(machine);
            fifo = new Queue<byte>();
            registers = new byte[256];
            Reset();
        }

        public bool SuppressData { get; set; }

        public void Reset()
        {
            Array.Clear(registers, 0, registers.Length);
            fifo.Clear();
            registers[ProductId] = 0x01;
            registers[WhoAmI] = whoAmI;
            pointer = 0;
        }

        public void Write(byte[] data)
        {
            if(data.Length == 0)
            {
                return;
            }
            pointer = data[0];
            for(var index = 1; index < data.Length; index++)
            {
                var register = (byte)pointer;
                WriteRegister(register, data[index]);
                pointer = (byte)(register + 1);
            }
        }

        public byte[] Read(int count = 1)
        {
            var result = new byte[count];
            for(var index = 0; index < count; index++)
            {
                var register = (byte)pointer;
                result[index] = ReadRegister(register);
                if(register != FifoData)
                {
                    pointer = (byte)(register + 1);
                }
            }
            return result;
        }

        public void FinishTransmission()
        {
        }

        private byte ReadRegister(byte register)
        {
            switch(register)
            {
            case FifoCountHigh:
                fifo.Clear();
                if(!SuppressData && (registers[UserControl] & FifoEnable) != 0)
                {
                    QueueSample();
                }
                return (byte)(fifo.Count >> 8);
            case FifoCountLow:
                return (byte)(fifo.Count & 0xFF);
            case FifoData:
                return fifo.Count > 0 ? fifo.Dequeue() : (byte)0;
            case InterruptStatus:
                return SuppressData ? (byte)0 : DataReady;
            case TemperatureHigh:
                return (byte)((TemperatureRaw() >> 8) & 0xFF);
            case TemperatureLow:
                return (byte)(TemperatureRaw() & 0xFF);
            default:
                return registers[register];
            }
        }

        private void WriteRegister(byte register, byte value)
        {
            switch(register)
            {
            case PowerManagement:
                if((value & DeviceReset) != 0)
                {
                    Reset();
                    return;
                }
                registers[register] = value;
                break;
            case UserControl:
                if((value & FifoReset) != 0)
                {
                    fifo.Clear();
                }
                registers[register] = (byte)(value & ~FifoReset);
                break;
            case FifoData:
                break;
            default:
                registers[register] = value;
                break;
            }
        }

        private void QueueSample()
        {
            var truth = physics.Current;
            var acceleration = AP_SensorOrientation.BodyToSensor(
                truth.SpecificForceMS2, rotation);
            var gyro = AP_SensorOrientation.BodyToSensor(
                truth.GyroRadS, rotation);

            // The production backend maps packet words to sensor axes as
            // (word1, word0, -word2), for acceleration and gyro.
            PushWord(ScaleWord(acceleration[1], AccelScale));
            PushWord(ScaleWord(acceleration[0], AccelScale));
            PushWord(ScaleWord(-acceleration[2], AccelScale));
            PushWord(TemperatureRaw());
            PushWord(ScaleWord(gyro[1], GyroScale));
            PushWord(ScaleWord(gyro[0], GyroScale));
            PushWord(ScaleWord(-gyro[2], GyroScale));
        }

        private short TemperatureRaw()
        {
            var temperatureC = physics.Current.TemperatureK - 273.15;
            return ScaleWord(temperatureC - temperatureZeroC,
                temperatureSensitivity);
        }

        private static short ScaleWord(double value, double scale)
        {
            return (short)Math.Max(Int16.MinValue,
                Math.Min(Int16.MaxValue, Math.Round(value / scale)));
        }

        private void PushWord(short value)
        {
            fifo.Enqueue((byte)((value >> 8) & 0xFF));
            fifo.Enqueue((byte)(value & 0xFF));
        }

        private readonly byte whoAmI;
        private readonly byte rotation;
        private readonly double temperatureZeroC;
        private readonly double temperatureSensitivity;
        private readonly AP_PhysicsState physics;
        private readonly Queue<byte> fifo;
        private readonly byte[] registers;
        private int pointer;

        private const byte ProductId = 0x0C;
        private const byte InterruptStatus = 0x3A;
        private const byte TemperatureHigh = 0x41;
        private const byte TemperatureLow = 0x42;
        private const byte UserControl = 0x6A;
        private const byte PowerManagement = 0x6B;
        private const byte FifoCountHigh = 0x72;
        private const byte FifoCountLow = 0x73;
        private const byte FifoData = 0x74;
        private const byte WhoAmI = 0x75;

        private const byte DataReady = 0x01;
        private const byte FifoReset = 0x04;
        private const byte FifoEnable = 0x40;
        private const byte DeviceReset = 0x80;
        private const double Gravity = 9.80665;
        private const double AccelScale = Gravity / 2048.0;
        private const double GyroScale = Math.PI / 180.0 / 16.4;
    }

    public class AP_ICM20789I2CIMU : AP_I2CRegisterIMU
    {
        public AP_ICM20789I2CIMU(IMachine machine, byte rotation = 0)
            : base(machine, 0x03, rotation, 25.0, 0.003)
        {
        }
    }

    public class AP_BMI160I2C : II2CPeripheral
    {
        public AP_BMI160I2C(IMachine machine, byte rotation = 0)
        {
            this.rotation = rotation;
            physics = AP_PhysicsState.ForMachine(machine);
            fifo = new Queue<byte>();
            registers = new byte[256];
            Reset();
        }

        public bool SuppressData { get; set; }

        public void Reset()
        {
            Array.Clear(registers, 0, registers.Length);
            fifo.Clear();
            registers[ChipId] = ChipIdValue;
            pointer = 0;
        }

        public void Write(byte[] data)
        {
            if(data.Length == 0)
            {
                return;
            }
            pointer = (byte)(data[0] & RegisterMask);
            for(var index = 1; index < data.Length; index++)
            {
                var register = pointer;
                WriteRegister(register, data[index]);
                pointer = (byte)(register + 1);
            }
        }

        public byte[] Read(int count = 1)
        {
            var result = new byte[count];
            for(var index = 0; index < count; index++)
            {
                var register = pointer;
                result[index] = ReadRegister(register);
                if(register != FifoData)
                {
                    pointer = (byte)(register + 1);
                }
            }
            return result;
        }

        public void FinishTransmission()
        {
        }

        private byte ReadRegister(byte register)
        {
            switch(register)
            {
            case FifoLengthLow:
                fifo.Clear();
                if(!SuppressData && (registers[FifoConfig] & FifoSensors) != 0)
                {
                    QueueSample();
                }
                return (byte)(fifo.Count & 0xFF);
            case FifoLengthHigh:
                return (byte)(fifo.Count >> 8);
            case FifoData:
                return fifo.Count > 0 ? fifo.Dequeue() : (byte)0;
            default:
                return registers[register];
            }
        }

        private void WriteRegister(byte register, byte value)
        {
            if(register == Command && value == SoftwareReset)
            {
                Reset();
                return;
            }
            if(register == Command && value == FifoFlush)
            {
                fifo.Clear();
            }
            registers[register] = value;
        }

        private void QueueSample()
        {
            var truth = physics.Current;
            var acceleration = AP_SensorOrientation.BodyToSensor(
                truth.SpecificForceMS2, rotation);
            var gyro = AP_SensorOrientation.BodyToSensor(
                truth.GyroRadS, rotation);
            foreach(var value in gyro)
            {
                PushInt16(ScaleWord(value, GyroScale));
            }
            foreach(var value in acceleration)
            {
                PushInt16(ScaleWord(value, AccelScale));
            }
        }

        private static short ScaleWord(double value, double scale)
        {
            return (short)Math.Max(Int16.MinValue,
                Math.Min(Int16.MaxValue, Math.Round(value / scale)));
        }

        private void PushInt16(short value)
        {
            fifo.Enqueue((byte)(value & 0xFF));
            fifo.Enqueue((byte)((value >> 8) & 0xFF));
        }

        private readonly byte rotation;
        private readonly AP_PhysicsState physics;
        private readonly Queue<byte> fifo;
        private readonly byte[] registers;
        private byte pointer;

        private const byte ChipId = 0x00;
        private const byte FifoLengthLow = 0x22;
        private const byte FifoLengthHigh = 0x23;
        private const byte FifoData = 0x24;
        private const byte FifoConfig = 0x47;
        private const byte Command = 0x7E;
        private const byte ChipIdValue = 0xD1;
        private const byte FifoSensors = 0xC0;
        private const byte SoftwareReset = 0xB6;
        private const byte FifoFlush = 0xB0;
        private const byte RegisterMask = 0x7F;
        private const double Gravity = 9.80665;
        private const double AccelScale = Gravity / 2048.0;
        private const double GyroScale = Math.PI / 180.0 / 16.384;
    }

    public class AP_BMI270I2C : II2CPeripheral
    {
        public AP_BMI270I2C(IMachine machine, byte rotation = 0)
        {
            this.rotation = rotation;
            physics = AP_PhysicsState.ForMachine(machine);
            fifo = new Queue<byte>();
            registers = new byte[256];
            Reset();
        }

        public bool SuppressData { get; set; }

        public void Reset()
        {
            Array.Clear(registers, 0, registers.Length);
            fifo.Clear();
            registers[ChipId] = ChipIdValue;
            pointer = 0;
            readDummyBytes = 0;
            configurationUploaded = false;
        }

        public void Write(byte[] data)
        {
            if(data.Length == 0)
            {
                return;
            }

            pointer = (byte)(data[0] & RegisterMask);
            if((data[0] & ReadFlag) != 0)
            {
                // The production backend uses transfer_fullduplex() for both
                // SPI and I2C and discards two leading response bytes.
                readDummyBytes = 2;
                return;
            }

            if(pointer == InitData && data.Length > 2)
            {
                configurationUploaded = true;
                return;
            }

            for(var index = 1; index < data.Length; index++)
            {
                var register = pointer;
                WriteRegister(register, data[index]);
                pointer = (byte)(register + 1);
            }
        }

        public byte[] Read(int count = 1)
        {
            var result = new byte[count];
            for(var index = 0; index < count; index++)
            {
                if(readDummyBytes > 0)
                {
                    readDummyBytes--;
                    continue;
                }

                var register = pointer;
                result[index] = ReadRegister(register);
                if(register != FifoData)
                {
                    pointer = (byte)(register + 1);
                }
            }
            return result;
        }

        public void FinishTransmission()
        {
        }

        private byte ReadRegister(byte register)
        {
            switch(register)
            {
            case FifoLengthLow:
                fifo.Clear();
                if(!SuppressData &&
                    (registers[FifoConfig] & FifoSensors) == FifoSensors)
                {
                    QueueSample();
                }
                return (byte)(fifo.Count & 0xFF);
            case FifoLengthHigh:
                return (byte)(fifo.Count >> 8);
            case FifoData:
                return fifo.Count > 0 ? fifo.Dequeue() : (byte)0;
            case InternalStatus:
                return configurationUploaded && registers[InitControl] == 1
                    ? InitOk : (byte)0;
            case TemperatureLow:
                return (byte)(TemperatureRaw() & 0xFF);
            case TemperatureHigh:
                return (byte)((TemperatureRaw() >> 8) & 0xFF);
            default:
                return registers[register];
            }
        }

        private void WriteRegister(byte register, byte value)
        {
            if(register == Command && value == SoftwareReset)
            {
                Reset();
                return;
            }
            if(register == Command && value == FifoFlush)
            {
                fifo.Clear();
            }
            registers[register] = value;
        }

        private void QueueSample()
        {
            var truth = physics.Current;
            var acceleration = AP_SensorOrientation.BodyToSensor(
                truth.SpecificForceMS2, rotation);
            var gyro = AP_SensorOrientation.BodyToSensor(
                truth.GyroRadS, rotation);
            fifo.Enqueue(CombinedFrame);
            foreach(var value in gyro)
            {
                PushInt16(ScaleWord(value, GyroScale));
            }
            foreach(var value in acceleration)
            {
                PushInt16(ScaleWord(value, AccelScale));
            }
        }

        private short TemperatureRaw()
        {
            var temperatureC = physics.Current.TemperatureK - 273.15;
            return ScaleWord(temperatureC - TemperatureZeroC,
                TemperatureScale);
        }

        private static short ScaleWord(double value, double scale)
        {
            return (short)Math.Max(Int16.MinValue,
                Math.Min(Int16.MaxValue, Math.Round(value / scale)));
        }

        private void PushInt16(short value)
        {
            fifo.Enqueue((byte)(value & 0xFF));
            fifo.Enqueue((byte)((value >> 8) & 0xFF));
        }

        private readonly byte rotation;
        private readonly AP_PhysicsState physics;
        private readonly Queue<byte> fifo;
        private readonly byte[] registers;
        private byte pointer;
        private byte readDummyBytes;
        private bool configurationUploaded;

        private const byte ChipId = 0x00;
        private const byte InternalStatus = 0x21;
        private const byte TemperatureLow = 0x22;
        private const byte TemperatureHigh = 0x23;
        private const byte FifoLengthLow = 0x24;
        private const byte FifoLengthHigh = 0x25;
        private const byte FifoData = 0x26;
        private const byte FifoConfig = 0x49;
        private const byte InitControl = 0x59;
        private const byte InitData = 0x5E;
        private const byte Command = 0x7E;
        private const byte ChipIdValue = 0x24;
        private const byte CombinedFrame = 0x8C;
        private const byte FifoSensors = 0xC0;
        private const byte InitOk = 0x01;
        private const byte SoftwareReset = 0xB6;
        private const byte FifoFlush = 0xB0;
        private const byte ReadFlag = 0x80;
        private const byte RegisterMask = 0x7F;
        private const double Gravity = 9.80665;
        private const double AccelScale = Gravity / 2048.0;
        private const double GyroScale = Math.PI / 180.0 * 2000.0 / 32767.0;
        private const double TemperatureZeroC = 23.0;
        private const double TemperatureScale = 0.002;
    }

    public class AP_InvensenseV2I2C : II2CPeripheral
    {
        public AP_InvensenseV2I2C(IMachine machine, byte rotation = 0)
        {
            this.rotation = rotation;
            physics = AP_PhysicsState.ForMachine(machine);
            fifo = new Queue<byte>();
            registers = new byte[BankCount, 256];
            Reset();
        }

        public bool SuppressData { get; set; }

        public void Reset()
        {
            Array.Clear(registers, 0, registers.Length);
            fifo.Clear();
            registers[0, WhoAmI] = WhoAmIValue;
            bank = 0;
            pointer = 0;
        }

        public void Write(byte[] data)
        {
            if(data.Length == 0)
            {
                return;
            }

            pointer = data[0];
            for(var index = 1; index < data.Length; index++)
            {
                var register = pointer;
                WriteRegister(register, data[index]);
                pointer = (byte)(register + 1);
            }
        }

        public byte[] Read(int count = 1)
        {
            var result = new byte[count];
            for(var index = 0; index < count; index++)
            {
                var register = pointer;
                result[index] = ReadRegister(register);
                if(register != FifoData)
                {
                    pointer = (byte)(register + 1);
                }
            }
            return result;
        }

        public void FinishTransmission()
        {
        }

        private byte ReadRegister(byte register)
        {
            if(register == BankSelect)
            {
                return (byte)(bank << 4);
            }
            if(bank != 0)
            {
                return registers[bank, register];
            }

            switch(register)
            {
            case FifoCountHigh:
                fifo.Clear();
                if(!SuppressData &&
                    (registers[0, UserControl] & FifoEnable) != 0 &&
                    (registers[0, FifoEnable2] & FifoSensors) == FifoSensors)
                {
                    QueueSample();
                }
                return (byte)(fifo.Count >> 8);
            case FifoCountLow:
                return (byte)(fifo.Count & 0xFF);
            case FifoData:
                return fifo.Count > 0 ? fifo.Dequeue() : (byte)0;
            case InterruptStatus:
                return SuppressData ? (byte)0 : DataReady;
            case TemperatureHigh:
                return (byte)((TemperatureRaw() >> 8) & 0xFF);
            case TemperatureLow:
                return (byte)(TemperatureRaw() & 0xFF);
            default:
                return registers[bank, register];
            }
        }

        private void WriteRegister(byte register, byte value)
        {
            if(register == BankSelect)
            {
                bank = (byte)((value >> 4) & (BankCount - 1));
                return;
            }
            if(bank == 0 && register == PowerManagement &&
                (value & DeviceReset) != 0)
            {
                Reset();
                return;
            }
            if(bank == 0 && register == FifoReset && value != 0)
            {
                fifo.Clear();
            }
            registers[bank, register] = value;
        }

        private void QueueSample()
        {
            var truth = physics.Current;
            var acceleration = AP_SensorOrientation.BodyToSensor(
                truth.SpecificForceMS2, rotation);
            var gyro = AP_SensorOrientation.BodyToSensor(
                truth.GyroRadS, rotation);

            // The production backend maps FIFO words to sensor axes as
            // (word1, word0, -word2), for acceleration and gyro.
            PushInt16(ScaleWord(acceleration[1], AccelScale));
            PushInt16(ScaleWord(acceleration[0], AccelScale));
            PushInt16(ScaleWord(-acceleration[2], AccelScale));
            PushInt16(ScaleWord(gyro[1], GyroScale));
            PushInt16(ScaleWord(gyro[0], GyroScale));
            PushInt16(ScaleWord(-gyro[2], GyroScale));
            PushInt16(TemperatureRaw());
        }

        private short TemperatureRaw()
        {
            var temperatureC = physics.Current.TemperatureK - 273.15;
            return ScaleWord(temperatureC - TemperatureZeroC,
                TemperatureScale);
        }

        private static short ScaleWord(double value, double scale)
        {
            return (short)Math.Max(Int16.MinValue,
                Math.Min(Int16.MaxValue, Math.Round(value / scale)));
        }

        private void PushInt16(short value)
        {
            fifo.Enqueue((byte)((value >> 8) & 0xFF));
            fifo.Enqueue((byte)(value & 0xFF));
        }

        private readonly byte rotation;
        private readonly AP_PhysicsState physics;
        private readonly Queue<byte> fifo;
        private readonly byte[,] registers;
        private byte bank;
        private byte pointer;

        private const byte BankCount = 4;
        private const byte WhoAmI = 0x00;
        private const byte UserControl = 0x03;
        private const byte PowerManagement = 0x06;
        private const byte InterruptStatus = 0x1A;
        private const byte TemperatureHigh = 0x39;
        private const byte TemperatureLow = 0x3A;
        private const byte FifoEnable2 = 0x67;
        private const byte FifoReset = 0x68;
        private const byte FifoCountHigh = 0x70;
        private const byte FifoCountLow = 0x71;
        private const byte FifoData = 0x72;
        private const byte BankSelect = 0x7F;
        private const byte WhoAmIValue = 0xEA;
        private const byte FifoEnable = 0x40;
        private const byte FifoSensors = 0x1F;
        private const byte DeviceReset = 0x80;
        private const byte DataReady = 0x01;
        private const double Gravity = 9.80665;
        private const double AccelScale = Gravity / 2048.0;
        private const double GyroScale = Math.PI / 180.0 / 16.4;
        private const double TemperatureZeroC = 21.0;
        private const double TemperatureScale = 1.0 / 333.87;
    }

    public class AP_RegisterIMU : ISPIPeripheral, IGPIOReceiver
    {
        public AP_RegisterIMU(byte whoAmI)
        {
            this.whoAmI = whoAmI;
            registers = new byte[RegisterCount];
            Reset();
        }

        public void Reset()
        {
            Array.Clear(registers, 0, registers.Length);
            registers[WhoAmI] = whoAmI;
            registers[LsmWhoAmI] = whoAmI;
            registers[Bmi270InternalStatus] = Bmi270InitOk;
            transferByte = 0;
            currentRegister = 0;
            reading = false;
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
                var result = ReadRegister(currentRegister);
                currentRegister++;
                return result;
            }

            WriteRegister(currentRegister, value);
            currentRegister++;
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

        private byte ReadRegister(byte register)
        {
            switch(register)
            {
            case BmiStatus:
                return BmiAccelReady | BmiGyroReady;
            case LsmStatus:
                return LsmAccelReady | LsmGyroReady | LsmTemperatureReady;
            case Bmi270InternalStatus:
                return Bmi270InitOk;
            default:
                return registers[register];
            }
        }

        private void WriteRegister(byte register, byte value)
        {
            // Reset commands self-clear. Preserve the immutable identity and
            // make the BMI270 configuration-loader completion visible.
            if((register == BoschCommand && value == BoschSoftReset) ||
               (register == LsmControl3 && (value & LsmSoftwareReset) != 0))
            {
                Reset();
                return;
            }
            if(register == Bmi270InitControl && value != 0)
            {
                registers[Bmi270InternalStatus] = Bmi270InitOk;
            }
            registers[register] = value;
        }

        private readonly byte whoAmI;
        private readonly byte[] registers;
        private int transferByte;
        private byte currentRegister;
        private bool reading;

        private const int RegisterCount = 256;
        private const byte ReadFlag = 0x80;
        private const byte RegisterMask = 0x7F;
        private const byte WhoAmI = 0x00;
        private const byte LsmWhoAmI = 0x0F;
        private const byte BmiStatus = 0x03;
        private const byte Bmi270InternalStatus = 0x21;
        private const byte Bmi270InitControl = 0x59;
        private const byte LsmControl3 = 0x12;
        private const byte LsmStatus = 0x1E;
        private const byte BoschCommand = 0x7E;
        private const byte BoschSoftReset = 0xB6;
        private const byte LsmSoftwareReset = 0x01;
        private const byte BmiAccelReady = 0x80;
        private const byte BmiGyroReady = 0x40;
        private const byte LsmAccelReady = 0x01;
        private const byte LsmGyroReady = 0x02;
        private const byte LsmTemperatureReady = 0x04;
        private const byte Bmi270InitOk = 0x01;
    }
}
