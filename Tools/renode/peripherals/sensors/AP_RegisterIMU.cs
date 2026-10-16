// Register-oriented IMUs used by ArduPilot, with physics-driven samples:
// I2C MPU6000/ICM20789/BMI160/BMI270/ICM20948 models, and an SPI model for
// Bosch BMI055/BMI088/BMI160/BMI270 and ST LSM9DS0/LSM6DSV devices.
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
    // Murata SCHA63T UNO (accel, X gyro) or DUE (Y/Z gyro) die. Responses are
    // off-frame: each 32-bit frame returns the reply to the previous command.
    public class AP_SCHA63T : ISPIPeripheral, IGPIOReceiver
    {
        public AP_SCHA63T(IMachine machine, byte rotation = 0)
        {
            this.rotation = rotation;
            physics = AP_PhysicsState.ForMachine(machine);
            thermal = AP_BoardThermalState.ForMachine(machine);
        }

        // the second hwdef device is the DUE die
        public bool Due { get; set; }

        public byte Transmit(byte value)
        {
            if(transferByte == 0)
            {
                current = pending;
                pending = Response((byte)((value >> 2) & 0x1F));
            }
            byte result;
            switch(transferByte)
            {
            case 0:
            case 1:
            case 2:
                result = current[transferByte];
                break;
            case 3:
                result = Crc(current[0], current[1], current[2]);
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
            current = new byte[3];
            pending = new byte[3];
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

        private byte[] Response(byte register)
        {
            short data = 0;
            switch(register)
            {
            case SummaryStatus:
                return new byte[] { (byte)(register << 2), 0x98, 0x02 };
            case RateXZ:
                // the backend negates Z to change handedness
                data = Due ? Scale(-Gyro()[2], GyroScale) : Scale(Gyro()[0], GyroScale);
                break;
            case RateY:
                data = Scale(Gyro()[1], GyroScale);
                break;
            case AccelX:
            case AccelY:
                data = Scale(Accel()[register - AccelX], AccelScale);
                break;
            case AccelZ:
                data = Scale(-Accel()[2], AccelScale);
                break;
            case Temperature:
                // inverse of the backend's 0.5 * (25 + raw / 30) + 23 degC
                data = Scale((float)(((thermal.TemperatureC - 23.0) * 2.0 - 25.0) * 30.0), 1.0);
                break;
            }
            return new byte[] { (byte)(register << 2), (byte)(data >> 8), (byte)data };
        }

        private float[] Accel()
        {
            return AP_SensorOrientation.BodyToSensor(physics.Current.SpecificForceMS2, rotation);
        }

        private float[] Gyro()
        {
            return AP_SensorOrientation.BodyToSensor(physics.Current.GyroRadS, rotation);
        }

        private static short Scale(float value, double scale)
        {
            return (short)Math.Max(Int16.MinValue, Math.Min(Int16.MaxValue, Math.Round(value / scale)));
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

        private readonly byte rotation;
        private readonly AP_PhysicsState physics;
        private readonly AP_BoardThermalState thermal;
        private int transferByte;
        private byte[] current = new byte[3];
        private byte[] pending = new byte[3];

        private const byte RateXZ = 0x01;
        private const byte RateY = 0x03;
        private const byte AccelX = 0x04;
        private const byte AccelY = 0x05;
        private const byte AccelZ = 0x06;
        private const byte Temperature = 0x07;
        private const byte SummaryStatus = 0x0E;
        private const double AccelScale = 9.80665 / 4905.0;
        private const double GyroScale = Math.PI / 180.0 / 80.0;
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
            thermal = AP_BoardThermalState.ForMachine(machine);
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
            var temperatureC = thermal.TemperatureC;
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
        private readonly AP_BoardThermalState thermal;
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
            thermal = AP_BoardThermalState.ForMachine(machine);
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
            var temperatureC = thermal.TemperatureC;
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
        private readonly AP_BoardThermalState thermal;
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
            thermal = AP_BoardThermalState.ForMachine(machine);
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
            var temperatureC = thermal.TemperatureC;
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
        private readonly AP_BoardThermalState thermal;
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

    // SPI Bosch BMI055/BMI088 (accel and gyro dies), BMI160, BMI270 and ST
    // LSM9DS0 (gyro and accel/mag dies) and LSM6DSV, selected by whoAmI, on
    // SPI or, for the Bosch dies, I2C.
    // Each FIFO or status poll latches one physics sample in the register
    // and FIFO layout the ArduPilot backend decodes.
    public class AP_RegisterIMU : ISPIPeripheral, II2CPeripheral, IGPIOReceiver
    {
        // the generator passes every IMU its hwdef rotation
        public AP_RegisterIMU(IMachine machine, byte whoAmI, byte rotation = 0)
        {
            this.whoAmI = whoAmI;
            this.rotation = rotation;
            chip = ChipFor(whoAmI);
            physics = AP_PhysicsState.ForMachine(machine);
            thermal = AP_BoardThermalState.ForMachine(machine);
            registers = new byte[RegisterCount];
            fifo = new Queue<byte>();
            Reset();
        }

        // rotation of the LSM303D magnetometer in the LSM9DS0 accel/mag die
        public byte CompassRotation { get; set; }

        public void Reset()
        {
            Array.Clear(registers, 0, registers.Length);
            fifo.Clear();
            registers[IsLsm ? LsmWhoAmI : WhoAmI] = whoAmI;
            transferByte = 0;
            currentRegister = 0;
            reading = false;
            skipDummy = false;
        }

        public byte Transmit(byte value)
        {
            if(transferByte++ == 0)
            {
                reading = (value & ReadFlag) != 0;
                currentRegister = (byte)(value & AddressMask);
                // BMI088 accel and BMI270 SPI reads return a dummy byte first
                skipDummy = reading && (chip == Chip.Bmi088Accel || chip == Chip.Bmi270);
                return 0;
            }

            byte result = 0;
            if(reading)
            {
                if(skipDummy)
                {
                    skipDummy = false;
                    return 0;
                }
                result = ReadRegister(currentRegister);
            }
            else
            {
                WriteRegister(currentRegister, value);
            }
            Advance();
            return result;
        }

        public void Write(byte[] data)
        {
            if(data.Length == 0)
            {
                return;
            }
            currentRegister = (byte)(data[0] & AddressMask);
            for(var index = 1; index < data.Length; index++)
            {
                WriteRegister(currentRegister, data[index]);
                Advance();
            }
        }

        public byte[] Read(int count = 1)
        {
            // I2C reads have no SPI dummy byte
            var result = new byte[count];
            for(var index = 0; index < count; index++)
            {
                result[index] = ReadRegister(currentRegister);
                Advance();
            }
            return result;
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

        private void Advance()
        {
            if(!IsStreamRegister(currentRegister))
            {
                currentRegister = (byte)((currentRegister + 1) & AddressMask);
            }
        }

        private enum Chip
        {
            Unknown,
            Bmi055Accel,
            Bmi088Accel,
            BoschGyro,
            Bmi160,
            Bmi270,
            Lsm9ds0Gyro,
            Lsm9ds0AccelMag,
            Lsm6dsv,
        }

        private static Chip ChipFor(byte whoAmI)
        {
            switch(whoAmI)
            {
            case 0xFA:
                return Chip.Bmi055Accel;
            case 0x1E:
            case 0x1F:
                return Chip.Bmi088Accel;
            case 0x0F:
                return Chip.BoschGyro;
            case 0xD1:
                return Chip.Bmi160;
            case 0x24:
                return Chip.Bmi270;
            case 0xD4:
            case 0xD7:
                return Chip.Lsm9ds0Gyro;
            case 0x49:
                return Chip.Lsm9ds0AccelMag;
            case 0x70:
                return Chip.Lsm6dsv;
            default:
                return Chip.Unknown;
            }
        }

        private bool IsLsm => chip == Chip.Lsm9ds0Gyro ||
            chip == Chip.Lsm9ds0AccelMag || chip == Chip.Lsm6dsv;

        // LSM9DS0 uses address bit 6 as the auto-increment flag
        private byte AddressMask =>
            chip == Chip.Lsm9ds0Gyro || chip == Chip.Lsm9ds0AccelMag ? (byte)0x3F : (byte)0x7F;

        private bool IsStreamRegister(byte register)
        {
            switch(chip)
            {
            case Chip.Bmi055Accel:
            case Chip.BoschGyro:
                return register == BoschFifoData;
            case Chip.Bmi088Accel:
            case Chip.Bmi270:
                return register == Bmi088FifoData || (chip == Chip.Bmi270 && register == Bmi270InitData);
            case Chip.Bmi160:
                return register == Bmi160FifoData;
            case Chip.Lsm6dsv:
                return register == Lsm6dsvFifoTag;
            default:
                return false;
            }
        }

        private byte ReadRegister(byte register)
        {
            switch(chip)
            {
            case Chip.Bmi055Accel:
                switch(register)
                {
                case Bmi055Temperature:
                    return (byte)(sbyte)Clamp(Math.Round((thermal.TemperatureC - 23.0) * 2.0), -128, 127);
                case BoschFifoStatus:
                    return Refill((registers[BoschFifoConfig1] & 0xC0) != 0, 6);
                case BoschFifoData:
                    return Dequeue();
                }
                break;
            case Chip.BoschGyro:
                switch(register)
                {
                case BoschFifoStatus:
                    return Refill((registers[BoschFifoConfig1] & 0xC0) != 0, 6);
                case BoschFifoData:
                    return Dequeue();
                }
                break;
            case Chip.Bmi088Accel:
                switch(register)
                {
                case Bmi088TemperatureMsb:
                case Bmi088TemperatureLsb:
                    var temperature = (int)Clamp(Math.Round((thermal.TemperatureC - 23.0) * 8.0), -1024, 1023) & 0x7FF;
                    return register == Bmi088TemperatureMsb ? (byte)(temperature >> 3) : (byte)((temperature & 0x07) << 5);
                case Bmi088FifoLength0:
                    Refill((registers[Bmi088FifoConfig1] & 0x40) != 0, 1);
                    return (byte)fifo.Count;
                case Bmi088FifoLength1:
                    return (byte)(fifo.Count >> 8);
                case Bmi088FifoData:
                    return Dequeue();
                }
                break;
            case Chip.Bmi160:
                switch(register)
                {
                case Bmi160TemperatureLsb:
                    return (byte)Bmi160Temperature();
                case Bmi160TemperatureMsb:
                    return (byte)(Bmi160Temperature() >> 8);
                case Bmi160FifoLength0:
                    Refill((registers[Bmi160FifoConfig1] & 0xC0) == 0xC0, 1);
                    return (byte)fifo.Count;
                case Bmi160FifoLength1:
                    return (byte)(fifo.Count >> 8);
                case Bmi160FifoData:
                    return Dequeue();
                }
                break;
            case Chip.Bmi270:
                switch(register)
                {
                case Bmi270InternalStatus:
                    return registers[Bmi270InitControl] == 1 ? Bmi270InitOk : (byte)0;
                case Bmi270TemperatureLsb:
                    return (byte)Bmi270Temperature();
                case Bmi270TemperatureMsb:
                    return (byte)(Bmi270Temperature() >> 8);
                case Bmi088FifoLength0:
                    Refill((registers[Bmi270FifoConfig1] & 0xC0) == 0xC0, 1);
                    return (byte)fifo.Count;
                case Bmi088FifoLength1:
                    return (byte)(fifo.Count >> 8);
                case Bmi088FifoData:
                    return Dequeue();
                }
                break;
            case Chip.Lsm9ds0Gyro:
                switch(register)
                {
                case LsmStatusA:
                    return LsmDataReady;
                case LsmOutX:
                    LatchLsm9ds0(Truth().GyroRadS, rotation, Lsm9ds0GyroScale);
                    break;
                }
                break;
            case Chip.Lsm9ds0AccelMag:
                switch(register)
                {
                case Lsm9ds0Temperature:
                    WriteInt16(Lsm9ds0Temperature, (short)Clamp(Math.Round((thermal.TemperatureC - 25.0) * 8.0), -2048, 2047));
                    break;
                case Lsm9ds0StatusM:
                    LatchMagnetometer();
                    return LsmDataReady;
                case LsmStatusA:
                    return LsmDataReady;
                case LsmOutX:
                    LatchLsm9ds0(Truth().SpecificForceMS2, rotation, Lsm9ds0AccelScale);
                    break;
                }
                break;
            case Chip.Lsm6dsv:
                switch(register)
                {
                case Lsm6dsvFifoStatus1:
                    Refill((registers[Lsm6dsvFifoCtrl4] & 0x07) == Lsm6dsvContinuous &&
                           registers[Lsm6dsvFifoCtrl3] != 0, 1);
                    return (byte)(fifo.Count / Lsm6dsvWordSize);
                case Lsm6dsvFifoStatus2:
                    return (byte)((fifo.Count / Lsm6dsvWordSize) >> 8);
                case Lsm6dsvStatus:
                    return Lsm6dsvDataReady;
                case Lsm6dsvTemperature:
                    WriteInt16(Lsm6dsvTemperature, (short)Clamp(Math.Round((thermal.TemperatureC - 25.0) * 256.0),
                                                                Int16.MinValue, Int16.MaxValue));
                    break;
                case Lsm6dsvFifoTag:
                    return Dequeue();
                }
                break;
            }
            return registers[register];
        }

        private void WriteRegister(byte register, byte value)
        {
            if(IsReset(register, value))
            {
                Reset();
                return;
            }
            if(IsStreamRegister(register))
            {
                return;
            }
            if(register == BoschCommand && (chip == Chip.Bmi088Accel || chip == Chip.Bmi160 || chip == Chip.Bmi270))
            {
                // commands self-clear
                if(value == BoschFifoFlush)
                {
                    fifo.Clear();
                }
                return;
            }
            registers[register] = value;
        }

        private bool IsReset(byte register, byte value)
        {
            switch(chip)
            {
            case Chip.Bmi055Accel:
            case Chip.BoschGyro:
                return register == BoschBgwSoftReset && value == BoschSoftReset;
            case Chip.Bmi088Accel:
            case Chip.Bmi160:
            case Chip.Bmi270:
                return register == BoschCommand && value == BoschSoftReset;
            case Chip.Lsm6dsv:
                return register == Lsm6dsvControl3 && (value & Lsm6dsvSoftwareReset) != 0;
            default:
                return false;
            }
        }

        // latch one sample into the FIFO and return its BMI055/BMI088 gyro frame count
        private byte Refill(bool enabled, int frameBytes)
        {
            fifo.Clear();
            if(!enabled)
            {
                return 0;
            }
            var truth = Truth();
            var accel = AP_SensorOrientation.BodyToSensor(truth.SpecificForceMS2, rotation);
            var gyro = AP_SensorOrientation.BodyToSensor(truth.GyroRadS, rotation);
            switch(chip)
            {
            case Chip.Bmi055Accel:
                foreach(var axis in accel)
                {
                    // 12-bit sample, left-justified
                    PushInt16((short)(Clamp(Math.Round(axis / (Bmi055AccelScale * 16.0)), -2048, 2047) * 16));
                }
                break;
            case Chip.BoschGyro:
                PushVector(gyro, BoschGyroScale);
                break;
            case Chip.Bmi088Accel:
                fifo.Enqueue(BoschAccelFrame);
                PushVector(accel, Bmi088AccelScale);
                break;
            case Chip.Bmi160:
                PushVector(gyro, Bmi160GyroScale);
                PushVector(accel, Accel16GScale);
                break;
            case Chip.Bmi270:
                fifo.Enqueue(Bmi270CombinedFrame);
                PushVector(gyro, BoschGyroScale);
                PushVector(accel, Accel16GScale);
                break;
            case Chip.Lsm6dsv:
                fifo.Enqueue(Lsm6dsvGyroTag);
                PushVector(gyro, LsmGyroScale);
                fifo.Enqueue(Lsm6dsvAccelTag);
                PushVector(accel, Accel16GScale);
                break;
            }
            return (byte)(fifo.Count / frameBytes);
        }

        // the LSM9DS0 backend reads its axes as (x, -y, -z)
        private void LatchLsm9ds0(float[] body, byte sensorRotation, double scale)
        {
            var value = AP_SensorOrientation.BodyToSensor(body, sensorRotation);
            WriteInt16(LsmOutX, Scale(value[0], scale));
            WriteInt16(LsmOutX + 2, Scale(-value[1], scale));
            WriteInt16(LsmOutX + 4, Scale(-value[2], scale));
        }

        private void LatchMagnetometer()
        {
            var field = AP_SensorOrientation.BodyToSensor(Truth().MagneticFieldBodyMgauss, CompassRotation);
            var scale = Lsm303dMagScales[(registers[Lsm303dControl6] >> 5) & 0x03];
            for(var axis = 0; axis < 3; axis++)
            {
                WriteInt16(Lsm9ds0OutM + 2 * axis, Scale(field[axis], scale));
            }
        }

        private AP_PhysicsTruth Truth()
        {
            return physics.Current;
        }

        private short Bmi160Temperature()
        {
            return (short)Clamp(Math.Round((thermal.TemperatureC - 23.0) * 512.0), Int16.MinValue, Int16.MaxValue);
        }

        private short Bmi270Temperature()
        {
            return (short)Clamp(Math.Round((thermal.TemperatureC - 23.0) / 0.002), Int16.MinValue, Int16.MaxValue);
        }

        private byte Dequeue()
        {
            return fifo.Count > 0 ? fifo.Dequeue() : (byte)0;
        }

        private void PushVector(float[] value, double scale)
        {
            foreach(var axis in value)
            {
                PushInt16(Scale(axis, scale));
            }
        }

        private void PushInt16(short value)
        {
            fifo.Enqueue((byte)value);
            fifo.Enqueue((byte)(value >> 8));
        }

        private void WriteInt16(int register, short value)
        {
            registers[register] = (byte)value;
            registers[register + 1] = (byte)(value >> 8);
        }

        private static short Scale(float value, double scale)
        {
            return (short)Clamp(Math.Round(value / scale), Int16.MinValue, Int16.MaxValue);
        }

        private static double Clamp(double value, double minimum, double maximum)
        {
            return Math.Max(minimum, Math.Min(maximum, value));
        }

        private readonly byte rotation;
        private readonly byte whoAmI;
        private readonly Chip chip;
        private readonly AP_PhysicsState physics;
        private readonly AP_BoardThermalState thermal;
        private readonly byte[] registers;
        private readonly Queue<byte> fifo;
        private int transferByte;
        private byte currentRegister;
        private bool reading;
        private bool skipDummy;

        private const int RegisterCount = 128;
        private const byte ReadFlag = 0x80;
        private const byte WhoAmI = 0x00;
        private const byte LsmWhoAmI = 0x0F;

        private const byte Bmi055Temperature = 0x08;
        private const byte BoschFifoStatus = 0x0E;
        private const byte BoschBgwSoftReset = 0x14;
        private const byte BoschFifoConfig1 = 0x3E;
        private const byte BoschFifoData = 0x3F;
        private const byte Bmi088TemperatureMsb = 0x22;
        private const byte Bmi088TemperatureLsb = 0x23;
        private const byte Bmi088FifoLength0 = 0x24;
        private const byte Bmi088FifoLength1 = 0x25;
        private const byte Bmi088FifoData = 0x26;
        private const byte Bmi088FifoConfig1 = 0x49;
        private const byte Bmi160TemperatureLsb = 0x20;
        private const byte Bmi160TemperatureMsb = 0x21;
        private const byte Bmi160FifoLength0 = 0x22;
        private const byte Bmi160FifoLength1 = 0x23;
        private const byte Bmi160FifoData = 0x24;
        private const byte Bmi160FifoConfig1 = 0x47;
        private const byte Bmi270InternalStatus = 0x21;
        private const byte Bmi270TemperatureLsb = 0x22;
        private const byte Bmi270TemperatureMsb = 0x23;
        private const byte Bmi270FifoConfig1 = 0x49;
        private const byte Bmi270InitControl = 0x59;
        private const byte Bmi270InitData = 0x5E;
        private const byte BoschCommand = 0x7E;
        private const byte BoschSoftReset = 0xB6;
        private const byte BoschFifoFlush = 0xB0;
        private const byte BoschAccelFrame = 0x84;
        private const byte Bmi270CombinedFrame = 0x8C;
        private const byte Bmi270InitOk = 0x01;

        private const byte Lsm9ds0Temperature = 0x05;
        private const byte Lsm9ds0StatusM = 0x07;
        private const byte Lsm9ds0OutM = 0x08;
        private const byte Lsm303dControl6 = 0x25;
        private const byte LsmStatusA = 0x27;
        private const byte LsmOutX = 0x28;
        private const byte LsmDataReady = 0x0F;
        private const byte Lsm6dsvFifoCtrl3 = 0x09;
        private const byte Lsm6dsvFifoCtrl4 = 0x0A;
        private const byte Lsm6dsvControl3 = 0x12;
        private const byte Lsm6dsvFifoStatus1 = 0x1B;
        private const byte Lsm6dsvFifoStatus2 = 0x1C;
        private const byte Lsm6dsvStatus = 0x1E;
        private const byte Lsm6dsvTemperature = 0x20;
        private const byte Lsm6dsvFifoTag = 0x78;
        private const byte Lsm6dsvSoftwareReset = 0x01;
        private const byte Lsm6dsvContinuous = 0x06;
        private const byte Lsm6dsvDataReady = 0x07;
        private const byte Lsm6dsvGyroTag = 0x01 << 3;
        private const byte Lsm6dsvAccelTag = 0x02 << 3;
        private const int Lsm6dsvWordSize = 7;

        private const double Gravity = 9.80665;
        private const double DegreesToRadians = Math.PI / 180.0;
        private const double Bmi055AccelScale = 7.81e-3 * Gravity / 16.0;
        private const double Bmi088AccelScale = Gravity * 24.0 / 32768.0;
        private const double Accel16GScale = Gravity * 16.0 / 32768.0;
        private const double BoschGyroScale = DegreesToRadians * 2000.0 / 32767.0;
        private const double Bmi160GyroScale = DegreesToRadians * 1000.0 / 16384.0;
        private const double LsmGyroScale = DegreesToRadians * 0.070;
        private const double Lsm9ds0GyroScale = LsmGyroScale;
        private const double Lsm9ds0AccelScale = Gravity * 0.000732;
        private static readonly double[] Lsm303dMagScales = { 0.080, 0.160, 0.320, 0.479 };
    }
}
