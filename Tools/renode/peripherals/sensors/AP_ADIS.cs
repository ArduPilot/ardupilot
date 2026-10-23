// Register transports for the two ADIS protocol variants used by ArduPilot,
// with product identity, writable configuration registers and clear
// diagnostic state. Burst and FIFO samples follow physics truth after
// applying the board's sensor rotation.
//
using System;
using System.Collections.Generic;
using Antmicro.Renode.Core;
using Antmicro.Renode.Peripherals;
using Antmicro.Renode.Peripherals.Miscellaneous;
using Antmicro.Renode.Peripherals.SPI;

namespace Antmicro.Renode.Peripherals.Sensors
{
    public class AP_ADIS1647x : ISPIPeripheral, IGPIOReceiver
    {
        public AP_ADIS1647x(IMachine machine, ushort productId = ProductId16470, byte rotation = 0)
        {
            registers = new byte[RegisterCount];
            burst = new byte[BurstLength];
            this.productId = productId;
            this.rotation = rotation;
            physics = AP_PhysicsState.ForMachine(machine);
            thermal = AP_BoardThermalState.ForMachine(machine);
            Reset();
        }

        public void Reset()
        {
            Array.Clear(registers, 0, registers.Length);
            WriteWord(ProductIdRegister, productId);
            transferByte = 0;
            pendingRead = false;
        }

        public byte Transmit(byte value)
        {
            if(pendingRead)
            {
                var result = transferByte == 0
                    ? registers[(pendingRegister + 1) & RegisterMask]
                    : registers[pendingRegister];
                transferByte++;
                return result;
            }
            if(transferByte == 0)
            {
                command = value;
            }
            else if((command & WriteFlag) != 0)
            {
                registers[command & RegisterMask] = value;
            }
            else if(command == BurstCommand && transferByte >= 2)
            {
                // burst reads follow the two command bytes in one transaction
                if(transferByte == 2)
                {
                    LatchBurst();
                }
                var index = transferByte - 2;
                transferByte++;
                return index < burst.Length ? burst[index] : (byte)0;
            }
            transferByte++;
            return 0;
        }

        public void FinishTransmission()
        {
            if(pendingRead)
            {
                pendingRead = false;
            }
            else if((command & WriteFlag) == 0 && transferByte <= 2)
            {
                pendingRegister = (byte)(command & RegisterMask);
                pendingRead = true;
            }
            transferByte = 0;
        }

        public void OnGPIO(int number, bool value)
        {
            if(value)
            {
                FinishTransmission();
            }
        }

        // DIAG_STAT, X/Y/Z gyro, X/Y/Z accel, TEMP, DATA_CNTR and checksum, big-endian
        private void LatchBurst()
        {
            var truth = physics.Current;
            var gyro = AP_SensorOrientation.BodyToSensor(truth.GyroRadS, rotation);
            var accel = AP_SensorOrientation.BodyToSensor(truth.SpecificForceMS2, rotation);
            var words = new short[] {
                0,
                Scale(gyro[0], GyroScale), Scale(gyro[1], GyroScale), Scale(gyro[2], GyroScale),
                Scale(accel[0], AccelScale), Scale(accel[1], AccelScale), Scale(accel[2], AccelScale),
                (short)Math.Round(thermal.TemperatureC * 10.0),
                (short)++counter,
            };
            byte sum = 0;
            for(var index = 0; index < words.Length; index++)
            {
                burst[2 * index] = (byte)(words[index] >> 8);
                burst[2 * index + 1] = (byte)words[index];
                sum += burst[2 * index];
                sum += burst[2 * index + 1];
            }
            burst[BurstLength - 2] = 0;
            burst[BurstLength - 1] = sum;
        }

        private static short Scale(float value, double scale)
        {
            return (short)Math.Max(Int16.MinValue, Math.Min(Int16.MaxValue, Math.Round(value / scale)));
        }

        private void WriteWord(byte register, ushort value)
        {
            registers[register] = (byte)value;
            registers[register + 1] = (byte)(value >> 8);
        }

        private readonly byte[] registers;
        private readonly byte[] burst;
        private readonly ushort productId;
        private readonly byte rotation;
        private readonly AP_PhysicsState physics;
        private readonly AP_BoardThermalState thermal;
        private int transferByte;
        private byte command;
        private byte pendingRegister;
        private bool pendingRead;
        private ushort counter;

        private const int RegisterCount = 128;
        private const int BurstLength = 20;
        private const byte WriteFlag = 0x80;
        private const byte RegisterMask = 0x7F;
        private const byte BurstCommand = 0x68;
        private const byte ProductIdRegister = 0x72;
        private const ushort ProductId16470 = 0x4056;
        private const double AccelScale = 1.25e-3 * 9.80665;
        private const double GyroScale = 0.1 * Math.PI / 180.0;
    }

    public class AP_ADIS16607 : ISPIPeripheral, IGPIOReceiver
    {
        public AP_ADIS16607(IMachine machine, byte rotation = 0)
        {
            this.rotation = rotation;
            physics = AP_PhysicsState.ForMachine(machine);
            thermal = AP_BoardThermalState.ForMachine(machine);
            registers = new ushort[RegisterCount];
            fifo = new Queue<ushort>();
            Reset();
        }

        public void Reset()
        {
            Array.Clear(registers, 0, registers.Length);
            registers[DeviceId] = DeviceIdValue;
            registers[WriteLock] = 1;
            fifo.Clear();
            channel = WordsPerSample - 1;
            transferByte = 0;
        }

        public byte Transmit(byte value)
        {
            if(transferByte == 0)
            {
                command = value;
                transferByte++;
                return 0;
            }
            var register = command & RegisterMask;
            if((command & ReadFlag) != 0)
            {
                byte result = 0;
                if(transferByte >= 2)
                {
                    // FIFO_DATA streams consecutive words within a transaction
                    if(transferByte % 2 == 0 && (transferByte == 2 || register == FifoData))
                    {
                        readWord = ReadWord(register);
                    }
                    result = transferByte % 2 == 0 ? (byte)(readWord >> 8) : (byte)readWord;
                }
                transferByte++;
                return result;
            }
            if(transferByte == 1)
            {
                writeHigh = value;
            }
            else if(transferByte == 2)
            {
                WriteRegister(register, (ushort)((writeHigh << 8) | value));
            }
            transferByte++;
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

        private ushort ReadWord(int register)
        {
            switch(register)
            {
            case FifoWordCount:
                if(fifo.Count < WordsPerSample)
                {
                    QueueSample();
                }
                return (ushort)fifo.Count;
            case FifoData:
                if(fifo.Count == 0)
                {
                    return 0;
                }
                channel = (channel + 1) % WordsPerSample;
                return fifo.Dequeue();
            case FifoChannelIndex:
                return (ushort)(channel == WordsPerSample - 1 ? CounterChannel : channel);
            default:
                return registers[register];
            }
        }

        private void WriteRegister(int register, ushort value)
        {
            switch(register)
            {
            case WriteLock:
                // unlock is 0xAAAA then 0x5555, lock is 0x5555 then 0xAAAA
                registers[register] = (ushort)(value == 0xAAAA ? 1 : 0);
                break;
            case SoftReset:
                if((value & 0x01) != 0)
                {
                    Reset();
                }
                break;
            case UserFifoConfig:
                if((value & ClearFifo) != 0)
                {
                    fifo.Clear();
                    channel = WordsPerSample - 1;
                }
                registers[register] = (ushort)(value & ~ClearFifo);
                break;
            default:
                registers[register] = value;
                break;
            }
        }

        // X/Y/Z accel, X/Y/Z gyro as 24-bit values in 32-bit words, then TEMP and DATA_CNTR
        private void QueueSample()
        {
            var truth = physics.Current;
            var accel = AP_SensorOrientation.BodyToSensor(truth.SpecificForceMS2, rotation);
            var gyro = AP_SensorOrientation.BodyToSensor(truth.GyroRadS, rotation);
            foreach(var axis in accel)
            {
                Push24(axis / AccelScale);
            }
            foreach(var axis in gyro)
            {
                Push24(axis / GyroScale);
            }
            fifo.Enqueue((ushort)(short)Math.Max(Int16.MinValue, Math.Min(Int16.MaxValue,
                Math.Round((thermal.TemperatureC - 25.0) / 0.005))));
            fifo.Enqueue(++counter);
        }

        private void Push24(double value)
        {
            var word = (int)Math.Max(-0x800000, Math.Min(0x7FFFFF, Math.Round(value))) << 8;
            fifo.Enqueue((ushort)(word >> 16));
            fifo.Enqueue((ushort)word);
        }

        private readonly byte rotation;
        private readonly AP_PhysicsState physics;
        private readonly AP_BoardThermalState thermal;
        private readonly ushort[] registers;
        private readonly Queue<ushort> fifo;
        private int transferByte;
        private byte command;
        private byte writeHigh;
        private ushort readWord;
        private ushort counter;
        private int channel;

        private const int RegisterCount = 128;
        private const byte ReadFlag = 0x80;
        private const byte RegisterMask = 0x7F;
        private const byte DeviceId = 0x00;
        private const byte FifoData = 0x29;
        private const byte FifoChannelIndex = 0x2A;
        private const byte FifoWordCount = 0x2B;
        private const byte WriteLock = 0x2D;
        private const byte UserFifoConfig = 0x35;
        private const byte SoftReset = 0x36;
        private const ushort DeviceIdValue = 0x6000;
        private const ushort ClearFifo = 0x8000;
        private const int WordsPerSample = 14;
        private const int CounterChannel = 28;
        private const double AccelScale = 9.80665 / 200000.0;
        private const double GyroScale = Math.PI / 180.0 / 4000.0;
    }
}
