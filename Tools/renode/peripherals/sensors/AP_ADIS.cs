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
using Antmicro.Renode.Peripherals.Timers;
using Antmicro.Renode.Time;

namespace Antmicro.Renode.Peripherals.Sensors
{
    // ADIS16470, ADIS16477-x and ADIS16507-x: 2kHz internal clock divided by
    // DEC_RATE, data-ready pulse, 16 or 32-bit burst of gyro/accel or
    // delta angle/velocity selected by MSC_CTRL.
    public class AP_ADIS1647x : ISPIPeripheral, IGPIOReceiver
    {
        public AP_ADIS1647x(IMachine machine, ushort productId = ProductId16470, byte rotation = 0,
                            ushort rangeModel = 0x000F)
        {
            registers = new byte[RegisterCount];
            burst = new byte[MaxBurstLength];
            this.productId = productId;
            this.rotation = rotation;
            this.rangeModel = rangeModel;
            physics = AP_PhysicsState.ForMachine(machine);
            thermal = AP_BoardThermalState.ForMachine(machine);
            DataReady = new GPIO();
            sampleTimer = new LimitTimer(machine.ClockSource, InternalRateHz, this, "adis1647x odr",
                                         limit: 1, direction: Direction.Ascending,
                                         enabled: true, workMode: WorkMode.Periodic, eventEnabled: true);
            sampleTimer.LimitReached += OnSampleTick;
            Reset();
        }

        public GPIO DataReady { get; }

        public void Reset()
        {
            Array.Clear(registers, 0, registers.Length);
            WriteWord(ProductIdRegister, productId);
            WriteWord(RangeModelRegister, rangeModel);
            WriteWord(MscCtrlRegister, MscCtrlDefault);
            sampleTimer.Limit = 1;
            transferByte = 0;
            pendingRead = false;
            DataReady.Set((MscCtrl & DataReadyPolarity) == 0);
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
                if(transferByte == 1)
                {
                    WriteRegister((byte)(command & RegisterMask), value);
                }
            }
            else if(command == BurstCommand && transferByte >= 2)
            {
                // burst reads follow the two command bytes in one transaction
                var index = transferByte - 2;
                transferByte++;
                return index < burstLength ? burst[index] : (byte)0;
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

        private void WriteRegister(byte register, byte value)
        {
            if(register == GlobCmdRegister)
            {
                if((value & SoftwareReset) != 0)
                {
                    Reset();
                }
                return;
            }
            if(register == ProductIdRegister || register == ProductIdRegister + 1 ||
               register == RangeModelRegister || register == RangeModelRegister + 1)
            {
                return;
            }
            registers[register] = value;
            if(register == DecRateRegister || register == DecRateRegister + 1)
            {
                sampleTimer.Limit = (ulong)Math.Min((int)ReadWord(DecRateRegister), MaxDecRate) + 1;
            }
        }

        private void OnSampleTick()
        {
            LatchBurst();
            var active = (MscCtrl & DataReadyPolarity) != 0;
            DataReady.Set(active);
            DataReady.Set(!active);
        }

        private ushort MscCtrl => ReadWord(MscCtrlRegister);

        // 16477/16507 gyro range from RANG_MDL bits [3:2]; 16470 is +-2000 dps
        private double GyroLsbPerDps
        {
            get
            {
                if(productId == ProductId16470)
                {
                    return 10.0;
                }
                switch((rangeModel >> 2) & 3)
                {
                case 0:
                    return 160.0;
                case 1:
                    return 40.0;
                default:
                    return 10.0;
                }
            }
        }

        private double DeltaAngleRangeDeg
        {
            get
            {
                if(productId == ProductId16470)
                {
                    return 2160.0;
                }
                switch((rangeModel >> 2) & 3)
                {
                case 0:
                    return 360.0;
                case 1:
                    return 720.0;
                default:
                    return 2160.0;
                }
            }
        }

        // DIAG_STAT, X/Y/Z gyro (or delta angle), X/Y/Z accel (or delta
        // velocity), TEMP, DATA_CNTR and 16-bit byte-sum checksum, big-endian.
        // 32-bit values are sent low word first.
        private void LatchBurst()
        {
            var truth = physics.Current;
            var gyro = AP_SensorOrientation.BodyToSensor(truth.GyroRadS, rotation);
            var accel = AP_SensorOrientation.BodyToSensor(truth.SpecificForceMS2, rotation);
            var msc = MscCtrl;
            var wide = (msc & Burst32) != 0;
            var delta = (msc & BurstSelect) != 0;
            var full = wide ? 2147483648.0 : 32768.0;
            var values = new double[6];
            if(delta)
            {
                var dt = (double)sampleTimer.Limit / InternalRateHz;
                for(var axis = 0; axis < 3; axis++)
                {
                    values[axis] = gyro[axis] * 180.0 / Math.PI * dt * full / DeltaAngleRangeDeg;
                    values[axis + 3] = accel[axis] * dt * full / DeltaVelocityRangeMS;
                }
            }
            else
            {
                var scale = wide ? 65536.0 : 1.0;
                for(var axis = 0; axis < 3; axis++)
                {
                    values[axis] = gyro[axis] * 180.0 / Math.PI * GyroLsbPerDps * scale;
                    values[axis + 3] = accel[axis] / Gravity * AccelLsbPerG * scale;
                }
            }
            if(wide)
            {
                // the backend negates Y and Z of 32-bit bursts
                values[1] = -values[1];
                values[2] = -values[2];
                values[4] = -values[4];
                values[5] = -values[5];
            }
            var words = new List<ushort> { 0 };
            foreach(var value in values)
            {
                if(wide)
                {
                    var raw = (int)Math.Max(Int32.MinValue, Math.Min(Int32.MaxValue, Math.Round(value)));
                    words.Add((ushort)raw);
                    words.Add((ushort)(raw >> 16));
                }
                else
                {
                    words.Add((ushort)(short)Math.Max(Int16.MinValue, Math.Min(Int16.MaxValue, Math.Round(value))));
                }
            }
            words.Add((ushort)(short)Math.Round(thermal.TemperatureC * 10.0));
            words.Add(++counter);
            ushort sum = 0;
            for(var index = 0; index < words.Count; index++)
            {
                burst[2 * index] = (byte)(words[index] >> 8);
                burst[2 * index + 1] = (byte)words[index];
                sum += burst[2 * index];
                sum += burst[2 * index + 1];
            }
            burst[2 * words.Count] = (byte)(sum >> 8);
            burst[2 * words.Count + 1] = (byte)sum;
            burstLength = 2 * words.Count + 2;
        }

        private ushort ReadWord(byte register)
        {
            return (ushort)(registers[register] | (registers[register + 1] << 8));
        }

        private void WriteWord(byte register, ushort value)
        {
            registers[register] = (byte)value;
            registers[register + 1] = (byte)(value >> 8);
        }

        private readonly byte[] registers;
        private readonly byte[] burst;
        private readonly ushort productId;
        private readonly ushort rangeModel;
        private readonly byte rotation;
        private readonly AP_PhysicsState physics;
        private readonly AP_BoardThermalState thermal;
        private readonly LimitTimer sampleTimer;
        private int transferByte;
        private int burstLength;
        private byte command;
        private byte pendingRegister;
        private bool pendingRead;
        private ushort counter;

        private const int RegisterCount = 128;
        private const int MaxBurstLength = 36;
        private const ulong InternalRateHz = 2000;
        private const int MaxDecRate = 1999;
        private const byte WriteFlag = 0x80;
        private const byte RegisterMask = 0x7F;
        private const byte BurstCommand = 0x68;
        private const byte GlobCmdRegister = 0x68;
        private const byte SoftwareReset = 0x80;
        private const byte RangeModelRegister = 0x5E;
        private const byte MscCtrlRegister = 0x60;
        private const byte DecRateRegister = 0x64;
        private const byte ProductIdRegister = 0x72;
        private const ushort MscCtrlDefault = 0x00C1;
        private const ushort DataReadyPolarity = 0x0001;
        private const ushort BurstSelect = 0x0100;
        private const ushort Burst32 = 0x0200;
        private const ushort ProductId16470 = 0x4056;
        private const double AccelLsbPerG = 800.0;
        private const double DeltaVelocityRangeMS = 400.0;
        private const double Gravity = 9.80665;
    }

    public class AP_ADIS16607 : ISPIPeripheral, IGPIOReceiver
    {
        public AP_ADIS16607(IMachine machine, byte rotation = 0, uint syncClockHz = 8000)
        {
            this.rotation = rotation;
            this.syncClockHz = syncClockHz;
            physics = AP_PhysicsState.ForMachine(machine);
            thermal = AP_BoardThermalState.ForMachine(machine);
            registers = new ushort[RegisterCount];
            fifo = new Queue<ushort>();
            sampleTimer = new LimitTimer(machine.ClockSource, InternalRateHz, this, "adis16607 odr",
                                         limit: 1, direction: Direction.Ascending,
                                         enabled: true, workMode: WorkMode.Periodic, eventEnabled: true);
            sampleTimer.LimitReached += QueueSample;
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
            UpdateRate();
        }

        // 9.56kHz internal clock, or the SYNC input when GPIO2 is mapped to it
        private void UpdateRate()
        {
            var sync = (registers[UserGpioCfg1] & SyncMask) == SyncInput;
            sampleTimer.Frequency = sync ? syncClockHz : InternalRateHz;
            sampleTimer.Limit = (ulong)Math.Min((int)registers[DecRate], MaxDecRate) + 1;
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
                return (ushort)fifo.Count;
            case FifoData:
                if(fifo.Count == 0)
                {
                    return 0xFFFF;
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
            case DecRate:
            case UserGpioCfg1:
                registers[register] = value;
                UpdateRate();
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
            // a full FIFO overwrites the oldest words
            while(fifo.Count > FifoCapacity)
            {
                fifo.Dequeue();
            }
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
        private readonly LimitTimer sampleTimer;
        private readonly uint syncClockHz;
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
        private const byte UserGpioCfg1 = 0x2F;
        private const byte UserFifoConfig = 0x35;
        private const byte DecRate = 0x3A;
        private const ushort SyncMask = 0x01C0;
        private const ushort SyncInput = 0x0040;
        private const uint InternalRateHz = 9560;
        private const int MaxDecRate = 1999;
        private const int FifoCapacity = 1024;
        private const byte SoftReset = 0x36;
        private const ushort DeviceIdValue = 0x6000;
        private const ushort ClearFifo = 0x8000;
        private const int WordsPerSample = 14;
        private const int CounterChannel = 28;
        private const double AccelScale = 9.80665 / 200000.0;
        private const double GyroScale = Math.PI / 180.0 / 4000.0;
    }
}
