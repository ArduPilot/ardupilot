//
// ICM-42688-P IMU as AP_InertialSensor_Invensensev3 drives it. The
// model supplies banked register storage, the 0x47 product ID, and
// 16-byte little-endian FIFO records at 1kHz. Acceleration and angular-rate
// samples follow physics truth after applying the board's sensor rotation;
// temperature remains constant at 25C.
//
using System;
using System.Collections.Generic;
using Antmicro.Renode.Core;
using Antmicro.Renode.Peripherals;
using Antmicro.Renode.Peripherals.SPI;
using Antmicro.Renode.Peripherals.Miscellaneous;
using Antmicro.Renode.Peripherals.Timers;
using Antmicro.Renode.Time;

namespace Antmicro.Renode.Peripherals.Sensors
{
    // IGPIOReceiver provides chip-select transaction framing in addition
    // to the H7 SPI model's TSIZE completion. PC15 deassertion also resets
    // the parser after an aborted or endless-mode transfer.
    public class AP_ICM42688 : ISPIPeripheral, IGPIOReceiver
    {
        public AP_ICM42688(IMachine machine, byte whoAmI = DefaultWhoAmI, byte rotation = 8)
        {
            this.whoAmI = whoAmI;
            this.rotation = rotation;
            physics = AP_PhysicsState.ForMachine(machine);
            fifo = new Queue<byte>();
            registers = new byte[BankCount, RegisterCount];
            sampleTimer = new LimitTimer(machine.ClockSource, 1000000, this, "icm42688 odr",
                                         limit: SamplePeriodUs, direction: Direction.Ascending,
                                         enabled: true, workMode: WorkMode.Periodic, eventEnabled: true);
            sampleTimer.LimitReached += OnSampleTick;
            Reset();
        }

        public void Reset()
        {
            Array.Clear(registers, 0, registers.Length);
            fifo.Clear();
            transferByte = 0;
            currentBank = 0;
            currentRegister = 0;
            reading = false;
            timestamp = 0;
            registers[0, WhoAmI] = whoAmI;
            registers[0, Icm45686WhoAmI] = whoAmI;
        }

        public byte Transmit(byte value)
        {
            byte response = 0;
            if(transferByte == 0)
            {
                reading = (value & ReadFlag) != 0;
                currentRegister = (byte)(value & RegisterMask);
            }
            else if(reading)
            {
                response = ReadRegister(currentRegister);
                if(currentRegister != FifoData)
                {
                    currentRegister = (byte)((currentRegister + 1) & RegisterMask);
                }
            }
            else
            {
                WriteRegister(currentRegister, value);
                if(currentRegister != FifoData)
                {
                    currentRegister = (byte)((currentRegister + 1) & RegisterMask);
                }
            }
            transferByte++;
            return response;
        }

        public void FinishTransmission()
        {
            transferByte = 0;
        }

        public void OnGPIO(int number, bool value)
        {
            if(value)
            {
                transferByte = 0;
            }
        }

        private byte ReadRegister(byte register)
        {
            if(currentBank == 0)
            {
                switch(register)
                {
                case FifoCountLow:
                    return (byte)(fifo.Count / CurrentSampleSize);
                case FifoCountHigh:
                    return (byte)((fifo.Count / CurrentSampleSize) >> 8);
                case FifoData:
                    return fifo.Count > 0 ? fifo.Dequeue() : (byte)0;
                case InterruptStatus:
                    return fifo.Count >= SampleSize ? DataReady : (byte)0;
                case BankSelect:
                    return currentBank;
                }
            }
            return registers[currentBank, register];
        }

        private void WriteRegister(byte register, byte value)
        {
            if(register == BankSelect)
            {
                var bank = (byte)(value & BankMask);
                currentBank = bank < BankCount ? bank : (byte)0;
                return;
            }

            if(currentBank == 0 && register == SignalPathReset && (value & FifoFlush) != 0)
            {
                fifo.Clear();
                registers[0, register] = 0;
                return;
            }
            if(currentBank == 0 && register == FifoData)
            {
                return;
            }
            registers[currentBank, register] = value;
        }

        private void OnSampleTick()
        {
            var sampleSize = CurrentSampleSize;
            if((registers[0, FifoConfig1] & FifoSensorsEnabled) != FifoSensorsEnabled ||
               (registers[0, PowerManagement] & SensorsLowNoise) != SensorsLowNoise ||
               fifo.Count + sampleSize > FifoCapacity)
            {
                return;
            }

            if(HighResolutionEnabled)
            {
                PushHighResolutionSample();
                return;
            }

            fifo.Enqueue(FifoHeader);
            var truth = physics.Current;
            var acceleration = AP_SensorOrientation.BodyToSensor(truth.SpecificForceMS2, rotation);
            var gyro = AP_SensorOrientation.BodyToSensor(truth.GyroRadS, rotation);
            PushWord(ScaleWord(acceleration[0], AccelScale));
            PushWord(ScaleWord(acceleration[1], AccelScale));
            PushWord(ScaleWord(acceleration[2], AccelScale));
            PushWord(ScaleWord(gyro[0], GyroScale));
            PushWord(ScaleWord(gyro[1], GyroScale));
            PushWord(ScaleWord(gyro[2], GyroScale));
            fifo.Enqueue(0);
            PushWord((short)timestamp++);
        }

        private void PushHighResolutionSample()
        {
            fifo.Enqueue(FifoHighResolutionHeader);
            var truth = physics.Current;
            var acceleration = AP_SensorOrientation.BodyToSensor(truth.SpecificForceMS2, rotation);
            var gyro = AP_SensorOrientation.BodyToSensor(truth.GyroRadS, rotation);
            var values = new int[] {
                ScaleHighResolution(acceleration[0], AccelHighResolutionScale),
                ScaleHighResolution(acceleration[1], AccelHighResolutionScale),
                ScaleHighResolution(acceleration[2], AccelHighResolutionScale),
                ScaleHighResolution(gyro[0], GyroHighResolutionScale),
                ScaleHighResolution(gyro[1], GyroHighResolutionScale),
                ScaleHighResolution(gyro[2], GyroHighResolutionScale),
            };
            foreach(var value in values)
            {
                fifo.Enqueue((byte)(value >> 4));
                fifo.Enqueue((byte)(value >> 12));
            }
            PushWord(0);
            PushWord((short)timestamp++);
            fifo.Enqueue((byte)((values[3] & 0xF) | (values[0] & 0xF) << 4));
            fifo.Enqueue((byte)((values[4] & 0xF) | (values[1] & 0xF) << 4));
            fifo.Enqueue((byte)((values[5] & 0xF) | (values[2] & 0xF) << 4));
        }

        private static short ScaleWord(float value, double scale)
        {
            return (short)Math.Max(Int16.MinValue,
                Math.Min(Int16.MaxValue, Math.Round(value / scale)));
        }

        private static int ScaleHighResolution(float value, double scale)
        {
            return (int)Math.Max(-524288,
                Math.Min(524287, Math.Round(value / scale)));
        }

        private void PushWord(short value)
        {
            fifo.Enqueue((byte)(value & 0xFF));
            fifo.Enqueue((byte)((value >> 8) & 0xFF));
        }

        private bool HighResolutionEnabled =>
            (registers[0, FifoConfig1] & FifoHighResolutionEnable) != 0;
        private int CurrentSampleSize => HighResolutionEnabled ? HighResolutionSampleSize : SampleSize;

        private readonly Queue<byte> fifo;
        private readonly byte[,] registers;
        private readonly LimitTimer sampleTimer;
        private readonly byte whoAmI;
        private readonly byte rotation;
        private readonly AP_PhysicsState physics;
        private int transferByte;
        private byte currentBank;
        private byte currentRegister;
        private bool reading;
        private ushort timestamp;

        private const int BankCount = 5;
        private const int RegisterCount = 128;
        private const int SamplePeriodUs = 1000;
        private const int SampleSize = 16;
        private const int HighResolutionSampleSize = 20;
        private const int FifoCapacity = 2048;

        private const byte SignalPathReset = 0x4B;
        private const byte PowerManagement = 0x4E;
        private const byte FifoConfig1 = 0x5F;
        private const byte InterruptStatus = 0x2D;
        private const byte FifoCountLow = 0x2E;
        private const byte FifoCountHigh = 0x2F;
        private const byte FifoData = 0x30;
        private const byte WhoAmI = 0x75;
        private const byte Icm45686WhoAmI = 0x72;
        private const byte BankSelect = 0x76;

        private const byte ReadFlag = 0x80;
        private const byte RegisterMask = 0x7F;
        private const byte BankMask = 0x07;
        private const byte DefaultWhoAmI = 0x47;
        private const byte DataReady = 0x08;
        private const byte FifoFlush = 0x02;
        private const byte FifoSensorsEnabled = 0x07;
        private const byte FifoHighResolutionEnable = 0x10;
        private const byte SensorsLowNoise = 0x0F;
        private const byte FifoHeader = 0x68;
        private const byte FifoHighResolutionHeader = 0x78;
        private const double Gravity = 9.80665;
        private const double AccelScale = Gravity * 16.0 / 32768.0;
        private const double GyroScale = Math.PI / 180.0 * 2000.0 / 32768.0;
        private const double AccelHighResolutionScale = Gravity * 16.0 / 524288.0;
        private const double GyroHighResolutionScale = Math.PI / 180.0 * 2000.0 / 524288.0;
    }
}
