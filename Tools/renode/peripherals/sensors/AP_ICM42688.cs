//
// ICM-42688-P IMU as AP_InertialSensor_Invensensev3 drives it. The
// model supplies banked register storage, the 0x47 product ID, and
// 16-byte little-endian FIFO records at 1kHz. Samples describe a still,
// level board: +1g on sensor Z becomes -1g after the board's configured
// ROTATION_ROLL_180, with zero angular rate and a constant 25C.
//
using System;
using System.Collections.Generic;
using Antmicro.Renode.Core;
using Antmicro.Renode.Peripherals;
using Antmicro.Renode.Peripherals.SPI;
using Antmicro.Renode.Peripherals.Timers;
using Antmicro.Renode.Time;

namespace Antmicro.Renode.Peripherals.Sensors
{
    // IGPIOReceiver provides chip-select transaction framing in addition
    // to the H7 SPI model's TSIZE completion. PC15 deassertion also resets
    // the parser after an aborted or endless-mode transfer.
    public class AP_ICM42688 : ISPIPeripheral, IGPIOReceiver
    {
        public AP_ICM42688(IMachine machine, byte whoAmI = DefaultWhoAmI)
        {
            this.whoAmI = whoAmI;
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
                    return (byte)(fifo.Count / SampleSize);
                case FifoCountHigh:
                    return (byte)((fifo.Count / SampleSize) >> 8);
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
            if((registers[0, FifoConfig1] & FifoSensorsEnabled) != FifoSensorsEnabled ||
               (registers[0, PowerManagement] & SensorsLowNoise) != SensorsLowNoise ||
               fifo.Count + SampleSize > FifoCapacity)
            {
                return;
            }

            fifo.Enqueue(FifoHeader);
            PushWord(0);
            PushWord(0);
            PushWord(2048);
            PushWord(0);
            PushWord(0);
            PushWord(0);
            fifo.Enqueue(0);
            PushWord((short)timestamp++);
        }

        private void PushWord(short value)
        {
            fifo.Enqueue((byte)(value & 0xFF));
            fifo.Enqueue((byte)((value >> 8) & 0xFF));
        }

        private readonly Queue<byte> fifo;
        private readonly byte[,] registers;
        private readonly LimitTimer sampleTimer;
        private readonly byte whoAmI;
        private int transferByte;
        private byte currentBank;
        private byte currentRegister;
        private bool reading;
        private ushort timestamp;

        private const int BankCount = 5;
        private const int RegisterCount = 128;
        private const int SamplePeriodUs = 1000;
        private const int SampleSize = 16;
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
        private const byte SensorsLowNoise = 0x0F;
        private const byte FifoHeader = 0x68;
    }
}
