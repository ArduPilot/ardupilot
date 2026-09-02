// ICM-20649/ICM-20948-compatible SPI IMU for ArduPilot's Invensense-v2
// backend. It implements banked register access, checked-register readback,
// reset/wake behavior, and a 1125 Hz FIFO containing stationary samples.
//
using System;
using System.Collections.Generic;
using Antmicro.Renode.Core;
using Antmicro.Renode.Peripherals.SPI;
using Antmicro.Renode.Peripherals.Timers;
using Antmicro.Renode.Time;

namespace Antmicro.Renode.Peripherals.Sensors
{
    public class AP_InvensenseV2 : ISPIPeripheral
    {
        public AP_InvensenseV2(IMachine machine, byte whoAmI = 0xEA)
        {
            this.whoAmI = whoAmI;
            registers = new byte[BankCount, RegisterCount];
            fifo = new Queue<byte>();
            sampleTimer = new LimitTimer(machine.ClockSource, 1000000, this, "invensense-v2 odr",
                                         limit: SamplePeriodUs, direction: Direction.Ascending,
                                         enabled: true, workMode: WorkMode.Periodic, eventEnabled: true);
            sampleTimer.LimitReached += OnSampleTick;
            Reset();
        }

        public void Reset()
        {
            ResetRegisters();
            transferByte = 0;
            reading = false;
            currentRegister = 0;
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
                if(currentRegister != FifoReadWrite)
                {
                    currentRegister = (byte)((currentRegister + 1) & RegisterMask);
                }
                return result;
            }

            WriteRegister(currentRegister, value);
            currentRegister = (byte)((currentRegister + 1) & RegisterMask);
            return 0;
        }

        public void FinishTransmission()
        {
            transferByte = 0;
        }

        private byte ReadRegister(byte register)
        {
            if(currentBank == 0)
            {
                switch(register)
                {
                case WhoAmI:
                    return whoAmI;
                case InterruptStatus1:
                    return fifo.Count > 0 ? (byte)1 : (byte)0;
                case FifoCountHigh:
                    return (byte)(fifo.Count >> 8);
                case FifoCountLow:
                    return (byte)fifo.Count;
                case FifoReadWrite:
                    return fifo.Count > 0 ? fifo.Dequeue() : (byte)0;
                }
            }
            return registers[currentBank, register];
        }

        private void WriteRegister(byte register, byte value)
        {
            if(register == BankSelect)
            {
                currentBank = (byte)((value >> 4) & 0x3);
                return;
            }
            if(currentBank == 0 && register == PowerManagement1 && (value & DeviceReset) != 0)
            {
                ResetRegisters();
                return;
            }
            if(currentBank == 0 && register == FifoReset && value != 0)
            {
                fifo.Clear();
            }
            if(currentBank == 0 && register == UserControl && (value & SramReset) != 0)
            {
                fifo.Clear();
                value = (byte)(value & ~SramReset);
            }
            registers[currentBank, register] = value;
        }

        private void ResetRegisters()
        {
            Array.Clear(registers, 0, registers.Length);
            fifo.Clear();
            currentBank = 0;
            registers[0, PowerManagement1] = Sleep;
        }

        private void OnSampleTick()
        {
            if((registers[0, UserControl] & FifoEnable) == 0 || registers[0, FifoEnable2] == 0)
            {
                return;
            }
            if(fifo.Count + SampleSize > FifoCapacity)
            {
                return;
            }
            PushWord(0);
            PushWord(0);
            PushWord(whoAmI == WhoAmIIcm20649 ? (short)1024 : (short)2048);
            PushWord(0);
            PushWord(0);
            PushWord(0);
            PushWord(0);
        }

        private void PushWord(short value)
        {
            fifo.Enqueue((byte)((value >> 8) & 0xFF));
            fifo.Enqueue((byte)value);
        }

        private readonly byte whoAmI;
        private readonly byte[,] registers;
        private readonly Queue<byte> fifo;
        private readonly LimitTimer sampleTimer;
        private byte currentBank;
        private byte currentRegister;
        private int transferByte;
        private bool reading;

        private const int BankCount = 4;
        private const int RegisterCount = 128;
        private const int SamplePeriodUs = 889;
        private const int SampleSize = 14;
        private const int FifoCapacity = 4096;

        private const byte ReadFlag = 0x80;
        private const byte RegisterMask = 0x7F;
        private const byte WhoAmI = 0x00;
        private const byte UserControl = 0x03;
        private const byte PowerManagement1 = 0x06;
        private const byte InterruptStatus1 = 0x1A;
        private const byte FifoEnable2 = 0x67;
        private const byte FifoReset = 0x68;
        private const byte FifoCountHigh = 0x70;
        private const byte FifoCountLow = 0x71;
        private const byte FifoReadWrite = 0x72;
        private const byte BankSelect = 0x7F;

        private const byte DeviceReset = 0x80;
        private const byte Sleep = 0x40;
        private const byte FifoEnable = 0x40;
        private const byte SramReset = 0x04;
        private const byte WhoAmIIcm20649 = 0xE1;
    }
}
