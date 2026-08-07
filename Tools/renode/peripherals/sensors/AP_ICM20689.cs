//
// ICM-20689 IMU on SPI, as AP_InertialSensor_Invensense drives it:
// WHOAMI 0x98, PWR_MGMT_1 reset self-clears and the CLK_ZGYRO wake
// value must read back verbatim (the probe's pass/fail gate), all
// configuration registers read back what was written (the driver
// re-reads its 13 checked registers every 20 transfers and resets on
// any mismatch), and samples arrive through the FIFO: big-endian
// int16 records of accel XYZ, temp, gyro XYZ (14 bytes), count in
// FIFO_COUNTH/L, streamed from non-incrementing FIFO_R_W. All other
// registers are plain read/write storage.
//
// A LimitTimer fills the FIFO at 1kHz while USER_CTRL.FIFO_EN is set.
// Bench values: level and still - accel (0, 0, +1g raw, which the
// driver's axis swap turns into z = -9.81), gyro zero, temp raw 0
// (= 25C for this part, and constant so the driver's raw-temp
// consistency check never trips).
//
using System;
using System.Collections.Generic;
using Antmicro.Renode.Core;
using Antmicro.Renode.Logging;
using Antmicro.Renode.Peripherals;
using Antmicro.Renode.Peripherals.SPI;
using Antmicro.Renode.Peripherals.Timers;
using Antmicro.Renode.Time;

namespace Antmicro.Renode.Peripherals.Sensors
{
    // IGPIOReceiver: the chip-select pin. Renode's STM32SPI never calls
    // FinishTransmission, so CS deassert (input 0 going high) is the only
    // usable transaction framing - without it the first transfer works and
    // every later one is misparsed as a continuation.
    public class AP_ICM20689 : ISPIPeripheral, IGPIOReceiver
    {
        public AP_ICM20689(IMachine machine)
        {
            fifo = new Queue<byte>();
            registers = new byte[128];
            sampleTimer = new LimitTimer(machine.ClockSource, 1000000, this, "icm20689 odr",
                                         limit: SamplePeriodUs, direction: Antmicro.Renode.Time.Direction.Ascending,
                                         enabled: true, workMode: WorkMode.Periodic, eventEnabled: true);
            sampleTimer.LimitReached += OnSampleTick;
            Reset();
        }

        public void Reset()
        {
            Array.Clear(registers, 0, registers.Length);
            fifo.Clear();
            transferByte = 0;
            reading = false;
            currentReg = 0;
            registers[WHOAMI] = 0x98;
            registers[PRODUCT_ID] = 0x01;
        }

        public byte Transmit(byte value)
        {
            byte response = 0;
            if(transferByte == 0)
            {
                reading = (value & 0x80) != 0;
                currentReg = (byte)(value & 0x7F);
            }
            else if(reading)
            {
                response = ReadRegister(currentReg);
                if(currentReg != FIFO_R_W)
                {
                    currentReg = (byte)((currentReg + 1) & 0x7F);
                }
            }
            else
            {
                WriteRegister(currentReg, value);
                if(currentReg != FIFO_R_W)
                {
                    currentReg = (byte)((currentReg + 1) & 0x7F);
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
                // CS deasserted: transaction over
                transferByte = 0;
            }
        }

        private byte ReadRegister(byte reg)
        {
            switch(reg)
            {
            case FIFO_COUNTH:
                return (byte)(fifo.Count >> 8);
            case FIFO_COUNTL:
                return (byte)(fifo.Count & 0xFF);
            case FIFO_R_W:
                return fifo.Count > 0 ? fifo.Dequeue() : (byte)0;
            case INT_STATUS:
                return 0x01; // RAW_RDY: a sample is always available
            default:
                return registers[reg];
            }
        }

        private void WriteRegister(byte reg, byte value)
        {
            switch(reg)
            {
            case PWR_MGMT_1:
                // DEVICE_RESET self-clears; everything else reads back
                registers[reg] = (byte)(value & 0x7F);
                break;
            case USER_CTRL:
                if((value & FIFO_RESET) != 0)
                {
                    fifo.Clear();
                }
                registers[reg] = (byte)(value & ~FIFO_RESET); // FIFO_RESET self-clears
                break;
            case FIFO_R_W:
                break; // writes to the FIFO are not used by the driver
            default:
                registers[reg] = value;
                break;
            }
        }

        private void OnSampleTick()
        {
            if((registers[USER_CTRL] & FIFO_EN_BIT) == 0)
            {
                return;
            }
            if(fifo.Count + SampleSize > FifoCapacity)
            {
                return; // full: real part would overwrite; driver resets anyway
            }
            // accel X, Y, Z / temp / gyro X, Y, Z - big-endian int16.
            // +1g on raw Z; the driver maps z = -raw_z.
            PushWord(0);
            PushWord(0);
            PushWord(2048);      // 1g at the 16g full-scale the driver configures
            PushWord(0);         // 25C, constant
            PushWord(0);
            PushWord(0);
            PushWord(0);
        }

        private void PushWord(short value)
        {
            fifo.Enqueue((byte)((value >> 8) & 0xFF));
            fifo.Enqueue((byte)(value & 0xFF));
        }

        private readonly Queue<byte> fifo;
        private readonly byte[] registers;
        private readonly LimitTimer sampleTimer;
        private int transferByte;
        private bool reading;
        private byte currentReg;

        private const int SamplePeriodUs = 1000;
        private const int SampleSize = 14;
        private const int FifoCapacity = 512;

        private const byte PRODUCT_ID = 0x0C;
        private const byte INT_STATUS = 0x3A;
        private const byte USER_CTRL = 0x6A;
        private const byte PWR_MGMT_1 = 0x6B;
        private const byte FIFO_COUNTH = 0x72;
        private const byte FIFO_COUNTL = 0x73;
        private const byte FIFO_R_W = 0x74;
        private const byte WHOAMI = 0x75;

        private const byte FIFO_RESET = 0x04;
        private const byte FIFO_EN_BIT = 0x40;
    }
}
