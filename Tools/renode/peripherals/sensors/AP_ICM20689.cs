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
// Populate enough interpolated records to cover elapsed physics time when the
// driver reads FIFO_COUNTH. Renode's emulated F405 cannot drain a free-running
// 1kHz FIFO in real time; coupling production to the driver's poll preserves
// fresh lockstep feedback without accumulating stale motion data. Acceleration
// and angular-rate samples follow physics truth; temperature remains at 25C so
// the driver's raw-temperature consistency check does not trip.
//
using System;
using System.Collections.Generic;
using Antmicro.Renode.Core;
using Antmicro.Renode.Peripherals;
using Antmicro.Renode.Peripherals.SPI;
using Antmicro.Renode.Peripherals.Miscellaneous;

namespace Antmicro.Renode.Peripherals.Sensors
{
    // IGPIOReceiver: the chip-select pin. Renode's STM32SPI never calls
    // FinishTransmission, so CS deassert (input 0 going high) is the only
    // usable transaction framing - without it the first transfer works and
    // every later one is misparsed as a continuation.
    public class AP_ICM20689 : ISPIPeripheral, IGPIOReceiver
    {
        public AP_ICM20689(IMachine machine, byte whoAmI = 0x98, byte rotation = 0)
        {
            this.whoAmI = whoAmI;
            this.rotation = rotation;
            physics = AP_PhysicsState.ForMachine(machine);
            fifo = new Queue<byte>();
            registers = new byte[128];
            Reset();
        }

        public void Reset()
        {
            Array.Clear(registers, 0, registers.Length);
            fifo.Clear();
            previousAcceleration = null;
            previousGyro = null;
            previousTimestampUs = 0;
            transferByte = 0;
            reading = false;
            currentReg = 0;
            registers[WHOAMI] = whoAmI;
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
                fifo.Clear();
                if((registers[USER_CTRL] & FIFO_EN_BIT) != 0)
                {
                    var truth = physics.Current;
                    var acceleration = AP_SensorOrientation.BodyToSensor(
                        truth.SpecificForceMS2, rotation);
                    var gyro = AP_SensorOrientation.BodyToSensor(
                        truth.GyroRadS, rotation);
                    var elapsedUs = previousTimestampUs == 0 ||
                        truth.TimestampUs <= previousTimestampUs ?
                        NominalPollPeriodUs : truth.TimestampUs - previousTimestampUs;
                    var sampleCount = Math.Max(1, (int)Math.Min(
                        MaxSamplesPerPoll,
                        (elapsedUs + SamplePeriodUs / 2) / SamplePeriodUs));
                    if(previousAcceleration == null)
                    {
                        previousAcceleration = acceleration;
                        previousGyro = gyro;
                    }
                    for(var i = 0; i < sampleCount; i++)
                    {
                        var alpha = (i + 1.0f) / sampleCount;
                        QueueSample(
                            Interpolate(previousAcceleration, acceleration, alpha),
                            Interpolate(previousGyro, gyro, alpha));
                    }
                    previousAcceleration = acceleration;
                    previousGyro = gyro;
                    previousTimestampUs = truth.TimestampUs;
                }
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

        private void QueueSample(float[] acceleration, float[] gyro)
        {
            // The Invensense backend maps packet words to sensor axes as
            // (word1, word0, -word2), for both acceleration and gyro.
            PushWord(ScaleWord(acceleration[1], AccelScale));
            PushWord(ScaleWord(acceleration[0], AccelScale));
            PushWord(ScaleWord(-acceleration[2], AccelScale));
            PushWord(0);         // 25C, constant
            PushWord(ScaleWord(gyro[1], GyroScale));
            PushWord(ScaleWord(gyro[0], GyroScale));
            PushWord(ScaleWord(-gyro[2], GyroScale));
        }

        private static float[] Interpolate(float[] from, float[] to, float alpha)
        {
            return new float[] {
                from[0] + (to[0] - from[0]) * alpha,
                from[1] + (to[1] - from[1]) * alpha,
                from[2] + (to[2] - from[2]) * alpha,
            };
        }

        private static short ScaleWord(float value, double scale)
        {
            return (short)Math.Max(Int16.MinValue,
                Math.Min(Int16.MaxValue, Math.Round(value / scale)));
        }

        private void PushWord(short value)
        {
            fifo.Enqueue((byte)((value >> 8) & 0xFF));
            fifo.Enqueue((byte)(value & 0xFF));
        }

        private readonly Queue<byte> fifo;
        private readonly byte[] registers;
        private readonly byte whoAmI;
        private readonly byte rotation;
        private readonly AP_PhysicsState physics;
        private float[] previousAcceleration;
        private float[] previousGyro;
        private ulong previousTimestampUs;
        private int transferByte;
        private bool reading;
        private byte currentReg;

        private const ulong SamplePeriodUs = 1000;
        private const ulong NominalPollPeriodUs = 8000;
        private const ulong MaxSamplesPerPoll = 64;

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
        private const double Gravity = 9.80665;
        private const double AccelScale = Gravity / 2048.0;
        private const double GyroScale = Math.PI / 180.0 / 16.4;
    }
}
