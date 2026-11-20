//
// Invensense MPU6000/MPU6500/MPU9250/ICM2060x/ICM20689/ICM20789 IMU on SPI,
// as AP_InertialSensor_Invensense drives it: WHOAMI selects the part
// (default 0x98, ICM-20689), PWR_MGMT_1 reset self-clears and the CLK_ZGYRO wake
// value must read back verbatim (the probe's pass/fail gate), all
// configuration registers read back what was written (the driver
// re-reads its 13 checked registers every 20 transfers and resets on
// any mismatch), and samples arrive through the FIFO: big-endian
// int16 records of accel XYZ, temp, gyro XYZ (14 bytes), count in
// FIFO_COUNTH/L, streamed from non-incrementing FIFO_R_W. All other
// registers are plain read/write storage. Scaling follows ACCEL_CONFIG and
// GYRO_CONFIG (ICM-20601 ranges are doubled).
//
// A 1kHz virtual-time tick fills the FIFO at the configured rate: 8kHz with
// the DLPF bypassed (DLPF_CFG 0/7, fast sampling), 32kHz with FCHOICE_B set,
// else 1kHz / (1 + SMPLRT_DIV). The FIFO has the part's size and, when full,
// drops new records (CONFIG FIFO_MODE) or the oldest. Samples follow physics
// truth; FIFO and TEMP_OUT report the same board temperature so the driver's
// raw-temperature check passes.
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
            thermal = AP_BoardThermalState.ForMachine(machine);
            fifo = new Queue<byte>();
            registers = new byte[128];
            sampleTimer = new LimitTimer(machine.ClockSource, TickRateHz * 8, this, "invensense odr",
                                         limit: 8, direction: Direction.Ascending,
                                         enabled: true, workMode: WorkMode.Periodic, eventEnabled: true);
            sampleTimer.LimitReached += OnSampleTick;
            Reset();
        }

        public void Reset()
        {
            Array.Clear(registers, 0, registers.Length);
            fifo.Clear();
            rateAccumulator = 0;
            latchedCount = 0;
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
                // latch so COUNTL matches even if a tick lands between them
                latchedCount = fifo.Count;
                return (byte)(latchedCount >> 8);
            case FIFO_COUNTL:
                return (byte)(latchedCount & 0xFF);
            case FIFO_R_W:
                return fifo.Count > 0 ? fifo.Dequeue() : (byte)0;
            case INT_STATUS:
                return 0x01; // RAW_RDY: a sample is always available
            case TEMP_OUT_H:
                return (byte)(TemperatureRaw() >> 8);
            case TEMP_OUT_L:
                return (byte)TemperatureRaw();
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
                    rateAccumulator = 0;
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
            if((registers[USER_CTRL] & FIFO_EN_BIT) == 0 || registers[FIFO_EN] == 0)
            {
                return;
            }
            rateAccumulator += FifoRateHz();
            var samples = rateAccumulator / TickRateHz;
            rateAccumulator %= TickRateHz;
            if(samples == 0)
            {
                return;
            }
            var truth = physics.Current;
            var acceleration = AP_SensorOrientation.BodyToSensor(
                truth.SpecificForceMS2, rotation);
            var gyro = AP_SensorOrientation.BodyToSensor(
                truth.GyroRadS, rotation);
            var capacity = FifoCapacity();
            for(var i = 0; i < samples; i++)
            {
                if(fifo.Count + SampleSize > capacity)
                {
                    if((registers[CONFIG] & FIFO_MODE_STOP) != 0)
                    {
                        break;
                    }
                    for(var b = 0; b < SampleSize && fifo.Count > 0; b++)
                    {
                        fifo.Dequeue();
                    }
                }
                QueueSample(acceleration, gyro);
            }
        }

        private int FifoRateHz()
        {
            var div = 1 + registers[SMPLRT_DIV];
            if(whoAmI != WHOAMI_MPU6000 && (registers[GYRO_CONFIG] & 3) != 0)
            {
                return 32000; // FCHOICE_B: DLPF and SMPLRT_DIV bypassed
            }
            var dlpf = registers[CONFIG] & 7;
            if(dlpf == 0 || dlpf == 7)
            {
                // SMPLRT_DIV only divides the 8kHz rate on the MPU6000
                return whoAmI == WHOAMI_MPU6000 ? 8000 / div : 8000;
            }
            return 1000 / div;
        }

        private int FifoCapacity()
        {
            switch(whoAmI)
            {
            case WHOAMI_MPU6000:
                return 1024;
            case 0x12: // ICM20602
                return 1008;
            case 0x98: // ICM20689, ICM20789: ACCEL_CONFIG2 FIFO_SIZE
            case 0x02:
            case 0x03:
                return 512 << (registers[ACCEL_CONFIG2] >> 6);
            default:
                return 512;
            }
        }

        private void QueueSample(float[] acceleration, float[] gyro)
        {
            // The Invensense backend maps packet words to sensor axes as
            // (word1, word0, -word2), for both acceleration and gyro.
            var accelScale = AccelScale();
            var gyroScale = GyroScale();
            PushWord(ScaleWord(acceleration[1], accelScale));
            PushWord(ScaleWord(acceleration[0], accelScale));
            PushWord(ScaleWord(-acceleration[2], accelScale));
            PushWord(TemperatureRaw());
            PushWord(ScaleWord(gyro[1], gyroScale));
            PushWord(ScaleWord(gyro[0], gyroScale));
            PushWord(ScaleWord(-gyro[2], gyroScale));
        }

        // m/s^2 per LSB for ACCEL_FS_SEL: +/-2g..16g, ICM-20601 +/-4g..32g
        private double AccelScale()
        {
            var lsbPerG = (whoAmI == WHOAMI_ICM20601 ? 8192.0 : 16384.0) /
                (1 << ((registers[ACCEL_CONFIG] >> 3) & 3));
            return Gravity / lsbPerG;
        }

        // rad/s per LSB for FS_SEL: 250..2000dps, ICM-20601 500..4000dps;
        // 16.4 LSB/dps at 2000dps as the driver decodes it
        private double GyroScale()
        {
            var lsbPerDps = (whoAmI == WHOAMI_ICM20601 ? 65.6 : 131.2) /
                (1 << ((registers[GYRO_CONFIG] >> 3) & 3));
            return Math.PI / 180.0 / lsbPerDps;
        }

        // zero and LSB/C from AP_InertialSensor_Invensense for each part
        private short TemperatureRaw()
        {
            double zero = 25.0, scale = 1.0 / 326.8;
            switch(whoAmI)
            {
            case 0x98: // ICM20689
                scale = 0.003;
                break;
            case 0x68: // MPU6000
            case 0x70: // MPU6500
                zero = 36.53;
                scale = 1.0 / 340;
                break;
            case 0x02: // ICM20789
            case 0x03:
                scale = 0.003;
                break;
            case 0x71: // MPU9250
            case 0x73: // MPU9255
                zero = 21.0;
                scale = 1.0 / 340;
                break;
            }
            return ScaleWord((float)(thermal.TemperatureC - zero), scale);
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
        private readonly AP_BoardThermalState thermal;
        private readonly LimitTimer sampleTimer;
        private int rateAccumulator;
        private int latchedCount;
        private int transferByte;
        private bool reading;
        private byte currentReg;

        private const int TickRateHz = 1000;
        private const int SampleSize = 14;

        private const byte PRODUCT_ID = 0x0C;
        private const byte SMPLRT_DIV = 0x19;
        private const byte CONFIG = 0x1A;
        private const byte GYRO_CONFIG = 0x1B;
        private const byte ACCEL_CONFIG = 0x1C;
        private const byte ACCEL_CONFIG2 = 0x1D;
        private const byte FIFO_EN = 0x23;
        private const byte INT_STATUS = 0x3A;
        private const byte TEMP_OUT_H = 0x41;
        private const byte TEMP_OUT_L = 0x42;
        private const byte USER_CTRL = 0x6A;
        private const byte PWR_MGMT_1 = 0x6B;
        private const byte FIFO_COUNTH = 0x72;
        private const byte FIFO_COUNTL = 0x73;
        private const byte FIFO_R_W = 0x74;
        private const byte WHOAMI = 0x75;

        private const byte FIFO_RESET = 0x04;
        private const byte FIFO_EN_BIT = 0x40;
        private const byte FIFO_MODE_STOP = 0x40;
        private const byte WHOAMI_MPU6000 = 0x68;
        private const double Gravity = 9.80665;
        private const byte WHOAMI_ICM20601 = 0xAC;
    }
}
