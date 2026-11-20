//
// ICM-42688-P IMU as AP_InertialSensor_Invensensev3 drives it. The
// model supplies banked register storage, the 0x47 product ID, and
// 16/20-byte little-endian FIFO records at the configured ODR, scaled for
// the full range of the part given by whoAmI. The ICM-42670 register map
// (MREG indirect access, 1.6kHz) is used when whoAmI is 0x67, and the
// ICM-45686/56686 map (IREG indirect access) for 0xE9/0x08. Acceleration
// and angular-rate samples follow physics truth after applying the board's
// sensor rotation; temperature follows the board thermal model.
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
            icm42670 = whoAmI == WhoAmIIcm42670;
            icm456 = whoAmI == WhoAmIIcm45686 || whoAmI == WhoAmIIcm56686;
            // ICM-56686 shifts the 456xy user registers below WHO_AM_I up by 4
            reg456Offset = (byte)(whoAmI == WhoAmIIcm56686 ? 4 : 0);
            fifoData = icm42670 ? Icm42670FifoData :
                icm456 ? (byte)(Icm456FifoData + reg456Offset) : FifoData;
            var accelRangeG = 16.0;
            var gyroRangeDps = 2000.0;
            if(whoAmI == WhoAmIIcm40609 || whoAmI == WhoAmIIim42653 || icm456)
            {
                accelRangeG = 32.0;
            }
            if(whoAmI == WhoAmIIim42653 || icm456)
            {
                gyroRangeDps = 4000.0;
            }
            accelScale = Gravity * accelRangeG / 32768.0;
            gyroScale = Math.PI / 180.0 * gyroRangeDps / 32768.0;
            accelHighResolutionScale = Gravity * accelRangeG / 524288.0;
            gyroHighResolutionScale = Math.PI / 180.0 * gyroRangeDps / 524288.0;
            // FIFO temperature LSB/C, 8 bit and high resolution 16 bit
            var halfDegree = icm42670 || icm456;
            temperatureSensitivity = whoAmI == WhoAmIIcm40605 ? 115.49 / 128.0 : halfDegree ? 2.0 : 2.07;
            highResolutionTemperatureSensitivity = halfDegree ? 128.0 : 132.48;
            iregs = new Dictionary<ushort, byte>();
            physics = AP_PhysicsState.ForMachine(machine);
            thermal = AP_BoardThermalState.ForMachine(machine);
            fifo = new Queue<byte>();
            registers = new byte[BankCount, RegisterCount];
            mregs = new byte[MregBankCount, RegisterCount];
            tickPeriodUs = icm42670 ? (ulong)Icm42670SamplePeriodUs : SamplePeriodUs;
            sampleTimer = new LimitTimer(machine.ClockSource, 1000000, this, "icm42688 odr",
                                         limit: tickPeriodUs, direction: Direction.Ascending,
                                         enabled: true, workMode: WorkMode.Periodic, eventEnabled: true);
            sampleTimer.LimitReached += OnSampleTick;
            Reset();
        }

        public void Reset()
        {
            Array.Clear(registers, 0, registers.Length);
            Array.Clear(mregs, 0, mregs.Length);
            fifo.Clear();
            transferByte = 0;
            currentBank = 0;
            currentRegister = 0;
            reading = false;
            timestamp = 0;
            pendingSamples = 0;
            iregs.Clear();
            if(icm456)
            {
                registers[0, Icm45686WhoAmI] = whoAmI;
                registers[0, Icm456RegMisc2] = IregDone;
                registers[0, Reg456(Icm456Int1Status0)] = ResetDone;
                registers[0, Reg456(Icm456FifoConfig2)] = 0x20;
                registers[0, Reg456(Icm456GyroConfig0)] = 0x06;
                return;
            }
            registers[0, WhoAmI] = whoAmI;
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
                if(currentRegister != fifoData)
                {
                    currentRegister = (byte)((currentRegister + 1) & RegisterMask);
                }
            }
            else
            {
                WriteRegister(currentRegister, value);
                if(currentRegister != fifoData)
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
            if(icm42670)
            {
                return ReadRegister42670(register);
            }
            if(icm456)
            {
                return ReadRegister456(register);
            }
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
            if(icm42670)
            {
                WriteRegister42670(register, value);
                return;
            }
            if(icm456)
            {
                WriteRegister456(register, value);
                return;
            }
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

        private byte ReadRegister42670(byte register)
        {
            switch(register)
            {
            case Icm42670MclkReady:
                return (registers[0, Icm42670PowerManagement] & SensorsLowNoise) != 0 ? MclkReady : (byte)0;
            case Icm42670FifoCountLow:
                return (byte)(fifo.Count / CurrentSampleSize);
            case Icm42670FifoCountHigh:
                return (byte)((fifo.Count / CurrentSampleSize) >> 8);
            case Icm42670FifoData:
                return fifo.Count > 0 ? fifo.Dequeue() : (byte)0;
            case Icm42670MregRead:
                return mregs[MregBank(registers[0, Icm42670BlockSelectRead]), registers[0, Icm42670MregAddressRead] & RegisterMask];
            }
            return registers[0, register];
        }

        private void WriteRegister42670(byte register, byte value)
        {
            switch(register)
            {
            case Icm42670SignalPathReset:
                // soft reset and FIFO flush both empty the FIFO
                fifo.Clear();
                return;
            case Icm42670FifoData:
                return;
            case Icm42670MregWrite:
                mregs[MregBank(registers[0, Icm42670BlockSelectWrite]), registers[0, Icm42670MregAddressWrite] & RegisterMask] = value;
                return;
            }
            registers[0, register] = value;
        }

        private byte Reg456(byte register)
        {
            return (byte)(register + reg456Offset);
        }

        private ushort IregAddress =>
            (ushort)(registers[0, Icm456IregAddressHigh] << 8 | registers[0, Icm456IregAddressLow]);

        private byte ReadRegister456(byte register)
        {
            if(register == Reg456(Icm456FifoCountLow))
            {
                return (byte)(fifo.Count / CurrentSampleSize);
            }
            if(register == Reg456(Icm456FifoCountHigh))
            {
                return (byte)((fifo.Count / CurrentSampleSize) >> 8);
            }
            if(register == fifoData)
            {
                return fifo.Count > 0 ? fifo.Dequeue() : (byte)0;
            }
            if(register == Reg456(Icm456Int1Status0))
            {
                // read to clear
                var status = registers[0, register];
                registers[0, register] = 0;
                return status;
            }
            if(register == Icm456IregData)
            {
                byte value;
                return iregs.TryGetValue(IregAddress, out value) ? value : (byte)0;
            }
            return registers[0, register];
        }

        private void WriteRegister456(byte register, byte value)
        {
            if(register == Icm456RegMisc2)
            {
                if((value & SoftReset) != 0)
                {
                    Reset();
                }
                return;
            }
            if(register == fifoData)
            {
                return;
            }
            if(register == Icm456IregData)
            {
                iregs[IregAddress] = value;
                return;
            }
            if(register == Reg456(Icm456FifoConfig2) && (value & Icm456FifoFlush) != 0)
            {
                fifo.Clear();
                value &= unchecked((byte)~Icm456FifoFlush);
            }
            registers[0, register] = value;
            if(register == Reg456(Icm456FifoConfig0) && (value & Icm456FifoModeMask) == 0)
            {
                // bypass mode empties the FIFO
                fifo.Clear();
            }
        }

        // output data rate from GYRO_CONFIG0
        private int OdrHz
        {
            get
            {
                if(icm42670)
                {
                    return 1600;
                }
                if(icm456)
                {
                    var code = registers[0, Reg456(Icm456GyroConfig0)] & 0x0F;
                    return code >= 3 && code <= 12 ? 6400 >> (code - 3) : 800;
                }
                var odr = registers[0, GyroConfig0] & 0x0F;
                if(odr >= 1 && odr <= 6)
                {
                    return 32000 >> (odr - 1);
                }
                return odr == 0x0F ? 500 : 1000;
            }
        }

        // MREG1/2/3 are selected by block values 0x00/0x28/0x50
        private static int MregBank(byte block)
        {
            return Math.Min(block / 0x28, MregBankCount - 1);
        }

        private bool SamplingEnabled
        {
            get
            {
                if(icm42670)
                {
                    return (registers[0, Icm42670PowerManagement] & SensorsLowNoise) == SensorsLowNoise &&
                           (mregs[0, Icm42670FifoConfig5] & Icm42670FifoSensorsEnabled) == Icm42670FifoSensorsEnabled;
                }
                if(icm456)
                {
                    return (registers[0, Reg456(Icm456PowerManagement)] & SensorsLowNoise) == SensorsLowNoise &&
                           (registers[0, Reg456(Icm456FifoConfig0)] & Icm456FifoModeMask) != 0 &&
                           (registers[0, Reg456(Icm456FifoConfig3)] & Icm456FifoSensorsEnabled) == Icm456FifoSensorsEnabled;
                }
                return (registers[0, FifoConfig1] & FifoSensorsEnabled) == FifoSensorsEnabled &&
                       (registers[0, PowerManagement] & SensorsLowNoise) == SensorsLowNoise;
            }
        }

        private void OnSampleTick()
        {
            if(!SamplingEnabled)
            {
                pendingSamples = 0;
                return;
            }
            // the tick is fixed, so push as many samples as the ODR gives per tick
            pendingSamples += OdrHz * (double)tickPeriodUs / 1000000.0;
            while(pendingSamples >= 1.0)
            {
                pendingSamples -= 1.0;
                if(fifo.Count + CurrentSampleSize > FifoCapacity)
                {
                    continue;
                }
                if(HighResolutionEnabled)
                {
                    PushHighResolutionSample();
                }
                else
                {
                    PushSample();
                }
            }
        }

        private void PushSample()
        {
            fifo.Enqueue(FifoHeader);
            var truth = physics.Current;
            var acceleration = AP_SensorOrientation.BodyToSensor(truth.SpecificForceMS2, rotation);
            var gyro = AP_SensorOrientation.BodyToSensor(truth.GyroRadS, rotation);
            PushWord(ScaleSampleWord(acceleration[0], accelScale));
            PushWord(ScaleSampleWord(acceleration[1], accelScale));
            PushWord(ScaleSampleWord(acceleration[2], accelScale));
            PushWord(ScaleSampleWord(gyro[0], gyroScale));
            PushWord(ScaleSampleWord(gyro[1], gyroScale));
            PushWord(ScaleSampleWord(gyro[2], gyroScale));
            fifo.Enqueue((byte)(sbyte)Math.Max(sbyte.MinValue, Math.Min(sbyte.MaxValue,
                Math.Round((thermal.TemperatureC - 25.0) * temperatureSensitivity))));
            PushWord((short)timestamp++);
        }

        private void PushHighResolutionSample()
        {
            fifo.Enqueue(FifoHighResolutionHeader);
            var truth = physics.Current;
            var acceleration = AP_SensorOrientation.BodyToSensor(truth.SpecificForceMS2, rotation);
            var gyro = AP_SensorOrientation.BodyToSensor(truth.GyroRadS, rotation);
            var values = new int[] {
                ScaleHighResolution(acceleration[0], accelHighResolutionScale),
                ScaleHighResolution(acceleration[1], accelHighResolutionScale),
                ScaleHighResolution(acceleration[2], accelHighResolutionScale),
                ScaleHighResolution(gyro[0], gyroHighResolutionScale),
                ScaleHighResolution(gyro[1], gyroHighResolutionScale),
                ScaleHighResolution(gyro[2], gyroHighResolutionScale),
            };
            foreach(var value in values)
            {
                fifo.Enqueue((byte)(value >> 4));
                fifo.Enqueue((byte)(value >> 12));
            }
            PushWord(ScaleWord((float)(thermal.TemperatureC - 25.0),
                1.0 / highResolutionTemperatureSensitivity));
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

        private short ScaleSampleWord(float value, double scale)
        {
            var minimum = icm456 ? -32767 : Int16.MinValue;
            return (short)Math.Max(minimum,
                Math.Min(Int16.MaxValue, Math.Round(value / scale)));
        }

        private int ScaleHighResolution(float value, double scale)
        {
            // 456xy uses upper 16 bits 0x8000 as its invalid marker, so
            // 0x80000 through 0x8000F are not valid saturated samples.
            var minimum = icm456 ? -524272 : -524288;
            return (int)Math.Max(minimum,
                Math.Min(524287, Math.Round(value / scale)));
        }

        private void PushWord(short value)
        {
            fifo.Enqueue((byte)(value & 0xFF));
            fifo.Enqueue((byte)((value >> 8) & 0xFF));
        }

        private bool HighResolutionEnabled => icm42670 ?
            (mregs[0, Icm42670FifoConfig5] & Icm42670FifoHighResolutionEnable) != 0 :
            icm456 ? (registers[0, Reg456(Icm456FifoConfig3)] & Icm456FifoHighResolutionEnable) != 0 :
            (registers[0, FifoConfig1] & FifoHighResolutionEnable) != 0;
        private int CurrentSampleSize => HighResolutionEnabled ? HighResolutionSampleSize : SampleSize;

        private readonly Queue<byte> fifo;
        private readonly byte[,] registers;
        private readonly byte[,] mregs;
        private readonly bool icm42670;
        private readonly bool icm456;
        private readonly byte reg456Offset;
        private readonly Dictionary<ushort, byte> iregs;
        private readonly ulong tickPeriodUs;
        private readonly double accelScale;
        private readonly double gyroScale;
        private readonly double accelHighResolutionScale;
        private readonly double gyroHighResolutionScale;
        private readonly double temperatureSensitivity;
        private readonly double highResolutionTemperatureSensitivity;
        private double pendingSamples;
        private readonly byte fifoData;
        private readonly LimitTimer sampleTimer;
        private readonly byte whoAmI;
        private readonly byte rotation;
        private readonly AP_PhysicsState physics;
        private readonly AP_BoardThermalState thermal;
        private int transferByte;
        private byte currentBank;
        private byte currentRegister;
        private bool reading;
        private ushort timestamp;

        private const int BankCount = 5;
        private const int RegisterCount = 128;
        private const int SamplePeriodUs = 1000;
        private const int Icm42670SamplePeriodUs = 625;
        private const int MregBankCount = 3;
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

        private const byte Icm42670MclkReady = 0x00;
        private const byte Icm42670SignalPathReset = 0x02;
        private const byte Icm42670PowerManagement = 0x1F;
        private const byte Icm42670FifoCountLow = 0x3D;
        private const byte Icm42670FifoCountHigh = 0x3E;
        private const byte Icm42670FifoData = 0x3F;
        private const byte Icm42670BlockSelectWrite = 0x79;
        private const byte Icm42670MregAddressWrite = 0x7A;
        private const byte Icm42670MregWrite = 0x7B;
        private const byte Icm42670BlockSelectRead = 0x7C;
        private const byte Icm42670MregAddressRead = 0x7D;
        private const byte Icm42670MregRead = 0x7E;
        private const byte Icm42670FifoConfig5 = 0x01;
        private const byte Icm42670FifoSensorsEnabled = 0x03;
        private const byte Icm42670FifoHighResolutionEnable = 0x08;
        private const byte MclkReady = 0x08;

        private const byte ReadFlag = 0x80;
        private const byte RegisterMask = 0x7F;
        private const byte BankMask = 0x07;
        private const byte DefaultWhoAmI = 0x47;
        private const byte WhoAmIIcm42670 = 0x67;
        private const byte WhoAmIIcm45686 = 0xE9;
        private const byte DataReady = 0x08;
        private const byte FifoFlush = 0x02;
        private const byte FifoSensorsEnabled = 0x07;
        private const byte FifoHighResolutionEnable = 0x10;
        private const byte SensorsLowNoise = 0x0F;
        private const byte FifoHeader = 0x68;
        private const byte FifoHighResolutionHeader = 0x78;
        private const double Gravity = 9.80665;

        private const byte GyroConfig0 = 0x4F;
        private const byte WhoAmIIcm40605 = 0x33;
        private const byte WhoAmIIcm40609 = 0x3B;
        private const byte WhoAmIIim42653 = 0x56;
        private const byte WhoAmIIcm56686 = 0x08;

        // ICM-45686 addresses; ICM-56686 adds reg456Offset below 0x72
        private const byte Icm456PowerManagement = 0x10;
        private const byte Icm456FifoCountLow = 0x12;
        private const byte Icm456FifoCountHigh = 0x13;
        private const byte Icm456FifoData = 0x14;
        private const byte Icm456Int1Status0 = 0x19;
        private const byte Icm456GyroConfig0 = 0x1C;
        private const byte Icm456FifoConfig0 = 0x1D;
        private const byte Icm456FifoConfig2 = 0x20;
        private const byte Icm456FifoConfig3 = 0x21;
        private const byte Icm456IregAddressHigh = 0x7C;
        private const byte Icm456IregAddressLow = 0x7D;
        private const byte Icm456IregData = 0x7E;
        private const byte Icm456RegMisc2 = 0x7F;
        private const byte Icm456FifoModeMask = 0xC0;
        private const byte Icm456FifoFlush = 0x80;
        private const byte Icm456FifoSensorsEnabled = 0x07;
        private const byte Icm456FifoHighResolutionEnable = 0x08;
        private const byte ResetDone = 0x80;
        private const byte IregDone = 0x01;
        private const byte SoftReset = 0x02;
    }
}
