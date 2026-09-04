// ICM-20649/ICM-20948-compatible SPI IMU for ArduPilot's Invensense-v2
// backend. It implements banked register access, checked-register readback,
// reset/wake behavior, and a 1125 Hz FIFO containing physics-driven samples.
// The ICM-20948 variant also carries an AK09916 compass reachable through
// the I2C master slave registers and EXT_SLV_SENS_DATA.
//
using System;
using System.Collections.Generic;
using Antmicro.Renode.Core;
using Antmicro.Renode.Peripherals.Miscellaneous;
using Antmicro.Renode.Peripherals.SPI;
using Antmicro.Renode.Peripherals.Timers;
using Antmicro.Renode.Time;

namespace Antmicro.Renode.Peripherals.Sensors
{
    public class AP_InvensenseV2 : ISPIPeripheral
    {
        public AP_InvensenseV2(IMachine machine, byte whoAmI = 0xEA,
                              byte rotation = 0)
        {
            this.whoAmI = whoAmI;
            this.rotation = rotation;
            physics = AP_PhysicsState.ForMachine(machine);
            registers = new byte[BankCount, RegisterCount];
            fifo = new Queue<byte>();
            if(whoAmI == WhoAmIIcm20948)
            {
                // AK09916 on the auxiliary I2C master; the hwdef COMPASS
                // rotation is applied through AuxiliaryCompassRotation
                auxiliaryCompass = new AP_AK09916(machine)
                {
                    CompensateExternalProbeRotation = false,
                };
            }
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
                if(reading && currentBank == 0 && IsExternalSensorData(currentRegister))
                {
                    // snapshot slave data once per transfer so a block read is consistent
                    UpdateAuxiliaryReads();
                }
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

        public byte AuxiliaryCompassRotation
        {
            get { return auxiliaryCompass == null ? (byte)0 : auxiliaryCompass.Rotation; }
            set
            {
                if(auxiliaryCompass != null)
                {
                    auxiliaryCompass.Rotation = value;
                }
            }
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
            if(currentBank == 3 && IsAuxiliaryControl(register))
            {
                ServiceAuxiliarySlave((register - I2cSlave0Control) / I2cSlaveStride);
            }
        }

        private void ResetRegisters()
        {
            Array.Clear(registers, 0, registers.Length);
            fifo.Clear();
            currentBank = 0;
            registers[0, PowerManagement1] = Sleep;
            auxiliaryCompass?.Reset();
        }

        private static bool IsExternalSensorData(byte register)
        {
            return register >= ExternalSensorData && register < ExternalSensorData + ExternalSensorDataSize;
        }

        private bool IsAuxiliaryControl(byte register)
        {
            return register >= I2cSlave0Control && register <= I2cSlave3Control &&
                (register - I2cSlave0Control) % I2cSlaveStride == 0;
        }

        private void UpdateAuxiliaryReads()
        {
            var outputOffset = 0;
            for(var slave = 0; slave < AuxiliarySlaveCount; slave++)
            {
                var baseRegister = I2cSlave0Address + slave * I2cSlaveStride;
                var address = registers[3, baseRegister];
                var control = registers[3, baseRegister + 2];
                if((control & I2cSlaveEnabled) == 0 || (address & I2cRead) == 0)
                {
                    continue;
                }
                ReadAuxiliaryDevice(
                    address, registers[3, baseRegister + 1], control & I2cTransferSizeMask,
                    outputOffset);
                outputOffset += control & I2cTransferSizeMask;
            }
        }

        private void ServiceAuxiliarySlave(int slave)
        {
            var baseRegister = I2cSlave0Address + slave * I2cSlaveStride;
            var address = registers[3, baseRegister];
            var control = registers[3, baseRegister + 2];
            if((control & I2cSlaveEnabled) == 0)
            {
                return;
            }
            var transferSize = control & I2cTransferSizeMask;
            if((address & I2cRead) != 0)
            {
                UpdateAuxiliaryReads();
                return;
            }
            if(auxiliaryCompass != null && (address & I2cAddressMask) == AuxiliaryCompassAddress &&
               transferSize == 1)
            {
                auxiliaryCompass.Write(new byte[] {
                    registers[3, baseRegister + 1], registers[3, baseRegister + 3]
                });
            }
        }

        private void ReadAuxiliaryDevice(byte address, byte register, int count, int outputOffset)
        {
            if(auxiliaryCompass == null || (address & I2cAddressMask) != AuxiliaryCompassAddress ||
               outputOffset + count > ExternalSensorDataSize)
            {
                return;
            }
            auxiliaryCompass.Write(new byte[] { register });
            var data = auxiliaryCompass.Read(count);
            for(var index = 0; index < data.Length; index++)
            {
                registers[0, ExternalSensorData + outputOffset + index] = data[index];
            }
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
            var truth = physics.Current;
            var acceleration = AP_SensorOrientation.BodyToSensor(
                truth.SpecificForceMS2, rotation);
            var gyro = AP_SensorOrientation.BodyToSensor(
                truth.GyroRadS, rotation);
            var accelScale = Gravity /
                (whoAmI == WhoAmIIcm20649 ? 1024.0 : 2048.0);
            PushWord(ScaleWord(acceleration[1], accelScale));
            PushWord(ScaleWord(acceleration[0], accelScale));
            PushWord(ScaleWord(-acceleration[2], accelScale));
            PushWord(ScaleWord(gyro[1], GyroScale));
            PushWord(ScaleWord(gyro[0], GyroScale));
            PushWord(ScaleWord(-gyro[2], GyroScale));
            PushWord(0);
        }

        private static short ScaleWord(float value, double scale)
        {
            return (short)Math.Max(Int16.MinValue,
                Math.Min(Int16.MaxValue, Math.Round(value / scale)));
        }

        private void PushWord(short value)
        {
            fifo.Enqueue((byte)((value >> 8) & 0xFF));
            fifo.Enqueue((byte)value);
        }

        private readonly byte whoAmI;
        private readonly byte rotation;
        private readonly AP_PhysicsState physics;
        private readonly byte[,] registers;
        private readonly Queue<byte> fifo;
        private readonly AP_AK09916 auxiliaryCompass;
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
        private const int AuxiliarySlaveCount = 4;
        private const int I2cSlaveStride = 4;
        private const int ExternalSensorDataSize = 24;

        private const byte ReadFlag = 0x80;
        private const byte RegisterMask = 0x7F;
        private const byte WhoAmI = 0x00;
        private const byte UserControl = 0x03;
        private const byte PowerManagement1 = 0x06;
        private const byte InterruptStatus1 = 0x1A;
        private const byte ExternalSensorData = 0x3B;
        private const byte FifoEnable2 = 0x67;
        private const byte FifoReset = 0x68;
        private const byte FifoCountHigh = 0x70;
        private const byte FifoCountLow = 0x71;
        private const byte FifoReadWrite = 0x72;
        private const byte BankSelect = 0x7F;
        private const byte I2cSlave0Address = 0x03;
        private const byte I2cSlave0Control = 0x05;
        private const byte I2cSlave3Control = 0x11;

        private const byte DeviceReset = 0x80;
        private const byte Sleep = 0x40;
        private const byte FifoEnable = 0x40;
        private const byte SramReset = 0x04;
        private const byte I2cRead = 0x80;
        private const byte I2cSlaveEnabled = 0x80;
        private const byte I2cAddressMask = 0x7F;
        private const byte I2cTransferSizeMask = 0x0F;
        private const byte AuxiliaryCompassAddress = 0x0C;
        private const byte WhoAmIIcm20948 = 0xEA;
        private const byte WhoAmIIcm20649 = 0xE1;
        private const double Gravity = 9.80665;
        private const double GyroScale = Math.PI / 180.0 / 16.4;
    }
}
