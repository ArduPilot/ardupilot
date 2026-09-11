// Reusable 8-bit register-pointer I2C peripheral for ArduPilot sensor models.
using System;
using Antmicro.Renode.Peripherals.I2C;

namespace Antmicro.Renode.Peripherals.Sensors
{
    public abstract class AP_I2CRegisterDevice : II2CPeripheral
    {
        protected AP_I2CRegisterDevice()
        {
            Registers = new byte[RegisterCount];
        }

        public virtual void Reset()
        {
            Array.Clear(Registers, 0, Registers.Length);
            Pointer = 0;
        }

        public virtual void Write(byte[] data)
        {
            if(data.Length == 0)
            {
                return;
            }
            Pointer = data[0];
            for(var index = 1; index < data.Length; index++)
            {
                LastWriteRegister = Pointer;
                LastWriteValue = data[index];
                WriteCount++;
                if(!IgnoreWrites)
                {
                    WriteRegister(Pointer, data[index]);
                }
                Pointer = (Pointer + 1) & RegisterMask;
            }
        }

        public virtual byte[] Read(int count = 1)
        {
            var result = new byte[count];
            for(var index = 0; index < count; index++)
            {
                LastReadRegister = Pointer;
                ReadCount++;
                result[index] = (byte)(ReadRegister(Pointer) ^ ReadXorMask);
                Pointer = (Pointer + 1) & RegisterMask;
            }
            return result;
        }

        public void FinishTransmission()
        {
        }

        public byte ReadXorMask { get; set; }
        public bool IgnoreWrites { get; set; }
        public ulong ReadCount { get; private set; }
        public ulong WriteCount { get; private set; }
        public int LastReadRegister { get; private set; }
        public int LastWriteRegister { get; private set; }
        public byte LastWriteValue { get; private set; }

        protected virtual byte ReadRegister(int register)
        {
            return Registers[register];
        }

        protected virtual void WriteRegister(int register, byte value)
        {
            Registers[register] = value;
        }

        protected void WriteU16(int register, ushort value)
        {
            Registers[register] = (byte)(value >> 8);
            Registers[register + 1] = (byte)value;
        }

        protected void WriteS16(int register, short value)
        {
            WriteU16(register, (ushort)value);
        }

        protected void WriteU16LE(int register, ushort value)
        {
            Registers[register] = (byte)value;
            Registers[register + 1] = (byte)(value >> 8);
        }

        protected void WriteS16LE(int register, short value)
        {
            WriteU16LE(register, (ushort)value);
        }

        protected void WriteU24LE(int register, uint value)
        {
            Registers[register] = (byte)value;
            Registers[register + 1] = (byte)(value >> 8);
            Registers[register + 2] = (byte)(value >> 16);
        }

        protected readonly byte[] Registers;
        protected int Pointer;

        private const int RegisterCount = 256;
        private const int RegisterMask = RegisterCount - 1;
    }
}
