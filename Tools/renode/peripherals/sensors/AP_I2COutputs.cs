// Observable I2C RGB LED controllers for ArduPilot notification backends.
using System;
using Antmicro.Renode.Peripherals.I2C;

namespace Antmicro.Renode.Peripherals.Sensors
{
    public abstract class AP_I2CRegisterOutput : II2CPeripheral
    {
        protected AP_I2CRegisterOutput()
        {
            registers = new byte[256];
        }

        public abstract void Reset();

        public virtual void Write(byte[] data)
        {
            if(data.Length == 0)
            {
                return;
            }
            pointer = data[0];
            for(var index = 1; index < data.Length; index++)
            {
                WriteRegister((byte)pointer, data[index]);
                pointer = (byte)(pointer + 1);
            }
        }

        public virtual byte[] Read(int count = 1)
        {
            var result = new byte[count];
            for(var index = 0; index < count; index++)
            {
                result[index] = registers[(byte)pointer];
                pointer = (byte)(pointer + 1);
            }
            return result;
        }

        public void FinishTransmission()
        {
        }

        protected virtual void WriteRegister(byte register, byte value)
        {
            registers[register] = value;
            WriteCount++;
        }

        public int WriteCount { get; protected set; }

        protected readonly byte[] registers;
        protected int pointer;
    }

    public class AP_IS31FL3195 : AP_I2CRegisterOutput
    {
        public AP_IS31FL3195(byte address = 0x54)
        {
            this.address = address;
            Reset();
        }

        public override void Reset()
        {
            Array.Clear(registers, 0, registers.Length);
            registers[ProductId] = (byte)(address << 1);
            registers[ShutdownControl] = ShutdownReset;
            Red = Green = Blue = 0;
            Initialized = false;
            WriteCount = 0;
            pointer = 0;
        }

        protected override void WriteRegister(byte register, byte value)
        {
            WriteCount++;
            if(register == ResetRegister && value == Magic)
            {
                var writes = WriteCount;
                Reset();
                WriteCount = writes;
                return;
            }
            registers[register] = value;
            if(register == ShutdownControl)
            {
                Initialized = value == ShutdownEnabled;
            }
            if(register == ColorUpdate && value == Magic)
            {
                Red = registers[Out1];
                Green = registers[Out2];
                Blue = registers[Out3];
                UpdateCount++;
            }
        }

        public int Red { get; private set; }
        public int Green { get; private set; }
        public int Blue { get; private set; }
        public int UpdateCount { get; private set; }
        public bool Initialized { get; private set; }

        private readonly byte address;
        private const byte ProductId = 0x00;
        private const byte ShutdownControl = 0x01;
        private const byte Out1 = 0x10;
        private const byte Out2 = 0x21;
        private const byte Out3 = 0x32;
        private const byte ColorUpdate = 0x50;
        private const byte ResetRegister = 0x5F;
        private const byte Magic = 0xC5;
        private const byte ShutdownReset = 0xF0;
        private const byte ShutdownEnabled = 0xF1;
    }

    public class AP_LP5562 : AP_I2CRegisterOutput
    {
        public AP_LP5562()
        {
            Reset();
        }

        public override void Reset()
        {
            Array.Clear(registers, 0, registers.Length);
            registers[BlueCurrent] = CurrentReset;
            registers[GreenCurrent] = CurrentReset;
            registers[RedCurrent] = CurrentReset;
            WriteCount = 0;
            pointer = 0;
        }

        protected override void WriteRegister(byte register, byte value)
        {
            WriteCount++;
            if(register == ResetRegister && value == ResetValue)
            {
                var writes = WriteCount;
                Reset();
                WriteCount = writes;
                return;
            }
            registers[register] = value;
        }

        public int Red { get { return registers[RedPwm]; } }
        public int Green { get { return registers[GreenPwm]; } }
        public int Blue { get { return registers[BluePwm]; } }
        public bool Initialized
        {
            get
            {
                return registers[Enable] == EnableChip &&
                    registers[Configuration] == InternalClock &&
                    registers[LedMap] == DirectPwm;
            }
        }

        private const byte Enable = 0x00;
        private const byte BluePwm = 0x02;
        private const byte GreenPwm = 0x03;
        private const byte RedPwm = 0x04;
        private const byte BlueCurrent = 0x05;
        private const byte GreenCurrent = 0x06;
        private const byte RedCurrent = 0x07;
        private const byte Configuration = 0x08;
        private const byte ResetRegister = 0x0D;
        private const byte LedMap = 0x70;
        private const byte CurrentReset = 0xAF;
        private const byte ResetValue = 0xFF;
        private const byte EnableChip = 0x40;
        private const byte InternalClock = 0x01;
        private const byte DirectPwm = 0x00;
    }

    public class AP_NCP5623 : II2CPeripheral
    {
        public AP_NCP5623()
        {
            Reset();
        }

        public void Reset()
        {
            Red = Green = Blue = 0;
            Initialized = false;
            WriteCount = 0;
        }

        public void Write(byte[] data)
        {
            foreach(var value in data)
            {
                var command = value & CommandMask;
                var level = value & LevelMask;
                switch(command)
                {
                case Enable:
                    Initialized = level == LevelMask;
                    break;
                case RedPwm:
                    Red = level;
                    break;
                case GreenPwm:
                    Green = level;
                    break;
                case BluePwm:
                    Blue = level;
                    break;
                }
                WriteCount++;
            }
        }

        public byte[] Read(int count = 1)
        {
            return new byte[count];
        }

        public void FinishTransmission()
        {
        }

        public int Red { get; private set; }
        public int Green { get; private set; }
        public int Blue { get; private set; }
        public int WriteCount { get; private set; }
        public bool Initialized { get; private set; }

        private const byte CommandMask = 0xE0;
        private const byte LevelMask = 0x1F;
        private const byte Enable = 0x20;
        private const byte RedPwm = 0x40;
        private const byte GreenPwm = 0x60;
        private const byte BluePwm = 0x80;
    }

    public class AP_OreoLED : II2CPeripheral
    {
        public AP_OreoLED(byte address = BaseAddress)
        {
            Address = address;
            Reset();
        }

        public byte Address { get; set; }

        public void Reset()
        {
            Pattern = Red = Green = Blue = 0;
            AmplitudeRed = AmplitudeGreen = AmplitudeBlue = 0;
            Period = PhaseOffset = 0;
            Macro = 0;
            Initialized = false;
            WriteCount = 0;
            UpdateCount = 0;
            ChecksumErrors = 0;
        }

        public void Write(byte[] data)
        {
            WriteCount++;
            if(data.Length < 2 || !ValidChecksum(data))
            {
                ChecksumErrors++;
                return;
            }
            if(data.Length == 4 && data[0] == BootApplication &&
                data[1] == BootNonce && data[2] == Address)
            {
                Initialized = true;
                return;
            }

            Pattern = data[0];
            for(var index = 1; index + 1 < data.Length - 1; index += 2)
            {
                var parameter = data[index];
                var value = data[index + 1];
                switch(parameter)
                {
                case BiasRed:
                    Red = value;
                    break;
                case BiasGreen:
                    Green = value;
                    break;
                case BiasBlue:
                    Blue = value;
                    break;
                case AmplitudeRedParameter:
                    AmplitudeRed = value;
                    break;
                case AmplitudeGreenParameter:
                    AmplitudeGreen = value;
                    break;
                case AmplitudeBlueParameter:
                    AmplitudeBlue = value;
                    break;
                case MacroParameter:
                    Macro = value;
                    break;
                case PeriodParameter:
                    if(index + 2 < data.Length - 1)
                    {
                        Period = (data[index + 1] << 8) | data[index + 2];
                        index++;
                    }
                    break;
                case PhaseOffsetParameter:
                    if(index + 2 < data.Length - 1)
                    {
                        PhaseOffset = (data[index + 1] << 8) |
                            data[index + 2];
                        index++;
                    }
                    break;
                }
            }
            UpdateCount++;
        }

        public byte[] Read(int count = 1)
        {
            return new byte[count];
        }

        public void FinishTransmission()
        {
        }

        private bool ValidChecksum(byte[] data)
        {
            byte checksum = Address;
            foreach(var value in data)
            {
                checksum ^= value;
            }
            return checksum == 0;
        }

        public int Pattern { get; private set; }
        public int Red { get; private set; }
        public int Green { get; private set; }
        public int Blue { get; private set; }
        public int AmplitudeRed { get; private set; }
        public int AmplitudeGreen { get; private set; }
        public int AmplitudeBlue { get; private set; }
        public int Period { get; private set; }
        public int PhaseOffset { get; private set; }
        public int Macro { get; private set; }
        public int WriteCount { get; private set; }
        public int UpdateCount { get; private set; }
        public int ChecksumErrors { get; private set; }
        public bool Initialized { get; private set; }

        private const byte BaseAddress = 0x68;
        private const byte BootApplication = 0x60;
        private const byte BootNonce = 0xA2;
        private const byte BiasRed = 0;
        private const byte BiasGreen = 1;
        private const byte BiasBlue = 2;
        private const byte AmplitudeRedParameter = 3;
        private const byte AmplitudeGreenParameter = 4;
        private const byte AmplitudeBlueParameter = 5;
        private const byte PeriodParameter = 6;
        private const byte PhaseOffsetParameter = 8;
        private const byte MacroParameter = 9;
    }

    public class AP_OreoLED0 : AP_OreoLED
    {
        public AP_OreoLED0() : base(0x68)
        {
        }
    }

    public class AP_OreoLED1 : AP_OreoLED
    {
        public AP_OreoLED1() : base(0x69)
        {
        }
    }

    public class AP_OreoLED2 : AP_OreoLED
    {
        public AP_OreoLED2() : base(0x6A)
        {
        }
    }

    public class AP_OreoLED3 : AP_OreoLED
    {
        public AP_OreoLED3() : base(0x6B)
        {
        }
    }

    public class AP_PCA9685LED : AP_I2CRegisterOutput
    {
        public AP_PCA9685LED()
        {
            Reset();
        }

        public override void Reset()
        {
            Array.Clear(registers, 0, registers.Length);
            registers[Mode1] = Sleep;
            WriteCount = 0;
            pointer = 0;
        }

        public int Red { get { return ChannelLevel(RedChannel); } }
        public int Green { get { return ChannelLevel(GreenChannel); } }
        public int Blue { get { return ChannelLevel(BlueChannel); } }
        public bool Initialized
        {
            get
            {
                return (registers[Mode1] & AutoIncrement) != 0 &&
                    (registers[Mode1] & Sleep) == 0;
            }
        }

        private int ChannelLevel(byte channel)
        {
            var offLow = registers[channel + 2];
            var offHigh = registers[channel + 3] & 0x0F;
            return ((offHigh << 8) | offLow) / 16;
        }

        private const byte Mode1 = 0x00;
        private const byte BlueChannel = 0x06;
        private const byte GreenChannel = 0x0A;
        private const byte RedChannel = 0x0E;
        private const byte Sleep = 0x10;
        private const byte AutoIncrement = 0x20;
    }

    public class AP_ToshibaLED : AP_I2CRegisterOutput
    {
        public AP_ToshibaLED()
        {
            Reset();
        }

        public override void Reset()
        {
            Array.Clear(registers, 0, registers.Length);
            WriteCount = 0;
            pointer = 0;
        }

        public int Red { get { return registers[RedPwm] & 0x0F; } }
        public int Green { get { return registers[GreenPwm] & 0x0F; } }
        public int Blue { get { return registers[BluePwm] & 0x0F; } }
        public bool Initialized { get { return registers[Enable] == EnableValue; } }

        private const byte BluePwm = 0x01;
        private const byte GreenPwm = 0x02;
        private const byte RedPwm = 0x03;
        private const byte Enable = 0x04;
        private const byte EnableValue = 0x03;
    }
}
