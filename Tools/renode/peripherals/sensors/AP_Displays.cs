// Observable monochrome I2C displays used by AP_Notify.
using System;
using Antmicro.Renode.Peripherals.I2C;

namespace Antmicro.Renode.Peripherals.Sensors
{
    public abstract class AP_MonochromeDisplay : II2CPeripheral
    {
        protected AP_MonochromeDisplay(int columns)
        {
            Columns = columns;
            ram = new byte[columns * Pages];
            Reset();
        }

        public bool Initialized { get; protected set; }
        public uint DataWriteCount { get; protected set; }
        public uint ContentHash
        {
            get
            {
                uint hash = 2166136261;
                foreach(var value in ram)
                {
                    hash = (hash ^ value) * 16777619;
                }
                return hash;
            }
        }
        public uint LitPixels
        {
            get
            {
                uint result = 0;
                foreach(var value in ram)
                {
                    var bits = value;
                    while(bits != 0)
                    {
                        result += (uint)(bits & 1);
                        bits >>= 1;
                    }
                }
                return result;
            }
        }

        public virtual void Reset()
        {
            Array.Clear(ram, 0, ram.Length);
            Initialized = false;
            DataWriteCount = 0;
            page = 0;
            column = 0;
        }

        public byte[] Read(int count = 1)
        {
            return new byte[count];
        }

        public void Write(byte[] data)
        {
            if(data.Length == 0)
            {
                return;
            }
            if(data[0] == CommandControl)
            {
                HandleCommands(data, 1);
            }
            else if(data[0] == DataControl)
            {
                WriteData(data, 1);
            }
        }

        public void FinishTransmission()
        {
        }

        protected abstract void HandleCommands(byte[] data, int offset);

        protected void WriteData(byte[] data, int offset)
        {
            for(var index = offset; index < data.Length; index++)
            {
                if(page < Pages && column < Columns)
                {
                    ram[page * Columns + column] = data[index];
                }
                column++;
            }
            DataWriteCount++;
        }

        protected readonly int Columns;
        protected int page;
        protected int column;
        private readonly byte[] ram;

        protected const int Pages = 8;
        protected const byte CommandControl = 0x00;
        protected const byte DataControl = 0x40;
        protected const byte DisplayOn = 0xAF;
    }

    public class AP_SSD1306 : AP_MonochromeDisplay
    {
        public AP_SSD1306() : base(128)
        {
        }

        protected override void HandleCommands(byte[] data, int offset)
        {
            while(offset < data.Length)
            {
                var command = data[offset++];
                if(command == DisplayOn)
                {
                    Initialized = true;
                }
                else if(command == ColumnAddress && offset + 1 < data.Length)
                {
                    column = data[offset];
                    offset += 2;
                }
                else if(command == PageAddress && offset + 1 < data.Length)
                {
                    page = data[offset];
                    offset += 2;
                }
                else if(CommandHasArgument(command) && offset < data.Length)
                {
                    offset++;
                }
            }
        }

        private static bool CommandHasArgument(byte command)
        {
            switch(command)
            {
            case 0x20:
            case 0x81:
            case 0x8D:
            case 0xA8:
            case 0xD3:
            case 0xD5:
            case 0xD9:
            case 0xDA:
            case 0xDB:
                return true;
            default:
                return false;
            }
        }

        private const byte ColumnAddress = 0x21;
        private const byte PageAddress = 0x22;
    }

    public class AP_SH1106 : AP_MonochromeDisplay
    {
        public AP_SH1106() : base(132)
        {
        }

        protected override void HandleCommands(byte[] data, int offset)
        {
            while(offset < data.Length)
            {
                var command = data[offset++];
                if(command == DisplayOn)
                {
                    Initialized = true;
                }
                else if((command & 0xF0) == 0xB0)
                {
                    page = command & 0x0F;
                }
                else if((command & 0xF0) == 0x10)
                {
                    column = (column & 0x0F) | ((command & 0x0F) << 4);
                }
                else if((command & 0xF0) == 0x00)
                {
                    column = (column & 0xF0) | (command & 0x0F);
                }
                else if(CommandHasArgument(command) && offset < data.Length)
                {
                    offset++;
                }
            }
        }

        private static bool CommandHasArgument(byte command)
        {
            switch(command)
            {
            case 0x81:
            case 0xA8:
            case 0xAD:
            case 0xD3:
            case 0xD5:
            case 0xD9:
            case 0xDA:
            case 0xDB:
                return true;
            default:
                return false;
            }
        }
    }
}
