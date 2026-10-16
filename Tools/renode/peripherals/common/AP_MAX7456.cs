// MAX7456/AT7456E analog OSD for AP_OSD_MAX7456. Each SPI command is an
// address byte then a data byte; reads use the address with bit 7 set. The
// font NVM is kept in a file so the firmware only uploads its font once,
// and Screen shows the display memory as text.
//
using System;
using System.IO;
using System.Text;
using Antmicro.Renode.Core;
using Antmicro.Renode.Logging;
using Antmicro.Renode.Peripherals;
using Antmicro.Renode.Peripherals.SPI;

namespace Antmicro.Renode.Peripherals.Miscellaneous
{
    public class AP_MAX7456 : ISPIPeripheral, IGPIOReceiver
    {
        public AP_MAX7456(string fileName = null)
        {
            this.fileName = fileName;
            for(var i = 0; i < nvm.Length; i++)
            {
                nvm[i] = 0xFF;
            }
            if(!string.IsNullOrEmpty(fileName) && File.Exists(fileName))
            {
                try
                {
                    var saved = File.ReadAllBytes(fileName);
                    Array.Copy(saved, nvm, Math.Min(saved.Length, nvm.Length));
                }
                catch(IOException error)
                {
                    this.Log(LogLevel.Warning, "cannot read {0}: {1}", fileName, error.Message);
                }
            }
            Reset();
        }

        // input video standard: PAL, NTSC or none
        public string Video { get; set; } = "PAL";

        public string Screen
        {
            get
            {
                var text = new StringBuilder();
                var rows = (registers[VM0] & VideoModePal) != 0 ? PalRows : NtscRows;
                for(var row = 0; row < rows; row++)
                {
                    for(var column = 0; column < Columns; column++)
                    {
                        var c = display[row * Columns + column];
                        text.Append(c >= 0x20 && c < 0x7F ? (char)c : ' ');
                    }
                    text.Append('\n');
                }
                return text.ToString();
            }
        }

        public void Reset()
        {
            Array.Clear(registers, 0, registers.Length);
            Array.Clear(display, 0, display.Length);
            Array.Clear(attributes, 0, attributes.Length);
            Array.Clear(characterBuffer, 0, characterBuffer.Length);
            displayAddress = 0;
            FinishTransmission();
        }

        public byte Transmit(byte value)
        {
            if(!haveAddress)
            {
                address = value;
                haveAddress = true;
                return 0;
            }
            haveAddress = false;
            if((address & ReadFlag) != 0)
            {
                return Read((byte)(address & ~ReadFlag));
            }
            Write(address, value);
            return 0;
        }

        public void FinishTransmission()
        {
            haveAddress = false;
        }

        public void OnGPIO(int number, bool value)
        {
            if(value)
            {
                FinishTransmission();
            }
        }

        private byte Read(byte register)
        {
            switch(register)
            {
            case Stat:
                return Status();
            case Dmdo:
                return display[displayAddress % DisplaySize];
            case Cmdo:
                return characterBuffer[registers[Cmal] % CharacterSize];
            default:
                return register < registers.Length ? registers[register] : (byte)0;
            }
        }

        private void Write(byte register, byte value)
        {
            switch(register)
            {
            case VM0:
                if((value & SoftwareReset) != 0)
                {
                    Reset();
                    return;
                }
                break;
            case Dmm:
                if((value & ClearDisplay) != 0)
                {
                    Array.Clear(display, 0, display.Length);
                    Array.Clear(attributes, 0, attributes.Length);
                    value = (byte)(value & ~ClearDisplay);
                }
                break;
            case Dmah:
                displayAddress = (displayAddress & 0xFF) | ((value & 1) << 8);
                break;
            case Dmal:
                displayAddress = (displayAddress & 0x100) | value;
                break;
            case Dmdi:
                WriteDisplay(value);
                return;
            case Cmdi:
                characterBuffer[registers[Cmal] % CharacterSize] = value;
                return;
            case Cmm:
                CharacterMemory(value);
                return;
            }
            if(register < registers.Length)
            {
                registers[register] = value;
            }
        }

        private void WriteDisplay(byte value)
        {
            var autoIncrement = (registers[Dmm] & AutoIncrement) != 0;
            if(autoIncrement && value == 0xFF)
            {
                // 0xFF ends auto-increment mode
                registers[Dmm] = (byte)(registers[Dmm] & ~AutoIncrement);
                return;
            }
            var index = displayAddress % DisplaySize;
            if((registers[Dmah] & AttributeByte) != 0)
            {
                attributes[index] = value;
            }
            else
            {
                display[index] = value;
            }
            if(autoIncrement)
            {
                displayAddress = (displayAddress + 1) % DisplaySize;
            }
        }

        private void CharacterMemory(byte command)
        {
            var offset = registers[Cmah] * CharacterSize;
            if(command == ReadNvm)
            {
                Array.Copy(nvm, offset, characterBuffer, 0, CharacterSize);
            }
            else if(command == WriteNvm)
            {
                Array.Copy(characterBuffer, 0, nvm, offset, CharacterSize);
                Save();
            }
        }

        private byte Status()
        {
            switch(Video.ToUpperInvariant())
            {
            case "PAL":
                return StatPal;
            case "NTSC":
                return StatNtsc;
            default:
                return StatLossOfSync;
            }
        }

        private void Save()
        {
            if(string.IsNullOrEmpty(fileName))
            {
                return;
            }
            try
            {
                File.WriteAllBytes(fileName, nvm);
            }
            catch(IOException error)
            {
                this.Log(LogLevel.Warning, "cannot write {0}: {1}", fileName, error.Message);
            }
        }

        private readonly string fileName;
        private readonly byte[] registers = new byte[0x70];
        private readonly byte[] display = new byte[DisplaySize];
        private readonly byte[] attributes = new byte[DisplaySize];
        private readonly byte[] characterBuffer = new byte[CharacterSize];
        private readonly byte[] nvm = new byte[256 * CharacterSize];
        private int displayAddress;
        private byte address;
        private bool haveAddress;

        private const byte ReadFlag = 0x80;
        private const byte VM0 = 0x00;
        private const byte Dmm = 0x04;
        private const byte Dmah = 0x05;
        private const byte Dmal = 0x06;
        private const byte Dmdi = 0x07;
        private const byte Cmm = 0x08;
        private const byte Cmah = 0x09;
        private const byte Cmal = 0x0A;
        private const byte Cmdi = 0x0B;
        // read addresses without the read flag
        private const byte Stat = 0x20;
        private const byte Dmdo = 0x30;
        private const byte Cmdo = 0x40;

        private const byte SoftwareReset = 0x02;
        private const byte VideoModePal = 0x40;
        private const byte AutoIncrement = 0x01;
        private const byte ClearDisplay = 0x04;
        private const byte AttributeByte = 0x02;
        private const byte ReadNvm = 0x50;
        private const byte WriteNvm = 0xA0;
        private const byte StatPal = 0x01;
        private const byte StatNtsc = 0x02;
        private const byte StatLossOfSync = 0x04;

        private const int Columns = 30;
        private const int PalRows = 16;
        private const int NtscRows = 13;
        private const int DisplaySize = 480;
        private const int CharacterSize = 54;
    }
}
