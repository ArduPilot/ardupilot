// Cypress FM25V02A-compatible SPI FRAM for AP_RAMTRON. The backing file is
// updated at the end of every write transaction, so parameter and mission
// storage survives resets and Renode restarts.
//
using System;
using System.IO;
using Antmicro.Renode.Core;
using Antmicro.Renode.Logging;
using Antmicro.Renode.Peripherals;
using Antmicro.Renode.Peripherals.SPI;

namespace Antmicro.Renode.Peripherals.Miscellaneous
{
    public class AP_RAMTRON : ISPIPeripheral, IGPIOReceiver
    {
        public AP_RAMTRON(string fileName)
        {
            this.fileName = fileName;
            contents = new byte[Capacity];
            for(var i = 0; i < contents.Length; i++)
            {
                contents[i] = 0xFF;
            }
            if(File.Exists(fileName))
            {
                var saved = File.ReadAllBytes(fileName);
                Array.Copy(saved, contents, Math.Min(saved.Length, contents.Length));
            }
            else
            {
                Save();
            }
            Reset();
        }

        public void Reset()
        {
            command = Command.None;
            transferByte = 0;
            address = 0;
            writeEnabled = false;
            writeOccurred = false;
        }

        public byte Transmit(byte value)
        {
            if(transferByte++ == 0)
            {
                command = (Command)value;
                if(command == Command.WriteEnable)
                {
                    writeEnabled = true;
                }
                return 0;
            }

            switch(command)
            {
            case Command.ReadId:
                return ReadId(transferByte - 2);
            case Command.ReadStatus:
                return writeEnabled ? WriteEnableLatch : (byte)0;
            case Command.Read:
            case Command.Write:
                if(transferByte <= AddressLength + 1)
                {
                    address = ((address << 8) | value) % Capacity;
                    return 0;
                }
                if(command == Command.Read)
                {
                    var result = contents[address];
                    address = (address + 1) % Capacity;
                    return result;
                }
                if(writeEnabled)
                {
                    contents[address] = value;
                    address = (address + 1) % Capacity;
                    writeOccurred = true;
                }
                return 0;
            default:
                return 0;
            }
        }

        public void FinishTransmission()
        {
            // ChibiOS can keep chip select asserted across multiple hardware
            // transfers. STM32 SPI models call this at each TSIZE boundary,
            // so only the GPIO chip-select edge may end a FRAM transaction.
        }

        public void OnGPIO(int number, bool value)
        {
            if(value)
            {
                EndTransaction();
            }
        }

        private void EndTransaction()
        {
            if(command == Command.Write)
            {
                if(writeOccurred)
                {
                    Save();
                }
                writeEnabled = false;
            }
            command = Command.None;
            transferByte = 0;
            address = 0;
            writeOccurred = false;
        }

        private byte ReadId(int index)
        {
            return index < DeviceId.Length ? DeviceId[index] : (byte)0;
        }

        private void Save()
        {
            try
            {
                var directory = Path.GetDirectoryName(fileName);
                if(!string.IsNullOrEmpty(directory))
                {
                    Directory.CreateDirectory(directory);
                }
                File.WriteAllBytes(fileName, contents);
            }
            catch(Exception error)
            {
                this.Log(LogLevel.Error, "Failed to persist FRAM to {0}: {1}", fileName, error.Message);
            }
        }

        private readonly string fileName;
        private readonly byte[] contents;
        private Command command;
        private int transferByte;
        private int address;
        private bool writeEnabled;
        private bool writeOccurred;

        // Cypress RDID layout: six manufacturer bytes, memory family, ID1,
        // ID2. AP_RAMTRON identifies the FM25V02A from the final two bytes.
        private static readonly byte[] DeviceId = {0, 0, 0, 0, 0, 0, 0, 0x22, 0x08};

        private const int Capacity = 32 * 1024;
        private const int AddressLength = 2;
        private const byte WriteEnableLatch = 1 << 1;

        private enum Command : byte
        {
            None = 0,
            Write = 0x02,
            Read = 0x03,
            ReadStatus = 0x05,
            WriteEnable = 0x06,
            ReadId = 0x9F,
        }
    }
}
