// Minimal ArduPilot IO-MCU register protocol peer. It represents a healthy
// IO board with no RC input and keeps enough register state for FMU startup,
// safety handling, and output writes. The firmware CRC is supplied from the
// board's hwdef ROMFS entry, so the FMU does not enter the bootloader uploader.
//
using System;
using System.Collections.Generic;
using Antmicro.Migrant;
using Antmicro.Renode.Peripherals;
using Antmicro.Renode.Peripherals.Bus;
using Antmicro.Renode.Peripherals.UART;

namespace Antmicro.Renode.Peripherals.Miscellaneous
{
    public class AP_IOMCU : IUART, IDoubleWordPeripheral, IKnownSize
    {
        public AP_IOMCU(uint firmwareCrc)
        {
            this.firmwareCrc = firmwareCrc;
            request = new List<byte>();
            setup = new ushort[SetupRegisterCount];
            servos = new ushort[MaxChannels];
            Reset();
        }

        public void Reset()
        {
            request.Clear();
            compactReads = false;
            Array.Clear(setup, 0, setup.Length);
            Array.Clear(servos, 0, servos.Length);
            setup[DefaultRateRegister] = 50;
            setup[AlternateRateRegister] = 200;
            setup[FirmwareCrcRegister] = (ushort)firmwareCrc;
            setup[FirmwareCrcRegister + 1] = (ushort)(firmwareCrc >> 16);
        }

        public void WriteChar(byte value)
        {
            request.Add(value);
            if(request.Count < HeaderSize)
            {
                return;
            }

            var code = (byte)(request[0] >> CodeShift);
            var count = request[0] & CountMask;
            var expected = code == CodeRead && compactReads
                ? HeaderSize : HeaderSize + 2 * count;
            if(request.Count < expected)
            {
                return;
            }

            ProcessRequest(code, count, request[2], request[3]);
            request.Clear();
        }

        // AP_IOMCU is mapped in the synthetic Renode peripheral window only
        // to give the UART connector a registered endpoint. The FMU exchanges
        // data with it exclusively through IUART.
        public uint ReadDoubleWord(long offset)
        {
            return 0;
        }

        public void WriteDoubleWord(long offset, uint value)
        {
        }

        public long Size => 4;

        public uint BaudRate => 1500000;
        public Parity ParityBit => Parity.None;
        public Bits StopBits => Bits.One;

        [field: Transient]
        public event Action<byte> CharReceived;

        private void ProcessRequest(byte code, int count, byte page, byte offset)
        {
            if(!ValidCrc(request))
            {
                SendReply(CodeCorrupt, page, offset, Array.Empty<ushort>());
                return;
            }

            if(code == CodeRead)
            {
                var values = ReadRegisters(page, offset, count);
                SendReply(CodeSuccess, page, offset, values);
                if(page == PageConfig)
                {
                    compactReads = true;
                }
                return;
            }

            if(code == CodeWrite)
            {
                WriteRegisters(page, offset, count);
                SendReply(CodeSuccess, 0, 0, Array.Empty<ushort>());
                return;
            }

            SendReply(CodeError, 0, 0, Array.Empty<ushort>());
        }

        private ushort[] ReadRegisters(byte page, byte offset, int count)
        {
            var result = new ushort[count];
            for(var i = 0; i < count; i++)
            {
                var register = offset + i;
                switch(page)
                {
                case PageConfig:
                    result[i] = ReadConfig(register);
                    break;
                case PageSetup:
                    result[i] = register < setup.Length ? setup[register] : (ushort)0;
                    break;
                case PageServos:
                    result[i] = register < servos.Length ? servos[register] : (ushort)0;
                    break;
                default:
                    result[i] = 0;
                    break;
                }
            }
            return result;
        }

        private ushort ReadConfig(int register)
        {
            switch(register)
            {
            case 0:
                return ProtocolVersion;
            case 1:
                return ProtocolVersion2;
            case 2:
                return 0x6410; // STM32F103 medium-density MCU ID, low half
            case 3:
                return 0x2003;
            case 4:
                return 0xC231; // Cortex-M3 CPUID, low half
            case 5:
                return 0x410F;
            default:
                return 0;
            }
        }

        private void WriteRegisters(byte page, byte offset, int count)
        {
            for(var i = 0; i < count; i++)
            {
                var index = HeaderSize + 2 * i;
                var value = (ushort)(request[index] | (request[index + 1] << 8));
                var register = offset + i;
                if(page == PageSetup && register < setup.Length)
                {
                    setup[register] = value;
                }
                else if(page == PageDirectPwm && register < servos.Length)
                {
                    servos[register] = value;
                }
            }
        }

        private void SendReply(byte code, byte page, byte offset, ushort[] registers)
        {
            var reply = new byte[HeaderSize + 2 * registers.Length];
            reply[0] = (byte)((code << CodeShift) | registers.Length);
            reply[2] = page;
            reply[3] = offset;
            for(var i = 0; i < registers.Length; i++)
            {
                reply[HeaderSize + 2 * i] = (byte)registers[i];
                reply[HeaderSize + 2 * i + 1] = (byte)(registers[i] >> 8);
            }
            reply[1] = Crc8(reply);
            foreach(var value in reply)
            {
                CharReceived?.Invoke(value);
            }
        }

        private static bool ValidCrc(List<byte> packet)
        {
            var saved = packet[1];
            packet[1] = 0;
            var crc = Crc8(packet);
            packet[1] = saved;
            return saved == crc;
        }

        private static byte Crc8(IEnumerable<byte> bytes)
        {
            byte crc = 0;
            foreach(var value in bytes)
            {
                crc ^= value;
                for(var bit = 0; bit < 8; bit++)
                {
                    crc = (byte)((crc & 0x80) != 0 ? (crc << 1) ^ 0x07 : crc << 1);
                }
            }
            return crc;
        }

        private readonly uint firmwareCrc;
        private readonly List<byte> request;
        private readonly ushort[] setup;
        private readonly ushort[] servos;
        private bool compactReads;

        private const int HeaderSize = 4;
        private const int MaxChannels = 16;
        private const int SetupRegisterCount = 28;
        private const int CodeShift = 6;
        private const int CountMask = 0x3F;

        private const byte CodeRead = 0;
        private const byte CodeWrite = 1;
        private const byte CodeSuccess = 0;
        private const byte CodeCorrupt = 1;
        private const byte CodeError = 2;

        private const byte PageConfig = 0;
        private const byte PageServos = 3;
        private const byte PageSetup = 50;
        private const byte PageDirectPwm = 54;

        private const int DefaultRateRegister = 3;
        private const int AlternateRateRegister = 4;
        private const int FirmwareCrcRegister = 11;
        private const ushort ProtocolVersion = 4;
        private const ushort ProtocolVersion2 = 10;
    }
}
