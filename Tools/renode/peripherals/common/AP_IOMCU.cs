// Minimal ArduPilot IO-MCU register protocol peer. It represents a healthy
// IO board with no RC input and keeps enough register state for FMU startup,
// safety handling, and output writes. The firmware CRC is supplied from the
// board's hwdef ROMFS entry, so the FMU does not enter the bootloader uploader.
//
using System;
using System.Collections.Generic;
using Antmicro.Renode.Core;
using Antmicro.Renode.Peripherals;
using Antmicro.Renode.Peripherals.Bus;
using Antmicro.Renode.Peripherals.UART;
using Antmicro.Renode.Time;

namespace Antmicro.Renode.Peripherals.Miscellaneous
{
    public class AP_IOMCU : IDoubleWordPeripheral, IKnownSize
    {
        public AP_IOMCU(IMachine machine, IUART uart, uint firmwareCrc)
        {
            this.machine = machine;
            this.uart = uart;
            this.firmwareCrc = firmwareCrc;
            request = new List<byte>();
            setup = new ushort[SetupRegisterCount];
            servos = new ushort[MaxChannels];
            uart.CharReceived += WriteChar;
            Reset();
        }

        public void Reset()
        {
            generation++;
            request.Clear();
            Array.Clear(setup, 0, setup.Length);
            Array.Clear(servos, 0, servos.Length);
            safetyOff = false;
            totalPackets = 0;
            setup[DefaultRateRegister] = 50;
            setup[AlternateRateRegister] = 200;
            setup[FirmwareCrcRegister] = (ushort)firmwareCrc;
            setup[FirmwareCrcRegister + 1] = (ushort)(firmwareCrc >> 16);
        }

        private void WriteChar(byte value)
        {
            request.Add(value);
            if(request.Count < HeaderSize)
            {
                return;
            }

            var code = (byte)(request[0] >> CodeShift);
            var count = request[0] & CountMask;
            // ChibiOS peers use a four-byte read request after discovering
            // the protocol version; older peers pad reads to the expected
            // reply size. Determine the form from this packet's header CRC
            // instead of remembering a previous config read. The FMU can
            // retry that config read in legacy form after a timeout.
            var compactRead = code == CodeRead && request.Count == HeaderSize &&
                ValidCrc(request);
            var expected = compactRead ? HeaderSize : HeaderSize + 2 * count;
            if(request.Count < expected)
            {
                return;
            }

            ProcessRequest(code, count, request[2], request[3]);
            request.Clear();
        }

        // AP_IOMCU is mapped in the synthetic Renode peripheral window only
        // so Renode instantiates it. The FMU exchanges data with it exclusively
        // through the UART supplied to the constructor.
        public uint ReadDoubleWord(long offset)
        {
            return 0;
        }

        public void WriteDoubleWord(long offset, uint value)
        {
        }

        public long Size => 4;

        private void ProcessRequest(byte code, int count, byte page, byte offset)
        {
            if(!ValidCrc(request))
            {
                SendReply(CodeCorrupt, page, offset, Array.Empty<ushort>());
                return;
            }

            totalPackets++;
            if(code == CodeRead)
            {
                var values = ReadRegisters(page, offset, count);
                SendReply(CodeSuccess, page, offset, values);
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
                case PageStatus:
                    result[i] = ReadStatus(register);
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

        private ushort ReadStatus(int register)
        {
            var elapsedUs = (ulong)machine.ElapsedVirtualTime.TimeElapsed.TotalMicroseconds;
            var timestampMs = (uint)Math.Max(1UL, elapsedUs / 1000);
            switch(register)
            {
            case 0:
                return 1024; // Free memory
            case 1:
            case 2:
                return 512; // Free thread stack
            // page_reg_status has one alignment word before timestamp_ms.
            case 4:
                return (ushort)timestampMs;
            case 5:
                return (ushort)(timestampMs >> 16);
            case 10:
                return (ushort)totalPackets;
            case 11:
                return (ushort)(totalPackets >> 16);
            case 16:
                return safetyOff ? (ushort)1 : (ushort)0;
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
                    if(register == ForceSafetyOffRegister && value == ForceSafetyMagic)
                    {
                        safetyOff = true;
                    }
                    else if(register == ForceSafetyOnRegister && value == ForceSafetyMagic)
                    {
                        safetyOff = false;
                    }
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
            // A real IO MCU cannot start its reply until the request has left
            // the FMU UART, and then shifts the reply one frame at a time.
            // Injecting the whole reply synchronously from the final request
            // byte overruns the STM32 receive path before DMA can consume it.
            var scheduledGeneration = generation;
            for(var i = 0; i < reply.Length; i++)
            {
                var value = reply[i];
                machine.ScheduleAction(TimeInterval.FromMicroseconds(
                    ReplyDelayUs + (uint)i * FrameDelayUs),
                    _ =>
                    {
                        if(scheduledGeneration == generation)
                        {
                            uart.WriteChar(value);
                        }
                    }, name: "AP IOMCU reply");
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

        private readonly IMachine machine;
        private readonly IUART uart;
        private readonly uint firmwareCrc;
        private readonly List<byte> request;
        private readonly ushort[] setup;
        private readonly ushort[] servos;
        private bool safetyOff;
        private uint totalPackets;
        private uint generation;
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
        private const byte PageStatus = 1;
        private const byte PageServos = 3;
        private const byte PageSetup = 50;
        private const byte PageDirectPwm = 54;

        private const int DefaultRateRegister = 3;
        private const int AlternateRateRegister = 4;
        private const int FirmwareCrcRegister = 11;
        private const int ForceSafetyOffRegister = 12;
        private const int ForceSafetyOnRegister = 14;
        private const ushort ForceSafetyMagic = 22027;
        private const ushort ProtocolVersion = 4;
        private const ushort ProtocolVersion2 = 10;
        private const uint ReplyDelayUs = 7;
        private const uint FrameDelayUs = 7;
    }
}
