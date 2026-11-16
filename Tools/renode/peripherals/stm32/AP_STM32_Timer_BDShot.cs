//
// Bidirectional DShot bridge for one STM32 timer group.
//
// Outbound DShot frames are decoded from ArduPilot's interleaved TIMx_DMAR
// buffer when the update DMA stream is enabled and sent to an external ESC
// simulator over UDP. Replies are turned back into input-capture edge
// timestamps and written through the capture DMA stream the firmware armed,
// so the real RCOutput_bdshot receive and decode path runs unmodified.
//
// The wire format is the AM32 SITL / ESCSim signal protocol, so an AM32
// SITL, an ESCSim bench or a scripted test ESC can be attached per motor.
// Little endian:
//   u16 magic 0x4453
//   u8  type: 1=DSHOT150, 2=DSHOT300, 3=DSHOT600
//   u8  len: 4
//   u16 flags: bit0 = idle high (inverted, bidirectional DShot)
//   u16 data: 16 bit DShot frame
// A reply uses the same format with data the GCR-decoded 16 bit telemetry
// frame (12 bit eRPM/EDT value and 4 bit CRC).
//
// Renode's STM32 timer does not implement DCR, so the capture channel is
// taken from the one channel ArduPilot leaves enabled in input mode, and its
// CCxS selection says which pin of the pair it samples.
//
// The ESC runs in another process, so a reply is used by the next capture
// on that channel, once. A capture with no reply since the previous one
// is silent, as on hardware.
using System;
using System.Collections.Generic;
using System.Net;
using System.Net.Sockets;
using System.Runtime.CompilerServices;
using System.Threading;

using Antmicro.Renode.Core;
using Antmicro.Renode.Exceptions;
using Antmicro.Renode.Logging;
using Antmicro.Renode.Peripherals;
using Antmicro.Renode.Peripherals.Bus;
using Antmicro.Renode.Peripherals.DMA;
using Antmicro.Renode.Peripherals.Timers;
using Antmicro.Renode.Time;

namespace Antmicro.Renode.Peripherals.Miscellaneous
{
    public sealed class AP_STM32_Timer_BDShot : IDoubleWordPeripheral, IKnownSize, IDisposable
    {
        // streamN encodes (controller - 1) * 8 + stream, or -1 when unused.
        // escNPort is the ESC UDP port for timer channel N, or 0 when unused.
        public AP_STM32_Timer_BDShot(IMachine machine, STM32_Timer timer,
            uint timerAddress, STM32DMA dma1, STM32DMA dma2,
            int stream1 = -1, int stream2 = -1, int stream3 = -1,
            int stream4 = -1, int stream5 = -1,
            int esc1Port = 0, int esc2Port = 0, int esc3Port = 0, int esc4Port = 0)
        {
            this.machine = machine;
            this.timer = timer;
            dmarAddress = timerAddress + DmaAddressForFullTransfer;
            controllers = new[] { dma1, dma2 };
            var ports = new[] { esc1Port, esc2Port, esc3Port, esc4Port };
            for(var channel = 0; channel < Channels; channel++)
            {
                if(ports[channel] <= 0)
                {
                    continue;
                }
                var socket = new UdpClient(new IPEndPoint(IPAddress.Loopback, 0));
                socket.Connect(IPAddress.Loopback, ports[channel]);
                socket.Client.ReceiveTimeout = ReceiveTimeoutMs;
                sockets[channel] = socket;
                var captured = channel;
                new Thread(() => Receive(captured))
                {
                    IsBackground = true,
                    Name = string.Format("BDShot 0x{0:X} ch{1} reply", timerAddress, channel + 1),
                }.Start();
            }
            foreach(var stream in new[] { stream1, stream2, stream3, stream4, stream5 })
            {
                if(stream < 0)
                {
                    continue;
                }
                var dma = controllers[stream / StreamsPerController];
                if(dma == null)
                {
                    throw new ConstructionException(string.Format(
                        "DMA controller for stream {0} is not connected", stream));
                }
                Action<uint> listener = value => StreamWritten(dma, stream % StreamsPerController, value);
                registrations.Add((Subscribe(dma, stream % StreamsPerController, listener), listener));
            }
        }

        public void Reset()
        {
            lock(sync)
            {
                Array.Clear(replies, 0, replies.Length);
            }
            framesSent = 0;
            repliesReceived = 0;
            repliesInjected = 0;
            capturesSilent = 0;
        }

        public uint ReadDoubleWord(long offset)
        {
            switch(offset)
            {
            case 0x00: return FramesSent;
            case 0x04: return RepliesReceived;
            case 0x08: return RepliesInjected;
            case 0x0C: return CapturesSilent;
            default:
                if(offset >= LastFrameBase && offset < LastFrameBase + Channels * 4)
                {
                    return lastFrames[(offset - LastFrameBase) / 4];
                }
                return 0;
            }
        }

        public void WriteDoubleWord(long offset, uint value)
        {
        }

        public long Size => 0x20;

        public uint FramesSent => (uint)framesSent;
        public uint RepliesReceived => (uint)repliesReceived;
        public uint RepliesInjected => (uint)repliesInjected;
        public uint CapturesSilent => (uint)capturesSilent;

        public void Dispose()
        {
            disposed = true;
            foreach(var (listeners, listener) in registrations)
            {
                lock(listeners)
                {
                    listeners.Remove(listener);
                }
            }
            foreach(var socket in sockets)
            {
                socket?.Close();
            }
        }

        // Renode allows one before-write hook per register, and timers can
        // share a stream, so hooks are dispatched from a shared table. The
        // table is weak so a cleared machine's controllers can be collected.
        private static List<Action<uint>> Subscribe(STM32DMA dma, int stream, Action<uint> listener)
        {
            lock(subscriptions)
            {
                var streams = subscriptions.GetOrCreateValue(dma);
                if(!streams.TryGetValue(stream, out var listeners))
                {
                    listeners = new List<Action<uint>>();
                    streams[stream] = listeners;
                    dma.RegistersCollection.AddBeforeWriteHook(StreamConfiguration(stream),
                        (offset, value) =>
                        {
                            Action<uint>[] hooks;
                            lock(listeners)
                            {
                                hooks = listeners.ToArray();
                            }
                            foreach(var hook in hooks)
                            {
                                hook(value);
                            }
                            return null;
                        });
                }
                lock(listeners)
                {
                    listeners.Add(listener);
                }
                return listeners;
            }
        }

        private void StreamWritten(STM32DMA dma, int stream, uint configuration)
        {
            if((configuration & StreamEnable) == 0 ||
               dma.ReadDoubleWord(StreamPeripheral(stream)) != dmarAddress)
            {
                return;
            }
            if((configuration & DirectionMask) == MemoryToPeripheral)
            {
                SendFrames(dma, stream);
            }
            else if((configuration & DirectionMask) == PeripheralToMemory)
            {
                // the enable write has not landed yet
                machine.ScheduleAction(TimeInterval.FromMicroseconds(1),
                    _ => Inject(dma, stream), name: "BDShot capture");
            }
        }

        private void SendFrames(STM32DMA dma, int stream)
        {
            if(dma.ReadDoubleWord(StreamCount(stream)) < DshotWords)
            {
                return;
            }
            var memory = dma.ReadDoubleWord(StreamMemory(stream));
            var autoReload = timer.ReadDoubleWord(AutoReload);
            var type = DshotType(autoReload);
            for(var channel = 0; channel < Channels; channel++)
            {
                if(sockets[channel] == null || !TryDecode(memory, channel, autoReload, out var frame))
                {
                    continue;
                }
                var bidirectional = HasInvertedChecksum(frame);
                lastFrames[channel] = frame;
                var packet = new byte[8];
                Put16(packet, 0, Magic);
                packet[2] = type;
                packet[3] = 4;
                Put16(packet, 4, bidirectional ? FlagIdleHigh : (ushort)0);
                Put16(packet, 6, frame);
                try
                {
                    sockets[channel].Send(packet, packet.Length);
                    Interlocked.Increment(ref framesSent);
                }
                catch(SocketException)
                {
                    // ESC not listening yet; DShot is lossy and resends
                }
            }
        }

        private bool TryDecode(uint memory, int channel, uint autoReload, out ushort frame)
        {
            frame = 0;
            var any = false;
            for(var bit = 0; bit < DshotBits; bit++)
            {
                var word = machine.SystemBus.ReadDoubleWord(
                    memory + (uint)(((DshotPreamble + bit) * Channels + channel) * sizeof(uint)));
                frame = (ushort)((frame << 1) | (word > autoReload / 2 ? 1 : 0));
                any |= word != 0;
            }
            return any && (HasNormalChecksum(frame) || HasInvertedChecksum(frame));
        }

        private void Inject(STM32DMA dma, int stream)
        {
            var configuration = dma.ReadDoubleWord(StreamConfiguration(stream));
            if((configuration & StreamEnable) == 0 ||
               (configuration & DirectionMask) != PeripheralToMemory)
            {
                return;
            }
            var channel = CaptureInputChannel();
            if(channel < 0 || sockets[channel] == null)
            {
                return;
            }
            ushort? reply;
            lock(sync)
            {
                reply = replies[channel];
                replies[channel] = null;
            }
            if(!reply.HasValue)
            {
                Interlocked.Increment(ref capturesSilent);
                return;
            }
            var remaining = dma.ReadDoubleWord(StreamCount(stream));
            if(remaining == 0)
            {
                return;
            }
            var memory = dma.ReadDoubleWord(StreamMemory(stream));
            var gcr = EncodeGcr(reply.Value);
            var timestamp = CaptureTick;
            machine.SystemBus.WriteDoubleWord(memory, timestamp);
            var used = 1u;
            for(var bit = GcrBits - 1; bit >= 0 && used < remaining; bit--)
            {
                timestamp += CaptureTick;
                if((gcr & (1u << bit)) != 0)
                {
                    machine.SystemBus.WriteDoubleWord(memory + used * sizeof(uint), timestamp);
                    used++;
                }
            }
            // leave NDTR as hardware would after one capture per edge; the
            // firmware's receive timeout then disables the stream and decodes
            dma.WriteDoubleWord(StreamCount(stream), remaining - used);
            Interlocked.Increment(ref repliesInjected);
        }

        // the input channel (0-3) of the single enabled capture channel
        private int CaptureInputChannel()
        {
            var enable = timer.ReadDoubleWord(CaptureCompareEnable);
            for(var ccr = 0; ccr < Channels; ccr++)
            {
                if((enable & (1u << (4 * ccr))) == 0)
                {
                    continue;
                }
                var mode = timer.ReadDoubleWord(ccr < 2 ? CaptureCompareMode1 : CaptureCompareMode2);
                var selection = (mode >> (8 * (ccr & 1))) & 3;
                if(selection == 1)
                {
                    return ccr;
                }
                if(selection == 2)
                {
                    return ccr ^ 1;
                }
            }
            return -1;
        }

        private void Receive(int channel)
        {
            var endpoint = new IPEndPoint(IPAddress.Loopback, 0);
            while(!disposed)
            {
                try
                {
                    var packet = sockets[channel].Receive(ref endpoint);
                    if(packet.Length < 8 || Get16(packet, 0) != Magic ||
                       packet[3] != 4 || packet[2] < Dshot150 || packet[2] > Dshot600)
                    {
                        continue;
                    }
                    lock(sync)
                    {
                        replies[channel] = Get16(packet, 6);
                    }
                    Interlocked.Increment(ref repliesReceived);
                }
                catch(SocketException exception)
                {
                    // an ESC that is not running yet or restarting gives
                    // refused/reset errors; keep listening for it
                    if(exception.SocketErrorCode != SocketError.TimedOut && !disposed)
                    {
                        this.Log(LogLevel.Debug, "ESC channel {0} socket: {1}",
                            channel + 1, exception.Message);
                        Thread.Sleep(ErrorRetryMs);
                    }
                }
                catch(ObjectDisposedException)
                {
                    return;
                }
            }
        }

        private byte DshotType(uint autoReload)
        {
            var prescaler = timer.ReadDoubleWord(Prescaler);
            var bitrate = timer.Frequency / ((prescaler + 1UL) * (autoReload + 1UL));
            if(bitrate < 225000) return Dshot150;
            if(bitrate < 450000) return Dshot300;
            return Dshot600;
        }

        private static uint EncodeGcr(ushort frame)
        {
            uint encoded = 0;
            for(var shift = 12; shift >= 0; shift -= 4)
            {
                encoded = (encoded << 5) | GcrTable[(frame >> shift) & 0xF];
            }
            return encoded;
        }

        private static uint Checksum(ushort frame)
        {
            var payload = frame >> 4;
            return (uint)((payload ^ (payload >> 4) ^ (payload >> 8)) & 0xF);
        }

        private static bool HasNormalChecksum(ushort frame) => (frame & 0xF) == Checksum(frame);
        private static bool HasInvertedChecksum(ushort frame) => (frame & 0xF) == (~Checksum(frame) & 0xF);

        private static long StreamConfiguration(int stream) => StreamBase + StreamStride * stream;
        private static long StreamCount(int stream) => StreamConfiguration(stream) + 0x04;
        private static long StreamPeripheral(int stream) => StreamConfiguration(stream) + 0x08;
        private static long StreamMemory(int stream) => StreamConfiguration(stream) + 0x0C;

        private static ushort Get16(byte[] data, int offset) =>
            (ushort)(data[offset] | data[offset + 1] << 8);
        private static void Put16(byte[] data, int offset, ushort value)
        {
            data[offset] = (byte)value;
            data[offset + 1] = (byte)(value >> 8);
        }

        private static readonly ConditionalWeakTable<STM32DMA, Dictionary<int, List<Action<uint>>>> subscriptions =
            new ConditionalWeakTable<STM32DMA, Dictionary<int, List<Action<uint>>>>();
        private readonly List<(List<Action<uint>>, Action<uint>)> registrations =
            new List<(List<Action<uint>>, Action<uint>)>();

        private readonly IMachine machine;
        private readonly STM32_Timer timer;
        private readonly uint dmarAddress;
        private readonly STM32DMA[] controllers;
        private readonly UdpClient[] sockets = new UdpClient[Channels];
        private readonly ushort?[] replies = new ushort?[Channels];
        private readonly uint[] lastFrames = new uint[Channels];
        private readonly object sync = new object();
        private volatile bool disposed;
        private int framesSent;
        private int repliesReceived;
        private int repliesInjected;
        private int capturesSilent;

        private static readonly uint[] GcrTable = {
            0x19, 0x1B, 0x12, 0x13, 0x1D, 0x15, 0x16, 0x17,
            0x1A, 0x09, 0x0A, 0x0B, 0x1E, 0x0D, 0x0E, 0x0F,
        };

        private const ushort Magic = 0x4453;
        private const ushort FlagIdleHigh = 1;
        private const byte Dshot150 = 1;
        private const byte Dshot300 = 2;
        private const byte Dshot600 = 3;
        private const int Channels = 4;
        private const int StreamsPerController = 8;
        private const int DshotPreamble = 1;
        private const int DshotBits = 16;
        private const uint DshotWords = (DshotPreamble + DshotBits) * Channels;
        private const int GcrBits = 20;
        // ArduPilot sets the capture prescaler for 16 ticks per GCR bit
        private const uint CaptureTick = 16;
        private const int ReceiveTimeoutMs = 100;
        private const int ErrorRetryMs = 10;
        private const long LastFrameBase = 0x10;

        private const long StreamBase = 0x10;
        private const long StreamStride = 0x18;
        private const uint StreamEnable = 1;
        private const uint DirectionMask = 3u << 6;
        private const uint PeripheralToMemory = 0;
        private const uint MemoryToPeripheral = 1u << 6;

        private const uint DmaAddressForFullTransfer = 0x4C;
        private const long CaptureCompareMode1 = 0x18;
        private const long CaptureCompareMode2 = 0x1C;
        private const long CaptureCompareEnable = 0x20;
        private const long Prescaler = 0x28;
        private const long AutoReload = 0x2C;
    }
}
