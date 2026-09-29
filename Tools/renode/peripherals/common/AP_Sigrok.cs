// Streaming logic analyser for the renode-la libsigrok driver. Renode's UART
// and SPI models exchange complete bytes, so this peripheral turns those byte
// transactions back into timestamped pin edges and rasterises them only while
// a client is capturing.
using System;
using System.Collections.Generic;
using System.Net;
using System.Net.Sockets;
using System.Text;
using System.Threading;
using Antmicro.Migrant;
using Antmicro.Renode.Core;
using Antmicro.Renode.Exceptions;
using Antmicro.Renode.Logging;
using Antmicro.Renode.Peripherals;
using Antmicro.Renode.Peripherals.Bus;
using Antmicro.Renode.Peripherals.UART;

namespace Antmicro.Renode.Peripherals.Miscellaneous
{
    public class AP_Sigrok : IDoubleWordPeripheral, IKnownSize, IGPIOReceiver,
                             IUARTWithBufferState, IAPSigrok, IDisposable
    {
        public AP_Sigrok(IMachine machine)
        {
            this.machine = machine;
            lifecycle = new object();
            captureLock = new object();
            pendingInputs = new Dictionary<int, bool>();
            edges = new List<Edge>();
            deviceName = "ArduPilot";
            sampleRate = DefaultSampleRate;
            spiFrequency = DefaultSpiFrequency;
        }

        public long Size => 0x100;

        // The analyser represents external bench equipment. A firmware reset
        // must not disconnect PulseView or clear the waveform already queued.
        public void Reset()
        {
        }

        public void Dispose()
        {
            lock(lifecycle)
            {
                Close();
                AttachUART(null);
            }
        }

        public uint ReadDoubleWord(long offset)
        {
            switch(offset)
            {
            case 0x00: return (uint)port;
            case 0x04: return sampleRate;
            case 0x08: return (uint)edgeCount;
            case 0x0C: return captures;
            default: return 0;
            }
        }

        public void WriteDoubleWord(long offset, uint value)
        {
            if(offset == 0x00)
            {
                Port = (int)value;
            }
            else if(offset == 0x04)
            {
                SampleRate = value;
            }
        }

        public int Port
        {
            get { return port; }
            set
            {
                if(value < 0 || value > 65535)
                {
                    throw new RecoverableException("sigrok port must be 0..65535");
                }
                lock(lifecycle)
                {
                    if(value == port)
                    {
                        return;
                    }
                    Close();
                    if(value != 0)
                    {
                        Open(value);
                    }
                }
            }
        }

        public uint SampleRate
        {
            get { return sampleRate; }
            set
            {
                if(value == 0 || value > MaxSampleRate)
                {
                    throw new RecoverableException(
                        "sigrok sample rate must be 1..1000000000 Hz");
                }
                sampleRate = value;
            }
        }

        public uint SpiFrequency
        {
            get { return spiFrequency; }
            set
            {
                if(value == 0 || value > MaxSampleRate)
                {
                    throw new RecoverableException(
                        "sigrok SPI frequency must be 1..1000000000 Hz");
                }
                spiFrequency = value;
            }
        }

        public int SpiMode
        {
            get { return spiMode; }
            set
            {
                if(value < 0 || value > 3)
                {
                    throw new RecoverableException("sigrok SPI mode must be 0..3");
                }
                spiMode = value;
            }
        }

        public int ChipSelectCount
        {
            get { return chipSelectCount; }
            set
            {
                if(value < 0 || value > MaxChannels - FixedSignalCount)
                {
                    throw new RecoverableException(String.Format(
                        "sigrok chip select count must be 0..{0}",
                        MaxChannels - FixedSignalCount));
                }
                chipSelectCount = value;
            }
        }

        public string DeviceName
        {
            get { return deviceName; }
            set
            {
                if(String.IsNullOrEmpty(value))
                {
                    throw new RecoverableException("sigrok device name cannot be empty");
                }
                deviceName = value;
            }
        }

        public void ConfigureSignals(string names)
        {
            if(String.IsNullOrEmpty(names))
            {
                throw new RecoverableException("sigrok signal names cannot be empty");
            }
            var configuredNames = names.Split('|');
            if(configuredNames.Length < FixedSignalCount ||
               configuredNames.Length > MaxChannels)
            {
                throw new RecoverableException(String.Format(
                    "sigrok needs {0}..{1} signals", FixedSignalCount,
                    MaxChannels));
            }
            if(FixedSignalCount + chipSelectCount > configuredNames.Length)
            {
                throw new RecoverableException(
                    "sigrok chip select count exceeds the signal count");
            }
            foreach(var name in configuredNames)
            {
                if(String.IsNullOrEmpty(name))
                {
                    throw new RecoverableException(
                        "sigrok signal names cannot be empty");
                }
            }

            lock(captureLock)
            {
                if(activeCapture != null)
                {
                    throw new RecoverableException(
                        "cannot change sigrok signals during capture");
                }
                signalNames = configuredNames;
                visibleChannels = new int[signalNames.Length];
                for(var channel = 0; channel < visibleChannels.Length; channel++)
                {
                    visibleChannels[channel] = channel;
                }
                baseState = new bool[signalNames.Length];
                scheduledState = new bool[signalNames.Length];
                // UART wires idle high; SPI clock idles according to CPOL;
                // GPIO chip selects are active-low and idle high.
                baseState[UartTxChannel] = true;
                baseState[UartRxChannel] = true;
                baseState[SpiClockChannel] = spiMode >= 2;
                for(var channel = FirstChipSelectChannel;
                    channel < FirstChipSelectChannel + chipSelectCount;
                    channel++)
                {
                    baseState[channel] = true;
                }
                Array.Copy(baseState, scheduledState, baseState.Length);
                foreach(var input in pendingInputs)
                {
                    if(input.Key >= FirstChipSelectChannel &&
                       input.Key < baseState.Length)
                    {
                        baseState[input.Key] = input.Value;
                        scheduledState[input.Key] = input.Value;
                    }
                }
                edges.Clear();
                edgeCount = 0;
                uartTxNextNs = uartRxNextNs = spiNextNs = NowNs;
            }
        }

        public void SelectSignals(string channels)
        {
            if(String.IsNullOrEmpty(channels))
            {
                throw new RecoverableException(
                    "sigrok signal selection cannot be empty");
            }
            lock(captureLock)
            {
                if(signalNames == null)
                {
                    throw new RecoverableException(
                        "configure sigrok signals before selecting them");
                }
                if(activeCapture != null)
                {
                    throw new RecoverableException(
                        "cannot change sigrok signals during capture");
                }
                var selected = new List<int>();
                var seen = new bool[signalNames.Length];
                foreach(var entry in channels.Split(','))
                {
                    int channel;
                    if(!Int32.TryParse(entry, out channel) || channel < 0 ||
                       channel >= signalNames.Length)
                    {
                        throw new RecoverableException(String.Format(
                            "invalid sigrok signal index {0}", entry));
                    }
                    if(seen[channel])
                    {
                        throw new RecoverableException(String.Format(
                            "duplicate sigrok signal index {0}", channel));
                    }
                    seen[channel] = true;
                    selected.Add(channel);
                }
                visibleChannels = selected.ToArray();
            }
        }

        public void AttachUART(IUART value)
        {
            if(uart != null)
            {
                uart.CharReceived -= HandleUARTTransmit;
            }
            if(uartWithBufferState != null)
            {
                uartWithBufferState.BufferStateChanged -= HandleBufferStateChanged;
            }
            uart = value;
            uartWithBufferState = value as IUARTWithBufferState;
            if(uart != null)
            {
                uart.CharReceived += HandleUARTTransmit;
            }
            if(uartWithBufferState != null)
            {
                uartWithBufferState.BufferStateChanged += HandleBufferStateChanged;
            }
        }

        public void WriteChar(byte value)
        {
            ObserveUART(value, UartRxChannel);
            if(uart != null)
            {
                uart.WriteChar(value);
            }
        }

        public uint BaudRate
        {
            get
            {
                if(uart == null)
                {
                    return lastBaudRate;
                }
                try
                {
                    var value = uart.BaudRate;
                    if(value != 0)
                    {
                        lastBaudRate = value;
                    }
                }
                catch(DivideByZeroException)
                {
                }
                return lastBaudRate;
            }
        }

        public Parity ParityBit => uart == null ? Parity.None : uart.ParityBit;
        public Bits StopBits => uart == null ? Bits.One : uart.StopBits;
        public BufferState BufferState => uartWithBufferState == null ?
            BufferState.Empty : uartWithBufferState.BufferState;

        [field: Transient]
        public event Action<byte> CharReceived;
        [field: Transient]
        public event Action<BufferState> BufferStateChanged;

        public void OnGPIO(int number, bool value)
        {
            lock(captureLock)
            {
                pendingInputs[number] = value;
                if(signalNames == null || number < FirstChipSelectChannel ||
                   number >= signalNames.Length)
                {
                    return;
                }
                AddEdgeLocked(number, value, Math.Max(NowNs, spiNextNs));
                PruneIdleEdgesLocked();
            }
        }

        public void ObserveSPI(byte transmitted, byte received)
        {
            lock(captureLock)
            {
                if(signalNames == null)
                {
                    return;
                }
                var start = Math.Max(NowNs, spiNextNs);
                var halfPeriodNs = Math.Max(1L,
                    500000000L / Math.Max(1U, spiFrequency));
                var idleClock = spiMode >= 2;
                AddEdgeLocked(SpiClockChannel, idleClock, start);
                for(var bit = 7; bit >= 0; bit--)
                {
                    var bitStart = start + (7 - bit) * 2 * halfPeriodNs;
                    AddEdgeLocked(SpiMosiChannel,
                        (transmitted & (1 << bit)) != 0, bitStart);
                    AddEdgeLocked(SpiMisoChannel,
                        (received & (1 << bit)) != 0, bitStart);
                    AddEdgeLocked(SpiClockChannel, !idleClock,
                        bitStart + halfPeriodNs);
                    AddEdgeLocked(SpiClockChannel, idleClock,
                        bitStart + 2 * halfPeriodNs);
                }
                spiNextNs = start + 16 * halfPeriodNs;
                PruneIdleEdgesLocked();
            }
        }

        private void HandleUARTTransmit(byte value)
        {
            ObserveUART(value, UartTxChannel);
            var handler = CharReceived;
            if(handler != null)
            {
                handler(value);
            }
        }

        private void HandleBufferStateChanged(BufferState value)
        {
            var handler = BufferStateChanged;
            if(handler != null)
            {
                handler(value);
            }
        }

        private void ObserveUART(byte value, int channel)
        {
            lock(captureLock)
            {
                if(signalNames == null)
                {
                    return;
                }
                var baudRate = Math.Max(1U, BaudRate);
                var bitPeriodNs = Math.Max(1L, 1000000000L / baudRate);
                var next = channel == UartTxChannel ? uartTxNextNs : uartRxNextNs;
                var start = Math.Max(NowNs, next);
                AddEdgeLocked(channel, false, start);
                for(var bit = 0; bit < 8; bit++)
                {
                    AddEdgeLocked(channel, (value & (1 << bit)) != 0,
                        start + (bit + 1) * bitPeriodNs);
                }
                AddEdgeLocked(channel, true, start + 9 * bitPeriodNs);
                next = start + 10 * bitPeriodNs;
                if(channel == UartTxChannel)
                {
                    uartTxNextNs = next;
                }
                else
                {
                    uartRxNextNs = next;
                }
                PruneIdleEdgesLocked();
            }
        }

        private void AddEdgeLocked(int channel, bool value, long timeNs)
        {
            if(scheduledState[channel] == value)
            {
                return;
            }
            scheduledState[channel] = value;
            if(activeCapture != null && !activeCapture.Enabled[channel])
            {
                return;
            }
            var edge = new Edge(timeNs, channel, value, edgeSequence++);
            var index = edges.BinarySearch(edge, EdgeComparer.Instance);
            if(index < 0)
            {
                index = ~index;
            }
            edges.Insert(index, edge);
            edgeCount++;
        }

        private void PruneIdleEdgesLocked()
        {
            if(activeCapture == null)
            {
                AdvanceBaseLocked(NowNs - IdleHistoryNs);
            }
        }

        private void AdvanceBaseLocked(long throughNs)
        {
            var count = 0;
            while(count < edges.Count && edges[count].TimeNs <= throughNs)
            {
                var edge = edges[count++];
                baseState[edge.Channel] = edge.Value;
            }
            if(count != 0)
            {
                edges.RemoveRange(0, count);
            }
        }

        private Capture BeginCapture(uint rate, byte[] enabledMask)
        {
            lock(captureLock)
            {
                if(signalNames == null)
                {
                    throw new InvalidOperationException(
                        "sigrok signals have not been configured");
                }
                if(activeCapture != null)
                {
                    throw new InvalidOperationException(
                        "another sigrok capture is active");
                }
                var maskSize = (visibleChannels.Length + 7) / 8;
                if(enabledMask == null || enabledMask.Length != maskSize)
                {
                    throw new InvalidOperationException(String.Format(
                        "sigrok channel mask must be {0} bytes", maskSize));
                }
                var finalBits = visibleChannels.Length % 8;
                if(finalBits != 0 &&
                   (enabledMask[maskSize - 1] & ~((1 << finalBits) - 1)) != 0)
                {
                    throw new InvalidOperationException(
                        "sigrok channel mask has bits beyond the channel count");
                }
                var enabledChannels = new List<int>();
                var enabled = new bool[signalNames.Length];
                for(var visibleChannel = 0;
                    visibleChannel < visibleChannels.Length; visibleChannel++)
                {
                    if((enabledMask[visibleChannel / 8] &
                        (1 << (visibleChannel % 8))) == 0)
                    {
                        continue;
                    }
                    var channel = visibleChannels[visibleChannel];
                    enabled[channel] = true;
                    enabledChannels.Add(channel);
                }
                if(enabledChannels.Count == 0)
                {
                    throw new InvalidOperationException(
                        "sigrok capture needs at least one enabled channel");
                }
                var now = NowNs;
                AdvanceBaseLocked(now);
                activeCapture = new Capture(rate, now,
                    (bool[])baseState.Clone(), enabled,
                    enabledChannels.ToArray());
                edges.RemoveAll(edge => !enabled[edge.Channel]);
                captures++;
                return activeCapture;
            }
        }

        private void EndCapture(Capture capture)
        {
            lock(captureLock)
            {
                if(activeCapture == capture)
                {
                    for(var channel = 0; channel < capture.Enabled.Length;
                        channel++)
                    {
                        if(!capture.Enabled[channel])
                        {
                            baseState[channel] = scheduledState[channel];
                        }
                    }
                    activeCapture = null;
                }
            }
        }

        private byte[] BuildSamples(Capture capture)
        {
            lock(captureLock)
            {
                var now = NowNs;
                if(capture.NextNs > now)
                {
                    return null;
                }
                var maxSamples = Math.Max(1, MaxDataPayload / capture.UnitSize);
                maxSamples = Math.Min(maxSamples, SamplesPerFrame);
                var data = new byte[maxSamples * capture.UnitSize];
                var samples = 0;
                var consumedEdges = 0;
                while(samples < maxSamples && capture.NextNs <= now)
                {
                    while(consumedEdges < edges.Count &&
                          edges[consumedEdges].TimeNs <= capture.NextNs)
                    {
                        var edge = edges[consumedEdges++];
                        capture.State[edge.Channel] = edge.Value;
                        baseState[edge.Channel] = edge.Value;
                    }
                    var offset = samples * capture.UnitSize;
                    for(var packedChannel = 0;
                        packedChannel < capture.EnabledChannels.Length;
                        packedChannel++)
                    {
                        var channel = capture.EnabledChannels[packedChannel];
                        if(capture.State[channel])
                        {
                            data[offset + packedChannel / 8] |=
                                (byte)(1 << (packedChannel % 8));
                        }
                    }
                    samples++;
                    capture.Advance();
                }
                if(consumedEdges != 0)
                {
                    edges.RemoveRange(0, consumedEdges);
                }
                if(samples == maxSamples)
                {
                    return data;
                }
                Array.Resize(ref data, samples * capture.UnitSize);
                return data;
            }
        }

        private long NowNs => (long)(machine.ElapsedVirtualTime.TimeElapsed
            .TotalMicroseconds * 1000.0);

        private void Open(int value)
        {
            lock(captureLock)
            {
                if(signalNames == null)
                {
                    throw new RecoverableException(
                        "configure sigrok signals before opening its port");
                }
            }
            var listener = new Socket(AddressFamily.InterNetwork,
                                      SocketType.Stream, ProtocolType.Tcp);
            try
            {
                listener.SetSocketOption(SocketOptionLevel.Socket,
                                         SocketOptionName.ReuseAddress, true);
                listener.Bind(new IPEndPoint(IPAddress.Loopback, value));
                listener.Listen(1);
            }
            catch(SocketException e)
            {
                listener.Close();
                throw new RecoverableException(String.Format(
                    "could not bind the sigrok port {0}: {1}", value, e.Message));
            }
            listenerSocket = listener;
            port = value;
            listenerThread = new Thread(() => AcceptLoop(listener))
            {
                IsBackground = true,
                Name = "ArduPilot sigrok " + value,
            };
            listenerThread.Start();
            this.Log(LogLevel.Info,
                "renode-la listening on tcp 127.0.0.1:{0}", value);
        }

        private void Close()
        {
            port = 0;
            var listener = listenerSocket;
            var client = clientSocket;
            var thread = listenerThread;
            listenerSocket = null;
            clientSocket = null;
            listenerThread = null;
            if(client != null)
            {
                client.Close();
            }
            if(listener != null)
            {
                listener.Close();
            }
            if(thread != null && thread != Thread.CurrentThread &&
               !thread.Join(2000))
            {
                this.Log(LogLevel.Warning,
                    "sigrok listener thread did not stop in time");
            }
        }

        private void AcceptLoop(Socket listener)
        {
            try
            {
                while(listenerSocket == listener)
                {
                    Socket client;
                    try
                    {
                        client = listener.Accept();
                    }
                    catch(SocketException)
                    {
                        if(listenerSocket != listener)
                        {
                            return;
                        }
                        continue;
                    }
                    client.NoDelay = true;
                    clientSocket = client;
                    try
                    {
                        new ProtocolSession(this, client).Run();
                    }
                    catch(SocketException)
                    {
                    }
                    catch(ObjectDisposedException)
                    {
                    }
                    catch(Exception e)
                    {
                        this.Log(LogLevel.Error, "sigrok client stopped: {0}", e);
                    }
                    finally
                    {
                        if(clientSocket == client)
                        {
                            clientSocket = null;
                        }
                        client.Close();
                    }
                }
            }
            catch(ObjectDisposedException)
            {
            }
            catch(Exception e)
            {
                this.Log(LogLevel.Error, "sigrok listener stopped: {0}", e);
            }
        }

        private sealed class ProtocolSession
        {
            public ProtocolSession(AP_Sigrok owner, Socket socket)
            {
                this.owner = owner;
                this.socket = socket;
            }

            public void Run()
            {
                SendGreeting();
                Frame frame;
                while(ReceiveFrame(out frame))
                {
                    if(frame.Type == StartFrame && frame.Payload.Length >= 8)
                    {
                        var rate = ReadUInt64(frame.Payload, 0);
                        if(rate == 0 || rate > MaxSampleRate)
                        {
                            SendError("sample rate must be 1..1000000000 Hz");
                            return;
                        }
                        var enabledMask = new byte[frame.Payload.Length - 8];
                        Array.Copy(frame.Payload, 8, enabledMask, 0,
                            enabledMask.Length);
                        Stream((uint)rate, enabledMask);
                    }
                    else if(frame.Type != StopFrame || frame.Payload.Length != 0)
                    {
                        SendError("invalid renode-la command");
                        return;
                    }
                }
            }

            private void Stream(uint rate, byte[] enabledMask)
            {
                Capture capture;
                try
                {
                    capture = owner.BeginCapture(rate, enabledMask);
                }
                catch(Exception e)
                {
                    SendError(e.Message);
                    return;
                }
                try
                {
                    while(true)
                    {
                        if(socket.Poll(0, SelectMode.SelectRead))
                        {
                            Frame command;
                            if(!ReceiveFrame(out command))
                            {
                                return;
                            }
                            if(command.Type == StopFrame &&
                               command.Payload.Length == 0)
                            {
                                return;
                            }
                            SendError("expected STOP during capture");
                            return;
                        }
                        var data = owner.BuildSamples(capture);
                        if(data == null || data.Length == 0)
                        {
                            Thread.Sleep(1);
                            continue;
                        }
                        SendFrame(DataFrame, data);
                    }
                }
                finally
                {
                    owner.EndCapture(capture);
                }
            }

            private void SendGreeting()
            {
                string[] names;
                string name;
                uint rate;
                lock(owner.captureLock)
                {
                    names = new string[owner.visibleChannels.Length];
                    for(var channel = 0; channel < names.Length; channel++)
                    {
                        names[channel] = owner.signalNames[
                            owner.visibleChannels[channel]];
                    }
                    name = owner.deviceName;
                    rate = owner.sampleRate;
                }
                var metadata = new List<byte>();
                AddString(metadata, name);
                foreach(var signal in names)
                {
                    AddString(metadata, signal);
                }
                var greeting = new byte[GreetingSize + metadata.Count];
                Array.Copy(ProtocolMagic, greeting, ProtocolMagic.Length);
                WriteUInt16(greeting, 8, ProtocolVersion);
                WriteUInt16(greeting, 10, (ushort)names.Length);
                WriteUInt32(greeting, 12, (uint)metadata.Count);
                WriteUInt64(greeting, 16, rate);
                metadata.CopyTo(greeting, GreetingSize);
                SendAll(greeting);
            }

            private bool ReceiveFrame(out Frame frame)
            {
                frame = new Frame();
                var header = ReceiveExact(FrameHeaderSize);
                if(header == null)
                {
                    return false;
                }
                if(header[1] != 0 || ReadUInt16(header, 2) != 0)
                {
                    throw new InvalidOperationException(
                        "non-zero renode-la reserved fields");
                }
                var length = ReadUInt32(header, 4);
                if(length > MaxDataPayload)
                {
                    throw new InvalidOperationException(
                        "renode-la frame is too large");
                }
                var payload = ReceiveExact((int)length);
                if(payload == null)
                {
                    return false;
                }
                frame.Type = header[0];
                frame.Payload = payload;
                return true;
            }

            private byte[] ReceiveExact(int length)
            {
                var result = new byte[length];
                var offset = 0;
                while(offset < length)
                {
                    var received = socket.Receive(result, offset,
                        length - offset, SocketFlags.None);
                    if(received <= 0)
                    {
                        return null;
                    }
                    offset += received;
                }
                return result;
            }

            private void SendError(string message)
            {
                SendFrame(ErrorFrame, Encoding.UTF8.GetBytes(message));
            }

            private void SendFrame(byte type, byte[] payload)
            {
                var header = new byte[FrameHeaderSize];
                header[0] = type;
                WriteUInt32(header, 4, (uint)payload.Length);
                SendAll(header);
                SendAll(payload);
            }

            private void SendAll(byte[] data)
            {
                var offset = 0;
                while(offset < data.Length)
                {
                    offset += socket.Send(data, offset, data.Length - offset,
                        SocketFlags.None);
                }
            }

            private static void AddString(List<byte> target, string value)
            {
                var encoded = Encoding.UTF8.GetBytes(value);
                if(encoded.Length == 0 || encoded.Length > UInt16.MaxValue)
                {
                    throw new InvalidOperationException(
                        "invalid renode-la metadata string");
                }
                target.Add((byte)encoded.Length);
                target.Add((byte)(encoded.Length >> 8));
                target.AddRange(encoded);
            }

            private readonly AP_Sigrok owner;
            private readonly Socket socket;

            private struct Frame
            {
                public byte Type;
                public byte[] Payload;
            }
        }

        private sealed class Capture
        {
            public Capture(uint rate, long startNs, bool[] state,
                           bool[] enabled, int[] enabledChannels)
            {
                Rate = rate;
                NextNs = startNs;
                State = state;
                Enabled = enabled;
                EnabledChannels = enabledChannels;
                UnitSize = (enabledChannels.Length + 7) / 8;
                wholePeriodNs = 1000000000L / rate;
                periodRemainder = 1000000000UL % rate;
            }

            public void Advance()
            {
                NextNs += wholePeriodNs;
                remainder += periodRemainder;
                if(remainder >= Rate)
                {
                    NextNs++;
                    remainder -= Rate;
                }
            }

            public readonly uint Rate;
            public long NextNs;
            public readonly bool[] State;
            public readonly bool[] Enabled;
            public readonly int[] EnabledChannels;
            public readonly int UnitSize;
            private readonly long wholePeriodNs;
            private readonly ulong periodRemainder;
            private ulong remainder;
        }

        private struct Edge
        {
            public Edge(long timeNs, int channel, bool value, ulong sequence)
            {
                TimeNs = timeNs;
                Channel = channel;
                Value = value;
                Sequence = sequence;
            }

            public readonly long TimeNs;
            public readonly int Channel;
            public readonly bool Value;
            public readonly ulong Sequence;
        }

        private sealed class EdgeComparer : IComparer<Edge>
        {
            public int Compare(Edge left, Edge right)
            {
                var result = left.TimeNs.CompareTo(right.TimeNs);
                return result != 0 ? result : left.Sequence.CompareTo(right.Sequence);
            }

            public static readonly EdgeComparer Instance = new EdgeComparer();
        }

        private static ushort ReadUInt16(byte[] data, int offset)
        {
            return (ushort)(data[offset] | data[offset + 1] << 8);
        }

        private static uint ReadUInt32(byte[] data, int offset)
        {
            return (uint)(data[offset] | data[offset + 1] << 8 |
                data[offset + 2] << 16 | data[offset + 3] << 24);
        }

        private static ulong ReadUInt64(byte[] data, int offset)
        {
            return ReadUInt32(data, offset) |
                ((ulong)ReadUInt32(data, offset + 4) << 32);
        }

        private static void WriteUInt16(byte[] data, int offset, ushort value)
        {
            data[offset] = (byte)value;
            data[offset + 1] = (byte)(value >> 8);
        }

        private static void WriteUInt32(byte[] data, int offset, uint value)
        {
            data[offset] = (byte)value;
            data[offset + 1] = (byte)(value >> 8);
            data[offset + 2] = (byte)(value >> 16);
            data[offset + 3] = (byte)(value >> 24);
        }

        private static void WriteUInt64(byte[] data, int offset, ulong value)
        {
            WriteUInt32(data, offset, (uint)value);
            WriteUInt32(data, offset + 4, (uint)(value >> 32));
        }

        private readonly IMachine machine;
        private readonly object lifecycle;
        private readonly object captureLock;
        private readonly Dictionary<int, bool> pendingInputs;
        private readonly List<Edge> edges;
        private IUART uart;
        private IUARTWithBufferState uartWithBufferState;
        private string[] signalNames;
        private int[] visibleChannels;
        private bool[] baseState;
        private bool[] scheduledState;
        private string deviceName;
        private uint sampleRate;
        private uint spiFrequency;
        private int spiMode;
        private int chipSelectCount;
        private uint lastBaudRate = DefaultBaudRate;
        private long uartTxNextNs;
        private long uartRxNextNs;
        private long spiNextNs;
        private ulong edgeSequence;
        private ulong edgeCount;
        private uint captures;
        private Capture activeCapture;
        private int port;

        [Transient]
        private Socket listenerSocket;
        [Transient]
        private Socket clientSocket;
        [Transient]
        private Thread listenerThread;

        private const int UartTxChannel = 0;
        private const int UartRxChannel = 1;
        private const int SpiClockChannel = 2;
        private const int SpiMosiChannel = 3;
        private const int SpiMisoChannel = 4;
        private const int FirstChipSelectChannel = 5;
        private const int FixedSignalCount = 5;
        private const int MaxChannels = 256;
        private const uint DefaultSampleRate = 10000000;
        private const uint DefaultSpiFrequency = 1000000;
        private const uint DefaultBaudRate = 115200;
        private const uint MaxSampleRate = 1000000000;
        private const int SamplesPerFrame = 65536;
        private const int MaxDataPayload = 16 * 1024 * 1024;
        private const long IdleHistoryNs = 1000000000L;
        private const int GreetingSize = 24;
        private const int FrameHeaderSize = 8;
        private const ushort ProtocolVersion = 2;
        private const byte StartFrame = 0x01;
        private const byte StopFrame = 0x02;
        private const byte DataFrame = 0x80;
        private const byte ErrorFrame = 0x81;
        private static readonly byte[] ProtocolMagic = Encoding.ASCII.GetBytes(
            "RenodeLA");
    }
}
