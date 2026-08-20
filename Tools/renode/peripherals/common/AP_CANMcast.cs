// Bridge a Renode CAN hub to ArduPilot's UDP multicast CAN transport.
//
// The wire format matches the mcast driver used by SITL, libcanard, and
// dronecan_gui_tool: 239.65.82.<bus>:57732 with a 10-byte header followed by
// up to 64 data bytes. Both classic CAN and CAN FD frames are supported.
//
using System;
using System.Net;
using System.Net.Sockets;
using System.Threading;
using Antmicro.Migrant;
using Antmicro.Migrant.Hooks;
using Antmicro.Renode.Core;
using Antmicro.Renode.Core.CAN;
using Antmicro.Renode.Exceptions;
using Antmicro.Renode.Logging;
using Antmicro.Renode.Peripherals;
using Antmicro.Renode.Peripherals.Bus;

namespace Antmicro.Renode.Peripherals.CAN
{
    [AllowedTranslations(AllowedTranslation.ByteToDoubleWord | AllowedTranslation.WordToDoubleWord)]
    public class AP_CANMcast : IDoubleWordPeripheral, IKnownSize, ICAN, IDisposable
    {
        public AP_CANMcast(IMachine machine)
        {
            bus = -1;
        }

        public void Reset()
        {
            // A firmware reset does not disconnect a physical CAN bus.
        }

        public void Dispose()
        {
            lock(lifecycle)
            {
                Close();
            }
        }

        public int Bus
        {
            get => bus;
            set
            {
                if(value > MaximumBus)
                {
                    throw new RecoverableException(string.Format(
                        "mcast bus {0} is out of range, expected 0..{1}",
                        value, MaximumBus));
                }
                lock(lifecycle)
                {
                    if(value == bus)
                    {
                        return;
                    }
                    Close();
                    if(value >= 0)
                    {
                        Open(value);
                    }
                }
            }
        }

        public uint FramesToHost => framesToHost;
        public uint FramesFromHost => framesFromHost;
        public long Size => 0x100;

        public event Action<CANMessageFrame> FrameSent;

        public uint ReadDoubleWord(long offset)
        {
            switch(offset)
            {
            case 0x00:
                return (uint)bus;
            case 0x04:
                return framesToHost;
            case 0x08:
                return framesFromHost;
            default:
                return 0;
            }
        }

        public void WriteDoubleWord(long offset, uint value)
        {
            if(offset == 0x00)
            {
                Bus = (int)value;
            }
        }

        public void OnFrameReceived(CANMessageFrame message)
        {
            var socket = txSocket;
            var maximumLength = message.FDFormat ? MaximumFdDataLength : MaximumClassicDataLength;
            if(socket == null || message.Data.Length > maximumLength)
            {
                return;
            }

            var id = message.ExtendedFormat
                ? (message.Id & ExtendedIdMask) | ExtendedFrameFlag
                : message.Id & StandardIdMask;
            if(message.RemoteFrame)
            {
                id |= RemoteFrameFlag;
            }
            var packet = new byte[HeaderLength + message.Data.Length];
            packet[0] = (byte)(Magic & 0xFF);
            packet[1] = (byte)(Magic >> 8);
            if(message.FDFormat)
            {
                packet[4] = (byte)CanFdFlag;
            }
            packet[6] = (byte)id;
            packet[7] = (byte)(id >> 8);
            packet[8] = (byte)(id >> 16);
            packet[9] = (byte)(id >> 24);
            Array.Copy(message.Data, 0, packet, HeaderLength, message.Data.Length);
            var crc = Crc16(packet, 4, packet.Length - 4);
            packet[2] = (byte)crc;
            packet[3] = (byte)(crc >> 8);

            try
            {
                socket.Send(packet);
                framesToHost++;
            }
            catch(SocketException error)
            {
                this.Log(LogLevel.Warning, "mcast send failed: {0}", error.Message);
            }
            catch(ObjectDisposedException)
            {
                // A concurrent Bus change closed the socket.
            }
        }

        private void Open(int busNumber)
        {
            var group = IPAddress.Parse(string.Format("239.65.82.{0}", busNumber));
            Socket tx = null;
            Socket rx = null;
            try
            {
                tx = new Socket(AddressFamily.InterNetwork, SocketType.Dgram,
                    ProtocolType.Udp);
                tx.SetSocketOption(SocketOptionLevel.IP,
                    SocketOptionName.MulticastTimeToLive, 1);
                tx.Connect(new IPEndPoint(group, Port));

                rx = new Socket(AddressFamily.InterNetwork, SocketType.Dgram,
                    ProtocolType.Udp);
                rx.SetSocketOption(SocketOptionLevel.Socket,
                    SocketOptionName.ReuseAddress, true);
                rx.Bind(new IPEndPoint(group, Port));
                rx.SetSocketOption(SocketOptionLevel.IP,
                    SocketOptionName.AddMembership, new MulticastOption(group));
            }
            catch(SocketException error)
            {
                tx?.Close();
                rx?.Close();
                throw new RecoverableException(string.Format(
                    "cannot open mcast bus {0}: {1}", busNumber, error.Message));
            }

            txSocket = tx;
            rxSocket = rx;
            bus = busNumber;
            rxThread = new Thread(() => ReceiveLoop(rx, tx))
            {
                IsBackground = true,
                Name = "ArduPilot CAN mcast " + busNumber,
            };
            rxThread.Start();
            this.Log(LogLevel.Info, "CAN bridged to mcast bus {0} ({1}:{2})",
                busNumber, group, Port);
        }

        private void Close()
        {
            bus = -1;
            var tx = txSocket;
            var rx = rxSocket;
            var thread = rxThread;
            txSocket = null;
            rxSocket = null;
            rxThread = null;
            rx?.Close();
            tx?.Close();
            if(thread != null && !thread.Join(2000))
            {
                this.Log(LogLevel.Warning, "mcast receive thread did not stop in time");
            }
        }

        private void ReceiveLoop(Socket rx, Socket tx)
        {
            var own = (IPEndPoint)tx.LocalEndPoint;
            var buffer = new byte[HeaderLength + MaximumFdDataLength];
            EndPoint sender = new IPEndPoint(IPAddress.Any, 0);
            try
            {
                while(true)
                {
                    int length;
                    try
                    {
                        sender = new IPEndPoint(IPAddress.Any, 0);
                        length = rx.ReceiveFrom(buffer, ref sender);
                    }
                    catch(SocketException)
                    {
                        if(rxSocket != rx)
                        {
                            return;
                        }
                        continue;
                    }
                    if(rxSocket != rx)
                    {
                        return;
                    }

                    var source = (IPEndPoint)sender;
                    if(source.Port == own.Port &&
                       (source.Address.Equals(own.Address) ||
                        IPAddress.IsLoopback(source.Address) ||
                        IsLocalAddress(source.Address)))
                    {
                        continue;
                    }
                    var frame = Decode(buffer, length);
                    if(frame == null)
                    {
                        continue;
                    }
                    framesFromHost++;
                    FrameSent?.Invoke(frame);
                }
            }
            catch(ObjectDisposedException)
            {
                // Close() stopped the receive thread.
            }
            catch(Exception error)
            {
                this.Log(LogLevel.Error, "mcast receive thread stopped: {0}", error);
            }
        }

        private static CANMessageFrame Decode(byte[] buffer, int length)
        {
            if(length < HeaderLength || length > HeaderLength + MaximumFdDataLength)
            {
                return null;
            }
            var magic = (ushort)(buffer[0] | buffer[1] << 8);
            var crc = (ushort)(buffer[2] | buffer[3] << 8);
            if(magic != Magic || crc != Crc16(buffer, 4, length - 4))
            {
                return null;
            }
            var flags = (ushort)(buffer[4] | buffer[5] << 8);
            var fdFormat = (flags & CanFdFlag) != 0;
            var dataLength = length - HeaderLength;
            if((!fdFormat && dataLength > MaximumClassicDataLength) ||
               (flags & ~CanFdFlag) != 0)
            {
                return null;
            }
            var id = (uint)(buffer[6] | buffer[7] << 8 | buffer[8] << 16 |
                            (uint)buffer[9] << 24);
            if((id & ErrorFrameFlag) != 0)
            {
                return null;
            }
            var extendedFormat = (id & ExtendedFrameFlag) != 0;
            if(!extendedFormat && (id & ~RemoteFrameFlag) > StandardIdMask)
            {
                return null;
            }
            var data = new byte[dataLength];
            Array.Copy(buffer, HeaderLength, data, 0, dataLength);
            return new CANMessageFrame(
                id & (extendedFormat ? ExtendedIdMask : StandardIdMask), data,
                extendedFormat: extendedFormat,
                remoteFrame: (id & RemoteFrameFlag) != 0,
                fdFormat: fdFormat);
        }

        private static bool IsLocalAddress(IPAddress address)
        {
            try
            {
                foreach(var candidate in Dns.GetHostAddresses(Dns.GetHostName()))
                {
                    if(candidate.Equals(address))
                    {
                        return true;
                    }
                }
            }
            catch(Exception)
            {
                // Treat failed local address discovery as a non-local sender.
            }
            return false;
        }

        private static ushort Crc16(byte[] buffer, int offset, int length)
        {
            ushort crc = 0xFFFF;
            for(var index = 0; index < length; index++)
            {
                crc ^= (ushort)(buffer[offset + index] << 8);
                for(var bit = 0; bit < 8; bit++)
                {
                    crc = (crc & 0x8000) != 0
                        ? (ushort)((crc << 1) ^ 0x1021)
                        : (ushort)(crc << 1);
                }
            }
            return crc;
        }

        [PostDeserialization]
        private void AfterDeserialization()
        {
            if(bus >= 0)
            {
                Open(bus);
            }
        }

        private readonly object lifecycle = new object();
        [Transient]
        private volatile Socket txSocket;
        [Transient]
        private volatile Socket rxSocket;
        [Transient]
        private Thread rxThread;
        private int bus;
        private uint framesToHost;
        private uint framesFromHost;

        private const ushort Magic = 0x2934;
        private const ushort CanFdFlag = 0x0001;
        private const uint ExtendedFrameFlag = 0x80000000;
        private const uint RemoteFrameFlag = 0x40000000;
        private const uint ErrorFrameFlag = 0x20000000;
        private const uint ExtendedIdMask = 0x1FFFFFFF;
        private const uint StandardIdMask = 0x7FF;
        private const int Port = 57732;
        private const int HeaderLength = 10;
        private const int MaximumClassicDataLength = 8;
        private const int MaximumFdDataLength = 64;
        private const int MaximumBus = 9;
    }
}
