
using System;

public partial class MAVLink
{
    public class MAVLinkMessage
    {
        public static readonly MAVLinkMessage Invalid = new MAVLinkMessage();
        object _locker = new object();

        private byte[] _buffer;

        public byte[] buffer
        {
            get { return _buffer; }
            set
            {
                _buffer = value;
                processBuffer(_buffer);
            }
        }

        public DateTime rxtime { get; set; }
        public byte header { get; internal set; }
        public byte payloadlength { get; internal set; }

        public byte incompat_flags { get; internal set; }
        public byte compat_flags { get; internal set; }

        public byte seq { get; internal set; }
        public byte sysid { get; internal set; }
        public byte compid { get; internal set; }

        public uint msgid { get; internal set; }

        public bool ismavlink2 {
            get
            {
                if (buffer != null && buffer.Length > 0)
                    return (buffer[0] == MAVLINK_STX);

                return false;
            }
        }

        public string msgtypename
        {
            get { return MAVLINK_MESSAGE_INFOS.GetMessageInfo(msgid).name; }
        }

        object _data;
        public object data
        {
            get
            {
                // lock the entire creation of the packet. to prevent returning a incomplete packet.
                lock (_locker)
                {
                    if (_data != null)
                        return _data;

                    var typeinfo = MAVLINK_MESSAGE_INFOS.GetMessageInfo(msgid);

                    if (typeinfo.type == null)
                        return null;

                    _data = Activator.CreateInstance(typeinfo.type);

                    try
                    {
                        if (payloadlength == 0)
                            return _data;
                        // fill in the data of the object
                        if (ismavlink2)
                        {
                            MavlinkUtil.ByteArrayToStructure(buffer, ref _data, MAVLINK_NUM_HEADER_BYTES, payloadlength);
                        }
                        else
                        {
                            MavlinkUtil.ByteArrayToStructure(buffer, ref _data, 6, payloadlength);
                        }
                    }
                    catch (Exception ex)
                    {
                        System.Diagnostics.Debug.WriteLine(ex);
                    }
                }

                return _data;
            }
        }

        public T ToStructure<T>()
        {
            return (T)data;
        }

        public ushort crc16 { get; internal set; }

        public byte[] sig { get; internal set; }

        public byte sigLinkid
        {
            get
            {
                if (sig != null)
                {
                    return sig[0];
                }

                return 0;
            }
        }

        public ulong sigTimestamp 
        {
            get
            {
                if (sig != null)
                {
                    byte[] temp = new byte[8];
                    Array.Copy(sig, 1, temp, 0, 6);
                    return BitConverter.ToUInt64(temp, 0);
                }

                return 0;
            }
        }

        public int Length
        {
            get
            {
                if (buffer == null) return 0;
                return buffer.Length;
            }
        }

        public MAVLinkMessage()
        {
            this.rxtime = DateTime.MinValue;
        }

        public MAVLinkMessage(byte[] buffer): this(buffer, DateTime.UtcNow)
        {
        }

        public MAVLinkMessage(byte[] buffer, DateTime rxTime)
        {
            this.buffer = buffer;
            this.rxtime = rxTime;

            processBuffer(buffer);
        }

        internal void processBuffer(byte[] buffer)
        {
            _data = null;

            if (buffer[0] == MAVLINK_STX)
            {
                if (buffer.Length < 10)
                {
                    return;
                }
                header = buffer[0];
                payloadlength = buffer[1];
                incompat_flags = buffer[2];
                compat_flags = buffer[3];
                seq = buffer[4];
                sysid = buffer[5];
                compid = buffer[6];
                msgid = (uint) ((buffer[9] << 16) + (buffer[8] << 8) + buffer[7]);

                var crc1 = MAVLINK_CORE_HEADER_LEN + payloadlength + 1;
                var crc2 = MAVLINK_CORE_HEADER_LEN + payloadlength + 2;

                crc16 = (ushort) ((buffer[crc2] << 8) + buffer[crc1]);

                if ((incompat_flags & MAVLINK_IFLAG_SIGNED) > 0)
                {
                    sig = new byte[MAVLINK_SIGNATURE_BLOCK_LEN];
                    Array.ConstrainedCopy(buffer, buffer.Length - MAVLINK_SIGNATURE_BLOCK_LEN, sig, 0,
                        MAVLINK_SIGNATURE_BLOCK_LEN);
                }
            }
            else
            {
                if (buffer.Length < 6)
                {
                    return;
                }
                header = buffer[0];
                payloadlength = buffer[1];
                seq = buffer[2];
                sysid = buffer[3];
                compid = buffer[4];
                msgid = buffer[5];

                var crc1 = MAVLINK_CORE_HEADER_MAVLINK1_LEN + payloadlength + 1;
                var crc2 = MAVLINK_CORE_HEADER_MAVLINK1_LEN + payloadlength + 2;

                crc16 = (ushort) ((buffer[crc2] << 8) + buffer[crc1]);
            }
        }

        public override string ToString()
        {
            return String.Format("{5},{4},{0},{1},{2},{3}", sysid, compid, msgid, msgtypename, ismavlink2, rxtime);
        }
    }
}
