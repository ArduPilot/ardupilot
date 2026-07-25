using System;
using System.IO;
using System.Linq;
using System.Security.Cryptography;

public partial class MAVLink
{
    public static string GetUnit(string fieldname, Type packetype = null, string name="", uint msgid = UInt32.MaxValue)
    {
        try
        {
            message_info msginfo = new message_info();
            if(packetype != null)
                msginfo = MAVLink.MAVLINK_MESSAGE_INFOS.First(a => a.type == packetype);
            if (msgid != UInt32.MaxValue)
                msginfo = MAVLink.MAVLINK_MESSAGE_INFOS.First(a => a.msgid == msgid);
            if (!string.IsNullOrEmpty(name))
                msginfo = MAVLink.MAVLINK_MESSAGE_INFOS.First(a => a.name == name);

            if (msginfo.name == "")
                return "";
            
            var typeofthing = msginfo.type.GetField(fieldname);
            if (typeofthing != null)
            {
                var attrib = typeofthing.GetCustomAttributes(false);
                if (attrib.Length > 0)
                    return attrib.OfType<MAVLink.Units>().First().Unit;
            }
        }
        catch
        {
        }

        return "";
    }

    public class Units : Attribute
    {
        public Units(string unit)
        {
            Unit = unit;
        }

        public string Unit { get; set; }
    }

    public class Description : Attribute
    {
        public Description(string desc)
        {
            Text = desc;
        }

        public string Text { get; set; }
    }

    public class MavlinkParse
    {
        public int packetcount = 0;

        public int badCRC = 0;
        public int badLength = 0;

        public bool hasTimestamp = false;

        public MavlinkParse(bool hasTimestamp = false)
        {
            this.hasTimestamp = hasTimestamp;
        }

        public static void ReadWithTimeout(Stream BaseStream, byte[] buffer, int offset, int count)
        {
            int timeout = 500;

            if (BaseStream.CanSeek)
            {
                timeout = 0;

                if((BaseStream.Position + count) > BaseStream.Length)
                    throw new EndOfStreamException("End of data");
            }

            if (BaseStream.CanTimeout)
            {
                timeout = BaseStream.ReadTimeout;

                if (timeout == -1)
                    timeout = 60000;
            }

            DateTime to = DateTime.Now.AddMilliseconds(timeout);

            int toread = count;
            int pos = offset;

            while (true)
            {
                // read from stream
                int read = BaseStream.Read(buffer, pos, toread);

                // update counter
                toread -= read;
                pos += read;

                // reset timeout if we get data
                if (read > 0)
                    to = DateTime.Now.AddMilliseconds(timeout);

                if (toread == 0)
                    break;

                if (DateTime.Now > to)
                {
                    throw new TimeoutException("Timeout waiting for data");
                }
                System.Threading.Thread.Sleep(1);
            }
        }

        public MAVLinkMessage ReadPacket(Stream BaseStream)
        {
            byte[] buffer = new byte[MAVLink.MAVLINK_MAX_PACKET_LEN];

            DateTime packettime = DateTime.MinValue;

            if (hasTimestamp)
            {
                byte[] datearray = new byte[8];

                int tem = BaseStream.Read(datearray, 0, datearray.Length);

                Array.Reverse(datearray);

                DateTime date1 = new DateTime(1970, 1, 1, 0, 0, 0, DateTimeKind.Utc);

                UInt64 dateint = BitConverter.ToUInt64(datearray, 0);

                if ((dateint / 1000 / 1000 / 60 / 60) < 9999999)
                {
                    date1 = date1.AddMilliseconds(dateint / 1000);

                    packettime = date1.ToLocalTime();
                }
            }

            int readcount = 0;

            while (readcount <= MAVLink.MAVLINK_MAX_PACKET_LEN)
            {
                // read STX byte
                ReadWithTimeout(BaseStream, buffer, 0, 1);

                if (buffer[0] == MAVLink.MAVLINK_STX || buffer[0] == MAVLINK_STX_MAVLINK1)
                    break;

                readcount++;
            }

            if (readcount >= MAVLink.MAVLINK_MAX_PACKET_LEN)
            {
                return null;
                throw new InvalidDataException("No header found in data");
            }

            var headerlength = buffer[0] == MAVLINK_STX ? MAVLINK_CORE_HEADER_LEN : MAVLINK_CORE_HEADER_MAVLINK1_LEN;
            var headerlengthstx = headerlength + 1;

            // read header
            try {
                ReadWithTimeout(BaseStream, buffer, 1, headerlength);
            }
            catch (EndOfStreamException)
            {
                return null;
            }

            // packet length
            int lengthtoread = 0;
            if (buffer[0] == MAVLINK_STX)
            {
                lengthtoread = buffer[1] + headerlengthstx + 2 - 2; // data + header + checksum - magic - length
                if ((buffer[2] & MAVLINK_IFLAG_SIGNED) > 0)
                {
                    lengthtoread += MAVLINK_SIGNATURE_BLOCK_LEN;
                }
            }
            else
            {
                lengthtoread = buffer[1] + headerlengthstx + 2 - 2; // data + header + checksum - U - length    
            }

            try
            {
                //read rest of packet
                ReadWithTimeout(BaseStream, buffer, headerlengthstx, lengthtoread - (headerlengthstx - 2));
            }
            catch (EndOfStreamException)
            {
                return null;
            }

            // resize the packet to the correct length
            Array.Resize<byte>(ref buffer, lengthtoread + 2);

            MAVLinkMessage message = new MAVLinkMessage(buffer, packettime);

            // calc crc
            ushort crc = MavlinkCRC.crc_calculate(buffer, buffer.Length - 2);

            // calc extra bit of crc for mavlink 1.0+
            if (message.header == MAVLINK_STX || message.header == MAVLINK_STX_MAVLINK1)
            {
                crc = MavlinkCRC.crc_accumulate(MAVLINK_MESSAGE_INFOS.GetMessageInfo(message.msgid).crc, crc);
            }

            // check crc
            if ((message.crc16 >> 8) != (crc >> 8) ||
                      (message.crc16 & 0xff) != (crc & 0xff))
            {
                badCRC++;
                // crc fail
                return null;
            }

            return message;
        }

        public byte[] GenerateMAVLinkPacket10(MAVLINK_MSG_ID messageType, object indata, byte sysid = 255, byte compid = (byte)MAV_COMPONENT.MAV_COMP_ID_MISSIONPLANNER, int sequence = -1)
        {
            byte[] data;

            data = MavlinkUtil.StructureToByteArray(indata);

            byte[] packet = new byte[data.Length + 6 + 2];

            packet[0] = MAVLINK_STX_MAVLINK1;
            packet[1] = (byte)data.Length;
            packet[2] = (byte)packetcount;
            if (sequence != -1)
                packet[2] = (byte)sequence;

            packetcount++;

            packet[3] = sysid; // this is always 255 - MYGCS
            packet[4] = compid;
            packet[5] = (byte)messageType;


            int i = 6;
            foreach (byte b in data)
            {
                packet[i] = b;
                i++;
            }

            ushort checksum = MavlinkCRC.crc_calculate(packet, packet[1] + 6);

            checksum = MavlinkCRC.crc_accumulate(MAVLINK_MESSAGE_INFOS.GetMessageInfo((uint)messageType).crc, checksum);

            byte ck_a = (byte)(checksum & 0xFF); ///< High byte
            byte ck_b = (byte)(checksum >> 8); ///< Low byte

            packet[i] = ck_a;
            i += 1;
            packet[i] = ck_b;
            i += 1;

            return packet;
        }

        public byte[] GenerateMAVLinkPacket20(MAVLINK_MSG_ID messageType, object indata, bool sign = false, byte sysid = 255, byte compid= (byte)MAV_COMPONENT.MAV_COMP_ID_MISSIONPLANNER, int sequence = -1)
        {
            byte[] data;

            data = MavlinkUtil.StructureToByteArray(indata);

            MavlinkUtil.trim_payload(ref data);

            int extra = 0;
            if (sign)
                extra = MAVLINK_SIGNATURE_BLOCK_LEN;

            byte[] packet = new byte[data.Length + MAVLINK_NUM_NON_PAYLOAD_BYTES + extra];

            packet[0] = MAVLINK_STX;
            packet[1] = (byte)data.Length;
            packet[2] = 0;//incompat  signing
            if (sign)
                packet[2] |= MAVLINK_IFLAG_SIGNED;
            packet[3] = 0;//compat
            packet[4] = (byte)packetcount;
            if (sequence != -1)
                packet[4] = (byte)sequence;
            packetcount++;

            packet[5] = sysid;
            packet[6] = compid;
            packet[7] = (byte)((UInt32)messageType);
            packet[8] = (byte)((UInt32)messageType >> 8);
            packet[9] = (byte)((UInt32)messageType >> 16);

            int i = MAVLINK_NUM_HEADER_BYTES;
            foreach (byte b in data)
            {
                packet[i] = b;
                i++;
            }

            ushort checksum = MavlinkCRC.crc_calculate(packet, data.Length + MAVLINK_NUM_HEADER_BYTES);

            checksum = MavlinkCRC.crc_accumulate(MAVLINK_MESSAGE_INFOS.GetMessageInfo((uint)messageType).crc, checksum);

            byte ck_a = (byte)(checksum & 0xFF); ///< High byte
            byte ck_b = (byte)(checksum >> 8); ///< Low byte

            packet[i] = ck_a;
            i += 1;
            packet[i] = ck_b;
            i += 1;

            if (sign)
            {
                //https://docs.google.com/document/d/1ETle6qQRcaNWAmpG2wz0oOpFKSF_bcTmYMQvtTGI8ns/edit

                /*
                8 bits of link ID
                48 bits of timestamp
                48 bits of signature
                */

                // signature = sha256_48(secret_key + header + payload + CRC + link-ID + timestamp)

                var timestamp = (UInt64)((DateTime.UtcNow - new DateTime(2015, 1, 1)).TotalMilliseconds * 100);

                if (timestamp == lasttimestamp)
                    timestamp++;

                lasttimestamp = timestamp;

                var timebytes = BitConverter.GetBytes(timestamp);

                var sig = new byte[7]; // 13 includes the outgoing hash
                sig[0] = sendlinkid;
                Array.Copy(timebytes, 0, sig, 1, 6); // timestamp

                //Console.WriteLine("gen linkid {0}, time {1} {2} {3} {4} {5} {6} {7}", sig[0], sig[1], sig[2], sig[3], sig[4], sig[5], sig[6], timestamp);
                
                if (signingKey == null || signingKey.Length != 32)
                {
                    signingKey = new byte[32];
                }
                
                using (SHA256CryptoServiceProvider signit = new SHA256CryptoServiceProvider())
                {
                    MemoryStream ms = new MemoryStream();
                    ms.Write(signingKey, 0, signingKey.Length);
                    ms.Write(packet, 0, i);
                    ms.Write(sig, 0, sig.Length);

                    var ctx = signit.ComputeHash(ms.GetBuffer());
                    // trim to 48
                    Array.Resize(ref ctx, 6);

                    foreach (byte b in sig)
                    {
                        packet[i] = b;
                        i++;
                    }

                    foreach (byte b in ctx)
                    {
                        packet[i] = b;
                        i++;
                    }
                }
            }

            return packet;
        }

        public byte sendlinkid { get; set; }

        public ulong lasttimestamp { get; set; }

        public byte[] signingKey { get; set; }
    }
}
