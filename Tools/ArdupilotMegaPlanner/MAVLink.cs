using System;
using System.Collections.Generic;
using System.Text;
using System.IO.Ports;
using System.Runtime.InteropServices;
using System.Collections; // hashs
using System.Diagnostics; // stopwatch
using System.Reflection;
using System.Reflection.Emit;
using System.IO;


namespace ArdupilotMega
{
    public partial class MAVLink
    {

        public ICommsSerial BaseStream = new SerialPort();

        /// <summary>
        /// used for outbound packet sending
        /// </summary>
        byte packetcount = 0;
        public byte sysid = 0;
        public byte compid = 0;
        public Hashtable param = new Hashtable();
        public byte[][] packets = new byte[256][];
        public double[] packetspersecond = new double[256];
        DateTime[] packetspersecondbuild = new DateTime[256];
        object objlock = new object();
        object readlock = new object();
        object logwritelock = new object();
        public DateTime lastvalidpacket = DateTime.Now;
        bool oldlogformat = false;

        byte mavlinkversion = 0;
        public byte aptype = 0;
        byte[] readingpacket = new byte[256];

        public PointLatLngAlt[] wps = new PointLatLngAlt[200];

        public bool debugmavlink = false;

        public bool logreadmode = false;
        public DateTime lastlogread = DateTime.MinValue;
        public BinaryReader logplaybackfile = null;
        public BinaryWriter logfile = null;

        public byte[] streams = new byte[256];

        int bps1 = 0;
        int bps2 = 0;
        public int bps = 0;
        public DateTime bpstime = DateTime.Now;
        int recvpacketcount = 0;

        float synclost;
        float packetslost = 0;
        float packetsnotlost = 0;
        DateTime packetlosttimer = DateTime.Now;


        //Stopwatch stopwatch = new Stopwatch();

        public void Close()
        {
            BaseStream.Close();
        }

        public void Open()
        {
            Open(false);
        }

        public void Open(bool getparams)
        {
            if (BaseStream.IsOpen)
                return;

            System.Windows.Forms.Form frm = Common.LoadingBox("Mavlink Connecting..", "Mavlink Connecting..");
            frm.TopMost = true;

            // reset
            sysid = 0;
            compid = 0;
            param = new Hashtable();

            try
            {
                MainV2.givecomport = true;

                BaseStream.ReadBufferSize = 4 * 1024;

                lock (objlock) // so we dont have random traffic
                {

                    BaseStream.Open();

                    BaseStream.DiscardInBuffer();

                    BaseStream.toggleDTR();

                    // allow 2560 connect timeout on usb
                    System.Threading.Thread.Sleep(1000);

                }

                byte[] buffer;
                byte[] buffer1;

                DateTime start = DateTime.Now;

                int count = 0;

                while (true)
                {
                    System.Windows.Forms.Application.DoEvents();

                    // incase we are in setup mode
                    BaseStream.WriteLine("planner\rgcs\r");

                    frm.Controls[0].Text = (start.AddSeconds(30) - DateTime.Now).Seconds.ToString("Timeout in 0");

                    if (lastbad[0] == '!' && lastbad[1] == 'G' || lastbad[0] == 'G' && lastbad[1] == '!') // waiting for gps lock
                    {
                        frm.Controls[0].Text = "Waiting for GPS detection..";
                        start = start.AddSeconds(5); // each round is 1.1 seconds
                    }

                    System.Windows.Forms.Application.DoEvents();

                    if (!(start.AddSeconds(30) > DateTime.Now))
                    {
                        /*
                        System.Windows.Forms.DialogResult dr = System.Windows.Forms.MessageBox.Show("Data recevied but no mavlink packets where read from this port\nWhat do you want to do",
                            "Read Fail", System.Windows.Forms.MessageBoxButtons.RetryCancel);
                        if (dr == System.Windows.Forms.DialogResult.Retry)
                        {
                            port.toggleDTRnow(); // force reset on usb
                            start = DateTime.Now;
                        }
                        else*/
                        {
                            frm.Close();
                            this.Close();
                            throw new Exception("No Mavlink Heartbeat Packets where read from this port - Verify Baud Rate and setup\nIt might also be waiting for GPS Lock\nAPM Planner waits for 2 valid heartbeat packets before connecting");
                        }
                    }

                    System.Threading.Thread.Sleep(1);

                    // incase we are in setup mode
                    BaseStream.WriteLine("planner\rgcs\r");

                    System.Windows.Forms.Application.DoEvents();

                    buffer = getHeartBeat();

                    System.Windows.Forms.Application.DoEvents();

                    // incase we are in setup mode
                    BaseStream.WriteLine("planner\rgcs\r");

                    System.Threading.Thread.Sleep(1);

                    System.Windows.Forms.Application.DoEvents();

                    buffer1 = getHeartBeat();

                    System.Windows.Forms.Application.DoEvents();

                    try
                    {
                        Console.WriteLine("MAv Data: len " + buffer.Length + " btr " + BaseStream.BytesToRead);
                    }
                    catch { }

                    count++;

                    if (buffer.Length > 5 && buffer1.Length > 5 && buffer[3] == buffer1[3] && buffer[4] == buffer1[4])
                    {
                        __mavlink_heartbeat_t hb = new __mavlink_heartbeat_t();

                        object temp = hb;

                        MAVLink.ByteArrayToStructure(buffer, ref temp, 6);

                        hb = (MAVLink.__mavlink_heartbeat_t)(temp);

                        mavlinkversion = hb.mavlink_version;
                        aptype = hb.type;

                        sysid = buffer[3];
                        compid = buffer[4];
                        recvpacketcount = buffer[2];
                        Console.WriteLine("ID sys " + sysid + " comp " + compid + " ver" + mavlinkversion);
                        break;
                    }
                }

                frm.Controls[0].Text = "Getting Params.. (sysid " + sysid + " compid " + compid + ") ";
                frm.Refresh();
                if (getparams == true)
                    getParamList();
            }
            catch (Exception e)
            {
                try
                {
                    BaseStream.Close();
                }
                catch { }
                MainV2.givecomport = false;
                frm.Close();
                throw e;
            }

            frm.Close();

            MainV2.givecomport = false;

            Console.WriteLine("Done open " + sysid + " " + compid);

            packetslost = 0;
        }

        byte[] StructureToByteArrayEndian(params object[] list)
        {
            // The copy is made becuase SetValue won't work on a struct.
            // Boxing was used because SetValue works on classes/objects.
            // Unfortunately, it results in 2 copy operations.
            object thisBoxed = list[0]; // Why make a copy?
            Type test = thisBoxed.GetType();

            int offset = 0;
            byte[] data = new byte[Marshal.SizeOf(thisBoxed)];

            // System.Net.IPAddress.NetworkToHostOrder is used to perform byte swapping.
            // To convert unsigned to signed, 'unchecked()' was used.
            // See http://stackoverflow.com/questions/1131843/how-do-i-convert-uint-to-int-in-c

            object fieldValue;
            TypeCode typeCode;

            byte[] temp;

            // Enumerate each structure field using reflection.
            foreach (var field in test.GetFields())
            {
                // field.Name has the field's name.

                fieldValue = field.GetValue(thisBoxed); // Get value

                // Get the TypeCode enumeration. Multiple types get mapped to a common typecode.
                typeCode = Type.GetTypeCode(fieldValue.GetType());

                switch (typeCode)
                {
                    case TypeCode.Single: // float
                        {
                            temp = BitConverter.GetBytes((Single)fieldValue);
                            Array.Reverse(temp);
                            Array.Copy(temp, 0, data, offset, sizeof(Single));
                            break;
                        }
                    case TypeCode.Int32:
                        {
                            temp = BitConverter.GetBytes((Int32)fieldValue);
                            Array.Reverse(temp);
                            Array.Copy(temp, 0, data, offset, sizeof(Int32));
                            break;
                        }
                    case TypeCode.UInt32:
                        {
                            temp = BitConverter.GetBytes((UInt32)fieldValue);
                            Array.Reverse(temp);
                            Array.Copy(temp, 0, data, offset, sizeof(UInt32));
                            break;
                        }
                    case TypeCode.Int16:
                        {
                            temp = BitConverter.GetBytes((Int16)fieldValue);
                            Array.Reverse(temp);
                            Array.Copy(temp, 0, data, offset, sizeof(Int16));
                            break;
                        }
                    case TypeCode.UInt16:
                        {
                            temp = BitConverter.GetBytes((UInt16)fieldValue);
                            Array.Reverse(temp);
                            Array.Copy(temp, 0, data, offset, sizeof(UInt16));
                            break;
                        }
                    case TypeCode.Int64:
                        {
                            temp = BitConverter.GetBytes((Int64)fieldValue);
                            Array.Reverse(temp);
                            Array.Copy(temp, 0, data, offset, sizeof(Int64));
                            break;
                        }
                    case TypeCode.UInt64:
                        {
                            temp = BitConverter.GetBytes((UInt64)fieldValue);
                            Array.Reverse(temp);
                            Array.Copy(temp, 0, data, offset, sizeof(UInt64));
                            break;
                        }
                    case TypeCode.Double:
                        {
                            temp = BitConverter.GetBytes((Double)fieldValue);
                            Array.Reverse(temp);
                            Array.Copy(temp, 0, data, offset, sizeof(Double));
                            break;
                        }
                    case TypeCode.Byte:
                        {
                            data[offset] = (Byte)fieldValue;
                            break;
                        }
                    default:
                        {
                            //System.Diagnostics.Debug.Fail("No conversion provided for this type : " + typeCode.ToString());
                            break;
                        }
                }; // switch
                if (typeCode == TypeCode.Object)
                {
                    int length = ((byte[])fieldValue).Length;
                    Array.Copy(((byte[])fieldValue), 0, data, offset, length);
                    offset += length;
                }
                else
                {
                    offset += Marshal.SizeOf(fieldValue);
                }
            } // foreach

            return data;
        } // Swap

        byte[] getHeartBeat()
        {
            DateTime start = DateTime.Now;
            while (true)
            {
                byte[] buffer = readPacket();
                if (buffer.Length > 5)
                {
                    if (buffer[5] == MAVLINK_MSG_ID_HEARTBEAT)
                    {
                        return buffer;
                    }
                }
                if (DateTime.Now > start.AddMilliseconds(2200)) // was 1200 , now 2.2 sec
                    return new byte[0];
            }
        }

        public void sendPacket(object indata)
        {
            bool run = false;
            byte a = 0;
            foreach (Type ty in mavstructs)
            {
                if (ty == indata.GetType())
                {
                    run = true;
                    generatePacket(a, indata);
                    return;
                }
                a++;
            }
            if (!run)
            {
                Console.WriteLine("Mavlink : NOT VALID PACKET sendPacket() " + indata.GetType().ToString());
            }
        }

        /// <summary>
        /// Generate a Mavlink Packet and write to serial
        /// </summary>
        /// <param name="messageType">type number</param>
        /// <param name="indata">struct of data</param>
        void generatePacket(byte messageType, object indata)
        {
            byte[] data;

            if (mavlinkversion == 3)
            {
                data = StructureToByteArray(indata);
            }
            else
            {
                data = StructureToByteArrayEndian(indata);
            }

            //Console.WriteLine(DateTime.Now + " PC Doing req "+ messageType + " " + this.BytesToRead);
            byte[] packet = new byte[data.Length + 6 + 2];

            if (mavlinkversion == 3)
            {
                packet[0] = 254;
            }
            else if (mavlinkversion == 2)
            {
                packet[0] = (byte)'U';
            }
            packet[1] = (byte)data.Length;
            packet[2] = packetcount;
            packet[3] = 255; // this is always 255 - MYGCS
#if MAVLINK10
            packet[4] = (byte)MAV_COMPONENT.MAV_COMP_ID_MISSIONPLANNER;
#else
            packet[4] = (byte)MAV_COMPONENT.MAV_COMP_ID_WAYPOINTPLANNER;
#endif
            packet[5] = messageType;

            int i = 6;
            foreach (byte b in data)
            {
                packet[i] = b;
                i++;
            }

            ushort checksum = crc_calculate(packet, packet[1] + 6);

            if (mavlinkversion == 3)
            {
                checksum = crc_accumulate(MAVLINK_MESSAGE_CRCS[messageType], checksum);
            }

            byte ck_a = (byte)(checksum & 0xFF); ///< High byte
            byte ck_b = (byte)(checksum >> 8); ///< Low byte

            packet[i] = ck_a;
            i += 1;
            packet[i] = ck_b;
            i += 1;

            if (BaseStream.IsOpen)
            {
                lock (objlock)
                {
                    BaseStream.Write(packet, 0, i);
                }
            }

            try
            {
                if (logfile != null)
                {
                    lock (logwritelock)
                    {
                        byte[] datearray = BitConverter.GetBytes((UInt64)((DateTime.UtcNow - new DateTime(1970, 1, 1)).TotalMilliseconds * 1000)); //ASCIIEncoding.ASCII.GetBytes(DateTime.Now.ToBinary() + ":");
                        Array.Reverse(datearray);
                        logfile.Write(datearray, 0, datearray.Length);
                        logfile.Write(packet, 0, i);
                    }
                }

            }
            catch { }

            if (messageType == ArdupilotMega.MAVLink.MAVLINK_MSG_ID_REQUEST_DATA_STREAM)
            {
                try
                {
                    BinaryWriter bw = new BinaryWriter(File.OpenWrite("serialsent.raw"));
                    bw.Seek(0, SeekOrigin.End);
                    bw.Write(packet, 0, i);
                    bw.Write((byte)'\n');
                    bw.Close();
                }
                catch { } // been getting errors from this. people must have it open twice.
            }

            packetcount++;



            //System.Threading.Thread.Sleep(1);
        }

        public bool Write(string line)
        {
            lock (objlock)
            {
                BaseStream.Write(line);
            }
            return true;
        }

        /// <summary>
        /// Set parameter on apm
        /// </summary>
        /// <param name="paramname">name as a string</param>
        /// <param name="value"></param>
        public bool setParam(string paramname, float value)
        {
            if (!param.ContainsKey(paramname))
            {
                Console.WriteLine("Param doesnt exist " + paramname);
                return false;
            }

            MainV2.givecomport = true;

            __mavlink_param_set_t req = new __mavlink_param_set_t();
            req.target_system = sysid;
            req.target_component = compid;

            byte[] temp = ASCIIEncoding.ASCII.GetBytes(paramname);

            modifyParamForDisplay(false, paramname, ref value);
#if MAVLINK10
            Array.Resize(ref temp, 16);
#else
            Array.Resize(ref temp, 15);
#endif
            req.param_id = temp;
            req.param_value = (value);

            generatePacket(MAVLINK_MSG_ID_PARAM_SET, req);

            Console.WriteLine("setParam '{0}' = '{1}' sysid {2} compid {3}", paramname, req.param_value, sysid, compid);

            DateTime start = DateTime.Now;
            int retrys = 3;

            while (true)
            {
                if (!(start.AddMilliseconds(500) > DateTime.Now))
                {
                    if (retrys > 0)
                    {
                        Console.WriteLine("setParam Retry " + retrys);
                        generatePacket(MAVLINK_MSG_ID_PARAM_SET, req);
                        start = DateTime.Now;
                        retrys--;
                        continue;
                    }
                    MainV2.givecomport = false;
                    throw new Exception("Timeout on read - setParam " + paramname);
                }

                byte[] buffer = readPacket();
                if (buffer.Length > 5)
                {
                    if (buffer[5] == MAVLINK_MSG_ID_PARAM_VALUE)
                    {
                        __mavlink_param_value_t par = new __mavlink_param_value_t();

                        object tempobj = par;

                        ByteArrayToStructure(buffer, ref tempobj, 6);

                        par = (__mavlink_param_value_t)tempobj;

                        string st = System.Text.ASCIIEncoding.ASCII.GetString(par.param_id);

                        int pos = st.IndexOf('\0');

                        if (pos != -1)
                        {
                            st = st.Substring(0, pos);
                        }

                        if (st != paramname)
                        {
                            Console.WriteLine("MAVLINK bad param responce - {0} vs {1}", paramname, st);
                            continue;
                        }

                        modifyParamForDisplay(true, st, ref par.param_value);

                        param[st] = (par.param_value);

                        MainV2.givecomport = false;
                        //System.Threading.Thread.Sleep(100);//(int)(8.5 * 5)); // 8.5ms per byte
                        return true;
                    }
                }
            }
        }

        /// <summary>
        /// Get param list from apm
        /// </summary>
        /// <returns></returns>
        public Hashtable getParamList()
        {
            MainV2.givecomport = true;
            List<int> missed = new List<int>();

            // ryan - re start
            __mavlink_param_request_read_t rereq = new __mavlink_param_request_read_t();
            rereq.target_system = sysid;
            rereq.target_component = compid;

            __mavlink_param_request_list_t req = new __mavlink_param_request_list_t();
            req.target_system = sysid;
            req.target_component = compid;

            generatePacket(MAVLINK_MSG_ID_PARAM_REQUEST_LIST, req);

            DateTime start = DateTime.Now;
            DateTime restart = DateTime.Now;

            int retrys = 3;
            int nextid = 0;
            int param_count = 0;
            int param_total = 5;
            while (param_count < param_total)
            {
                if (!(start.AddMilliseconds(5000) > DateTime.Now))
                {
                    if (retrys > 0)
                    {
                        Console.WriteLine("getParamList Retry " + retrys + " sys " + sysid + " comp " + compid);
                        generatePacket(MAVLINK_MSG_ID_PARAM_REQUEST_LIST, req);
                        start = DateTime.Now;
                        retrys--;
                        continue;
                    }
                    MainV2.givecomport = false;
                    throw new Exception("Timeout on read - getParamList");
                }
                if (!(restart.AddMilliseconds(1000) > DateTime.Now))
                {
                    rereq.param_id = new byte[] { 0x0, 0x0 };
                    rereq.param_index = (short)nextid;
                    sendPacket(rereq);
                    restart = DateTime.Now;
                }

                System.Windows.Forms.Application.DoEvents();
                byte[] buffer = readPacket();
                if (buffer.Length > 5)
                {
                    //stopwatch.Reset();
                    //stopwatch.Start();
                    if (buffer[5] == MAVLINK_MSG_ID_PARAM_VALUE)
                    {
                        restart = DateTime.Now;
                        start = DateTime.Now;

                        __mavlink_param_value_t par = new __mavlink_param_value_t();

                        object temp = par;

                        ByteArrayToStructure(buffer, ref temp, 6);

                        par = (__mavlink_param_value_t)temp;

                        param_total = (par.param_count);

                        // for out of order udp packets
                        if (BaseStream.GetType() != typeof(UdpSerial))
                        {
                            if (nextid == (par.param_index))
                            {
                                nextid++;
                            }
                            else
                            {

                                if (retrys > 0)
                                {
                                    generatePacket(MAVLINK_MSG_ID_PARAM_REQUEST_LIST, req);
                                    param_count = 0;
                                    nextid = 0;
                                    retrys--;
                                    continue;
                                }
                                missed.Add(nextid); // for later devel
                                MainV2.givecomport = false;
                                throw new Exception("Missed ID expecting " + nextid + " got " + (par.param_index) + "\nPlease try loading again");
                            }
                        }                        

                        string st = System.Text.ASCIIEncoding.ASCII.GetString(par.param_id);

                        int pos = st.IndexOf('\0');

                        if (pos != -1)
                        {
                            st = st.Substring(0, pos);
                        }

                        Console.WriteLine(DateTime.Now.Millisecond + " got param " + (par.param_index) + " of " + (param_total - 1) + " name: " + st);

                        modifyParamForDisplay(true, st, ref par.param_value);

                        param[st] = (par.param_value);

                        param_count++;
                    }
                    else
                    {
                        //Console.WriteLine(DateTime.Now + " PC paramlist " + buffer[5] + " " + this.BytesToRead);
                    }
                    //stopwatch.Stop();
                    //Console.WriteLine("Time elapsed: {0}", stopwatch.Elapsed);
                }
            }
            MainV2.givecomport = false;
            return param;
        }

        public static void modifyParamForDisplay(bool fromapm, string paramname, ref float value)
        {
            if (paramname.ToUpper().EndsWith("_IMAX") || paramname.ToUpper().EndsWith("ALT_HOLD_RTL") || paramname.ToUpper().EndsWith("TRIM_ARSPD_CM")
                || paramname.ToUpper().EndsWith("XTRK_ANGLE_CD") || paramname.ToUpper().EndsWith("LIM_PITCH_MAX") || paramname.ToUpper().EndsWith("LIM_PITCH_MIN")
                || paramname.ToUpper().EndsWith("LIM_ROLL_CD") || paramname.ToUpper().EndsWith("PITCH_MAX") || paramname.ToUpper().EndsWith("WP_SPEED_MAX"))
            {
                if (paramname.ToUpper().EndsWith("THR_HOLD_IMAX"))
                {
                    return;
                }

                if (fromapm)
                {
                    value /= 100.0f;
                }
                else
                {
                    value *= 100.0f;
                }
            }
        }

        /// <summary>
        /// Stops all requested data packets.
        /// </summary>
        public void stopall(bool forget)
        {
            __mavlink_request_data_stream_t req = new __mavlink_request_data_stream_t();
            req.target_system = sysid;
            req.target_component = compid;

            req.req_message_rate = 10;
            req.start_stop = 0; // stop
            req.req_stream_id = 0; // all

            // reset all
            if (forget)
            {
                streams = new byte[streams.Length];
            }

            // no error on bad
            try
            {
                generatePacket(MAVLINK_MSG_ID_REQUEST_DATA_STREAM, req);
                System.Threading.Thread.Sleep(20);
                generatePacket(MAVLINK_MSG_ID_REQUEST_DATA_STREAM, req);
                System.Threading.Thread.Sleep(20);
                generatePacket(MAVLINK_MSG_ID_REQUEST_DATA_STREAM, req);
                Console.WriteLine("Stopall Done");

            }
            catch { }
        }

        public void setWPACK()
        {
#if MAVLINK10
            MAVLink.__mavlink_mission_ack_t req = new MAVLink.__mavlink_mission_ack_t();
            req.target_system = sysid;
            req.target_component = compid;
            req.type = 0;

            generatePacket(MAVLINK_MSG_ID_MISSION_ACK, req);
#else
            MAVLink.__mavlink_waypoint_ack_t req = new MAVLink.__mavlink_waypoint_ack_t();
            req.target_system = sysid;
            req.target_component = compid;
            req.type = 0;

            generatePacket(MAVLINK_MSG_ID_WAYPOINT_ACK, req);
#endif
        }

        public bool setWPCurrent(ushort index)
        {
#if MAVLINK10
            MainV2.givecomport = true;
            byte[] buffer;

            __mavlink_mission_set_current_t req = new __mavlink_mission_set_current_t();

            req.target_system = sysid;
            req.target_component = compid;
            req.seq = index;

            generatePacket(MAVLINK_MSG_ID_MISSION_SET_CURRENT, req);

            DateTime start = DateTime.Now;
            int retrys = 5;

            while (true)
            {
                if (!(start.AddMilliseconds(2000) > DateTime.Now))
                {
                    if (retrys > 0)
                    {
                        Console.WriteLine("setWPCurrent Retry " + retrys);
                        generatePacket(MAVLINK_MSG_ID_MISSION_SET_CURRENT, req);
                        start = DateTime.Now;
                        retrys--;
                        continue;
                    }
                    MainV2.givecomport = false;
                    throw new Exception("Timeout on read - setWPCurrent");
                }

                buffer = readPacket();
                if (buffer.Length > 5)
                {
                    if (buffer[5] == MAVLINK_MSG_ID_MISSION_CURRENT)
                    {
                        MainV2.givecomport = false;
                        return true;
                    }
                }
            }
        }

        public bool doCommand(MAV_CMD actionid, float p1, float p2, float p3, float p4, float p5, float p6, float p7)
        {

            MainV2.givecomport = true;
            byte[] buffer;

            __mavlink_command_long_t req = new __mavlink_command_long_t();

            req.target_system = sysid;
            req.target_component = compid;

            req.command = (ushort)actionid;

            req.param1 = p1;
            req.param2 = p2;
            req.param3 = p3;
            req.param4 = p4;
            req.param5 = p5;
            req.param6 = p6;
            req.param7 = p7;

            generatePacket(MAVLINK_MSG_ID_COMMAND_LONG, req);

            DateTime start = DateTime.Now;
            int retrys = 3;

            int timeout = 2000;

            // imu calib take a little while
            if (actionid == MAV_CMD.PREFLIGHT_CALIBRATION)
            {
                retrys = 1;
                timeout = 6000;
            }

            while (true)
            {
                if (!(start.AddMilliseconds(timeout) > DateTime.Now))
                {
                    if (retrys > 0)
                    {
                        Console.WriteLine("doAction Retry " + retrys);
                        generatePacket(MAVLINK_MSG_ID_COMMAND_LONG, req);
                        start = DateTime.Now;
                        retrys--;
                        continue;
                    }
                    MainV2.givecomport = false;
                    throw new Exception("Timeout on read - doAction");
                }

                buffer = readPacket();
                if (buffer.Length > 5)
                {
                    if (buffer[5] == MAVLINK_MSG_ID_COMMAND_ACK)
                    {
                        __mavlink_command_ack_t ack = new __mavlink_command_ack_t();

                        object temp = (object)ack;

                        ByteArrayToStructure(buffer, ref temp, 6);

                        ack = (__mavlink_command_ack_t)(temp);

                        if (ack.result == (byte)MAV_RESULT.MAV_RESULT_ACCEPTED)
                        {
                            MainV2.givecomport = false;
                            return true;
                        }
                        else
                        {
                            MainV2.givecomport = false;
                            return false;
                        }
                    }
                }
            }
#else
            MainV2.givecomport = true;
            byte[] buffer;

            __mavlink_waypoint_set_current_t req = new __mavlink_waypoint_set_current_t();

            req.target_system = sysid;
            req.target_component = compid;
            req.seq = index;

            generatePacket(MAVLINK_MSG_ID_WAYPOINT_SET_CURRENT, req);

            DateTime start = DateTime.Now;
            int retrys = 5;

            while (true)
            {
                if (!(start.AddMilliseconds(2000) > DateTime.Now))
                {
                    if (retrys > 0)
                    {
                        Console.WriteLine("setWPCurrent Retry " + retrys);
                        generatePacket(MAVLINK_MSG_ID_WAYPOINT_SET_CURRENT, req);
                        start = DateTime.Now;
                        retrys--;
                        continue;
                    }
                    MainV2.givecomport = false;
                    throw new Exception("Timeout on read - setWPCurrent");
                }

                buffer = readPacket();
                if (buffer.Length > 5)
                {
                    if (buffer[5] == MAVLINK_MSG_ID_WAYPOINT_CURRENT)
                    {
                        MainV2.givecomport = false;
                        return true;
                    }
                }
            }
        }

        public bool doAction(MAV_ACTION actionid)
        {
            MainV2.givecomport = true;
            byte[] buffer;

            __mavlink_action_t req = new __mavlink_action_t();

            req.target = sysid;
            req.target_component = compid;

            req.action = (byte)actionid;

            generatePacket(MAVLINK_MSG_ID_ACTION, req);

            DateTime start = DateTime.Now;
            int retrys = 3;

            int timeout = 2000;

            // imu calib take a little while
            if (actionid == MAV_ACTION.MAV_ACTION_CALIBRATE_ACC ||
                actionid == MAV_ACTION.MAV_ACTION_CALIBRATE_GYRO ||
                actionid == MAV_ACTION.MAV_ACTION_CALIBRATE_MAG ||
                actionid == MAV_ACTION.MAV_ACTION_CALIBRATE_PRESSURE ||
                actionid == MAV_ACTION.MAV_ACTION_REBOOT)
            {
                retrys = 1;
                timeout = 6000;
            }

            while (true)
            {
                if (!(start.AddMilliseconds(timeout) > DateTime.Now))
                {
                    if (retrys > 0)
                    {
                        Console.WriteLine("doAction Retry " + retrys);
                        generatePacket(MAVLINK_MSG_ID_ACTION, req);
                        start = DateTime.Now;
                        retrys--;
                        continue;
                    }
                    MainV2.givecomport = false;
                    throw new Exception("Timeout on read - doAction");
                }

                buffer = readPacket();
                if (buffer.Length > 5)
                {
                    if (buffer[5] == MAVLINK_MSG_ID_ACTION_ACK)
                    {
                        if (buffer[7] == 1)
                        {
                            MainV2.givecomport = false;
                            return true;
                        }
                        else
                        {
                            MainV2.givecomport = false;
                            return false;
                        }
                    }
                }
            }

#endif
        }

        public void requestDatastream(byte id, byte hzrate)
        {
            streams[id] = hzrate;

            double pps = 0;

            switch (id)
            {
                case (byte)MAVLink.MAV_DATA_STREAM.MAV_DATA_STREAM_ALL:

                    break;
                case (byte)MAVLink.MAV_DATA_STREAM.MAV_DATA_STREAM_EXTENDED_STATUS:
                    if (packetspersecondbuild[MAVLINK_MSG_ID_SYS_STATUS] < DateTime.Now.AddSeconds(-2))
                        break;
                    pps = packetspersecond[MAVLINK_MSG_ID_SYS_STATUS];
                    if (hzratecheck(pps, hzrate))
                    {
                        return;
                    }
                    break;
                case (byte)MAVLink.MAV_DATA_STREAM.MAV_DATA_STREAM_EXTRA1:
                    if (packetspersecondbuild[MAVLINK_MSG_ID_ATTITUDE] < DateTime.Now.AddSeconds(-2))
                        break;
                    pps = packetspersecond[MAVLINK_MSG_ID_ATTITUDE];
                    if (hzratecheck(pps, hzrate))
                    {
                        return;
                    }
                    break;
                case (byte)MAVLink.MAV_DATA_STREAM.MAV_DATA_STREAM_EXTRA2:
                    if (packetspersecondbuild[MAVLINK_MSG_ID_VFR_HUD] < DateTime.Now.AddSeconds(-2))
                        break;
                    pps = packetspersecond[MAVLINK_MSG_ID_VFR_HUD];
                    if (hzratecheck(pps, hzrate))
                    {
                        return;
                    }
                    break;
                case (byte)MAVLink.MAV_DATA_STREAM.MAV_DATA_STREAM_EXTRA3:

                    break;
                case (byte)MAVLink.MAV_DATA_STREAM.MAV_DATA_STREAM_POSITION:
                    if (packetspersecondbuild[MAVLINK_MSG_ID_GLOBAL_POSITION_INT] < DateTime.Now.AddSeconds(-2))
                        break;
                    pps = packetspersecond[MAVLINK_MSG_ID_GLOBAL_POSITION_INT];
                    if (hzratecheck(pps, hzrate))
                    {
                        return;
                    }
                    break;
                case (byte)MAVLink.MAV_DATA_STREAM.MAV_DATA_STREAM_RAW_CONTROLLER:
                    if (packetspersecondbuild[MAVLINK_MSG_ID_RC_CHANNELS_SCALED] < DateTime.Now.AddSeconds(-2))
                        break;
                    pps = packetspersecond[MAVLINK_MSG_ID_RC_CHANNELS_SCALED];
                    if (hzratecheck(pps, hzrate))
                    {
                        return;
                    }
                    break;
                case (byte)MAVLink.MAV_DATA_STREAM.MAV_DATA_STREAM_RAW_SENSORS:
                    if (packetspersecondbuild[MAVLINK_MSG_ID_RAW_IMU] < DateTime.Now.AddSeconds(-2))
                        break;
                    pps = packetspersecond[MAVLINK_MSG_ID_RAW_IMU];
                    if (hzratecheck(pps, hzrate))
                    {
                        return;
                    }
                    break;
                case (byte)MAVLink.MAV_DATA_STREAM.MAV_DATA_STREAM_RC_CHANNELS:
                    if (packetspersecondbuild[MAVLINK_MSG_ID_RC_CHANNELS_RAW] < DateTime.Now.AddSeconds(-2))
                        break;
                    pps = packetspersecond[MAVLINK_MSG_ID_RC_CHANNELS_RAW];
                    if (hzratecheck(pps, hzrate))
                    {
                        return;
                    }
                    break;
            }

            //packetspersecond[temp[5]];

            if (pps == 0 && hzrate == 0)
            {
                return;
            }

            Console.WriteLine("Request stream {0} at {1} hz : currently {2}", Enum.Parse(typeof(MAV_DATA_STREAM), id.ToString()), hzrate, pps);
            getDatastream(id, hzrate);
        }

        // returns true for ok
        bool hzratecheck(double pps, int hzrate)
        {

            if (hzrate == 0 && pps == 0)
            {
                return true;
            }
            else if (hzrate == 1 && pps >= 0.5 && pps <= 2)
            {
                return true;
            }
            else if (hzrate == 3 && pps >= 2 && hzrate < 5)
            {
                return true;
            }
            else if (hzrate == 10 && pps > 5 && hzrate < 15)
            {
                return true;
            }
            else if (hzrate > 15 && pps > 15)
            {
                return true;
            }

            return false;

        }

        void getDatastream(byte id, byte hzrate)
        {
            __mavlink_request_data_stream_t req = new __mavlink_request_data_stream_t();
            req.target_system = sysid;
            req.target_component = compid;

            req.req_message_rate = hzrate;
            req.start_stop = 1; // start
            req.req_stream_id = id; // id

            generatePacket(MAVLINK_MSG_ID_REQUEST_DATA_STREAM, req);
            generatePacket(MAVLINK_MSG_ID_REQUEST_DATA_STREAM, req);
        }

        /// <summary>
        /// Returns WP count
        /// </summary>
        /// <returns></returns>
        public byte getWPCount()
        {
            MainV2.givecomport = true;
            byte[] buffer;
#if MAVLINK10
            __mavlink_mission_request_list_t req = new __mavlink_mission_request_list_t();

            req.target_system = sysid;
            req.target_component = compid;

            // request list
            generatePacket(MAVLINK_MSG_ID_MISSION_REQUEST_LIST, req);

            DateTime start = DateTime.Now;
            int retrys = 6;

            while (true)
            {
                if (!(start.AddMilliseconds(500) > DateTime.Now))
                {
                    if (retrys > 0)
                    {
                        Console.WriteLine("getWPCount Retry " + retrys + " - giv com " + MainV2.givecomport);
                        generatePacket(MAVLINK_MSG_ID_MISSION_REQUEST_LIST, req);
                        start = DateTime.Now;
                        retrys--;
                        continue;
                    }
                    MainV2.givecomport = false;
                    //return (byte)int.Parse(param["WP_TOTAL"].ToString());
                    throw new Exception("Timeout on read - getWPCount");
                }

                buffer = readPacket();
                if (buffer.Length > 5)
                {
                    if (buffer[5] == MAVLINK_MSG_ID_MISSION_COUNT)
                    {
                        __mavlink_mission_count_t count = new __mavlink_mission_count_t();

                        object temp = (object)count;

                        ByteArrayToStructure(buffer, ref temp, 6);

                        count = (__mavlink_mission_count_t)(temp);


                        Console.WriteLine("wpcount: " + count.count);
                        MainV2.givecomport = false;
                        return (byte)count.count; // should be ushort, but apm has limited wp count < byte
                    }
                    else
                    {
                        Console.WriteLine(DateTime.Now + " PC wpcount " + buffer[5] + " need " + MAVLINK_MSG_ID_MISSION_COUNT + " " + this.BaseStream.BytesToRead);
                    }
                }
            }
#else

            __mavlink_waypoint_request_list_t req = new __mavlink_waypoint_request_list_t();

            req.target_system = sysid;
            req.target_component = compid;

            // request list
            generatePacket(MAVLINK_MSG_ID_WAYPOINT_REQUEST_LIST, req);

            DateTime start = DateTime.Now;
            int retrys = 6;

            while (true)
            {
                if (!(start.AddMilliseconds(500) > DateTime.Now))
                {
                    if (retrys > 0)
                    {
                        Console.WriteLine("getWPCount Retry " + retrys + " - giv com " + MainV2.givecomport);
                        generatePacket(MAVLINK_MSG_ID_WAYPOINT_REQUEST_LIST, req);
                        start = DateTime.Now;
                        retrys--;
                        continue;
                    }
                    MainV2.givecomport = false;
                    //return (byte)int.Parse(param["WP_TOTAL"].ToString());
                    throw new Exception("Timeout on read - getWPCount");
                }

                buffer = readPacket();
                if (buffer.Length > 5)
                {
                    if (buffer[5] == MAVLINK_MSG_ID_WAYPOINT_COUNT)
                    {

                        Console.WriteLine("wpcount: " + buffer[9]);
                        MainV2.givecomport = false;
                        return buffer[9]; // should be ushort, but apm has limited wp count < byte
                    }
                    else
                    {
                        Console.WriteLine(DateTime.Now + " PC wpcount " + buffer[5] + " need " + MAVLINK_MSG_ID_WAYPOINT_COUNT + " " + this.BaseStream.BytesToRead);
                    }
                }
            }

#endif
        }
        /// <summary>
        /// Gets specfied WP
        /// </summary>
        /// <param name="index"></param>
        /// <returns>WP</returns>
        public Locationwp getWP(ushort index)
        {
            MainV2.givecomport = true;
            Locationwp loc = new Locationwp();
#if MAVLINK10
            __mavlink_mission_request_t req = new __mavlink_mission_request_t();

            req.target_system = sysid;
            req.target_component = compid;

            req.seq = index;

            //Console.WriteLine("getwp req "+ DateTime.Now.Millisecond);

            // request
            generatePacket(MAVLINK_MSG_ID_MISSION_REQUEST, req);

            DateTime start = DateTime.Now;
            int retrys = 5;

            while (true)
            {
                if (!(start.AddMilliseconds(800) > DateTime.Now)) // apm times out after 1000ms
                {
                    if (retrys > 0)
                    {
                        Console.WriteLine("getWP Retry " + retrys);
                        generatePacket(MAVLINK_MSG_ID_MISSION_REQUEST, req);
                        start = DateTime.Now;
                        retrys--;
                        continue;
                    }
                    MainV2.givecomport = false;
                    throw new Exception("Timeout on read - getWP");
                }
                //Console.WriteLine("getwp read " + DateTime.Now.Millisecond);
                byte[] buffer = readPacket();
                //Console.WriteLine("getwp readend " + DateTime.Now.Millisecond);
                if (buffer.Length > 5)
                {
                    if (buffer[5] == MAVLINK_MSG_ID_MISSION_ITEM)
                    {
                        //Console.WriteLine("getwp ans " + DateTime.Now.Millisecond);
                        __mavlink_mission_item_t wp = new __mavlink_mission_item_t();

                        object temp = (object)wp;

                        //Array.Copy(buffer, 6, buffer, 0, buffer.Length - 6);

                        ByteArrayToStructure(buffer, ref temp, 6);

                        wp = (__mavlink_mission_item_t)(temp);

#else

            __mavlink_waypoint_request_t req = new __mavlink_waypoint_request_t();

            req.target_system = sysid;
            req.target_component = compid;

            req.seq = index;

            //Console.WriteLine("getwp req "+ DateTime.Now.Millisecond);

            // request
            generatePacket(MAVLINK_MSG_ID_WAYPOINT_REQUEST, req);

            DateTime start = DateTime.Now;
            int retrys = 5;

            while (true)
            {
                if (!(start.AddMilliseconds(800) > DateTime.Now)) // apm times out after 1000ms
                {
                    if (retrys > 0)
                    {
                        Console.WriteLine("getWP Retry " + retrys);
                        generatePacket(MAVLINK_MSG_ID_WAYPOINT_REQUEST, req);
                        start = DateTime.Now;
                        retrys--;
                        continue;
                    }
                    MainV2.givecomport = false;
                    throw new Exception("Timeout on read - getWP");
                }
                //Console.WriteLine("getwp read " + DateTime.Now.Millisecond);
                byte[] buffer = readPacket();
                //Console.WriteLine("getwp readend " + DateTime.Now.Millisecond);
                if (buffer.Length > 5)
                {
                    if (buffer[5] == MAVLINK_MSG_ID_WAYPOINT)
                    {
                        //Console.WriteLine("getwp ans " + DateTime.Now.Millisecond);
                        __mavlink_waypoint_t wp = new __mavlink_waypoint_t();

                        object temp = (object)wp;

                        //Array.Copy(buffer, 6, buffer, 0, buffer.Length - 6);

                        ByteArrayToStructure(buffer, ref temp, 6);

                        wp = (__mavlink_waypoint_t)(temp);

#endif

                        loc.options = (byte)(wp.frame & 0x1);
                        loc.id = (byte)(wp.command);
                        loc.p1 = (wp.param1);
                        loc.p2 = (wp.param2);
                        loc.p3 = (wp.param3);
                        loc.p4 = (wp.param4);

                        loc.alt = ((wp.z));
                        loc.lat = ((wp.x));
                        loc.lng = ((wp.y));
                        /*
                        if (MainV2.cs.firmware == MainV2.Firmwares.ArduPlane)
                        {
                            switch (loc.id)
                            {					// Switch to map APM command fields inot MAVLink command fields
                                case (byte)MAV_CMD.LOITER_TURNS:
                                case (byte)MAV_CMD.TAKEOFF:
                                case (byte)MAV_CMD.DO_SET_HOME:
                                    //case (byte)MAV_CMD.DO_SET_ROI:
                                    loc.alt = (float)((wp.z));
                                    loc.lat = (float)((wp.x));
                                    loc.lng = (float)((wp.y));
                                    loc.p1 = (float)wp.param1;
                                    break;

                                case (byte)MAV_CMD.CONDITION_CHANGE_ALT:
                                    loc.lat = (int)wp.param1;
                                    loc.p1 = 0;
                                    break;

                                case (byte)MAV_CMD.LOITER_TIME:
                                    if (MainV2.APMFirmware == MainV2.Firmwares.ArduPlane)
                                    {
                                        loc.p1 = (byte)(wp.param1 / 10);	// APM loiter time is in ten second increments
                                    }
                                    else
                                    {
                                        loc.p1 = (byte)wp.param1;
                                    }
                                    break;

                                case (byte)MAV_CMD.CONDITION_DELAY:
                                case (byte)MAV_CMD.CONDITION_DISTANCE:
                                    loc.lat = (int)wp.param1;
                                    break;

                                case (byte)MAV_CMD.DO_JUMP:
                                    loc.lat = (int)wp.param2;
                                    loc.p1 = (byte)wp.param1;
                                    break;

                                case (byte)MAV_CMD.DO_REPEAT_SERVO:
                                    loc.lng = (int)wp.param4;
                                    goto case (byte)MAV_CMD.DO_CHANGE_SPEED;
                                case (byte)MAV_CMD.DO_REPEAT_RELAY:
                                case (byte)MAV_CMD.DO_CHANGE_SPEED:
                                    loc.lat = (int)wp.param3;
                                    loc.alt = (int)wp.param2;
                                    loc.p1 = (byte)wp.param1;
                                    break;

                                case (byte)MAV_CMD.DO_SET_PARAMETER:
                                case (byte)MAV_CMD.DO_SET_RELAY:
                                case (byte)MAV_CMD.DO_SET_SERVO:
                                    loc.alt = (int)wp.param2;
                                    loc.p1 = (byte)wp.param1;
                                    break;

                                case (byte)MAV_CMD.WAYPOINT:
                                    loc.p1 = (byte)wp.param1;
                                    break;
                            }
                        }
                        */
                        Console.WriteLine("getWP {0} {1} {2} {3} {4} opt {5}", loc.id, loc.p1, loc.alt, loc.lat, loc.lng, loc.options);

                        break;
                    }
                    else
                    {
                        Console.WriteLine(DateTime.Now + " PC getwp " + buffer[5]);
                    }
                }
            }
            MainV2.givecomport = false;
            return loc;
        }

        public object DebugPacket(byte[] datin)
        {
            string text = "";
            return DebugPacket(datin, ref text);
        }

        /// <summary>
        /// Print entire decoded packet to console
        /// </summary>
        /// <param name="datin">packet byte array</param>
        /// <returns>struct of data</returns>
        public object DebugPacket(byte[] datin, ref string text)
        {
            string textoutput;
            try
            {
                if (datin.Length > 5)
                {
                    byte header = datin[0];
                    byte length = datin[1];
                    byte seq = datin[2];
                    byte sysid = datin[3];
                    byte compid = datin[4];
                    byte messid = datin[5];

                    textoutput = string.Format("{0:X} {1:X} {2:X} {3:X} {4:X} {5:X} ", header, length, seq, sysid, compid, messid);

                    object data = Activator.CreateInstance(mavstructs[messid]);

                    ByteArrayToStructure(datin, ref data, 6);

                    Type test = data.GetType();

                    textoutput = textoutput + test.Name + " ";

                    foreach (var field in test.GetFields())
                    {
                        // field.Name has the field's name.

                        object fieldValue = field.GetValue(data); // Get value

                        if (field.FieldType.IsArray)
                        {
                            textoutput = textoutput + field.Name + "=";
                            byte[] crap = (byte[])fieldValue;
                            foreach (byte fiel in crap)
                            {
                                textoutput = textoutput + fiel + ",";
                            }
                            textoutput = textoutput + " ";
                        }
                        else
                        {
                            textoutput = textoutput + field.Name + "=" + fieldValue.ToString() + " ";
                        }
                    }
                    textoutput = textoutput + " Len:" + datin.Length + "\r\n";
                    Console.Write(textoutput);

                    if (text != null)
                        text = textoutput;

                    return data;
                }
            }
            catch { }

            return null;
        }

        /// <summary>
        /// Sets wp total count
        /// </summary>
        /// <param name="wp_total"></param>
        public void setWPTotal(ushort wp_total)
        {
#if MAVLINK10		
            MainV2.givecomport = true;
            __mavlink_mission_count_t req = new __mavlink_mission_count_t();

            req.target_system = sysid;
            req.target_component = compid; // MAVLINK_MSG_ID_MISSION_COUNT

            req.count = wp_total;

            generatePacket(MAVLINK_MSG_ID_MISSION_COUNT, req);

            DateTime start = DateTime.Now;
            int retrys = 3;

            while (true)
            {
                if (!(start.AddMilliseconds(700) > DateTime.Now))
                {
                    if (retrys > 0)
                    {
                        Console.WriteLine("setWPTotal Retry " + retrys);
                        generatePacket(MAVLINK_MSG_ID_MISSION_COUNT, req);
                        start = DateTime.Now;
                        retrys--;
                        continue;
                    }
                    MainV2.givecomport = false;
                    throw new Exception("Timeout on read - setWPTotal");
                }
                byte[] buffer = readPacket();
                if (buffer.Length > 9)
                {
                    if (buffer[5] == MAVLINK_MSG_ID_MISSION_REQUEST)
                    {
                        __mavlink_mission_request_t request = new __mavlink_mission_request_t();

                        object temp = (object)request;

                        ByteArrayToStructure(buffer, ref temp, 6);

                        request = (__mavlink_mission_request_t)(temp);

                        if (request.seq == 0)
                        {
                            if (param["WP_TOTAL"] != null)
                                param["WP_TOTAL"] = (float)wp_total - 1;
                            if (param["CMD_TOTAL"] != null)
                                param["CMD_TOTAL"] = (float)wp_total - 1;
                            MainV2.givecomport = false;
                            return;
                        }
                    }
                    else
                    {
                        //Console.WriteLine(DateTime.Now + " PC getwp " + buffer[5]);
                    }
                }
            }
#else
            MainV2.givecomport = true;
            __mavlink_waypoint_count_t req = new __mavlink_waypoint_count_t();

            req.target_system = sysid;
            req.target_component = compid; // MAVLINK_MSG_ID_WAYPOINT_COUNT

            req.count = wp_total;

            generatePacket(MAVLINK_MSG_ID_WAYPOINT_COUNT, req);

            DateTime start = DateTime.Now;
            int retrys = 3;

            while (true)
            {
                if (!(start.AddMilliseconds(700) > DateTime.Now))
                {
                    if (retrys > 0)
                    {
                        Console.WriteLine("setWPTotal Retry " + retrys);
                        generatePacket(MAVLINK_MSG_ID_WAYPOINT_COUNT, req);
                        start = DateTime.Now;
                        retrys--;
                        continue;
                    }
                    MainV2.givecomport = false;
                    throw new Exception("Timeout on read - setWPTotal");
                }
                byte[] buffer = readPacket();
                if (buffer.Length > 9)
                {
                    if (buffer[5] == MAVLINK_MSG_ID_WAYPOINT_REQUEST)
                    {
                        __mavlink_waypoint_request_t request = new __mavlink_waypoint_request_t();

                        object temp = (object)request;

                        ByteArrayToStructure(buffer, ref temp, 6);

                        request = (__mavlink_waypoint_request_t)(temp);

                        if (request.seq == 0)
                        {
                            if (param["WP_TOTAL"] != null)
                                param["WP_TOTAL"] = (float)wp_total - 1;
                            if (param["CMD_TOTAL"] != null)
                                param["CMD_TOTAL"] = (float)wp_total - 1;
                            MainV2.givecomport = false;
                            return;
                        }
                    }
                    else
                    {
                        //Console.WriteLine(DateTime.Now + " PC getwp " + buffer[5]);
                    }
                }
            }

#endif
        }

        /// <summary>
        /// Save wp to eeprom
        /// </summary>
        /// <param name="loc">location struct</param>
        /// <param name="index">wp no</param>
        /// <param name="frame">global or relative</param>
        /// <param name="current">0 = no , 2 = guided mode</param>
        public void setWP(Locationwp loc, ushort index, MAV_FRAME frame, byte current)
        {
            MainV2.givecomport = true;
#if MAVLINK10
            __mavlink_mission_item_t req = new __mavlink_mission_item_t();
#else
            __mavlink_waypoint_t req = new __mavlink_waypoint_t();
#endif

            req.target_system = sysid;
            req.target_component = compid; // MAVLINK_MSG_ID_MISSION_ITEM

            req.command = loc.id;
            req.param1 = loc.p1;

            req.current = current;

            req.frame = (byte)frame;
            req.y = (float)(loc.lng);
            req.x = (float)(loc.lat);
            req.z = (float)(loc.alt);

            req.param1 = loc.p1;
            req.param2 = loc.p2;
            req.param3 = loc.p3;
            req.param4 = loc.p4;
            /*
            if (MainV2.cs.firmware == MainV2.Firmwares.ArduPlane)
            {
                switch (loc.id)
                {					// Switch to map APM command fields inot MAVLink command fields
                    case (byte)MAV_CMD.LOITER_TURNS:
                    case (byte)MAV_CMD.TAKEOFF:
                        req.param1 = loc.p1;
                        break;
                    case (byte)MAV_CMD.DO_SET_HOME:
                        req.param1 = loc.p1;
                        break;

                    case (byte)MAV_CMD.CONDITION_CHANGE_ALT:
                        req.param1 = loc.lat;
                        req.x = 0;
                        req.y = 0;
                        break;

                    case (byte)MAV_CMD.LOITER_TIME:
                        req.param1 = loc.p1 * 10;	// APM loiter time is in ten second increments
                        break;

                    case (byte)MAV_CMD.CONDITION_DELAY:
                    case (byte)MAV_CMD.CONDITION_DISTANCE:
                        req.param1 = loc.lat;
                        break;

                    case (byte)MAV_CMD.DO_JUMP:
                        req.param2 = loc.lat;
                        req.param1 = loc.p1;
                        break;

                    case (byte)MAV_CMD.DO_REPEAT_SERVO:
                        req.param4 = loc.lng;
                        goto case (byte)MAV_CMD.DO_CHANGE_SPEED;
                    case (byte)MAV_CMD.DO_REPEAT_RELAY:
                    case (byte)MAV_CMD.DO_CHANGE_SPEED:
                        req.param3 = loc.lat;
                        req.param2 = loc.alt;
                        req.param1 = loc.p1;
                        break;

                    case (byte)MAV_CMD.DO_SET_PARAMETER:
                    case (byte)MAV_CMD.DO_SET_RELAY:
                    case (byte)MAV_CMD.DO_SET_SERVO:
                        req.param2 = loc.alt;
                        req.param1 = loc.p1;
                        break;
                }
            }
            */
            req.seq = index;

            Console.WriteLine("setWP {6} frame {0} cmd {1} p1 {2} x {3} y {4} z {5}", req.frame, req.command, req.param1, req.x, req.y, req.z, index);

            // request
#if MAVLINK10
            generatePacket(MAVLINK_MSG_ID_MISSION_ITEM, req);
#else
            generatePacket(MAVLINK_MSG_ID_WAYPOINT, req);
#endif

            DateTime start = DateTime.Now;
            int retrys = 6;

            while (true)
            {
                if (!(start.AddMilliseconds(500) > DateTime.Now))
                {
                    if (retrys > 0)
                    {
                        Console.WriteLine("setWP Retry " + retrys);
#if MAVLINK10
            generatePacket(MAVLINK_MSG_ID_MISSION_ITEM, req);
#else
                        generatePacket(MAVLINK_MSG_ID_WAYPOINT, req);
#endif
                        start = DateTime.Now;
                        retrys--;
                        continue;
                    }
                    MainV2.givecomport = false;
                    throw new Exception("Timeout on read - setWP");
                }
                byte[] buffer = readPacket();
                if (buffer.Length > 5)
                {
#if MAVLINK10
                    if (buffer[5] == MAVLINK_MSG_ID_MISSION_ACK)
                    {
                        __mavlink_mission_ack_t ans = new __mavlink_mission_ack_t();

                        object temp = (object)ans;

                        ByteArrayToStructure(buffer, ref temp, 6);

                        ans = (__mavlink_mission_ack_t)(temp);

                        Console.WriteLine("set wp " + index + " ACK 47 : " + buffer[5] + " ans " + Enum.Parse(typeof(MAV_MISSION_RESULT), ans.type.ToString()));
                        break;
                    }
                    else if (buffer[5] == MAVLINK_MSG_ID_MISSION_REQUEST)
                    {
                        __mavlink_mission_request_t ans = new __mavlink_mission_request_t();

                        object temp = (object)ans;

                        ByteArrayToStructure(buffer, ref temp, 6);

                        ans = (__mavlink_mission_request_t)(temp);

                        if (ans.seq == (index + 1))
                        {
                            Console.WriteLine("set wp doing " + index + " req " + ans.seq + " REQ 40 : " + buffer[5]);
                            MainV2.givecomport = false;
                            break;
                        }
                        else
                        {
                            Console.WriteLine("set wp fail doing " + index + " req " + ans.seq + " ACK 47 or REQ 40 : " + buffer[5] + " seq {0} ts {1} tc {2}", req.seq, req.target_system, req.target_component);
                            //break;
                        }
                    }
                    else
                    {
                        //Console.WriteLine(DateTime.Now + " PC setwp " + buffer[5]);
                    }
#else
                    if (buffer[5] == MAVLINK_MSG_ID_WAYPOINT_ACK)
                    { //__mavlink_waypoint_request_t
                        Console.WriteLine("set wp " + index + " ACK 47 : " + buffer[5]);
                        break;
                    }
                    else if (buffer[5] == MAVLINK_MSG_ID_WAYPOINT_REQUEST)
                    {
                        __mavlink_waypoint_request_t ans = new __mavlink_waypoint_request_t();

                        object temp = (object)ans;

                        //Array.Copy(buffer, 6, buffer, 0, buffer.Length - 6);

                        ByteArrayToStructure(buffer, ref temp, 6);

                        ans = (__mavlink_waypoint_request_t)(temp);

                        if (ans.seq == (index + 1))
                        {
                            Console.WriteLine("set wp doing " + index + " req " + ans.seq + " REQ 40 : " + buffer[5]);
                            MainV2.givecomport = false;
                            break;
                        }
                        else
                        {
                            Console.WriteLine("set wp fail doing " + index + " req " + ans.seq + " ACK 47 or REQ 40 : " + buffer[5] + " seq {0} ts {1} tc {2}", req.seq, req.target_system, req.target_component);
                            //break;
                        }
                    }
                    else
                    {
                        //Console.WriteLine(DateTime.Now + " PC setwp " + buffer[5]);
                    }
#endif
                }
            }
        }

        public void setMountConfigure(MAV_MOUNT_MODE mountmode, bool stabroll, bool stabpitch, bool stabyaw)
        {
            __mavlink_mount_configure_t req = new __mavlink_mount_configure_t();

            req.target_system = sysid;
            req.target_component = compid;
            req.mount_mode = (byte)mountmode;
            req.stab_pitch = (stabpitch == true) ? (byte)1 : (byte)0;
            req.stab_roll = (stabroll == true) ? (byte)1 : (byte)0;
            req.stab_yaw = (stabyaw == true) ? (byte)1 : (byte)0;

            generatePacket(MAVLINK_MSG_ID_MOUNT_CONFIGURE, req);
            System.Threading.Thread.Sleep(20);
            generatePacket(MAVLINK_MSG_ID_MOUNT_CONFIGURE, req);
        }

        public void setMountControl(double pa, double pb, double pc, bool islatlng)
        {
            __mavlink_mount_control_t req = new __mavlink_mount_control_t();

            req.target_system = sysid;
            req.target_component = compid;
            if (!islatlng)
            {
                req.input_a = (int)pa;
                req.input_b = (int)pb;
                req.input_c = (int)pc;
            }
            else
            {
                req.input_a = (int)(pa * 10000000.0);
                req.input_b = (int)(pb * 10000000.0);
                req.input_c = (int)(pc * 100.0);
            }

            generatePacket(MAVLINK_MSG_ID_MOUNT_CONTROL, req);
            System.Threading.Thread.Sleep(20);
            generatePacket(MAVLINK_MSG_ID_MOUNT_CONTROL, req);
        }

        public void setMode(string modein)
        {
#if MAVLINK10
            try
            {
                MAVLink.__mavlink_set_mode_t mode = new MAVLink.__mavlink_set_mode_t();

                if (Common.translateMode(modein, ref mode))
                {
                    MainV2.comPort.generatePacket((byte)MAVLink.MAVLINK_MSG_ID_SET_MODE, mode);
                    System.Threading.Thread.Sleep(10);
                    MainV2.comPort.generatePacket((byte)MAVLink.MAVLINK_MSG_ID_SET_MODE, mode);
                }
            }
            catch { System.Windows.Forms.MessageBox.Show("Failed to change Modes"); }
#else
            try
            {
                MAVLink.__mavlink_set_nav_mode_t navmode = new MAVLink.__mavlink_set_nav_mode_t();

                MAVLink.__mavlink_set_mode_t mode = new MAVLink.__mavlink_set_mode_t();

                if (Common.translateMode(modein, ref navmode, ref mode))
                {
                    MainV2.comPort.generatePacket((byte)MAVLink.MAVLINK_MSG_ID_SET_NAV_MODE, navmode);
                    System.Threading.Thread.Sleep(10);
                    MainV2.comPort.generatePacket((byte)MAVLink.MAVLINK_MSG_ID_SET_NAV_MODE, navmode);
                    System.Threading.Thread.Sleep(10);
                    MainV2.comPort.generatePacket((byte)MAVLink.MAVLINK_MSG_ID_SET_MODE, mode);
                    System.Threading.Thread.Sleep(10);
                    MainV2.comPort.generatePacket((byte)MAVLink.MAVLINK_MSG_ID_SET_MODE, mode);
                }
            }
            catch { System.Windows.Forms.MessageBox.Show("Failed to change Modes"); }

#endif
        }

        /// <summary>
        /// used for last bad serial characters
        /// </summary>
        byte[] lastbad = new byte[2];

        /// <summary>
        /// Serial Reader to read mavlink packets. POLL method
        /// </summary>
        /// <returns></returns>
        public byte[] readPacket()
        {
            byte[] temp = new byte[300];
            int count = 0;
            int length = 0;
            int readcount = 0;
            lastbad = new byte[2];

            BaseStream.ReadTimeout = 1100; // 1100 ms between bytes

            DateTime start = DateTime.Now;

            lock (readlock)
            {

                while (BaseStream.IsOpen || logreadmode)
                {
                    try
                    {
                        if (readcount > 300)
                        {
                            Console.WriteLine("MAVLink readpacket No valid mavlink packets");
                            break;
                        }
                        readcount++;
                        if (logreadmode)
                        {
                            try
                            {
                                if (logplaybackfile.BaseStream.Position == 0)
                                {
                                    if (logplaybackfile.PeekChar() == '-')
                                    {
                                        oldlogformat = true;
                                    }
                                    else
                                    {
                                        oldlogformat = false;
                                    }
                                }
                            }
                            catch { oldlogformat = false; }

                            if (oldlogformat)
                            {
                                temp = readlogPacket(); //old style log
                            }
                            else
                            {
                                temp = readlogPacketMavlink();
                            }
                        }
                        else
                        {
                            MainV2.cs.datetime = DateTime.Now;
                            temp[count] = (byte)BaseStream.ReadByte();
                        }
                    }
                    catch (Exception e) { Console.WriteLine("MAVLink readpacket read error: " + e.Message); break; }

                    if (temp[0] != 254 && temp[0] != 'U' || lastbad[0] == 'I' && lastbad[1] == 'M') // out of sync
                    {
                        if (temp[0] >= 0x20 && temp[0] <= 127 || temp[0] == '\n')
                        {
                            Console.Write((char)temp[0]);
                        }
                        count = 0;
                        lastbad[0] = lastbad[1];
                        lastbad[1] = temp[0];
                        temp[1] = 0;
                        continue;
                    }
                    // reset count on valid packet
                    readcount = 0;

                    if (temp[0] == 'U' || temp[0] == 254)
                    {
                        length = temp[1] + 6 + 2 - 2; // data + header + checksum - U - length
                        if (count >= 5 || logreadmode)
                        {
                            if (sysid != 0)
                            {
                                if (sysid != temp[3] || compid != temp[4])
                                {
                                    Console.WriteLine("Mavlink Bad Packet (not addressed to this MAV) got {0} {1} vs {2} {3}", temp[3], temp[4], sysid, compid);
                                    return new byte[0];
                                }
                            }

                            try
                            {
                                if (logreadmode)
                                {

                                }
                                else
                                {
                                    int to = 0;
                                    while (BaseStream.BytesToRead < (length - 4))
                                    {
                                        if (to > 1000)
                                        {
                                            Console.WriteLine("MAVLINK: wait time out btr {0} len {1}", BaseStream.BytesToRead, length);
                                            break;
                                        }
                                        System.Threading.Thread.Sleep(1);
                                        System.Windows.Forms.Application.DoEvents(); // when connecting this is in the main thread
                                        to++;

                                        //Console.WriteLine("data " + 0 + " " + length + " aval " + BaseStream.BytesToRead);
                                    }
                                    int read = BaseStream.Read(temp, 6, length - 4);
                                }
                                //Console.WriteLine("data " + read + " " + length + " aval " + this.BytesToRead);
                                count = length + 2;
                            }
                            catch { break; }
                            break;
                        }
                    }

                    count++;
                    if (count == 299)
                        break;
                }
            }// end readlock

            Array.Resize<byte>(ref temp, count);

            if (packetlosttimer.AddSeconds(10) < DateTime.Now)
            {
                packetlosttimer = DateTime.Now;
                packetslost = (int)(packetslost * 0.8f);
                packetsnotlost = (int)(packetsnotlost * 0.8f);
            }

            MainV2.cs.linkqualitygcs = (ushort)((packetsnotlost / (packetsnotlost + packetslost)) * 100);

            if (bpstime.Second != DateTime.Now.Second && !logreadmode)
            {
                //                Console.Write("bps {0} loss {1} left {2} mem {3}      \n", bps1, synclost, BaseStream.BytesToRead, System.GC.GetTotalMemory(false));
                bps2 = bps1; // prev sec
                bps1 = 0; // current sec
                bpstime = DateTime.Now;
            }

            bps1 += temp.Length;

            bps = (bps1 + bps2) / 2;

            if (temp.Length >= 5 && temp[3] == 255 && logreadmode) // gcs packet
            {
                getWPsfromstream(ref temp);
                return temp;// new byte[0];
            }

            ushort crc = crc_calculate(temp, temp.Length - 2);

            if (temp.Length > 5 && temp[0] == 254)
            {
                crc = crc_accumulate(MAVLINK_MESSAGE_CRCS[temp[5]], crc);
            }

            if (temp.Length > 5 && temp[1] != MAVLINK_MESSAGE_LENGTHS[temp[5]])
            {
                Console.WriteLine("Mavlink Bad Packet (Len Fail) len {0} pkno {1}", temp.Length, temp[5]);
                return new byte[0];
            }

            if (temp.Length < 5 || temp[temp.Length - 1] != (crc >> 8) || temp[temp.Length - 2] != (crc & 0xff))
            {
                int packetno = 0;
                if (temp.Length > 5)
                {
                    packetno = temp[5];
                }
                Console.WriteLine("Mavlink Bad Packet (crc fail) len {0} crc {1} pkno {2}", temp.Length, crc, packetno);
                return new byte[0];
            }

            try
            {

                if ((temp[0] == 'U' || temp[0] == 254) && temp.Length >= temp[1])
                {
                    if (temp[2] != ((recvpacketcount + 1) % 0x100))
                    {
                        synclost++; // actualy sync loss's

                        if (temp[2] < ((recvpacketcount + 1) % 0x100))
                        {
                            packetslost += 0x100 - recvpacketcount + temp[2];
                        }
                        else
                        {
                            packetslost += temp[2] - recvpacketcount;
                        }

                        Console.WriteLine("lost {0} pkts {1}", temp[2], (int)packetslost);
                    }

                    packetsnotlost++;

                    recvpacketcount = temp[2];

                    //MAVLINK_MSG_ID_GPS_STATUS
                    //if (temp[5] == MAVLINK_MSG_ID_GPS_STATUS)

                    //                    Console.Write(temp[5] + " " + DateTime.Now.Millisecond + " " + packetspersecond[temp[5]] + " " + (DateTime.Now - packetspersecondbuild[temp[5]]).TotalMilliseconds + "     \n");

                    if (double.IsInfinity(packetspersecond[temp[5]]))
                        packetspersecond[temp[5]] = 0;

                    packetspersecond[temp[5]] = (((1000 / ((DateTime.Now - packetspersecondbuild[temp[5]]).TotalMilliseconds) + packetspersecond[temp[5]]) / 2));

                    packetspersecondbuild[temp[5]] = DateTime.Now;

                    //Console.WriteLine("Packet {0}",temp[5]);
                    // store packet history
                    lock (objlock)
                    {
                        packets[temp[5]] = temp;
                    }

                    if (debugmavlink)
                        DebugPacket(temp);

                    if (temp[5] == MAVLink.MAVLINK_MSG_ID_STATUSTEXT) // status text
                    {
                        string logdata = Encoding.ASCII.GetString(temp, 7, temp.Length - 7);
                        int ind = logdata.IndexOf('\0');
                        if (ind != -1)
                            logdata = logdata.Substring(0, ind);
                        Console.WriteLine(DateTime.Now + " " + logdata);

                        if (MainV2.talk != null && MainV2.config["speechenable"] != null && MainV2.config["speechenable"].ToString() == "True")
                        {
                            //MainV2.talk.SpeakAsync(logdata);
                        }

                    }

                    getWPsfromstream(ref temp);

                    try
                    {
                        if (logfile != null)
                        {
                            lock (logwritelock)
                            {
                                byte[] datearray = BitConverter.GetBytes((UInt64)((DateTime.UtcNow - new DateTime(1970, 1, 1)).TotalMilliseconds * 1000)); //ASCIIEncoding.ASCII.GetBytes(DateTime.Now.ToBinary() + ":");
                                Array.Reverse(datearray);
                                logfile.Write(datearray, 0, datearray.Length);
                                logfile.Write(temp, 0, temp.Length);
                            }
                        }

                    }
                    catch { }
                }
            }
            catch { }

            lastvalidpacket = DateTime.Now;

            //            Console.Write((DateTime.Now - start).TotalMilliseconds.ToString("00.000") + "\t" + temp.Length + "     \r");

            return temp;
        }

        /// <summary>
        /// Used to extract mission from log file
        /// </summary>
        /// <param name="temp">packet</param>
        void getWPsfromstream(ref byte[] temp)
        {
#if MAVLINK10
                    if (temp[5] == MAVLINK_MSG_ID_MISSION_COUNT)
                    {
                        // clear old
                        wps = new PointLatLngAlt[wps.Length];
                    }

                    if (temp[5] == MAVLink.MAVLINK_MSG_ID_MISSION_ITEM)
                    {
                        __mavlink_mission_item_t wp = new __mavlink_mission_item_t();

                        object structtemp = (object)wp;

                        //Array.Copy(buffer, 6, buffer, 0, buffer.Length - 6);

                        ByteArrayToStructure(temp, ref structtemp, 6);

                        wp = (__mavlink_mission_item_t)(structtemp);
#else

            if (temp[5] == MAVLINK_MSG_ID_WAYPOINT_COUNT)
            {
                // clear old
                wps = new PointLatLngAlt[wps.Length];
            }

            if (temp[5] == MAVLink.MAVLINK_MSG_ID_WAYPOINT)
            {
                __mavlink_waypoint_t wp = new __mavlink_waypoint_t();

                object structtemp = (object)wp;

                //Array.Copy(buffer, 6, buffer, 0, buffer.Length - 6);

                ByteArrayToStructure(temp, ref structtemp, 6);

                wp = (__mavlink_waypoint_t)(structtemp);

#endif
                wps[wp.seq] = new PointLatLngAlt(wp.x, wp.y, wp.z, wp.seq.ToString());
            }
        }

        public PointLatLngAlt getFencePoint(int no, ref int total)
        {
            byte[] buffer;

            MainV2.givecomport = true;

            PointLatLngAlt plla = new PointLatLngAlt();
            __mavlink_fence_fetch_point_t req = new __mavlink_fence_fetch_point_t();

            req.idx = (byte)no;
            req.target_component = compid;
            req.target_system = sysid;

            // request point
            generatePacket(MAVLINK_MSG_ID_FENCE_FETCH_POINT, req);

            DateTime start = DateTime.Now;
            int retrys = 3;

            while (true)
            {
                if (!(start.AddMilliseconds(500) > DateTime.Now))
                {
                    if (retrys > 0)
                    {
                        Console.WriteLine("getFencePoint Retry " + retrys + " - giv com " + MainV2.givecomport);
                        generatePacket(MAVLINK_MSG_ID_FENCE_FETCH_POINT, req);
                        start = DateTime.Now;
                        retrys--;
                        continue;
                    }
                    MainV2.givecomport = false;
                    throw new Exception("Timeout on read - getFencePoint");
                }

                buffer = readPacket();
                if (buffer.Length > 5)
                {
                    if (buffer[5] == MAVLINK_MSG_ID_FENCE_POINT)
                    {
                        MainV2.givecomport = false;

                        __mavlink_fence_point_t fp = new __mavlink_fence_point_t();

                        object structtemp = (object)fp;

                        ByteArrayToStructure(buffer, ref structtemp, 6);

                        fp = (__mavlink_fence_point_t)(structtemp);

                        plla.Lat = fp.lat;
                        plla.Lng = fp.lng;
                        plla.Tag = fp.idx.ToString();

                        total = fp.count;

                        return plla;
                    }
                }
            }
        }

        public bool setFencePoint(byte index, PointLatLngAlt plla, byte fencepointcount)
        {
            __mavlink_fence_point_t fp = new __mavlink_fence_point_t();

            fp.idx = index;
            fp.count = fencepointcount;
            fp.lat = (float)plla.Lat;
            fp.lng = (float)plla.Lng;
            fp.target_component = compid;
            fp.target_system = sysid;

            int retry = 3;

            while (retry > 0)
            {
                generatePacket(MAVLINK_MSG_ID_FENCE_POINT, fp);
                int counttemp = 0;
                PointLatLngAlt newfp = getFencePoint(fp.idx, ref counttemp);

                if (newfp.Lat == plla.Lat && newfp.Lng == fp.lng)
                    return true;
                retry--;
            }

            return false;
        }

        byte[] readlogPacket()
        {
            byte[] temp = new byte[300];

            sysid = 0;

            int a = 0;
            while (a < temp.Length && logplaybackfile.BaseStream.Position != logplaybackfile.BaseStream.Length)
            {
                temp[a] = (byte)logplaybackfile.BaseStream.ReadByte();
                //Console.Write((char)temp[a]);
                if (temp[a] == ':')
                {
                    break;
                }
                a++;
                if (temp[0] != '-')
                {
                    a = 0;
                }
            }

            //Console.Write('\n');

            //Encoding.ASCII.GetString(temp, 0, a);
            string datestring = Encoding.ASCII.GetString(temp, 0, a);
            //Console.WriteLine(datestring);
            long date = Int64.Parse(datestring);
            DateTime date1 = DateTime.FromBinary(date);

            lastlogread = date1;

            int length = 5;
            a = 0;
            while (a < length)
            {
                temp[a] = (byte)logplaybackfile.BaseStream.ReadByte();
                if (a == 1)
                {
                    length = temp[1] + 6 + 2 + 1;
                }
                a++;
            }

            return temp;
        }

        byte[] readlogPacketMavlink()
        {
            byte[] temp = new byte[300];

            sysid = 0;

            //byte[] datearray = BitConverter.GetBytes((ulong)(DateTime.UtcNow - new DateTime(1970, 1, 1)).TotalMilliseconds);

            byte[] datearray = new byte[8];

            logplaybackfile.BaseStream.Read(datearray, 0, datearray.Length);

            Array.Reverse(datearray);

            DateTime date1 = new DateTime(1970, 1, 1, 0, 0, 0, DateTimeKind.Utc);

            UInt64 dateint = BitConverter.ToUInt64(datearray, 0);

            date1 = date1.AddMilliseconds(dateint / 1000);

            lastlogread = date1.ToLocalTime();

            MainV2.cs.datetime = lastlogread;

            int length = 5;
            int a = 0;
            while (a < length)
            {
                temp[a] = (byte)logplaybackfile.ReadByte();
                if (temp[0] != 'U' && temp[0] != 254)
                {
                    Console.WriteLine("lost sync byte {0} pos {1}", temp[0], logplaybackfile.BaseStream.Position);
                    a = 0;
                    continue;
                }
                if (a == 1)
                {
                    length = temp[1] + 6 + 2; // 6 header + 2 checksum
                }
                a++;
            }

            return temp;
        }

        const int X25_INIT_CRC = 0xffff;
        const int X25_VALIDATE_CRC = 0xf0b8;

        ushort crc_accumulate(byte b, ushort crc)
        {
            unchecked
            {
                byte ch = (byte)(b ^ (byte)(crc & 0x00ff));
                ch = (byte)(ch ^ (ch << 4));
                return (ushort)((crc >> 8) ^ (ch << 8) ^ (ch << 3) ^ (ch >> 4));
            }
        }

        ushort crc_calculate(byte[] pBuffer, int length)
        {
            if (length < 1)
            {
                return 0xffff;
            }
            // For a "message" of length bytes contained in the unsigned char array
            // pointed to by pBuffer, calculate the CRC
            // crcCalculate(unsigned char* pBuffer, int length, unsigned short* checkConst) < not needed

            ushort crcTmp;
            int i;

            crcTmp = X25_INIT_CRC;

            for (i = 1; i < length; i++) // skips header U
            {
                crcTmp = crc_accumulate(pBuffer[i], crcTmp);
                //Console.WriteLine(crcTmp + " " + pBuffer[i] + " " + length);
            }

            return (crcTmp);
        }


        byte[] StructureToByteArray(object obj)
        {

            int len = Marshal.SizeOf(obj);

            byte[] arr = new byte[len];

            IntPtr ptr = Marshal.AllocHGlobal(len);

            Marshal.StructureToPtr(obj, ptr, true);

            Marshal.Copy(ptr, arr, 0, len);

            Marshal.FreeHGlobal(ptr);

            return arr;

        }

        public static void ByteArrayToStructure(byte[] bytearray, ref object obj, int startoffset)
        {
            if (bytearray[0] == 'U')
            {
                ByteArrayToStructureEndian(bytearray, ref obj, startoffset);
            }
            else
            {
                int len = Marshal.SizeOf(obj);

                IntPtr i = Marshal.AllocHGlobal(len);

                // create structure from ptr
                obj = Marshal.PtrToStructure(i, obj.GetType());

                try
                {
                    // copy byte array to ptr
                    Marshal.Copy(bytearray, startoffset, i, len);
                }
                catch (Exception ex) { Console.WriteLine("ByteArrayToStructure FAIL: error " + ex.ToString()); }

                obj = Marshal.PtrToStructure(i, obj.GetType());

                Marshal.FreeHGlobal(i);
            }
        }

        public static void ByteArrayToStructureEndian(byte[] bytearray, ref object obj, int startoffset)
        {
            int len = Marshal.SizeOf(obj);

            IntPtr i = Marshal.AllocHGlobal(len);

            byte[] temparray = (byte[])bytearray.Clone();

            // create structure from ptr
            obj = Marshal.PtrToStructure(i, obj.GetType());

            // do endian swap

            object thisBoxed = obj;
            Type test = thisBoxed.GetType();

            int reversestartoffset = startoffset;

            // Enumerate each structure field using reflection.
            foreach (var field in test.GetFields())
            {
                // field.Name has the field's name.

                object fieldValue = field.GetValue(thisBoxed); // Get value

                // Get the TypeCode enumeration. Multiple types get mapped to a common typecode.
                TypeCode typeCode = Type.GetTypeCode(fieldValue.GetType());

                if (typeCode != TypeCode.Object)
                {
                    Array.Reverse(temparray, reversestartoffset, Marshal.SizeOf(fieldValue));
                    reversestartoffset += Marshal.SizeOf(fieldValue);
                }
                else
                {
                    reversestartoffset += ((byte[])fieldValue).Length;
                }

            }

            try
            {
                // copy byte array to ptr
                Marshal.Copy(temparray, startoffset, i, len);
            }
            catch (Exception ex) { Console.WriteLine("ByteArrayToStructure FAIL: error " + ex.ToString()); }

            obj = Marshal.PtrToStructure(i, obj.GetType());

            Marshal.FreeHGlobal(i);

        }

        public short swapend11(short value)
        {
            int len = Marshal.SizeOf(value);

            byte[] temp = BitConverter.GetBytes(value);

            Array.Reverse(temp);

            return BitConverter.ToInt16(temp, 0);
        }

        public ushort swapend11(ushort value)
        {
            int len = Marshal.SizeOf(value);

            byte[] temp = BitConverter.GetBytes(value);

            Array.Reverse(temp);

            return BitConverter.ToUInt16(temp, 0);
        }

        public ulong swapend11(ulong value)
        {
            int len = Marshal.SizeOf(value);

            byte[] temp = BitConverter.GetBytes(value);

            Array.Reverse(temp);

            return BitConverter.ToUInt64(temp, 0);
        }

        public float swapend11(float value)
        {
            byte[] temp = BitConverter.GetBytes(value);
            if (temp[0] == 0xff)
                temp[0] = 0xfe;
            Array.Reverse(temp);
            return BitConverter.ToSingle(temp, 0);
        }

        public int swapend11(int value)
        {
            int len = Marshal.SizeOf(value);

            byte[] temp = BitConverter.GetBytes(value);

            Array.Reverse(temp);

            return BitConverter.ToInt32(temp, 0);
        }

        public double swapend11(double value)
        {
            int len = Marshal.SizeOf(value);

            byte[] temp = BitConverter.GetBytes(value);

            Array.Reverse(temp);

            return BitConverter.ToDouble(temp, 0);
        }
    }
}