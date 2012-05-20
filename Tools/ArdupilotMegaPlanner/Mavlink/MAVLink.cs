using System;
using System.Collections.Generic;
using System.Reactive.Subjects;
using System.Text;
using System.Runtime.InteropServices;
using System.Collections; // hashs
using System.Diagnostics; // stopwatch
using System.Reflection;
using System.Reflection.Emit;
using System.IO;
using System.Drawing;
using System.Threading;
using ArdupilotMega.Controls;
using System.ComponentModel;
using log4net;
using ArdupilotMega.Comms;
using ArdupilotMega.Utilities;
using System.Windows.Forms;

namespace ArdupilotMega
{
    public partial class MAVLink: IMAVLink
    {
        private static readonly ILog log = LogManager.GetLogger(MethodBase.GetCurrentMethod().DeclaringType);
        public ICommsSerial BaseStream { get; set; }
        public event EventHandler ParamListChanged;

        private const double CONNECT_TIMEOUT_SECONDS = 30;

        /// <summary>
        /// progress form to handle connect and param requests
        /// </summary>
        ProgressReporterDialogue frmProgressReporter;

        /// <summary>
        /// used for outbound packet sending
        /// </summary>
        byte packetcount = 0;
        /// <summary>
        /// mavlink remote sysid
        /// </summary>
        public byte sysid { get; set; }
        /// <summary>
        /// mavlink remove compid
        /// </summary>
        public byte compid { get; set; }
        /// <summary>
        /// storage for whole paramater list
        /// </summary>
        public Hashtable param  { get; set; }
        /// <summary>
        /// storage of a previous packet recevied of a specific type
        /// </summary>
        public byte[][] packets  { get; set; }
        /// <summary>
        /// used to calc packets per second on any single message type - used for stream rate comparaison
        /// </summary>
        public double[] packetspersecond   { get; set; }
        /// <summary>
        /// time last seen a packet of a type
        /// </summary>
        DateTime[] packetspersecondbuild = new DateTime[256];


        private readonly Subject<int> _bytesReceivedSubj = new Subject<int>();
        private readonly Subject<int> _bytesSentSubj = new Subject<int>();

        /// <summary>
        /// Observable of the count of bytes received, notified when the bytes themselves are received
        /// </summary>
        public IObservable<int> BytesReceived { get { return _bytesReceivedSubj; } }

        /// <summary>
        /// Observable of the count of bytes sent, notified when the bytes themselves are received
        /// </summary>
        public IObservable<int> BytesSent { get { return _bytesSentSubj; } }

        /// <summary>
        /// Observable of the count of packets skipped (on reception), 
        /// calculated from periods where received packet sequence is not
        /// contiguous
        /// </summary>
        public Subject<int> WhenPacketLost { get; set; }

        public Subject<int> WhenPacketReceived { get; set; }

        /// <summary>
        /// used as a serial port write lock
        /// </summary>
        object objlock = new object();
        /// <summary>
        /// used for a readlock on readpacket
        /// </summary>
        object readlock = new object();
        /// <summary>
        /// used for tlog file lock
        /// </summary>
        object logwritelock = new object();
        /// <summary>
        /// time seen of last mavlink packet
        /// </summary>
        public DateTime lastvalidpacket  { get; set; }
        /// <summary>
        /// old log support
        /// </summary>
        bool oldlogformat = false;

        /// <summary>
        /// mavlink version
        /// </summary>
        byte mavlinkversion = 0;
        /// <summary>
        /// mavlink ap type
        /// </summary>
        public MAV_TYPE aptype { get; set; }
        /// <summary>
        /// used as a snapshot of what is loaded on the ap atm. - derived from the stream
        /// </summary>
        public PointLatLngAlt[] wps  { get; set; }
        /// <summary>
        /// turns on console packet display
        /// </summary>
        public bool debugmavlink { get; set; }
        /// <summary>
        /// enabled read from file mode
        /// </summary>
        public bool logreadmode { get; set; }
        public DateTime lastlogread { get; set; }
        public BinaryReader logplaybackfile { get; set; }
        public BinaryWriter logfile { get; set; }
        public BinaryWriter rawlogfile { get; set; }

        int bps1 = 0;
        int bps2 = 0;
        public int bps { get; set; }
        public DateTime bpstime { get; set; }
        int recvpacketcount = 0;

        float synclost;
        float packetslost = 0;
        float packetsnotlost = 0;
        DateTime packetlosttimer = DateTime.Now;

        public MAVLink()
        {
            // init fields
            this.BaseStream = new SerialPort();
            this.packetcount = 0;
            this.sysid = 0;
            this.compid = 0;
            this.param = new Hashtable();
            this.packets = new byte[0x100][];
            this.packetspersecond = new double[0x100];
            this.packetspersecondbuild = new DateTime[0x100];
            this._bytesReceivedSubj = new Subject<int>();
            this._bytesSentSubj = new Subject<int>();
            this.WhenPacketLost = new Subject<int>();
            this.WhenPacketReceived = new Subject<int>();
            this.objlock = new object();
            this.readlock = new object();
            this.logwritelock = new object();
            this.lastvalidpacket = DateTime.Now;
            this.oldlogformat = false;
            this.mavlinkversion = 0;
            this.aptype = 0;
            this.wps = new PointLatLngAlt[200];
            this.debugmavlink = false;
            this.logreadmode = false;
            this.lastlogread = DateTime.MinValue;
            this.logplaybackfile = null;
            this.logfile = null;
            this.rawlogfile = null;
            this.bps1 = 0;
            this.bps2 = 0;
            this.bps = 0;
            this.bpstime = DateTime.Now;
            this.recvpacketcount = 0;
            this.packetslost = 0f;
            this.packetsnotlost = 0f;
            this.packetlosttimer = DateTime.Now;
            this.lastbad = new byte[2];

        }

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

            frmProgressReporter = new ProgressReporterDialogue
                                      {
                                          StartPosition = System.Windows.Forms.FormStartPosition.CenterScreen,
                                          Text = "Connecting Mavlink"
                                      };

            if (getparams)
            {
                frmProgressReporter.DoWork += FrmProgressReporterDoWorkAndParams;
            }
            else
            {
                frmProgressReporter.DoWork += FrmProgressReporterDoWorkNOParams;
            }
            frmProgressReporter.UpdateProgressAndStatus(-1, "Mavlink Connecting...");
            ThemeManager.ApplyThemeTo(frmProgressReporter);

            frmProgressReporter.RunBackgroundOperationAsync();
        }

        void FrmProgressReporterDoWorkAndParams(object sender, ProgressWorkerEventArgs e)
        {
            OpenBg(true, e);
        }

        void FrmProgressReporterDoWorkNOParams(object sender, ProgressWorkerEventArgs e)
        {
            OpenBg(false, e);
        }

        private void OpenBg(bool getparams, ProgressWorkerEventArgs progressWorkerEventArgs)
        {
            frmProgressReporter.UpdateProgressAndStatus(-1, "Mavlink Connecting...");

            // reset
            sysid = 0;
            compid = 0;
            param = new Hashtable();
            packets.Initialize();

            try
            {
                MainV2.giveComport = true;

                BaseStream.ReadBufferSize = 4 * 1024;

                lock (objlock) // so we dont have random traffic
                {
                    log.Info("Open port with " + BaseStream.PortName + " " + BaseStream.BaudRate);

                    BaseStream.Open();

                    BaseStream.DiscardInBuffer();

                    Thread.Sleep(1000);
                }

                byte[] buffer = new byte[0];
                byte[] buffer1 = new byte[0];

                DateTime start = DateTime.Now;
                DateTime deadline = start.AddSeconds(CONNECT_TIMEOUT_SECONDS);

                var countDown = new System.Timers.Timer { Interval = 1000, AutoReset = false };
                countDown.Elapsed += (sender, e) =>
                {
                    int secondsRemaining = (deadline - e.SignalTime).Seconds;
                    //if (Progress != null)
                    //    Progress(-1, string.Format("Trying to connect.\nTimeout in {0}", secondsRemaining));
                    frmProgressReporter.UpdateProgressAndStatus(-1, string.Format("Trying to connect.\nTimeout in {0}", secondsRemaining));
                    if (secondsRemaining > 0) countDown.Start();
                };
                countDown.Start();

                int count = 0;

                while (true)
                {
                    if (progressWorkerEventArgs.CancelRequested)
                    {
                        progressWorkerEventArgs.CancelAcknowledged = true;
                        countDown.Stop();
                        if (BaseStream.IsOpen)
                            BaseStream.Close();
                        MainV2.giveComport = false;
                        return;
                    }

                    // incase we are in setup mode
                    BaseStream.WriteLine("planner\rgcs\r");

                    log.Info(DateTime.Now.Millisecond + " Start connect loop ");

                    if (lastbad[0] == '!' && lastbad[1] == 'G' || lastbad[0] == 'G' && lastbad[1] == '!') // waiting for gps lock
                    {
                        //if (Progress != null)
                        //    Progress(-1, "Waiting for GPS detection..");
                        frmProgressReporter.UpdateProgressAndStatus(-1, "Waiting for GPS detection..");
                        deadline = deadline.AddSeconds(5); // each round is 1.1 seconds
                    }

                    if (DateTime.Now > deadline)
                    {
                        //if (Progress != null)
                        //    Progress(-1, "No Heatbeat Packets");
                        this.Close();
                        progressWorkerEventArgs.ErrorMessage = "No Heatbeat Packets Received";
                        throw new Exception("No Mavlink Heartbeat Packets where read from this port - Verify Baud Rate and setup\nAPM Planner waits for 2 valid heartbeat packets before connecting");
                    }

                    System.Threading.Thread.Sleep(1);

                    // incase we are in setup mode
                    BaseStream.WriteLine("planner\rgcs\r");

                    // can see 2 heartbeat packets at any time, and will connect - was one after the other

                    if (buffer.Length == 0)
                        buffer = getHeartBeat();

                    // incase we are in setup mode
                    BaseStream.WriteLine("planner\rgcs\r");

                    System.Threading.Thread.Sleep(1);

                    if (buffer1.Length == 0)
                        buffer1 = getHeartBeat();

                    try
                    {
                        log.Debug("MAv Data: len " + buffer.Length + " btr " + BaseStream.BytesToRead);
                    }
                    catch { }

                    count++;

                    if (buffer.Length > 5 && buffer1.Length > 5 && buffer[3] == buffer1[3] && buffer[4] == buffer1[4])
                    {
                        mavlink_heartbeat_t hb = buffer.ByteArrayToStructure<mavlink_heartbeat_t>(6);

                        mavlinkversion = hb.mavlink_version;
                        aptype = (MAV_TYPE)hb.type;

                        sysid = buffer[3];
                        compid = buffer[4];
                        recvpacketcount = buffer[2];
                        log.InfoFormat("ID sys {0} comp {1} ver{2}", sysid, compid, mavlinkversion);
                        break;
                    }

                }

                countDown.Stop();

                //                if (Progress != null)
                //                    Progress(-1, "Getting Params.. (sysid " + sysid + " compid " + compid + ") ");
                frmProgressReporter.UpdateProgressAndStatus(0, "Getting Params.. (sysid " + sysid + " compid " + compid + ") ");

                if (getparams)
                    getParamListBG();

                if (frmProgressReporter.doWorkArgs.CancelAcknowledged == true)
                {
                    MainV2.giveComport = false;
                    if (BaseStream.IsOpen)
                        BaseStream.Close();
                    return;
                }
            }
            catch (Exception e)
            {
                try
                {
                    BaseStream.Close();
                }
                catch { }
                MainV2.giveComport = false;
                //                if (Progress != null)
                //                    Progress(-1, "Connect Failed\n" + e.Message);
                if (string.IsNullOrEmpty(progressWorkerEventArgs.ErrorMessage))
                    progressWorkerEventArgs.ErrorMessage = "Connect Failed";
                throw e;
            }
            //frmProgressReporter.Close();
            MainV2.giveComport = false;
            frmProgressReporter.UpdateProgressAndStatus(100, "Done.");
            log.Info("Done open " + sysid + " " + compid);
            packetslost = 0;
            synclost = 0;
        }

        byte[] getHeartBeat()
        {
            DateTime start = DateTime.Now;
            while (true)
            {
                byte[] buffer = readPacket();
                if (buffer.Length > 5)
                {
                    //log.Info("getHB packet received: " + buffer.Length + " btr " + BaseStream.BytesToRead + " type " + buffer[5] );
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
            bool validPacket = false;
            byte a = 0;
            foreach (Type ty in MAVLINK_MESSAGE_INFO)
            {
                if (ty == indata.GetType())
                {
                    validPacket = true;
                    generatePacket(a, indata);
                    return;
                }
                a++;
            }
            if (!validPacket)
            {
                log.Info("Mavlink : NOT VALID PACKET sendPacket() " + indata.GetType().ToString());
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
                data = MavlinkUtil.StructureToByteArray(indata);
            }
            else
            {
                data = MavlinkUtil.StructureToByteArrayBigEndian(indata);
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

            ushort checksum = MavlinkCRC.crc_calculate(packet, packet[1] + 6);

            if (mavlinkversion == 3)
            {
                checksum = MavlinkCRC.crc_accumulate(MAVLINK_MESSAGE_CRCS[messageType], checksum);
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
                _bytesSentSubj.OnNext(i);
            }

            try
            {
                if (logfile != null && logfile.BaseStream.CanWrite)
                {
                    lock (logwritelock)
                    {
                        byte[] datearray = BitConverter.GetBytes((UInt64)((DateTime.UtcNow - new DateTime(1970, 1, 1)).TotalMilliseconds * 1000)); //ASCIIEncoding.ASCII.GetBytes(DateTime.Now.ToBinary() + ":");
                        Array.Reverse(datearray);
                        logfile.Write(datearray, 0, datearray.Length);
                        logfile.Write(packet, 0, i);
                        logfile.Flush();
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
            _bytesSentSubj.OnNext(line.Length);
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
                log.Info("Param doesnt exist " + paramname);
                return false;
            }

            if ((float)param[paramname] == value)
            {
                log.Debug("setParam " + paramname + " not modified");
                return true;
            }

            MainV2.giveComport = true;

            mavlink_param_set_t req = new mavlink_param_set_t();
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

            log.InfoFormat("setParam '{0}' = '{1}' sysid {2} compid {3}", paramname, req.param_value, sysid, compid);

            DateTime start = DateTime.Now;
            int retrys = 3;

            while (true)
            {
                if (!(start.AddMilliseconds(500) > DateTime.Now))
                {
                    if (retrys > 0)
                    {
                        log.Info("setParam Retry " + retrys);
                        generatePacket(MAVLINK_MSG_ID_PARAM_SET, req);
                        start = DateTime.Now;
                        retrys--;
                        continue;
                    }
                    MainV2.giveComport = false;
                    throw new Exception("Timeout on read - setParam " + paramname);
                }

                byte[] buffer = readPacket();
                if (buffer.Length > 5)
                {
                    if (buffer[5] == MAVLINK_MSG_ID_PARAM_VALUE)
                    {
                        mavlink_param_value_t par = buffer.ByteArrayToStructure<mavlink_param_value_t>(6);

                        string st = System.Text.ASCIIEncoding.ASCII.GetString(par.param_id);

                        int pos = st.IndexOf('\0');

                        if (pos != -1)
                        {
                            st = st.Substring(0, pos);
                        }

                        if (st != paramname)
                        {
                            log.InfoFormat("MAVLINK bad param responce - {0} vs {1}", paramname, st);
                            continue;
                        }

                        modifyParamForDisplay(true, st, ref par.param_value);

                        param[st] = (par.param_value);

                        MainV2.giveComport = false;
                        //System.Threading.Thread.Sleep(100);//(int)(8.5 * 5)); // 8.5ms per byte
                        return true;
                    }
                }
            }
        }
        /*
        public Bitmap getImage()
        {
            MemoryStream ms = new MemoryStream();

        }
        */
        public void getParamList()
        {
            frmProgressReporter = new ProgressReporterDialogue
            {
                StartPosition = System.Windows.Forms.FormStartPosition.CenterScreen,
                Text = "Getting Params"
            };

            frmProgressReporter.DoWork += FrmProgressReporterGetParams;
            frmProgressReporter.UpdateProgressAndStatus(-1, "Getting Params...");
            ThemeManager.ApplyThemeTo(frmProgressReporter);

            frmProgressReporter.RunBackgroundOperationAsync();

            if (ParamListChanged != null)
            {
                ParamListChanged(this, null);
            }
        }

        void FrmProgressReporterGetParams(object sender, ProgressWorkerEventArgs e)
        {
            Hashtable old = new Hashtable(param);
            getParamListBG();
            if (frmProgressReporter.doWorkArgs.CancelRequested)
            {
                param = old;
            }
        }

        /// <summary>
        /// Get param list from apm
        /// </summary>
        /// <returns></returns>
        private Hashtable getParamListBG()
        {
            MainV2.giveComport = true;
            List<int> got = new List<int>();

            // clear old
            param = new Hashtable();

            int retrys = 6;
            int param_count = 0;
            int param_total = 5;

        goagain:

            mavlink_param_request_list_t req = new mavlink_param_request_list_t();
            req.target_system = sysid;
            req.target_component = compid;

            generatePacket(MAVLINK_MSG_ID_PARAM_REQUEST_LIST, req);

            DateTime start = DateTime.Now;
            DateTime restart = DateTime.Now;

            do
            {

                if (frmProgressReporter.doWorkArgs.CancelRequested)
                {
                    frmProgressReporter.doWorkArgs.CancelAcknowledged = true;
                    MainV2.giveComport = false;
                    frmProgressReporter.doWorkArgs.ErrorMessage = "User Canceled";
                    return param;
                }

                // 4 seconds between valid packets
                if (!(start.AddMilliseconds(4000) > DateTime.Now))
                {
                    if (retrys > 0)
                    {
                        log.InfoFormat("getParamList Retry {0} sys {1} comp {2}", retrys, sysid, compid);
                        generatePacket(MAVLINK_MSG_ID_PARAM_REQUEST_LIST, req);
                        start = DateTime.Now;
                        retrys--;
                        continue;
                    }
                    MainV2.giveComport = false;
                    throw new Exception("Timeout on read - getParamList " + got.Count + " " + param_total + "\n\nYour serial link isn't fast enough\n");
                }

                byte[] buffer = readPacket();
                if (buffer.Length > 5)
                {
                    //stopwatch.Reset();
                    //stopwatch.Start();
                    if (buffer[5] == MAVLINK_MSG_ID_PARAM_VALUE)
                    {
                        restart = DateTime.Now;
                        start = DateTime.Now;

                        mavlink_param_value_t par = buffer.ByteArrayToStructure<mavlink_param_value_t>(6);

                        // set new target
                        param_total = (par.param_count);


                        string paramID = System.Text.ASCIIEncoding.ASCII.GetString(par.param_id);

                        int pos = paramID.IndexOf('\0');
                        if (pos != -1)
                        {
                            paramID = paramID.Substring(0, pos);
                        }

                        // check if we already have it
                        if (got.Contains(par.param_index))
                        {
                            log.Info("Already got " + (par.param_index) + " '" + paramID + "'");
                            this.frmProgressReporter.UpdateProgressAndStatus((got.Count * 100) / param_total, "Already Got param " + paramID);
                            continue;
                        }

                        log.Info(DateTime.Now.Millisecond + " got param " + (par.param_index) + " of " + (par.param_count) + " name: " + paramID);

                        modifyParamForDisplay(true, paramID, ref par.param_value);
                        param[paramID] = (par.param_value);
                        param_count++;
                        got.Add(par.param_index);

                        //                        if (Progress != null)
                        //                            Progress((param.Count * 100) / param_total, "Got param " + paramID);
                        this.frmProgressReporter.UpdateProgressAndStatus((got.Count * 100) / param_total, "Got param " + paramID);

                        // we have them all - lets escape eq total = 176 index = 0-175
                        if (par.param_index == (param_total -1))
                            break;
                    }
                    else
                    {
                        //Console.WriteLine(DateTime.Now + " PC paramlist " + buffer[5] + " want " + MAVLINK_MSG_ID_PARAM_VALUE + " btr " + BaseStream.BytesToRead);
                    }
                    //stopwatch.Stop();
                    //Console.WriteLine("Time elapsed: {0}", stopwatch.Elapsed);
                }
            } while (got.Count < param_total);

            if (got.Count != param_total)
            {
                if (retrys > 0)
                {
                    this.frmProgressReporter.UpdateProgressAndStatus((got.Count * 100) / param_total, "Getting missed params");
                    retrys--;
                    goto goagain;
                }
                throw new Exception("Missing Params");
            }
            MainV2.giveComport = false;
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
            else if (paramname.ToUpper().StartsWith("TUNE_"))
            {
                if (fromapm)
                {
                    value /= 1000.0f;
                }
                else
                {
                    value *= 1000.0f;
                }
            }
        }

        /// <summary>
        /// Stops all requested data packets.
        /// </summary>
        public void stopall(bool forget)
        {
            mavlink_request_data_stream_t req = new mavlink_request_data_stream_t();
            req.target_system = sysid;
            req.target_component = compid;

            req.req_message_rate = 10;
            req.start_stop = 0; // stop
            req.req_stream_id = 0; // all

            // no error on bad
            try
            {
                generatePacket(MAVLINK_MSG_ID_REQUEST_DATA_STREAM, req);
                System.Threading.Thread.Sleep(20);
                generatePacket(MAVLINK_MSG_ID_REQUEST_DATA_STREAM, req);
                System.Threading.Thread.Sleep(20);
                generatePacket(MAVLINK_MSG_ID_REQUEST_DATA_STREAM, req);
                log.Info("Stopall Done");

            }
            catch { }
        }

        public void setWPACK()
        {
#if MAVLINK10
            MAVLink.mavlink_mission_ack_t req = new MAVLink.mavlink_mission_ack_t();
            req.target_system = sysid;
            req.target_component = compid;
            req.type = 0;

            generatePacket(MAVLINK_MSG_ID_MISSION_ACK, req);
#else
            MAVLink.mavlink_waypoint_ack_t req = new MAVLink.mavlink_waypoint_ack_t();
            req.target_system = sysid;
            req.target_component = compid;
            req.type = 0;

            generatePacket(MAVLINK_MSG_ID_WAYPOINT_ACK, req);
#endif
        }

        public bool setWPCurrent(ushort index)
        {
#if MAVLINK10
            MainV2.giveComport = true;
            byte[] buffer;

            mavlink_mission_set_current_t req = new mavlink_mission_set_current_t();

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
                        log.Info("setWPCurrent Retry " + retrys);
                        generatePacket(MAVLINK_MSG_ID_MISSION_SET_CURRENT, req);
                        start = DateTime.Now;
                        retrys--;
                        continue;
                    }
                    MainV2.giveComport = false;
                    throw new Exception("Timeout on read - setWPCurrent");
                }

                buffer = readPacket();
                if (buffer.Length > 5)
                {
                    if (buffer[5] == MAVLINK_MSG_ID_MISSION_CURRENT)
                    {
                        MainV2.giveComport = false;
                        return true;
                    }
                }
            }
        }

        [Obsolete("Mavlink 09", true)]
        public bool doAction(object actionid)
        {
            // mavlink 09
            throw new NotImplementedException();
        }

        public bool doCommand(MAV_CMD actionid, float p1, float p2, float p3, float p4, float p5, float p6, float p7)
        {

            MainV2.giveComport = true;
            byte[] buffer;

            mavlink_command_long_t req = new mavlink_command_long_t();

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
                        log.Info("doAction Retry " + retrys);
                        generatePacket(MAVLINK_MSG_ID_COMMAND_LONG, req);
                        start = DateTime.Now;
                        retrys--;
                        continue;
                    }
                    MainV2.giveComport = false;
                    throw new Exception("Timeout on read - doAction");
                }

                buffer = readPacket();
                if (buffer.Length > 5)
                {
                    if (buffer[5] == MAVLINK_MSG_ID_COMMAND_ACK)
                    {


                        var ack = buffer.ByteArrayToStructure<mavlink_command_ack_t>(6);


                        if (ack.result == (byte)MAV_RESULT.ACCEPTED)
                        {
                            MainV2.giveComport = false;
                            return true;
                        }
                        else
                        {
                            MainV2.giveComport = false;
                            return false;
                        }
                    }
                }
            }
#else
            MainV2.giveComport = true;
            byte[] buffer;

            mavlink_waypoint_set_current_t req = new mavlink_waypoint_set_current_t();

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
                        log.Info("setWPCurrent Retry " + retrys);
                        generatePacket(MAVLINK_MSG_ID_WAYPOINT_SET_CURRENT, req);
                        start = DateTime.Now;
                        retrys--;
                        continue;
                    }
                    MainV2.giveComport = false;
                    throw new Exception("Timeout on read - setWPCurrent");
                }

                buffer = readPacket();
                if (buffer.Length > 5)
                {
                    if (buffer[5] == MAVLINK_MSG_ID_WAYPOINT_CURRENT)
                    {
                        MainV2.giveComport = false;
                        return true;
                    }
                }
            }
        }

        public bool doCommand(ArdupilotMega.MAVLink.MAV_CMD actionid, float p1, float p2, float p3, float p4, float p5, float p6, float p7)
        {
            // mavlink 10
            throw new NotImplementedException();
        }

        public bool doAction(object actionidin)
        {
            MAV_ACTION actionid = (MAV_ACTION)actionidin;

            MainV2.giveComport = true;
            byte[] buffer;

            mavlink_action_t req = new mavlink_action_t();

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
                timeout = 20000;
            }

            while (true)
            {
                if (!(start.AddMilliseconds(timeout) > DateTime.Now))
                {
                    if (retrys > 0)
                    {
                        log.Info("doAction Retry " + retrys);
                        generatePacket(MAVLINK_MSG_ID_ACTION, req);
                        start = DateTime.Now;
                        retrys--;
                        continue;
                    }
                    MainV2.giveComport = false;
                    throw new Exception("Timeout on read - doAction");
                }

                buffer = readPacket();
                if (buffer.Length > 5)
                {
                    if (buffer[5] == MAVLINK_MSG_ID_ACTION_ACK)
                    {
                        if (buffer[7] == 1)
                        {
                            MainV2.giveComport = false;
                            return true;
                        }
                        else
                        {
                            MainV2.giveComport = false;
                            return false;
                        }
                    }
                }
            }

#endif
        }

        public void requestDatastream(byte id, byte hzrate)
        {

            double pps = 0;

            switch (id)
            {
                case (byte)MAVLink.MAV_DATA_STREAM.ALL:

                    break;
                case (byte)MAVLink.MAV_DATA_STREAM.EXTENDED_STATUS:
                    if (packetspersecondbuild[MAVLINK_MSG_ID_SYS_STATUS] < DateTime.Now.AddSeconds(-2))
                        break;
                    pps = packetspersecond[MAVLINK_MSG_ID_SYS_STATUS];
                    if (hzratecheck(pps, hzrate))
                    {
                        return;
                    }
                    break;
                case (byte)MAVLink.MAV_DATA_STREAM.EXTRA1:
                    if (packetspersecondbuild[MAVLINK_MSG_ID_ATTITUDE] < DateTime.Now.AddSeconds(-2))
                        break;
                    pps = packetspersecond[MAVLINK_MSG_ID_ATTITUDE];
                    if (hzratecheck(pps, hzrate))
                    {
                        return;
                    }
                    break;
                case (byte)MAVLink.MAV_DATA_STREAM.EXTRA2:
                    if (packetspersecondbuild[MAVLINK_MSG_ID_VFR_HUD] < DateTime.Now.AddSeconds(-2))
                        break;
                    pps = packetspersecond[MAVLINK_MSG_ID_VFR_HUD];
                    if (hzratecheck(pps, hzrate))
                    {
                        return;
                    }
                    break;
                case (byte)MAVLink.MAV_DATA_STREAM.EXTRA3:
                    if (packetspersecondbuild[MAVLINK_MSG_ID_AHRS] < DateTime.Now.AddSeconds(-2))
                        break;
                    pps = packetspersecond[MAVLINK_MSG_ID_AHRS];
                    if (hzratecheck(pps, hzrate))
                    {
                        return;
                    }
                    break;
                case (byte)MAVLink.MAV_DATA_STREAM.POSITION:
                    if (packetspersecondbuild[MAVLINK_MSG_ID_GLOBAL_POSITION_INT] < DateTime.Now.AddSeconds(-2))
                        break;
                    pps = packetspersecond[MAVLINK_MSG_ID_GLOBAL_POSITION_INT];
                    if (hzratecheck(pps, hzrate))
                    {
                        return;
                    }
                    break;
                case (byte)MAVLink.MAV_DATA_STREAM.RAW_CONTROLLER:
                    if (packetspersecondbuild[MAVLINK_MSG_ID_RC_CHANNELS_SCALED] < DateTime.Now.AddSeconds(-2))
                        break;
                    pps = packetspersecond[MAVLINK_MSG_ID_RC_CHANNELS_SCALED];
                    if (hzratecheck(pps, hzrate))
                    {
                        return;
                    }
                    break;
                case (byte)MAVLink.MAV_DATA_STREAM.RAW_SENSORS:
                    if (packetspersecondbuild[MAVLINK_MSG_ID_RAW_IMU] < DateTime.Now.AddSeconds(-2))
                        break;
                    pps = packetspersecond[MAVLINK_MSG_ID_RAW_IMU];
                    if (hzratecheck(pps, hzrate))
                    {
                        return;
                    }
                    break;
                case (byte)MAVLink.MAV_DATA_STREAM.RC_CHANNELS:
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


            log.InfoFormat("Request stream {0} at {1} hz", Enum.Parse(typeof(MAV_DATA_STREAM), id.ToString()), hzrate);
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
            mavlink_request_data_stream_t req = new mavlink_request_data_stream_t();
            req.target_system = sysid;
            req.target_component = compid;

            req.req_message_rate = hzrate;
            req.start_stop = 1; // start
            req.req_stream_id = id; // id

            // send each one twice.
            generatePacket(MAVLINK_MSG_ID_REQUEST_DATA_STREAM, req);
            generatePacket(MAVLINK_MSG_ID_REQUEST_DATA_STREAM, req);
        }

        /// <summary>
        /// Returns WP count
        /// </summary>
        /// <returns></returns>
        public byte getWPCount()
        {
            MainV2.giveComport = true;
            byte[] buffer;
#if MAVLINK10
            mavlink_mission_request_list_t req = new mavlink_mission_request_list_t();

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
                        log.Info("getWPCount Retry " + retrys + " - giv com " + MainV2.giveComport);
                        generatePacket(MAVLINK_MSG_ID_MISSION_REQUEST_LIST, req);
                        start = DateTime.Now;
                        retrys--;
                        continue;
                    }
                    MainV2.giveComport = false;
                    //return (byte)int.Parse(param["WP_TOTAL"].ToString());
                    throw new Exception("Timeout on read - getWPCount");
                }

                buffer = readPacket();
                if (buffer.Length > 5)
                {
                    if (buffer[5] == MAVLINK_MSG_ID_MISSION_COUNT)
                    {



                        var count = buffer.ByteArrayToStructure<mavlink_mission_count_t>(6);


                        log.Info("wpcount: " + count.count);
                        MainV2.giveComport = false;
                        return (byte)count.count; // should be ushort, but apm has limited wp count < byte
                    }
                    else
                    {
                        log.Info(DateTime.Now + " PC wpcount " + buffer[5] + " need " + MAVLINK_MSG_ID_MISSION_COUNT + " " + this.BaseStream.BytesToRead);
                    }
                }
            }
#else

            mavlink_waypoint_request_list_t req = new mavlink_waypoint_request_list_t();

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
                        log.Info("getWPCount Retry " + retrys + " - giv com " + MainV2.giveComport);
                        generatePacket(MAVLINK_MSG_ID_WAYPOINT_REQUEST_LIST, req);
                        start = DateTime.Now;
                        retrys--;
                        continue;
                    }
                    MainV2.giveComport = false;
                    //return (byte)int.Parse(param["WP_TOTAL"].ToString());
                    throw new Exception("Timeout on read - getWPCount");
                }

                buffer = readPacket();
                if (buffer.Length > 5)
                {
                    if (buffer[5] == MAVLINK_MSG_ID_WAYPOINT_COUNT)
                    {

                        log.Info("wpcount: " + buffer[9]);
                        MainV2.giveComport = false;
                        return buffer[9]; // should be ushort, but apm has limited wp count < byte
                    }
                    else
                    {
                        log.Info(DateTime.Now + " PC wpcount " + buffer[5] + " need " + MAVLINK_MSG_ID_WAYPOINT_COUNT + " " + this.BaseStream.BytesToRead);
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
            MainV2.giveComport = true;
            Locationwp loc = new Locationwp();
#if MAVLINK10
            mavlink_mission_request_t req = new mavlink_mission_request_t();

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
                        log.Info("getWP Retry " + retrys);
                        generatePacket(MAVLINK_MSG_ID_MISSION_REQUEST, req);
                        start = DateTime.Now;
                        retrys--;
                        continue;
                    }
                    MainV2.giveComport = false;
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


                        //Array.Copy(buffer, 6, buffer, 0, buffer.Length - 6);

                        var wp = buffer.ByteArrayToStructure<mavlink_mission_item_t>(6);


#else

            mavlink_waypoint_request_t req = new mavlink_waypoint_request_t();

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
                        log.Info("getWP Retry " + retrys);
                        generatePacket(MAVLINK_MSG_ID_WAYPOINT_REQUEST, req);
                        start = DateTime.Now;
                        retrys--;
                        continue;
                    }
                    MainV2.giveComport = false;
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
                        mavlink_waypoint_t wp = buffer.ByteArrayToStructure<mavlink_waypoint_t>(6);

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
                        log.InfoFormat("getWP {0} {1} {2} {3} {4} opt {5}", loc.id, loc.p1, loc.alt, loc.lat, loc.lng, loc.options);

                        break;
                    }
                    else
                    {
                        log.Info(DateTime.Now + " PC getwp " + buffer[5]);
                    }
                }
            }
            MainV2.giveComport = false;
            return loc;
        }

        public object DebugPacket(byte[] datin)
        {
            string text = "";
            return DebugPacket(datin, ref text, true);
        }

        public object DebugPacket(byte[] datin, bool PrintToConsole)
        {
            string text = "";
            return DebugPacket(datin, ref text, PrintToConsole);
        }

        public object DebugPacket(byte[] datin, ref string text)
        {
            return DebugPacket(datin, ref text, true);
        }

        /// <summary>
        /// Print entire decoded packet to console
        /// </summary>
        /// <param name="datin">packet byte array</param>
        /// <returns>struct of data</returns>
        public object DebugPacket(byte[] datin, ref string text, bool PrintToConsole, string delimeter = " ")
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

                    textoutput = string.Format("{0:X}{6}{1:X}{6}{2:X}{6}{3:X}{6}{4:X}{6}{5:X}{6}", header, length, seq, sysid, compid, messid, delimeter);

                    object data = Activator.CreateInstance(MAVLINK_MESSAGE_INFO[messid]);

                    MavlinkUtil.ByteArrayToStructure(datin, ref data, 6);

                    Type test = data.GetType();

                    if (PrintToConsole)
                    {

                        textoutput = textoutput + test.Name + delimeter;

                        foreach (var field in test.GetFields())
                        {
                            // field.Name has the field's name.

                            object fieldValue = field.GetValue(data); // Get value

                            if (field.FieldType.IsArray)
                            {
                                textoutput = textoutput + field.Name + delimeter;
                                byte[] crap = (byte[])fieldValue;
                                foreach (byte fiel in crap)
                                {
                                    if (fiel == 0)
                                    {
                                        break;
                                    }
                                    else
                                    {
                                        textoutput = textoutput + (char)fiel;
                                    }
                                }
                                textoutput = textoutput + delimeter;
                            }
                            else
                            {
                                textoutput = textoutput + field.Name + delimeter + fieldValue.ToString() + delimeter;
                            }
                        }
                        textoutput = textoutput + delimeter + "Len" + delimeter + datin.Length + "\r\n";
                        if (PrintToConsole)
                            Console.Write(textoutput);

                        if (text != null)
                            text = textoutput;
                    }

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
            MainV2.giveComport = true;
            mavlink_mission_count_t req = new mavlink_mission_count_t();

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
                        log.Info("setWPTotal Retry " + retrys);
                        generatePacket(MAVLINK_MSG_ID_MISSION_COUNT, req);
                        start = DateTime.Now;
                        retrys--;
                        continue;
                    }
                    MainV2.giveComport = false;
                    throw new Exception("Timeout on read - setWPTotal");
                }
                byte[] buffer = readPacket();
                if (buffer.Length > 9)
                {
                    if (buffer[5] == MAVLINK_MSG_ID_MISSION_REQUEST)
                    {



                        var request = buffer.ByteArrayToStructure<mavlink_mission_request_t>(6);

                        if (request.seq == 0)
                        {
                            if (param["WP_TOTAL"] != null)
                                param["WP_TOTAL"] = (float)wp_total - 1;
                            if (param["CMD_TOTAL"] != null)
                                param["CMD_TOTAL"] = (float)wp_total - 1;
                            MainV2.giveComport = false;
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
            MainV2.giveComport = true;
            mavlink_waypoint_count_t req = new mavlink_waypoint_count_t();

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
                        log.Info("setWPTotal Retry " + retrys);
                        generatePacket(MAVLINK_MSG_ID_WAYPOINT_COUNT, req);
                        start = DateTime.Now;
                        retrys--;
                        continue;
                    }
                    MainV2.giveComport = false;
                    throw new Exception("Timeout on read - setWPTotal");
                }
                byte[] buffer = readPacket();
                if (buffer.Length > 9)
                {
                    if (buffer[5] == MAVLINK_MSG_ID_WAYPOINT_REQUEST)
                    {
                        mavlink_waypoint_request_t request = buffer.ByteArrayToStructure<mavlink_waypoint_request_t>(6);

                        if (request.seq == 0)
                        {
                            if (param["WP_TOTAL"] != null)
                                param["WP_TOTAL"] = (float)wp_total - 1;
                            if (param["CMD_TOTAL"] != null)
                                param["CMD_TOTAL"] = (float)wp_total - 1;
                            MainV2.giveComport = false;
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
            MainV2.giveComport = true;
#if MAVLINK10
            mavlink_mission_item_t req = new mavlink_mission_item_t();
#else
            mavlink_waypoint_t req = new mavlink_waypoint_t();
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

            log.InfoFormat("setWP {6} frame {0} cmd {1} p1 {2} x {3} y {4} z {5}", req.frame, req.command, req.param1, req.x, req.y, req.z, index);

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
                        log.Info("setWP Retry " + retrys);
#if MAVLINK10
            generatePacket(MAVLINK_MSG_ID_MISSION_ITEM, req);
#else
                        generatePacket(MAVLINK_MSG_ID_WAYPOINT, req);
#endif
                        start = DateTime.Now;
                        retrys--;
                        continue;
                    }
                    MainV2.giveComport = false;
                    throw new Exception("Timeout on read - setWP");
                }
                byte[] buffer = readPacket();
                if (buffer.Length > 5)
                {
#if MAVLINK10
                    if (buffer[5] == MAVLINK_MSG_ID_MISSION_ACK)
                    {


                        var ans = buffer.ByteArrayToStructure<mavlink_mission_ack_t>(6);


                        log.Info("set wp " + index + " ACK 47 : " + buffer[5] + " ans " + Enum.Parse(typeof(MAV_MISSION_RESULT), ans.type.ToString()));
                        break;
                    }
                    else if (buffer[5] == MAVLINK_MSG_ID_MISSION_REQUEST)
                    {
                        var ans = buffer.ByteArrayToStructure<mavlink_mission_request_t>(6);




                        if (ans.seq == (index + 1))
                        {
                            log.Info("set wp doing " + index + " req " + ans.seq + " REQ 40 : " + buffer[5]);
                            MainV2.giveComport = false;
                            break;
                        }
                        else
                        {
                            log.InfoFormat("set wp fail doing " + index + " req " + ans.seq + " ACK 47 or REQ 40 : " + buffer[5] + " seq {0} ts {1} tc {2}", req.seq, req.target_system, req.target_component);
                            //break;
                        }
                    }
                    else
                    {
                        //Console.WriteLine(DateTime.Now + " PC setwp " + buffer[5]);
                    }
#else
                    if (buffer[5] == MAVLINK_MSG_ID_WAYPOINT_ACK)
                    { //mavlink_waypoint_request_t
                        log.Info("set wp " + index + " ACK 47 : " + buffer[5]);
                        break;
                    }
                    else if (buffer[5] == MAVLINK_MSG_ID_WAYPOINT_REQUEST)
                    {
                        mavlink_waypoint_request_t ans = buffer.ByteArrayToStructure<mavlink_waypoint_request_t>(6);

                        if (ans.seq == (index + 1))
                        {
                            log.Info("set wp doing " + index + " req " + ans.seq + " REQ 40 : " + buffer[5]);
                            MainV2.giveComport = false;
                            break;
                        }
                        else
                        {
                            log.InfoFormat("set wp fail doing " + index + " req " + ans.seq + " ACK 47 or REQ 40 : " + buffer[5] + " seq {0} ts {1} tc {2}", req.seq, req.target_system, req.target_component);
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
            mavlink_mount_configure_t req = new mavlink_mount_configure_t();

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
            mavlink_mount_control_t req = new mavlink_mount_control_t();

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
                MAVLink.mavlink_set_mode_t mode = new MAVLink.mavlink_set_mode_t();

                if (translateMode(modein, ref mode))
                {
                    generatePacket((byte)MAVLink.MAVLINK_MSG_ID_SET_MODE, mode);
                    System.Threading.Thread.Sleep(10);
                    generatePacket((byte)MAVLink.MAVLINK_MSG_ID_SET_MODE, mode);
                }
            }
            catch { System.Windows.Forms.MessageBox.Show("Failed to change Modes"); }
#else
            try
            {
                MAVLink.mavlink_set_nav_mode_t navmode = new MAVLink.mavlink_set_nav_mode_t();

                MAVLink.mavlink_set_mode_t mode = new MAVLink.mavlink_set_mode_t();

                if (translateMode(modein, ref navmode, ref mode))
                {
                    generatePacket((byte)MAVLink.MAVLINK_MSG_ID_SET_NAV_MODE, navmode);
                    System.Threading.Thread.Sleep(10);
                    generatePacket((byte)MAVLink.MAVLINK_MSG_ID_SET_MODE, mode);
                    System.Threading.Thread.Sleep(10);
                    generatePacket((byte)MAVLink.MAVLINK_MSG_ID_SET_NAV_MODE, navmode);
                    System.Threading.Thread.Sleep(10);
                    generatePacket((byte)MAVLink.MAVLINK_MSG_ID_SET_MODE, mode);
                }
            }
            catch { System.Windows.Forms.CustomMessageBox.Show("Failed to change Modes"); }

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

            BaseStream.ReadTimeout = 1200; // 1200 ms between chars - the gps detection requires this.

            DateTime start = DateTime.Now;

            try
            {
                // test fabs idea - http://diydrones.com/profiles/blogs/flying-with-joystick?commentId=705844%3AComment%3A818712&xg_source=msg_com_blogpost
                if (BaseStream.IsOpen && BaseStream.BytesToWrite > 0)
                {
                    // slow down execution. else 100% cpu
                    Thread.Sleep(1);
                    return new byte[0];
                }
            }
            catch (Exception ex) { log.Info(ex.ToString()); }

            lock (readlock)
            {

                while (BaseStream.IsOpen || logreadmode)
                {
                    try
                    {
                        if (readcount > 300)
                        {
                            log.Info("MAVLink readpacket No valid mavlink packets");
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

                            DateTime to = DateTime.Now.AddMilliseconds(BaseStream.ReadTimeout);

                            while (BaseStream.BytesToRead <= 0)
                            {
                                if (DateTime.Now > to)
                                {
                                    log.InfoFormat("MAVLINK: S wait time out btr {0} len {1}", BaseStream.BytesToRead, length);
                                    throw new Exception("Timeout");
                                }
                                System.Threading.Thread.Sleep(1);
                            }
                            if (BaseStream.IsOpen)
                            {
                                temp[count] = (byte)BaseStream.ReadByte();
                                if (rawlogfile != null && rawlogfile.BaseStream.CanWrite)
                                    rawlogfile.Write(temp[count]);
                            }
                        }
                    }
                    catch (Exception e) { log.Info("MAVLink readpacket read error: " + e.ToString()); break; }

                    // check if looks like a mavlink packet and check for exclusions and write to console
                    if (temp[0] != 254 && temp[0] != 'U' || lastbad[0] == 'I' && lastbad[1] == 'M' || lastbad[1] == 'G' || lastbad[1] == 'A') // out of sync "AUTO" "GUIDED" "IMU"
                    {
                        if (temp[0] >= 0x20 && temp[0] <= 127 || temp[0] == '\n' || temp[0] == '\r')
                        {
                            TCPConsole.Write(temp[0]);
                            Console.Write((char)temp[0]);
                        }
                        _bytesReceivedSubj.OnNext(1);
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
                                    if (temp[3] == '3' && temp[4] == 'D')
                                    {
                                        // this is a 3dr radio rssi packet
                                    }
                                    else
                                    {
                                        log.InfoFormat("Mavlink Bad Packet (not addressed to this MAV) got {0} {1} vs {2} {3}", temp[3], temp[4], sysid, compid);
                                        return new byte[0];
                                    }
                                }
                            }

                            try
                            {
                                if (logreadmode)
                                {

                                }
                                else
                                {
                                    DateTime to = DateTime.Now.AddMilliseconds(BaseStream.ReadTimeout);

                                    while (BaseStream.BytesToRead < (length - 4))
                                    {
                                        if (DateTime.Now > to)
                                        {
                                            log.InfoFormat("MAVLINK: L wait time out btr {0} len {1}", BaseStream.BytesToRead, length);
                                            break;
                                        }
                                        //Console.WriteLine("data " + 0 + " " + length + " aval " + BaseStream.BytesToRead);
                                    }
                                    if (BaseStream.IsOpen)
                                    {
                                        int read = BaseStream.Read(temp, 6, length - 4);
                                        if (rawlogfile != null && rawlogfile.BaseStream.CanWrite)
                                        {
                                            // write only what we read, temp is the whole packet, so 6-end
                                            rawlogfile.Write(temp, 6, read);
                                            rawlogfile.BaseStream.Flush();
                                        }
                                    }
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

            _bytesReceivedSubj.OnNext(temp.Length);

            if (packetlosttimer.AddSeconds(5) < DateTime.Now)
            {
                packetlosttimer = DateTime.Now;
                packetslost = (packetslost * 0.8f);
                packetsnotlost = (packetsnotlost * 0.8f);
            }

            MainV2.cs.linkqualitygcs = (ushort)((packetsnotlost / (packetsnotlost + packetslost)) * 100.0);

            if (bpstime.Second != DateTime.Now.Second && !logreadmode)
            {
                Console.Write("bps {0} loss {1} left {2} mem {3}      \n", bps1, synclost, BaseStream.BytesToRead, System.GC.GetTotalMemory(false) / 1024 / 1024.0);
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

            ushort crc = MavlinkCRC.crc_calculate(temp, temp.Length - 2);

            if (temp.Length > 5 && temp[0] == 254)
            {
                crc = MavlinkCRC.crc_accumulate(MAVLINK_MESSAGE_CRCS[temp[5]], crc);
            }

            if (temp.Length > 5 && temp[1] != MAVLINK_MESSAGE_LENGTHS[temp[5]])
            {
                if (MAVLINK_MESSAGE_LENGTHS[temp[5]] == 0) // pass for unknown packets
                {

                }
                else
                {
                    log.InfoFormat("Mavlink Bad Packet (Len Fail) len {0} pkno {1}", temp.Length, temp[5]);
#if MAVLINK10
                    if (temp.Length == 11 && temp[0] == 'U' && temp[5] == 0){
                        string message ="Mavlink 0.9 Heartbeat, Please upgrade your AP, This planner is for Mavlink 1.0\n\n";
                        System.Windows.Forms.CustomMessageBox.Show(message);
                        throw new Exception(message);
                    }
#else
                    if (temp.Length == 17 && temp[0] == 254 && temp[5] == 0)
                    {
                        string message = "Mavlink 1.0 Heartbeat, Please Upgrade your Mission Planner, This planner is for Mavlink 0.9\n\n";
                        System.Windows.Forms.CustomMessageBox.Show(message);
                        throw new Exception(message);
                    }
#endif
                    return new byte[0];
                }
            }

            if (temp.Length < 5 || temp[temp.Length - 1] != (crc >> 8) || temp[temp.Length - 2] != (crc & 0xff))
            {
                int packetno = -1;
                if (temp.Length > 5)
                {
                    packetno = temp[5];
                }
                log.InfoFormat("Mavlink Bad Packet (crc fail) len {0} crc {1} pkno {2}", temp.Length, crc, packetno);
                return new byte[0];
            }

            try
            {
                if ((temp[0] == 'U' || temp[0] == 254) && temp.Length >= temp[1])
                {
                    if (temp[3] == '3' && temp[4] == 'D')
                    {

                    }
                    else
                    {
                        byte packetSeqNo = temp[2];
                        int expectedPacketSeqNo = ((recvpacketcount + 1) % 0x100);

                        if (packetSeqNo != expectedPacketSeqNo)
                        {
                            synclost++; // actualy sync loss's
                            int numLost = 0;

                            if (packetSeqNo < ((recvpacketcount + 1))) // recvpacketcount = 255 then   10 < 256 = true if was % 0x100 this would fail
                            {
                                numLost = 0x100 - expectedPacketSeqNo + packetSeqNo;
                            }
                            else
                            {
                                numLost = packetSeqNo - recvpacketcount;
                            }
                            packetslost += numLost;
                            WhenPacketLost.OnNext(numLost);

                            log.InfoFormat("lost {0} pkts {1}", packetSeqNo, (int)packetslost);
                        }

                        packetsnotlost++;

                        recvpacketcount = packetSeqNo;
                        WhenPacketReceived.OnNext(1);
                       // Console.WriteLine(DateTime.Now.Millisecond);
                    }

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
                        log.Info(DateTime.Now + " " + logdata);

                        if (MainV2.speechEngine != null && MainV2.config["speechenable"] != null && MainV2.config["speechenable"].ToString() == "True")
                        {
                            //MainV2.talk.SpeakAsync(logdata);
                        }

                    }

                    getWPsfromstream(ref temp);

                    try
                    {
                        if (logfile != null && logfile.BaseStream.CanWrite)
                        {
                            lock (logwritelock)
                            {
                                byte[] datearray = BitConverter.GetBytes((UInt64)((DateTime.UtcNow - new DateTime(1970, 1, 1)).TotalMilliseconds * 1000)); //ASCIIEncoding.ASCII.GetBytes(DateTime.Now.ToBinary() + ":");
                                Array.Reverse(datearray);
                                logfile.Write(datearray, 0, datearray.Length);
                                logfile.Write(temp, 0, temp.Length);
                                logfile.Flush();
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
        /// <param name="buffer">packet</param>
        void getWPsfromstream(ref byte[] buffer)
        {
#if MAVLINK10
                    if (buffer[5] == MAVLINK_MSG_ID_MISSION_COUNT)
                    {
                        // clear old
                        wps = new PointLatLngAlt[wps.Length];
                    }

                    if (buffer[5] == MAVLink.MAVLINK_MSG_ID_MISSION_ITEM)
                    {
                        mavlink_mission_item_t wp = buffer.ByteArrayToStructure<mavlink_mission_item_t>(6);
#else

            if (buffer[5] == MAVLINK_MSG_ID_WAYPOINT_COUNT)
            {
                // clear old
                wps = new PointLatLngAlt[wps.Length];
            }

            if (buffer[5] == MAVLink.MAVLINK_MSG_ID_WAYPOINT)
            {
                mavlink_waypoint_t wp = buffer.ByteArrayToStructure<mavlink_waypoint_t>(6);

#endif
                wps[wp.seq] = new PointLatLngAlt(wp.x, wp.y, wp.z, wp.seq.ToString());
            }
        }

        public PointLatLngAlt getFencePoint(int no, ref int total)
        {
            byte[] buffer;

            MainV2.giveComport = true;

            PointLatLngAlt plla = new PointLatLngAlt();
            mavlink_fence_fetch_point_t req = new mavlink_fence_fetch_point_t();

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
                        log.Info("getFencePoint Retry " + retrys + " - giv com " + MainV2.giveComport);
                        generatePacket(MAVLINK_MSG_ID_FENCE_FETCH_POINT, req);
                        start = DateTime.Now;
                        retrys--;
                        continue;
                    }
                    MainV2.giveComport = false;
                    throw new Exception("Timeout on read - getFencePoint");
                }

                buffer = readPacket();
                if (buffer.Length > 5)
                {
                    if (buffer[5] == MAVLINK_MSG_ID_FENCE_POINT)
                    {
                        MainV2.giveComport = false;

                        mavlink_fence_point_t fp = buffer.ByteArrayToStructure<mavlink_fence_point_t>(6);

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
            mavlink_fence_point_t fp = new mavlink_fence_point_t();

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
                    log.InfoFormat("lost sync byte {0} pos {1}", temp[0], logplaybackfile.BaseStream.Position);
                    a = 0;
                    continue;
                }
                if (a == 1)
                {
                    length = temp[1] + 6 + 2; // 6 header + 2 checksum
                }
                a++;
            }

            // set ap type for log file playback
            if (temp[5] == 0) {
                mavlink_heartbeat_t hb = temp.ByteArrayToStructure<mavlink_heartbeat_t>(6);

                mavlinkversion = hb.mavlink_version;
                aptype = (MAV_TYPE)hb.type;
            }

            return temp;
        }


#if MAVLINK10
		
        public static bool translateMode(string modein, ref MAVLink.mavlink_set_mode_t mode)
        {
            //MAVLink09.mavlink_set_mode_t mode = new MAVLink09.mavlink_set_mode_t();
            mode.target_system = MainV2.comPort.sysid;

            try
            {
                if (Common.getModes() == typeof(Common.apmmodes))
                {
                    switch (EnumTranslator.GetValue<Common.apmmodes>(modein))
                    {
                        case (int)Common.apmmodes.MANUAL:
                        case (int)Common.apmmodes.CIRCLE:
                        case (int)Common.apmmodes.STABILIZE:
                        case (int)Common.apmmodes.AUTO:
                        case (int)Common.apmmodes.RTL:
                        case (int)Common.apmmodes.LOITER:
                        case (int)Common.apmmodes.FLY_BY_WIRE_A:
                        case (int)Common.apmmodes.FLY_BY_WIRE_B:
                            mode.base_mode = (byte)MAVLink.MAV_MODE_FLAG.CUSTOM_MODE_ENABLED;
                            mode.custom_mode = (uint)EnumTranslator.GetValue<Common.apmmodes>(modein);
                            break;
                        default:
                            MessageBox.Show("No Mode Changed " +  modein);
                            return false;
                    }
                }
                else if (Common.getModes() == typeof(Common.ac2modes))
                {
                    switch (EnumTranslator.GetValue<Common.ac2modes>(modein))
                    {
                        case (int)Common.ac2modes.STABILIZE:
                        case (int)Common.ac2modes.AUTO:
                        case (int)Common.ac2modes.RTL:
                        case (int)Common.ac2modes.LOITER:
                        case (int)Common.ac2modes.ACRO:
                        case (int)Common.ac2modes.ALT_HOLD:
                        case (int)Common.ac2modes.CIRCLE:
                        case (int)Common.ac2modes.POSITION:
                            mode.base_mode = (byte)MAVLink.MAV_MODE_FLAG.CUSTOM_MODE_ENABLED;
                            mode.custom_mode = (uint)EnumTranslator.GetValue<Common.ac2modes>(modein);
                            break;
                        default:
                            MessageBox.Show("No Mode Changed " +  modein);
                            return false;
                    }
                }
            }
            catch { System.Windows.Forms.MessageBox.Show("Failed to find Mode"); return false; }

            return true;
        }
		
#else
        public static bool translateMode(string modein, ref  MAVLink.mavlink_set_nav_mode_t navmode, ref MAVLink.mavlink_set_mode_t mode)
        {

            //MAVLink.mavlink_set_nav_mode_t navmode = new MAVLink.mavlink_set_nav_mode_t();
            navmode.target = MainV2.comPort.sysid;
            navmode.nav_mode = 255;

            //MAVLink.mavlink_set_mode_t mode = new MAVLink.mavlink_set_mode_t();
            mode.target = MainV2.comPort.sysid;

            try
            {
                if (Common.getModes() == typeof(Common.apmmodes))
                {
                    switch (EnumTranslator.GetValue<Common.apmmodes>(modein))
                    {
                        case (int)Common.apmmodes.MANUAL:
                            mode.mode = (byte)MAVLink.MAV_MODE.MAV_MODE_MANUAL;
                            break;
                        case (int)Common.apmmodes.GUIDED:
                            mode.mode = (byte)MAVLink.MAV_MODE.MAV_MODE_GUIDED;
                            break;
                        case (int)Common.apmmodes.STABILIZE:
                            mode.mode = (byte)MAVLink.MAV_MODE.MAV_MODE_TEST1;
                            break;
                        // AUTO MODES
                        case (int)Common.apmmodes.AUTO:
                            navmode.nav_mode = (byte)MAVLink.MAV_NAV.MAV_NAV_WAYPOINT;
                            mode.mode = (byte)MAVLink.MAV_MODE.MAV_MODE_AUTO;
                            break;
                        case (int)Common.apmmodes.RTL:
                            navmode.nav_mode = (byte)MAVLink.MAV_NAV.MAV_NAV_RETURNING;
                            mode.mode = (byte)MAVLink.MAV_MODE.MAV_MODE_AUTO;
                            break;
                        case (int)Common.apmmodes.LOITER:
                            navmode.nav_mode = (byte)MAVLink.MAV_NAV.MAV_NAV_LOITER;
                            mode.mode = (byte)MAVLink.MAV_MODE.MAV_MODE_AUTO;
                            break;
                        // FBW
                        case (int)Common.apmmodes.FLY_BY_WIRE_A:
                            navmode.nav_mode = (byte)1;
                            mode.mode = (byte)MAVLink.MAV_MODE.MAV_MODE_TEST2;
                            break;
                        case (int)Common.apmmodes.FLY_BY_WIRE_B:
                            navmode.nav_mode = (byte)2;
                            mode.mode = (byte)MAVLink.MAV_MODE.MAV_MODE_TEST2;
                            break;
                        default:
                            CustomMessageBox.Show("No Mode Changed " + modein);
                            return false;
                    }
                }
                else if (Common.getModes() == typeof(Common.aprovermodes))
                {
                    switch (EnumTranslator.GetValue<Common.aprovermodes>(modein))
                    {
                        case (int)Common.aprovermodes.MANUAL:
                            mode.mode = (byte)MAVLink.MAV_MODE.MAV_MODE_MANUAL;
                            break;
                        case (int)Common.aprovermodes.GUIDED:
                            mode.mode = (byte)MAVLink.MAV_MODE.MAV_MODE_GUIDED;
                            break;
                        case (int)Common.aprovermodes.LEARNING:
                            mode.mode = (byte)MAVLink.MAV_MODE.MAV_MODE_TEST1;
                            break;
                        // AUTO MODES
                        case (int)Common.aprovermodes.AUTO:
                            navmode.nav_mode = (byte)MAVLink.MAV_NAV.MAV_NAV_WAYPOINT;
                            mode.mode = (byte)MAVLink.MAV_MODE.MAV_MODE_AUTO;
                            break;
                        case (int)Common.aprovermodes.RTL:
                            navmode.nav_mode = (byte)MAVLink.MAV_NAV.MAV_NAV_RETURNING;
                            mode.mode = (byte)MAVLink.MAV_MODE.MAV_MODE_AUTO;
                            break;
                        case (int)Common.aprovermodes.LOITER:
                            navmode.nav_mode = (byte)MAVLink.MAV_NAV.MAV_NAV_LOITER;
                            mode.mode = (byte)MAVLink.MAV_MODE.MAV_MODE_AUTO;
                            break;
                        // FBW
                        case (int)Common.aprovermodes.FLY_BY_WIRE_A:
                            navmode.nav_mode = (byte)1;
                            mode.mode = (byte)MAVLink.MAV_MODE.MAV_MODE_TEST2;
                            break;
                        case (int)Common.aprovermodes.FLY_BY_WIRE_B:
                            navmode.nav_mode = (byte)2;
                            mode.mode = (byte)MAVLink.MAV_MODE.MAV_MODE_TEST2;
                            break;
                        default:
                            CustomMessageBox.Show("No Mode Changed " + modein);
                            return false;
                    }
                }
                else if (Common.getModes() == typeof(Common.ac2modes))
                {
                    switch (EnumTranslator.GetValue<Common.ac2modes>(modein))
                    {
                        case (int)Common.ac2modes.GUIDED:
                            mode.mode = (byte)MAVLink.MAV_MODE.MAV_MODE_GUIDED;
                            break;
                        case (int)Common.ac2modes.STABILIZE:
                            mode.mode = (byte)MAVLink.MAV_MODE.MAV_MODE_MANUAL;
                            break;
                        // AUTO MODES
                        case (int)Common.ac2modes.AUTO:
                            navmode.nav_mode = (byte)MAVLink.MAV_NAV.MAV_NAV_WAYPOINT;
                            mode.mode = (byte)MAVLink.MAV_MODE.MAV_MODE_AUTO;
                            break;
                        case (int)Common.ac2modes.RTL:
                            navmode.nav_mode = (byte)MAVLink.MAV_NAV.MAV_NAV_RETURNING;
                            mode.mode = (byte)MAVLink.MAV_MODE.MAV_MODE_AUTO;
                            break;
                        case (int)Common.ac2modes.LOITER:
                            navmode.nav_mode = (byte)MAVLink.MAV_NAV.MAV_NAV_LOITER;
                            mode.mode = (byte)MAVLink.MAV_MODE.MAV_MODE_AUTO;
                            break;
                        default:
                            CustomMessageBox.Show("No Mode Changed " + modein);
                            return false;
                    }
                }
            }
            catch { System.Windows.Forms.CustomMessageBox.Show("Failed to find Mode"); return false; }

            return true;
        }
#endif

#if MAVLINK10
        public void setAPType()
        {
            switch (aptype)
            {
                case MAVLink.MAV_TYPE.FIXED_WING:
                    MainV2.cs.firmware = MainV2.Firmwares.ArduPlane;
                    break;
                case MAVLink.MAV_TYPE.QUADROTOR:
                    MainV2.cs.firmware = MainV2.Firmwares.ArduCopter2;
                    break;
                case MAVLink.MAV_TYPE.GROUND_ROVER:
                    MainV2.cs.firmware = MainV2.Firmwares.ArduRover;
                    break;
                default:
                    break;
            }
        }
#else 
        public void setAPType()
        {
            switch (aptype)
            {
                case MAVLink.MAV_TYPE.MAV_FIXED_WING:
                    MainV2.cs.firmware = MainV2.Firmwares.ArduPlane;
                    break;
                case MAVLink.MAV_TYPE.MAV_QUADROTOR:
                    MainV2.cs.firmware = MainV2.Firmwares.ArduCopter2;
                    break;
                case MAVLink.MAV_TYPE.MAV_GROUND:
                    MainV2.cs.firmware = MainV2.Firmwares.ArduRover;
                    break;
                default:
                    break;
            }
        }
#endif
    }
}