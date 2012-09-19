using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Net;
using System.Net.Sockets;

namespace ArdupilotMega.HIL
{
    class XPlane : Hil
    {
        Socket SimulatorRECV;
        UdpClient XplanesSEND;
        EndPoint Remote = (EndPoint)(new IPEndPoint(IPAddress.Any, 0));

        // place to store the xplane packet data
        float[][] DATA = new float[113][];

        public override void SetupSockets(int recvPort, int SendPort, string simIP)
        {
            // setup receiver
            IPEndPoint ipep = new IPEndPoint(IPAddress.Any, recvPort);

            SimulatorRECV = new Socket(AddressFamily.InterNetwork,
                            SocketType.Dgram, ProtocolType.Udp);

            SimulatorRECV.Bind(ipep);

            UpdateStatus(-1, "Listerning on port UDP " + recvPort + " (sim->planner)\n");


            // setup sender
            XplanesSEND = new UdpClient(simIP, SendPort);

            UpdateStatus(-1, "Sending to port UDP " + SendPort + " (planner->sim)\n");

            setupXplane();

            UpdateStatus(-1, "Sent xplane settings\n");
        }
        public override void Shutdown()
        {
            try
            {
                SimulatorRECV.Close();
            }
            catch { }
            try
            {
                XplanesSEND.Close();
            }
            catch { }
        }

        public override void GetFromSim(ref sitl_fdm sitldata)
        {
            if (SimulatorRECV.Available > 0)
            {
                byte[] udpdata = new byte[1500];
                int receviedbytes = 0;
                try
                {
                    while (SimulatorRECV.Available > 0)
                    {
                        receviedbytes = SimulatorRECV.ReceiveFrom(udpdata, ref Remote);
                    }
                }
                catch { }

                if (udpdata[0] == 'D' && udpdata[1] == 'A')
                {
                    // Xplanes sends
                    // 5 byte header
                    // 1 int for the index - numbers on left of output
                    // 8 floats - might be useful. or 0 if not
                    int count = 5;
                    while (count < receviedbytes)
                    {
                        int index = BitConverter.ToInt32(udpdata, count);

                        DATA[index] = new float[8];

                        DATA[index][0] = BitConverter.ToSingle(udpdata, count + 1 * 4); ;
                        DATA[index][1] = BitConverter.ToSingle(udpdata, count + 2 * 4); ;
                        DATA[index][2] = BitConverter.ToSingle(udpdata, count + 3 * 4); ;
                        DATA[index][3] = BitConverter.ToSingle(udpdata, count + 4 * 4); ;
                        DATA[index][4] = BitConverter.ToSingle(udpdata, count + 5 * 4); ;
                        DATA[index][5] = BitConverter.ToSingle(udpdata, count + 6 * 4); ;
                        DATA[index][6] = BitConverter.ToSingle(udpdata, count + 7 * 4); ;
                        DATA[index][7] = BitConverter.ToSingle(udpdata, count + 8 * 4); ;

                        count += 36; // 8 * float
                    }

                    bool xplane9 = !xplane10;

                    if (xplane9)
                    {
                        sitldata.pitchDeg = (DATA[18][0]);
                        sitldata.rollDeg = (DATA[18][1]);
                        sitldata.yawDeg = (DATA[18][2]);
                        sitldata.pitchRate = (DATA[17][0] * rad2deg);
                        sitldata.rollRate = (DATA[17][1] * rad2deg);
                        sitldata.yawRate = (DATA[17][2] * rad2deg);

                        sitldata.heading = ((float)DATA[19][2]);
                    }
                    else
                    {
                        sitldata.pitchDeg = (DATA[17][0]);
                        sitldata.rollDeg = (DATA[17][1]);
                        sitldata.yawDeg = (DATA[17][2]);
                        sitldata.pitchRate = (DATA[16][0] * rad2deg);
                        sitldata.rollRate = (DATA[16][1] * rad2deg);
                        sitldata.yawRate = (DATA[16][2] * rad2deg);

                        sitldata.heading = (DATA[18][2]);
                    }

                    sitldata.airspeed = ((DATA[3][5] * .44704));

                    sitldata.latitude = (DATA[20][0]);
                    sitldata.longitude = (DATA[20][1]);
                    sitldata.altitude = (DATA[20][2] * ft2m);

                    sitldata.speedN = DATA[21][3];// (DATA[3][7] * 0.44704 * Math.Sin(sitldata.heading * deg2rad));
                    sitldata.speedE = -DATA[21][5];// (DATA[3][7] * 0.44704 * Math.Cos(sitldata.heading * deg2rad));

                    Matrix3 dcm = new Matrix3();
                    dcm.from_euler(sitldata.rollDeg * deg2rad, sitldata.pitchDeg * deg2rad, sitldata.yawDeg * deg2rad);

                    // rad = tas^2 / (tan(angle) * G)
                    float turnrad = (float)(((DATA[3][7] * 0.44704) * (DATA[3][7] * 0.44704)) / (float)(9.8f * Math.Tan(sitldata.rollDeg * deg2rad)));

                    float gload = (float)(1 / Math.Cos(sitldata.rollDeg * deg2rad)); // calculated Gs

                    // a = v^2/r
                    float centripaccel = (float)((DATA[3][7] * 0.44704) * (DATA[3][7] * 0.44704)) / turnrad;

                    Vector3 accel_body = dcm.transposed() * (new Vector3(0, 0, -9.8));

                    Vector3 centrip_accel = new Vector3(0, centripaccel * Math.Cos(sitldata.rollDeg * deg2rad), centripaccel * Math.Sin(sitldata.rollDeg * deg2rad));

                    accel_body -= centrip_accel;

                    sitldata.xAccel = DATA[4][5] * 9.8;
                    sitldata.yAccel = DATA[4][6] * 9.8;
                    sitldata.zAccel = (0 - DATA[4][4]) * 9.8;

                    //      Console.WriteLine(accel_body.ToString());
                    //      Console.WriteLine("        {0} {1} {2}",sitldata.xAccel, sitldata.yAccel, sitldata.zAccel);

                }
            }
        }
        public override void SendToSim()
        {
            roll_out = (float)MainV2.cs.hilch1 / rollgain;
            pitch_out = (float)MainV2.cs.hilch2 / pitchgain;
            throttle_out = ((float)MainV2.cs.hilch3) / throttlegain;
            rudder_out = (float)MainV2.cs.hilch4 / ruddergain;

            // Limit min and max
            roll_out = Constrain(roll_out, -1, 1);
            pitch_out = Constrain(pitch_out, -1, 1);
            rudder_out = Constrain(rudder_out, -1, 1);
            throttle_out = Constrain(throttle_out, 0, 1);

            // sending only 1 packet instead of many.

            byte[] Xplane = new byte[5 + 36 + 36];

            if (heli)
            {
                Xplane = new byte[5 + 36 + 36 + 36];
            }

            Xplane[0] = (byte)'D';
            Xplane[1] = (byte)'A';
            Xplane[2] = (byte)'T';
            Xplane[3] = (byte)'A';
            Xplane[4] = 0;

            Array.Copy(BitConverter.GetBytes((int)25), 0, Xplane, 5, 4); // packet index

            Array.Copy(BitConverter.GetBytes((float)throttle_out), 0, Xplane, 9, 4); // start data
            Array.Copy(BitConverter.GetBytes((float)throttle_out), 0, Xplane, 13, 4);
            Array.Copy(BitConverter.GetBytes((float)throttle_out), 0, Xplane, 17, 4);
            Array.Copy(BitConverter.GetBytes((float)throttle_out), 0, Xplane, 21, 4);

            Array.Copy(BitConverter.GetBytes((int)-999), 0, Xplane, 25, 4);
            Array.Copy(BitConverter.GetBytes((int)-999), 0, Xplane, 29, 4);
            Array.Copy(BitConverter.GetBytes((int)-999), 0, Xplane, 33, 4);
            Array.Copy(BitConverter.GetBytes((int)-999), 0, Xplane, 37, 4);

            // NEXT ONE - control surfaces

            Array.Copy(BitConverter.GetBytes((int)11), 0, Xplane, 41, 4); // packet index

            Array.Copy(BitConverter.GetBytes((float)(pitch_out * REV_pitch)), 0, Xplane, 45, 4); // start data
            Array.Copy(BitConverter.GetBytes((float)(roll_out * REV_roll)), 0, Xplane, 49, 4);
            Array.Copy(BitConverter.GetBytes((float)(rudder_out * REV_rudder)), 0, Xplane, 53, 4);
            Array.Copy(BitConverter.GetBytes((int)-999), 0, Xplane, 57, 4);

            Array.Copy(BitConverter.GetBytes((float)(roll_out * REV_roll * 0.5)), 0, Xplane, 61, 4);
            Array.Copy(BitConverter.GetBytes((int)-999), 0, Xplane, 65, 4);
            Array.Copy(BitConverter.GetBytes((int)-999), 0, Xplane, 69, 4);
            Array.Copy(BitConverter.GetBytes((int)-999), 0, Xplane, 73, 4);

            if (heli)
            {
                Array.Copy(BitConverter.GetBytes((float)(0)), 0, Xplane, 53, 4);


                int a = 73 + 4;
                Array.Copy(BitConverter.GetBytes((int)39), 0, Xplane, a, 4); // packet index
                a += 4;
                Array.Copy(BitConverter.GetBytes((float)(12 * collective_out)), 0, Xplane, a, 4); // main rotor 0 - 12
                a += 4;
                Array.Copy(BitConverter.GetBytes((float)(12 * rudder_out)), 0, Xplane, a, 4); // tail rotor -12 - 12
                a += 4;
                Array.Copy(BitConverter.GetBytes((int)-999), 0, Xplane, a, 4);
                a += 4;
                Array.Copy(BitConverter.GetBytes((int)-999), 0, Xplane, a, 4);
                a += 4;
                Array.Copy(BitConverter.GetBytes((int)-999), 0, Xplane, a, 4);
                a += 4;
                Array.Copy(BitConverter.GetBytes((int)-999), 0, Xplane, a, 4);
                a += 4;
                Array.Copy(BitConverter.GetBytes((int)-999), 0, Xplane, a, 4);
                a += 4;
                Array.Copy(BitConverter.GetBytes((int)-999), 0, Xplane, a, 4);
            }

            try
            {
                XplanesSEND.Send(Xplane, Xplane.Length);

            }
            catch (Exception e) { log.Info("Xplanes udp send error " + e.Message); }
        }
        public override void SendToAP(sitl_fdm sitldata)
        {
            TimeSpan gpsspan = DateTime.Now - lastgpsupdate;

            // add gps delay
            if (gpsspan.TotalMilliseconds >= GPS_rate)
            {
                lastgpsupdate = DateTime.Now;

                // save current fix = 3
                sitl_fdmbuffer[gpsbufferindex % sitl_fdmbuffer.Length] = sitldata;

                //                Console.WriteLine((gpsbufferindex % gpsbuffer.Length) + " " + ((gpsbufferindex + (gpsbuffer.Length - 1)) % gpsbuffer.Length));

                // return buffer index + 5 = (3 + 5) = 8 % 6 = 2
                oldgps = sitl_fdmbuffer[(gpsbufferindex + (sitl_fdmbuffer.Length - 1)) % sitl_fdmbuffer.Length];

                //comPort.sendPacket(oldgps);

                gpsbufferindex++;
            }

            MAVLink.mavlink_hil_state_t hilstate = new MAVLink.mavlink_hil_state_t();

            hilstate.time_usec = (UInt64)DateTime.Now.Ticks; // microsec

            hilstate.lat = (int)(oldgps.latitude * 1e7); // * 1E7
            hilstate.lon = (int)(oldgps.longitude * 1e7); // * 1E7
            hilstate.alt = (int)(oldgps.altitude * 1000); // mm

            //   Console.WriteLine(hilstate.alt);

            hilstate.pitch = (float)sitldata.pitchDeg * deg2rad; // (rad)
            hilstate.pitchspeed = (float)sitldata.pitchRate * deg2rad; // (rad/s)
            hilstate.roll = (float)sitldata.rollDeg * deg2rad; // (rad)
            hilstate.rollspeed = (float)sitldata.rollRate * deg2rad; // (rad/s)
            hilstate.yaw = (float)sitldata.yawDeg * deg2rad; // (rad)
            hilstate.yawspeed = (float)sitldata.yawRate * deg2rad; // (rad/s)

            hilstate.vx = (short)(sitldata.speedN * 100); // m/s * 100
            hilstate.vy = (short)(sitldata.speedE * 100); // m/s * 100
            hilstate.vz = 0; // m/s * 100

            hilstate.xacc = (short)(sitldata.xAccel * 1000); // (mg)
            hilstate.yacc = (short)(sitldata.yAccel * 1000); // (mg)
            hilstate.zacc = (short)(sitldata.zAccel * 1000); // (mg)

            MainV2.comPort.sendPacket(hilstate);

            MainV2.comPort.sendPacket(new MAVLink.mavlink_vfr_hud_t()
            {
                airspeed = (float)sitldata.airspeed
            });
        }
        public override void GetFromAP() { }

        void setupXplane()
        {
            // sending only 1 packet instead of many.

            byte[] Xplane = new byte[5 + 4 * 8];

            Xplane[0] = (byte)'D';
            Xplane[1] = (byte)'S';
            Xplane[2] = (byte)'E';
            Xplane[3] = (byte)'L';
            Xplane[4] = 0;

            if (xplane10)
            {
                int pos = 5;
                Xplane[pos] = 0x3;
                pos += 4;
                Xplane[pos] = 0x4;
                pos += 4;
                Xplane[pos] = 0x6;
                pos += 4;
                Xplane[pos] = 0x10;
                pos += 4;
                Xplane[pos] = 0x11;
                pos += 4;
                Xplane[pos] = 0x12;
                pos += 4;
                Xplane[pos] = 0x14;
                pos += 4;
                Xplane[pos] = 0x15;
                pos += 4;
            }
            else
            {
                int pos = 5;
                Xplane[pos] = 0x3;
                pos += 4;
                Xplane[pos] = 0x4;
                pos += 4;
                Xplane[pos] = 0x6;
                pos += 4;
                Xplane[pos] = 0x11;
                pos += 4;
                Xplane[pos] = 0x12;
                pos += 4;
                Xplane[pos] = 0x13;
                pos += 4;
                Xplane[pos] = 0x14;
                pos += 4;
                Xplane[pos] = 0x15;
                pos += 4;
            }

            try
            {
                XplanesSEND.Send(Xplane, Xplane.Length);

            }
            catch (Exception e) { log.Info("Xplanes udp send error " + e.Message); }

        }
    }
}