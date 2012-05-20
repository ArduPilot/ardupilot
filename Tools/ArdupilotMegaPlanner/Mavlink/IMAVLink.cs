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
namespace ArdupilotMega
{
    public interface IMAVLink
    {
        // there is too much message intergration in the main code, so i dont think i will pursue this any further
        // if it an't broke dont fix it - mav09 and mav10 are compile time defined
        // i will probly revert this change at some point, unless someone else has a good idea.

        // Fields

        byte[][] packets { get; set; }


        double[] packetspersecond { get; set; }

        Hashtable param { get; set; }

        BinaryWriter rawlogfile { get; set; }



        byte sysid { get; set; }
        Subject<int> WhenPacketLost { get; set; }
        Subject<int> WhenPacketReceived { get; set; }
        PointLatLngAlt[] wps { get; set; }


        ArdupilotMega.MAVLink.MAV_TYPE aptype { get; set; }
        ICommsSerial BaseStream { get; set; }
        int bps { get; set; }


        DateTime bpstime { get; set; }
        byte compid { get; set; }

        bool debugmavlink { get; set; }


        DateTime lastlogread { get; set; }
        DateTime lastvalidpacket { get; set; }

        BinaryWriter logfile { get; set; }
        BinaryReader logplaybackfile { get; set; }
        bool logreadmode { get; set; }

        // Events

        event EventHandler ParamListChanged;

        // Methods


        void Close();
        object DebugPacket(byte[] datin);
        object DebugPacket(byte[] datin, bool PrintToConsole);
        object DebugPacket(byte[] datin, ref string text);
        object DebugPacket(byte[] datin, ref string text, bool PrintToConsole, string delimeter = " ");

        // mav 09
        bool doAction(object actionid);
        // mac 10
        bool doCommand(ArdupilotMega.MAVLink.MAV_CMD actionid, float p1, float p2, float p3, float p4, float p5, float p6, float p7);

        void setAPType();


        PointLatLngAlt getFencePoint(int no, ref int total);

        void getParamList();

        Locationwp getWP(ushort index);
        byte getWPCount();


        // static void modifyParamForDisplay(bool fromapm, string paramname, ref float value);
        void Open();
        void Open(bool getparams);



        byte[] readPacket();
        void requestDatastream(byte id, byte hzrate);
        void sendPacket(object indata);
        bool setFencePoint(byte index, PointLatLngAlt plla, byte fencepointcount);
        void setMode(string modein);
        void setMountConfigure(ArdupilotMega.MAVLink.MAV_MOUNT_MODE mountmode, bool stabroll, bool stabpitch, bool stabyaw);
        void setMountControl(double pa, double pb, double pc, bool islatlng);
        bool setParam(string paramname, float value);
        void setWP(Locationwp loc, ushort index, ArdupilotMega.MAVLink.MAV_FRAME frame, byte current);
        void setWPACK();
        bool setWPCurrent(ushort index);
        void setWPTotal(ushort wp_total);
        void stopall(bool forget);
        bool Write(string line);

        // Properties

        IObservable<int> BytesReceived { get; }
        IObservable<int> BytesSent { get; }

    }
}