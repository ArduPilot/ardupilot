using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Collections;
using System.IO;

namespace ArdupilotMega
{
    partial interface  Protocol
    {
        byte[][] packets { get; set; }

        PointLatLngAlt[] wps { get; set; }

        ICommsSerial BaseStream { get; set; }

        int bps { get; set; }
        bool debugmavlink { get; set; }

        DateTime lastvalidpacket {get;set;}

        bool logreadmode { get; set; }
        DateTime lastlogread { get; set; }
        BinaryReader logplaybackfile { get; set; }
        BinaryWriter logfile { get; set; }

        byte sysid { get; set; }
        byte compid { get; set; }
        Hashtable param { get; set; }

        void UpdateCurrentSettings(ref CurrentState cs);

        void Close();

        void Open();
        void Open(bool getparams);

        void sendPacket(object indata);
        bool Write(string line);
        bool setParam(string paramname, float value);
        Hashtable getParamList();
        void modifyParamForDisplay(bool fromapm, string paramname, ref float value);

        void stopall(bool forget);
        void setWPACK();
        bool setWPCurrent(ushort index);
        bool doAction(byte actionid); // MAV_ACTION

        void requestDatastream(byte id, byte hzrate);
        byte getWPCount();
        Locationwp getWP(ushort index);
        object DebugPacket(byte[] datin);
        object DebugPacket(byte[] datin, ref string text);
        void setWPTotal(ushort wp_total);
        void setWP(Locationwp loc, ushort index, byte frame, byte current); //MAV_FRAME

        void setMountConfigure(byte mountmode, bool stabroll, bool stabpitch, bool stabyaw); //MAV_MOUNT_MODE
        void setMountControl(double pa, double pb, double pc, bool islatlng);
        void setMode(string modein);
        byte[] readPacket();

        bool translateMode(string modein, ref object navmode, ref object mode);
    }
}
