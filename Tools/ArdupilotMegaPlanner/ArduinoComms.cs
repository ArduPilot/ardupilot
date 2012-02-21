using System;
using System.Collections.Generic;
using System.Text;
using System.IO.Ports;
using System.IO;

namespace ArdupilotMega
{
    public delegate void ProgressEventHandler(int progress,string status);

    /// <summary>
    /// Arduino STK interface
    /// </summary>
    interface ArduinoComms
    {
         bool connectAP();
         bool keepalive();
         bool sync();
         byte[] download(short length);
         byte[] downloadflash(short length);
         bool setaddress(int address);
         bool upload(byte[] data, short startfrom, short length, short startaddress);
         bool uploadflash(byte[] data, int startfrom, int length, int startaddress);

         event ProgressEventHandler Progress;

        // from serialport class
         int BaudRate { get; set; }
         bool DtrEnable { get; set; }
         string PortName { get; set; }
         StopBits StopBits { get; set; }
         Parity Parity { get; set; }
         bool IsOpen { get; }
         void Open();
         void Close();
         int DataBits { get; set; }
    }
}
