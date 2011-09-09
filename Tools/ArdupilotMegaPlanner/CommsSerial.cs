using System;
using System.Collections.Generic;
using System.Text;
using System.IO.Ports;
using System.IO;

namespace ArdupilotMega
{
    public interface ICommsSerial
    {
        // from serialport class
         int BaudRate { get; set; }
         bool DtrEnable { get; set; }
         string PortName { get; set; }
         StopBits StopBits { get; set; }
         Parity Parity { get; set; }
         bool IsOpen { get; }
         void Open();
         void Open(bool getparams);
         void Close();
         int DataBits { get; set; }
    }
}
