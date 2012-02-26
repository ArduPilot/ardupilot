using System;
using System.Collections.Generic;
using System.Text;
using System.IO.Ports;
using System.IO;

namespace ArdupilotMega
{
    class SerialPort : System.IO.Ports.SerialPort,ICommsSerial
    {
        public new void Open()
        {
            if (base.IsOpen)
                return;

            base.Open();
        }

        public void toggleDTR()
        {
            base.DtrEnable = false;
            base.RtsEnable = false;

            System.Threading.Thread.Sleep(50);

            base.DtrEnable = true;
            base.RtsEnable = true;

            System.Threading.Thread.Sleep(50);
        }
    }
}
