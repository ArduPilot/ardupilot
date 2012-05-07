using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;

namespace ArdupilotMega
{
    class MainV2
    {
        public static portproxy comPort = new portproxy();
    }

    class portproxy
    {
        public ArdupilotMega.Comms.SerialPort BaseStream = new ArdupilotMega.Comms.SerialPort();
    }
}
