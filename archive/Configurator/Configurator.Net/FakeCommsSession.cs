using System;
using System.Collections.Generic;
using System.ComponentModel;
using System.Windows.Forms;

namespace ArducopterConfigurator
{
    /// <summary>
    /// Fake implementation of Comms + APM for testing without a device connected
    /// </summary>
    public class FakeCommsSession : IComms
    {
        private bool _connected;
        private string _jabberData;
        private readonly Timer _jabberTimer;

        #region Implementation of IComms

        public FakeCommsSession()
        {
            _jabberTimer = new Timer();
            _jabberTimer.Interval = 1000;
            _jabberTimer.Tick += timer_Tick;
        }

        public event Action<string> LineOfDataReceived;
        
        public string CommPort { get; set; }

        public bool IsConnected
        {
            get { return _connected; }
        }

        public int BaudRate { get; set; }
     

        public IEnumerable<string> ListCommPorts()
        {
            return new[] {"FakePort1", "FakePort2"};
        }

        public void Send(string stringSent)
        {
            if (!_connected)
                throw new InvalidOperationException("Not Connected");

            if (stringSent == "!")
                ReturnData("Fake");

            if (stringSent == "D")  // position
                ReturnData("0.015,0.005,0.010,0.015,0.005,0.010,22.000,0.870");
              if (stringSent == "B")  // stable
                ReturnData("1.950,0.100,0.200,1.950,0.300,0.400,3.200,0.500,0.600,0.320,1.00");
               if (stringSent == "P")  // acro
                ReturnData("3.950,0.100,0.000,0.000,0.300,0.400,3.200,0.500,0.600,0.320");
              if (stringSent == "F")  // alti
                  ReturnData("0.800,0.200,0.300");
            if (stringSent == "J")  // calib
                ReturnData("0.100,0.200,0.300,0.400,0.500,0.600");



            
         
           
            if (stringSent == "S")
            {
                // Loop Time = 2
                // Roll Gyro Rate = -10
                // Pitch Gyro Rate = 3
                // Yaw Gyro Rate = -2
                // Throttle Output = 1011
                // Roll PID Output = 1012
                // Pitch PID Output = 1002
                // Yaw PID Output 1000
                // Front Motor Command = 1001 PWM output sent to right motor (ranges from 1000-2000)
                // Rear Motor Command 1003
                // Right Motor Command = 1002
                // Left Motor Command = 1004
                // then adc 4,3, and 5
                // mag heading float * 3
                // mag heading int * 3

                _jabberData = "2,-10,3,-2,1011,1012,1002,1000,1001,1200,1003,1400,1000,1000,1000,1.000,1.000,1.000,0,0,0";
                StartJabber();
            }
            if (stringSent == "X")
                StopJabber();
        }

        private void StopJabber()
        {
            _jabberTimer.Stop();
        }

        private void StartJabber()
        {
            _jabberTimer.Start();
        }

        void timer_Tick(object sender, EventArgs e)
        {
            ReturnData(_jabberData);
        }

   
        private void ReturnData(string data)
        {
            if (LineOfDataReceived != null)
                LineOfDataReceived(data + "\n");
        }

        public bool Connect()
        {
            if (_connected)
                throw new InvalidOperationException("Already Connected");
            _connected = true;
            return true;
        }

        public bool DisConnect()
        {
            if (!_connected)
                throw new InvalidOperationException("Already DisConnected");
            _connected = false;
            return true;
        }

        #endregion
    }
}