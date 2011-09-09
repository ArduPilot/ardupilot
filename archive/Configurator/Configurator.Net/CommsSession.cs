using System;
using System.Collections.Generic;
using System.ComponentModel;
using System.IO.Ports;
using System.Text;

namespace ArducopterConfigurator
{
    public interface IComms
    {
        event Action<string> LineOfDataReceived;
        string CommPort { get; set; }
        bool IsConnected { get; }
        int BaudRate { get; set; }
        IEnumerable<string> ListCommPorts();
        void Send(string send);
        bool Connect();
        bool DisConnect();
    }

    /// <summary>
    /// Represents a session of communication with the Arducopter
    /// </summary>
    /// <remarks>
    /// Looks after connection state etc
    /// </remarks>
    public class CommsSession : IComms
    {
        private readonly SerialPort _sp;
        private BackgroundWorker _bgWorker;

        public event Action<string> LineOfDataReceived;

        public CommsSession()
        {
              _sp = new SerialPort();
        }

        public string CommPort { get; set; }

        public int BaudRate { get; set; }
     
        public bool Connect()
        {
            _sp.BaudRate = BaudRate;
            _sp.PortName = CommPort;
            _sp.NewLine = "\n";
            _sp.Handshake = Handshake.None;

            try
            {
                _sp.Open();
                _sp.ReadTimeout = 50000;
                

                // start the reading BG thread
                _bgWorker = new BackgroundWorker();
                _bgWorker.DoWork += bgWorker_DoWork;
                _bgWorker.WorkerReportsProgress = true;
                _bgWorker.WorkerSupportsCancellation = true;
                _bgWorker.ProgressChanged += bgWorker_ProgressChanged;
                _bgWorker.RunWorkerAsync();
            }
            catch (Exception ex)
            {
                Error = ex.Message;
                return false;
            }
            return true;
        }

     

        public bool DisConnect()
        {
            _bgWorker.CancelAsync();
            
            _sp.Close();
            return true;
        }

        void bgWorker_ProgressChanged(object sender, ProgressChangedEventArgs e)
        {
            // Thanks to BG worker, this should be raised on the UI thread
            var lineReceived = e.UserState as string;

            // TODO: POSSIBLE MONO ISSUE
            // Weird thing happening with the serial port on mono; sometimes the first
            // char is nulled. Work around for now is to drop it, and just send the remaining 
            // chars up the stack. This is of course exteremely bogus
            if (lineReceived[0] == 0)
            {
                Console.WriteLine("Warning - received null first character. Dropping.");
                lineReceived = lineReceived.Substring(1);
            }

            //Console.WriteLine("Processing Update: " + lineReceived);
            
            if (LineOfDataReceived != null)
                 LineOfDataReceived(lineReceived);
        }

        void bgWorker_DoWork(object sender, DoWorkEventArgs e)
        {
            while (!_bgWorker.CancellationPending)
            {
                try
                {
                    var line = _sp.ReadLine();
                    _bgWorker.ReportProgress(0, line);
                }
                catch(TimeoutException)
                {
                    // continue
                }
                catch(System.IO.IOException) // when the port gets killed
                {}
                catch(ObjectDisposedException)
                {}
            }
        }

        private string Error { get; set;}


        public bool IsConnected
        {
            get { return _sp.IsOpen; }
        }

      

        public IEnumerable<string> ListCommPorts()
        {
            return SerialPort.GetPortNames();
        }

        public void Send(string send)
        {
            if (_sp.IsOpen)
                _sp.Write(send);
        }
    }
}
