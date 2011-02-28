using System;
using System.Collections.Generic;
using ArducopterConfigurator;

namespace ArducopterConfiguratorTest
{
    public class MockComms : IComms
    {
        public event Action<string> LineOfDataReceived;
        public string CommPort { get; set; }
        public List<string> SentItems = new List<string>();
        private bool _isConnected;

        public bool IsConnected
        {
            get { return _isConnected; }
        }

        public int BaudRate
        {
            get { throw new System.NotImplementedException(); }
            set { throw new System.NotImplementedException(); }
        }

        public IEnumerable<string> ListCommPorts()
        {
            return new[] {"MockComport1"};
        }

        public void Send(string send)
        {
            SentItems.Add(send);
        }

        public bool Connect()
        {
            _isConnected = true;
            return true;
        }

        public bool DisConnect()
        {
            _isConnected = false;
            return true;
        }

        public void FireLineRecieve(string s)
        {
            if (LineOfDataReceived != null)
                LineOfDataReceived(s);
        }
    }
}