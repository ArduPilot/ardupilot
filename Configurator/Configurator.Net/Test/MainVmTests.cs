using System;
using System.Collections.Generic;
using System.Text;
using ArducopterConfigurator;
using ArducopterConfigurator.PresentationModels;
using NUnit.Framework;

namespace ArducopterConfiguratorTest
{
    public class MockComms : IComms
    {
        public event Action<string> LineOfDataReceived;
        public string CommPort { get; set; }
        public List<string> SentItems = new List<string>();

        public bool IsConnected
        {
            get { throw new NotImplementedException(); }
        }

        public IEnumerable<string> ListCommPorts()
        {
            throw new NotImplementedException();
        }

        public void Send(string send)
        {
            SentItems.Add(send);
        }

        public bool Connect()
        {
            return true;
        }

        public bool DisConnect()
        {
            return true;
        }

        public void FireLineRecieve(string s)
        {
            if (LineOfDataReceived != null)
                LineOfDataReceived(s);
        }
    }

    [TestFixture]
    public class MainVmTests
    {
        private MockComms _mockComms;
        private MainVm _vm;

        [SetUp]
        public void Setup()
        {
            _mockComms = new MockComms();
            _vm = new MainVm(_mockComms);
        }

        [Test]
        public void StateInitiallyDisconnected()
        {
            Assert.AreEqual(MainVm.SessionStates.Disconnected, _vm.ConnectionState);
        }


    }
}

