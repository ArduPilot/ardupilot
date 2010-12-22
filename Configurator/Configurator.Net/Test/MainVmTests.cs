using System;
using System.Collections.Generic;
using System.Text;
using ArducopterConfigurator;
using ArducopterConfigurator.PresentationModels;
using NUnit.Framework;

namespace ArducopterConfiguratorTest
{
    public class FakeComms : IComms
    {
        public event Action<string> LineOfDataReceived;
        public string CommPort { get; set; }
        public List<string> SentItems = new List<string>();

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
        private FakeComms _fakeComms;
        private MainVm _vm;

        [SetUp]
        public void Setup()
        {
            _fakeComms = new FakeComms();
            _vm = new MainVm(_fakeComms);
        }

        [Test]
        public void StateInitiallyDisconnected()
        {
            Assert.AreEqual(MainVm.SessionStates.Disconnected, _vm.ConnectionState);
        }


    }
}

