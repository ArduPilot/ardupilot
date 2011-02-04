using System.Text;
using ArducopterConfigurator.PresentationModels;
using NUnit.Framework;

namespace ArducopterConfiguratorTest
{
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

