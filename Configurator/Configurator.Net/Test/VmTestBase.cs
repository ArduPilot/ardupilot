using ArducopterConfigurator;
using NUnit.Framework;

namespace ArducopterConfiguratorTest
{
//    public abstract class VmTestBase<T> where T : MonitorVm
//    {
//        protected T _vm;
//        protected MockComms _mockComms;
//        protected string sampleLineOfData; // should be taken from a real APM if possible
//        protected string getCommand;
//        protected string setCommand;
//
//        [Test]
//        public void ActivateSendsCorrectCommand()
//        {
//            _vm.Activate();
//            Assert.AreEqual(1, _mockComms.SentItems.Count);
//            Assert.AreEqual(getCommand, _mockComms.SentItems[0]);
//        }
//
//        [Test]
//        public void ReceivedDataIgnoredWhenNotActive()
//        {
//            bool inpcFired = false;
//            _vm.PropertyChanged += delegate { inpcFired = true; };
//
//            _mockComms.FireLineRecieve(sampleLineOfData);
//            Assert.False(inpcFired);
//        }
//
//        [Test]
//        public void ReceivedDataIgnoredAfterDeActive()
//        {
//            _vm.Activate();
//            _mockComms.FireLineRecieve(sampleLineOfData);
//            _vm.DeActivate();
//            _mockComms.FireLineRecieve(sampleLineOfData);
//            bool inpcFired = false;
//            _vm.PropertyChanged += delegate { inpcFired = true; };
//
//            Assert.False(inpcFired);
//        }
//
//        [Test]
//        public void UpdateStringReceivedPopulatesValues()
//        {
//            bool inpcFired = false;
//            _vm.PropertyChanged += delegate { inpcFired = true; };
//
//            _vm.Activate();
//            _mockComms.FireLineRecieve(sampleLineOfData);
//
//            Assert.True(inpcFired);
//        }
//    }
}