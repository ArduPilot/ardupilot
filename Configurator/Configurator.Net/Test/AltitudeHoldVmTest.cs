using ArducopterConfigurator.PresentationModels;
using NUnit.Framework;

namespace ArducopterConfiguratorTest
{
//    [TestFixture]
//    public class AltitudeHoldVmTest : VmTestBase<AltitudeHoldConfigVm>
//    {
//
//        [SetUp]
//        public void Setup()
//        {
//            sampleLineOfData = "0.800,0.200,0.300";
//            getCommand = "F";
//            setCommand = "E";
//
//            _mockComms = new MockComms();
//            _mockComms.Connect();
//            _vm = new AltitudeHoldConfigVm(_mockComms);
//        }
//
//
//        [Test]
//        // For whatever reason, for Altitude the properties are sent in P, D ,I
//        // order, but received in P,I,D order
//        public void UpdateStringSentIsCorrect()
//        {
//            _vm.P = 1.0F;
//            _vm.I = 2.0F;
//            _vm.D = 3.0F;
//
//            _vm.UpdateCommand.Execute(null);
//
//            Assert.AreEqual(1, _mockComms.SentItems.Count);
//            Assert.AreEqual("E1;3;2", _mockComms.SentItems[0]);
//        }
//
//        [Test]
//        // For whatever reason, for Altitude the properties are sent in P, D ,I
//        // order, but received in P,I,D order
//        public void UpdateStringReceivedPopulatesValuesCorrectly()
//        {
//            _vm.Activate();
//            _mockComms.FireLineRecieve(sampleLineOfData);
//
//            Assert.AreEqual(0.8f, _vm.P);
//            Assert.AreEqual(0.2f, _vm.I);
//            Assert.AreEqual(0.3f, _vm.D);
//        }
//
//
//    }
}
