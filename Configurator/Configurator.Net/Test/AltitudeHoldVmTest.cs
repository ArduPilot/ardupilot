using ArducopterConfigurator.PresentationModels;
using NUnit.Framework;

namespace ArducopterConfiguratorTest
{
    [TestFixture]
    public class AltitudeHoldVmTest
    {
        private FakeComms _fakeComms;
        private AltitudeHoldConfigVm _vm;

        [SetUp]
        public void Setup()
        {
            _fakeComms = new FakeComms();
            _vm = new AltitudeHoldConfigVm(_fakeComms);
        }

        [Test]
        public void UpdateStringSentIsCorrect()
        {
            _vm.P = 1.0F;
            _vm.I = 2.0F;
            _vm.D = 3.0F;

            _vm.UpdateCommand.Execute(null);

            Assert.AreEqual(1, _fakeComms.SentItems.Count);
            
            Assert.AreEqual("E1;3;2", _fakeComms.SentItems[0]);

        }

        [Test]
        // For whatever reason, for Altitude the properties are sent in P, D ,I
        // order, but received in P,I,D order
        public void UpdateStringReceivedPopulatesValuesCorrectly()
        {
            _fakeComms.FireLineRecieve("F1;2;3");

            Assert.AreEqual(1f, _vm.P);
            Assert.AreEqual(2f, _vm.I);
            Assert.AreEqual(3f, _vm.D);
        }


    }
}