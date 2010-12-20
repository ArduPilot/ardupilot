using ArducopterConfigurator.PresentationModels;
using NUnit.Framework;

namespace ArducopterConfiguratorTest
{
    [TestFixture]
    public class MotorCommandsVmTest
    {
        private FakeComms _fakeComms;
        private MotorCommandsVm _vm;

        [SetUp]
        public void Setup()
        {
            _fakeComms = new FakeComms();
            _vm = new MotorCommandsVm(_fakeComms);
        }

       


    }
}