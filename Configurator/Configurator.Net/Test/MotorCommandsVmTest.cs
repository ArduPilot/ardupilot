using ArducopterConfigurator.PresentationModels;
using NUnit.Framework;

namespace ArducopterConfiguratorTest
{
    [TestFixture]
    public class MotorCommandsVmTest
    {
        private MockComms _mockComms;
        private MotorCommandsVm _vm;

        [SetUp]
        public void Setup()
        {
            _mockComms = new MockComms();
            _vm = new MotorCommandsVm(_mockComms);
        }

       


    }
}