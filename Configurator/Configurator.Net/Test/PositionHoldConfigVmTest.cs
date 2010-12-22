using ArducopterConfigurator.PresentationModels;
using NUnit.Framework;

namespace ArducopterConfiguratorTest
{
    [TestFixture]
    public class PositionHoldConfigVmTest : VmTestBase<PositionHoldConfigVm>
    {

        [SetUp]
        public void Setup()
        {
            sampleLineOfData = "0.015,0.005,0.010,0.015,0.005,0.010,22.000,0.870";
            getCommand = "D";
            setCommand = "C";

            _fakeComms = new FakeComms();
            _vm = new PositionHoldConfigVm(_fakeComms);
        }

    }
}