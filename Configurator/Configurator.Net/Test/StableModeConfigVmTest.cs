using ArducopterConfigurator.PresentationModels;
using NUnit.Framework;

namespace ArducopterConfiguratorTest
{
    [TestFixture]
    public class StableModeConfigVmTest : VmTestBase<StableModeConfigVm>
    {

        [SetUp]
        public void Setup()
        {
            sampleLineOfData = "1.950,0.100,0.200,1.950,0.300,0.400,3.200,0.500,0.600,0.320,1.00";
            getCommand = "B";
            setCommand = "A";

            _fakeComms = new FakeComms();
            _vm = new StableModeConfigVm(_fakeComms);
        }

        [Test]
        public void UpdateStringSentIsCorrect()
        {
            _vm.PitchP = 1.0F;
            _vm.PitchI = 2.0F;
            _vm.PitchD = 3.0F;
            _vm.RollP = 5.0F;
            _vm.RollI = 6.0F;
            _vm.RollD = 7.0F;
            _vm.YawP = 8.0F;
            _vm.YawI = 9.0F;
            _vm.YawD = 10.0F;
            _vm.MagnetometerEnable = true;
            _vm.KPrate = 4.0F;

            _vm.UpdateCommand.Execute(null);


            Assert.AreEqual(1, _fakeComms.SentItems.Count);

            //A[KP Quad Roll];[KI Quad Roll];[KP RATE ROLL];
            // [KP Quad Pitch];[KI Quad Pitch];[KP RATE PITCH];
            // [KP Quad Yaw];[KI Quad Yaw];[KP Rate Yaw];
            // [KP Rate];[Magneto]
            Assert.AreEqual("A5;6;7;1;2;3;8;9;10;4;1", _fakeComms.SentItems[0]);
        }

        [Test]
        public void UpdateStringReceivedPopulatesValuesCorrectly()
        {
            _fakeComms.FireLineRecieve("B5;6;7;1;2;3;8;9;10;4;1");

            Assert.AreEqual(1.0f, _vm.PitchP);
            Assert.AreEqual(2f, _vm.PitchI);
            Assert.AreEqual(3f, _vm.PitchD);
            Assert.AreEqual(5f, _vm.RollP);
            Assert.AreEqual(6f, _vm.RollI);
            Assert.AreEqual(7f, _vm.RollD);
            Assert.AreEqual(8f, _vm.YawP);
            Assert.AreEqual(9f, _vm.YawI);
            Assert.AreEqual(10f, _vm.YawD);
            Assert.AreEqual(1f, _vm.MagnetometerEnable);
            Assert.AreEqual(4f, _vm.KPrate);
        }
    }
}