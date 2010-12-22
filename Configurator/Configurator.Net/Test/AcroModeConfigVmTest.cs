using ArducopterConfigurator.PresentationModels;
using NUnit.Framework;

namespace ArducopterConfiguratorTest
{
    [TestFixture]
    public class AcroModeConfigVmTest : VmTestBase<AcroModeConfigVm>
    {
        [SetUp]
        public void Setup()
        {
            sampleLineOfData = "1.950,0.100,0.200,1.950,0.300,0.400,3.200,0.500,0.600,0.320";
            getCommand = "P";
            setCommand = "O";

            _fakeComms = new FakeComms();
            _vm = new AcroModeConfigVm(_fakeComms);
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
            _vm.TransmitterFactor = 4.0F;

            _vm.UpdateCommand.Execute(null);

            Assert.AreEqual(1, _fakeComms.SentItems.Count);
            Assert.AreEqual("O5;6;7;1;2;3;8;9;10;4", _fakeComms.SentItems[0]);
        }
    }


    [TestFixture]
        public class AltitudeHoldVmTest : VmTestBase<AltitudeHoldConfigVm>
        {

            [SetUp]
            public void Setup()
            {
                sampleLineOfData = "0.800,0.200,0.300";
                getCommand = "F";
                setCommand = "E";

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
                _vm.Activate();
                _fakeComms.FireLineRecieve(sampleLineOfData);

                Assert.AreEqual(0.8f, _vm.P);
                Assert.AreEqual(0.2f, _vm.I);
                Assert.AreEqual(0.3f, _vm.D);
            }


        }
    }
