using ArducopterConfigurator.PresentationModels;
using NUnit.Framework;

namespace ArducopterConfiguratorTest
{
//    [TestFixture]
//    public class TransmitterChannelsVmTests
//    {
//        private MockComms _mockComms;
//        private TransmitterChannelsVm _vm;
//
//        [SetUp]
//        public void Setup()
//        {
//            _mockComms = new MockComms();
//            _mockComms.Connect();
//            _vm = new TransmitterChannelsVm(_mockComms);
//           
//        }
//
//        [Test]
//        public void SendsCorrectCommandOnActivate()
//        {
//            _vm.Activate();
//            Assert.AreEqual(1,_mockComms.SentItems.Count);
//            Assert.AreEqual("U",_mockComms.SentItems[0]);
//        }
//
//        [Test]
//        public void SendsCorrectCommandOnDeActivate()
//        {
//            _vm.Activate();
//            _vm.DeActivate();
//
//            Assert.AreEqual(2, _mockComms.SentItems.Count);
//            Assert.AreEqual("X", _mockComms.SentItems[1]);
//        }
//
//        [Test]
//        public void ValuesAreSet()
//        {
//            _vm.Activate();
//            // What do the MID values do?
//            //1403,1620,1523,1501,1900,1950,0,0,0
//            // Aileron,Elevator,Yaw,Throttle,AUX1 (Mode),AUX2 ,Roll MID value,Pitch MID value,Yaw MID Value
//
//            var sampleData = "1403,1620,1523,1501,1900,1950,0,0,0";
//            _mockComms.FireLineRecieve(sampleData);
//            Assert.AreEqual(1403, _vm.Roll);
//            Assert.AreEqual(1620, _vm.Pitch);
//            Assert.AreEqual(1523, _vm.Yaw);
//            Assert.AreEqual(1501, _vm.Throttle);
//            Assert.AreEqual(1900, _vm.Mode);
//            Assert.AreEqual(1950, _vm.Aux);
//        }
//
//
//        [Test]
//        public void MaximumsAndMinimumsAreSet()
//        {
//            _vm.Activate();
//            // What do the MID values do?
//            //1403,1620,1523,1501,1900,1950,0,0,0
//            // Aileron,Elevator,Yaw,Throttle,AUX1 (Mode),AUX2 ,Roll MID value,Pitch MID value,Yaw MID Value
//           
//            var sampleData = "1403,1620,1523,1501,1900,1950,0,0,0";
//            _mockComms.FireLineRecieve(sampleData);
//            _vm.ResetCommand.Execute(null);
//            _mockComms.FireLineRecieve(sampleData);
//
//            Assert.AreEqual(1403,_vm.Roll);
//            Assert.AreEqual(1403,_vm.RollMin);
//            Assert.AreEqual(1403,_vm.RollMax);
//        }
//        
//    }
}
/*
roll_slope,ch_roll_offset,
ch_pitch_slope,ch_pitch_offset
ch_yaw_slope,ch_yaw_offset,
ch_throttle_slope,ch_throttle_offset
ch_aux_slope,ch_aux_offset
ch_aux2_slope,ch_aux2_offset
*/
// 1.20,-327.73,1.20,-328.92,1.21,-344.66,1.21,-343.83,1.79,-1225.81,1.79,-1227.60

