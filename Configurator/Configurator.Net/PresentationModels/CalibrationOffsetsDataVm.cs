using System;
using System.Collections.Generic;
using System.Diagnostics;

namespace ArducopterConfigurator.PresentationModels
{
    public class CalibrationOffsetsDataVm : MonitorVm
    {
        public CalibrationOffsetsDataVm(IComms sp) : base(sp)
        {
            PropsInUpdateOrder = new[] 
            { 
                "GyroRollOffset", 
                "GyroPitchOffset", 
                "GyroYawOffset", 
                "AccelRollOffset", 
                "AccelPitchOffset", 
                "AccelZOffset" 
            };

            RefreshCommand = new DelegateCommand(_ => RefreshValues());
            UpdateCommand = new DelegateCommand(_ => UpdateValues());
        }

        public ICommand RefreshCommand { get; private set; }

        public ICommand UpdateCommand { get; private set; }

      

        public int GyroRollOffset { get; set; }
        public int GyroPitchOffset { get; set; }
        public int GyroYawOffset { get; set; }

        public int AccelRollOffset { get; set; }
        public int AccelPitchOffset { get; set; }
        public int AccelZOffset { get; set; }

        protected override void OnActivated()
        {
            RefreshValues();
        }

        private void RefreshValues()
        {
            SendString("J");
        }

        public void UpdateValues()
        {
            SendPropsWithCommand("I");
        }


        protected override void OnStringReceived(string strReceived)
        {
            PopulatePropsFromUpdate(strReceived,true);
        }

        public override string Name
        {
            get { return "Calibration"; }
        }
    }
}