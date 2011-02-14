using System;
using System.Collections.Generic;
using System.Diagnostics;

namespace ArducopterConfigurator.PresentationModels
{
    public class CalibrationOffsetsDataVm : VmBase, IPresentationModel
    {
        public CalibrationOffsetsDataVm() 
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

        public float GyroRollOffset { get; set; }
        public float GyroPitchOffset { get; set; }
        public float GyroYawOffset { get; set; }

        public float AccelRollOffset { get; set; }
        public float AccelPitchOffset { get; set; }
        public float AccelZOffset { get; set; }

        private void RefreshValues()
        {
            if (sendTextToApm != null)
                sendTextToApm(this, new sendTextToApmEventArgs("J"));
        }

        public void UpdateValues()
        {
            if (sendTextToApm != null)
                sendTextToApm(this, new sendTextToApmEventArgs(ComposePropsWithCommand("I")));
        }

        public string Name
        {
            get { return "Calibration"; }
        }

        public void Activate()
        {
            RefreshValues();
        }

        public void DeActivate()
        {
        }

        public event EventHandler updatedByApm;

        public void handleLineOfText(string strRx)
        {
            PopulatePropsFromUpdate(strRx, true);
        }

        public event EventHandler<sendTextToApmEventArgs> sendTextToApm;
    }
}