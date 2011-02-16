using System;
using System.Collections.Generic;
using System.Diagnostics;

namespace ArducopterConfigurator.PresentationModels
{
    public class CalibrationOffsetsDataVm : NotifyProperyChangedBase, IPresentationModel
    {
        private readonly string[] PropsInUpdateOrder = new[] 
            { 
                "GyroRollOffset", 
                "GyroPitchOffset", 
                "GyroYawOffset", 
                "AccelRollOffset", 
                "AccelPitchOffset", 
                "AccelZOffset" 
            };

        public CalibrationOffsetsDataVm() 
        {
          

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
            {
                var apmString = PropertyHelper.ComposePropValuesWithCommand(this, PropsInUpdateOrder, "I");
                sendTextToApm(this, new sendTextToApmEventArgs(apmString));
            }
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
            PropertyHelper.PopulatePropsFromUpdate(this, PropsInUpdateOrder, strRx, true);

            if (updatedByApm != null)
                updatedByApm(this, EventArgs.Empty);
        }

        public event EventHandler<sendTextToApmEventArgs> sendTextToApm;
    }
}