using System;

namespace ArducopterConfigurator.PresentationModels
{
    public class PositionHoldConfigVm : ConfigWithPidsBase, IPresentationModel, ItalksToApm
    {
        public PositionHoldConfigVm()
        {
            PropsInUpdateOrder = new[] 
                                     { 
                                         "RollP", 
                                         "RollI", 
                                         "RollD", 
                                         "PitchP", 
                                         "PitchI", 
                                         "PitchD", 
                                         "MaximumAngle", 
                                         "GeoCorrectionFactor", 
                                     };

            RefreshCommand = new DelegateCommand(_ => RefreshValues());
            UpdateCommand = new DelegateCommand(_ => UpdateValues());
        }

        public float MaximumAngle { get;  set; }

        public float GeoCorrectionFactor { get; set; }

        public ICommand RefreshCommand { get; private set; }

        public ICommand UpdateCommand { get; private set; }

        //Send an 'C' followed by numeric values to transmit user defined roll and pitch PID values for gyro stabilized flight control. Each value is separated by a semi-colon in the form:
        //C[P GPS ROLL];[I GPS ROLL];[D GPS ROLL];[P GPS PITCH];[I GPS PITCH];[D GPS PITCH];[GPS MAX ANGLE];[GEOG Correction factor]
        public void UpdateValues()
        {
            if (sendTextToApm != null)
                sendTextToApm(this,new sendTextToApmEventArgs(ComposePropsWithCommand("C")));
        }

        public void RefreshValues()
        {
            if (sendTextToApm != null)
                sendTextToApm(this,new sendTextToApmEventArgs("D"));
        }

        public string Name
        {
            get { return "Position Hold"; }
        }

        public void Activate()
        {
            RefreshValues();
        }

        public void DeActivate()
        {
            
        }

        public void handleLineOfText(string strRx)
        {
            PopulatePropsFromUpdate(strRx,true);
            if (updatedByApm != null)
                updatedByApm(this,EventArgs.Empty);

        }

        public event EventHandler<sendTextToApmEventArgs> sendTextToApm;
        
        public event EventHandler updatedByApm;
    }
}