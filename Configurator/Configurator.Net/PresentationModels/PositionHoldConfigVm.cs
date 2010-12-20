namespace ArducopterConfigurator.PresentationModels
{
    public class PositionHoldConfigVm : ConfigWithPidsBase
    {
        public PositionHoldConfigVm(IComms sp)
            : base(sp)
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
            SendPropsWithCommand("C");
        }

        public void RefreshValues()
        {
            SendString("D");
        }

        protected override void OnActivated()
        {
            RefreshValues();
        }

        public override string Name
        {
            get { return "Position Hold"; }
        }
    }
}