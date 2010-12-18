namespace ArducopterConfigurator.PresentationModels
{
    public class StableModeConfigVm : ConfigWithPidsBase
    {
        public StableModeConfigVm(CommsSession sp)
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
                                         "YawP",
                                         "YawI",
                                         "YawD",
                                         "KPrate",
                                         "MagnetometerEnable",
                                     };
         
            RefreshCommand = new DelegateCommand(_ => RefreshValues());
            UpdateCommand = new DelegateCommand(_ => UpdateValues());
        }

        public ICommand RefreshCommand { get; private set; }
 
        public ICommand UpdateCommand { get; private set; }

        public float KPrate { get; private set; }

        public bool MagnetometerEnable { get; private set; }

        private void RefreshValues()
        {
            SendString("B");
        }

        public void UpdateValues()
        {
            SendPropsWithCommand("A");
        }

        protected override void OnActivated()
        {
            RefreshValues();
        }

        public override string Name
        {
            get { return "Stable Mode"; }
        }
    }
}