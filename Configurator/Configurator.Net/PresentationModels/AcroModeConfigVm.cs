using System;

namespace ArducopterConfigurator.PresentationModels
{
    public class AcroModeConfigVm : ConfigWithPidsBase
    {
        public AcroModeConfigVm(IComms sp) : base(sp)
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
                           "TransmitterFactor", 
                       };

            RefreshCommand = new DelegateCommand(_ => RefreshValues());
            UpdateCommand = new DelegateCommand(_ => UpdateValues());
        }

        public float TransmitterFactor { get; private set; }

        public ICommand RefreshCommand { get; private set; }
 
        public ICommand UpdateCommand { get; private set; }

        protected override void OnActivated()
        {
            RefreshValues();
        }

        private void RefreshValues()
        {
            SendString("P");
        }

        public void UpdateValues()
        {
            SendPropsWithCommand("O");
        }

        public override string Name
        {
            get { return "Acro Mode"; }
        }
    }
}