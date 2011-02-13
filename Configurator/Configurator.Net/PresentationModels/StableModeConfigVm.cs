using System;

namespace ArducopterConfigurator.PresentationModels
{
    public class StableModeConfigVm : ConfigWithPidsBase, IPresentationModel, ItalksToApm
    {
        public StableModeConfigVm()
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

        public float KPrate { get;  set; }

        private bool magnetometerEnable;
        public bool MagnetometerEnable
        {
            get { return magnetometerEnable; }
            set { magnetometerEnable = value; }
        }

        private void RefreshValues()
        {
            if (sendTextToApm != null)
                sendTextToApm(this, new sendTextToApmEventArgs("B"));
                
        }

        public void UpdateValues()
        {
            if (sendTextToApm != null)
                sendTextToApm(this,new sendTextToApmEventArgs(ComposePropsWithCommand("A")));
        }

        public string Name
        {
            get { return "Stable Mode"; }
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
           PopulatePropsFromUpdate(strRx,true);
        }

        public event EventHandler<sendTextToApmEventArgs> sendTextToApm;
    }
}