using System;

namespace ArducopterConfigurator.PresentationModels
{
    public class AcroModeConfigVm : ConfigWithPidsBase, IPresentationModel, ItalksToApm
    {
        public AcroModeConfigVm()
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

        public float TransmitterFactor { get; set; }

        public ICommand RefreshCommand { get; private set; }
 
        public ICommand UpdateCommand { get; private set; }
       

        public void UpdateValues()
        {
            if (sendTextToApm != null)
                sendTextToApm(this, new sendTextToApmEventArgs(ComposePropsWithCommand("O")));
        }

        public void RefreshValues()
        {
            if (sendTextToApm != null)
                sendTextToApm(this, new sendTextToApmEventArgs("P"));
                
        }

        public string Name
        {
            get { return "Acro Mode"; }
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
            if (updatedByApm != null)
                updatedByApm(this, EventArgs.Empty);
        }

        public event EventHandler<sendTextToApmEventArgs> sendTextToApm;
    }
}