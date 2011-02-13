using System;

namespace ArducopterConfigurator.PresentationModels
{
    /// <summary>
    /// Vm for Altitude hold settings
    /// </summary>
    /// <remarks>
    /// Todo: this one is weird because the APM sends and receives the values
    /// in a different order
    /// There is a unit test to cover it but it will need fixing.
    /// </remarks>
    public class AltitudeHoldConfigVm : VmBase, IPresentationModel, ItalksToApm
    {
        public AltitudeHoldConfigVm()
        {
            PropsInUpdateOrder = new[] {  "P",  "I",  "D", };
        
            RefreshCommand = new DelegateCommand(_ => RefreshValues());
            UpdateCommand = new DelegateCommand(_ => UpdateValues());
        }

        public ICommand RefreshCommand { get; private set; }
        public ICommand UpdateCommand { get; private set; }

        public float P { get;  set; }
        public float I { get;  set; }
        public float D { get;  set; }


        private void RefreshValues()
        {
            if (sendTextToApm != null)
                sendTextToApm(this, new sendTextToApmEventArgs("F"));
        }

        public void UpdateValues()
        {
            if (sendTextToApm != null)
                sendTextToApm(this, new sendTextToApmEventArgs(ComposePropsWithCommand("E")));
        }

        public string Name
        {
            get { return "Altitude Hold"; }
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