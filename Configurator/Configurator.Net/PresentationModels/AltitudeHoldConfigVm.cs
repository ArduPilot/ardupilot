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
    public class AltitudeHoldConfigVm : MonitorVm
    {
        public AltitudeHoldConfigVm(IComms sp)
            : base(sp)
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
            SendString("F");
        }

        public void UpdateValues()
        {
            SendPropsWithCommand("E");
        }

        protected override void OnActivated()
        {
            RefreshValues();
        }

        protected override void OnStringReceived(string strReceived)
        {
            PopulatePropsFromUpdate(strReceived,true);
        }

        public override string Name
        {
            get { return "Altitude Hold"; }
        }
    }
}