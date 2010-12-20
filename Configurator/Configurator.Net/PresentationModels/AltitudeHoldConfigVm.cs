namespace ArducopterConfigurator.PresentationModels
{
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