namespace ArducopterConfigurator.PresentationModels
{
    public class MotorCommandsVm : MonitorVm
    {
        public MotorCommandsVm(IComms _sp)
            : base(_sp)
        {
        }

        public override string Name
        {
            get { return "Motor Commands"; }
        }

        public int MotorFront { get; set; }
        public int MotorRear { get; set; }
        public int MotorLeft { get; set; }
        public int MotorRight { get; set; }

        public void SendCommand()
        {
        }

        public void StopCommand()
        {
        }


        protected override void OnDeactivated()
        {
          
        }

        protected override void OnActivated()
        {
           
        }

        protected override void OnStringReceived(string strReceived)
        {
            
        }
    }
}