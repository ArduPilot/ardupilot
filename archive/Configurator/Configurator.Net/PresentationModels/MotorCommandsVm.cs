using System;

namespace ArducopterConfigurator.PresentationModels
{
    public class MotorCommandsVm : NotifyProperyChangedBase, IPresentationModel
    {
        public string Name
        {
            get { return "Motor Commands"; }
        }

        public void Activate()
        {
        }

        public void DeActivate()
        {
            // todo stop
        }

        public event EventHandler updatedByApm;

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

        public void handleLineOfText(string strRx)
        {
        }

        public event EventHandler<sendTextToApmEventArgs> sendTextToApm;
    }
}