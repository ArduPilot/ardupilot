using System;
using System.IO.Ports;

namespace ArducopterConfigurator.PresentationModels
{
  public class SerialMonitorVm : MonitorVm
    {
        private string _text;

        public SerialMonitorVm(CommsSession _sp) : base(_sp)
        {
            _sp.LineOfDataReceived += new Action<string>(_sp_DataReceived);
          
        }

        void _sp_DataReceived(string obj)
        {
            _text += obj;
            FirePropertyChanged("ReceviedText");
        }

        public string ReceviedText { get { return _text; } }
        
        public string SendText { get; set;  }


      protected override void OnActivated()
        {
          
            SendString("X");
            _text = string.Empty;
            FirePropertyChanged("ReceviedText");
        }
       

        protected override void OnStringReceived(string strReceived)
        {
            _text += strReceived;
            FirePropertyChanged("ReceviedText");
        }

        public override string Name
        {
            get { return "Serial Monitor"; }
        }

        public void SendTextCommand()
        {
            SendString(SendText);
            SendText = "";
            FirePropertyChanged("SendText");
        }
    }
}