using System;
using System.IO.Ports;

namespace ArducopterConfigurator.PresentationModels
{
    public class SerialMonitorVm : NotifyProperyChangedBase, IPresentationModel
    {
        private string _text;

        public string ReceviedText { get { return _text; } }
        
        public string SendText { get; set;  }
     
        public string Name
        {
            get { return "Serial Monitor"; }
        }

        public void Activate()
        {
            _text = string.Empty;
            FirePropertyChanged("ReceviedText");
        }

        public void DeActivate()
        {
              if (sendTextToApm!=null)
                sendTextToApm(this, new sendTextToApmEventArgs("X"));
        }

        public event EventHandler updatedByApm;

        public void SendTextCommand()
        {
            if (sendTextToApm != null)
                sendTextToApm(this, new sendTextToApmEventArgs(SendText));
            SendText = "";
            FirePropertyChanged("SendText");
        }

        public void handleLineOfText(string strRx)
        {
            _text += strRx + Environment.NewLine;
            FirePropertyChanged("ReceviedText");
        }

        public event EventHandler<sendTextToApmEventArgs> sendTextToApm;
    }
}