using System;

namespace ArducopterConfigurator
{
    public interface ItalksToApm
    {
        void handleLineOfText(string strRx);
        event EventHandler<sendTextToApmEventArgs> sendTextToApm;

    }

    public class sendTextToApmEventArgs : EventArgs
    {
        public string textToSend;

        public sendTextToApmEventArgs(string textToSend)
        {
            this.textToSend = textToSend;
        }
    }
}