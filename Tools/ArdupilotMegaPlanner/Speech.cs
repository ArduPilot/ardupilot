using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Speech.Synthesis;

namespace ArdupilotMega
{
    public class Speech
    {
        SpeechSynthesizer _speechwindows;
        System.Diagnostics.Process _speechlinux;

        bool MONO = false;

        public SynthesizerState State { 
            get {
                if (MONO)
                {
                    return SynthesizerState.Ready;
                }
                else
                {
                    return _speechwindows.State;
                }
            } 
            private set { } 
        }

        public Speech()
        {
            var t = Type.GetType("Mono.Runtime");
            MONO = (t != null);

            if (MONO)
            {
                _speechlinux = new System.Diagnostics.Process();
                _speechlinux.StartInfo.FileName = "festival";
            }
            else
            {
                _speechwindows = new SpeechSynthesizer();
            }
        }

        public void SpeakAsync(string text)
        {
            if (MONO)
            {

            }
            else
            {
                _speechwindows.SpeakAsync(text);
            }
        }

        public void SpeakAsyncCancelAll()
        {
            if (MONO)
            {

            }
            else
            {
                _speechwindows.SpeakAsyncCancelAll();
            }
        }
    }
}