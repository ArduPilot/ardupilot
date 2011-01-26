using System;
using System.Collections.Generic;
using System.Windows.Forms;
using ArducopterConfigurator.PresentationModels;

namespace ArducopterConfigurator
{
    static class Program
    {
        public static bool IsMonoRuntime;

        /// <summary>
        /// The main entry point for the application.
        /// </summary>
        [STAThread]
        static void Main()
        {
            Application.EnableVisualStyles();
            Application.SetCompatibleTextRenderingDefault(false);

            var t = Type.GetType("Mono.Runtime");
            IsMonoRuntime = (t != null);

            var session = new CommsSession();
            //var session = new FakeCommsSession();
			

            var mainVm = new MainVm(session);
		

            Application.Run(new mainForm(mainVm));
        }
    
    
    }
}
