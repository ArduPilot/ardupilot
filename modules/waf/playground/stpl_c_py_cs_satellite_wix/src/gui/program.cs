using System;
using System.Windows.Forms;
using System.Globalization;
using System.Threading;
using Funi;

namespace Funi
{
    static class Program
    {
        [STAThread]
        static void Main()
        {

//            /* for testing only:
//             * the according Satellite Assembly is chosen based on CurrentThread.CurrentCulture
            CultureInfo culture;
            culture = CultureInfo.CreateSpecificCulture("fr");

            CultureInfo.DefaultThreadCurrentCulture = culture;
            CultureInfo.DefaultThreadCurrentUICulture = culture;

            Thread.CurrentThread.CurrentCulture = culture;
            Thread.CurrentThread.CurrentUICulture = culture;
//             * fallback is english
//             */

            Application.Run(new FormFuni());
        }
    }
}
