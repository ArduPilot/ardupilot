using System;
using System.Collections.Generic;
using System.Windows.Forms;
using System.Net;
using System.IO;
using System.Text;
using System.Threading;
using log4net;
using log4net.Config;

namespace ArdupilotMega
{
    static class Program
    {
        private static readonly ILog log = LogManager.GetLogger("Program");

        /// <summary>
        /// The main entry point for the application.
        /// </summary>
        [STAThread]
        static void Main()
        {

            Application.EnableVisualStyles();
            XmlConfigurator.Configure();
            log.Info("******************* Logging Configured *******************");
            Application.SetCompatibleTextRenderingDefault(false);

            Application.ThreadException += Application_ThreadException;

            Application.Idle += Application_Idle;

            //MagCalib.ProcessLog();

            //MessageBox.Show("NOTE: This version may break advanced mission scripting");

            //Common.linearRegression();

            //Console.WriteLine(srtm.getAltitude(-35.115676879882812, 117.94178754638671,20));
            /*
            Arduino.ArduinoSTK comport = new Arduino.ArduinoSTK();

            comport.PortName = "com4";

            comport.BaudRate = 57600;

            comport.Open();

            comport.connectAP();

            comport.sync();

            comport.sync();

            Console.WriteLine( comport.getChipType(0));
            
            Console.WriteLine(comport.getChipType(1));

            Console.WriteLine(comport.getChipType(2));

            Console.ReadLine();

            return;
            */

            try
            {
                Thread.CurrentThread.Name = "Base Thread";

                Application.Run(new MainV2());
            }
            catch (Exception ex)
            {
                log.Fatal("Fatal app exception", ex);
                Console.WriteLine(ex.ToString());

                Console.ReadLine();
            }
        }

        static DateTime lastidle = DateTime.Now;

        static void Application_Idle(object sender, EventArgs e)
        {
            //System.Threading.Thread.Sleep(10);
            //Console.Write("Idle\n");
            if (lastidle.AddMilliseconds(20) < DateTime.Now)
            {
                Application.DoEvents();
                lastidle = DateTime.Now;
            }

            System.Threading.Thread.Sleep(1);
        }

        static void Application_ThreadException(object sender, System.Threading.ThreadExceptionEventArgs e)
        {
            Exception ex = e.Exception;

            log.Debug(ex.ToString());

            if (ex.Message == "Requested registry access is not allowed.")
            {
                return;
            }
            if (ex.Message == "The port is closed.")
            {
                CustomMessageBox.Show("Serial connection has been lost");
                return;
            }
            if (ex.Message == "A device attached to the system is not functioning.")
            {
                CustomMessageBox.Show("Serial connection has been lost");
                return;
            }
            if (e.Exception.GetType() == typeof(MissingMethodException))
            {
                CustomMessageBox.Show("Please Update - Some older library dlls are causing problems\n" + e.Exception.Message);
                return;
            }
            if (e.Exception.GetType() == typeof(ObjectDisposedException) || e.Exception.GetType() == typeof(InvalidOperationException)) // something is trying to update while the form, is closing.
            {
                return; // ignore
            }
            if (e.Exception.GetType() == typeof(FileNotFoundException) || e.Exception.GetType() == typeof(BadImageFormatException)) // i get alot of error from people who click the exe from inside a zip file.
            {
                CustomMessageBox.Show("You are missing some DLL's. Please extract the zip file somewhere. OR Use the update feature from the menu " + e.Exception.ToString());
                // return;
            }
            DialogResult dr = CustomMessageBox.Show("An error has occurred\n" + ex.ToString() + "\n\nReport this Error???", "Send Error", MessageBoxButtons.YesNo);
            if (DialogResult.Yes == dr)
            {
                try
                {
                    // Create a request using a URL that can receive a post. 
                    WebRequest request = WebRequest.Create("http://vps.oborne.me/mail.php");
                    request.Timeout = 10000; // 10 sec
                    // Set the Method property of the request to POST.
                    request.Method = "POST";
                    // Create POST data and convert it to a byte array.
                    string postData = "message=" + Environment.OSVersion.VersionString + " " + System.Reflection.Assembly.GetExecutingAssembly().GetName().Version.ToString() + " " + Application.ProductVersion + " Exception " + ex.ToString().Replace('&', ' ').Replace('=', ' ') + " Stack: " + ex.StackTrace.ToString().Replace('&', ' ').Replace('=', ' ');
                    byte[] byteArray = Encoding.ASCII.GetBytes(postData);
                    // Set the ContentType property of the WebRequest.
                    request.ContentType = "application/x-www-form-urlencoded";
                    // Set the ContentLength property of the WebRequest.
                    request.ContentLength = byteArray.Length;
                    // Get the request stream.
                    Stream dataStream = request.GetRequestStream();
                    // Write the data to the request stream.
                    dataStream.Write(byteArray, 0, byteArray.Length);
                    // Close the Stream object.
                    dataStream.Close();
                    // Get the response.
                    WebResponse response = request.GetResponse();
                    // Display the status.
                    Console.WriteLine(((HttpWebResponse)response).StatusDescription);
                    // Get the stream containing content returned by the server.
                    dataStream = response.GetResponseStream();
                    // Open the stream using a StreamReader for easy access.
                    StreamReader reader = new StreamReader(dataStream);
                    // Read the content.
                    string responseFromServer = reader.ReadToEnd();
                    // Display the content.
                    Console.WriteLine(responseFromServer);
                    // Clean up the streams.
                    reader.Close();
                    dataStream.Close();
                    response.Close();
                }
                catch
                {
                    CustomMessageBox.Show("Error sending Error report!! Youre most likerly are not on the internet");
                }
            }
        }
    }
}