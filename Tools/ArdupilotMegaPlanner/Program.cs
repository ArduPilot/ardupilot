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

        public static Splash Splash;
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

            AppDomain.CurrentDomain.UnhandledException += new UnhandledExceptionEventHandler(CurrentDomain_UnhandledException);

            Application.Idle += Application_Idle;

            //MagCalib.ProcessLog();

            //MessageBox.Show("NOTE: This version may break advanced mission scripting");

            //Common.linearRegression();

            //Console.WriteLine(srtm.getAltitude(-35.115676879882812, 117.94178754638671,20));

           // Console.ReadLine();
           // return;
            /*
            Arduino.ArduinoSTKv2 comport = new Arduino.ArduinoSTKv2();

            comport.PortName = "com8";

            comport.BaudRate = 115200;

            comport.Open();

            Arduino.Chip.Populate();

            if (comport.connectAP())
            {
                Arduino.Chip chip = comport.getChipType();
                Console.WriteLine(chip);
            }
            Console.ReadLine();

            return;
            */
            /*
            Comms.SerialPort sp = new Comms.SerialPort();

            sp.PortName = "com8";
            sp.BaudRate = 115200;

            CurrentState cs = new CurrentState();

            MAVLink mav = new MAVLink();

            mav.BaseStream = sp;

            mav.Open();

            HIL.XPlane xp = new HIL.XPlane();

            xp.SetupSockets(49005, 49000, "127.0.0.1");

            HIL.Hil.sitl_fdm data = new HIL.Hil.sitl_fdm();

            while (true)
            {
                while (mav.BaseStream.BytesToRead > 0)
                    mav.readPacket();

                // update all stats
                cs.UpdateCurrentSettings(null);

                xp.GetFromSim(ref data);
                xp.GetFromAP(); // no function

                xp.SendToAP(data);
                xp.SendToSim();

                MAVLink.mavlink_rc_channels_override_t rc = new MAVLink.mavlink_rc_channels_override_t();

                rc.chan3_raw = 1500;

                mav.sendPacket(rc);
                
            }       */
            /*
            MAVLink mav = new MAVLink();

            mav.BaseStream = new Comms.CommsFile() { PortName = @"C:\Users\hog\Documents\Visual Studio 2010\Projects\ArdupilotMega\ArdupilotMega\bin\Debug\logs\2012-09-09 15-07-25.tlog" };

            mav.Open(true);

            while (mav.BaseStream.BytesToRead > 0)
            {

                byte[] packet = mav.readPacket();

                mav.DebugPacket(packet, true);
            }
            */

            Splash = new ArdupilotMega.Splash();
            Splash.Show();

            try
            {
                Thread.CurrentThread.Name = "Base Thread";

                Application.Run(new MainV2());
            }
            catch (Exception ex)
            {
                if (ex.GetType() == typeof(FileNotFoundException))
                {
                    Console.WriteLine("If your error is about Microsoft.DirectX.DirectInput, please install the latest directx redist from here http://www.microsoft.com/en-us/download/details.aspx?id=35 \n\n");
                }

                log.Fatal("Fatal app exception", ex);
                Console.WriteLine(ex.ToString());

                Console.ReadLine();
            }
        }

        static void CurrentDomain_UnhandledException(object sender, UnhandledExceptionEventArgs e)
        {
            handleException((Exception)e.ExceptionObject);
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

        static void handleException(Exception ex)
        {
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
            if (ex.GetType() == typeof(MissingMethodException))
            {
                CustomMessageBox.Show("Please Update - Some older library dlls are causing problems\n" + ex.Message);
                return;
            }
            if (ex.GetType() == typeof(ObjectDisposedException) || ex.GetType() == typeof(InvalidOperationException)) // something is trying to update while the form, is closing.
            {
                return; // ignore
            }
            if (ex.GetType() == typeof(FileNotFoundException) || ex.GetType() == typeof(BadImageFormatException)) // i get alot of error from people who click the exe from inside a zip file.
            {
                CustomMessageBox.Show("You are missing some DLL's. Please extract the zip file somewhere. OR Use the update feature from the menu " + ex.ToString());
                // return;
            }
            DialogResult dr = CustomMessageBox.Show("An error has occurred\n" + ex.ToString() + "\n\nReport this Error???", "Send Error", MessageBoxButtons.YesNo);
            if (DialogResult.Yes == dr)
            {
                try
                {
                    string data = "";
                        foreach (System.Collections.DictionaryEntry de in ex.Data)
                            data += String.Format("-> {0}: {1}", de.Key, de.Value);
              

                    // Create a request using a URL that can receive a post. 
                    WebRequest request = WebRequest.Create("http://vps.oborne.me/mail.php");
                    request.Timeout = 10000; // 10 sec
                    // Set the Method property of the request to POST.
                    request.Method = "POST";
                    // Create POST data and convert it to a byte array.
                    string postData = "message=" + Environment.OSVersion.VersionString + " " + System.Reflection.Assembly.GetExecutingAssembly().GetName().Version.ToString() 
                        + " " + Application.ProductVersion 
                        + "\nException " + ex.ToString().Replace('&', ' ').Replace('=', ' ') 
                        + "\nStack: " + ex.StackTrace.ToString().Replace('&', ' ').Replace('=', ' ') 
                        + "\nTargetSite " + ex.TargetSite + " " + ex.TargetSite.DeclaringType
                        + "\ndata " + data;
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

        static void Application_ThreadException(object sender, System.Threading.ThreadExceptionEventArgs e)
        {
            Exception ex = e.Exception;

            handleException(ex);
        }
    }
}