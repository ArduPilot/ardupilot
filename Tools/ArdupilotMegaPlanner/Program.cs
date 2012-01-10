using System;
using System.Collections.Generic;
using System.Windows.Forms;
using System.Net;
using System.IO;
using System.Text;


namespace ArdupilotMega
{
    static class Program
    {

        /// <summary>
        /// The main entry point for the application.
        /// </summary>
        [STAThread]
        static void Main()
        {
            //System.Threading.Thread.CurrentThread.CurrentUICulture = new System.Globalization.CultureInfo("en-US");
            //System.Threading.Thread.CurrentThread.CurrentCulture = new System.Globalization.CultureInfo("en-US");

            Application.ThreadException += new System.Threading.ThreadExceptionEventHandler(Application_ThreadException);

            Application.Idle += new EventHandler(Application_Idle);

            //MessageBox.Show("NOTE: This version may break advanced mission scripting");

            //Common.linearRegression();

            Application.EnableVisualStyles();
            Application.SetCompatibleTextRenderingDefault(false);
            try
            {

                Application.Run(new MainV2());

            }
            catch (Exception ex) { Console.WriteLine(ex.ToString()); }
        }

        static void Application_Idle(object sender, EventArgs e)
        {
            //Console.WriteLine("Idle");
        }

        static void Application_ThreadException(object sender, System.Threading.ThreadExceptionEventArgs e)
        {
            Exception ex = e.Exception;
            if (ex.Message == "The port is closed.") {
                MessageBox.Show("Serial connection has been lost");
                return;
            }
            if (ex.Message == "A device attached to the system is not functioning.")
            {
                MessageBox.Show("Serial connection has been lost");
                return;
            }
            if (e.Exception.GetType() == typeof(MissingMethodException))
            {
                MessageBox.Show("Please Update - Some older library dlls are causing problems\n" + e.Exception.Message);
                return;
            }
            if (e.Exception.GetType() == typeof(ObjectDisposedException) || e.Exception.GetType() == typeof(InvalidOperationException)) // something is trying to update while the form, is closing.
            {
                return; // ignore
            }
            if (e.Exception.GetType() == typeof(FileNotFoundException) || e.Exception.GetType() == typeof(BadImageFormatException)) // i get alot of error from people who click the exe from inside a zip file.
            {
                MessageBox.Show("You are missing some DLL's. Please extract the zip file somewhere. OR Use the update feature from the menu");
                return;
            }
            DialogResult dr = MessageBox.Show("An error has occurred\nReport this Error??? "+ex.ToString(), "Send Error", MessageBoxButtons.YesNo);
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
                catch { MessageBox.Show("Error sending Error report!! Youre most likerly are not on the internet"); }
            }
        }
    }
}
