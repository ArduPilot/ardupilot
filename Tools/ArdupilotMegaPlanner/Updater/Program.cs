using System;
using System.Collections.Generic;
using System.Windows.Forms;
using System.IO;

namespace Updater
{
    static class Program
    {
        static bool MAC = false;

        /// <summary>
        /// The main entry point for the application.
        /// </summary>
        [STAThread]
        static void Main()
        {
            OperatingSystem os = Environment.OSVersion;

            Console.WriteLine(os.VersionString.ToString());

            if (os.VersionString.ToString().ToUpper().Contains("UNIX"))
            {
                MAC = true;
            }

            string path = Path.GetDirectoryName(Application.ExecutablePath);

            // give 4 seconds grace
            System.Threading.Thread.Sleep(5000);

            //UpdateFiles(path);

            if (!UpdateFiles(path))
            {
                Console.WriteLine("Update failed, please try it later.");
                Console.WriteLine("Press any key to continue.");
                Console.ReadKey();
            }
            else
            {
                try
                {
                    System.Diagnostics.Process P = new System.Diagnostics.Process();
                    if (MAC)
                    {
                        P.StartInfo.FileName = "mono";
                        P.StartInfo.Arguments = " \"" + Path.GetDirectoryName(Application.ExecutablePath) + Path.DirectorySeparatorChar + "ArdupilotMegaPlanner.exe\"";
                    }
                    else
                    {
                        P.StartInfo.FileName = Path.GetDirectoryName(Application.ExecutablePath) + Path.DirectorySeparatorChar + "ArdupilotMegaPlanner.exe";
                        P.StartInfo.Arguments = "";
                    }
                    Console.WriteLine("Start " + P.StartInfo.FileName + " with " + P.StartInfo.Arguments);
                    P.Start();
                }
                catch { } // likely file didnt exist
            }
        }

        static bool UpdateFiles(string directory)
        {
            bool all_done = true;
            try
            {
                string[] files = Directory.GetFiles(directory);

                foreach (string file in files)
                {
                    if (file.ToLower().EndsWith(".new") && file.ToLower() != ".new") // cant move ".new" to ""
                    {
                        bool done = false;
                        for (int try_count = 0; try_count < 10 && !done; try_count++)  // try no more than 5 times
                        {
                            if (file.ToLower().Contains("updater.exe")) // cant self update on windows
                                break;
                            try
                            {
                                Console.Write("Move: " + file + " TO " + file.Remove(file.Length - 4));
                                File.Copy(file, file.Remove(file.Length - 4), true);
                                done = true;
                                File.Delete(file);
                                Console.WriteLine(" Done.");
                            }
                            catch
                            {
                                Console.WriteLine(" Failed.");
                                System.Threading.Thread.Sleep(500);
                            }
                        }
                        all_done = all_done && done;
                    }
                }

            }
            catch { }

            foreach (string subdir in Directory.GetDirectories(directory))
                all_done = all_done && UpdateFiles(subdir);

            return all_done;
            //P.StartInfo.RedirectStandardOutput = true;
        }
    }
}
