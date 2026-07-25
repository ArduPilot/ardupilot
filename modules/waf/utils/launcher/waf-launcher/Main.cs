using System;
using System.Collections.Generic;
using System.Diagnostics;

namespace waflauncher
{
	class MainClass
	{
		public static System.Diagnostics.Process exec(string command,params string[] args) {
			String argstring = String.Join(" ",args);
			System.Diagnostics.ProcessStartInfo startinfo = new System.Diagnostics.ProcessStartInfo(command,argstring);
			startinfo.UseShellExecute = false;
			System.Diagnostics.Process p;
			try {
				p = Process.Start(startinfo);
			} catch (System.ComponentModel.Win32Exception){
				return null;
			}
			p.WaitForExit();
			return p;
		}

		public static int Main (string[] args)
		{
			//I run waf and if not successful we try on-the-fly install of python
			if(!runWaf(args)){
				//but first we ask the user if it's okay to install software on their computer
				if(mayInstall()){
					//I install python and try running waf yet another time
					installPython();
					if(!runWaf(args)){
						//If it still fails something has gone horrible wrong
						Console.WriteLine("Python not fully working");
						return 1;
					}
				} else {
					Console.WriteLine("Not automatically installing Python");
					Console.WriteLine("Please download and install http://www.python.org/ftp/python/2.7.1/python-2.7.1.msi");
					Console.WriteLine("or if you have python installed make sure it is on %PATH%");
					Console.WriteLine("or run this command again and answer yes");
				}
			}
			return 0;
		}

		public static bool mayInstall() {
			Console.Write("Download and install python [Y/n]? ");
			ConsoleKeyInfo a = Console.ReadKey();
			Console.WriteLine();
			switch(a.KeyChar){
			case 'Y':
			case 'y':
			case '\n':
			case '\r':
				return true;
			//If unsure default to not doing it
			default:
				return false;
			}
		}

		public static String getwafDir(){
			//This changes the current directory to the place where the exe exists
			System.Reflection.Assembly a = System.Reflection.Assembly.GetEntryAssembly();
			String path = System.IO.Path.GetDirectoryName(a.Location);
			return path + System.IO.Path.DirectorySeparatorChar;
		}

		public static bool runWaf(string[] args){
			Process p = exec("python", getwafDir() + "waf", String.Join(" ",args));
			//If command could be execeuted return true
			if (p != null) return true;
			//If not try with the direct path to the default installation which is where installPython() will install it to
			//This is done since the %PATH% variable might not be setup to include python

			List<String> versions = new List<String>() { "27", "32", "26", "31", "25", "30" };
			foreach (String v in versions) {
				p = exec("C:\\Python"+v+"\\python.exe", "waf", String.Join(" ",args));
				if (p != null) return true;
			}
			return false;
		}

		public static void installPython(){
			//Make a filename to download python to
			String filename = System.IO.Path.GetTempPath() + Char.ToString(System.IO.Path.DirectorySeparatorChar) + "python-2.7.1.msi";

			System.Net.WebClient web = new System.Net.WebClient();
			Console.WriteLine ("Downloading python 2.7");
			web.DownloadFile("http://www.python.org/ftp/python/2.7.1/python-2.7.1.msi",filename);
			Console.WriteLine ("python2.7 downloaded to " + filename);

			Console.WriteLine ("Installing python");
			//filename must be quoted or else msiexec will fail
			exec("msiexec","/qn","/i","\"" +filename + "\"");
			Console.WriteLine ("Python is now installed");
		}
	}
}

