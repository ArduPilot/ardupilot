using System;
using System.Collections.Generic;
using System.Text;
using System.IO;
using System.Windows.Forms;
using System.Diagnostics;

namespace wix
{
    class Program
    {
        static int no = 0;

        static StreamWriter sw;

        static List<string> components = new List<string>();

        static string mainexeid = "";

        static void Main(string[] args)
        {
            if (args.Length == 0)
            {
                Console.WriteLine("Bad Directory");
                return;
            }

            string path = args[0];

            string file = Path.GetDirectoryName(Application.ExecutablePath) + Path.DirectorySeparatorChar+ "installer.wxs";

            sw = new StreamWriter(file);

            header();

            sw.WriteLine("<Directory Id=\"APMPlanner\" Name=\"APM Planner\">");

            sw.WriteLine(@"<Component Id=""InstallDirPermissions"" Guid=""{525389D7-EB3C-4d77-A5F6-A285CF99437D}"" KeyPath=""yes""> 
                        <CreateFolder> 
                            <Permission User=""Everyone"" GenericAll=""yes"" /> 
                        </CreateFolder>
                    </Component>");

            //sw.WriteLine("<File Id=\"_" + no + "\" Source=\"" + file + "\" />");
            

            dodirectory(path, 0);


            footer();

            sw.Close();

            /*
            System.Diagnostics.Process P = new System.Diagnostics.Process();
            P.StartInfo.FileName = "cmd.exe";
                
            P.StartInfo.Arguments =  " /c \"" + Path.GetDirectoryName(Application.ExecutablePath) + Path.DirectorySeparatorChar + "installer.bat\"";
            P.StartInfo.WorkingDirectory = Path.GetDirectoryName(Application.ExecutablePath);
            P.Start();
            */
            //Console.ReadLine();
        }

        static void header()
        {
            string newid = System.Guid.NewGuid().ToString();

            newid = "{625389D7-EB3C-4d77-A5F6-A285CF99437D}";

            StreamReader sr = new StreamReader(File.OpenRead("../Properties/AssemblyInfo.cs"));

            string version = "0";

            while (!sr.EndOfStream) {
                string line = sr.ReadLine();
                if (line.Contains("AssemblyFileVersion"))
                {
                    string[] items = line.Split(new char[] { '"' },StringSplitOptions.RemoveEmptyEntries);
                    version = items[1];
                    break;
                }
            }
            sr.Close();

            string data = @"<?xml version=""1.0"" encoding=""utf-8""?>
<Wix xmlns=""http://schemas.microsoft.com/wix/2006/wi"" xmlns:netfx=""http://schemas.microsoft.com/wix/NetFxExtension"" xmlns:difx=""http://schemas.microsoft.com/wix/DifxAppExtension"">


    <Product Id=""" + newid + @""" Name=""APM Planner"" Language=""1033"" Version="""+version+@""" Manufacturer=""Michael Oborne"" UpgradeCode=""{625389D7-EB3C-4d77-A5F6-A285CF99437D}"">
        <Package Description=""APM Planner Installer"" Comments=""Apm Planner Installer"" Manufacturer=""Michael Oborne"" InstallerVersion=""200"" Compressed=""yes"" />

<MajorUpgrade DowngradeErrorMessage=""A later version of [ProductName] is already installed. Setup will now exit.""/>


        <PropertyRef Id=""NETFRAMEWORK35"" />

        <Condition Message=""This application requires .NET Framework 3.5. Please install the .NET Framework then run this installer again.""><![CDATA[Installed OR NETFRAMEWORK35]]></Condition>

        <Media Id=""1"" Cabinet=""product.cab"" EmbedCab=""yes"" />

        <Directory Id=""TARGETDIR"" Name=""SourceDir"">
            <Directory Id=""ProgramFilesFolder"" Name=""PFiles"">
                ";

            sw.WriteLine(data);
        }

        static void footer()
        {
            string data = @"
                    
                    <Directory Id=""drivers"" Name=""Drivers"">
                        <Component Id=""MyDriver"" Guid=""{6AC8226E-A005-437e-A3CD-0FC32D9A346F}"">
                            <File Id=""apm2inf""  Source=""..\Driver\Arduino MEGA 2560.inf"" />
                            <difx:Driver AddRemovePrograms='no' Legacy=""yes"" PlugAndPlayPrompt=""no"" />
                        </Component>
                    </Directory>
                </Directory>
            </Directory>

            <Directory Id=""ProgramMenuFolder"">
                <Directory Id=""ApplicationProgramsFolder"" Name=""APM Planner"" />
            </Directory>

        </Directory>

        <DirectoryRef Id=""ApplicationProgramsFolder"">
            <Component Id=""ApplicationShortcut"" Guid=""{8BC628BA-08A0-43d6-88C8-D4C007AC4607}"">
                <Shortcut Id=""ApplicationStartMenuShortcut"" Name=""APM Planner"" Description=""Ardupilot Mega Planner"" Target=""[APMPlanner]ArdupilotMegaPlanner.exe"" WorkingDirectory=""APMPlanner"" />
                <RemoveFolder Id=""ApplicationProgramsFolder"" On=""uninstall"" />

                <Shortcut Id=""UninstallProduct"" Name=""Uninstall APM Planner"" Description=""Uninstalls My Application"" Target=""[System64Folder]msiexec.exe"" Arguments=""/x [ProductCode]"" />



                <RegistryValue Root=""HKCU"" Key=""Software\MichaelOborne\APMPlanner"" Name=""installed"" Type=""integer"" Value=""1"" KeyPath=""yes"" />




            </Component>
        </DirectoryRef>


        <Feature Id=""MyFeature"" Title=""My 1st Feature"" Level=""1"">
            <ComponentRef Id=""InstallDirPermissions"" />
";
            sw.WriteLine(data);

            foreach (string comp in components)
            {
                sw.WriteLine(@"<ComponentRef Id="""+comp+@""" />");
            }

data = @"
            
            <ComponentRef Id=""ApplicationShortcut"" />
            <ComponentRef Id=""MyDriver"" />
        </Feature>

        
            <!-- Step 2: Add UI to your installer / Step 4: Trigger the custom action -->
    <Property Id=""WIXUI_INSTALLDIR"" Value=""APMPlanner"" />

    <UI>
        <UIRef Id=""WixUI_InstallDir"" />
        <Publish Dialog=""ExitDialog"" 
            Control=""Finish"" 
            Event=""DoAction"" 
            Value=""LaunchApplication"">WIXUI_EXITDIALOGOPTIONALCHECKBOX = 1 and NOT Installed</Publish>
    </UI>
    <Property Id=""WIXUI_EXITDIALOGOPTIONALCHECKBOXTEXT"" Value=""Launch APM Planner"" />

    <!-- Step 3: Include the custom action -->
    <Property Id=""WixShellExecTarget"" Value=""[#" + mainexeid + @"]"" />
    <CustomAction Id=""LaunchApplication"" 
        BinaryKey=""WixCA"" 
        DllEntry=""WixShellExec""
        Impersonate=""yes"" />
    </Product>
    
</Wix>";

            sw.WriteLine(data);
        }

        static void dodirectory(string path, int level = 1)
        {
            string[] dirs = Directory.GetDirectories(path);

            if (level != 0)
                sw.WriteLine("<Directory Id=\"" + Path.GetFileName(path).Replace('-', '_') + "\" Name=\"" + Path.GetFileName(path) + "\">");

            string[] files = Directory.GetFiles(path);

            sw.WriteLine("<Component Id=\"_comp"+no+"\" Guid=\""+ System.Guid.NewGuid().ToString() +"\">");

            components.Add("_comp"+no);

            foreach (string filepath in files)
            {
                if (filepath.EndsWith("config.xml") || filepath.Contains("ArdupilotPlanner.log"))
                    continue;
                no++;
                sw.WriteLine("<File Id=\"_" + no + "\" Source=\"" + filepath + "\" />");

                if (filepath.EndsWith("ArdupilotMegaPlanner.exe")) {
                    mainexeid = "_" + no;
                }
            }

            sw.WriteLine("</Component>");

            foreach (string dir in dirs)
            {
                if (dir.EndsWith("gmapcache") || dir.EndsWith("srtm"))
                    continue;
                dodirectory(dir);
            }

            if (level != 0)
                sw.WriteLine("</Directory>");
        }

        static string fixname(string name)
        {
            name = name.Replace("-", "_");
            name = name.Replace(" ", "_");
            name = name.Replace(" ", "_");

            return name;
        }
    }
}
