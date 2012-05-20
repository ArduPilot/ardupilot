using System;
using System.Collections.Generic;
using System.Text;
using System.IO;
using System.Windows.Forms;
using System.Diagnostics;
using System.Runtime.InteropServices;
using System.Reflection;

namespace wix
{
    class Program
    {
        /// <summary>
        /// The operation completed successfully.
        /// </summary>
        public const int ERROR_SUCCESS = 0;
        /// <summary>
        /// Incorrect function.
        /// </summary>
        public const int ERROR_INVALID_FUNCTION = 1;
        /// <summary>
        /// The system cannot find the file specified.
        /// </summary>
        public const int ERROR_FILE_NOT_FOUND = 2;
        /// <summary>
        /// The system cannot find the path specified.
        /// </summary>
        public const int ERROR_PATH_NOT_FOUND = 3;
        /// <summary>
        /// The system cannot open the file.
        /// </summary>
        public const int ERROR_TOO_MANY_OPEN_FILES = 4;
        /// <summary>
        /// Access is denied.
        /// </summary>
        public const int ERROR_ACCESS_DENIED = 5;

        const Int32 DRIVER_PACKAGE_REPAIR = 0x00000001;
        const Int32 DRIVER_PACKAGE_SILENT = 0x00000002;
        const Int32 DRIVER_PACKAGE_FORCE = 0x00000004;
        const Int32 DRIVER_PACKAGE_ONLY_IF_DEVICE_PRESENT = 0x00000008;
        const Int32 DRIVER_PACKAGE_LEGACY_MODE = 0x00000010;
        const Int32 DRIVER_PACKAGE_DELETE_FILES = 0x00000020;

        [DllImport("DIFXApi.dll", CharSet = CharSet.Unicode)]
        public static extern Int32 DriverPackagePreinstall(string DriverPackageInfPath, Int32 Flags);

        static void driverinstall()
        {
            int result = DriverPackagePreinstall(@"..\Driver\Arduino MEGA 2560.inf", 0);
            if (result != 0)
                MessageBox.Show("Driver installation failed. " + result);

        }

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

            if (args[0] == "driver")
            {
                driverinstall();
                return;
            }

            string path = args[0];
            //Path.GetDirectoryName(Application.ExecutablePath) + Path.DirectorySeparatorChar+ 
            string file = "installer.wxs";

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

            string exepath = Path.GetFullPath(path) + Path.DirectorySeparatorChar + "ArdupilotMegaPlanner.exe";
            string version = Assembly.LoadFile(exepath).GetName().Version.ToString();

            System.Diagnostics.FileVersionInfo fvi = FileVersionInfo.GetVersionInfo(exepath);

            StreamWriter st = new StreamWriter("create.bat", false);

            st.WriteLine("del installer.wixobj");

            st.WriteLine(@"""%wix%\bin\candle"" installer.wxs -ext WiXNetFxExtension -ext WixDifxAppExtension -ext WixUIExtension.dll -ext WixUtilExtension");

            st.WriteLine(@"""%wix%\bin\light"" installer.wixobj ""%wix%\bin\difxapp_x86.wixlib"" -o MissionPlanner32-" + fvi.FileVersion + ".msi -ext WiXNetFxExtension -ext WixDifxAppExtension -ext WixUIExtension.dll -ext WixUtilExtension");

            st.WriteLine(@"""%wix%\bin\light"" installer.wixobj ""%wix%\bin\difxapp_x64.wixlib"" -o MissionPlanner64-" + fvi.FileVersion + ".msi -ext WiXNetFxExtension -ext WixDifxAppExtension -ext WixUIExtension.dll -ext WixUtilExtension");

            st.WriteLine(@"""C:\Program Files\7-Zip\7z.exe"" a -tzip -xr!*.log -xr!ArdupilotPlanner.log* -xr!*.tlog -xr!config.xml -xr!gmapcache -xr!eeprom.bin -xr!dataflash.bin -xr!*.new ""Mission Planner " + fvi.FileVersion + @".zip"" ..\bin\release\*");

            st.WriteLine("pause");

            st.WriteLine("googlecode_upload.py -s \"Mission Planner zip file, " + fvi.FileVersion + "\" -p ardupilot-mega \"Mission Planner " + fvi.FileVersion + @".zip""");

            st.WriteLine("googlecode_upload.py -s \"Mission Planner installer (32-bit)\" -p ardupilot-mega MissionPlanner32-" + fvi.FileVersion + ".msi");
            st.WriteLine("googlecode_upload.py -s \"Mission Planner installer (64-bit)\" -p ardupilot-mega MissionPlanner64-" + fvi.FileVersion + ".msi");


            st.Close();

            runProgram("create.bat");


        }

        static void runProgram(string run)
        {
            System.Diagnostics.Process P = new System.Diagnostics.Process();
            P.StartInfo.FileName = run;

//            P.StartInfo.WorkingDirectory = Path.GetDirectoryName(Application.ExecutablePath);
            P.StartInfo.UseShellExecute = true;
            P.Start();
        }

        static void header()
        {
            string newid = System.Guid.NewGuid().ToString();

            newid = "*";

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


<Upgrade Id=""{625389D7-EB3C-4d77-A5F6-A285CF99437D}"">
    <UpgradeVersion OnlyDetect=""yes"" Minimum=""" + version + @""" Property=""NEWERVERSIONDETECTED"" IncludeMinimum=""no"" />
    <UpgradeVersion OnlyDetect=""no"" Maximum=""" + version + @""" Property=""OLDERVERSIONBEINGUPGRADED"" IncludeMaximum=""no"" />
</Upgrade>

<InstallExecuteSequence>
    <RemoveExistingProducts After=""InstallInitialize"" />
</InstallExecuteSequence>

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
                sw.WriteLine("<Directory Id=\"" + Path.GetFileName(path).Replace('-', '_') + no + "\" Name=\"" + Path.GetFileName(path) + "\">");

            string[] files = Directory.GetFiles(path);

            no++;

            sw.WriteLine("<Component Id=\"_comp"+no+"\" Guid=\""+ System.Guid.NewGuid().ToString() +"\">");
            components.Add("_comp"+no);

            foreach (string filepath in files)
            {
                if (filepath.ToLower().EndsWith("release\\config.xml") || filepath.ToLower().Contains("ardupilotplanner.log") || 
                    filepath.ToLower().Contains("dataflash.bin") || filepath.ToLower().Contains(".etag"))
                    continue;

                no++;
                

                if (filepath.EndsWith("ArdupilotMegaPlanner.exe")) {
                    mainexeid = "_" + no;

                    sw.WriteLine("<File Id=\"_" + no + "\" Source=\"" + filepath + "\" ><netfx:NativeImage Id=\"ngen_ArdupilotMegaPlannerexe\"/> </File>");

                } else {
                    sw.WriteLine("<File Id=\"_" + no + "\" Source=\"" + filepath + "\" />");
                }
            }

            sw.WriteLine("</Component>");

            foreach (string dir in dirs)
            {
                if (dir.ToLower().EndsWith("gmapcache") || dir.ToLower().EndsWith("srtm") || dir.ToLower().EndsWith("logs"))
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
