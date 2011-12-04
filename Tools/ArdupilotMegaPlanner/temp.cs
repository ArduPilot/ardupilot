using System;
using System.Collections.Generic;
using System.ComponentModel;
using System.Data;
using System.Drawing;
using System.Text;
using System.Windows.Forms;
using System.Text.RegularExpressions;
using System.IO.Ports;
using System.IO;
using System.Runtime.InteropServices;
using System.Net;

using GMap.NET.WindowsForms;
using GMap.NET.CacheProviders;

using OpenGlobe.Core;
using OpenGlobe.Renderer;
using OpenGlobe.Scene;


namespace ArdupilotMega
{
    public partial class temp : Form
    {
        public temp()
        {
            InitializeComponent();
        }

        private void temp_Load(object sender, EventArgs e)
        {

        }

        public static byte[] Swap(params object[] list)
        {
            // The copy is made becuase SetValue won't work on a struct.
            // Boxing was used because SetValue works on classes/objects.
            // Unfortunately, it results in 2 copy operations.
            object thisBoxed = list[0]; // Why make a copy?
            Type test = thisBoxed.GetType();

            int offset = 0;
            byte[] data = new byte[Marshal.SizeOf(thisBoxed)];

            // System.Net.IPAddress.NetworkToHostOrder is used to perform byte swapping.
            // To convert unsigned to signed, 'unchecked()' was used.
            // See http://stackoverflow.com/questions/1131843/how-do-i-convert-uint-to-int-in-c

            // Enumerate each structure field using reflection.
            foreach (var field in test.GetFields())
            {
                // field.Name has the field's name.

                object fieldValue = field.GetValue(thisBoxed); // Get value

                // Get the TypeCode enumeration. Multiple types get mapped to a common typecode.
                TypeCode typeCode = Type.GetTypeCode(fieldValue.GetType());

                switch (typeCode)
                {
                    case TypeCode.Single: // float
                        {
                            Array.Copy(BitConverter.GetBytes((Single)fieldValue), data, offset);
                            break;
                        }
                    case TypeCode.Int32:
                        {
                            Array.Copy(BitConverter.GetBytes((Int32)fieldValue), data, offset);
                            break;
                        }
                    case TypeCode.UInt32:
                        {
                            Array.Copy(BitConverter.GetBytes((UInt32)fieldValue), data, offset);
                            break;
                        }
                    case TypeCode.Int16:
                        {
                            Array.Copy(BitConverter.GetBytes((Int16)fieldValue), data, offset);
                            break;
                        }
                    case TypeCode.UInt16:
                        {
                            Array.Copy(BitConverter.GetBytes((UInt16)fieldValue), data, offset);
                            break;
                        }
                    case TypeCode.Int64:
                        {
                            Array.Copy(BitConverter.GetBytes((Int64)fieldValue), data, offset);
                            break;
                        }
                    case TypeCode.UInt64:
                        {
                            Array.Copy(BitConverter.GetBytes((UInt64)fieldValue), data, offset);
                            break;
                        }
                    case TypeCode.Double:
                        {
                            Array.Copy(BitConverter.GetBytes((Double)fieldValue), data, offset);
                            break;
                        }
                    default:
                        {
                            // System.Diagnostics.Debug.Fail("No conversion provided for this type");
                            break;
                        }
                }; // switch

                offset += Marshal.SizeOf(fieldValue);
            } // foreach

            return data;
        } // Swap



        private void button1_Click(object sender, EventArgs e)
        {


            OpenFileDialog openFileDialog1 = new OpenFileDialog();
            openFileDialog1.Filter = "EEPROM.bin|*.bin";
            openFileDialog1.FilterIndex = 2;
            openFileDialog1.RestoreDirectory = true;
            openFileDialog1.InitialDirectory = Path.GetDirectoryName(Application.ExecutablePath);

            if (openFileDialog1.ShowDialog() == DialogResult.OK)
            {
                try
                {
                    StreamReader sr = new StreamReader(openFileDialog1.FileName);
                    BinaryReader br = new BinaryReader(sr.BaseStream);
                    byte[] EEPROM = br.ReadBytes(1024 * 4);
                    br.Close();
                    sr.Close();

                    ArduinoComms port = new ArduinoSTK();

                    if (DialogResult.Yes == MessageBox.Show("is this a 1280?", "", MessageBoxButtons.YesNo))
                    {
                        port = new ArduinoSTK();
                        port.BaudRate = 57600;
                    }
                    else
                    {
                        port = new ArduinoSTKv2();
                        port.BaudRate = 115200;
                    }

            port.DataBits = 8;
            port.StopBits = StopBits.One;
            port.Parity = Parity.None;
            port.DtrEnable = true;

            port.PortName = ArdupilotMega.MainV2.comportname;
            try
            {
                port.Open();

                if (port.connectAP())
                {
                    // waypoints
                    int start = 0;
                    int end = 1024*4;

                        Console.WriteLine(start + " to " + end);
                        port.upload(EEPROM, (short)start, (short)(end - start), (short)start);

                        if (port.keepalive())
                        {
                            // Config

                            if (port.keepalive())
                            {
                                System.Threading.Thread.Sleep(2000);
                                //MessageBox.Show("Upload Completed");
                            }
                            else
                            {
                                MessageBox.Show("Communication Error - WPs wrote but no config");
                            }
                        }
                        else
                        {
                            MessageBox.Show("Communication Error - Bad data");
                        }
                }
                else
                {
                    MessageBox.Show("Communication Error - no connection");
                }
                port.Close();
            }
            catch (Exception ex) { MessageBox.Show("Port in use? " + ex.ToString()); port.Close(); }
                }
                catch (Exception) { MessageBox.Show("Error reading file"); }
            }
        }

        private void BUT_wipeeeprom_Click(object sender, EventArgs e)
        {
            byte[] EEPROM = new byte[4*1024];

            for (int i = 0; i < EEPROM.Length;i++)
            {
                EEPROM[i] = 0xff;
            }

            ArduinoComms port = new ArduinoSTK();

            if (DialogResult.Yes == MessageBox.Show("is this a 1280?", "", MessageBoxButtons.YesNo))
            {
                port = new ArduinoSTK();
                port.BaudRate = 57600;
            }
            else
            {
                port = new ArduinoSTKv2();
                port.BaudRate = 115200;
            }
            port.DataBits = 8;
            port.StopBits = StopBits.One;
            port.Parity = Parity.None;
            port.DtrEnable = true;

            port.PortName = ArdupilotMega.MainV2.comportname;
            try
            {
                port.Open();

                if (port.connectAP())
                {
                    // waypoints
                    int start = 0;
                    int end = 1024*4;

                        Console.WriteLine(start + " to " + end);
                        port.upload(EEPROM, (short)start, (short)(end - start), (short)start);

                        if (port.keepalive())
                        {
                            // Config

                            if (port.keepalive())
                            {
                                System.Threading.Thread.Sleep(2000);
                                //MessageBox.Show("Upload Completed");
                            }
                            else
                            {
                                MessageBox.Show("Communication Error - WPs wrote but no config");
                            }
                        }
                        else
                        {
                            MessageBox.Show("Communication Error - Bad data");
                        }
                }
                else
                {
                    MessageBox.Show("Communication Error - no connection");
                }
                port.Close();
            }
            catch (Exception ex) { MessageBox.Show("Port in use? " + ex.ToString()); port.Close(); }
        }

        private void BUT_flashdl_Click(object sender, EventArgs e)
        {
            byte[] FLASH = new byte[256 * 1024];

            ArduinoComms port = new ArduinoSTK();

            if (DialogResult.Yes == MessageBox.Show("is this a 1280?", "", MessageBoxButtons.YesNo))
            {
                port = new ArduinoSTK();
                port.BaudRate = 57600;
            }
            else
            {
                port = new ArduinoSTKv2();
                port.BaudRate = 115200;
            }
            port.DataBits = 8;
            port.StopBits = StopBits.One;
            port.Parity = Parity.None;
            port.DtrEnable = true;

            port.PortName = ArdupilotMega.MainV2.comportname;
            try
            {
                port.Open();

                System.Threading.Thread.Sleep(100);
                
                if (port.connectAP())
                {
                    // waypoints
                    int start = 0;
                    short length = 0x100;

                    Console.WriteLine(start + " to " + FLASH.Length);

                    while (start < FLASH.Length)
                    {
                        Console.WriteLine("Doing " + length + " at " + start);
                        port.setaddress(start);
                        port.downloadflash(length).CopyTo(FLASH, start);
                        start += length;
                    }

                    StreamWriter sw = new StreamWriter(Path.GetDirectoryName(Application.ExecutablePath) + Path.DirectorySeparatorChar + @"flash.bin", false);
                    BinaryWriter bw = new BinaryWriter(sw.BaseStream);
                    bw.Write(FLASH, 0, FLASH.Length);
                    bw.Close();

                    sw = new StreamWriter(Path.GetDirectoryName(Application.ExecutablePath) + Path.DirectorySeparatorChar + @"flash.hex", false);
                    for (int i = 0; i < FLASH.Length; i += 16)
                    {
                        string add = string.Format("{0:X4}", i);
                        if (i % (0x1000 << 4) == 0)
                        {
                            if (i != 0)
                                sw.WriteLine(":02000002{0:X4}{1:X2}", ((i >> 4) & 0xf000), 0x100 - (2 + 2 + (((i >> 4) & 0xf000) >> 8) & 0xff));
                        }
                        if (add.Length == 5)
                        {
                            add = add.Substring(1);
                        }
                        sw.Write(":{0:X2}{1}00", 16, add);
                        byte ck = (byte)(16 + (i & 0xff) + ((i >> 8) & 0xff));
                        for (int a = 0; a < 16; a++)
                        {
                            ck += FLASH[i + a];
                            sw.Write("{0:X2}", FLASH[i + a]);
                        }
                        sw.WriteLine("{0:X2}", (byte)(0x100 - ck));
                    }

                    sw.Close();

                    Console.WriteLine("Downloaded");
                }
                else
                {
                    MessageBox.Show("Communication Error - no connection");
                }
                port.Close();
            }
            catch (Exception ex) { MessageBox.Show("Port in use? " + ex.ToString()); port.Close(); }
        }

        public int swapend(int value)
        {
            int len = Marshal.SizeOf(value);

            byte[] temp = BitConverter.GetBytes(value);

            Array.Reverse(temp);

            return BitConverter.ToInt32(temp, 0);
        }

        private void BUT_flashup_Click(object sender, EventArgs e)
        {
            byte[] FLASH = new byte[1];

            try
            {
                StreamReader sr = new StreamReader(Path.GetDirectoryName(Application.ExecutablePath) + Path.DirectorySeparatorChar + @"firmware.hex");
                FLASH = readIntelHEXv2(sr);
                sr.Close();

            }
            catch (Exception ex) { MessageBox.Show("Failed to read firmware.hex : " + ex.Message); }
            ArduinoComms port = new ArduinoSTK();

            if (DialogResult.Yes == MessageBox.Show("is this a 1280?", "", MessageBoxButtons.YesNo))
            {
                port = new ArduinoSTK();
                port.BaudRate = 57600;
            }
            else
            {
                port = new ArduinoSTKv2();
                port.BaudRate = 115200;
            }
          
            port.DataBits = 8;
            port.StopBits = StopBits.One;
            port.Parity = Parity.None;
            port.DtrEnable = true;

            try
            {
                port.PortName = ArdupilotMega.MainV2.comportname;

                port.Open();

                

                if (port.connectAP())
                {
                    Console.WriteLine("starting");
                    
                    
                    port.uploadflash(FLASH, 0, FLASH.Length, 0);

                    
                    

                    Console.WriteLine("Uploaded");

                    
                }
                else
                {
                    
                    MessageBox.Show("Communication Error - no connection");
                }
                port.Close();



            }
            catch (Exception ex) {  MessageBox.Show("Check port settings or Port in use? " + ex.ToString()); port.Close(); }

        }

        byte[] readIntelHEX(StreamReader sr)
        {
            byte[] FLASH = new byte[sr.BaseStream.Length / 2];

            int optionoffset = 0;
            int total = 0;

            while (!sr.EndOfStream)
            {

                string line = sr.ReadLine();

                Regex regex = new Regex(@"^:(..)(....)(..)(.*)(..)$"); // length - address - option - data - checksum

                Match match = regex.Match(line);

                int length = Convert.ToInt32(match.Groups[1].Value.ToString(), 16);
                int address = Convert.ToInt32(match.Groups[2].Value.ToString(), 16);
                int option = Convert.ToInt32(match.Groups[3].Value.ToString(), 16);
                Console.WriteLine("len {0} add {1} opt {2}", length, address, option);
                if (option == 0)
                {
                    string data = match.Groups[4].Value.ToString();
                    for (int i = 0; i < length; i++)
                    {
                        byte byte1 = Convert.ToByte(data.Substring(i * 2, 2), 16);
                        FLASH[optionoffset + address] = byte1;
                        address++;
                        if ((optionoffset + address) > total)
                            total = optionoffset + address;
                    }
                }
                else if (option == 2)
                {
                    optionoffset += (int)Convert.ToUInt16(match.Groups[4].Value.ToString(), 16) << 4;
                }
                int checksum = Convert.ToInt32(match.Groups[5].Value.ToString(), 16);
            }

            Array.Resize<byte>(ref FLASH, total);

            return FLASH;
        }

        byte[] readIntelHEXv2(StreamReader sr)
        {
            byte[] FLASH = new byte[sr.BaseStream.Length / 2];

            int optionoffset = 0;
            int total = 0;

            while (!sr.EndOfStream)
            {

                string line = sr.ReadLine();

                if (line.StartsWith(":"))
                {
                    int length = Convert.ToInt32(line.Substring(1, 2), 16);
                    int address = Convert.ToInt32(line.Substring(3, 4), 16);
                    int option = Convert.ToInt32(line.Substring(7, 2), 16);
                    Console.WriteLine("len {0} add {1} opt {2}", length, address, option);

                    if (option == 0)
                    {
                        string data = line.Substring(9, length * 2);
                        for (int i = 0; i < length; i++)
                        {
                            byte byte1 = Convert.ToByte(data.Substring(i * 2, 2), 16);
                            FLASH[optionoffset + address] = byte1;
                            address++;
                            if ((optionoffset + address) > total)
                                total = optionoffset + address;
                        }
                    }
                    else if (option == 2)
                    {
                        optionoffset += (int)Convert.ToUInt16(line.Substring(9, 4), 16) << 4;
                    }
                    int checksum = Convert.ToInt32(line.Substring(line.Length - 2, 2), 16);
                }
                //Regex regex = new Regex(@"^:(..)(....)(..)(.*)(..)$"); // length - address - option - data - checksum
            }

            Array.Resize<byte>(ref FLASH, total);

            return FLASH;
        }

        private void BUT_dleeprom_Click(object sender, EventArgs e)
        {
            ArduinoComms port = new ArduinoSTK();

            if (DialogResult.Yes == MessageBox.Show("is this a 1280?", "", MessageBoxButtons.YesNo))
            {
                port = new ArduinoSTK();
                port.BaudRate = 57600;
            }
            else
            {
                port = new ArduinoSTKv2();
                port.BaudRate = 115200;
            }
            port.DataBits = 8;
            port.StopBits = StopBits.One;
            port.Parity = Parity.None;
            port.DtrEnable = true;

            try
            {
                port.PortName = ArdupilotMega.MainV2.comportname;

                Console.WriteLine("Open Port");
                port.Open();
                Console.WriteLine("Connect AP");
                if (port.connectAP())
                {
                    Console.WriteLine("Download AP");
                    byte[] EEPROM = new byte[1024*4];

                    for (int a = 0; a < 4 * 1024; a += 0x100)
                    {
                        port.setaddress(a);
                        port.download(0x100).CopyTo(EEPROM,a);
                    }
                    Console.WriteLine("Verify State");
                    if (port.keepalive())
                    {
                        StreamWriter sw = new StreamWriter(Path.GetDirectoryName(Application.ExecutablePath) + Path.DirectorySeparatorChar + @"EEPROM.bin");
                        BinaryWriter bw = new BinaryWriter(sw.BaseStream);
                        bw.Write(EEPROM, 0, 1024 * 4);
                        bw.Close();

                       
                    }
                    else
                    {
                        MessageBox.Show("Communication Error - Bad data");
                    }
                }
                else
                {
                    MessageBox.Show("Communication Error - no connection");
                }
                port.Close();
            }
            catch (Exception ex) { MessageBox.Show("Port Error? " + ex.ToString()); if (port != null && port.IsOpen) { port.Close(); } }

        }

        private void BUT_copy1280_Click(object sender, EventArgs e)
        {
            ArduinoSTK port = new ArduinoSTK();
            port.BaudRate = 57600;
            port.DataBits = 8;
            port.StopBits = StopBits.One;
            port.Parity = Parity.None;
            port.DtrEnable = true;

            try
            {
                port.PortName = ArdupilotMega.MainV2.comportname;

                Console.WriteLine("Open Port");
                port.Open();
                Console.WriteLine("Connect AP");
                if (port.connectAP())
                {
                    Console.WriteLine("Download AP");
                    byte[] EEPROM = new byte[1024 * 4];

                    for (int a = 0; a < 4 * 1024; a += 0x100)
                    {
                        port.setaddress(a);
                        port.download(0x100).CopyTo(EEPROM, a);
                    }
                    Console.WriteLine("Verify State");
                    if (port.keepalive())
                    {
                        StreamWriter sw = new StreamWriter(Path.GetDirectoryName(Application.ExecutablePath) + Path.DirectorySeparatorChar + @"EEPROM1280.bin");
                        BinaryWriter bw = new BinaryWriter(sw.BaseStream);
                        bw.Write(EEPROM, 0, EEPROM.Length);
                        bw.Close();

                        Console.WriteLine("Download AP");
                        byte[] FLASH = new byte[1024 * 128];

                        for (int a = 0; a < FLASH.Length; a += 0x100)
                        {
                            port.setaddress(a);
                            port.downloadflash(0x100).CopyTo(FLASH, a);
                        }

                        sw = new StreamWriter(Path.GetDirectoryName(Application.ExecutablePath) + Path.DirectorySeparatorChar + @"FLASH1280.bin");
                        bw = new BinaryWriter(sw.BaseStream);
                        bw.Write(FLASH, 0, FLASH.Length);
                        bw.Close();

                    }
                    else
                    {
                        MessageBox.Show("Communication Error - Bad data");
                    }
                }
                else
                {
                    MessageBox.Show("Communication Error - no connection");
                }
                port.Close();
            }
            catch (Exception ex) { MessageBox.Show("Port Error? " + ex.ToString()); if (port != null && port.IsOpen) { port.Close(); } }
            
        }

        private void BUT_copy2560_Click(object sender, EventArgs e)
        {
            ArduinoSTKv2 port = new ArduinoSTKv2();
            port.BaudRate = 115200;
            port.DataBits = 8;
            port.StopBits = StopBits.One;
            port.Parity = Parity.None;
            port.DtrEnable = true;

            try
            {
                port.PortName = ArdupilotMega.MainV2.comportname;

                Console.WriteLine("Open Port");
                port.Open();
                Console.WriteLine("Connect AP");
                if (port.connectAP())
                {
                    Console.WriteLine("Download AP");
                    byte[] EEPROM = new byte[1024 * 4];

                    for (int a = 0; a < EEPROM.Length; a += 0x100)
                    {
                        port.setaddress(a);
                        port.download(0x100).CopyTo(EEPROM, a);
                    }
                    Console.WriteLine("Verify State");
                    if (port.keepalive())
                    {
                        StreamWriter sw = new StreamWriter(Path.GetDirectoryName(Application.ExecutablePath) + Path.DirectorySeparatorChar + @"EEPROM2560.bin");
                        BinaryWriter bw = new BinaryWriter(sw.BaseStream);
                        bw.Write(EEPROM, 0, EEPROM.Length);
                        bw.Close();

                        Console.WriteLine("Download AP");
                        byte[] FLASH = new byte[1024 * 256];

                        for (int a = 0; a < FLASH.Length; a += 0x100)
                        {
                            port.setaddress(a);
                            port.downloadflash(0x100).CopyTo(FLASH, a);
                        }

                        sw = new StreamWriter(Path.GetDirectoryName(Application.ExecutablePath) + Path.DirectorySeparatorChar + @"FLASH2560.bin");
                        bw = new BinaryWriter(sw.BaseStream);
                        bw.Write(FLASH, 0, FLASH.Length);
                        bw.Close();

                    }
                    else
                    {
                        MessageBox.Show("Communication Error - Bad data");
                    }
                }
                else
                {
                    MessageBox.Show("Communication Error - no connection");
                }
                port.Close();
            }
            catch (Exception ex) { MessageBox.Show("Port Error? " + ex.ToString()); if (port != null && port.IsOpen) { port.Close(); } }
        }

        private void BUT_copyto1280_Click(object sender, EventArgs e)
        {
            ArduinoComms port = new ArduinoSTK();

            port.BaudRate = 57600;

            port.DataBits = 8;
            port.StopBits = StopBits.One;
            port.Parity = Parity.None;
            port.DtrEnable = true;

            StreamReader sr = new StreamReader(Path.GetDirectoryName(Application.ExecutablePath) + Path.DirectorySeparatorChar + @"EEPROM1280.bin");
            BinaryReader br = new BinaryReader(sr.BaseStream);
            byte[] EEPROM = br.ReadBytes(1024 * 4);
            br.Close();
            sr.Close();

            sr = new StreamReader(Path.GetDirectoryName(Application.ExecutablePath) + Path.DirectorySeparatorChar + @"FLASH1280.bin");
            br = new BinaryReader(sr.BaseStream);
            byte[] FLASH = br.ReadBytes(1024 * 128);
            br.Close();
            sr.Close();

            try
            {
                port.PortName = ArdupilotMega.MainV2.comportname;

                port.Open();



                if (port.connectAP())
                {
                    Console.WriteLine("starting");


                    port.uploadflash(FLASH, 0, FLASH.Length, 0);

                    port.upload(EEPROM, 0, (short)EEPROM.Length, 0);


                    Console.WriteLine("Uploaded");


                }
                else
                {

                    MessageBox.Show("Communication Error - no connection");
                }
                port.Close();



            }
            catch (Exception ex) { MessageBox.Show("Check port settings or Port in use? " + ex.ToString()); port.Close(); }
        }

        private void button2_Click(object sender, EventArgs e)
        {
            byte[] FLASH = new byte[1];
                try
                {
                    StreamReader sr = new StreamReader(Path.GetDirectoryName(Application.ExecutablePath) + Path.DirectorySeparatorChar + @"firmware.hex");
                    FLASH = readIntelHEXv2(sr);
                    sr.Close();

                }
                catch (Exception ex) { MessageBox.Show("Failed to read firmware.hex : " + ex.Message); }

                StreamWriter sw = new StreamWriter(Path.GetDirectoryName(Application.ExecutablePath) + Path.DirectorySeparatorChar + @"firmware.bin");
                BinaryWriter bw = new BinaryWriter(sw.BaseStream);
                bw.Write(FLASH, 0, FLASH.Length);
                bw.Close();
            
        }

        private void BUT_geinjection_Click(object sender, EventArgs e)
        {
            GMapControl MainMap = new GMapControl();   
            MainMap.MapType = GMap.NET.MapType.GoogleSatellite;

            MainMap.CacheLocation = Path.GetDirectoryName(Application.ExecutablePath) + "/gmapcache/";

            FolderBrowserDialog fbd = new FolderBrowserDialog();

            try
            {
                fbd.SelectedPath = @"C:\Users\hog\Documents\albany 2011\New folder";
            }
            catch { }

            fbd.ShowDialog();

            if (fbd.SelectedPath != "") {

                string[] files = Directory.GetFiles(fbd.SelectedPath,"*.jpg",SearchOption.AllDirectories);

                foreach (string file in files)
                {
                    Console.WriteLine(DateTime.Now.Millisecond +  " Doing "+ file);
                    Regex reg = new Regex(@"Z([0-9]+)\\([0-9]+)\\([0-9]+)");

                    Match mat = reg.Match(file);

                    int temp = 1 << int.Parse(mat.Groups[1].Value);

                    GMap.NET.GPoint pnt = new GMap.NET.GPoint(int.Parse(mat.Groups[3].Value), int.Parse(mat.Groups[2].Value));

                    BUT_geinjection.Text = file;
                    BUT_geinjection.Refresh();

                    //MainMap.Projection.

                    MemoryStream tile = new MemoryStream();

                    Image Img = Image.FromFile(file);
                    Img.Save(tile,System.Drawing.Imaging.ImageFormat.Jpeg);

                    tile.Seek(0, SeekOrigin.Begin);
                    Console.WriteLine(pnt.X + " " + pnt.Y);

                    Application.DoEvents();

                    MainMap.Manager.ImageCacheLocal.PutImageToCache(tile, GMap.NET.MapType.Custom, pnt, int.Parse(mat.Groups[1].Value)); 

                    Application.DoEvents();
                }
            }
          
        }

        private string getfilepath(int x, int y, int zoom)
        {
            var tileRange = 1 << zoom;

            if (x < 0 || x >= tileRange)
            {
                x = (x % tileRange + tileRange) % tileRange;
            }

            return ("Z" + zoom + "/" + y + "/" + x + ".png");

            //return new GMap.NET.GPoint(x, y);
        }

        private void BUT_clearcustommaps_Click(object sender, EventArgs e)
        {
            GMapControl MainMap = new GMapControl();
            MainMap.MapType = GMap.NET.MapType.GoogleSatellite;

            MainMap.CacheLocation = Path.GetDirectoryName(Application.ExecutablePath) + "/gmapcache/";

            int removed =  ((GMap.NET.CacheProviders.SQLitePureImageCache)MainMap.Manager.ImageCacheLocal).DeleteOlderThan(DateTime.Now, GMap.NET.MapType.Custom);

            MessageBox.Show("Removed "+removed + " images\nshrinking file next");

            GMap.NET.CacheProviders.SQLitePureImageCache.VacuumDb(MainMap.CacheLocation + @"\TileDBv3\en\Data.gmdb");


            Console.WriteLine("Removed {0} images",removed);
        }

        private void BUT_lang_edit_Click(object sender, EventArgs e)
        {
            new resedit.Form1().Show();
        }

        GraphicsWindow _window;
        SceneState _sceneState;
        CameraLookAtPoint _lookCamera;
        CameraFly _flyCamera;
        ClearState _clearState;
        GlobeClipmapTerrain _clipmap;
        HeadsUpDisplay _hud;
        Font _hudFont;
        RayCastedGlobe _globe;
        ClearState _clearDepth;
        Ellipsoid _ellipsoid;

        private void myButton1_Click(object sender, EventArgs e)
        {


            _window = Device.CreateWindow(800, 600, "Chapter 13:  Clipmap Terrain on a Globe");

            _ellipsoid = Ellipsoid.Wgs84;

            WorldWindTerrainSource terrainSource = new WorldWindTerrainSource();
            GMapRestImagery imagery = new GMapRestImagery();
            _clipmap = new GlobeClipmapTerrain(_window.Context, terrainSource, imagery, _ellipsoid, 511);
            _clipmap.HeightExaggeration = 1.0f;



            IList<GridResolution> gridResolutions = new List<GridResolution>();
            gridResolutions.Add(new GridResolution(
                new Interval(0, 1000000, IntervalEndpoint.Closed, IntervalEndpoint.Open),
                new Vector2D(0.005, 0.005)));
            gridResolutions.Add(new GridResolution(
                new Interval(1000000, 2000000, IntervalEndpoint.Closed, IntervalEndpoint.Open),
                new Vector2D(0.01, 0.01)));
            gridResolutions.Add(new GridResolution(
                new Interval(2000000, 20000000, IntervalEndpoint.Closed, IntervalEndpoint.Open),
                new Vector2D(0.05, 0.05)));
            gridResolutions.Add(new GridResolution(
                new Interval(20000000, double.MaxValue, IntervalEndpoint.Closed, IntervalEndpoint.Open),
                new Vector2D(0.1, 0.1)));



            _sceneState = new SceneState();
            _sceneState.DiffuseIntensity = 0.90f;
            _sceneState.SpecularIntensity = 0.05f;
            _sceneState.AmbientIntensity = 0.05f;
            _sceneState.Camera.FieldOfViewY = Math.PI / 3.0;

            _clearState = new ClearState();
            _clearState.Color = Color.White;

            _sceneState.Camera.PerspectiveNearPlaneDistance = 0.000001 * _ellipsoid.MaximumRadius;
            _sceneState.Camera.PerspectiveFarPlaneDistance = 10.0 * _ellipsoid.MaximumRadius;
            _sceneState.SunPosition = new Vector3D(200000, 300000, 200000) * _ellipsoid.MaximumRadius;

            _lookCamera = new CameraLookAtPoint(_sceneState.Camera, _window, _ellipsoid);
            _lookCamera.Range = 1.5 * _ellipsoid.MaximumRadius;

            _globe = new RayCastedGlobe(_window.Context);
            _globe.Shape = _ellipsoid;
            Bitmap bitmap = new Bitmap("NE2_50M_SR_W_4096.jpg");
            _globe.Texture = Device.CreateTexture2D(bitmap, TextureFormat.RedGreenBlue8, false);
            //_globe.GridResolutions = new GridResolutionCollection(gridResolutions);

            _clearDepth = new ClearState();
            _clearDepth.Buffers = ClearBuffers.DepthBuffer | ClearBuffers.StencilBuffer;

            //_window.Keyboard.KeyDown += OnKeyDown;

            _window.Resize += OnResize;
            _window.RenderFrame += OnRenderFrame;
            _window.PreRenderFrame += OnPreRenderFrame;

            _hudFont = new Font("Arial", 16);
            _hud = new HeadsUpDisplay();
            _hud.Color = Color.Blue;

            //_flyCamera = new CameraFly(_sceneState.Camera, _window);
            //_flyCamera.MovementRate = 1200.0;
            //_flyCamera.InputEnabled = true;

            _sceneState.Camera.Target = new Vector3D(115, -35, 100.0);

            _window.Run(30);


        }

        private void OnResize()
        {
            _window.Context.Viewport = new Rectangle(0, 0, _window.Width, _window.Height);
            _sceneState.Camera.AspectRatio = _window.Width / (double)_window.Height;
        }

        private void OnRenderFrame()
        {
            Context context = _window.Context;
            context.Clear(_clearState);

            _globe.Render(context, _sceneState);

            context.Clear(_clearDepth);

            _clipmap.Render(context, _sceneState);

            //_sceneState.Camera.Target = new Vector3D(115000, -350000, 100000.0);

            if (_hud != null)
            {
                //_hud.Render(context, _sceneState);
            }
        }

        private void OnPreRenderFrame()
        {
            Context context = _window.Context;
            _clipmap.PreRender(context, _sceneState);
        }
    }


    public class GMapRestImagery : RasterSource
    {
        GMapControl MainMap;
        public GMapRestImagery()
        {
            //_baseUri = baseUri;

            _levels = new RasterLevel[NumberOfLevels];
            _levelsCollection = new RasterLevelCollection(_levels);

            double deltaLongitude = LevelZeroDeltaLongitudeDegrees;
            double deltaLatitude = LevelZeroDeltaLatitudeDegrees;
            for (int i = 0; i < _levels.Length; ++i)
            {
                int longitudePosts = (int)Math.Round(360.0 / deltaLongitude) * TileLongitudePosts + 1;
                int latitudePosts = (int)Math.Round(180 / deltaLatitude) * TileLatitudePosts + 1;
                _levels[i] = new RasterLevel(this, i, _extent, longitudePosts, latitudePosts, TileLongitudePosts, TileLatitudePosts);
                deltaLongitude /= 2.0;
                deltaLatitude /= 2.0;
            }

            MainMap = new GMapControl();
            MainMap.MapType = GMap.NET.MapType.GoogleSatellite;

            MainMap.CacheLocation = Path.GetDirectoryName(Application.ExecutablePath) + "/gmapcache/";
        }

        public override GeodeticExtent Extent
        {
            get { return _extent; }
        }

        public int TileLongitudePosts
        {
            get { return 256; }
        }

        public int TileLatitudePosts
        {
            get { return 256; }
        }

        public override RasterLevelCollection Levels
        {
            get { return _levelsCollection; }
        }

        public override Texture2D LoadTileTexture(RasterTileIdentifier identifier)
        {
            //if (identifier.Level > 4)
                //return null;
            int level = identifier.Level; // 0 is -180 long
            int longitudeIndex = ((_levels[level].LongitudePosts / _levels[level].LongitudePostsPerTile) / 2 / 2) + identifier.X; //  (_levels[level].LongitudePosts / _levels[level].LongitudePostsPerTile) - 
            int latitudeIndex = identifier.Y; // (_levels[level].LatitudePosts / _levels[level].LatitudePostsPerTile) - 

            int damn = (1 << level) - latitudeIndex - 1;

            Console.WriteLine(" z {0} lat {1} lon {2} ", level, damn, longitudeIndex);

            GMap.NET.PureImage img = MainMap.Manager.ImageCacheLocal.GetImageFromCache(GMap.NET.MapType.GoogleSatellite, new GMap.NET.GPoint(longitudeIndex, damn), level);

            Application.DoEvents();

            try
            {
                Bitmap bitmap = new Bitmap(new Bitmap(img.Data), 256, 256);
                Graphics e = Graphics.FromImage(bitmap);
                e.DrawString(level + " " +longitudeIndex + "," + damn, new Font("Arial", 20), Brushes.White, new PointF(0, 0));

                return Device.CreateTexture2DRectangle(bitmap, TextureFormat.RedGreenBlue8);
            }
            catch {
                try
                {
                    return null; 
                }
                catch { return null; }
            }
        }

        private Uri _baseUri;
        private GeodeticExtent _extent = new GeodeticExtent(-0, -90, 360, 90);
        private int _tilesLoaded;
        private RasterLevel[] _levels;
        private RasterLevelCollection _levelsCollection;

        private const int NumberOfLevels = 20;
        private const double LevelZeroDeltaLongitudeDegrees = 180;
        private const double LevelZeroDeltaLatitudeDegrees = 180;
    }
}
