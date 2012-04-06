using System;
using System.Collections;
using System.Collections.Generic;
using System.ComponentModel;
using System.Drawing;
using System.Data;
using System.IO;
using System.Linq;
using System.Text;
using System.Windows.Forms;
using log4net;

namespace ArdupilotMega.GCSViews.ConfigurationView
{
    public partial class ConfigRawParams : UserControl
    {
        private static readonly ILog log =
          LogManager.GetLogger(System.Reflection.MethodBase.GetCurrentMethod().DeclaringType);

        // Changes made to the params between writing to the copter
        readonly Hashtable _changes = new Hashtable();

        // ?
        internal bool startup = true;


        public ConfigRawParams()
        {
            InitializeComponent();
        }

        Hashtable loadParamFile(string Filename)
        {
            Hashtable param = new Hashtable();

            StreamReader sr = new StreamReader(Filename);
            while (!sr.EndOfStream)
            {
                string line = sr.ReadLine();

                if (line.Contains("NOTE:"))
                    CustomMessageBox.Show(line, "Saved Note");

                if (line.StartsWith("#"))
                    continue;

                string[] items = line.Split(new char[] { ' ', ',', '\t' }, StringSplitOptions.RemoveEmptyEntries);

                if (items.Length != 2)
                    continue;

                string name = items[0];
                float value = float.Parse(items[1], new System.Globalization.CultureInfo("en-US"));

                MAVLink.modifyParamForDisplay(true, name, ref value);

                if (name == "SYSID_SW_MREV")
                    continue;
                if (name == "WP_TOTAL")
                    continue;
                if (name == "CMD_TOTAL")
                    continue;
                if (name == "FENCE_TOTAL")
                    continue;
                if (name == "SYS_NUM_RESETS")
                    continue;
                if (name == "ARSPD_OFFSET")
                    continue;
                if (name == "GND_ABS_PRESS")
                    continue;
                if (name == "GND_TEMP")
                    continue;
                if (name == "CMD_INDEX")
                    continue;
                if (name == "LOG_LASTFILE")
                    continue;

                param[name] = value;
            }
            sr.Close();

            return param;
        }

        private void BUT_load_Click(object sender, EventArgs e)
        {
            var ofd = new OpenFileDialog
                          {
                              AddExtension = true,
                              DefaultExt = ".param",
                              RestoreDirectory = true,
                              Filter = "Param List|*.param;*.parm"
                          };
            var dr = ofd.ShowDialog();

            if (dr == DialogResult.OK)
            {
                Hashtable param2 = loadParamFile(ofd.FileName);

                foreach (string name in param2.Keys)
                {
                    string value = param2[name].ToString();
                    // set param table as well
                    foreach (DataGridViewRow row in Params.Rows)
                    {
                        if (name == "SYSID_SW_MREV")
                            continue;
                        if (name == "WP_TOTAL")
                            continue;
                        if (name == "CMD_TOTAL")
                            continue;
                        if (name == "FENCE_TOTAL")
                            continue;
                        if (name == "SYS_NUM_RESETS")
                            continue;
                        if (name == "ARSPD_OFFSET")
                            continue;
                        if (name == "GND_ABS_PRESS")
                            continue;
                        if (name == "GND_TEMP")
                            continue;
                        if (name == "CMD_INDEX")
                            continue;
                        if (name == "LOG_LASTFILE")
                            continue;
                        if (row.Cells[0].Value.ToString() == name)
                        {
                            if (row.Cells[1].Value.ToString() != value.ToString())
                                row.Cells[1].Value = value;
                            break;
                        }
                    }
                }
            }
        }

        private void BUT_save_Click(object sender, EventArgs e)
        {
            var sfd = new SaveFileDialog
                          {
                              AddExtension = true,
                              DefaultExt = ".param",
                              RestoreDirectory = true,
                              Filter = "Param List|*.param;*.parm"
                          };

            var dr = sfd.ShowDialog();
            if (dr == DialogResult.OK)
            {
                StreamWriter sw = new StreamWriter(sfd.OpenFile());
                string input = DateTime.Now + " Frame : + | Arducopter Kit | Kit motors";
                if (MainV2.APMFirmware == MainV2.Firmwares.ArduPlane)
                {
                    input = DateTime.Now + " Plane: Skywalker";
                }
                Common.InputBox("Custom Note", "Enter your Notes/Frame Type etc", ref input);
                if (input != "")
                    sw.WriteLine("NOTE: " + input.Replace(',', '|'));
                foreach (DataGridViewRow row in Params.Rows)
                {
                    float value = float.Parse(row.Cells[1].Value.ToString());

                    MAVLink.modifyParamForDisplay(false, row.Cells[0].Value.ToString(), ref value);

                    sw.WriteLine(row.Cells[0].Value.ToString() + "," + value.ToString(new System.Globalization.CultureInfo("en-US")));
                }
                sw.Close();
            }
        }

        private void BUT_writePIDS_Click(object sender, EventArgs e)
        {
            var temp = (Hashtable)_changes.Clone();

            foreach (string value in temp.Keys)
            {
                try
                {
                    MainV2.comPort.setParam(value, (float)_changes[value]);

                    try
                    {
                        // set control as well
                        var textControls = this.Controls.Find(value, true);
                        if (textControls.Length > 0)
                        {
                            textControls[0].BackColor = Color.FromArgb(0x43, 0x44, 0x45);
                        }
                    }
                    catch
                    {
                        
                    }

                    try
                    {
                        // set param table as well
                        foreach (DataGridViewRow row in Params.Rows)
                        {
                            if (row.Cells[0].Value.ToString() == value)
                            {
                                row.Cells[1].Style.BackColor = Color.FromArgb(0x43, 0x44, 0x45);
                                _changes.Remove(value);
                                break;
                            }
                        }
                    }
                    catch { }

                }
                catch
                {
                    CustomMessageBox.Show("Set " + value + " Failed");
                }
            }
        }


        private void BUT_compare_Click(object sender, EventArgs e)
        {
            Hashtable param2 = new Hashtable();

            var ofd = new OpenFileDialog
                          {
                              AddExtension = true,
                              DefaultExt = ".param",
                              RestoreDirectory = true,
                              Filter = "Param List|*.param;*.parm"
                          };

            var dr = ofd.ShowDialog();
            if (dr == DialogResult.OK)
            {
                param2 = loadParamFile(ofd.FileName);

                int fixme;
                //var paramCompareForm = new ParamCompare((Form)this, MainV2.comPort.param, param2);
                
                //ThemeManager.ApplyThemeTo(paramCompareForm);
                //paramCompareForm.ShowDialog();
            }
        }


        private void BUT_rerequestparams_Click(object sender, EventArgs e)
        {
            if (!MainV2.comPort.BaseStream.IsOpen)
                return;

            ((Control)sender).Enabled = false;
            
            try
            {
                MainV2.comPort.getParamList();
            }
            catch (Exception ex)
            {
                log.Error("Exception getting param list", ex);
                CustomMessageBox.Show("Error: getting param list");
            }


            ((Control)sender).Enabled = true;
            
            startup = true;
            
            // Todo: this populates or the combos etc and what not. This shoudl prob be a bsv button
            
        }

        private void ConfigRawParams_Load(object sender, EventArgs e)
        {

        }

        private void ConfigRawParams_ControlRemoved(object sender, ControlEventArgs e)
        {

        }

        private void ConfigRawParams_ControlAdded(object sender, ControlEventArgs e)
        {

        }

    }
}
