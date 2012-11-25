using System;
using System.Collections.Generic;
using System.Collections;
using System.ComponentModel;
using System.Drawing;
using System.Data;
using System.Linq;
using System.Text;
using System.Windows.Forms;
using System.Xml;
using ArdupilotMega.Controls.BackstageView;
using ArdupilotMega.Utilities;

namespace ArdupilotMega.Controls
{
    public partial class ConfigPanel : UserControl, IActivate
    {
        /// <summary>
        /// store temp pending changes
        /// </summary>
        Hashtable _changed = new Hashtable();
        // store linked param options
        Hashtable _linkedParams = new Hashtable();

        public ConfigPanel(string XMLFile)
        {
            InitializeComponent();

            LoadXML(XMLFile);
        }

        /// <summary>
        /// fill numeric up down boxs with there value
        /// </summary>
        public void PopulateData()
        {
            // process hashdefines and update display
            foreach (string value in MainV2.comPort.param.Keys)
            {
                if (value == null || value == "") // older ap version have a null param
                    continue;
                Control[] text = this.Controls.Find(value, true);
                foreach (Control ctl in text)
                {
                    try
                    {
                        float numbervalue = (float)MainV2.comPort.param[value];

                        MAVLink.modifyParamForDisplay(true, value, ref numbervalue);

                        NumericUpDown thisctl = ((NumericUpDown)ctl);
                        thisctl.Validated -= thisctl_Validated;
                        thisctl.ValueChanged -= thisctl_Validated;
                        thisctl.Value = (decimal)numbervalue;
                        thisctl.Enabled = true;
                        thisctl.Validated += thisctl_Validated;
                        thisctl.ValueChanged += thisctl_Validated;
                    }
                    catch (Exception ex) { Console.WriteLine(ex.ToString()); }
                }
            }
        }

        void thisctl_Validated(object sender, EventArgs e)
        {
            string param = ((NumericUpDown)sender).Name;

            ((Control)sender).BackColor = Color.Green;

            foreach (string item in (List<string>)_linkedParams[param])
            {
                float value = (float)((NumericUpDown)sender).Value;
                MAVLink.modifyParamForDisplay(false, param, ref value);
                _changed[item] = value;
            }
        }

        /// <summary>
        /// disables all NumericUpDown box's
        /// </summary>
        /// <param name="inctl"></param>
        void disableNumericUpDownControls(Control inctl)
        {
            foreach (Control ctl in inctl.Controls)
            {
                if (ctl.Controls.Count > 0)
                {
                    disableNumericUpDownControls(ctl);
                }
                if (ctl.GetType() == typeof(NumericUpDown))
                {
                    ctl.Enabled = false;
                }
            }
        }

        /// <summary>
        /// The template xml for the screen
        /// </summary>
        /// <param name="FileName"></param>
        public void LoadXML(string FileName)
        {
            int x = 20;
            int y = 0;

            int optionx = 300;
            int optiony = 0;

            string name = "";
            List<string> paramname = new List<string>();
            double rangemin = 0;
            double rangemax = 10;
            double step = 0.001;

            using (XmlReader reader = XmlReader.Create(FileName))
            {
                while (reader.Read())
                {
                    switch (reader.Name.ToUpper())
                    {
                        case "ITEM":

                            break;
                        case "HEAD":
                            y += 30;
                            string heading = reader.ReadString();

                            Label lbl = new Label();
                            lbl.AutoSize = true;
                            lbl.Text = heading;
                            lbl.Location = new Point(x,y);
                            lbl.Font = new Font(FontFamily.GenericSansSerif, 15,FontStyle.Bold);

                            this.Controls.Add(lbl);

                            ArdupilotMega.Controls.MyButton but = new ArdupilotMega.Controls.MyButton();

                            but.Text = "Save";
                            but.Location = new Point(optionx + 100, y);
                            but.Click += new EventHandler(but_Click);
                            this.Controls.Add(but);

                            y = lbl.Location.Y + lbl.Height + 10;

                            LineSeparator ls = new LineSeparator();

                            ls.Width = this.Width - 40;
                            ls.Location = new Point(x,y);

                            ls.Anchor = AnchorStyles.Top | AnchorStyles.Left | AnchorStyles.Right;

                            this.Controls.Add(ls);

                            y = ls.Location.Y + ls.Height;

                            break;
                        case "SUBHEAD":
                            y += 15;

                            optiony = y;
                            optionx = 300;
                            string subhead = reader.ReadString();

                            Label lbl2 = new Label();
                            lbl2.AutoSize = true;
                            lbl2.Text = subhead;
                            lbl2.Location = new Point(x,y);
                            lbl2.Font = new Font(FontFamily.GenericSansSerif, 10, FontStyle.Bold);

                            this.Controls.Add(lbl2);

                            y = lbl2.Location.Y + lbl2.Height;

                            break;
                        case "DESC":
                            y += 2;
                            string desc = reader.ReadString();

                            Label lbl3 = new Label();
                            lbl3.AutoSize = true;
                            lbl3.Text = AddNewLinesToText(desc);
                            lbl3.Location = new Point(x,y);
                            lbl3.Font = new Font(FontFamily.GenericSansSerif, 8, FontStyle.Bold);

                            this.Controls.Add(lbl3);

                            y = lbl3.Location.Y + lbl3.Height;

                            break;
                        case "FIELDS":
                            if (reader.NodeType == XmlNodeType.EndElement)
                            {
                                if (optiony > y)
                                    y = optiony;
                            }
                            break;
                        case "FIELD":
                            if (reader.NodeType == XmlNodeType.EndElement)
                            {
                                if (name == "" || paramname.Count == 0)
                                    break;

                                Label lbl5 = new Label();
                                //lbl5.AutoSize = true;
                                lbl5.Text = name;
                                lbl5.Location = new Point(optionx, optiony);
                                lbl5.Size = new System.Drawing.Size(90,20);
                                lbl5.Font = new Font(FontFamily.GenericSansSerif, 8, FontStyle.Bold);
                                lbl5.TextAlign = ContentAlignment.MiddleRight;

                                this.Controls.Add(lbl5);

                                NumericUpDown nud = new NumericUpDown();
                                nud.Location = new Point(optionx + 100, optiony);
                                nud.Size = new System.Drawing.Size(78,20);
                                nud.Maximum = (decimal)rangemax;
                                nud.Minimum = (decimal)rangemin;
                                nud.Increment = (decimal)step;
                                nud.DecimalPlaces = (int)(step.ToString().Length - step.ToString(new System.Globalization.CultureInfo("en-US")).IndexOf('.') - 1);
                                nud.Name = paramname[0];

                                this.Controls.Add(nud);

                                optiony += nud.Height;

                                _linkedParams[paramname[0]] = paramname;

                            }
                            else
                            {
                                name = "";
                                paramname = new List<string>();
                                rangemax = 10;
                                rangemin = 0;
                                step = 0.001;
                            }
                            break;
                        case "NAME":
                            name = reader.ReadString();
                            break;
                        case "PARAMNAME":
                            paramname.Add(reader.ReadString());
                            break;
                        case "RANGEMIN":
                            rangemin = double.Parse(reader.ReadString(), new System.Globalization.CultureInfo("en-US"));
                            break;
                        case "RANGEMAX":
                            rangemax = double.Parse(reader.ReadString(), new System.Globalization.CultureInfo("en-US"));
                            break;
                        case "STEP":
                            step = double.Parse(reader.ReadString(), new System.Globalization.CultureInfo("en-US"));
                            break;

                    }
                }
            }

            ThemeManager.ApplyThemeTo(this);

            disableNumericUpDownControls(this);
        }

        void but_Click(object sender, EventArgs e)
        {

            Hashtable temp = (Hashtable)_changed.Clone();

            foreach (string value in temp.Keys)
            {
                try
                {
                    MainV2.comPort.setParam(value, (float)_changed[value]);
                    _changed.Remove(value);

                    try
                    {
                        // un green control as well
                        Control[] text = this.Controls.Find(value, true);
                        if (text.Length > 0)
                        {
                            ((Control)text[0]).BackColor = Color.FromArgb(0x43, 0x44, 0x45);
                        }
                    }
                    catch { }
                }
                catch { CustomMessageBox.Show("Set " + value + " Failed"); }
            }
        }

        // from http://stackoverflow.com/questions/2512781/winforms-big-paragraph-tooltip/2512895#2512895
        private static int maximumSingleLineTooltipLength = 40;

        private static string AddNewLinesToText(string text)
        {
            if (text.Length < maximumSingleLineTooltipLength)
                return text;
            int lineLength = maximumSingleLineTooltipLength;
            StringBuilder sb = new StringBuilder();
            int currentLinePosition = 0;
            for (int textIndex = 0; textIndex < text.Length; textIndex++)
            {
                // If we have reached the target line length and the next      
                // character is whitespace then begin a new line.   
                if (currentLinePosition >= lineLength &&
                    char.IsWhiteSpace(text[textIndex]))
                {
                    sb.Append(Environment.NewLine);
                    currentLinePosition = 0;
                }
                // reset line lnegth counter on existing new line
                if (text[textIndex] == Environment.NewLine[Environment.NewLine.Length - 1])
                {
                    currentLinePosition = 1;
                    // skip space at line start
                    if (textIndex == 0)
                        currentLinePosition = 0;
                }
                // If we have just started a new line, skip all the whitespace.    
                if (currentLinePosition == 0)
                    while (textIndex < text.Length && char.IsWhiteSpace(text[textIndex]))
                        textIndex++;
                // Append the next character.     
                if (textIndex < text.Length) sb.Append(text[textIndex]);
                currentLinePosition++;
            }
            return sb.ToString();
        }

        private void ConfigPanel_Load(object sender, EventArgs e)
        {
            PopulateData();
        }

        public void Activate()
        {
            PopulateData();
        }
    }
}