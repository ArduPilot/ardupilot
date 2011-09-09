using System;
using System.Collections.Generic;
using System.ComponentModel;
using System.Data;
using System.Drawing;
using System.Text;
using System.Windows.Forms;
using System.Xml;

namespace ArdupilotMega
{
    public partial class Gentemp : Form
    {
        public Gentemp()
        {
            InitializeComponent();

            string item = "";
            string name = "";
            int locx = 0;
            int locy = 0;
            int sizeh = 0;
            int sizew = 0;
            string text = "";
            string tooltip = "";
            string parent = "";
            string type = "";


            XmlTextReader scriptXmlReader = new XmlTextReader("APM_config_screen.xml");
            scriptXmlReader.WhitespaceHandling = WhitespaceHandling.None;

            while (scriptXmlReader.Read())
            {
                switch (scriptXmlReader.NodeType)
                {
                    case XmlNodeType.Element:
                        item = scriptXmlReader.Name;
                        break;

                    case XmlNodeType.Text:
                        switch (item)
                        {
                            case "Name":
                                name = scriptXmlReader.Value;
                                break;
                            case "Location.X":
                                locx = int.Parse(scriptXmlReader.Value);
                                break;
                            case "Location.Y":
                                locy = int.Parse(scriptXmlReader.Value);
                                break;
                            case "Size.Width":
                                sizew = int.Parse(scriptXmlReader.Value);
                                break;
                            case "Size.Height":
                                sizeh = int.Parse(scriptXmlReader.Value);
                                break;
                            case "Text":
                                text = scriptXmlReader.Value;
                                break;
                            case "ToolTip":
                                tooltip = scriptXmlReader.Value;
                                break;
                            case "Parent":
                                parent = scriptXmlReader.Value;
                                break;
                            case "Type":
                                type = scriptXmlReader.Value;
                                break;
                        }
                        break;

                    case XmlNodeType.EndElement:
                        item = scriptXmlReader.Name;
                        if (item == "Object")
                        {
                            switch (type) {
                                case "System.Windows.Forms.Button":
                                    Button but = new Button();
                                    but.Parent = FindControlByName(parent);
                                    but.Name = name;
                                    but.Text = text;
                                    but.Size = new Size(sizew,sizeh);
                                    but.Location = new Point(locx,locy);
                                    but.Parent.Controls.Add(but);
                                    break;
                                case "System.Windows.Forms.Label":
                                    Label lbl = new Label();
                                    lbl.Parent = FindControlByName(parent);
                                    lbl.Name = name;
                                    lbl.Text = text;
                                    lbl.Size = new Size(sizew, sizeh);
                                    lbl.Location = new Point(locx, locy);

                                    lbl.Parent.Controls.Add(lbl);
                                    break;
                                case "System.Windows.Forms.GroupBox":
                                    GroupBox Grp = new GroupBox();
                                    Grp.Parent = FindControlByName(parent);
                                    Grp.Name = name;
                                    Grp.Text = text;
                                    Grp.Size = new Size(sizew, sizeh);
                                    Grp.Location = new Point(locx, locy);

                                    Grp.Parent.Controls.Add(Grp);
                                    break;
                                case "System.Windows.Forms.TextBox":
                                    TextBox TXT = new TextBox();
                                    TXT.Parent = FindControlByName(parent);
                                    TXT.Name = name;
                                    TXT.Text = text;
                                    TXT.Size = new Size(sizew, sizeh);
                                    TXT.Location = new Point(locx, locy);

                                    TXT.Parent.Controls.Add(TXT);
                                    break;
                                case "System.Windows.Forms.CheckBox":
                                    CheckBox CHK = new CheckBox();
                                    CHK.Parent = FindControlByName(parent);
                                    CHK.Name = name;
                                    CHK.Text = text;
                                    CHK.Size = new Size(sizew, sizeh);
                                    CHK.Location = new Point(locx, locy);

                                    CHK.Parent.Controls.Add(CHK);
                                    break;
                                case "System.Windows.Forms.DomainUpDown":
                                    DomainUpDown DUD = new DomainUpDown();
                                    DUD.Parent = FindControlByName(parent);
                                    DUD.Name = name;
                                    DUD.Text = text;
                                    DUD.Size = new Size(sizew, sizeh);
                                    DUD.Location = new Point(locx, locy);

                                    DUD.Parent.Controls.Add(DUD);
                                    break;
                                case "System.Windows.Forms.ComboBox":
                                    ComboBox CMB = new ComboBox();
                                    CMB.Parent = FindControlByName(parent);
                                    CMB.Name = name;
                                    CMB.Text = text;
                                    CMB.Size = new Size(sizew, sizeh);
                                    CMB.Location = new Point(locx, locy);

                                    CMB.Parent.Controls.Add(CMB);
                                    break;
                            }
                        }
                        break;

                    default:

                        break;
                }
            }
        }

        private Control FindControlByName(string name)
        {
            foreach (Control c in this.Controls)
            {
                if (c.Name == name)
                    return c; 
            }
            return this;
        }
    }
}
