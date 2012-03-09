using System;
using System.Collections.Generic;
using System.ComponentModel;
using System.Data;
using System.Drawing;
using System.Linq;
using System.Text;
using System.Windows.Forms;
using System.Resources;
using System.Collections;
using System.Globalization;
using System.IO;
using System.Net;
using System.Text.RegularExpressions;
using System.Reflection;
using System.Xml;

namespace resedit
{
    public partial class Form1 : Form
    {
        public Form1()
        {
            InitializeComponent();

            List<string> list = new List<string>();

            list.Add("");

            CultureInfo[] temp = System.Globalization.CultureInfo.GetCultures(CultureTypes.AllCultures);

                foreach (CultureInfo cul in temp) 
                {
                    list.Add(cul.DisplayName + " " + cul.Name);
                }

            list.Sort();

            comboBox1.DataSource = list;
        }

        private void Form1_Load(object sender, EventArgs e)
        {
            Assembly thisAssembly = Assembly.GetExecutingAssembly();

            string[] test = Assembly.GetExecutingAssembly().GetManifestResourceNames();

            foreach (string file in test)
            {
                Stream rgbxml = thisAssembly.GetManifestResourceStream(
            file);
                try
                {
                    ResourceReader res = new ResourceReader(rgbxml);
                    IDictionaryEnumerator dict = res.GetEnumerator();
                    while (dict.MoveNext())
                    {
                        Console.WriteLine("   {0}: '{1}' (Type {2})",
                                          dict.Key, dict.Value, dict.Value.GetType().Name);

                        if (dict.Key.ToString().EndsWith(".ToolTip") || dict.Key.ToString().EndsWith(".Text") || dict.Key.ToString().EndsWith("HeaderText") || dict.Key.ToString().EndsWith("ToolTipText"))
                        {
                            dataGridView1.Rows.Add();

                            dataGridView1.Rows[dataGridView1.RowCount - 1].Cells[colFile.Index].Value = System.IO.Path.GetFileName(file);
                            dataGridView1.Rows[dataGridView1.RowCount - 1].Cells[colInternal.Index].Value = dict.Key.ToString();
                            dataGridView1.Rows[dataGridView1.RowCount - 1].Cells[colEnglish.Index].Value = dict.Value.ToString();
                            dataGridView1.Rows[dataGridView1.RowCount - 1].Cells[colOtherLang.Index].Value = dict.Value.ToString();
                        }

                    }

                } catch {}
            }
        }

        private void button1_Click(object sender, EventArgs e)
        {
            FolderBrowserDialog fbd = new FolderBrowserDialog();
            fbd.SelectedPath = System.IO.Path.GetDirectoryName(Application.ExecutablePath);
            fbd.ShowDialog();
            if (fbd.SelectedPath != "")
            {
                dataGridView1.Rows.Clear();
                string[] files = System.IO.Directory.GetFiles(fbd.SelectedPath, "*.resx", System.IO.SearchOption.AllDirectories);


                string ci = "";
                CultureInfo[] temp = System.Globalization.CultureInfo.GetCultures(CultureTypes.AllCultures);

                foreach (CultureInfo cul in temp)
                {
                    if ((cul.DisplayName + " " + cul.Name) == comboBox1.Text)
                    {
                        Console.WriteLine(cul.Name);
                        ci = cul.Name;
                    }
                }


                foreach (string file in files)
                {
                    // load only file of the slected lang
                    if (!file.ToLower().Contains(ci.ToString().ToLower() + ".resx"))
                        continue;

                    // dont load and tralations if no lang selected
                    if (file.ToLower().Contains("translation") && comboBox1.Text == "")
                        continue;

                    // must be a resx
                    if (!file.ToLower().EndsWith(".resx"))
                        continue;



                    ResXResourceReader reader = new ResXResourceReader(file);
                    Console.WriteLine(reader);

                    reader.BasePath = fbd.SelectedPath + System.IO.Path.DirectorySeparatorChar +"Resources";

                    try
                    {
                        foreach (DictionaryEntry entry in reader)
                        {

                            if (entry.Key.ToString().EndsWith(".ToolTip") || entry.Key.ToString().EndsWith(".Text") || entry.Key.ToString().EndsWith("HeaderText") || entry.Key.ToString().EndsWith("ToolTipText"))
                            {
                                dataGridView1.Rows.Add();

                                dataGridView1.Rows[dataGridView1.RowCount - 1].Cells[colFile.Index].Value = System.IO.Path.GetFileName(file);
                                dataGridView1.Rows[dataGridView1.RowCount - 1].Cells[colInternal.Index].Value = entry.Key.ToString();
                                dataGridView1.Rows[dataGridView1.RowCount - 1].Cells[colEnglish.Index].Value = entry.Value.ToString();
                                dataGridView1.Rows[dataGridView1.RowCount - 1].Cells[colOtherLang.Index].Value = entry.Value.ToString();
                            }

                        }
                    }
                    catch (Exception ex) { Console.WriteLine(ex.ToString()); }
                }
            }
        }

        private void button2_Click(object sender, EventArgs e)
        {
            string ci = "";
            CultureInfo[] temp = System.Globalization.CultureInfo.GetCultures(CultureTypes.AllCultures);

            foreach (CultureInfo cul in temp)
            {
                if ((cul.DisplayName + " " + cul.Name) == comboBox1.Text)
                {
                    Console.WriteLine(cul.Name);
                    ci = cul.Name;
                }
            }

            string fname = "";

            ResXResourceWriter writer = null;

            System.IO.Directory.CreateDirectory("translation");

            StreamWriter sw = new StreamWriter("translation/output.html");
            sw.Write("<html><body><table>");

            foreach (DataGridViewRow row in dataGridView1.Rows)
            {
                try
                {
                    if (row.Cells[colFile.Index].Value.ToString() != fname)
                    {
                        if (writer != null)
                            writer.Close();
                        writer = new ResXResourceWriter("translation/" + row.Cells[colFile.Index].Value.ToString().Replace(".resources", "." + ci + ".resx"));
                    }

                    writer.AddResource(row.Cells[colInternal.Index].Value.ToString(), row.Cells[colOtherLang.Index].Value.ToString());

                    fname = row.Cells[colFile.Index].Value.ToString();
                }
                catch { }
                try
                {
                    sw.Write("<tr><td>" + row.Cells[colFile.Index].Value.ToString() + "</td><td>" + row.Cells[colInternal.Index].Value.ToString() + "</td><td>" + row.Cells[colOtherLang.Index].Value.ToString() + "</td></tr>");
                }
                catch (Exception ex) { try { CustomMessageBox.Show("Failed to save " + row.Cells[colOtherLang.Index].Value.ToString() + " " + ex.ToString()); } catch { } }
            }
            if (writer != null)
                writer.Close();
            sw.Write("</table></html>");
            sw.Close();

            CustomMessageBox.Show("Saved");
        }

        private void button3_Click(object sender, EventArgs e)
        {
            if (!File.Exists("translation/output.html"))
            {
                CustomMessageBox.Show("No existing translation has been done");
                return;
            }

            StreamReader sr1 = new StreamReader("translation/output.html");

            string file = sr1.ReadToEnd();


            Regex regex = new Regex("<tr><td>([^<]*)</td><td>([^<]*)</td><td>([^<]*)</td></tr>",RegexOptions.Multiline | RegexOptions.IgnoreCase);

            MatchCollection matches = regex.Matches(file);

            int a = 0;

            foreach (Match mat in matches)
            {
                foreach (DataGridViewRow row in dataGridView1.Rows)
                {
                    if (mat.Groups.Count == 4)
                    {
                        if (row.Cells[0].Value.ToString() == mat.Groups[1].Value.ToString() && row.Cells[1].Value.ToString() == mat.Groups[2].Value.ToString())
                        {
                            row.Cells[3].Value = mat.Groups[3].Value.ToString();
                            a++;
                        }
                    }
                }
            }

            sr1.Close();

            CustomMessageBox.Show("Modified "+a+" entries");
        }

        private void comboBox1_SelectedIndexChanged(object sender, EventArgs e)
        {
            string ci = "";
            CultureInfo[] temp = System.Globalization.CultureInfo.GetCultures(CultureTypes.AllCultures);

            foreach (CultureInfo cul in temp)
            {
                if ((cul.DisplayName + " " + cul.Name) == comboBox1.Text)
                {
                    Console.WriteLine(cul.Name);
                    ci = cul.Name;
                }
            }

            Assembly thisAssembly;

            try
            {
                thisAssembly = Assembly.LoadFile(Path.GetDirectoryName(Application.ExecutablePath) + Path.DirectorySeparatorChar + ci + Path.DirectorySeparatorChar + "ArdupilotMegaPlanner.resources.dll");
            }
            catch { return; }

            string[] test = thisAssembly.GetManifestResourceNames();

            Encoding unicode = Encoding.Unicode;

            foreach (string file in test)
            {
                Stream rgbxml = thisAssembly.GetManifestResourceStream(
            file);
                try
                {
                    ResourceReader res = new ResourceReader(rgbxml);
                    IDictionaryEnumerator dict = res.GetEnumerator();
                    while (dict.MoveNext())
                    {
                        Console.WriteLine("   {0}: '{1}' (Type {2})",
                                          dict.Key, dict.Value, dict.Value.GetType().Name);

                        string thing = (string)dict.Value;

//                            dataGridView1.Rows[0].Cells[colOtherLang.Index].Value = dict.Value.ToString();
                        foreach (DataGridViewRow row in dataGridView1.Rows)
                        {
                            string t2 = file.Replace(ci + ".", "");

                            if (row.Cells[0].Value.ToString() == t2 && row.Cells[1].Value.ToString() == dict.Key.ToString())
                            {
                                row.Cells[3].Value = thing;
                            }


                        }
                    }

                }
                catch { }
            }

            CustomMessageBox.Show("Loaded Existing");
        }
    }
}

