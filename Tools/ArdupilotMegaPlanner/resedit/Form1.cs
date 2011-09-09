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

namespace resedit
{
    public partial class Form1 : Form
    {
        public Form1()
        {
            InitializeComponent();

            List<string> list = new List<string>();

            list.Add("");

            CultureInfo[] temp = System.Globalization.CultureInfo.GetCultures(CultureTypes.FrameworkCultures);

                foreach (CultureInfo cul in temp) 
                {
                    list.Add(cul.DisplayName);
                }

            list.Sort();

            comboBox1.DataSource = list;
        }

        private void Form1_Load(object sender, EventArgs e)
        {

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
                    if (cul.DisplayName == comboBox1.Text)
                    {
                        Console.WriteLine(cul.Name);
                        ci = cul.Name;
                    }
                }


                foreach (string file in files)
                {
                    if (!file.ToLower().Contains(ci.ToString().ToLower() + ".resx"))
                        continue;

                    if (!file.ToLower().EndsWith(".resx"))
                        continue;



                    ResXResourceReader reader = new ResXResourceReader(file);
                    Console.WriteLine(reader);
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
                if (cul.DisplayName == comboBox1.Text)
                {
                    Console.WriteLine(cul.Name);
                    ci = cul.Name;
                }
            }

            string fname = "";

            ResXResourceWriter writer = null;

            System.IO.Directory.CreateDirectory("translation");

            foreach (DataGridViewRow row in dataGridView1.Rows)
            {
                if (row.Cells[colFile.Index].Value.ToString() != fname)
                {
                    if (writer !=null)
                        writer.Close();
                    writer = new ResXResourceWriter("translation/" + row.Cells[colFile.Index].Value.ToString().Replace(".resx", "." + ci + ".resx"));
                }

                writer.AddResource(row.Cells[colInternal.Index].Value.ToString(),row.Cells[colOtherLang.Index].Value.ToString());

                fname = row.Cells[colFile.Index].Value.ToString();
            }
            if (writer != null)
                writer.Close();
        }
    }
}

