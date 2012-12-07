using System;
using System.Collections.Generic;
using System.ComponentModel;
using System.Data;
using System.Drawing;
using System.Reflection;
using System.Text;
using System.Windows.Forms;
using System.IO;
using log4net;
using ZedGraph; // Graphs
using System.Xml;

namespace ArdupilotMega
{
    public partial class LogBrowse : Form
    {
        private static readonly ILog log = LogManager.GetLogger(MethodBase.GetCurrentMethod().DeclaringType);
        int m_iColumnCount = 0;
        DataTable m_dtCSV = new DataTable();

        PointPairList list1 = new PointPairList();
        PointPairList list2 = new PointPairList();
        PointPairList list3 = new PointPairList();
        PointPairList list4 = new PointPairList();
        PointPairList list5 = new PointPairList();

        int graphs = 0;

        public LogBrowse()
        {
            InitializeComponent();
        }

        private void Form1_Load(object sender, EventArgs e)
        {
            OpenFileDialog openFileDialog1 = new OpenFileDialog();
            openFileDialog1.Filter = "Log Files|*.log";
            openFileDialog1.FilterIndex = 2;
            openFileDialog1.RestoreDirectory = true;

            openFileDialog1.InitialDirectory = Path.GetDirectoryName(Application.ExecutablePath) + Path.DirectorySeparatorChar + "logs";

            if (openFileDialog1.ShowDialog() == DialogResult.OK)
            {
                try
                {
                    Stream stream = File.Open(openFileDialog1.FileName, FileMode.Open);
                    PopulateDataTableFromUploadedFile(stream);
                    stream.Close();

                    dataGridView1.DataSource = m_dtCSV;

                }
                catch (Exception ex) { CustomMessageBox.Show("Failed to read File: " + ex.ToString()); }

                foreach (DataGridViewColumn column in dataGridView1.Columns)
                {
                    column.SortMode = DataGridViewColumnSortMode.NotSortable;
                }

                CreateChart(zg1);

                int a = 1;
                foreach (DataGridViewRow row in dataGridView1.Rows)
                {
                    //Commands.Rows[a].HeaderCell.Value
                    row.HeaderCell.Value = a.ToString();
                    a++;
                }
            }
            else
            {
                return;
            }
        }

        private void PopulateDataTableFromUploadedFile(System.IO.Stream strm)
        {
            System.IO.StreamReader srdr = new System.IO.StreamReader(strm);
            String strLine = String.Empty;
            Int32 iLineCount = 0;
            do
            {
                strLine = srdr.ReadLine();
                if (strLine == null)
                {
                    break;
                }
                if (0 == iLineCount++)
                {
                    m_dtCSV = new DataTable("CSVTable");
                    //m_dtCSV = this.CreateDataTableForCSVData(strLine);
                }
                this.AddDataRowToTable(strLine, m_dtCSV);
            } while (true);
        }

        private DataTable CreateDataTableForCSVData(String strLine)
        {
            DataTable dt = new DataTable("CSVTable");
            String[] strVals = strLine.Split(new char[] { ',', ':' });
            m_iColumnCount = strVals.Length;
            int idx = 0;
            foreach (String strVal in strVals)
            {
                String strColumnName = String.Format("{0}", idx++);
                dt.Columns.Add(strColumnName, Type.GetType("System.String"));
            }
            return dt;
        }

        private DataRow AddDataRowToTable(String strCSVLine, DataTable dt)
        {
            String[] strVals = strCSVLine.Split(new char[] { ',', ':' });
            Int32 iTotalNumberOfValues = strVals.Length;
            // If number of values in this line are more than the columns
            // currently in table, then we need to add more columns to table.
            if (iTotalNumberOfValues > m_iColumnCount)
            {
                Int32 iDiff = iTotalNumberOfValues - m_iColumnCount;
                for (Int32 i = 0; i < iDiff; i++)
                {
                    String strColumnName = String.Format("{0}", (m_iColumnCount + i));
                    dt.Columns.Add(strColumnName, Type.GetType("System.String"));
                }
                m_iColumnCount = iTotalNumberOfValues;
            }
            int idx = 0;
            DataRow drow = dt.NewRow();
            foreach (String strVal in strVals)
            {
                String strColumnName = String.Format("{0}", idx++);
                drow[strColumnName] = strVal.Trim();
            }
            dt.Rows.Add(drow);
            return drow;
        }

        private void dataGridView1_RowEnter(object sender, DataGridViewCellEventArgs e)
        {
            try
            {
                int a = 0;
                foreach (DataGridViewColumn col in dataGridView1.Columns)
                {
                        col.HeaderText = a.ToString();
                        a++;
                }
            }
            catch { }
            try
            {
                string option = dataGridView1[0, e.RowIndex].EditedFormattedValue.ToString();

                if (option.StartsWith("PID-"))
                    option = "PID-1";

                using (XmlReader reader = XmlReader.Create(Path.GetDirectoryName(Application.ExecutablePath) + Path.DirectorySeparatorChar + "dataflashlog.xml"))
                {
                    reader.Read();
                    reader.ReadStartElement("LOGFORMAT");
                    if (MainV2.comPort.MAV.cs.firmware == MainV2.Firmwares.ArduPlane)
                    {
                        reader.ReadToFollowing("APM");
                    }
                    else if (MainV2.comPort.MAV.cs.firmware == MainV2.Firmwares.ArduRover)
                    {
                        reader.ReadToFollowing("APRover");
                    }
                    else
                    {
                        reader.ReadToFollowing("AC2");
                    }
                    reader.ReadToFollowing(option);

                    dataGridView1.Columns[0].HeaderText = "";

                    XmlReader inner = reader.ReadSubtree();

                    inner.MoveToElement();

                    int a = 1;

                    while (inner.Read())
                    {
                        inner.MoveToElement();
                        if (inner.IsStartElement())
                        {
                            if (inner.Name.StartsWith("F"))
                            {
                                dataGridView1.Columns[a].HeaderText = inner.ReadString();
                                log.Info(a + " " + dataGridView1.Columns[a].HeaderText);
                                a++;
                            }

                        }
                    }

                    for (; a < dataGridView1.Columns.Count; a++)
                    {
                        dataGridView1.Columns[a].HeaderText = "";
                    }

                }
            }
            catch { log.Info("DGV logbrowse error"); }
        }

        public void CreateChart(ZedGraphControl zgc)
        {
            GraphPane myPane = zgc.GraphPane;

            // Set the titles and axis labels
            myPane.Title.Text = "Value Graph";
            myPane.XAxis.Title.Text = "Line Number";
            myPane.YAxis.Title.Text = "Output";

            LineItem myCurve;

            myCurve = myPane.AddCurve("Value", list1, Color.Red, SymbolType.None);
            myCurve = myPane.AddCurve("Value", list2, Color.Green, SymbolType.None);
            myCurve = myPane.AddCurve("Value", list3, Color.Blue, SymbolType.None);
            myCurve = myPane.AddCurve("Value", list4, Color.Pink, SymbolType.None);
            myCurve = myPane.AddCurve("Value", list5, Color.Yellow, SymbolType.None);

            // Show the x axis grid
            myPane.XAxis.MajorGrid.IsVisible = true;

            myPane.XAxis.Scale.Min = 0;
            //myPane.XAxis.Scale.Max = -1;

            // Make the Y axis scale red
            myPane.YAxis.Scale.FontSpec.FontColor = Color.Red;
            myPane.YAxis.Title.FontSpec.FontColor = Color.Red;
            // turn off the opposite tics so the Y tics don't show up on the Y2 axis
            myPane.YAxis.MajorTic.IsOpposite = false;
            myPane.YAxis.MinorTic.IsOpposite = false;
            // Don't display the Y zero line
            myPane.YAxis.MajorGrid.IsZeroLine = true;
            // Align the Y axis labels so they are flush to the axis
            myPane.YAxis.Scale.Align = AlignP.Inside;
            // Manually set the axis range
            //myPane.YAxis.Scale.Min = -1;
            //myPane.YAxis.Scale.Max = 1;

            // Fill the axis background with a gradient
            //myPane.Chart.Fill = new Fill(Color.White, Color.LightGray, 45.0f);


            // Calculate the Axis Scale Ranges
            try
            {
                zg1.AxisChange();
            }
            catch { }



        }

        private void Graphit_Click(object sender, EventArgs e)
        {
            if (dataGridView1.RowCount == 0 || dataGridView1.ColumnCount == 0)
            {
                CustomMessageBox.Show("Please load a valid file");
                return;
            }

            int col = dataGridView1.CurrentCell.ColumnIndex;
            int row = dataGridView1.CurrentCell.RowIndex;
            string type = dataGridView1[0, row].Value.ToString();
            double a = 0; // row counter

            if (col == 0)
            {
                CustomMessageBox.Show("Please pick another column, Highlight the cell you wish to graph");
                return;
            }

            int error = 0;

            foreach (DataGridViewRow datarow in dataGridView1.Rows)
            {
                if (datarow.Cells[0].Value.ToString() == type)
                {
                    try
                    {
                        double value = double.Parse(datarow.Cells[col].Value.ToString(), new System.Globalization.CultureInfo("en-US"));
                        if (graphs == 0)
                        {
                            zg1.GraphPane.CurveList[0].Label.Text = dataGridView1.Columns[col].HeaderText;
                            list1.Add(a, value);
                        }
                        else if (graphs == 1)
                        {
                            zg1.GraphPane.CurveList[1].Label.Text = dataGridView1.Columns[col].HeaderText;
                            list2.Add(a, value);
                        }
                        else if (graphs == 2)
                        {
                            zg1.GraphPane.CurveList[2].Label.Text = dataGridView1.Columns[col].HeaderText;
                            list3.Add(a, value);
                        }
                        else if (graphs == 3)
                        {
                            zg1.GraphPane.CurveList[3].Label.Text = dataGridView1.Columns[col].HeaderText;
                            list4.Add(a, value);
                        }
                        else if (graphs == 4)
                        {
                            zg1.GraphPane.CurveList[4].Label.Text = dataGridView1.Columns[col].HeaderText;
                            list5.Add(a, value);
                        }
                        else
                        {
                            CustomMessageBox.Show("Max of 5");
                            break;
                        }
                    }
                    catch { error++; log.Info("Bad Data : " + type + " " + col + " " + a); if (error >= 500) { CustomMessageBox.Show("There is to much bad data - failing"); break; } }
                }
                a++;
            }

            // Make sure the Y axis is rescaled to accommodate actual data
            try
            {
                zg1.AxisChange();
            }
            catch { }
            // Zoom all
            zg1.ZoomOutAll(zg1.GraphPane);
            // Force a redraw
            zg1.Invalidate();

            graphs++;
        }

        private void BUT_cleargraph_Click(object sender, EventArgs e)
        {
            graphs = 0;
            foreach (LineItem line in zg1.GraphPane.CurveList)
            {
                line.Clear();
                line.Label.Text = "Value";
            }
            zg1.Invalidate();
        }

        private void BUT_loadlog_Click(object sender, EventArgs e)
        {
            // reset column count
            m_iColumnCount = 0;
            // clear existing lists
            zg1.GraphPane.CurveList.Clear();
            // reload
            Form1_Load(sender, e);
        }
    }
}