using System;
using System.Collections.Generic;
using System.ComponentModel;
using System.Data;
using System.Drawing;
using System.Text;
using System.Windows.Forms;
using System.Collections;

namespace ArdupilotMega
{
    public partial class ParamCompare : Form
    {
        GCSViews.Configuration config;
        Hashtable param = new Hashtable();
        Hashtable param2 = new Hashtable();

        public ParamCompare(GCSViews.Configuration config, Hashtable param, Hashtable param2)
        {
            InitializeComponent();
            this.param = param;
            this.param2 = param2;
            this.config = config;

            processToScreen();
        }

        void processToScreen()
        {
            Params.Rows.Clear();

            // process hashdefines and update display
            foreach (string value in param.Keys)
            {
                if (value == null || value == "")
                    continue;

                //System.Diagnostics.Debug.WriteLine("Doing: " + value);
                try
                {
                    if (param[value] != param2[value])
                    {
                        Params.Rows.Add();
                        Params.Rows[Params.RowCount - 1].Cells[Command.Index].Value = value;
                        Params.Rows[Params.RowCount - 1].Cells[Value.Index].Value = ((float)param[value]).ToString("0.###");

                        Params.Rows[Params.RowCount - 1].Cells[newvalue.Index].Value = ((float)param2[value]).ToString("0.###");
                        Params.Rows[Params.RowCount - 1].Cells[Use.Index].Value = true;
                    }
                }
                catch { if (Params.RowCount > 1) { Params.Rows.RemoveAt(Params.RowCount - 1); } }

            }
            Params.Sort(Params.Columns[0], ListSortDirection.Ascending);
        }

        private void BUT_save_Click(object sender, EventArgs e)
        {
            foreach (DataGridViewRow row in Params.Rows)
            {
                if ((bool)row.Cells[Use.Index].Value == true)
                {
                    config.EEPROM_View_float_TextChanged(new Control() { Name = row.Cells[Command.Index].Value.ToString(), Text = row.Cells[newvalue.Index].Value.ToString() }, null);
                }
            }
            this.Close();
        }
    }
}
