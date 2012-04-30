using System;
using System.Collections.Generic;
using System.ComponentModel;
using System.Drawing;
using System.Data;
using System.Globalization;
using System.Linq;
using System.Text;
using System.Windows.Forms;

namespace ArdupilotMega.Controls
{
   public partial class RangeControl : UserControl, IDynamicParameterControl
   {
      #region Properties

      public NumericUpDown NumericUpDownControl { get { return numericUpDown1; } set { numericUpDown1 = value; } }
      public string DescriptionText { get { return label1.Text; } set { label1.Text = value; } }
      public string LabelText { get { return myLabel1.Text; } set { myLabel1.Text = value; } }
      public TrackBar TrackBarControl { get { return trackBar1; } set { trackBar1 = value; } }
      public int Scaler { get; set; }

      #region Interface Properties

      public string Value
      {
         get { return numericUpDown1.Value.ToString(CultureInfo.InvariantCulture); } 
         set
         {
            numericUpDown1.Value = decimal.Parse(value);
            numericUpDown1_ValueChanged(null, null);
         }
      }

      #endregion

      #endregion

      #region Constructor

      public RangeControl()
      {
         InitializeComponent();
      }

      #endregion

      #region Methods

      public void AttachEvents()
      {
         numericUpDown1.ValueChanged += numericUpDown1_ValueChanged;
         trackBar1.ValueChanged += trackBar1_ValueChanged;
      }

      #endregion

      #region Events

      protected void numericUpDown1_ValueChanged(object sender, EventArgs e)
      {
         trackBar1.Value = (Scaler > 0) ? (int)(numericUpDown1.Value * Scaler) : (int)numericUpDown1.Value;
      }

      protected void trackBar1_ValueChanged(object sender, EventArgs e)
      {
         numericUpDown1.Value = (Scaler > 0) ? (trackBar1.Value / (decimal)Scaler) : trackBar1.Value;
         numericUpDown1.Text = numericUpDown1.Value.ToString();
      }

      #endregion
   }
}
