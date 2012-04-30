using System;
using System.Collections.Generic;
using System.ComponentModel;
using System.Drawing;
using System.Data;
using System.Linq;
using System.Text;
using System.Windows.Forms;

namespace ArdupilotMega.Controls
{
   public partial class ValuesControl : UserControl, IDynamicParameterControl
   {
      #region Properties

      public string LabelText { get { return myLabel1.Text; } set { myLabel1.Text = value; } }
      public string DescriptionText { get { return label1.Text; } set { label1.Text = value; } }
      public ComboBox ComboBoxControl { get { return comboBox1; } set { comboBox1 = value; } }

      #region Interface Properties

      public string Value { get { return comboBox1.SelectedValue.ToString(); } set { comboBox1.SelectedValue = value; } }

      #endregion

      #endregion

      #region Constructor

      public ValuesControl()
      {
         InitializeComponent();
      }

      #endregion
   }
}
