using System;
using System.Collections.Generic;
using System.ComponentModel;
using System.Drawing;
using System.Data;
using System.Text;
using System.Windows.Forms;
using ArducopterConfigurator.PresentationModels;
using ArducopterConfigurator.Views;

namespace ArducopterConfigurator.views
{
    public partial class FlightDataView : FlightDataViewDesignable
    {
        public FlightDataView()
        {
            InitializeComponent();
        }

        public override void SetDataContext(FlightDataVm model)
        {
            FlightDataVmBindingSource.DataSource = model;
        }

     
    }
    // Required for VS2008 designer. No functional value
    public class FlightDataViewDesignable : ViewCommon<FlightDataVm> { }
}
