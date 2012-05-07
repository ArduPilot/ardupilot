using System;
using System.Collections;
using System.Collections.Generic;
using System.Drawing;
using System.Globalization;
using System.IO;
using System.Linq;
using System.Text;
using System.Windows.Forms;
using ArdupilotMega.Controls;
using ArdupilotMega.Controls.BackstageView;
using ArdupilotMega.Utilities;
using log4net;

namespace ArdupilotMega.GCSViews.ConfigurationView
{
    public partial class ConfigFriendlyParams : BackStageViewContentPanel
    {
        #region Class Fields

        private static readonly ILog log =
          LogManager.GetLogger(System.Reflection.MethodBase.GetCurrentMethod().DeclaringType);
        private readonly ParameterMetaDataRepository _parameterMetaDataRepository;
        private Dictionary<string, string> _params = new Dictionary<string, string>();

        #endregion

        #region Properties

        /// <summary>
        /// Gets or sets the parameter mode.
        /// </summary>
        /// <value>
        /// The parameter mode.
        /// </value>
        public string ParameterMode { get; set; }

        #endregion

        #region Constructor

        public ConfigFriendlyParams()
        {
            InitializeComponent();
            tableLayoutPanel1.Height = this.Height;
            _parameterMetaDataRepository = new ParameterMetaDataRepository();

            MainV2.comPort.ParamListChanged += comPort_ParamListChanged;
            Resize += this_Resize;

            BUT_rerequestparams.Click += BUT_rerequestparams_Click;
            BUT_writePIDS.Click += BUT_writePIDS_Click;
        }

        #endregion

        #region Events

        /// <summary>
        /// Handles the Click event of the BUT_writePIDS control.
        /// </summary>
        /// <param name="sender">The source of the event.</param>
        /// <param name="e">The <see cref="System.EventArgs"/> instance containing the event data.</param>
        protected void BUT_writePIDS_Click(object sender, EventArgs e)
        {
            bool errorThrown = false;
            _params.ForEach(x =>
            {
                var matchingControls = tableLayoutPanel1.Controls.Find(x.Key, true);
                if (matchingControls.Length > 0)
                {
                    var ctl = (IDynamicParameterControl)matchingControls[0];
                    try
                    {
                        MainV2.comPort.setParam(x.Key, float.Parse(ctl.Value));
                    }
                    catch
                    {
                        errorThrown = true;
                        CustomMessageBox.Show("Set " + x.Key + " Failed");
                    }
                }
            });
            if (!errorThrown)
            {
                CustomMessageBox.Show("Parameters successfully saved.");
            }
        }

        /// <summary>
        /// Handles the Click event of the BUT_rerequestparams control.
        /// </summary>
        /// <param name="sender">The source of the event.</param>
        /// <param name="e">The <see cref="System.EventArgs"/> instance containing the event data.</param>
        protected void BUT_rerequestparams_Click(object sender, EventArgs e)
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

            BindParamList();
        }

        /// <summary>
        /// Handles the Resize event of the this control.
        /// </summary>
        /// <param name="sender">The source of the event.</param>
        /// <param name="e">The <see cref="System.EventArgs"/> instance containing the event data.</param>
        protected void this_Resize(object sender, EventArgs e)
        {
            tableLayoutPanel1.Height = this.Height - 50;
        }

        /// <summary>
        /// Handles the Load event of the ConfigRawParamsV2 control.
        /// </summary>
        /// <param name="sender">The source of the event.</param>
        /// <param name="e">The <see cref="System.EventArgs"/> instance containing the event data.</param>
        protected void ConfigRawParamsV2_Load(object sender, EventArgs e)
        {
            BindParamList();
        }

        /// <summary>
        /// Handles the ParamListChanged event of the comPort control.
        /// </summary>
        /// <param name="sender">The source of the event.</param>
        /// <param name="e">The <see cref="System.EventArgs"/> instance containing the event data.</param>
        protected void comPort_ParamListChanged(object sender, EventArgs e)
        {
            SortParamList();
        }

        #endregion

        #region Methods

        /// <summary>
        /// Sorts the param list.
        /// </summary>
        private void SortParamList()
        {
            // Clear list
            _params.Clear();

            // When the parameter list is changed, re sort the list for our View's purposes
            MainV2.comPort.param.Keys.ForEach(x =>
            {
                string displayName = _parameterMetaDataRepository.GetParameterMetaData(x.ToString(), ParameterMetaDataConstants.DisplayName);
                string parameterMode = _parameterMetaDataRepository.GetParameterMetaData(x.ToString(), ParameterMetaDataConstants.User);

                // If we have a friendly display name AND
                if (!String.IsNullOrEmpty(displayName) &&
                    // The user type is equal to the ParameterMode specified at class instantiation OR
                      ((!String.IsNullOrEmpty(parameterMode) && parameterMode == ParameterMode) ||
                    // The user type is empty and this is in Advanced mode
                      String.IsNullOrEmpty(parameterMode) && ParameterMode == ParameterMetaDataConstants.Advanced))
                {
                    _params.Add(x.ToString(), displayName);
                }
            });
            _params = _params.OrderBy(x => x.Value).ToDictionary(x => x.Key, x => x.Value);
        }

        /// <summary>
        /// Binds the param list.
        /// </summary>
        private void BindParamList()
        {
            tableLayoutPanel1.Controls.Clear();
            if (_params == null || _params.Count == 0) SortParamList();

            // get the params if nothing exists already
            if (_params != null && _params.Count == 0)
                Utilities.ParameterMetaDataParser.GetParameterInformation();

            _params.ForEach(x => 
         {
            if(!String.IsNullOrEmpty(x.Key))
            {
                try
                {
                    bool controlAdded = false;

                    string value = ((float)MainV2.comPort.param[x.Key]).ToString("0.###");
                    string description = _parameterMetaDataRepository.GetParameterMetaData(x.Key, ParameterMetaDataConstants.Description);
                    string displayName = x.Value;
                    string units = _parameterMetaDataRepository.GetParameterMetaData(x.Key, ParameterMetaDataConstants.Units);

                    // If this is a range
                    string rangeRaw = _parameterMetaDataRepository.GetParameterMetaData(x.Key, ParameterMetaDataConstants.Range);
                    string incrementRaw = _parameterMetaDataRepository.GetParameterMetaData(x.Key, ParameterMetaDataConstants.Increment);
                    if (!String.IsNullOrEmpty(rangeRaw) && !String.IsNullOrEmpty(incrementRaw))
                    {
                        float increment, intValue;
                        float.TryParse(incrementRaw, out increment);
                        float.TryParse(value, out intValue);

                        string[] rangeParts = rangeRaw.Split(new[] { ' ' });
                        if (rangeParts.Count() == 2 && increment > 0)
                        {
                            float lowerRange;
                            float.TryParse(rangeParts[0], out lowerRange);
                            float upperRange;
                            float.TryParse(rangeParts[1], out upperRange);

                            int scaler = (int)float.Parse((1 / increment).ToString(CultureInfo.InvariantCulture));
                            int scaledLowerRange = 0, scaledUpperRange = 0;
                            int scaledIncrement = (int)increment;
                            if (scaler > 0)
                            {
                                scaledLowerRange = (int)(lowerRange * scaler);
                                scaledUpperRange = (int)(upperRange * scaler);
                                scaledIncrement = (int)float.Parse((increment * scaler).ToString(CultureInfo.InvariantCulture));
                                intValue *= scaler;
                            }

                            var rangeControl = new RangeControl();
                            rangeControl.Name = x.Key;
                            rangeControl.Scaler = scaler;
                            rangeControl.DescriptionText = FitDescriptionText(units, description);
                            rangeControl.LabelText = displayName;
                            rangeControl.TrackBarControl.Minimum = scaledLowerRange;
                            rangeControl.TrackBarControl.Maximum = scaledUpperRange;
                            rangeControl.TrackBarControl.TickFrequency = scaledIncrement;
                            rangeControl.TrackBarControl.Value = (int)intValue;

                            rangeControl.NumericUpDownControl.Increment = (decimal)increment;
                            rangeControl.NumericUpDownControl.DecimalPlaces = scaler.ToString(CultureInfo.InvariantCulture).Length - 1;
                            rangeControl.NumericUpDownControl.Minimum = (decimal)lowerRange;
                            rangeControl.NumericUpDownControl.Maximum = (decimal)upperRange;
                            rangeControl.NumericUpDownControl.Value = (decimal)((float)MainV2.comPort.param[x.Key]);

                            rangeControl.AttachEvents();

                            tableLayoutPanel1.Controls.Add(rangeControl);

                            controlAdded = true;
                        }
                    }

                    if (!controlAdded)
                    {
                        // If this is a subset of values
                        string availableValuesRaw = _parameterMetaDataRepository.GetParameterMetaData(x.Key, ParameterMetaDataConstants.Values);
                        if (!String.IsNullOrEmpty(availableValuesRaw))
                        {
                            string[] availableValues = availableValuesRaw.Split(new[] { ',' });
                            if (availableValues.Any())
                            {
                                var valueControl = new ValuesControl();
                                valueControl.Name = x.Key;
                                valueControl.DescriptionText = FitDescriptionText(units, description);
                                valueControl.LabelText = displayName;

                                var splitValues = new List<KeyValuePair<string, string>>();
                                // Add the values to the ddl
                                availableValues.ForEach(val =>
                                {
                                    string[] valParts = val.Split(new[] { ':' });
                                    splitValues.Add(new KeyValuePair<string, string>(valParts[0], (valParts.Length > 1) ? valParts[1] : valParts[0]));
                                });
                                valueControl.ComboBoxControl.DisplayMember = "Value";
                                valueControl.ComboBoxControl.ValueMember = "Key";
                                valueControl.ComboBoxControl.DataSource = splitValues;
                                valueControl.ComboBoxControl.SelectedValue = value;

                                tableLayoutPanel1.Controls.Add(valueControl);
                            }
                        }
                    }
                } // if there is an error simply dont show it, ie bad pde file, bad scale etc
                catch (Exception ex) { log.Error(ex); }
            }
         });
        }

        /// <summary>
        /// Fits the description text.
        /// </summary>
        /// <param name="description">The description.</param>
        /// <returns></returns>
        private string FitDescriptionText(string description)
        {
            return FitDescriptionText(string.Empty, description);
        }

        /// <summary>
        /// Fits the description text.
        /// </summary>
        /// <param name="units">The units.</param>
        /// <param name="description">The description.</param>
        /// <returns></returns>
        private string FitDescriptionText(string units, string description)
        {
            var returnDescription = new StringBuilder();

            if (!String.IsNullOrEmpty(units))
            {
                returnDescription.Append(String.Format("Units: {0}{1}", units, Environment.NewLine));
            }

            if (!String.IsNullOrEmpty(description))
            {
                returnDescription.Append("Description: ");
                var descriptionParts = description.Split(new char[] { ' ' });
                for (int i = 0; i < descriptionParts.Length; i++)
                {
                    returnDescription.Append(String.Format("{0} ", descriptionParts[i]));
                    if (i != 0 && i % 10 == 0) returnDescription.Append(Environment.NewLine);
                }
            }

            return returnDescription.ToString();
        }

        #endregion
    }
}
