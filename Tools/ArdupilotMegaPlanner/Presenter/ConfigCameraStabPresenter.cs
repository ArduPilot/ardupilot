using System;
using System.Collections.Generic;
using System.ComponentModel;
using System.Linq;
using System.Reactive.Subjects;
using System.Reflection;
using log4net;

namespace ArdupilotMega.Presenter
{
   public class ConfigCameraStabPresenter : INotifyPropertyChanged, IDataErrorInfo
    {
       private static readonly ILog log = LogManager.GetLogger(MethodBase.GetCurrentMethod().DeclaringType);

       // These are lightweight PropertyMetaData objects, just so that populating defaults
       // and saving etc can be done programatically, rather than explicitely
       // This is starting to step on the toes of PropertyDescriptor etc in System.Componentmodel,
       // however I wanted to keep 'normal' CLR properties here for testability, and more certain Mono
       // compatibility
        private readonly List<PropertyMap> _propertyMap = new List<PropertyMap>
                        {
                            PropertyMap.Create<int>("CameraPitchMin", "CAM_P_MIN", 1000),
                            PropertyMap.Create<int>("CameraPitchMax", "CAM_P_MAX", 2000),
                            PropertyMap.Create<int>("CameraPitchTrim", "CAM_P_TRIM", 1500),
                            PropertyMap.Create<bool>("CameraPitchReverse", "CAM_P_REV", false),
                            PropertyMap.Create<float>("CameraPitchGain", "CAM_P_G", 1.0F),
                            PropertyMap.Create<int>("CameraRollMin", "CAM_R_MIN", 1000),
                            PropertyMap.Create<int>("CameraRollMax", "CAM_R_MAX", 2000),
                            PropertyMap.Create<int>("CameraRollTrim", "CAM_R_TRIM", 1500),
                            PropertyMap.Create<bool>("CameraRollReverse", "CAM_R_REV", false),
                            PropertyMap.Create<float>("CameraRollGain", "CAM_R_G", 1.0F),
                        };

        private CameraAxisProperties _pitchAxis;
        private CameraAxisProperties _rollAxis;

        private readonly MAVLink _mavlink;
        private readonly Dictionary<string, string> _errors;

        public ConfigCameraStabPresenter(MAVLink mavlink)
        {
            _mavlink = mavlink;
            _errors = new Dictionary<string, string>();

            RefreshValuesCommand = new DelegateCommand(_ => GetCameraStabVals());
            WriteValuesCommand = new DelegateCommand(_ => WriteValues());
            SetDefaultsCommand = new DelegateCommand(_ => SetDefaults());

            // Todo change to load params once finished
            SetDefaults();
        }

        public event PropertyChangedEventHandler PropertyChanged;

        public ICommand SetDefaultsCommand { get; private set; }

        /// <summary>
        /// Try to write the cam stab values to the apm
        /// </summary>
        public ICommand WriteValuesCommand { get; private set; }

        /// <summary>
        /// Re-Fetch the cam stab values from teh apm
        /// </summary>
        public ICommand RefreshValuesCommand { get; private set; }

        #region publicBindableProperties

        /// <summary>
        /// Pitch The lowest PWM
        /// </summary>
        public int CameraPitchMin
        {
            get { return _pitchAxis.Min; }

            set
            {
                if (_pitchAxis.Min == value) return;
                _pitchAxis.Min = value;
                RaisePropertyChanged("CameraPitchMin");
            }
        }

        /// <summary>
        /// Pitch The highest PWM
        /// </summary>
        public int CameraPitchMax
        {
            get { return _pitchAxis.Max; }

            set
            {
                if (_pitchAxis.Max == value) return;
                _pitchAxis.Max = value;
                RaisePropertyChanged("CameraPitchMax");
            }
        }

        public int CameraPitchTrim
        {
            get { return _pitchAxis.Trim; }

            set
            {
                if (_pitchAxis.Trim == value) return;
                _pitchAxis.Trim = value;
                RaisePropertyChanged("CameraPitchTrim");
            }
        }

        /// <summary>
        /// Set higher or lower to scale the travel of the servo
        /// </summary>
        public float CameraPitchGain
        {
            get { return _pitchAxis.Gain; }

            set
            {
                if (_pitchAxis.Gain == value) return;
                _pitchAxis.Gain = value;
                RaisePropertyChanged("CameraPitchGain");
            }
        }

        public bool CameraPitchReverse
        {
            get { return _pitchAxis.Reverse; }

            set
            {
                if (_pitchAxis.Reverse == value) return;
                _pitchAxis.Reverse = value;
                RaisePropertyChanged("CameraPitchReverse");
            }
        }

        /// <summary>
        /// Roll The lowest PWM
        /// </summary>
        public int CameraRollMin
        {
            get { return _rollAxis.Min; }

            set
            {
                if (_rollAxis.Min == value) return;
                _rollAxis.Min = value;
                RaisePropertyChanged("CameraRollMin");
            }
        }

        /// <summary>
        /// Roll The highest PWM
        /// </summary>
        public int CameraRollMax
        {
            get { return _rollAxis.Max; }

            set
            {
                if (_rollAxis.Max == value) return;
                _rollAxis.Max = value;
                RaisePropertyChanged("CameraRollMax");
            }
        }

        public int CameraRollTrim
        {
            get { return _rollAxis.Trim; }

            set
            {
                if (_rollAxis.Trim == value) return;
                 _rollAxis.Trim= value;
                RaisePropertyChanged("CameraRollTrim");
            }
        }


        /// <summary>
        /// Set higher or lower to scale the travel of the servo
        /// </summary>
        public float CameraRollGain
        {
            get { return _rollAxis.Gain; }

            set
            {
                if (_rollAxis.Gain == value) return;
                _rollAxis.Gain = value;
                RaisePropertyChanged("CameraRollGain");
            }
        }

        public bool CameraRollReverse
        {
            get { return _rollAxis.Reverse; }

            set
            {
                if (_rollAxis.Reverse == value) return;
                _rollAxis.Reverse = value;
                RaisePropertyChanged("CameraRollReverse");
            }
        }


        private bool hasError;

       public bool HasError
        {
            get { return hasError; }

            set
            {
                if (hasError == value) return;
                hasError = value;
                RaisePropertyChanged("HasError", false);
            }
        }

        #endregion

        private void GetCameraStabVals()
       {
           foreach (var p in _propertyMap)
           {
               var pinfo = GetType().GetProperty(p.PresenterPropertyName);
               var paramVal = GetParam(p.ApmPropertyName, p.PropertyType);
               pinfo.SetValue(this, paramVal, null);
           }

        }

        private void SetDefaults()
        {
            _propertyMap.ForEach(p => GetType().GetProperty(p.PresenterPropertyName).SetValue(this, p.DefaultValue, null));
        }

        private void WriteValues()
        {
            foreach (var p in _propertyMap)
            {
                var pinfo = GetType().GetProperty(p.PresenterPropertyName);
                var val = pinfo.GetValue(this, null);

                try
                {
                    // handle boolean
                    if (val.GetType() == typeof(Boolean))
                    {
                        if (!_mavlink.setParam(p.ApmPropertyName, ((Boolean)val) ? 1 : -1))
                            log.Error("Error setting Camstab parameter");
                    }
                    else
                    {
                        if (!_mavlink.setParam(p.ApmPropertyName, float.Parse(val.ToString())))
                            log.Error("Error setting Camstab parameter");
                    }
                }
                catch (Exception e)
                {
                    log.Error("Error setting Camstab parameter", e);
                }
            }
        }

        private object GetParam(string paramName, Type paramType)
        {
            if (_mavlink.MAV.param.ContainsKey(paramName))
            {
                return paramType == typeof (bool)
                           ? _mavlink.MAV.param[paramName].ToString() == "1"
                           : Convert.ChangeType(_mavlink.MAV.param[paramName].ToString(), paramType);
            }
            log.ErrorFormat("Could not get param {0}", paramName);
            return null;
        }

        public void Load()
        {
            RefreshValuesCommand.Execute(null);
        }

        private void RaisePropertyChanged(string propName, bool shouldValidate = true)
        {
            if (shouldValidate)
                ValidatePropertyValues();

            if (PropertyChanged != null)
            {
                PropertyChanged(this, new PropertyChangedEventArgs(propName));
            }
        }

        private void ValidatePropertyValues()
        {
            var errCount = _errors.Count;
            _errors.Clear();
            if (CameraPitchMax < CameraPitchMin)
            {
                _errors["CameraPitchMax"] = "Max should be > Min";
                _errors["CameraPitchMin"] = "Max should be > Min";
            }
            if ((CameraPitchTrim < CameraPitchMin) || (CameraPitchTrim > CameraPitchMax))
            {
                _errors["CameraPitchTrim"] = "Trim should be between Min and Max";
            }

            if (CameraRollMax < CameraRollMin)
            {
                _errors["CameraRollMax"] = "Max should be > Min";
                _errors["CameraRollMin"] = "Max should be > Min";
            }
            if ((CameraRollTrim < CameraRollMin) || (CameraRollTrim > CameraRollMax))
            {
                _errors["CameraRollTrim"] = "Trim should be between Min and Max";
            }

            if (_errors.Count != errCount)
            {
                this.RaisePropertyChanged("Error", false);
            }

            HasError = _errors.Count > 0;
        }

        public string this[string columnName]
        {
            get { return _errors.ContainsKey(columnName) ? _errors[columnName] : null; }
        }

        public string Error
        {
            get { return _errors.Values.FirstOrDefault(); }
        }

        private struct CameraAxisProperties
        {
            public int Min;
            public int Max;
            public int Trim;
            public float Gain;
            public bool Reverse;
        }

        private struct PropertyMap
        {
            public string PresenterPropertyName;
            public string ApmPropertyName;
            public Type PropertyType;
            public object DefaultValue;

            public static PropertyMap Create<T>(string presenterPropertyName, string apmPropertyName, object defaultVal)
            {
                return new PropertyMap
                {
                    PresenterPropertyName = presenterPropertyName,
                    ApmPropertyName = apmPropertyName,
                    PropertyType = typeof(T),
                    DefaultValue = defaultVal
                };
            }
        }
    }
}