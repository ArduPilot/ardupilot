using System;
using System.ComponentModel;

namespace ArducopterConfigurator.PresentationModels
{
    /// <summary>
    /// Common base for the simple VMs that deal with an update from APM and Get and Update them
    /// </summary>
    public abstract class CrudVm : NotifyProperyChangedBase, IPresentationModel
    {
        protected string updateString;
        protected string refreshString;

        // true when we are populating the properties from a serial command
        protected bool _apmUpdatingProperties;

        protected CrudVm()
        {
            RefreshCommand = new DelegateCommand(_ => RefreshValues());
            UpdateCommand = new DelegateCommand(_ => UpdateValues());
            PropertyChanged += AltitudeHoldConfigVm_PropertyChanged;
        }

        void AltitudeHoldConfigVm_PropertyChanged(object sender, PropertyChangedEventArgs e)
        {
            // When this happens, a property has been set either when the user has adjusted it,
            // or when the APM has sent us an update, probably due to an refresh command.
            if (!_apmUpdatingProperties)
            {
                // Since here we know it was the user updating. Send to the apm
                UpdateValues();
            }
        }

        public string[] PropsInUpdateOrder { get; protected set; }

        public ICommand RefreshCommand { get; private set; }

        public ICommand UpdateCommand { get; private set; }

        protected void RefreshValues()
        {
            if (sendTextToApm != null)
                sendTextToApm(this, new sendTextToApmEventArgs(refreshString));
        }

        protected void UpdateValues()
        {
            if (sendTextToApm != null)
            {
                var apmString = PropertyHelper.ComposePropValuesWithCommand(this, PropsInUpdateOrder, updateString);
                sendTextToApm(this, new sendTextToApmEventArgs(apmString));
            }
        }

        public abstract string Name { get; }

        public void Activate()
        {
            RefreshValues();
        }

        public void DeActivate()
        {

        }

        public void handleLineOfText(string strRx)
        {
            _apmUpdatingProperties = true;
            PropertyHelper.PopulatePropsFromUpdate(this,PropsInUpdateOrder, strRx,false);
            _apmUpdatingProperties = false;
            
            if (updatedByApm != null)
                updatedByApm(this, EventArgs.Empty);
        }

        public event EventHandler<sendTextToApmEventArgs> sendTextToApm;

        public event EventHandler updatedByApm;
    }
}