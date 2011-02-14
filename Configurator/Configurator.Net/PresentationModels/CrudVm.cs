using System;

namespace ArducopterConfigurator.PresentationModels
{
    /// <summary>
    /// Common base for the simple VMs that deal with an update from APM and Get and Update them
    /// </summary>
    public abstract class CrudVm : NotifyProperyChangedBase, ISupportsPropertyPopulation, ItalksToApm, IPresentationModel
    {
        protected string updateString;
        protected string refreshString;

        protected CrudVm()
        {
            RefreshCommand = new DelegateCommand(_ => RefreshValues());
            UpdateCommand = new DelegateCommand(_ => UpdateValues());
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
                var apmString = PropertyHelper.ComposePropValuesWithCommand(this, updateString);
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
            PropertyHelper.PopulatePropsFromUpdate(this, strRx, true);
            
            if (updatedByApm != null)
                updatedByApm(this, EventArgs.Empty);
        }

        public event EventHandler<sendTextToApmEventArgs> sendTextToApm;

        public event EventHandler updatedByApm;
    }
}