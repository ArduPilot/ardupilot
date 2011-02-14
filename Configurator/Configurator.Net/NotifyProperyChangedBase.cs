using System.ComponentModel;

namespace ArducopterConfigurator
{
    public abstract class NotifyProperyChangedBase : INotifyPropertyChanged, ISupportsExternalInvokedInpc
    {
        public event PropertyChangedEventHandler PropertyChanged;
        
        protected bool CheckPropertyChanged<T>(string propertyName, ref T oldValue, ref T newValue)
        {
            if (oldValue == null && newValue == null)
                return false;

            if ((oldValue == null && newValue != null) || !oldValue.Equals((T)newValue))
            {
                oldValue = newValue;
                FirePropertyChanged(propertyName);
                return true;
            }

            return false;
        }

        public void FirePropertyChanged(string propertyName)
        {
            if (PropertyChanged != null)
                PropertyChanged(this, new PropertyChangedEventArgs(propertyName));
        }

    }
}