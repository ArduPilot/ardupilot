using System.Diagnostics;
using System.Text;

namespace ArducopterConfigurator
{
    /// <summary>
    /// Monitor VM base class
    /// </summary>
    /// <remarks>
    /// Common base for some factored out things like only passing data to an active
    /// Monitor, etc
    /// </remarks>
    public abstract class MonitorVm : NotifyProperyChangedBase, IPresentationModel
    {
        private IComms _sp;
        protected bool isActive;
        protected string[] PropsInUpdateOrder;

        protected MonitorVm(IComms sp)
        {
            _sp = sp;
            _sp.LineOfDataReceived += sp_LineOfDataReceived;
        }

        void sp_LineOfDataReceived(string obj)
        {
            if (isActive)
                OnStringReceived(obj);
        }

        /// <summary>
        /// Called when the mode is selected
        /// </summary>
        /// <remarks>
        /// Any initialising commands should be sent to the serial port 
        /// when this method is called. 
        /// </remarks>
        public void Activate()
        {
            isActive = true;
            OnActivated();
        }
    
        public void DeActivate()
        {
            isActive = false;
            OnDeactivated();
        }

        protected virtual void OnDeactivated()
        {
            
        }

        protected virtual void OnActivated()
        {

        }

        /// <summary>
        /// Send raw text to the Arducopter
        /// </summary>
        protected void SendString(string strToSend)
        {
            _sp.Send(strToSend);
        }

        /// <summary>
        /// Call-back for text sent back from the Arducopter
        /// </summary>
        /// <remarks>
        /// This is called when a full line of text is received from the APM
        /// This will not be called when the mode is not selected
        /// </remarks>
        protected abstract void OnStringReceived(string strReceived);

        #region Implementation of IPresentationModel

        public abstract string Name { get; }

        #endregion


        // Common method for populating properties, using a hardcoded 
        // property update order, and reflection to get the property type
        protected void PopulatePropsFromUpdate(string strRx, bool fireInpc)
        {
            var strs = strRx.Split(',');
            
            if (PropsInUpdateOrder.Length!=strs.Length)
            {
                Debug.WriteLine("Processing update with only " + strs.Length 
                    + " values, but have " + PropsInUpdateOrder.Length
                    + " properties to populate. Ignoring this update");
                return;
            }
            
            for (int i = 0; i < PropsInUpdateOrder.Length; i++)
            {
                var prop = this.GetType().GetProperty(PropsInUpdateOrder[i]);
                var s = strs[i];
                object value = null;

                if (prop.PropertyType == typeof(float))
                {
                    float val;
                    if (!float.TryParse(s, out val))
                    {
                        Debug.WriteLine("Error parsing float: " + s);
                        break;
                    }
                    value = val;
                }
                if (prop.PropertyType == typeof(bool))
                {
                    float val;
                    if (!float.TryParse(s, out val))
                    {
                        Debug.WriteLine("Error parsing float (bool): " + s);
                        break;
                    }
                    value = val != 0.0;
                }

                if (prop.PropertyType == typeof(int))
                {
                    int val;
                    if (!int.TryParse(s, out val))
                    {
                        Debug.WriteLine("Error parsing float: " + s);
                        break;
                    }
                    value = val;
                }

                prop.SetValue(this, value, null);

                if (fireInpc)
                    FirePropertyChanged(PropsInUpdateOrder[i]);
            }
        }
    
        // Common method for sending/updating data
        // sentence sent to APM is the commandChar followed by the property
        // vals in the correct order, seperated by semicolons
        protected void SendPropsWithCommand(string commandChar)
        {
            var strings = new string[PropsInUpdateOrder.Length];
            for (int i = 0; i < PropsInUpdateOrder.Length; i++)
             {
                 var prop = this.GetType().GetProperty(PropsInUpdateOrder[i]);
                 strings[i] = prop.GetValue(this, null).ToString();
                
             }

            var sentence = commandChar + string.Join(";", strings);
            SendString(sentence);
        }
    }
}